//
// Created by wyz on 2021/2/8.
//

#pragma once

#include <array>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <vector>
#include "RawStream.hpp"
#define DEFAULT_VALUE 0

/**
 * @brief Read blocks from large raw volume data(uint8 only).
 */
class BlockVolumeReader
{
  public:
    struct RawVolumeInfo
    {
        std::string input_file_path;
        uint32_t raw_x;
        uint32_t raw_y;
        uint32_t raw_z;
        size_t voxel_size = 1;//default
        uint32_t block_length;
        uint32_t padding;
        uint32_t mem_limit = 32; // GB
    };

  public:
    explicit BlockVolumeReader(RawVolumeInfo raw_volume_info);

    virtual ~BlockVolumeReader()
    {
        if (read_task.joinable())
        {
            read_task.join();
        }
    }

    /**
     * @brief just need call once for a huge volume data's read
     */
    void startRead();

    /**
     * @brief synchronous get a block and its index
     * @param block will replaced by data, original data of block will clear
     * @param index xyz-coord, will set to index of block
     */
    void getBlock(std::vector<uint8_t> &block, std::array<uint32_t, 3> &index);

    /**
     * @brief wait loading until return a block
     */
    auto getBlock();

    bool isEmpty()
    {
        return is_read_finish() && is_block_warehouse_empty();
    }

    std::array<uint32_t, 3> getDim() const;

    void set_batch_read_turn_and_count(int turn,int count);

    void set_raw_stream(std::unique_ptr<RawStream> stream);

  private:
    struct Block
    {
        std::array<uint32_t, 3> index;
        std::vector<uint8_t> data;
    };

    enum BlockState
    {
        UnRead,
        Store,
        Token
    };

    void set_mem_limit(uint64_t memLimit);

    void setup_raw_volume_info(RawVolumeInfo raw_volume_info);

    bool is_read_finish();

    bool is_block_warehouse_empty();

    /**
     * once read x_block_num=(raw_x+block_length-1)/block_length blocks
     */
    void read_patch_blocks();

    bool read_patch_enable();


  private:
    std::string input_file_path;
    uint32_t raw_x;
    uint32_t raw_y;
    uint32_t raw_z;
    size_t voxel_size;
    uint32_t block_length;
    uint32_t padding;

    uint32_t modify_x, modify_y, modify_z; // equal to dim*block_length_nopadding
    uint32_t block_length_nopadding;

//    std::fstream in;
    std::array<uint32_t, 3> dim;
    uint32_t total_block_num;
    uint64_t block_byte_size; // block_length^3 * voxel_size;
    uint32_t batch_read_cnt;  // total batch read number
    uint32_t batch_read_turn; // current batch read turn
    uint32_t block_num_per_batch;
    uint64_t batch_cached_byte_size; // equal block_byte_size*block_num_per_batch

    uint32_t max_memory_size_GB;

    std::mutex batch_blocks_mutex;

    std::condition_variable asyn_read;
    std::condition_variable asyn_get;
    struct BlocksManager
    {
        uint32_t max_cached_batch_num;
        uint32_t max_cached_block_num;
        uint32_t cur_cached_block_num;
        std::queue<Block> block_warehouse;    // size limited by memory limit
        std::vector<BlockState> block_states; // size equal to total block count
    } block_manager;
    std::thread read_task;

    //stream reader
    std::unique_ptr<RawStream> stream;
};

inline BlockVolumeReader::BlockVolumeReader(BlockVolumeReader::RawVolumeInfo raw_volume_info)
{
    setup_raw_volume_info(raw_volume_info);
}
inline void BlockVolumeReader::set_mem_limit(uint64_t memLimit)
{
    uint64_t mem_limit_byte_size = memLimit << 30;
    if (mem_limit_byte_size < batch_cached_byte_size * 2)
    {
        throw std::runtime_error("memory not enough to start read task");
    }
    this->max_memory_size_GB = memLimit;
}
inline void BlockVolumeReader::setup_raw_volume_info(BlockVolumeReader::RawVolumeInfo raw_volume_info)
{
//    in.open(raw_volume_info.input_file_path.c_str(), std::ios::in | std::ios::binary);
//    if (!in.is_open())
//    {
//        throw std::runtime_error("raw volume file open failed!");
//    }

    {
        input_file_path = raw_volume_info.input_file_path;
        raw_x = raw_volume_info.raw_x;
        raw_y = raw_volume_info.raw_y;
        raw_z = raw_volume_info.raw_z;
        voxel_size = raw_volume_info.voxel_size;
        block_length = raw_volume_info.block_length;
        padding = raw_volume_info.padding;

        block_length_nopadding = block_length - 2 * padding;
        this->dim[0] = (raw_x + block_length_nopadding - 1) / block_length_nopadding;
        this->dim[1] = (raw_y + block_length_nopadding - 1) / block_length_nopadding;
        this->dim[2] = (raw_z + block_length_nopadding - 1) / block_length_nopadding;
        this->modify_x = this->dim[0] * block_length_nopadding;
        this->modify_y = this->dim[1] * block_length_nopadding;
        this->modify_z = this->dim[2] * block_length_nopadding;
        if (dim[0] == 0 || dim[1] == 0 || dim[2] == 0)
        {
            throw std::runtime_error("bad block length!");
        }

        this->total_block_num = this->dim[0] * this->dim[1] * this->dim[2];
        // consider block_length with padding
        this->block_byte_size = block_length * block_length * block_length * voxel_size;
        this->batch_read_cnt = this->dim[1] * this->dim[2];
        this->batch_read_turn = 0;
        this->block_num_per_batch = this->dim[0];
        this->batch_cached_byte_size = this->block_num_per_batch * this->block_byte_size;

        set_mem_limit(raw_volume_info.mem_limit);

        this->block_manager.max_cached_batch_num =
            uint64_t(this->max_memory_size_GB) * 1024 * 1024 * 1024 / this->batch_cached_byte_size;
        this->block_manager.max_cached_block_num = this->block_manager.max_cached_batch_num * this->block_num_per_batch;
        this->block_manager.block_states.assign(this->total_block_num, UnRead);
        this->block_manager.cur_cached_block_num = 0;
    }

}
inline void BlockVolumeReader::startRead()
{
    if(!stream){
        std::cout<<"stream default open as raw"<<std::endl;
        stream = std::make_unique<RawStream>(raw_x,raw_y,raw_z,voxel_size);
        stream->open(input_file_path);
    }


    static bool first = true;
    if (!first)
        return;
    first = false;

    read_task = std::thread([this]() {
        std::cout << "start read task" << std::endl;
        while (!is_read_finish())
        {
            if (read_patch_enable())
            {
                read_patch_blocks();
            }
        }
        std::cout << "read finished" << std::endl;
    });
}

inline bool BlockVolumeReader::is_read_finish()
{
    return batch_read_turn == batch_read_cnt;
}

inline void BlockVolumeReader::getBlock(std::vector<uint8_t> &block, std::array<uint32_t, 3> &index)
{
    {
        std::unique_lock<std::mutex> lk(batch_blocks_mutex);
        asyn_get.wait(lk, [&]() { return !is_block_warehouse_empty(); });

        std::cout << "size " << this->block_manager.block_warehouse.size() << std::endl;
        index = this->block_manager.block_warehouse.front().index;
        block = std::move(this->block_manager.block_warehouse.front()).data;
        this->block_manager.block_warehouse.pop();

        this->block_manager.cur_cached_block_num--;
        std::cout << "remain " << this->block_manager.cur_cached_block_num << std::endl;
    }
    asyn_read.notify_one();
}
inline auto BlockVolumeReader::getBlock()
{
    Block block;
    getBlock(block.data, block.index);
    return block;
}
inline void BlockVolumeReader::read_patch_blocks()
{
    std::vector<uint8_t> read_buffer;

    uint64_t read_batch_size = (uint64_t)block_length * block_length * (modify_x + 2 * padding) * voxel_size;
    read_buffer.assign(read_batch_size, DEFAULT_VALUE);
//    std::cout<<read_buffer.size()<<std::endl;
//    std::cout<<read_buffer.max_size()<<std::endl;
    
    uint32_t z = this->batch_read_turn / this->dim[1];
    uint32_t y = this->batch_read_turn % this->dim[1];
    uint64_t batch_slice_read_num = 0;
    uint64_t batch_slice_line_read_num = 0;
    uint64_t batch_read_pos = 0;//in bytes
    uint64_t batch_slice_size = block_length * (modify_x + 2 * padding) * voxel_size; //in bytes
    uint64_t raw_slice_size = raw_x * raw_y * voxel_size; //in bytes
    uint64_t y_offset, z_offset;//in voxel

    if (z == 0 && z == (dim[2] - 1))
    { // for z==1
        batch_slice_read_num = raw_z;
        z_offset = padding;
        if (y == 0 && y == (dim[1] - 1))
        {
            y_offset = padding;
            batch_read_pos = 0;
            batch_slice_line_read_num = raw_y;
        }
        else if (y == 0)
        {
            y_offset = padding;
            batch_read_pos = 0;
            batch_slice_line_read_num = block_length_nopadding + padding;
        }
        else if (y == (dim[1] - 1))
        {
            y_offset = 0;
            batch_read_pos = (uint64_t)(y * block_length_nopadding - padding) * raw_x * voxel_size;
            batch_slice_line_read_num = raw_y - y * block_length_nopadding + padding;
        }
        else
        {
            y_offset = 0;
            batch_read_pos = (uint64_t)(y * block_length_nopadding - padding) * raw_x * voxel_size;
            batch_slice_line_read_num = block_length_nopadding + 2 * padding;
        }
    }
    else if (z == 0)
    {
        batch_slice_read_num = block_length_nopadding + padding;
        z_offset = padding;
        if (y == 0 && y == (dim[1] - 1))
        {
            y_offset = padding;
            batch_read_pos = 0;
            batch_slice_line_read_num = raw_y;
        }
        else if (y == 0)
        {
            y_offset = padding;
            batch_read_pos = 0;
            batch_slice_line_read_num = block_length_nopadding + padding;
        }
        else if (y == (dim[1] - 1))
        {
            y_offset = 0;
            batch_read_pos = (uint64_t)(y * block_length_nopadding - padding) * raw_x * voxel_size;
            batch_slice_line_read_num = raw_y - y * block_length_nopadding + padding;
        }
        else
        {
            y_offset = 0;
            batch_read_pos = (uint64_t)(y * block_length_nopadding - padding) * raw_x * voxel_size;
            batch_slice_line_read_num = block_length_nopadding + 2 * padding;
        }
    }
    else if (z == (dim[2] - 1))
    {
        z_offset = 0;
        batch_slice_read_num = raw_z - z * block_length_nopadding + padding;
        if (y == 0 && y == (dim[1] - 1))
        {
            y_offset = padding;
            batch_read_pos = (uint64_t)(z * block_length_nopadding - padding) * raw_slice_size;
            batch_slice_line_read_num = raw_y;
        }
        else if (y == 0)
        {
            y_offset = padding;
            batch_read_pos = (uint64_t)(z * block_length_nopadding - padding) * raw_slice_size;
            batch_slice_line_read_num = block_length_nopadding + padding;
        }
        else if (y == (dim[1] - 1))
        {
            y_offset = 0;
            batch_read_pos = (uint64_t)(z * block_length_nopadding - padding) * raw_slice_size +
                             (y * block_length_nopadding - padding) * raw_x * voxel_size;
            batch_slice_line_read_num = raw_y - y * block_length_nopadding + padding;
        }
        else
        {
            y_offset = 0;
            batch_read_pos = (uint64_t)(z * block_length_nopadding - padding) * raw_slice_size +
                             (y * block_length_nopadding - padding) * raw_x * voxel_size;
            batch_slice_line_read_num = block_length_nopadding + 2 * padding;
        }
    }
    else
    {
        z_offset = 0;
        batch_slice_read_num = block_length;

        if (y == 0 && y == (dim[1] - 1))
        {
            y_offset = padding;
            batch_read_pos = (uint64_t)(z * block_length_nopadding - padding) * raw_slice_size ;
            batch_slice_line_read_num = raw_y;
        }
        else if (y == 0)
        {
            y_offset = padding;
            batch_read_pos = (uint64_t)(z * block_length_nopadding - padding) * raw_slice_size;
            batch_slice_line_read_num = block_length_nopadding + padding;
        }
        else if (y == (dim[1] - 1))
        {
            y_offset = 0;
            batch_read_pos = (uint64_t)(z * block_length_nopadding - padding) * raw_slice_size +
                             (y * block_length_nopadding - padding) * raw_x * voxel_size;
            batch_slice_line_read_num = raw_y - y * block_length_nopadding + padding;
        }
        else
        {
            y_offset = 0;
            batch_read_pos = (uint64_t)(z * block_length_nopadding - padding) * raw_slice_size +
                             (y * block_length_nopadding - padding) * raw_x * voxel_size;
            batch_slice_line_read_num = block_length_nopadding + 2 * padding;
        }
    }

    for (uint64_t i = 0; i < batch_slice_read_num; i++)
    {
//        in.seekg(batch_read_pos + i * raw_slice_size, std::ios::beg);
        stream->seekg(batch_read_pos,i);

        std::vector<uint8_t> read_batch_slice_buffer;
        read_batch_slice_buffer.assign(raw_x * voxel_size * batch_slice_line_read_num, DEFAULT_VALUE);

//        in.read(reinterpret_cast<char *>(read_batch_slice_buffer.data()), read_batch_slice_buffer.size());
        stream->read(reinterpret_cast<char *>(read_batch_slice_buffer.data()),read_batch_slice_buffer.size());

        for (uint64_t j = 0; j < batch_slice_line_read_num; j++)
        {
            uint64_t offset = (z_offset + i) * (modify_x + 2 * padding) * block_length +
                              (y_offset + j) * (modify_x + 2 * padding) + padding;
            offset *= voxel_size;
            memcpy(read_buffer.data() + offset, read_batch_slice_buffer.data() + j * raw_x * voxel_size, raw_x * voxel_size);
        }
    }

    std::unique_lock<std::mutex> lk(batch_blocks_mutex);
    asyn_read.wait(lk, [&]() { return read_patch_enable(); });

    uint64_t batch_length = modify_x + 2 * padding;
    for (uint32_t i = 0; i < this->block_num_per_batch; i++)
    {
        Block block;
        block.index = {i, y, z};
        block.data.assign(block_byte_size, DEFAULT_VALUE);
        for (size_t z_i = 0; z_i < block_length; z_i++)
        {
            for (size_t y_i = 0; y_i < block_length; y_i++)
            {
                uint64_t offset = z_i * batch_slice_size + y_i * batch_length * voxel_size + i * block_length_nopadding * voxel_size;
                memcpy(block.data.data() + (z_i * block_length * block_length + y_i * block_length)*voxel_size,
                       read_buffer.data() + offset, block_length * voxel_size);
            }
        }

        this->block_manager.cur_cached_block_num++;
        this->block_manager.block_warehouse.push(std::move(block));
    }

    this->batch_read_turn++;

    read_buffer.clear();

    std::cout << "finish read batch" << std::endl;

    asyn_get.notify_one();
}

bool BlockVolumeReader::read_patch_enable()
{
    if (this->block_manager.cur_cached_block_num + this->block_num_per_batch <=
        this->block_manager.max_cached_block_num)
    {
        std::cout << "cur_cached_block_num: " << this->block_manager.cur_cached_block_num << std::endl;
        std::cout << "max_cached_block_num: " << this->block_manager.max_cached_block_num << std::endl;
        return true;
    }
    else
    {
        return false;
    }
}

inline bool BlockVolumeReader::is_block_warehouse_empty()
{
    return this->block_manager.cur_cached_block_num == 0;
}

std::array<uint32_t, 3> BlockVolumeReader::getDim() const
{
    return this->dim;
}
void BlockVolumeReader::set_batch_read_turn_and_count(int turn, int count)
{
    this->batch_read_turn = turn;
    this->batch_read_cnt = turn + count;
}
void BlockVolumeReader::set_raw_stream(std::unique_ptr<RawStream> stream)
{
    this->stream = std::move(stream);
}
