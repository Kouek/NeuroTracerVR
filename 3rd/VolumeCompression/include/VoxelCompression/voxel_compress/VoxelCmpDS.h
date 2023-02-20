//
// Created by wyz on 2021/3/1.
//

#pragma once

#include <array>
#include <condition_variable>
#include <cstring>
#include <fstream>
#include <iostream>
#include <json.hpp>//todo not find in other program
#include <map>
#include <mutex>
#include <vector>
#include <unordered_set>

#define VOXEL_COMPRESS_FILE_IDENTIFIER 0x123456
#define VOXEL_COMPRESS_VERSION 1
#define VOXEL_UNKNOWN 0
#define VOXEL_UINT8 1
#define VOXEL_INT8 2
#define VOXEL_UINT16 3
#define VOXEL_INT16 4
#define VOXEL_FLOAT16 5
#define VOXEL_UINT32 6
#define VOXEL_INT32 7
#define VOXEL_FLOAT32 8
#define VOXEL_UINT64 9
#define VOXEL_INT64 10
#define VOXEL_FLOAT64 11
namespace sv
{
inline size_t GetVoxelSize(uint32_t tp)
{
    size_t size = 0;
    switch (tp)
    {
    case VOXEL_UNKNOWN:
        break;
    case VOXEL_UINT8:
    case VOXEL_INT8:
        size = 1;
        break;
    case VOXEL_UINT16:
    case VOXEL_INT16:
    case VOXEL_FLOAT16:
        size = 2;
        break;
    case VOXEL_UINT32:
    case VOXEL_INT32:
    case VOXEL_FLOAT32:
        size = 4;
        break;
    case VOXEL_UINT64:
    case VOXEL_INT64:
    case VOXEL_FLOAT64:
        size = 8;
        break;
    default:
        size = 0;
    }

    return size;
}
struct Header
{
    uint64_t identifier;
    uint64_t version;
    uint32_t raw_x;
    uint32_t raw_y;
    uint32_t raw_z;
    uint32_t block_dim_x;
    uint32_t block_dim_y;
    uint32_t block_dim_z;
    uint32_t log_block_length;
    uint32_t padding;
    uint32_t frame_width;
    uint32_t frame_height;
    //    uint64_t codec_method;
    uint32_t codec_method;
    uint32_t voxel = 1;
    friend std::ostream &operator<<(std::ostream &os, const Header &header)
    {
        os << "[Header Info]"
           << "\n[identifier]: " << header.identifier << "\n[version]: " << header.version
           << "\n[raw_x,raw_y,raw_z]: " << header.raw_x << " " << header.raw_y << " " << header.raw_z
           << "\n[block dim]: " << header.block_dim_x << " " << header.block_dim_y << " " << header.block_dim_z
           << "\n[log_block_length]: " << header.log_block_length << "\n[padding]: " << header.padding
           << "\n[frame_width]: " << header.frame_width << "\n[frame_height]: " << header.frame_height
           << "\n[codec_method]: " << header.codec_method << "\n[voxel]: " << header.voxel << std::endl;
        return os;
    }
};
struct Index
{
    uint32_t index_x;
    uint32_t index_y;
    uint32_t index_z;
    uint32_t reserve;
    uint64_t offset;
    uint64_t size;
    friend std::ostream &operator<<(std::ostream &os, const Index &index)
    {
        os << "[Index Info]"
           << "\n[index_x,index_y,index_z]: " << index.index_x << " " << index.index_y << " " << index.index_z
           << "\n[reserve]: " << index.reserve << "\n[offset]: " << index.offset << "\n[size]: " << index.size
           << std::endl;
        return os;
    }
    bool operator==(const Index& idx) const{
        return index_x == idx.index_x && index_y == idx.index_y && index_z == idx.index_z;
    }
};
}
namespace std{
    template<>
    struct hash<sv::Index>{
        size_t operator()(const sv::Index& idx) const{
            size_t a = idx.index_x;
            size_t b = idx.index_y;
            size_t c = idx.index_z;
            size_t t = a ^ (b + 0x9e3779b97f4a7c15 + (a << 6) + (a >> 2));
            return t ^ (c + 0x9e3779b97f4a7c15 + (t << 6) + (t >> 2));
        }
    };
}

namespace sv{
class Reader
{
  public:
    Reader(const char *file_name)
    {
        in.open(file_name, std::ios::in | std::ios::binary);
        if (!in.is_open())
        {
            std::cout << "File open failed!" << std::endl;
            throw std::runtime_error("Volume data file open failed!");
        }
    }
    Reader(std::fstream &&in) : in(std::forward<std::fstream>(in))
    {
    }
    void read_header()
    {
        read_header(this->header);
    }
    void read_header(sv::Header &header)
    {
        in.seekg(std::ios::beg);
        in.read(reinterpret_cast<char *>(&header), sizeof(header));
        std::cout << header;
        if (header.identifier != VOXEL_COMPRESS_FILE_IDENTIFIER)
            throw std::runtime_error("file format error!");
        index_beg = sizeof(header);
        block_num = header.block_dim_x * header.block_dim_y * header.block_dim_z;
        data_beg = index_beg + block_num * sizeof(sv::Index);
        reader_indexes(this->indexes);
    }
    void reader_indexes(std::unordered_set<Index> &indexes)
    {
        indexes.clear();
        in.seekg(std::ios::beg + index_beg);

        for (size_t i = 0; i < block_num; i++)
        {
            Index index{};
            in.read(reinterpret_cast<char *>(&index), sizeof(Index));
            indexes.insert(index);
        }
    }
    void read_packet(const std::array<uint32_t, 3> &index, std::vector<std::vector<uint8_t>> &packet)
    {
        {
            std::unique_lock<std::mutex> lk(mtx);
            cv.wait(lk, []() { return true; });
            auto it = indexes.find(Index{index[0],index[1],index[2]});
            if(it != indexes.end())
            {

                in.seekg(std::ios::beg + data_beg + it->offset);
                uint64_t offset = 0, frame_size = 0;
                while (offset < it->size)
                {
                    in.read(reinterpret_cast<char *>(&frame_size), sizeof(uint64_t));
                    std::vector<uint8_t> tmp;
                    tmp.resize(frame_size);
                    in.read(reinterpret_cast<char *>(tmp.data()), frame_size);
                    packet.push_back(tmp);
                    offset += sizeof(uint64_t) + frame_size;
                }

            }
            else{
                std::cerr<<"not find block for index: "<<index[0]<<" "<<index[1]<<" "<<index[2]<<std::endl;
            }
        }
        cv.notify_one();
    }

    sv::Header get_header() const
    {
        return this->header;
    }

  private:
    sv::Header header;
//    std::vector<sv::Index> indexes;
    std::unordered_set<sv::Index> indexes;
    uint32_t block_num;
    uint64_t index_beg;
    uint64_t data_beg;
    std::fstream in;

    std::mutex mtx;
    std::condition_variable cv;
};

class Writer
{
  public:
    Writer(const char *file_name)
    {
        out.open(file_name, std::ios::out | std::ios::binary);
        if (!out.is_open())
        {
            std::cout << "File open failed!" << std::endl;
        }
        header_beg = 0;
        index_beg = 0;
        data_beg = 0;
        index_offset = 0;
        data_offset = 0;
    }
    ~Writer()
    {
        out.close();
    }
    void write_header(const sv::Header &header)
    {
        uint64_t block_num = header.block_dim_x * header.block_dim_y * header.block_dim_z;
        this->index_beg = sizeof(header);
        this->data_beg = block_num * sizeof(sv::Index) + this->index_beg;
        out.seekp(std::ios::beg);
        out.write(reinterpret_cast<const char *>(&header), sizeof(header));
    }

    void write_packet(const std::array<uint32_t, 3> &index, const std::vector<std::vector<uint8_t>> &packet)
    {
        std::unique_lock<std::mutex> lk(mtx);
        cv.wait(lk, []() { return true; });

        uint64_t packet_size = 0;
        out.seekp(std::ios::beg + data_beg + data_offset);
        for (size_t i = 0; i < packet.size(); i++)
        {
            packet_size += sizeof(uint64_t) + packet[i].size();
            uint64_t frame_size = packet[i].size();
            out.write(reinterpret_cast<char *>(&frame_size), sizeof(uint64_t));
            out.write(reinterpret_cast<const char *>(packet[i].data()), frame_size);
        }

        sv::Index idx{index[0], index[1], index[2], (uint32_t)packet.size(), data_offset, packet_size};
        out.seekp(std::ios::beg + index_beg + index_offset);
        out.write(reinterpret_cast<const char *>(&idx), sizeof(idx));
        index_offset += sizeof(idx);
        data_offset += packet_size;

        cv.notify_one();
    }

  private:
    std::fstream out;
    uint64_t header_beg;
    uint64_t index_beg;
    uint64_t data_beg;
    uint64_t index_offset;
    uint64_t data_offset;

    std::mutex mtx;
    std::condition_variable cv;
};

class LodFile
{
  public:
    LodFile() : valid(false), min_lod(-1), max_lod(-1){};
    void open_lod_file(const std::string &path);
    bool add_lod_file(int lod, const std::string &file_name);
    void save_lod_files(const std::string &name, const std::string &ext);
    int get_min_lod() const;
    int get_max_lod() const;
    auto get_volume_space() const -> std::array<float, 3>;
    std::string get_lod_file_path(int lod);

  private:
    void save_in_json(const std::string &name);
    void save_in_txt(const std::string &name);
    void open_in_json(const std::string &path);
    void open_in_txt(const std::string &path);

  private:
    bool valid;
    int min_lod, max_lod;
    std::array<float, 3> volume_space;
    std::map<int, std::string> lod_files;
};

inline void LodFile::open_lod_file(const std::string &path)
{

    auto ext_idx = path.find_last_of('.');
    if (ext_idx == std::string::npos)
    {
        throw std::runtime_error("error file ext!");
    }
    auto ext = path.substr(ext_idx);
    if (ext == ".json")
    {
        open_in_json(path);
    }
    else if (ext == ".txt")
    {
        open_in_txt(path);
    }
    else
    {
        throw std::runtime_error("only support json or txt!");
    }
}

inline void LodFile::open_in_txt(const std::string &path)
{
}

inline void LodFile::open_in_json(const std::string &path)
{
    nlohmann::json j;
    std::ifstream in(path);
    in >> j;
    min_lod = j["min_lod"];
    max_lod = j["max_lod"];
    volume_space = j.at("space");
    for (int i = min_lod; i <= max_lod; i++)
    {
        lod_files[i] = j[std::to_string(i)];
    }
    if (min_lod >= 0)
        valid = true;
}
inline bool LodFile::add_lod_file(int lod, const std::string &file_name)
{
    if (!valid && lod >= 0)
    {
        min_lod = max_lod = lod;
        lod_files[lod] = file_name;
        valid = true;
    }
    else if (valid && lod >= 0)
    {
        if (!lod_files[lod].empty())
        {
            std::cout << "lod: " << lod << " exists file: " << lod_files[lod] << " will replace by " << file_name
                      << std::endl;
        }
        if (lod < min_lod)
            min_lod = lod;
        else if (lod > max_lod)
            max_lod = lod;
        lod_files[lod] = file_name;
    }
    else
    {
        std::cout << "error: lod < 0" << std::endl;
        return false;
    }
}

inline void LodFile::save_lod_files(const std::string &name, const std::string &ext)
{
    if (ext == "json")
    {
        save_in_json(name);
    }
    else if (ext == "txt")
    {
        save_in_txt(name);
    }
    else
    {
        std::cout << "wrong ext! save failed!" << std::endl;
    }
}

inline void LodFile::save_in_json(const std::string &name)
{
    nlohmann::json j;
    j["max_lod"] = max_lod;
    j["min_lod"] = min_lod;
    for (int i = min_lod; i <= max_lod; i++)
    {
        j[std::to_string(i)] = lod_files[i];
    }
    std::ofstream out(name + ".json");
    out << j.dump(4);
}

inline void LodFile::save_in_txt(const std::string &name)
{
    throw std::runtime_error("Not imply method: LodFile::save_in_txt");
}

inline int LodFile::get_min_lod() const
{
    return min_lod;
}

inline int LodFile::get_max_lod() const
{
    return max_lod;
}

inline std::string LodFile::get_lod_file_path(int lod)
{
    if (!valid || lod < min_lod || lod > max_lod)
    {
        throw std::runtime_error("get lod out of range");
    }
    return lod_files[lod];
}
inline auto LodFile::get_volume_space() const -> std::array<float, 3>
{
    return volume_space;
}

} // namespace sv
