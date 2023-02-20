//
// Created by wyz on 2021/3/1.
//

#include<iostream>
#include<VoxelCompression/voxel_compress/VoxelCmpDS.h>
#include<VoxelCompression/voxel_compress/VoxelCompress.h>
#include<VolumeReader/volume_reader.h>

int main(int argc,char** argv)
{
    const char* in_file_name="mouselod6avg_366_463_161_uint8.raw";
    const char* out_file_name="mouse_23389_29581_10296_9p2_lod6avg.h264";
    uint32_t m_raw_x=366;
    uint32_t m_raw_y=463;
    uint32_t m_raw_z=161;
    uint32_t m_log_block_length=9;
    uint32_t m_padding=2;
    uint32_t m_frame_width=512;
    uint32_t m_frame_height=512;
    uint64_t m_codec_method=1;

    uint32_t m_block_length=std::pow(2,m_log_block_length);
    BlockVolumeReader reader;
    reader.setupRawVolumeInfo({in_file_name,"",m_raw_x,m_raw_y,m_raw_z,m_block_length,m_padding});

    VoxelCompressOptions opts;
    opts.width=m_frame_width;
    opts.height=m_frame_height;
    VoxelCompress cmp(opts);



    sv::Header header{};
    header.identifier=VOXEL_COMPRESS_FILE_IDENTIFIER;
    header.version=VOXEL_COMPRESS_VERSION;
    header.raw_x=m_raw_x;
    header.raw_y=m_raw_y;
    header.raw_z=m_raw_z;
    auto dim=reader.get_dim();
    header.block_dim_x=dim[0];
    header.block_dim_y=dim[1];
    header.block_dim_z=dim[2];
    header.log_block_length=m_log_block_length;
    header.padding=m_padding;
    header.frame_width=m_frame_width;
    header.frame_height=m_frame_height;
    header.codec_method=m_codec_method;

    sv::Writer writer(out_file_name);
    writer.write_header(header);

    reader.start_read();
    while(!reader.is_read_finish() || !reader.isBlockWareHouseEmpty()){
        std::vector<uint8_t> block_data;
        std::array<uint32_t,3> block_index{};
        reader.get_block(block_data,block_index);
        std::vector<std::vector<uint8_t >> packets;
        cmp.compress(block_data.data(),block_data.size(),packets);
        writer.write_packet(block_index,packets);
    }

    return 0;
}