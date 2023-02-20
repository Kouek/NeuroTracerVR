//
// Created by wyz on 2022/3/22.
//
#include <VoxelCompression/voxel_compress/VoxelCmpDS.h>
#include <VoxelCompression/voxel_uncompress/VoxelUncompress.h>

int main(){
    std::string h264_file_name = "D:/volumetifto264test_z_2540-3048_9p2_max_lod0_23389_29581_10296.h264";
//    std::string h264_file_name = "E:/MouseNeuronData/mouse_23389_29581_10296_9p2_lod0.h264";

    sv::Reader reader(h264_file_name.c_str());
    reader.read_header();
    std::array<uint32_t,3> block_index = {12,15,5};
    int block_length = 512;
    int ele_size = 1;
    int block_size_bytes = block_length * block_length * block_length * ele_size;
    std::vector<std::vector<uint8_t>> packets;
    reader.read_packet(block_index,packets);
    if(false){
//        std::string packets_fileaname = "D:/test_raw_packets_12_15_5.h264";
        std::string packets_fileaname = "D:/test_tif_packets_12_15_5.h264";
        std::ofstream p_out(packets_fileaname,std::ios::binary|std::ios::beg);
        assert(p_out.is_open());
        for(auto& p:packets){
            p_out.write(reinterpret_cast<char*>(p.data()),p.size());
        }
        p_out.close();
        return 0;
    }
    VoxelUncompressOptions opts;
    VoxelUncompress decoder(opts);



    std::vector<uint8_t> buffer(block_size_bytes,0);
    decoder.uncompress(buffer.data(),block_size_bytes,packets);

//    std::string out_file_name = "D:/mouselod0test_0_0_5_uint8.raw";
    std::string out_file_name = "D:/nvolumeblockfromtifh264test_12_15_5_uint8.raw";
    std::ofstream out(out_file_name,std::ios::binary);
    assert(out.is_open());
    out.write(reinterpret_cast<char*>(buffer.data()),buffer.size());
    out.close();
}