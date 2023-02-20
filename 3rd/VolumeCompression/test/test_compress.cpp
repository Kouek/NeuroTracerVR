//
// Created by wyz on 2021/2/5.
//
#include <fstream>
#include <iostream>
#include <vector>
#include <VoxelCompression/voxel_compress/VoxelCompress.h>
using namespace std;
#define volume_file_name_0 "D:/testoriginblock_12_15_5_uint8.raw"
#define volume_0_len 512*512*512
bool readVolumeData(uint8_t* &data, int64_t& len)
{
    std::fstream in(volume_file_name_0,ios::in|ios::binary);
    in.seekg(0,ios::beg);
    data=new uint8_t[volume_0_len];
    len=in.read(reinterpret_cast<char*>(data),volume_0_len).gcount();
    if(len==volume_0_len)
        return true;
    else
        return false;
}
int main(int argc,char** argv)
{
    VoxelCompressOptions opts;
    opts.height=512;
    opts.width=512;
    opts.input_buffer_format=NV_ENC_BUFFER_FORMAT_NV12;
    VoxelCompress v_cmp(opts);
    uint8_t* data=nullptr;
    int64_t len=0;
    readVolumeData(data,len);
    std::vector<std::vector<uint8_t>> packets;
    v_cmp.compress(data,len,packets);
    std::cout<<"Packet number is: "<<packets.size()<<std::endl;
    uint64_t packets_size=0;
    for(int i=0;i<packets.size();i++){
        std::cout<<"packet "<<i<<" size: "<<packets[i].size()<<std::endl;
        packets_size+=packets[i].size();
    }
    std::cout<<"Packets size is: "<<packets_size<<std::endl;
    std::string save_filename = "D:/testoriginblock_12_15_5_uint8.h264";
    std::ofstream out(save_filename,std::ios::binary);
    for(auto& p:packets){
        out.write(reinterpret_cast<char*>(p.data()),p.size());
    }
    out.close();

    return 0;
}

