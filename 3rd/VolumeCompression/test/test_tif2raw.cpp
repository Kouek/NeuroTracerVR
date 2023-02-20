//
// Created by wyz on 2021/4/13.
//
#include<iostream>
#include<fstream>
#include<sstream>
#include<string>
#include<iomanip>
#include<VoxelCompression/utils/util.h>
int main(){

    int raw_x=256;
    int raw_y=256;
    int raw_z=256;
    std::vector<uint8_t> data;
    data.reserve((size_t)raw_x*raw_y*raw_z);
    for(int i=0;i<raw_z;i++){
        std::stringstream ss;
        ss<<"test_"<<std::setw(3)<<std::setfill('0')<<std::to_string(i+1)<<".tif";
        std::vector<uint8_t> slice;
        load_volume_tif(slice,ss.str());
        data.insert(data.cend(),slice.cbegin(),slice.cend());
    }
    std::ofstream out("tif2raw_256_256_256_uint8.raw",std::ios::binary);
    if(!out.is_open()){
        std::cout<<"write file open failed"<<std::endl;
        exit(-1);
    }
    out.write(reinterpret_cast<char*>(data.data()),data.size());
    out.close();
    return 0;
}