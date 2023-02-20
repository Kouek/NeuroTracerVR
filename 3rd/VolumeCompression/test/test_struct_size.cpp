//
// Created by wyz on 2021/3/1.
//

#include<iostream>
#include<VoxelCompression/voxel_compress/VoxelCmpDS.h>

int main(int argc,char** argv)
{

    std::cout<<"size of Header: "<<sizeof(sv::Header)<<std::endl;
    sv::Header header;
    std::cout<<"size of Header instance: "<<sizeof(header)<<std::endl;
    std::cout<<"size of Index: "<<sizeof(sv::Index)<<std::endl;
    sv::Index index;
    std::cout<<"size of Index instance: "<<sizeof(index)<<std::endl;
    return 0;
}