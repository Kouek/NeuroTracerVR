//
// Created by wyz on 2021/4/14.
//
#include<tinytiffwriter.h>
#include<iostream>
#include<fstream>
#include<vector>
#include<string>
#include<cassert>
int main(){

    size_t raw_x=1462;
    size_t raw_y=1849;
    size_t raw_z=644;
    size_t z=250;
    std::string file_name="mouselod4_1462_1849_644_uint8.raw";
    std::ifstream in(file_name,std::ios::binary);
    if(!in.is_open()){
        std::cout<<"open file failed: "<<file_name<<std::endl;
        exit(-1);
    }
    in.seekg(std::ios::end);
    assert(in.tellg()==raw_z*raw_y*raw_x);
    auto offset=raw_x*raw_y*z;
    in.seekg(offset,std::ios::beg);
    std::vector<uint8_t> data(raw_x*raw_y);
    in.read(reinterpret_cast<char*>(data.data()),data.size());
    TinyTIFFWriterFile* tif=TinyTIFFWriter_open(("test_"+std::to_string(z)+".tif").c_str(),8,TinyTIFFWriter_UInt,1,raw_x,raw_y,TinyTIFFWriter_Greyscale);
    if(tif){
        TinyTIFFWriter_writeImage(tif,data.data());
    }
    else{
        std::cout<<"writing tif open failed: "<<"test_"<<std::to_string(z)+".tif"<<std::endl;
    }
    TinyTIFFWriter_close(tif);
    return 0;
}
