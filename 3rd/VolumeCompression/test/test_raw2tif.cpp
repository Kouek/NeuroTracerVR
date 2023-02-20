//
// Created by wyz on 2021/4/13.
//
#include<tinytiffwriter.h>
#include<string>
#include<fstream>
#include<iostream>
#include<vector>
#include<iomanip>
#include<cassert>
#include<sstream>

int main(){

    int raw_x=256;
    int raw_y=256;
    int raw_z=256;
    std::string file_name="aneurism_256_256_256_uint8.raw";
    std::ifstream in(file_name,std::ios::binary);
    std::vector<uint8_t> data(size_t(raw_x)*raw_y*raw_z);
    if(!in.is_open()){
        std::cout<<"file open failed: "<<file_name<<std::endl;
    }
    else{
        std::cout<<"data.size(): "<<data.size()<<std::endl;
        in.read(reinterpret_cast<char*>(data.data()),data.size());
    }
    in.close();
    for(int i=0;i<raw_z;i++){
        std::stringstream ss;
        ss<<std::setw(3)<<std::setfill('0')<<std::to_string(i+1)<<".tif";
        TinyTIFFWriterFile* tif=TinyTIFFWriter_open(("./raw2tif/test_"+ss.str()).c_str(),8,TinyTIFFWriter_UInt,1,raw_x,raw_y,TinyTIFFWriter_Greyscale);
        if(tif){
            TinyTIFFWriter_writeImage(tif,data.data()+(size_t)i*raw_x*raw_y);
        }
        else{
            std::cout<<"writing tif open failed: "<<"test_"<<ss.str()<<std::endl;
        }
        TinyTIFFWriter_close(tif);
    }
    std::cout<<"all tif finish"<<std::endl;
}
