//
// Created by wyz on 2021/4/14.
//
#include<tinytiffwriter.h>
#include<iostream>
#include<fstream>
#include<vector>
#include<string>
#include<cassert>
#define VoxelResampleIMPL
#include<VoxelCompression/utils/VoxelResample.h>

int main()
{
    size_t x,y;
    x=23389;y=29581;
    auto tif_size=x*y;
    std::vector<uint8_t> data1(tif_size);
    std::vector<uint8_t> data2(tif_size);
    load_volume_tif(data1,"D:/15107/z/test_02539.tif");
    load_volume_tif(data2,"D:/15107/z/test_02540.tif");
    std::vector<uint8_t> data;
    data.insert(data.cend(),data1.cbegin(),data1.cend());
    data.insert(data.cend(),data2.cbegin(),data2.cend());
    data1.clear();data2.clear();

    Worker worker;
    worker.set_busy(true);
    worker.setup_task<uint8_t>(x,y,data,[](uint8_t a, uint8_t b)->uint8_t {
        return (std::max)(a,b);
    });
    TinyTIFFWriterFile* tif=TinyTIFFWriter_open("test_1270_merge_max.tif",8,TinyTIFFWriter_UInt,1,(x+1)/2,(y+1)/2,TinyTIFFWriter_Greyscale);
    if(tif){
        TinyTIFFWriter_writeImage(tif,data.data());
    }
    else{
        std::cout<<"writing tif open failed: "<<"test_merge_max.tif"<<std::endl;
    }
    TinyTIFFWriter_close(tif);

    return 0;
}
