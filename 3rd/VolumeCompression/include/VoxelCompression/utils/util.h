//
// Created by wyz on 2021/4/13.
//

#pragma once

#include<vector>
#include<cassert>
#include<string>
#include<iostream>


#ifdef _WIN32

#include <windows.h>

unsigned long long get_system_memory() {
    MEMORYSTATUSEX status;
    status.dwLength = sizeof(status);
    GlobalMemoryStatusEx(&status);
    return status.ullTotalPhys;
}

#else
#include <unistd.h>
unsigned long long get_system_memory()
{
    long pages = sysconf( _SC_PHYS_PAGES );
    long page_size = sysconf( _SC_PAGE_SIZE );
    return pages * page_size;
}
#endif

#if defined(VoxelResampleIMPL) || defined(TinyTIFFIMPL)
#include<tinytiffreader.h>
//notice: STB_IMAGE_IMPLEMENTATION just need define once in .c or .cpp file
//#define STB_IMAGE_IMPLEMENTATION
//#include<stb_image.h>//can't load .tif
//just for volume tif that frame is 1 and data format is unsigned char(uint8_t)
void load_volume_tif(std::vector<uint8_t>& data,const std::string& path)
{
    TinyTIFFReaderFile* tiffr=nullptr;
    tiffr=TinyTIFFReader_open(path.c_str());
    if(!tiffr){
        std::cout<<"    ERROR reading (not existent, not accessible or no TIFF file)\n";
    }
    else{
        if(TinyTIFFReader_wasError(tiffr)){
            std::cout<<"   ERROR:"<<TinyTIFFReader_getLastError(tiffr)<<"\n";
        }
        else{
            uint32_t frames=TinyTIFFReader_countFrames(tiffr);
            assert(frames==1);
            if (TinyTIFFReader_wasError(tiffr)) {
                std::cout<<"   ERROR:"<<TinyTIFFReader_getLastError(tiffr)<<"\n";
            }
            else{
                uint32_t width=TinyTIFFReader_getWidth(tiffr);
                uint32_t height=TinyTIFFReader_getHeight(tiffr);
                uint16_t samples=TinyTIFFReader_getSamplesPerPixel(tiffr);
                uint16_t bitspersample=TinyTIFFReader_getBitsPerSample(tiffr, 0);
                assert(samples==1 && bitspersample==8);
//                std::cout<<"width: "<<width<<" height: "<<height<<std::endl;
                data.resize(width*height);
                TinyTIFFReader_getSampleData(tiffr,data.data(),0);
                if (TinyTIFFReader_wasError(tiffr)){
                    std::cout<<"   ERROR:"<<TinyTIFFReader_getLastError(tiffr)<<"\n";
                }
            }

        }
    }
    TinyTIFFReader_close(tiffr);
}
#endif