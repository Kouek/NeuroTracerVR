//
// Created by wyz on 2022/3/19.
//
#include <VoxelCompression/utils/TifHelper.hpp>
#include <tiffio.h>
#include <cassert>
#include <iostream>
struct TIFInStream::Impl{
    TIFF* tif{nullptr};
    uint32_t tif_width = 0;
    uint32_t tif_height = 0;
    uint32_t bits_per_sampler = 0;
    uint32_t rows_per_strip = 0;


    bool open(const std::string& filename,std::string mode){
        tif = TIFFOpen(filename.c_str(),mode.c_str());
        if(!tif) return false;
        TIFFGetField(tif,TIFFTAG_IMAGEWIDTH,&tif_width);
        TIFFGetField(tif,TIFFTAG_IMAGELENGTH,&tif_height);
        TIFFGetField(tif,TIFFTAG_BITSPERSAMPLE,&bits_per_sampler);
        TIFFGetField(tif,TIFFTAG_ROWSPERSTRIP,&rows_per_strip);

        return true;
    }
    bool readTif(void* src){
        return readRow(src,0,tif_height);
    }
    bool readRow(void* src,uint32_t row,uint32_t nrows){
        if(!tif) return false;
        bool s = true;
        for(auto r= row;r<row + nrows;r++){
            size_t offset = (size_t) (r-row) * tif_width * bits_per_sampler / 8;
            int e = TIFFReadScanline(tif,(uint8_t*)src + offset,r,0);
            if(e == -1){
                s = false;
                printf("read row %d failed\n",r);
            }
        }
        return s;
    }
    void close(){
        if(!tif) return;
        TIFFClose(tif);
        tif = nullptr;
    }
    bool same(uint32_t width, uint32_t height, uint32_t ele_size){
        return tif_width == width && height == tif_height && ele_size == bits_per_sampler / 8;
    }
    void get(uint32_t& width,uint32_t& height,uint32_t& bits_per_sampler,uint32_t& rows_per_strip){
        width = tif_width;
        height = tif_height;
        bits_per_sampler = this->bits_per_sampler;
        rows_per_strip = this->rows_per_strip;
    }
};
bool TIFInStream::open(const std::string &filename,std::string mode)
{
    return impl->open(filename,mode);
}
bool TIFInStream::readTif(void *src)
{
    return impl->readTif(src);
}
bool TIFInStream::readRow(void *src, uint32_t row, uint32_t nrows)
{
    return impl->readRow(src,row,nrows);
}
void TIFInStream::close()
{
    impl->close();

}
TIFInStream::~TIFInStream()
{
    close();
}
TIFInStream::TIFInStream()
{
    impl = std::make_unique<Impl>();
}
bool TIFInStream::same(uint32_t width, uint32_t height, uint32_t ele_size)
{
    return impl->same(width,height,ele_size);
}
void TIFInStream::get(uint32_t &width, uint32_t &height, uint32_t &bits_per_sampler, uint32_t &rows_per_strip)
{
    impl->get(width,height,bits_per_sampler,rows_per_strip);
}
//----------------------------------------------------
struct TIFOutStream::Impl{
    uint32_t tif_width = 0;
    uint32_t tif_height = 0;
    uint32_t bits_per_sampler = 0;
    uint32_t rows_per_strip = 0;
    TIFF* tif = nullptr;
    bool open(const std::string& filename,std::string mode){
        tif = TIFFOpen(filename.c_str(),mode.c_str());
        if(!tif){
            std::cerr<<filename<<" "<<mode<<std::endl;
            return false;
        }
        TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
        TIFFSetField(tif,TIFFTAG_PHOTOMETRIC,PHOTOMETRIC_MINISBLACK);
        TIFFSetField(tif,TIFFTAG_SAMPLESPERPIXEL,1);
        TIFFSetField(tif,TIFFTAG_PLANARCONFIG,PLANARCONFIG_CONTIG);
        return true;
    }
    void setTIFInfo(TIFInfo info){
        if(!tif) return;
        tif_width = info.width;
        tif_height = info.height;
        bits_per_sampler = info.bits_per_sampler;
        rows_per_strip = info.rows_per_strip;
        TIFFSetField(tif,TIFFTAG_IMAGEWIDTH,tif_width);
        TIFFSetField(tif,TIFFTAG_IMAGELENGTH,tif_height);
        TIFFSetField(tif,TIFFTAG_BITSPERSAMPLE,bits_per_sampler);
        TIFFSetField(tif,TIFFTAG_ROWSPERSTRIP,rows_per_strip);
        TIFFSetField(tif,TIFFTAG_COMPRESSION,info.compression?COMPRESSION_LZW:COMPRESSION_NONE);
    }
    void writeTif(void* src){
        writeRow(src,0,tif_height);
    }
    void writeRow(void* src,uint32_t row,uint32_t nrows){
        assert(tif && check());
        assert(row < tif_height);
        for(auto r = row;r<row+nrows;r++){
            size_t offset = (size_t)(r-row) * tif_width * bits_per_sampler / 8;
            TIFFWriteScanline(tif,(uint8_t*)src + offset,r,0);
        }
    }
    bool check(){
        return tif_width && tif_height && bits_per_sampler && rows_per_strip;
    }
    void close(){
        if(!tif) return;
        TIFFClose(tif);
        tif = nullptr;
    }
};
bool TIFOutStream::open(const std::string &filename, std::string mode)
{
    return impl->open(filename,mode);
}
void TIFOutStream::setTIFInfo(TIFOutStream::TIFInfo info)
{
    impl->setTIFInfo(info);
}
void TIFOutStream::writeTif(void *src)
{
    impl->writeTif(src);
}
void TIFOutStream::writeRow(void *src, uint32_t row, uint32_t nrows)
{
    impl->writeRow(src,row,nrows);
}
void TIFOutStream::close()
{
    impl->close();
}
TIFOutStream::TIFOutStream()
{
    impl = std::make_unique<Impl>();
}
TIFOutStream::~TIFOutStream()
{
    close();
}
