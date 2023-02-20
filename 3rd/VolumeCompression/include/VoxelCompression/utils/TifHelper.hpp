//
// Created by wyz on 2022/3/19.
//
#pragma once
#include <string>
#include <memory>
class TIFInStream
{
  public:
    TIFInStream();
    bool open(const std::string& filename,std::string mode = "r");
    //read entire tif image data
    bool readTif(void* src);

    bool readRow(void*src,uint32_t row,uint32_t nrows);

    void close();

    bool same(uint32_t width,uint32_t height,uint32_t ele_size);

    void get(uint32_t& width,uint32_t& height,uint32_t& bits_per_sampler,uint32_t& rows_per_strip);

    ~TIFInStream();
  private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

class TIFOutStream{
  public:
    bool open(const std::string& filename,std::string mode = "w");
    struct TIFInfo{
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t bits_per_sampler = 0;
        uint32_t rows_per_strip = 0;
        bool compression = false;
    };
    void setTIFInfo(TIFInfo info);

    void writeTif(void* src);

    void writeRow(void* src,uint32_t row,uint32_t nrows);

    void close();

    TIFOutStream();

    ~TIFOutStream();

  private:

    struct Impl;
    std::unique_ptr<Impl> impl;
};