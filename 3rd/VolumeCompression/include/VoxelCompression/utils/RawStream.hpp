//
// Created by wyz on 2022/3/18.
//

#pragma once
#include <cassert>
#include <functional>
#include <fstream>
#include <iostream>
#include <vector>
#include "TifHelper.hpp"
//http://www.libtiff.org/libtiff.html

class RawStream{
  public:

    RawStream(uint32_t x,uint32_t y,uint32_t z,size_t EleSize)
    :raw_x(x),raw_y(y),raw_z(z),ele_size(EleSize)
    {
        raw_size = (size_t)raw_x * raw_y * raw_z;
        tif_size = (size_t)raw_x * raw_y ;
        tif_size_bytes = tif_size * ele_size;
    }
    /**
     * @brief pos = offset + z * raw_x * raw_y * EleSize
     * @param z slice index in z-aix
     * @param offset in the slice of index z count in *bytes*
     */
    void seekg(size_t offset,uint32_t z){
        if(is_raw){
            assert(in.is_open());
            in.seekg(offset + (size_t)z * raw_x * raw_y * ele_size,std::ios::beg);
        }
        else{
            z = z + offset / tif_size_bytes;
            prepare(z);
            offset = offset % tif_size_bytes;
            tif_offset_bytes = offset;
        }
    }

    /**
     * @param size count in *bytes*
     */
    void read(char* buffer,size_t size){
        if(is_raw){
            assert(in.is_open());
            in.read(buffer,size);
        }
        else{
//            memcpy(buffer,tif_data+tif_offset,size);
            uint32_t row = tif_offset_bytes / (raw_x * ele_size);
            uint32_t rows = size / (raw_x * ele_size);
            tif_stream.readRow(buffer,row,rows);
        }
    }

    //for raw
    void open(const std::string& name){
        in.open(name,std::ios::binary|std::ios::beg);
        if(!in.is_open()){
            throw std::runtime_error("open raw file failed");
        }
        is_raw = true;
    }
    //for tif
    void setNamePattern(std::function<std::string(uint32_t z)> read,std::function<std::string(uint32_t z)> write){

        gen_read_name = std::move(read);
        gen_save_name = std::move(write);
        is_raw = false;
    }
    //for tif

    void setTifReadAndResampleWrite(){
        this->resample_and_write = true;
        this->saved.resize(raw_z,false);
    }

    void setResampleFunc(std::function<void(void*,void*,const std::string&)> f){
        if(!f){
            throw std::runtime_error("empty name resample func");
        }
        this->resample_f = f;
    }

    void close(){
        if(in.is_open())
            in.close();
    }
    ~RawStream(){
        close();
        deleteTIFBuffer(&tif_data);
        deleteTIFBuffer(&pre_tif_data);
        tif_stream.close();
    }
  private:
    uint32_t raw_x,raw_y,raw_z;
    size_t raw_size;
    size_t ele_size;
    bool is_raw;
    std::function<std::string(uint32_t)> gen_read_name,gen_save_name;
    int cur_z{-1};
    std::ifstream in;//for raw
    void* tif_data{nullptr};
    size_t tif_size;
    size_t tif_offset_bytes;
    size_t tif_size_bytes;
    bool resample_and_write{false};

    std::vector<bool> saved;
    void* pre_tif_data{nullptr};
    int pre_z{-1};
    std::function<void(void*,void*,std::string)> resample_f;
    TIFInStream tif_stream;
    void genTIFBuffer(void** buffer){
        *buffer = ::operator new(tif_size_bytes);
        if(*buffer == nullptr){
            throw std::runtime_error("malloc tif data failed");
        }
    }
    void deleteTIFBuffer(void** buffer){
        if(*buffer == nullptr) return;
        ::operator delete(*buffer);
        *buffer = nullptr;
    }
    void prepare(uint32_t z){
        if(is_raw) {
            assert(in.is_open());
            return;
        }
        if(cur_z == z) return;
        if(!gen_read_name){
            throw std::runtime_error("empty read name pattern");
        }
        auto name = gen_read_name(z);
        tif_stream.close();
        bool ok = tif_stream.open(name);
        if(!ok){
            throw std::runtime_error("open tif file failed");
        }
        if(!tif_stream.same(raw_x,raw_y,ele_size)){
            throw std::runtime_error("tif size error");
        }

        if(!resample_and_write) return;

        if(!tif_data && cur_z == -1 && resample_and_write){
            genTIFBuffer(&tif_data);
        }
        if(!pre_tif_data && pre_z == -1 && resample_and_write){
            genTIFBuffer(&pre_tif_data);
        }

//        tif_offset_bytes = 0;
        if(saved[z]){

            return ;
        }
        pre_z = cur_z;
        cur_z = z;
        std::swap(pre_tif_data,tif_data);

        if(!resample_and_write) return;
        if(!gen_save_name){
            throw std::runtime_error("empty save name pattern");
        }
        tif_stream.readTif(tif_data);

        if(pre_z == -1) return;
        if(pre_z/2 == cur_z /2)
        {

            resample_f(pre_tif_data, tif_data, gen_save_name(pre_z/2));
            saved[pre_z] = true;
            saved[z] = true;
        }
    }

};