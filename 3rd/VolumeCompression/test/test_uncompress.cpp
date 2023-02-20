//
// Created by wyz on 2021/2/7.
//
#include<VoxelCompression/voxel_uncompress/VoxelUncompress.h>
#include<VoxelCompression/utils/NvCodecUtils.h>
#include<VoxelCompression/voxel_compress/VoxelCompress.h>
#include<fstream>
#include<iostream>
#include<thread>
using namespace std;
#define volume_file_name_0 "aneurism_256_256_256_uint8.raw"
#define volume_file_name_1 "test_512_512_512_uint8.raw"
#define volume_0_len 256*256*256
#define volume_1_len 512*512*512
bool readVolumeData(uint8_t* &data, int64_t& len)
{
    std::fstream in(volume_file_name_1,ios::in|ios::binary);
    if(!in.is_open()){
        throw std::runtime_error("file open failed");
    }
    in.seekg(0,ios::beg);
    data=new uint8_t[volume_1_len];
    len=in.read(reinterpret_cast<char*>(data),volume_1_len).gcount();
    if(len==volume_1_len)
        return true;
    else
        return false;
}
int main(int argc,char** argv)
{
    checkCUDAErrors(cuInit(0));
    CUdevice cuDevice=0;
    checkCUDAErrors(cuDeviceGet(&cuDevice, 0));
    CUcontext cuContext = nullptr;
    checkCUDAErrors(cuCtxCreate(&cuContext,0,cuDevice));
    uint8_t* res_ptr=nullptr;
    int64_t length=512*512*512;
    checkCUDAErrors(cuMemAlloc((CUdeviceptr*)&res_ptr,length));//driver api prefix cu
    uint8_t* res_ptr1=nullptr;
    int64_t length1=512*512*512;
    checkCUDAErrors(cuMemAlloc((CUdeviceptr*)&res_ptr1,length1));//driver api prefix cu
    uint8_t* res_ptr2=nullptr;
    checkCUDAErrors(cuMemAlloc((CUdeviceptr*)&res_ptr2,length));//driver api prefix cu

//    cudaMalloc((void**)(CUdeviceptr*)&res_ptr,length);//runtime api prefix cuda

    std::cout<<"compress"<<std::endl;
    VoxelCompressOptions c_opts;
    c_opts.height=2048;
    c_opts.width=2048;
    c_opts.input_buffer_format=NV_ENC_BUFFER_FORMAT_NV12;
    VoxelCompress v_cmp(c_opts);
    uint8_t* data=nullptr;
    int64_t len=0;
    readVolumeData(data,len);
    std::vector<std::vector<uint8_t>> packets;
    std::vector<std::vector<uint8_t>> packets1;
    v_cmp.compress(data,len,packets);
    v_cmp.compress(data,len,packets1);
    int64_t cmp_size=0;
    for(size_t i=0;i<packets.size();i++){
        cmp_size+=packets[i].size();
    }
    std::cout<<"packets size is: "<<cmp_size<<std::endl;


//    auto packets1=packets;
//    auto packets2=packets;
    std::cout<<"uncompress"<<std::endl;
    VoxelUncompressOptions opts;
    opts.height=2048;
    opts.width=2048;
    opts.cu_ctx=nullptr;
    VoxelUncompress v_uncmp(opts);
//    std::cout<<"v 0"<<std::endl;
//    VoxelUncompress v_uncmp1(opts);
//    std::cout<<"v 1"<<std::endl;
//    VoxelUncompress v_uncmp2(opts);
//    std::cout<<"v 2"<<std::endl;

    auto task=[](VoxelUncompress& v,uint8_t* res_ptr,int64_t length,std::vector<std::vector<uint8_t>>& p){
        v.uncompress(res_ptr,length,p);
    };


    auto _start=std::chrono::steady_clock::now();

    v_uncmp.uncompress(res_ptr,length,packets);
    v_uncmp.uncompress(res_ptr1,length,packets1);
//    v_uncmp1.uncompress(res_ptr1,length1,packets1);
//    v_uncmp2.uncompress(res_ptr2,length1,packets2);
//    std::thread t1(task,std::ref(v_uncmp),res_ptr,length,std::ref(packets));
//    std::thread t2(task,std::ref(v_uncmp1),res_ptr1,length1,std::ref(packets1));
//    std::thread t3(task,std::ref(v_uncmp2),res_ptr2,length1,std::ref(packets2));
//    t1.join();
//    t2.join();
//    t3.join();
    auto _end=std::chrono::steady_clock::now();
    auto _t=std::chrono::duration_cast<std::chrono::milliseconds>(_end-_start);
    std::cout<<"cpu cost time is:"<<_t.count()<<"ms"<<std::endl;

    std::vector<uint8_t> uncmp_res;
    uncmp_res.resize(length);
    auto res=cuMemcpyDtoH(uncmp_res.data(), (CUdeviceptr)(res_ptr), length);
    if(res!=CUDA_SUCCESS){
        throw std::runtime_error("cuMemcpyDtoH failed");
    }
    std::fstream out("uncmpres_512_512_512_uint8.raw",std::ios::out|std::ios::binary);
    out.write(reinterpret_cast<const char *>(uncmp_res.data()), uncmp_res.size());
    out.close();
    std::cout<<"finish"<<std::endl;
    return 0;
}
