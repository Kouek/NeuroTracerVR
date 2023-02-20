//
// Created by wyz on 2021/2/5.
//
#include <VoxelCompression/utils/NvCodecUtils.h>
#include <VoxelCompression/utils/NvEncoderCLIOptions.h>
#include <VoxelCompression/voxel_compress/VoxelCompress.h>
#include <cuda.h>
#include <iostream>
#include <memory>
#include <nvenc/CUDA/NvEncoderCuda.h>

class VoxelCompressImpl
{
  public:

    explicit VoxelCompressImpl(const VoxelCompressOptions &opts);
    ~VoxelCompressImpl()
    {
        encoder.reset();
        cuCtxDestroy(cu_ctx);
    }
    bool compress(uint8_t *src_ptr, int64_t len, std::vector<std::vector<uint8_t>> &packets);

  private:
    bool initCUDA();
    bool initEncoder();

  private:
    std::unique_ptr<NvEncoderCuda> encoder;
    VoxelCompressOptions compress_opts;

    CUcontext cu_ctx = nullptr;
};

VoxelCompressImpl::VoxelCompressImpl(const VoxelCompressOptions &opts) : compress_opts(opts)
{
    if (!initCUDA())
    {
        throw std::runtime_error("VoxelCompress CUDA context init failed!");
    }
    if (!initEncoder())
    {
        throw std::runtime_error("VoxelCompress init encoder failed!");
    }
}

bool VoxelCompressImpl::compress(uint8_t *src_ptr, int64_t len, std::vector<std::vector<uint8_t>> &packets)
{
    if (src_ptr == nullptr || len <= 0)
        return false;
    //init encode for every compress
    initEncoder();

    int64_t frame_size = encoder->GetFrameSize(); // base on pixel format
    int frame_num = 0;
    int64_t cur_frame_offset = 0;
    int64_t compressed_size = 0;
//    int encode_turn = 0;

    while (true)
    {
        std::vector<std::vector<uint8_t>> tmp_packets;
        if (cur_frame_offset + frame_size <= len)
        {
            const NvEncInputFrame *encoderInputFrame = encoder->GetNextInputFrame();
            NvEncoderCuda::CopyToDeviceFrame(cu_ctx, src_ptr + cur_frame_offset, 0,
                                             (CUdeviceptr)encoderInputFrame->inputPtr, (int)encoderInputFrame->pitch,
                                             encoder->GetEncodeWidth(), encoder->GetEncodeHeight(), CU_MEMORYTYPE_HOST,
                                             encoderInputFrame->bufferFormat, encoderInputFrame->chromaOffsets,
                                             encoderInputFrame->numChromaPlanes);

            encoder->EncodeFrame(tmp_packets);
        }
        else
        {
            encoder->EndEncode(tmp_packets);
        }
        frame_num += tmp_packets.size();

        cur_frame_offset += frame_size;
        for (auto & tmp_packet : tmp_packets)
        {
            compressed_size += tmp_packet.size();
            packets.emplace_back(std::move(tmp_packet));
        }
        if (cur_frame_offset > len)
            break;
    }
    std::cout << "frame num is: " << frame_num << std::endl;
    std::cout << "compressed size is: " << compressed_size << std::endl;
    encoder.reset();
    return true;
}

bool VoxelCompressImpl::initCUDA()
{
    checkCUDAErrors(cuInit(0));
    CUdevice cu_device;
    int cu_device_cnt = 0;
    int using_gpu = compress_opts.device_id;
    char using_device_name[80];
    checkCUDAErrors(cuDeviceGetCount(&cu_device_cnt));
    checkCUDAErrors(cuDeviceGet(&cu_device, using_gpu));
    checkCUDAErrors(cuDeviceGetName(using_device_name, sizeof(using_device_name), cu_device));
    std::cout << "GPU in use: " << using_device_name << std::endl;
    cu_ctx = nullptr;
    checkCUDAErrors(cuCtxCreate(&cu_ctx, 0, cu_device));
    return cu_ctx;
}

bool VoxelCompressImpl::initEncoder()
{
    encoder = std::make_unique<NvEncoderCuda>(cu_ctx, compress_opts.width, compress_opts.height,
                                              compress_opts.input_buffer_format);

    NV_ENC_INITIALIZE_PARAMS initializeParams = {NV_ENC_INITIALIZE_PARAMS_VER};
    NV_ENC_CONFIG encodeConfig = {NV_ENC_CONFIG_VER};

    initializeParams.encodeConfig = &encodeConfig;
    auto &encode_opts = compress_opts.encode_init_params;
    encoder->CreateDefaultEncoderParams(&initializeParams, encode_opts.GetEncodeGUID(), encode_opts.GetPresetGUID(),
                                        encode_opts.GetTuningInfo());
    encode_opts.SetInitParams(&initializeParams, compress_opts.input_buffer_format);

    encoder->CreateEncoder(&initializeParams);

    return encoder.get();
}

VoxelCompress::VoxelCompress(const VoxelCompressOptions &opts)
{
    impl = std::make_unique<VoxelCompressImpl>(opts);
}

bool VoxelCompress::compress(uint8_t *src_ptr, int64_t len, std::vector<std::vector<uint8_t>> &packets)
{
    return impl->compress(src_ptr, len, packets);
}

VoxelCompress::~VoxelCompress()
{
}
