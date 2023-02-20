//
// Created by wyz on 2021/2/7.
//

#pragma once
#include <cuda.h>
#include <memory>
#include <nvEncodeAPI.h>
#include <nvcuvid.h>
#include <vector>

struct VoxelUncompressOptions
{
    [[deprecated]] uint32_t width = 0, height = 0;                                        // no-useful
    [[deprecated]] NV_ENC_BUFFER_FORMAT output_buffer_format = NV_ENC_BUFFER_FORMAT_NV12; // not necessary
    bool use_device_frame_buffer = true; // if first decoded data are store in device
    cudaVideoCodec codec_method = cudaVideoCodec_H264;//default codec method is h264
    CUcontext cu_ctx = nullptr;
    int device_id = 0;
};

class VoxelUncompressImpl;
/**
 * @brief Uncompress raw h264 bytes into volume data.
 */
class VoxelUncompress
{
  public:
    VoxelUncompress(const VoxelUncompressOptions &opts);
    ~VoxelUncompress();
    /**
     * @brief Decoding packets into host or device buffer.
     * @param dest_ptr valid host or device pointer
     * @param len buffer size in bytes
     * @param packets host buffer for encoded data
     */
    bool uncompress(uint8_t *dest_ptr, int64_t len, std::vector<std::vector<uint8_t>> &packets);

  private:
    std::unique_ptr<VoxelUncompressImpl> impl;
};
