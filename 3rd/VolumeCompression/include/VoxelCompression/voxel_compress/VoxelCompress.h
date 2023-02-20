//
// Created by wyz on 2021/2/5.
//

#pragma once
#include <VoxelCompression/utils/NvEncoderCLIOptions.h>
#include <cstdint>
#include <memory>
#include <nvEncodeAPI.h>
#include <string>
#include <vector>

struct VoxelCompressOptions
{
    NV_ENC_BUFFER_FORMAT input_buffer_format = NV_ENC_BUFFER_FORMAT_NV12;//best default setting
    // default encoder params setting include codec method(h264), these can be changed or just leave it alone.
    NvEncoderInitParam encode_init_params;
    uint32_t width = 0;  // input encoding frame width
    uint32_t height = 0; // input encoding frame height
    int device_id = 0;   // gpu index
};

class VoxelCompressImpl;
/**
 * @brief Compress a volume data into raw h264 bytes.
 * @note Most 3 instances of VoxelCompress can be created in the same time for RTX3090.
 */
class VoxelCompress
{
  public:
    explicit VoxelCompress(const VoxelCompressOptions &opts);
    ~VoxelCompress();
    /**
     * @brief Encoding stream which consists of multi-frames into packets.
     * For volume data compress, stream is the entire volume data buffer and len is the voxel_num x voxel_size.
     * @param src_ptr host pointer for input stream
     * @param len sizeof input stream buffer
     * @param packets empty encoding result container
     */
    bool compress(uint8_t *src_ptr, int64_t len, std::vector<std::vector<uint8_t>> &packets);

  private:
    std::unique_ptr<VoxelCompressImpl> impl;
};
