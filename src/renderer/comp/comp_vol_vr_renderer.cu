#include "comp_vol_vr_renderer.h"

#include <cassert>
#include <chrono>

#include <VolumeSlicer/helper.hpp>

#include <device_launch_parameters.h>

using namespace kouek;

extern __constant__ cudaTextureObject_t dc_preIntTF;
extern __constant__ VRRenderer::LightParam dc_lightParam;
extern __constant__ VRRenderer::RenderParam dc_renderParam;
extern __constant__ VRRenderer::CameraParam dc_cameraParam;
extern __constant__ VRRenderer::ProjectionParam dc_projectionParam;

extern size_t outputSize;
extern glm::u8vec4 *d_output2[2];
extern cudaGraphicsResource_t outputTexRsrc2[2];

extern cudaArray_t d_subsampleTexArr2[2];
extern __constant__ cudaTextureObject_t dc_subsampleTex2[2];
extern __constant__ cudaTextureObject_t
    dc_subsampleLookupTexes[VRRenderer::MAX_SUBSAMPLE_LEVEL];
extern __constant__ cudaTextureObject_t
    dc_reconsLookupTexes[VRRenderer::MAX_SUBSAMPLE_LEVEL];

static cudaArray_t d_outputTexArr2[2] = {nullptr, nullptr};

__constant__ CompVolVRRenderer::RendererParam dc_rendererParam;
__constant__ CompVolVRRenderer::VolumeParam dc_volumeParam;

__constant__ uint32_t dc_blockOffsets[CompVolVRRenderer::MAX_LOD + 1];
__constant__ cudaTextureObject_t
    dc_textures[CompVolVRRenderer::MAX_TEX_UNIT_NUM];

static uint32_t *d_mappingTable = nullptr;
__constant__ size_t dc_mappingTableSize = 0;
__constant__ glm::uvec4 *dc_mappingTableStride4 = nullptr;

static cudaStream_t stream = nullptr;

void uploadBlockOffset(const uint32_t *dat, size_t num) {
    assert(num <= CompVolVRRenderer::MAX_LOD + 1);
    CUDA_RUNTIME_API_CALL(
        cudaMemcpyToSymbol(dc_blockOffsets, dat, sizeof(uint32_t) * num));
}

void uploadTextureObject(const cudaTextureObject_t *tex, size_t num) {
    assert(num <= CompVolVRRenderer::MAX_TEX_UNIT_NUM);
    CUDA_RUNTIME_API_CALL(cudaMemcpyToSymbol(
        dc_textures, tex, sizeof(cudaTextureObject_t) * num));
}

void uploadMappingTable(const uint32_t *dat, size_t size) {
    if (d_mappingTable == nullptr) {
        cudaMalloc(&d_mappingTable, size);
        // cpy uint32_t ptr to uint4 ptr
        CUDA_RUNTIME_API_CALL(cudaMemcpyToSymbol(
            dc_mappingTableStride4, &d_mappingTable, sizeof(glm::uvec4 *)));
    }
    CUDA_RUNTIME_API_CALL(
        cudaMemcpyToSymbol(dc_mappingTableSize, &size, sizeof(size_t)));
    CUDA_RUNTIME_API_CALL(
        cudaMemcpy(d_mappingTable, dat, size, cudaMemcpyHostToDevice));
}

CompVolVRRenderer::CompVolVRRenderer(const RendererParam &param)
    : rendererParam(param) {
    CUDA_RUNTIME_API_CALL(
        cudaMemcpyToSymbol(dc_rendererParam, &rendererParam,
                           sizeof(CompVolVRRenderer::RendererParam)));
}

void CompVolVRRenderer::SetCameraParam(const CameraParam &param) {
    VRRenderer::SetCameraParam(param);

    static auto start = std::chrono::system_clock::now();
    auto now = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start) >=
        static_cast<std::chrono::milliseconds>(1000)) {
        start = now;
    } else
        return; // load block every 2 secs

    loadBlocks.clear();
    unloadBlocks.clear();
    {
        auto cntrPos = .5f * (param.pos2[0] + param.pos2[1]) -
                       (param.OBBHfSz.z + param.OBBBorderDistToEyeCntr) *
                           param.rotation[2];
        // according to CameraParam, find blocks needed
        vs::OBB obb(cntrPos, param.rotation[0], param.rotation[1],
                    param.rotation[2], param.OBBHfSz.x, param.OBBHfSz.y,
                    param.OBBHfSz.z);
        // AABB filter first
        vs::AABB abb = obb.getAABB();
        for (auto &blockAABB : blockAABBs)
            if (abb.intersect(blockAABB.second))
                currNeedBlocks.emplace(
                    std::array{blockAABB.first[0], blockAABB.first[1],
                               blockAABB.first[2], (uint32_t)0});
        // OBB filter then
        for (auto itr = currNeedBlocks.begin(); itr != currNeedBlocks.end();)
            if (!obb.intersect_obb(
                    blockAABBs[std::array{(*itr)[0], (*itr)[1], (*itr)[2]}]
                        .convertToOBB()))
                currNeedBlocks.erase(itr++);
            else
                ++itr;
    }
    // loadBlocks = currNeedBlocks - (old)needBlocks
    for (auto &e : currNeedBlocks)
        if (needBlocks.find(e) == needBlocks.end())
            loadBlocks.insert(e);
    // unloadBlocks = (old)needBlocks - currNeedBlocks
    for (auto &e : needBlocks)
        if (currNeedBlocks.find(e) == currNeedBlocks.end())
            unloadBlocks.insert(e);
    needBlocks = std::move(currNeedBlocks);
    if (loadBlocks.size() > 0 || unloadBlocks.size() > 0) {
        // loadBlocks = loadBlocks - cachedBlocks
        decltype(loadBlocks) tmp;
        for (auto &e : loadBlocks) {
            bool cached = blockCache->SetCachedBlockValid(e);
            if (!cached)
                tmp.insert(e);
        }
        loadBlocks = std::move(tmp);

        for (auto &e : unloadBlocks)
            blockCache->SetBlockInvalid(e);

        volume.get()->PauseLoadBlock();

        if (!needBlocks.empty()) {
            std::vector<std::array<uint32_t, 4>> targets;
            targets.reserve(needBlocks.size());
            for (auto &e : needBlocks)
                targets.push_back(e);
            volume.get()->ClearBlockInQueue(targets);
        }
        for (auto &e : loadBlocks)
            volume.get()->SetRequestBlock(e);
        for (auto &e : unloadBlocks)
            volume.get()->EraseBlockInRequest(e);

        volume.get()->StartLoadBlock();
    }
}

void CompVolVRRenderer::Render(RenderTarget renderTarget) {
    for (auto &e : needBlocks) {
        auto volumeBlock = volume.get()->GetBlock(e);
        if (volumeBlock.valid) {
            blockCache->UploadVolumeBlock(e,
                                          volumeBlock.block_data->GetDataPtr(),
                                          volumeBlock.block_data->GetSize());
            volumeBlock.Release();
        }
    }
    auto &mappingTable = blockCache->GetMappingTable();
    uploadMappingTable(mappingTable.data(),
                       sizeof(uint32_t) * mappingTable.size());

    if (stream == nullptr)
        CUDA_RUNTIME_API_CALL(cudaStreamCreate(&stream));
    render(renderTarget);
}

void CompVolVRRenderer::SetVolume(std::shared_ptr<vs::CompVolume> volume,
                                  const glm::vec3 &spaces) {
    this->volume = volume;
    VolumeParam volumeParam;
    {
        auto blockDimInfo = volume->GetBlockDim();
        auto blockLenInfo = volume->GetBlockLength();
        volumeParam.blockLength = blockLenInfo[0];
        volumeParam.LOD0BlockDim = {blockDimInfo[0][0], blockDimInfo[0][1],
                                    blockDimInfo[0][2]};
        volumeParam.noPaddingBlockLength =
            blockLenInfo[0] - 2 * blockLenInfo[1];
        volumeParam.padding = blockLenInfo[1];
        volumeParam.spaces = spaces;

        CUDA_RUNTIME_API_CALL(cudaMemcpyToSymbol(dc_volumeParam, &volumeParam,
                                                 sizeof(VolumeParam)));
    }

    blockCache = vs::CUDAVolumeBlockCache::Create();
    blockCache->SetCacheBlockLength(volume->GetBlockLength()[0]);
    blockCache->SetCacheCapacity(
        rendererParam.texUnitNum, rendererParam.texUnitShape.x,
        rendererParam.texUnitShape.y, rendererParam.texUnitShape.z);
    blockCache->CreateMappingTable(volume->GetBlockDim());
    {
        // map lod to flat({ lod,0,0,0 }),
        // which is the first Voxel idx of LOD lod
        auto &lodMappingTableOffsets = blockCache->GetLodMappingTableOffset();
        uint32_t maxLOD = 0, minLOD = std::numeric_limits<uint32_t>::max();
        for (auto &e : lodMappingTableOffsets) {
            if (e.first < minLOD)
                minLOD = e.first;
            if (e.first > maxLOD)
                maxLOD = e.first;
        }
        maxLOD--; // in lodMappingTableOffsets, Key ranges [0, MAX_LOD + 1]

        // map lod(idx of vector) to flat({ lod,0,0,0 }) / 4,
        // which is the first Block idx of LOD lod
        std::vector<uint32_t> blockOffsets((size_t)maxLOD + 1, 0);
        for (auto &e : lodMappingTableOffsets)
            // in lodMappingTableOffsets, Key ranges [0, MAX_LOD + 1],
            // while in blockOffsets, Key ranges [0, MAX_LOD]
            if (e.first <= maxLOD)
                blockOffsets[e.first] = e.second / 4;

        // upload to CUDA
        uploadBlockOffset(blockOffsets.data(), blockOffsets.size());
    }
    {
        auto &texObj = blockCache->GetCUDATextureObjects();
        uploadTextureObject(texObj.data(), texObj.size());
    }

    blockAABBs.clear(); // avoid conflict caused by reset
    for (uint32_t z = 0; z < volumeParam.LOD0BlockDim.z; ++z)
        for (uint32_t y = 0; y < volumeParam.LOD0BlockDim.y; ++y)
            for (uint32_t x = 0; x < volumeParam.LOD0BlockDim.x; ++x)
                blockAABBs.emplace(
                    std::piecewise_construct,
                    std::forward_as_tuple(std::array{x, y, z}),
                    std::forward_as_tuple(
                        glm::vec3{
                            x * volumeParam.noPaddingBlockLength * spaces.x,
                            y * volumeParam.noPaddingBlockLength * spaces.y,
                            z * volumeParam.noPaddingBlockLength * spaces.z},
                        glm::vec3{(x + 1) * volumeParam.noPaddingBlockLength *
                                      spaces.x,
                                  (y + 1) * volumeParam.noPaddingBlockLength *
                                      spaces.y,
                                  (z + 1) * volumeParam.noPaddingBlockLength *
                                      spaces.z},
                        // dummy in this program
                        std::array<uint32_t, 4>()));
}

__device__ float virtualSampleLOD0(const glm::vec3 &samplePos) {
    // sample pos in Voxel Space -> virtual sample Block idx
    glm::uvec3 vsBlockIdx =
        samplePos / (float)dc_volumeParam.noPaddingBlockLength;

    // virtual sample Block idx -> real sample Block idx (in GPU Mem)
    glm::uvec4 GPUMemBlockIdx;
    {
        size_t flatVSBlockIdx = dc_blockOffsets[0] +
                                vsBlockIdx.z * dc_volumeParam.LOD0BlockDim.y *
                                    dc_volumeParam.LOD0BlockDim.x +
                                vsBlockIdx.y * dc_volumeParam.LOD0BlockDim.x +
                                vsBlockIdx.x;
        if (flatVSBlockIdx >= dc_mappingTableSize)
            return 0;
        GPUMemBlockIdx = dc_mappingTableStride4[flatVSBlockIdx];
    }
    if (((GPUMemBlockIdx.w >> 16) & (0x0000ffff)) != 1)
        // not a valid GPU Mem block
        return 0;

    // sample pos in Voxel Space -> real sample pos (in GPU Mem)
    glm::vec3 GPUMemSamplePos;
    {
        glm::vec3 offsetInNoPaddingBlock =
            samplePos -
            glm::vec3{vsBlockIdx * dc_volumeParam.noPaddingBlockLength};
        GPUMemSamplePos =
            glm::vec3{GPUMemBlockIdx.x, GPUMemBlockIdx.y, GPUMemBlockIdx.z} *
                (float)dc_volumeParam.blockLength +
            offsetInNoPaddingBlock + (float)dc_volumeParam.padding;
        // normolized
        GPUMemSamplePos /= dc_rendererParam.texUnitShape;
    }

    return tex3D<float>(dc_textures[GPUMemBlockIdx.w & 0x0000ffff],
                        GPUMemSamplePos.x, GPUMemSamplePos.y,
                        GPUMemSamplePos.z);
}

__device__ glm::vec4 phongShadingLOD0(const glm::vec3 &rayDrc,
                                      const glm::vec3 &samplePos,
                                      const glm::vec4 &diffuseColor) {
    glm::vec3 N;
    {
        float val1, val2;
        val1 = virtualSampleLOD0(samplePos + glm::vec3{1.f, 0, 0});
        val2 = virtualSampleLOD0(samplePos - glm::vec3{1.f, 0, 0});
        N.x = val2 - val1;
        val1 = virtualSampleLOD0(samplePos + glm::vec3{0, 1.f, 0});
        val2 = virtualSampleLOD0(samplePos - glm::vec3{0, 1.f, 0});
        N.y = val2 - val1;
        val1 = virtualSampleLOD0(samplePos + glm::vec3{0, 0, 1.f});
        val2 = virtualSampleLOD0(samplePos - glm::vec3{0, 0, 1.f});
        N.z = val2 - val1;
    }
    N = glm::normalize(N);

    glm::vec3 L = {-rayDrc.x, -rayDrc.y, -rayDrc.z};
    glm::vec3 R = L;
    if (glm::dot(N, L) < 0)
        N = -N;

    glm::vec3 ambient = dc_lightParam.ka * diffuseColor;
    glm::vec3 specular =
        glm::vec3(dc_lightParam.ks * powf(fmaxf(dot(N, .5f * (L + R)), 0),
                                          dc_lightParam.shininess));
    glm::vec3 diffuse = dc_lightParam.kd * fmaxf(dot(N, L), 0.f) * diffuseColor;

    return glm::vec4{ambient + specular + diffuse, diffuseColor.a};
}

__device__ void rayIntersectAABB(float *tEnter, float *tExit,
                                 const glm::vec3 &rayOri,
                                 const glm::vec3 &rayDrc, const glm::vec3 &bot,
                                 const glm::vec3 &top) {
    // For  Ori + Drc * t3Bot.x = <Bot.x, 0, 0>
    // Thus t3Bot.x = Bot.x / Drc.x
    // Thus t3Bot.y = Bot.x / Drc.y
    // If  \
	//  \_\|\ 
	//   \_\|
    //      \.t3Bot.x
    //      |\
	//    __|_\.___|
    //      |  \t3Bot.y
    //    __|___\._|_
    //    t3Top.y\ |
    //      |     \.t3Top.x
    //
    // Then t3Min = t3Bot, t3Max = t3Top
    // And  the max of t3Min is tEnter
    // And  the min of t3Max is tExit

    glm::vec3 invRay = 1.f / rayDrc;
    glm::vec3 t3Bot = invRay * (bot - rayOri);
    glm::vec3 t3Top = invRay * (top - rayOri);
    glm::vec3 t3Min{fminf(t3Bot.x, t3Top.x), fminf(t3Bot.y, t3Top.y),
                    fminf(t3Bot.z, t3Top.z)};
    glm::vec3 t3Max{fmaxf(t3Bot.x, t3Top.x), fmaxf(t3Bot.y, t3Top.y),
                    fmaxf(t3Bot.z, t3Top.z)};
    *tEnter = fmaxf(fmaxf(t3Min.x, t3Min.y), fmaxf(t3Min.x, t3Min.z));
    *tExit = fminf(fminf(t3Max.x, t3Max.y), fminf(t3Max.x, t3Max.z));
}

__device__ glm::u8vec4 rgbaFloatToUbyte4(glm::vec4 color) {
    color.r = __saturatef(color.r); // clamp to [0.0, 1.0]
    color.g = __saturatef(color.g);
    color.b = __saturatef(color.b);
    color.a = __saturatef(color.a);
    color.r *= 255.f;
    color.g *= 255.f;
    color.b *= 255.f;
    color.a *= 255.f;
    return glm::u8vec4(color.r, color.g, color.b, color.a);
}

__global__ void renderKernel(glm::u8vec4 *d_outputLft,
                             glm::u8vec4 *d_outputRht) {
    glm::uvec2 renderXY{blockIdx.x * blockDim.x + threadIdx.x,
                        blockIdx.y * blockDim.y + threadIdx.y};
    if (renderXY.x >= dc_renderParam.renderSz.x ||
        renderXY.y >= dc_renderParam.renderSz.y)
        return;
    size_t flatIdx =
        (size_t)renderXY.y * dc_renderParam.renderSz.x + renderXY.x;

    glm::vec3 rayDrc;
    float t, tExit;
    {
        auto v4 = dc_projectionParam.unProjection2[blockIdx.z] *
                  glm::vec4{2.f * renderXY.x / dc_renderParam.renderSz.x - 1.f,
                            2.f * renderXY.y / dc_renderParam.renderSz.y - 1.f,
                            1.f, 1.f};
        rayDrc = v4;
        rayDrc = glm::normalize(rayDrc);

        glm::vec3 obbCntr{0, 0,
                          -dc_cameraParam.OBBHfSz.z -
                              dc_cameraParam.OBBBorderDistToEyeCntr};
        rayIntersectAABB(&t, &tExit, glm::zero<glm::vec3>(), rayDrc,
                         obbCntr - dc_cameraParam.OBBHfSz,
                         obbCntr + dc_cameraParam.OBBHfSz);
        if (t < 0)
            t = 0;
        if (tExit < 0)
            tExit = 0;
        rayDrc = dc_cameraParam.rotation * rayDrc;

#ifdef TEST_RAY_DRC
        if (blockIdx.z == 0)
            d_outputLft[flatIdx] = rgbaFloatToUbyte4(glm::vec4{rayDrc, 1.f});
        else
            d_outputRht[flatIdx] = rgbaFloatToUbyte4(glm::vec4{rayDrc, 1.f});
        return;
#endif
    }

    auto rayPos = dc_cameraParam.pos2[blockIdx.z] + t * rayDrc;
    auto rayDrcMultStep = dc_renderParam.step * rayDrc;
    auto lastSampleVal = 0.f;
    glm::vec4 color{0};

    decltype(dc_renderParam.maxStepCnt) stepCnt = 0;
    while (true) {
        if (t >= tExit || stepCnt >= dc_renderParam.maxStepCnt)
            break;

        auto samplePos = rayPos / dc_volumeParam.spaces;
        auto currSampleVal = virtualSampleLOD0(samplePos);
        if (currSampleVal > 0) {
            float4 currColor =
                tex2D<float4>(dc_preIntTF, lastSampleVal, currSampleVal);
            lastSampleVal = currSampleVal;
            if (currColor.w > 0) {
                auto shadedColor = phongShadingLOD0(
                    rayDrc, samplePos,
                    {currColor.x, currColor.y, currColor.z, currColor.w});
                color = color + (1.f - color.a) * shadedColor *
                                    glm::vec4{shadedColor.a, shadedColor.a,
                                              shadedColor.a, 1.f};
            }
        }
        if (color.a > .9f)
            break;

        ++stepCnt;
        t += dc_renderParam.step;
        rayPos += rayDrcMultStep;
    }

    constexpr float GAMMA_CORRECT_COEF = 1.f / 2.2f;
    color.r = powf(color.r, GAMMA_CORRECT_COEF);
    color.g = powf(color.g, GAMMA_CORRECT_COEF);
    color.b = powf(color.b, GAMMA_CORRECT_COEF);

    if (blockIdx.z == 0)
        d_outputLft[flatIdx] = rgbaFloatToUbyte4(color);
    else
        d_outputRht[flatIdx] = rgbaFloatToUbyte4(color);
}

__global__ void subsampleKernel(glm::u8vec4 *d_outputLft,
                                glm::u8vec4 *d_outputRht, uint32_t subsampleW) {
    glm::uvec2 renderXY{blockIdx.x * blockDim.x + threadIdx.x,
                        blockIdx.y * blockDim.y + threadIdx.y};
    if (renderXY.x >= subsampleW || renderXY.y >= dc_renderParam.renderSz.y)
        return;
    size_t flatIdx =
        (size_t)renderXY.y * dc_renderParam.renderSz.x + renderXY.x;

    uint2 subsamplePos;
    {
        auto FAVRIdx = dc_renderParam.FAVRLvl - 1;
        subsamplePos = tex2D<uint2>(dc_subsampleLookupTexes[FAVRIdx],
                                    renderXY.x, renderXY.y);
        if (subsamplePos.x == VRRenderer::SUBSAMPLE_MAP_TO_EMPTY) {
            if (blockIdx.z == 0)
                d_outputLft[flatIdx] = rgbaFloatToUbyte4(glm::vec4{1.f});
            else
                d_outputRht[flatIdx] = rgbaFloatToUbyte4(glm::vec4{1.f});
            return;
        }
    }

    glm::vec3 rayDrc;
    float t, tExit;
    {
        auto v4 =
            dc_projectionParam.unProjection2[blockIdx.z] *
            glm::vec4{2.f * subsamplePos.x / dc_renderParam.renderSz.x - 1.f,
                      2.f * subsamplePos.y / dc_renderParam.renderSz.y - 1.f,
                      1.f, 1.f};
        rayDrc = v4;
        rayDrc = glm::normalize(rayDrc);

        glm::vec3 obbCntr{0, 0,
                          -dc_cameraParam.OBBHfSz.z -
                              dc_cameraParam.OBBBorderDistToEyeCntr};
        rayIntersectAABB(&t, &tExit, glm::zero<glm::vec3>(), rayDrc,
                         obbCntr - dc_cameraParam.OBBHfSz,
                         obbCntr + dc_cameraParam.OBBHfSz);
        if (t < 0)
            t = 0;
        if (tExit < 0)
            tExit = 0;
        rayDrc = dc_cameraParam.rotation * rayDrc;

#ifdef TEST_RAY_DRC
        if (blockIdx.z == 0)
            d_outputLft[flatIdx] = rgbaFloatToUbyte4(glm::vec4{rayDrc, 1.f});
        else
            d_outputRht[flatIdx] = rgbaFloatToUbyte4(glm::vec4{rayDrc, 1.f});
        return;
#endif
    }

    auto rayPos = dc_cameraParam.pos2[blockIdx.z] + t * rayDrc;
    auto rayDrcMultStep = dc_renderParam.step * rayDrc;
    auto lastSampleVal = 0.f;
    glm::vec4 color{0};

    decltype(dc_renderParam.maxStepCnt) stepCnt = 0;
    while (true) {
        if (t >= tExit || stepCnt >= dc_renderParam.maxStepCnt)
            break;

        auto samplePos = rayPos / dc_volumeParam.spaces;
        auto currSampleVal = virtualSampleLOD0(samplePos);
        if (currSampleVal > 0) {
            float4 currColor =
                tex2D<float4>(dc_preIntTF, lastSampleVal, currSampleVal);
            lastSampleVal = currSampleVal;
            if (currColor.w > 0) {
                auto shadedColor = phongShadingLOD0(
                    rayDrc, samplePos,
                    {currColor.x, currColor.y, currColor.z, currColor.w});
                color = color + (1.f - color.a) * shadedColor *
                                    glm::vec4{shadedColor.a, shadedColor.a,
                                              shadedColor.a, 1.f};
            }
        }
        if (color.a > .9f)
            break;

        ++stepCnt;
        t += dc_renderParam.step;
        rayPos += rayDrcMultStep;
    }

    constexpr float GAMMA_CORRECT_COEF = 1.f / 2.2f;
    color.r = powf(color.r, GAMMA_CORRECT_COEF);
    color.g = powf(color.g, GAMMA_CORRECT_COEF);
    color.b = powf(color.b, GAMMA_CORRECT_COEF);

    if (blockIdx.z == 0)
        d_outputLft[flatIdx] = rgbaFloatToUbyte4(color);
    else
        d_outputRht[flatIdx] = rgbaFloatToUbyte4(color);
}

__global__ void reconsKernel(glm::u8vec4 *d_outputLft,
                             glm::u8vec4 *d_outputRht) {
    glm::uvec2 renderXY{blockIdx.x * blockDim.x + threadIdx.x,
                        blockIdx.y * blockDim.y + threadIdx.y};
    if (renderXY.x >= dc_renderParam.renderSz.x ||
        renderXY.y >= dc_renderParam.renderSz.y)
        return;
    size_t flatIdx =
        (size_t)renderXY.y * dc_renderParam.renderSz.x + renderXY.x;

    auto subsampleTexPos =
        tex2D<uint2>(dc_reconsLookupTexes[dc_renderParam.FAVRLvl - 1],
                     renderXY.x, renderXY.y);
    auto subsampleColor = tex2D<uchar4>(dc_subsampleTex2[blockIdx.z],
                                        subsampleTexPos.x, subsampleTexPos.y);

    if (blockIdx.z == 0)
        d_outputLft[flatIdx] = {subsampleColor.x, subsampleColor.y,
                                subsampleColor.z, subsampleColor.w};
    else
        d_outputRht[flatIdx] = {subsampleColor.x, subsampleColor.y,
                                subsampleColor.z, subsampleColor.w};
}

__global__ void testSubsampleKernel(glm::u8vec4 *d_outputLft,
                                    glm::u8vec4 *d_outputRht,
                                    uint32_t FAVRWid) {
    glm::uvec2 renderXY{blockIdx.x * blockDim.x + threadIdx.x,
                        blockIdx.y * blockDim.y + threadIdx.y};
    auto FAVRIdx = dc_renderParam.FAVRLvl - 1;
    if (renderXY.x >= FAVRWid || renderXY.y >= dc_renderParam.renderSz.y)
        return;
    size_t flatIdx =
        (size_t)renderXY.y * dc_renderParam.renderSz.x + renderXY.x;

    auto subsamplePos =
        tex2D<uint2>(dc_subsampleLookupTexes[FAVRIdx], renderXY.x, renderXY.y);
    auto color =
        subsamplePos.x == VRRenderer::SUBSAMPLE_MAP_TO_EMPTY
            ? glm::vec4{1.f}
            : glm::vec4{(float)subsamplePos.x / dc_renderParam.renderSz.x,
                        (float)subsamplePos.y / dc_renderParam.renderSz.y, 0,
                        1.f};
    if (blockIdx.z == 0)
        d_outputLft[flatIdx] = rgbaFloatToUbyte4(color);
    else
        d_outputRht[flatIdx] = rgbaFloatToUbyte4(color);
}

__global__ void testReconsKernel(glm::u8vec4 *d_outputLft,
                                 glm::u8vec4 *d_outputRht) {
    glm::uvec2 renderXY{blockIdx.x * blockDim.x + threadIdx.x,
                        blockIdx.y * blockDim.y + threadIdx.y};
    auto FAVRIdx = dc_renderParam.FAVRLvl - 1;
    if (renderXY.x >= dc_renderParam.renderSz.x ||
        renderXY.y >= dc_renderParam.renderSz.y)
        return;
    size_t flatIdx =
        (size_t)renderXY.y * dc_renderParam.renderSz.x + renderXY.x;

    glm::vec4 color{1.f};
    auto subsampleTexPos =
        tex2D<uint2>(dc_reconsLookupTexes[FAVRIdx], renderXY.x, renderXY.y);
    auto subsamplePos = tex2D<uint2>(dc_subsampleLookupTexes[FAVRIdx],
                                     subsampleTexPos.x, subsampleTexPos.y);
    if (subsamplePos.x != VRRenderer::SUBSAMPLE_MAP_TO_EMPTY) {
        color.r = (float)subsamplePos.x / dc_renderParam.renderSz.x;
        color.g = (float)subsamplePos.y / dc_renderParam.renderSz.y;
        color.b = 0;
    }

    static constexpr auto MOD = 100;
    static constexpr auto HF_WID = 5;
    uint32_t mod[2]{subsamplePos.x % MOD, subsamplePos.y % MOD};
    if (mod[0] < HF_WID || mod[0] > MOD - HF_WID || mod[1] < HF_WID ||
        mod[1] > MOD - HF_WID)
        color = glm::zero<glm::vec4>();

    if (blockIdx.z == 0)
        d_outputLft[flatIdx] = rgbaFloatToUbyte4(color);
    else
        d_outputRht[flatIdx] = rgbaFloatToUbyte4(color);
}

void kouek::CompVolVRRenderer::render(RenderTarget renderTarget) {
    switch (renderTarget) {
    case RenderTarget::FullResVol: {
        for (uint8_t lr = 0; lr < 2; ++lr) {
            cudaGraphicsMapResources(1, &outputTexRsrc2[lr], stream);
            cudaGraphicsSubResourceGetMappedArray(&d_outputTexArr2[lr],
                                                  outputTexRsrc2[lr], 0, 0);
        }

        dim3 threadPerBlock{16, 16};
        dim3 blockPerGrid{
            (renderSz.x + threadPerBlock.x - 1) / threadPerBlock.x,
            (renderSz.y + threadPerBlock.y - 1) / threadPerBlock.y, 2};
        renderKernel<<<blockPerGrid, threadPerBlock, 0, stream>>>(d_output2[0],
                                                                  d_output2[1]);

        for (uint8_t lr = 0; lr < 2; ++lr) {
            cudaMemcpyToArray(d_outputTexArr2[lr], 0, 0, d_output2[lr],
                              outputSize, cudaMemcpyDeviceToDevice);
            cudaGraphicsUnmapResources(1, &outputTexRsrc2[lr], stream);
            d_outputTexArr2[lr] = nullptr;
        }
    } break;
    case RenderTarget::FAVRVol: {
        for (uint8_t lr = 0; lr < 2; ++lr) {
            cudaGraphicsMapResources(1, &outputTexRsrc2[lr], stream);
            cudaGraphicsSubResourceGetMappedArray(&d_outputTexArr2[lr],
                                                  outputTexRsrc2[lr], 0, 0);
        }

        dim3 threadPerBlock{16, 16};
        dim3 blockPerGrid{
            (FAVRSubsampleWidths[FAVRIdx] + threadPerBlock.x - 1) /
                threadPerBlock.x,
            (renderSz.y + threadPerBlock.y - 1) / threadPerBlock.y, 2};
        subsampleKernel<<<blockPerGrid, threadPerBlock, 0, stream>>>(
            d_output2[0], d_output2[1], FAVRSubsampleWidths[FAVRIdx]);
        for (uint8_t lr = 0; lr < 2; ++lr)
            cudaMemcpyToArray(d_subsampleTexArr2[lr], 0, 0, d_output2[lr],
                              outputSize, cudaMemcpyDeviceToDevice);

        blockPerGrid.x = (renderSz.x + threadPerBlock.x - 1) / threadPerBlock.x;
        reconsKernel<<<blockPerGrid, threadPerBlock, 0, stream>>>(d_output2[0],
                                                                  d_output2[1]);

        for (uint8_t lr = 0; lr < 2; ++lr) {
            cudaMemcpyToArray(d_outputTexArr2[lr], 0, 0, d_output2[lr],
                              outputSize, cudaMemcpyDeviceToDevice);
            cudaGraphicsUnmapResources(1, &outputTexRsrc2[lr], stream);
            d_outputTexArr2[lr] = nullptr;
        }
    } break;
    case RenderTarget::FAVRSubsampleCoord: {
        for (uint8_t lr = 0; lr < 2; ++lr) {
            cudaGraphicsMapResources(1, &outputTexRsrc2[lr], stream);
            cudaGraphicsSubResourceGetMappedArray(&d_outputTexArr2[lr],
                                                  outputTexRsrc2[lr], 0, 0);
        }

        dim3 threadPerBlock{16, 16};
        dim3 blockPerGrid{
            (FAVRSubsampleWidths[FAVRIdx] + threadPerBlock.x - 1) /
                threadPerBlock.x,
            (renderSz.y + threadPerBlock.y - 1) / threadPerBlock.y, 2};
        cudaMemset(d_output2[0], 0,
                   sizeof(glm::u8vec4) * renderSz.x * renderSz.y);
        cudaMemset(d_output2[1], 0,
                   sizeof(glm::u8vec4) * renderSz.x * renderSz.y);
        testSubsampleKernel<<<blockPerGrid, threadPerBlock, 0, stream>>>(
            d_output2[0], d_output2[1], FAVRSubsampleWidths[FAVRIdx]);

        for (uint8_t lr = 0; lr < 2; ++lr) {
            cudaMemcpyToArray(d_outputTexArr2[lr], 0, 0, d_output2[lr],
                              outputSize, cudaMemcpyDeviceToDevice);
            cudaGraphicsUnmapResources(1, &outputTexRsrc2[lr], stream);
            d_outputTexArr2[lr] = nullptr;
        }
    } break;
    case RenderTarget::FAVRReconsCoord: {
        for (uint8_t lr = 0; lr < 2; ++lr) {
            cudaGraphicsMapResources(1, &outputTexRsrc2[lr], stream);
            cudaGraphicsSubResourceGetMappedArray(&d_outputTexArr2[lr],
                                                  outputTexRsrc2[lr], 0, 0);
        }

        dim3 threadPerBlock{16, 16};
        dim3 blockPerGrid{
            (renderSz.x + threadPerBlock.x - 1) / threadPerBlock.x,
            (renderSz.y + threadPerBlock.y - 1) / threadPerBlock.y, 2};
        cudaMemset(d_output2[0], 0,
                   sizeof(glm::u8vec4) * renderSz.x * renderSz.y);
        cudaMemset(d_output2[1], 0,
                   sizeof(glm::u8vec4) * renderSz.x * renderSz.y);
        testReconsKernel<<<blockPerGrid, threadPerBlock, 0, stream>>>(
            d_output2[0], d_output2[1]);

        for (uint8_t lr = 0; lr < 2; ++lr) {
            cudaMemcpyToArray(d_outputTexArr2[lr], 0, 0, d_output2[lr],
                              outputSize, cudaMemcpyDeviceToDevice);
            cudaGraphicsUnmapResources(1, &outputTexRsrc2[lr], stream);
            d_outputTexArr2[lr] = nullptr;
        }
    } break;
    default:
        break;
    }
}
