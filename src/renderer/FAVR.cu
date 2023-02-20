#include "vr_renderer.h"

#include <VolumeSlicer/helper.hpp>

#include <device_launch_parameters.h>

using namespace kouek;

cudaArray_t d_subsampleSurfArr2[2];
cudaSurfaceObject_t d_subsampleSurf2[2];
__constant__ cudaSurfaceObject_t dc_subsampleSurf2[2];

cudaArray_t d_subsampleLookupTexArrs[VRRenderer::MAX_SUBSAMPLE_LEVEL];
cudaTextureObject_t d_subsampleLookupTexes[VRRenderer::MAX_SUBSAMPLE_LEVEL];
__constant__ cudaTextureObject_t
    dc_subsampleLookupTexes[VRRenderer::MAX_SUBSAMPLE_LEVEL];

cudaArray_t d_reconsLookupTexArrs[VRRenderer::MAX_SUBSAMPLE_LEVEL];
cudaTextureObject_t d_reconsLookupTexes[VRRenderer::MAX_SUBSAMPLE_LEVEL];
__constant__ cudaTextureObject_t
    dc_reconsLookupTexes[VRRenderer::MAX_SUBSAMPLE_LEVEL];

__global__ void createSubsampleTexKernel(uint2 *d_output, uint8_t FAVRLvl,
                                         uint32_t w, uint32_t h) {
    glm::uvec2 renderXY{blockIdx.x * blockDim.x + threadIdx.x,
                        blockIdx.y * blockDim.y + threadIdx.y};
    if (renderXY.x >= w || renderXY.y >= h)
        return;
    size_t flatIdx = (size_t)renderXY.y * w + renderXY.x;

    uint32_t startY = 0, endY = w;
    float hfW = .5f * w;
    float hfWSqr = hfW * hfW;
    float fuleyeIdxesHfSz = .5f * h;
    glm::vec2 cntr{w * .5f};
    glm::vec2 pos{renderXY.x + .5f, renderXY.y + .5f};
    for (uint8_t stage = 0; stage < FAVRLvl; ++stage) {
        if (renderXY.y >= endY) {
            startY = endY;
            endY += w;
            cntr.y += w;
            continue;
        }

        auto scale = 1.f - (float)stage / FAVRLvl;
        auto invScale = 1.f / scale;
        auto scaleSqr = (1.f - scale) * (1.f - scale);

        auto d = pos - cntr;
        auto sqr = glm::dot(d, d);
        auto lowerSqr = stage == 0 ? 0 : hfWSqr * scaleSqr;
        if (sqr < lowerSqr || sqr > (stage == FAVRLvl - 1 ? INFINITY : hfWSqr))
            d_output[flatIdx] = {VRRenderer::SUBSAMPLE_MAP_TO_EMPTY,
                                 VRRenderer::SUBSAMPLE_MAP_TO_EMPTY};
        else {
            auto d0 = glm::normalize(d);
            d = invScale * (d - hfW * (1.f - scale) * d0);
            d0 = hfW * stage * d0;
            glm::vec2 targetPx = {floorf(fuleyeIdxesHfSz + d0.x + d.x),
                                  floorf(fuleyeIdxesHfSz + d0.y + d.y)};
            targetPx = glm::clamp(targetPx, glm::vec2{0}, glm::vec2{h});
            d_output[flatIdx] = {(uint32_t)targetPx.x, (uint32_t)targetPx.y};
        }
        break;
    }
}

__global__ void createReconsTexKernel(uint2 *d_output, uint8_t FAVRLvl,
                                      uint32_t sz, uint32_t subsampleW) {
    glm::uvec2 renderXY{blockIdx.x * blockDim.x + threadIdx.x,
                        blockIdx.y * blockDim.y + threadIdx.y};
    if (renderXY.x >= sz || renderXY.y >= sz)
        return;
    size_t flatIdx = (size_t)renderXY.y * sz + renderXY.x;

    glm::vec2 d{renderXY.x + .5f - sz * .5f, renderXY.y + .5f - sz * .5f};
    auto sqr = glm::dot(d, d);
    auto hfSubsampleW = subsampleW * .5f;
    glm::vec2 subsampleCntr{hfSubsampleW};
    auto rad = hfSubsampleW;
    auto radSqr = rad * rad;
    for (uint8_t stage = 0; stage < FAVRLvl; ++stage) {
        auto scale = 1.f - (float)stage / FAVRLvl;
        auto scaleSqr = scale * scale;
        if (stage != FAVRLvl - 1 && sqr >= radSqr) {
            subsampleCntr.y += subsampleW;
            rad += hfSubsampleW;
            radSqr = rad * rad;
            continue;
        }

        auto d0 = glm::normalize(d);
        d = scale * (d - hfSubsampleW * stage * d0);
        d0 = hfSubsampleW * (1.f - scale) * d0;
        d_output[flatIdx] = {(uint32_t)floorf(subsampleCntr.x + d0.x + d.x),
                             (uint32_t)floorf(subsampleCntr.y + d0.y + d.y)};
        break;
    }
}

static const uint8_t TEST_TIMES = 20;
__global__ void dismissGapKernel(uint2 *d_output, uint8_t FAVRLvl, uint32_t sz,
                                 uint32_t subsampleW) {
    glm::uvec2 renderXY{blockIdx.x * blockDim.x + threadIdx.x,
                        blockIdx.y * blockDim.y + threadIdx.y};
    if (renderXY.x >= sz || renderXY.y >= sz)
        return;
    size_t flatIdx = (size_t)renderXY.y * sz + renderXY.x;

    auto FAVRIdx = FAVRLvl - 1;
    {
        auto subsampleTexPos = d_output[flatIdx];
        auto subsamplePos = tex2D<uint2>(dc_subsampleLookupTexes[FAVRIdx],
                                         subsampleTexPos.x, subsampleTexPos.y);
        if (subsamplePos.x != VRRenderer::SUBSAMPLE_MAP_TO_EMPTY)
            return;
    }

    glm::vec2 d{renderXY.x + .5f - sz * .5f, renderXY.y + .5f - sz * .5f};
    d = glm::normalize(d);
    if (fabsf(d.x) > fabsf(d.y))
        d *= 1.f / fabsf(d.x);
    else
        d *= 1.f / fabsf(d.y);

    auto sqr = glm::dot(d, d);
    auto hfSubsampleW = subsampleW * .5f;
    auto rad = hfSubsampleW;
    auto radSqr = rad * rad;
    for (uint8_t stage = 0; stage < FAVRLvl; ++stage) {
        if (stage != FAVRLvl - 1 && sqr >= radSqr) {
            rad += hfSubsampleW;
            radSqr = rad * rad;
            continue;
        }
        glm::vec2 newRndrXY = {(float)renderXY.x + .5f - d.x,
                               (float)renderXY.y + .5f - d.y};
        bool found = false;
        for (uint8_t inStep = 0; inStep < TEST_TIMES; ++inStep) {
            size_t newFlatIdx =
                (size_t)floorf(newRndrXY.y) * sz + (size_t)floorf(newRndrXY.x);
            auto newSubsampleTexPos = d_output[newFlatIdx];
            auto subsamplePos =
                tex2D<uint2>(dc_subsampleLookupTexes[FAVRIdx],
                             newSubsampleTexPos.x, newSubsampleTexPos.y);
            if (subsamplePos.x != VRRenderer::SUBSAMPLE_MAP_TO_EMPTY) {
                d_output[flatIdx] = newSubsampleTexPos;
                found = true;
                break;
            }
            newRndrXY -= d;
        }
        if (found)
            break;
        newRndrXY = {(float)renderXY.x + .5f + d.x,
                     (float)renderXY.y + .5f + d.y};
        for (uint8_t ouStep = 0; ouStep < TEST_TIMES; ++ouStep) {
            size_t newFlatIdx =
                (size_t)floorf(newRndrXY.y) * sz + (size_t)floorf(newRndrXY.x);
            auto newSubsampleTexPos = d_output[newFlatIdx];
            auto subsamplePos =
                tex2D<uint2>(dc_subsampleLookupTexes[FAVRIdx],
                             newSubsampleTexPos.x, newSubsampleTexPos.y);
            if (subsamplePos.x != VRRenderer::SUBSAMPLE_MAP_TO_EMPTY) {
                d_output[flatIdx] = newSubsampleTexPos;
                break;
            }
            newRndrXY += d;
        }
        break;
    }
}

void VRRenderer::createSubsampleAndReconsTex() {
    assert(renderSz.x == renderSz.y);
    float hfSz = renderSz.y * .5f;

    cudaResourceDesc rsrcDesc;
    memset(&rsrcDesc, 0, sizeof(rsrcDesc));
    rsrcDesc.resType = cudaResourceTypeArray;

    for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx) {
        if (d_subsampleSurfArr2[eyeIdx] != nullptr) {
            CUDA_RUNTIME_API_CALL(
                cudaDestroySurfaceObject(d_subsampleSurf2[eyeIdx]));
            CUDA_RUNTIME_API_CALL(cudaFreeArray(d_subsampleSurfArr2[eyeIdx]));
        }
        cudaChannelFormatDesc chnnlDesc = cudaCreateChannelDesc<uchar4>();
        CUDA_RUNTIME_API_CALL(cudaMallocArray(
            &d_subsampleSurfArr2[eyeIdx], &chnnlDesc, renderSz.x, renderSz.y));

        rsrcDesc.res.array.array = d_subsampleSurfArr2[eyeIdx];
        CUDA_RUNTIME_API_CALL(
            cudaCreateSurfaceObject(&d_subsampleSurf2[eyeIdx], &rsrcDesc));
    }
    CUDA_RUNTIME_API_CALL(cudaMemcpyToSymbol(
        dc_subsampleSurf2, d_subsampleSurf2, sizeof(d_subsampleSurf2[0]) * 2));

    cudaTextureDesc texDesc;
    memset(&texDesc, 0, sizeof(texDesc));
    texDesc.addressMode[0] = texDesc.addressMode[1] = cudaAddressModeClamp;
    texDesc.filterMode = cudaFilterModePoint;
    texDesc.normalizedCoords = false;
    texDesc.readMode = cudaReadModeElementType;

    dim3 threadPerBlock{16, 16};
    for (uint8_t lvl = 1; lvl <= MAX_SUBSAMPLE_LEVEL; ++lvl) {
        auto idx = lvl - 1;
        auto subsampleHfW = hfSz / lvl;
        uint32_t w = FAVRSubsampleWidths[idx] = subsampleHfW * 2.f;

        if (d_subsampleLookupTexArrs[idx] != nullptr)
            CUDA_RUNTIME_API_CALL(cudaFreeArray(d_subsampleLookupTexArrs[idx]));
        cudaChannelFormatDesc chnnlDesc = cudaCreateChannelDesc<uint2>();
        CUDA_RUNTIME_API_CALL(cudaMallocArray(&d_subsampleLookupTexArrs[idx],
                                              &chnnlDesc, w, renderSz.y));

        rsrcDesc.res.array.array = d_subsampleLookupTexArrs[idx];
        if (d_subsampleLookupTexes[idx] != 0)
            CUDA_RUNTIME_API_CALL(
                cudaDestroyTextureObject(d_subsampleLookupTexes[idx]));
        CUDA_RUNTIME_API_CALL(cudaCreateTextureObject(
            &d_subsampleLookupTexes[idx], &rsrcDesc, &texDesc, nullptr));

        uint2 *d_tmp;
        auto cpySz = sizeof(uint2) * w * renderSz.y;
        CUDA_RUNTIME_API_CALL(cudaMalloc(&d_tmp, cpySz));

        dim3 blockPerGrid{
            (w + threadPerBlock.x - 1) / threadPerBlock.x,
            (renderSz.y + threadPerBlock.y - 1) / threadPerBlock.y, 2};
        createSubsampleTexKernel<<<blockPerGrid, threadPerBlock>>>(
            d_tmp, lvl, w, renderSz.y);

        CUDA_RUNTIME_API_CALL(cudaMemcpyToArray(d_subsampleLookupTexArrs[idx],
                                                0, 0, d_tmp, cpySz,
                                                cudaMemcpyDeviceToDevice));
        CUDA_RUNTIME_API_CALL(cudaFree(d_tmp));
    }
    CUDA_RUNTIME_API_CALL(
        cudaMemcpyToSymbol(dc_subsampleLookupTexes, d_subsampleLookupTexes,
                           sizeof(cudaTextureObject_t) * MAX_SUBSAMPLE_LEVEL));

    uint2 *d_tmp;
    auto cpySz = sizeof(uint2) * renderSz.x * renderSz.y;
    CUDA_RUNTIME_API_CALL(cudaMalloc(&d_tmp, cpySz));
    for (uint8_t lvl = 1; lvl <= MAX_SUBSAMPLE_LEVEL; ++lvl) {
        auto idx = lvl - 1;

        if (d_reconsLookupTexArrs[idx] != nullptr)
            CUDA_RUNTIME_API_CALL(cudaFreeArray(d_reconsLookupTexArrs[idx]));
        cudaChannelFormatDesc chnnlDesc = cudaCreateChannelDesc<uint2>();
        CUDA_RUNTIME_API_CALL(cudaMallocArray(
            &d_reconsLookupTexArrs[idx], &chnnlDesc, renderSz.x, renderSz.y));

        rsrcDesc.res.array.array = d_reconsLookupTexArrs[idx];
        if (d_reconsLookupTexes[idx] != 0)
            CUDA_RUNTIME_API_CALL(
                cudaDestroyTextureObject(d_reconsLookupTexes[idx]));
        CUDA_RUNTIME_API_CALL(cudaCreateTextureObject(
            &d_reconsLookupTexes[idx], &rsrcDesc, &texDesc, nullptr));

        dim3 blockPerGrid{
            (renderSz.x + threadPerBlock.x - 1) / threadPerBlock.x,
            (renderSz.y + threadPerBlock.y - 1) / threadPerBlock.y, 2};
        createReconsTexKernel<<<blockPerGrid, threadPerBlock>>>(
            d_tmp, lvl, renderSz.x, FAVRSubsampleWidths[idx]);
        dismissGapKernel<<<blockPerGrid, threadPerBlock>>>(
            d_tmp, lvl, renderSz.x, FAVRSubsampleWidths[idx]);
        CUDA_RUNTIME_API_CALL(cudaMemcpyToArray(d_reconsLookupTexArrs[idx], 0,
                                                0, d_tmp, cpySz,
                                                cudaMemcpyDeviceToDevice));
    }
    CUDA_RUNTIME_API_CALL(
        cudaMemcpyToSymbol(dc_reconsLookupTexes, d_reconsLookupTexes,
                           sizeof(cudaTextureObject_t) * MAX_SUBSAMPLE_LEVEL));

    CUDA_RUNTIME_API_CALL(cudaFree(d_tmp));
}
