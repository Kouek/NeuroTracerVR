#include "comp_vol_algorithm.h"
#include "comp_vol_vr_renderer.h"

#include <vs_core/VolumeSlicer/helper.hpp>

#include <thrust/async/reduce.h>
#include <thrust/device_vector.h>

#include <device_launch_parameters.h>

using namespace kouek;

struct BufElem {
    glm::vec3 pos;
    float scalar;

    __host__ __device__ bool operator<(const BufElem& other) const {
        return this->scalar < other.scalar;
    }
};

extern __constant__ CompVolVRRenderer::VolumeParam dc_volumeParam;

struct MaxVoxParam {
    glm::uvec3 sampleBoxSz;
    float minScalar;
};
static MaxVoxParam maxVoxParam;
static __constant__ MaxVoxParam dc_maxVoxParam;

static const dim3 threadPerBlock{16, 16};

static thrust::device_vector<BufElem> YZBuf;

static cudaStream_t algorithmStream = nullptr;
static thrust::future<thrust::cuda_cub::execute_on_stream, BufElem>
    maxPosFuture;

void kouek::prepareMaxVoxPos(const glm::uvec3 &sampleBoxSzVSp,
                             float minScalar) {
    if (algorithmStream == nullptr)
        CUDA_RUNTIME_CHECK(cudaStreamCreate(&algorithmStream));

    maxVoxParam.sampleBoxSz = sampleBoxSzVSp;
    maxVoxParam.minScalar = minScalar;
    CUDA_RUNTIME_API_CALL(
        cudaMemcpyToSymbol(dc_maxVoxParam, &maxVoxParam, sizeof(maxVoxParam)));
    YZBuf.resize((size_t)sampleBoxSzVSp.y * sampleBoxSzVSp.z);
}

extern __device__ float virtualSampleLOD0(const glm::vec3 &samplePos);

__global__ void maxXYZVoxKernel(BufElem *outBuf, glm::vec3 rangeVSp,
                                glm::vec3 minVSp) {
    glm::uvec2 yz{blockIdx.x * blockDim.x + threadIdx.x,
                  blockIdx.y * blockDim.y + threadIdx.y};
    if (yz[0] >= dc_maxVoxParam.sampleBoxSz.y ||
        yz[1] >= dc_maxVoxParam.sampleBoxSz.z)
        return;

    BufElem bufElem;
    bufElem.scalar = 0.f;

    auto step3 = rangeVSp;
    step3.x /= (float)dc_maxVoxParam.sampleBoxSz.x;
    step3.y /= (float)dc_maxVoxParam.sampleBoxSz.y;
    step3.z /= (float)dc_maxVoxParam.sampleBoxSz.z;

    auto pos = minVSp;
    pos.y += step3.y * yz[0];
    pos.z += step3.z * yz[1];
    for (glm::uint stepCnt = 0; stepCnt < dc_maxVoxParam.sampleBoxSz.x;
         ++stepCnt, pos.x += step3.x) {
        auto samplePos = pos / dc_volumeParam.spaces;
        auto scalar = virtualSampleLOD0(samplePos);
        if (scalar < dc_maxVoxParam.minScalar)
            continue;
        
        if (scalar > bufElem.scalar) {
            bufElem.scalar = scalar;
            bufElem.pos = pos;
        }
    }

    if(bufElem.scalar == 0.f)
        // no valid voxel found
        bufElem.pos.x = NONE_POS_VAL;
    outBuf[(size_t)yz[1] * dc_maxVoxParam.sampleBoxSz.z + yz[0]] = bufElem;
}

void kouek::execMaxVoxPos(const glm::vec3 &minVSp, const glm::vec3 &maxVSp) {
    dim3 blockPerGrid{
        (maxVoxParam.sampleBoxSz.y + threadPerBlock.x - 1) / threadPerBlock.x,
        (maxVoxParam.sampleBoxSz.z + threadPerBlock.y - 1) / threadPerBlock.y};
    maxXYZVoxKernel<<<blockPerGrid, threadPerBlock, 0, algorithmStream>>>(
        thrust::raw_pointer_cast(YZBuf.data()), maxVSp - minVSp, minVSp);

    BufElem initVal{glm::zero<glm::vec3>(), 0.f};
    maxPosFuture =
        thrust::async::reduce(thrust::device.on(algorithmStream), YZBuf.begin(),
                              YZBuf.end(), initVal, thrust::maximum<BufElem>());
}

std::tuple<bool, glm::vec3> kouek::fetchMaxVoxPos() {
    if (maxPosFuture.ready())
        return std::make_tuple(true, maxPosFuture.extract().pos);
    return std::make_tuple(false, glm::zero<glm::vec3>());
}

void kouek::waitForAllVoxAlgorithms() {
    cudaStreamSynchronize(algorithmStream);
    maxPosFuture = decltype(maxPosFuture)(); // reset
    cudaStreamDestroy(algorithmStream);
}