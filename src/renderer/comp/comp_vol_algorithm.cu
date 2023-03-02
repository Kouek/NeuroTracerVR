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

void kouek::prepareMaxVoxPos(const glm::uvec3 &sampleBoxSzVRSp,
                             float minScalar) {
    if (algorithmStream == nullptr)
        CUDA_RUNTIME_CHECK(cudaStreamCreate(&algorithmStream));

    maxVoxParam.sampleBoxSz = sampleBoxSzVRSp;
    maxVoxParam.minScalar = minScalar;
    CUDA_RUNTIME_API_CALL(
        cudaMemcpyToSymbol(dc_maxVoxParam, &maxVoxParam, sizeof(maxVoxParam)));
    YZBuf.resize((size_t)sampleBoxSzVRSp.y * sampleBoxSzVRSp.z);
}

extern __device__ float virtualSampleLOD0(const glm::vec3 &samplePos);

__global__ void maxXYZVoxKernel(BufElem *outBuf, glm::vec3 rangeVRSp,
                                glm::vec3 minVRSp) {
    glm::uvec2 yz{blockIdx.x * blockDim.x + threadIdx.x,
                  blockIdx.y * blockDim.y + threadIdx.y};
    if (yz[0] >= dc_maxVoxParam.sampleBoxSz.y ||
        yz[1] >= dc_maxVoxParam.sampleBoxSz.z)
        return;

    BufElem bufElem;
    bufElem.scalar = 0.f;
    bufElem.pos = glm::vec3{NONE_POS_VAL};

    auto step3 = rangeVRSp;
    step3.x /= (float)dc_maxVoxParam.sampleBoxSz.x;
    step3.y /= (float)dc_maxVoxParam.sampleBoxSz.y;
    step3.z /= (float)dc_maxVoxParam.sampleBoxSz.z;

    auto pos = minVRSp;
    pos.y += step3.y * yz[0];
    pos.z += step3.z * yz[1];
    for (glm::uint stepCnt = 0; stepCnt < dc_maxVoxParam.sampleBoxSz.x;
         ++stepCnt, pos.x += step3.x) {
        auto samplePos = pos / dc_volumeParam.spaces;
        auto scalar = virtualSampleLOD0(samplePos);
        if (scalar < dc_maxVoxParam.minScalar)
            continue;
        
        if (bufElem.scalar < scalar) {
            bufElem.scalar = scalar;
            bufElem.pos = pos;
        }
    }

    outBuf[(size_t)yz[1] * dc_maxVoxParam.sampleBoxSz.z + yz[0]] = bufElem;
}

void kouek::execMaxVoxPos(const glm::vec3 &minVRSp, const glm::vec3 &maxVRSp) {
    dim3 blockPerGrid{
        (maxVoxParam.sampleBoxSz.y + threadPerBlock.x - 1) / threadPerBlock.x,
        (maxVoxParam.sampleBoxSz.z + threadPerBlock.y - 1) / threadPerBlock.y};
    maxXYZVoxKernel<<<blockPerGrid, threadPerBlock, 0, algorithmStream>>>(
        thrust::raw_pointer_cast(YZBuf.data()), maxVRSp - minVRSp, minVRSp);

    BufElem initVal{glm::vec3{NONE_POS_VAL}, 0.f};
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
