#include "vr_renderer.h"

#include <VolumeSlicer/helper.hpp>

#include <cuda_gl_interop.h>
#include <device_launch_parameters.h>

using namespace kouek;

static cudaArray_t d_preIntTFArray;
static cudaTextureObject_t d_preIntTF;
__constant__ cudaTextureObject_t dc_preIntTF;
__constant__ VRRenderer::LightParam dc_lightParam;
__constant__ VRRenderer::RenderParam dc_renderParam;
__constant__ VRRenderer::CameraParam dc_cameraParam;
__constant__ VRRenderer::ProjectionParam dc_projectionParam;

cudaGraphicsResource_t outputSurfRsrc2[2];
cudaGraphicsResource_t inputDepTexRsrc2[2];

void VRRenderer::SetCUDAxGLParam(const CUDAxGLParam &param) {
    size_t pxNum = param.renderSz.x * param.renderSz.y;

    for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx) {
        CUDA_RUNTIME_API_CALL(cudaGraphicsGLRegisterImage(
            &outputSurfRsrc2[eyeIdx], param.outputTex2[eyeIdx], GL_TEXTURE_2D,
            cudaGraphicsRegisterFlagsWriteDiscard));

        CUDA_RUNTIME_API_CALL(cudaGraphicsGLRegisterImage(
            &inputDepTexRsrc2[eyeIdx], param.inputDepTex2[eyeIdx],
            GL_TEXTURE_2D, cudaGraphicsRegisterFlagsReadOnly));
    }
}

void VRRenderer::SetLightParam(const LightParam &param) {
    CUDA_RUNTIME_API_CALL(
        cudaMemcpyToSymbol(dc_lightParam, &param, sizeof(LightParam)));
}

void VRRenderer::SetRenderParam(const RenderParam &param) {
    assert(param.FAVRLvl <= MAX_SUBSAMPLE_LEVEL);
    FAVRLvl = param.FAVRLvl;
    FAVRIdx = FAVRLvl - 1;
    if (renderSz != param.renderSz) {
        renderSz = param.renderSz;
        createSubsampleAndReconsTex();
    }
    CUDA_RUNTIME_API_CALL(
        cudaMemcpyToSymbol(dc_renderParam, &param, sizeof(RenderParam)));
}

void VRRenderer::SetTransferFunction(const std::array<glm::vec4, 256> &tfPnts) {
    std::vector<glm::vec4> preIntTF(256 * 256);
    float rayStep = 1.f;
    for (int32_t sb = 0; sb < 256; ++sb)
        for (int32_t sf = 0; sf <= sb; sf++) {
            int32_t offset = sf != sb;
            int32_t n = 20 + std::abs(sb - sf);
            float stepWidth = rayStep / n;
            glm::vec4 rgba{0};
            for (int32_t i = 0; i < n; i++) {
                float s = sf + (sb - sf) * (float)i / n;
                float sFrac = s - std::floor(s);
                float opacity = (tfPnts[(int32_t)s].a * (1.f - sFrac) +
                                 tfPnts[(int32_t)s + offset].a * sFrac) *
                                stepWidth;
                float temp = std::exp(-rgba.a) * opacity;
                rgba.r += (tfPnts[(int32_t)s].r * (1.f - sFrac) +
                           tfPnts[(int32_t)s + offset].r * sFrac) *
                          temp;
                rgba.g += (tfPnts[(int32_t)s].g * (1.f - sFrac) +
                           tfPnts[(int32_t)s + offset].g * sFrac) *
                          temp;
                rgba.b += (tfPnts[(int32_t)s].b * (1.f - sFrac) +
                           tfPnts[(int32_t)s + offset].b * sFrac) *
                          temp;
                rgba.a += opacity;
            }
            preIntTF[sf * 256 + sb].r = preIntTF[sb * 256 + sf].r = rgba.r;
            preIntTF[sf * 256 + sb].g = preIntTF[sb * 256 + sf].g = rgba.g;
            preIntTF[sf * 256 + sb].b = preIntTF[sb * 256 + sf].b = rgba.b;
            preIntTF[sf * 256 + sb].a = preIntTF[sb * 256 + sf].a =
                1.f - std::exp(-rgba.a);
        }

    if (d_preIntTFArray == nullptr) {
        cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<float4>();
        CUDA_RUNTIME_API_CALL(
            cudaMallocArray(&d_preIntTFArray, &channelDesc, 256, 256));
        cudaResourceDesc texRes;
        memset(&texRes, 0, sizeof(cudaResourceDesc));
        texRes.resType = cudaResourceTypeArray;
        texRes.res.array.array = d_preIntTFArray;
        cudaTextureDesc texDescr;
        memset(&texDescr, 0, sizeof(cudaTextureDesc));
        texDescr.normalizedCoords = true;
        texDescr.filterMode = cudaFilterModeLinear;
        texDescr.addressMode[0] = cudaAddressModeClamp;
        texDescr.addressMode[1] = cudaAddressModeClamp;
        texDescr.readMode = cudaReadModeElementType;
        CUDA_RUNTIME_API_CALL(
            cudaCreateTextureObject(&d_preIntTF, &texRes, &texDescr, NULL));
        CUDA_RUNTIME_API_CALL(cudaMemcpyToSymbol(dc_preIntTF, &d_preIntTF,
                                                 sizeof(cudaTextureObject_t)));
    }
    CUDA_RUNTIME_API_CALL(cudaMemcpy2DToArray(
        d_preIntTFArray, 0, 0, preIntTF.data(), sizeof(glm::vec4) * 256,
        sizeof(glm::vec4) * 256, 256, cudaMemcpyHostToDevice));
}

void VRRenderer::SetCameraParam(const CameraParam &param) {
    CUDA_RUNTIME_API_CALL(
        cudaMemcpyToSymbol(dc_cameraParam, &param, sizeof(CameraParam)));
}

void VRRenderer::SetProjectionParam(const ProjectionParam &param) {
    CUDA_RUNTIME_API_CALL(cudaMemcpyToSymbol(dc_projectionParam, &param,
                                             sizeof(ProjectionParam)));
}
