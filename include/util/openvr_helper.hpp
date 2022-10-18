#ifndef KOUEK_OPEN_VR_HELPER
#define KOUEK_OPEN_VR_HELPER

#include <stdexcept>
#include <functional>
#include <thread>

#include <glm/glm.hpp>
#include <openvr.h>

namespace kouek {

static inline glm::mat4x3
SteamVRMat34ToGLMMat43(const vr::HmdMatrix34_t &stMat34) {
    return {stMat34.m[0][0], stMat34.m[1][0], stMat34.m[2][0], stMat34.m[0][1],
            stMat34.m[1][1], stMat34.m[2][1], stMat34.m[0][2], stMat34.m[1][2],
            stMat34.m[2][2], stMat34.m[0][3], stMat34.m[1][3], stMat34.m[2][3]};
}

static inline glm::mat4
SteamVRMat34ToGLMMat4(const vr::HmdMatrix34_t &stMat34) {
    return {stMat34.m[0][0], stMat34.m[1][0], stMat34.m[2][0], 0,
            stMat34.m[0][1], stMat34.m[1][1], stMat34.m[2][1], 0,
            stMat34.m[0][2], stMat34.m[1][2], stMat34.m[2][2], 0,
            stMat34.m[0][3], stMat34.m[1][3], stMat34.m[2][3], 1.f};
}

static inline glm::mat4
SteamVRMat44ToGLMMat4(const vr::HmdMatrix44_t &stMat44) {
    return {stMat44.m[0][0], stMat44.m[1][0], stMat44.m[2][0], stMat44.m[3][0],
            stMat44.m[0][1], stMat44.m[1][1], stMat44.m[2][1], stMat44.m[3][1],
            stMat44.m[0][2], stMat44.m[1][2], stMat44.m[2][2], stMat44.m[3][2],
            stMat44.m[0][3], stMat44.m[1][3], stMat44.m[2][3], stMat44.m[3][3]};
}

static std::tuple<vr::RenderModel_t *, vr::RenderModel_TextureMap_t *>
GetRenderModelAndTex(const char *modelName) {
    vr::RenderModel_t *model = nullptr;
    vr::EVRRenderModelError error;
    while (true) {
        error = vr::VRRenderModels()->LoadRenderModel_Async(modelName, &model);
        if (error != vr::VRRenderModelError_Loading)
            break;
        // block until loading model finished
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (error != vr::VRRenderModelError_None)
        throw std::runtime_error(
            std::string("Load render model FAILED: Model Name: ") + modelName +
            ", " +
            vr::VRRenderModels()->GetRenderModelErrorNameFromEnum(error));

    vr::RenderModel_TextureMap_t *tex;
    while (1) {
        error = vr::VRRenderModels()->LoadTexture_Async(model->diffuseTextureId,
                                                        &tex);
        if (error != vr::VRRenderModelError_Loading)
            break;
        // block until loading texture finished
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (error != vr::VRRenderModelError_None)
        throw std::runtime_error(
            std ::string("Load render model's texture FAILED: Model Name: ") +
            modelName +
            ", Texture ID: " + std::to_string(model->diffuseTextureId) + ", " +
            vr::VRRenderModels()->GetRenderModelErrorNameFromEnum(error));
    return {model, tex};
}

} // namespace kouek
#endif // !KOUEK_OPEN_VR_HELPER
