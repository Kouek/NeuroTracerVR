#ifndef KOUEK_SHARED_STATES_H
#define KOUEK_SHARED_STATES_H

#include <camera/dual_eye_camera.hpp>
#include <glm/glm.hpp>
#include <stateful/stateful.hpp>

#include <util/openvr_helper.hpp>

#include "renderer/vr_renderer.h"

namespace kouek {

struct HandStates {
    bool show = false;
    bool clicked = false;
    bool pressed = false;
    glm::mat4 pose;
    std::unique_ptr<VRRenderModel> model;
    std::string modelName;
};

enum class InteractionMode { SelectVert = 0, AddPath, AddVert, End };

struct SharedStates {
    static constexpr float NEAR_CLIP = .1f, FAR_CLIP = 10.f;

    bool canRun = true;
    bool canVRRun = false;

    VRRenderer::RenderTarget renderTarget = VRRenderer::RenderTarget::FAVRVol;
    InteractionMode interactiveMode = InteractionMode::AddPath;

    uint8_t FAVRLvl = 4;
    uint32_t maxStepCnt = 1000;
    glm::uvec2 renderSz{1080, 1080};

    float scaleW2V = .2f;
    glm::vec3 translateW2V{22.2718792f * .2f, 36.81226598f * .2f,
                           32.1920395f * .2f};
    std::array<glm::mat4, 2> projection2{
        glm::perspectiveFov(glm::radians(90.f), (float)renderSz.x,
                            (float)renderSz.y, scaleW2V *NEAR_CLIP,
                            scaleW2V *FAR_CLIP),
        glm::perspectiveFov(glm::radians(90.f), (float)renderSz.x,
                            (float)renderSz.y, scaleW2V *NEAR_CLIP,
                            scaleW2V *FAR_CLIP)};

    DualEyeCamera camera;
    std::array<HandStates, 2> handStates2;

    SharedStates(StatefulSystem &statefulSys) {
        statefulSys.RegisterExecOnce(
            [&]() {
                camera.SetEyeToHead(glm::zero<glm::vec3>(),
                                    glm::zero<glm::vec3>());
                camera.SetHeadPos(glm::zero<glm::vec3>());
                camera.SetSelfRotation(glm::mat3{
                    glm::lookAt(glm::vec3{.5f, .5f, .5f},
                                glm::vec3{.5f, .5f, 0}, glm::vec3{0, 1.f, 0})});
            },
            std::tie(camera));
        auto doNothing = []() {};
        statefulSys.RegisterExecOnce(
            doNothing, std::tie(FAVRLvl, maxStepCnt, renderSz, scaleW2V,
                                translateW2V, projection2));
    }
};

} // namespace kouek

#endif // !KOUEK_SHARED_STATES_H
