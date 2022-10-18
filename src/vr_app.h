#ifndef KOUEK_VR_APP_H
#define KOUEK_VR_APP_H

#include <vector>
#include <functional>

#include <util/openvr_helper.hpp>

#include "shared_states.h"

namespace kouek {
class VRApp {
  private:
    std::array<vr::Texture_t, 2> submitTex2{0};
    vr::IVRSystem *HMD;

    vr::VRActionSetHandle_t actionsetWanderMode =
        vr::k_ulInvalidActionSetHandle;

    vr::VRActionHandle_t actionLeftTriggerClick = vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t actionLeftTriggerPull = vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t actionLeftTrackpadSClick = vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t actionLeftTrackpadNClick = vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t actionLeftTrackpadWClick = vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t actionLeftTrackpadEClick = vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t actionLeftMenu = vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t actionRightTriggerClick = vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t actionRightTriggerPull = vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t actionRightTrackpadSClick =
        vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t actionRightTrackpadNClick =
        vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t actionRightTrackpadWClick =
        vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t actionRightTrackpadEClick =
        vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t actionRightMenu = vr::k_ulInvalidActionHandle;

    std::array<vr::VRActionHandle_t, 2> actionHandPose2 = {
        vr::k_ulInvalidActionHandle};
    std::array<vr::VRInputValueHandle_t, 2> sourceHand2 = {
        vr::k_ulInvalidInputValueHandle};

    std::array<glm::mat4x3, 2> eyeToHeadMat2;
    std::array<glm::mat4, vr::k_unMaxTrackedDeviceCount> devicePoses;

    std::shared_ptr<SharedStates> sharedStates;
    std::shared_ptr<StatefulSystem> statefulSys;

  public:
    VRApp(std::shared_ptr<SharedStates> sharedStates,
          std::shared_ptr<StatefulSystem> statefulSys);
    inline void SetSubmitTex2(const std::array<GLuint, 2>& submitTexID2) {
        submitTex2 = {
            vr::Texture_t{(void *)(uintptr_t)submitTexID2[vr::Eye_Left],
                          vr::TextureType_OpenGL, vr::ColorSpace_Gamma},
            vr::Texture_t{(void *)(uintptr_t)submitTexID2[vr::Eye_Right],
                          vr::TextureType_OpenGL, vr::ColorSpace_Gamma}};
    }
    void ProcessInput();
    void ProcessOutput();

  private:
    void initSignalSlots();
};
} // namespace kouek

#endif // !KOUEK_VR_APP_H
