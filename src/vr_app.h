#ifndef KOUEK_VR_APP_H
#define KOUEK_VR_APP_H

#include <functional>
#include <vector>

#include <util/openvr_helper.hpp>

#include "shared_states.h"

namespace kouek {
class VRApp {
  public:
    static constexpr uint8_t PathInteractHndIdx = 0;
    static constexpr uint8_t VolInteractHndIdx = 1;

  private:
    enum class DigitActIdx : uint8_t {
        TriggerClick = 0,
        TrackpadSClick,
        TrackpadNClick,
        TrackpadWClick,
        TrackpadEClick,
        Menu,
        End
    };
    enum class AnalogActIdx : uint8_t { TriggerPull = 0, End };

  private:
    template <typename ActIdxTy>
    static inline constexpr uint8_t actIdxToIdx(ActIdxTy idx) {
        return static_cast<uint8_t>(idx);
    };

  private:
    std::array<std::array<bool, actIdxToIdx(DigitActIdx::End)>, 2>
        digitActionActives2;
    std::array<std::array<bool, actIdxToIdx(DigitActIdx::End)>, 2>
        digitActionStates2;
    std::array<std::array<bool, actIdxToIdx(DigitActIdx::End)>, 2>
        digitActionChangeds2;
    std::array<std::array<vr::VRActionHandle_t, actIdxToIdx(DigitActIdx::End)>,
               2>
        digitActions2;

    std::array<std::array<bool, actIdxToIdx(DigitActIdx::End)>, 2>
        analogActionActives2;
    std::array<std::array<glm::vec3, actIdxToIdx(AnalogActIdx::End)>, 2>
        analogActionDeltas2;
    std::array<std::array<vr::VRActionHandle_t, actIdxToIdx(AnalogActIdx::End)>,
               2>
        analogActions2;

    std::array<vr::Texture_t, 2> submitTex2{0};
    vr::IVRSystem *HMD;

    vr::VRActionSetHandle_t actionsetWanderMode =
        vr::k_ulInvalidActionSetHandle;

    std::array<vr::VRActionHandle_t, 2> actionHandPose2 = {
        vr::k_ulInvalidActionHandle};

    std::array<glm::mat4x3, 2> eyeToHeadMat2;
    std::array<glm::mat4, vr::k_unMaxTrackedDeviceCount> devicePoses;

    std::shared_ptr<SharedStates> sharedStates;
    std::shared_ptr<StatefulSystem> statefulSys;

  public:
    VRApp(std::shared_ptr<SharedStates> sharedStates,
          std::shared_ptr<StatefulSystem> statefulSys);
    inline void SetSubmitTex2(const std::array<GLuint, 2> &submitTexID2) {
        submitTex2 = {
            vr::Texture_t{(void *)(uintptr_t)submitTexID2[vr::Eye_Left],
                          vr::TextureType_OpenGL, vr::ColorSpace_Gamma},
            vr::Texture_t{(void *)(uintptr_t)submitTexID2[vr::Eye_Right],
                          vr::TextureType_OpenGL, vr::ColorSpace_Gamma}};
    }
    void ProcessInput();
    void ProcessOutput();

  private:
    void initSignalAndSlots();
    void processInputToRender();
    void processInputToGUI();
};
} // namespace kouek

#endif // !KOUEK_VR_APP_H
