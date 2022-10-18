#include "vr_app.h"

#include <cmake_in.h>

#include <spdlog/spdlog.h>

kouek::VRApp::VRApp(std::shared_ptr<SharedStates> sharedStates,
                    std::shared_ptr<StatefulSystem> statefulSys)
    : sharedStates(sharedStates), statefulSys(statefulSys) {
    vr::EVRInitError initError;
    HMD = vr::VR_Init(&initError, vr::VRApplication_Scene);
    if (initError != vr::VRInitError_None) {
        sharedStates->canVRRun = false;
        return;
    }
    if (!vr::VRCompositor()) {
        sharedStates->canVRRun = false;
        return;
    }

    if (vr::EVRInputError inputError = vr::VRInput()->SetActionManifestPath(
            (std ::string(PROJECT_SOURCE_DIR) + "/cfg/actions.json").c_str());
        inputError != vr::VRInputError_None) {
        sharedStates->canVRRun = false;
        return;
    }

#define PROCESS_ERR(func)                                                      \
    if (vr::EVRInputError inputErr = func;                                     \
        inputErr != vr::VRInputError_None) {                                   \
        sharedStates->canVRRun = false;                                        \
        return;                                                                \
    }

    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/left_axis1_click", &actionLeftTriggerClick));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/left_axis1_pull", &actionLeftTriggerPull));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/left_south_click", &actionLeftTrackpadSClick));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/left_north_click", &actionLeftTrackpadNClick));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/left_west_click", &actionLeftTrackpadWClick));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/left_east_click", &actionLeftTrackpadEClick));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/right_axis1_click", &actionRightTriggerClick));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/right_axis1_pull", &actionRightTriggerPull));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/left_applicationmenu_press", &actionLeftMenu));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/right_south_click", &actionRightTrackpadSClick));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/right_north_click", &actionRightTrackpadNClick));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/right_west_click", &actionRightTrackpadWClick));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/right_east_click", &actionRightTrackpadEClick));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/right_applicationmenu_press", &actionRightMenu));

    PROCESS_ERR(vr::VRInput()->GetActionSetHandle("/actions/wander",
                                                  &actionsetWanderMode));

    PROCESS_ERR(vr::VRInput()->GetInputSourceHandle("/user/hand/left",
                                                    &sourceHand2[0]));
    PROCESS_ERR(vr::VRInput()->GetActionHandle("/actions/wander/in/Left_Pose",
                                               &actionHandPose2[0]));

    PROCESS_ERR(vr::VRInput()->GetInputSourceHandle("/user/hand/right",
                                                    &sourceHand2[1]));
    PROCESS_ERR(vr::VRInput()->GetActionHandle("/actions/wander/in/Right_Pose",
                                               &actionHandPose2[1]));
#undef PROCESS_ERR

    for (uint8_t lr = 0; lr < 2; ++lr) {
        eyeToHeadMat2[lr] = SteamVRMat34ToGLMMat43(
            HMD->GetEyeToHeadTransform(static_cast<vr::EVREye>(lr)));
    }

    sharedStates->canVRRun = true;

    initSignalSlots();
}

void kouek::VRApp::ProcessInput() {
    if (!sharedStates->canVRRun)
        return;

    {
        std::array<vr::TrackedDevicePose_t, vr::k_unMaxTrackedDeviceCount>
            trackedDevicePoses;
        vr::VRCompositor()->WaitGetPoses(
            trackedDevicePoses.data(), vr::k_unMaxTrackedDeviceCount, NULL, 0);
        for (uint32_t devIdx = 0; devIdx < vr::k_unMaxTrackedDeviceCount;
             ++devIdx) {
            if (trackedDevicePoses[devIdx].bPoseIsValid)
                devicePoses[devIdx] = SteamVRMat34ToGLMMat4(
                    trackedDevicePoses[devIdx].mDeviceToAbsoluteTracking);
        }
    }
}

void kouek::VRApp::ProcessOutput() {
    if (!sharedStates->canVRRun)
        return;
    vr::VRCompositor()->Submit(vr::Eye_Left, &submitTex2[vr::Eye_Left]);
    vr::VRCompositor()->Submit(vr::Eye_Right, &submitTex2[vr::Eye_Right]);
}

void kouek::VRApp::initSignalSlots() {
    {
        uint32_t w, h;
        HMD->GetRecommendedRenderTargetSize(&w, &h);
        spdlog::info(
            "VR App Info: get recommended rendering size: {0} x {1} x 2", w, h);
        spdlog::info("VR App Info: used rendering size: {0} x {1} x 2",
                     sharedStates->renderSz.x, sharedStates->renderSz.y);
    }

    statefulSys->Register(
        [&]() {
                sharedStates->camera.SetPosture(
                devicePoses[vr::k_unTrackedDeviceIndex_Hmd]);
        },
        std::tie(sharedStates->camera));

    statefulSys->Register(
        std::tie(sharedStates->preScale),
        [&]() {
            sharedStates->camera.SetEyeToHead(
                sharedStates->preScale * eyeToHeadMat2[0][3],
                sharedStates->preScale * eyeToHeadMat2[1][3]);
            for (uint8_t lr = 0; lr < 2; ++lr)
                sharedStates->projection2[lr] =
                    SteamVRMat44ToGLMMat4(HMD->GetProjectionMatrix(
                        static_cast<vr::EVREye>(lr),
                        sharedStates->preScale * SharedStates::NEAR_CLIP,
                        sharedStates->preScale * SharedStates::FAR_CLIP));
        },
        std::tie(sharedStates->camera, sharedStates->projection2));
}
