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

    PROCESS_ERR(vr::VRInput()->GetActionSetHandle("/actions/wander",
                                                  &actionsetWanderMode));

    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/left_axis1_pull",
        &analogActions2[0][actIdxToIdx(AnalogActIdx::TriggerPull)]));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/right_axis1_pull",
        &analogActions2[1][actIdxToIdx(AnalogActIdx::TriggerPull)]));

    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/left_axis1_click",
        &digitActions2[0][actIdxToIdx(DigitActIdx::TriggerClick)]));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/left_south_click",
        &digitActions2[0][actIdxToIdx(DigitActIdx::TrackpadSClick)]));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/left_north_click",
        &digitActions2[0][actIdxToIdx(DigitActIdx::TrackpadNClick)]));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/left_west_click",
        &digitActions2[0][actIdxToIdx(DigitActIdx::TrackpadWClick)]));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/left_east_click",
        &digitActions2[0][actIdxToIdx(DigitActIdx::TrackpadEClick)]));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/left_applicationmenu_press",
        &digitActions2[0][actIdxToIdx(DigitActIdx::Menu)]));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/right_axis1_click",
        &digitActions2[1][actIdxToIdx(DigitActIdx::TriggerClick)]));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/right_south_click",
        &digitActions2[1][actIdxToIdx(DigitActIdx::TrackpadSClick)]));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/right_north_click",
        &digitActions2[1][actIdxToIdx(DigitActIdx::TrackpadNClick)]));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/right_west_click",
        &digitActions2[1][actIdxToIdx(DigitActIdx::TrackpadWClick)]));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/right_east_click",
        &digitActions2[1][actIdxToIdx(DigitActIdx::TrackpadEClick)]));
    PROCESS_ERR(vr::VRInput()->GetActionHandle(
        "/actions/wander/in/right_applicationmenu_press",
        &digitActions2[1][actIdxToIdx(DigitActIdx::Menu)]));

    PROCESS_ERR(vr::VRInput()->GetActionHandle("/actions/wander/in/left_pose",
                                               &actionHandPose2[0]));
    PROCESS_ERR(vr::VRInput()->GetActionHandle("/actions/wander/in/right_pose",
                                               &actionHandPose2[1]));
#undef PROCESS_ERR

    for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx) {
        eyeToHeadMat2[eyeIdx] = SteamVRMat34ToGLMMat43(
            HMD->GetEyeToHeadTransform(static_cast<vr::EVREye>(eyeIdx)));
    }

    sharedStates->canVRRun = true;

    initSignalAndSlots();
}

void kouek::VRApp::ProcessInput() {
    if (!sharedStates->canVRRun)
        return;

    // update action set
    vr::VRActiveActionSet_t activeActionSet = {0};
    activeActionSet.ulActionSet = actionsetWanderMode;
    vr::VRInput()->UpdateActionState(&activeActionSet, sizeof(activeActionSet),
                                     1);
    // update HMD pose
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
    // update hand pose
    for (uint8_t hndIdx = 0; hndIdx < 2; ++hndIdx) {
        vr::InputPoseActionData_t poseData;
        if (vr::VRInput()->GetPoseActionDataForNextFrame(
                actionHandPose2[hndIdx], vr::TrackingUniverseStanding,
                &poseData, sizeof(poseData),
                vr::k_ulInvalidInputValueHandle) != vr::VRInputError_None ||
            !poseData.bActive || !poseData.pose.bPoseIsValid)
            sharedStates->handStates2[hndIdx].show = false;
        else {
            sharedStates->handStates2[hndIdx].show = true;
            sharedStates->handStates2[hndIdx].pose =
                SteamVRMat34ToGLMMat4(poseData.pose.mDeviceToAbsoluteTracking);

            vr::InputOriginInfo_t originInfo;
            if (vr::VRInput()->GetOriginTrackedDeviceInfo(
                    poseData.activeOrigin, &originInfo, sizeof(originInfo)) ==
                    vr::VRInputError_None &&
                originInfo.trackedDeviceIndex !=
                    vr::k_unTrackedDeviceIndexInvalid) {
                std::string modelName =
                    GetTrackedDeviceString(originInfo.trackedDeviceIndex,
                                           vr::Prop_RenderModelName_String);
                // when name changed, change render model
                if (modelName != sharedStates->handStates2[hndIdx].modelName) {
                    sharedStates->handStates2[hndIdx].modelName = modelName;
                    auto [model, tex] = GetRenderModelAndTex(modelName.c_str());
                    if (model != nullptr && tex != nullptr)
                        sharedStates->handStates2[hndIdx].model =
                            std::make_unique<VRRenderModel>(*model, *tex);
                }
            }
        }
    }
    // update digital input
    {
        vr::InputDigitalActionData_t actionData;
        for (uint8_t hndIdx = 0; hndIdx < 2; ++hndIdx)
            for (uint8_t idx = 0; idx < actIdxToIdx(DigitActIdx::End); ++idx) {
                vr::VRInput()->GetDigitalActionData(
                    digitActions2[hndIdx][idx], &actionData, sizeof(actionData),
                    vr::k_ulInvalidInputValueHandle);
                digitActionActives2[hndIdx][idx] = actionData.bActive;
                digitActionStates2[hndIdx][idx] = actionData.bState;
                digitActionChangeds2[hndIdx][idx] = actionData.bChanged;
            }
    }
    // update analog input
    {
        vr::InputAnalogActionData_t actionData;
        for (uint8_t hndIdx = 0; hndIdx < 2; ++hndIdx)
            for (uint8_t idx = 0; idx < actIdxToIdx(AnalogActIdx::End); ++idx) {
                vr::VRInput()->GetAnalogActionData(
                    analogActions2[hndIdx][idx], &actionData,
                    sizeof(actionData), vr::k_ulInvalidInputValueHandle);
                analogActionActives2[hndIdx][idx] = actionData.bActive;
                analogActionDeltas2[hndIdx][idx] = {
                    actionData.deltaX, actionData.deltaY, actionData.deltaZ};
            }
    }
}

void kouek::VRApp::ProcessOutput() {
    if (!sharedStates->canVRRun)
        return;
    vr::VRCompositor()->Submit(vr::Eye_Left, &submitTex2[vr::Eye_Left]);
    vr::VRCompositor()->Submit(vr::Eye_Right, &submitTex2[vr::Eye_Right]);
}

void kouek::VRApp::initSignalAndSlots() {
    // This function map VR inputs to sharedStates

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
        std::tie(sharedStates->scaleW2V),
        [&]() {
            sharedStates->camera.SetEyeToHead(
                sharedStates->scaleW2V * eyeToHeadMat2[0][3],
                sharedStates->scaleW2V * eyeToHeadMat2[1][3]);
            for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx)
                sharedStates->projection2[eyeIdx] =
                    SteamVRMat44ToGLMMat4(HMD->GetProjectionMatrix(
                        static_cast<vr::EVREye>(eyeIdx),
                        sharedStates->scaleW2V * SharedStates::NEAR_CLIP,
                        sharedStates->scaleW2V * SharedStates::FAR_CLIP));
        },
        std::tie(sharedStates->camera, sharedStates->projection2));

    static constexpr auto MOVE_SENSITY = .01f;
    // trackpad -> preTranslate
    statefulSys->Register(
        [&]() {
            sharedStates->translateW2V += sharedStates->scaleW2V *
                                          MOVE_SENSITY *
                                          sharedStates->camera.GetF();
        },
        std::tie(sharedStates->translateW2V),
        [&]() {
            return digitActionActives2[1][actIdxToIdx(
                       DigitActIdx::TrackpadNClick)] &&
                   digitActionStates2[1]
                                     [actIdxToIdx(DigitActIdx::TrackpadNClick)];
        });
    statefulSys->Register(
        [&]() {
            sharedStates->translateW2V -= sharedStates->scaleW2V *
                                          MOVE_SENSITY *
                                          sharedStates->camera.GetF();
        },
        std::tie(sharedStates->translateW2V),
        [&]() {
            return digitActionActives2[1][actIdxToIdx(
                       DigitActIdx::TrackpadSClick)] &&
                   digitActionStates2[1]
                                     [actIdxToIdx(DigitActIdx::TrackpadSClick)];
        });
    statefulSys->Register(
        [&]() {
            sharedStates->translateW2V += sharedStates->scaleW2V *
                                          MOVE_SENSITY *
                                          sharedStates->camera.GetF();
        },
        std::tie(sharedStates->translateW2V),
        [&]() {
            return digitActionActives2[1][actIdxToIdx(
                       DigitActIdx::TrackpadNClick)] &&
                   digitActionStates2[1]
                                     [actIdxToIdx(DigitActIdx::TrackpadNClick)];
        });
    statefulSys->Register(
        [&]() {
            sharedStates->translateW2V += sharedStates->scaleW2V *
                                          MOVE_SENSITY *
                                          sharedStates->camera.GetR();
        },
        std::tie(sharedStates->translateW2V),
        [&]() {
            return digitActionActives2[1][actIdxToIdx(
                       DigitActIdx::TrackpadEClick)] &&
                   digitActionStates2[1]
                                     [actIdxToIdx(DigitActIdx::TrackpadEClick)];
        });
    statefulSys->Register(
        [&]() {
            sharedStates->translateW2V -= sharedStates->scaleW2V *
                                          MOVE_SENSITY *
                                          sharedStates->camera.GetR();
        },
        std::tie(sharedStates->translateW2V),
        [&]() {
            return digitActionActives2[1][actIdxToIdx(
                       DigitActIdx::TrackpadWClick)] &&
                   digitActionStates2[1]
                                     [actIdxToIdx(DigitActIdx::TrackpadWClick)];
        });
    // trigger -> some actions
    statefulSys->Register(
        [&]() { sharedStates->handStates2[PathInteractHndIdx].clicked = true; },
        std::tie(sharedStates->handStates2[PathInteractHndIdx].clicked),
        [&]() {
            return digitActionActives2[PathInteractHndIdx][actIdxToIdx(
                       DigitActIdx::TriggerClick)] &&
                   !digitActionStates2[PathInteractHndIdx][actIdxToIdx(
                       DigitActIdx::TriggerClick)] &&
                   digitActionChangeds2[PathInteractHndIdx]
                                       [actIdxToIdx(DigitActIdx::TriggerClick)];
        });
    statefulSys->Register(
        [&]() { sharedStates->handStates2[PathInteractHndIdx].pressed = true; },
        std::tie(sharedStates->handStates2[PathInteractHndIdx].pressed),
        [&]() {
            return digitActionActives2[PathInteractHndIdx][actIdxToIdx(
                       DigitActIdx::TriggerClick)] &&
                   digitActionStates2[PathInteractHndIdx]
                                     [actIdxToIdx(DigitActIdx::TriggerClick)] &&
                   !digitActionChangeds2[PathInteractHndIdx][actIdxToIdx(
                       DigitActIdx::TriggerClick)];
        });
}
