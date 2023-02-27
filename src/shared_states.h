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

enum class InteractionMode {
    SelectVert = 0,
    AddPath,
    AddVert,
    MoveVert,
    DeleteVert,
    JoinPath,
    End
};
enum class GUIPage { None = 0, IntrctModePage, End };

// Selectable Widget in ImGui doesn't support text wrapping currently.
// Thus we use a \n to keep texts in the view scope.
static inline constexpr std::array InteractionModeNames{
    "Select\nVertex", "Add\nPath",      "Add\nVertex",
    "Move\nVertex",   "Delete\nVertex", "Join\nPath"};
static inline constexpr std::array GUIPageNames{"None",
                                                "Select Interaction Mode"};

struct GUIIntrctModePageStates {
    static constexpr uint8_t ROW_NUM = 3;
    static constexpr uint8_t COL_NUM = 3;

    std::array<bool, static_cast<size_t>(InteractionMode::End)> selecteds;
    uint8_t selectedIdx = 0;

    GUIIntrctModePageStates() {
        std::fill(selecteds.begin(), selecteds.end(), false);
        selecteds[selectedIdx] = true;
    }

    /// <summary>
    /// Change the current selected element
    /// to one element lft, rht, down or up direction.
    /// </summary>
    /// <param name="dir">0 for lft, 1 for rht, 2 for down and 3 for up</param>
    void ChangeSelected(uint8_t dir) {
        if (dir == 0 && selectedIdx >= 1 &&
            (selectedIdx / COL_NUM == (selectedIdx - 1) / COL_NUM)) {
            selecteds[selectedIdx] = false;
            selectedIdx -= 1;
            selecteds[selectedIdx] = true;
        } else if (dir == 1 && selectedIdx < selecteds.size() - 1 &&
            (selectedIdx / COL_NUM == (selectedIdx + 1) / COL_NUM)) {
            selecteds[selectedIdx] = false;
            selectedIdx += 1;
            selecteds[selectedIdx] = true;
        } else if (dir == 2 && selectedIdx < selecteds.size() - COL_NUM) {
            selecteds[selectedIdx] = false;
            selectedIdx += COL_NUM;
            selecteds[selectedIdx] = true;
        } else if (dir == 3 && selectedIdx >= COL_NUM) {
            selecteds[selectedIdx] = false;
            selectedIdx -= COL_NUM;
            selecteds[selectedIdx] = true;
        }
    }
    inline void ChangeSelected(InteractionMode mode) {
        selecteds[selectedIdx] = false;
        selectedIdx = static_cast<decltype(selectedIdx)>(mode);
        selecteds[selectedIdx] = true;
    }
};

struct SharedStates {
    static constexpr float NEAR_CLIP = .1f, FAR_CLIP = 10.f;
    static constexpr float MAX_SEARCH_MIN_SCALAR =
        30.f / 255.f + std::numeric_limits<float>::epsilon();
    static constexpr float INTERACT_CUBE_HF_WID = .02f;
    static constexpr float INTERACT_VERT_HF_WID = INTERACT_CUBE_HF_WID * .2f;
    static constexpr float INTERACT_CUBE_CNTR_TO_HND_DIST = .05f;
    static constexpr glm::vec3 INTERACT_CUBE_COL{1.f, 1.f, 1.f};
    static constexpr glm::vec3 INTERACT_VERT_COL{1.f, .5f, 1.f};
    static constexpr glm::uvec3 MAX_SEARCH_SAMPLE_DIM{128, 128, 128};

    bool canRun = true;
    bool canVRRun = false;

    VRRenderer::RenderTarget renderTarget = VRRenderer::RenderTarget::FAVRVol;
    InteractionMode interactiveMode = InteractionMode::SelectVert;
    GUIPage guiPage = GUIPage::None;

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
    GUIIntrctModePageStates guiIntrctModePageStates;

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
        statefulSys.SetModified(std::tie(FAVRLvl, maxStepCnt, renderSz,
                                         scaleW2V, translateW2V, projection2));
    }
};

} // namespace kouek

#endif // !KOUEK_SHARED_STATES_H
