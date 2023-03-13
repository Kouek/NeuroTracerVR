#include <cmake_in.h>

#include <limits>

#include "glfw_app.h"
#include "vr_app.h"

#include "renderer/comp/comp_vol_algorithm.h"
#include "renderer/comp/comp_vol_vr_renderer.h"
#include "shader/gl_helper.hpp"

#include <glm/gtc/matrix_transform.hpp>

#include <spdlog/spdlog.h>

#include <cg/color_tbl.hpp>
#include <cg/math.hpp>
#include <cg/shader.hpp>
#include <path/gl_path_renderer.hpp>
#include <util/vol_cfg.hpp>

using namespace kouek;

static std::shared_ptr<StatefulSystem> statefulSys;
static std::shared_ptr<SharedStates> sharedStates;

static std::unique_ptr<GLFWApp> glfwApp;
static std::unique_ptr<VRApp> vrApp;

static glm::vec3 spaces;
static float baseRayStep;
static glm::mat4 W2VR;
static glm::mat4 VR2W;
static uint32_t blockLength, noPaddingBlockLength;
static VRRenderer::CameraParam camParam;
static VRRenderer::ProjectionParam projParam;

static std::unique_ptr<VRRenderer> renderer;
static std::unique_ptr<GLPathRenderer> pathRenderer;

static ColorTable colTbl;

static std::tuple<bool, glm::vec3> interactingPos;
static glm::vec3 interactingCubeCntrPos;

// 2 pairs of FBOs for multisampling
static std::array<GLuint, 2> renderTex2;
static std::array<GLuint, 2> renderDep2;
static std::array<GLuint, 2> renderFBO2;
static std::array<GLuint, 2> submitEyeTex2;
static std::array<GLuint, 2> submitEyeDep2;
static std::array<GLuint, 2> submitEyeFBO2;

// Framebuffer's depth buffer format is not supported in CUDA,
// an RGAB8 format one is needed
static std::array<GLuint, 2> rndrDepTex2;
static std::array<GLuint, 2> rndrDepDep2;
static std::array<GLuint, 2> rndrDepFBO2;

static std::array<GLuint, 2> volRenderTex;

// Objects except for volume and hand model
static GLuint volCubeVAO, volCubeVBO, volCubeEBO;
static glm::vec3 hfVolSz;
static glm::mat4 volCubeM;

static GLuint intrctCubeVAO, intrctCubeVBO, intrctCubeEBO;

static GLuint quadVAO, quadVBO, quadEBO;

static std::unique_ptr<Shader> positionShader;
static GLint posShdrMPos, posShdrMatPos, posShdrMaxPosPos;
static std::unique_ptr<Shader> colorShader, colorDepShader;
static GLint colShdrMatPos, colShdrColPos, colDepShdrMatPos;
static std::unique_ptr<Shader> quadShader, diffuseShader, diffuseDepShader;
static GLint dfShdrMatPos, dfDepShdrMatPos;

inline auto isCalcInteractingPosReady() { return std::get<0>(interactingPos); };
inline glm::vec3 &refInteractingPos() { return std::get<1>(interactingPos); };
inline auto isInteractingPosFound() {
    return isCalcInteractingPosReady() &&
           refInteractingPos().x == refInteractingPos().x;
};

static void initRender() {
    for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx) {
        std::tie(renderFBO2[eyeIdx], renderTex2[eyeIdx], renderDep2[eyeIdx]) =
            createFrambuffer(sharedStates->renderSz.x, sharedStates->renderSz.y,
                             true);
        std::tie(submitEyeFBO2[eyeIdx], submitEyeTex2[eyeIdx],
                 submitEyeDep2[eyeIdx]) =
            createFrambuffer(sharedStates->renderSz.x,
                             sharedStates->renderSz.y);
        std::tie(rndrDepFBO2[eyeIdx], rndrDepTex2[eyeIdx],
                 rndrDepDep2[eyeIdx]) =
            createFrambuffer(sharedStates->renderSz.x,
                             sharedStates->renderSz.y);
        volRenderTex[eyeIdx] = createPlainTexture(sharedStates->renderSz.x,
                                                  sharedStates->renderSz.y);
    }
    if (sharedStates->canVRRun)
        vrApp->SetSubmitTex2(submitEyeTex2);

    std::tie(intrctCubeVAO, intrctCubeVBO, intrctCubeEBO) =
        createCubeFrame(SharedStates::INTERACT_CUBE_HF_WID);

    positionShader = std::make_unique<Shader>(
        (std::string(PROJECT_SOURCE_DIR) + "/src/shader/pos.vs").c_str(),
        (std::string(PROJECT_SOURCE_DIR) + "/src/shader/pos.fs").c_str());
    posShdrMPos = glGetUniformLocation(positionShader->ID, "M");
    assert(posShdrMPos != -1);
    posShdrMatPos = glGetUniformLocation(positionShader->ID, "matrix");
    assert(posShdrMatPos != -1);
    posShdrMaxPosPos = glGetUniformLocation(positionShader->ID, "maxPos");
    assert(posShdrMaxPosPos != -1);

    colorShader = std::make_unique<Shader>(
        (std::string(PROJECT_SOURCE_DIR) + "/src/shader/color.vs").c_str(),
        (std::string(PROJECT_SOURCE_DIR) + "/src/shader/color.fs").c_str());
    colShdrMatPos = glGetUniformLocation(colorShader->ID, "matrix");
    assert(colShdrMatPos != -1);
    colShdrColPos = glGetUniformLocation(colorShader->ID, "color");
    assert(colShdrColPos != -1);

    colorDepShader = std::make_unique<Shader>(
        (std::string(PROJECT_SOURCE_DIR) + "/src/shader/color.vs").c_str(),
        (std::string(PROJECT_SOURCE_DIR) + "/src/shader/dep.fs").c_str());
    colDepShdrMatPos = glGetUniformLocation(colorDepShader->ID, "matrix");
    assert(colDepShdrMatPos != -1);

    std::tie(quadVAO, quadVBO, quadEBO) = createQuad();

    quadShader = std::make_unique<Shader>(
        (std::string(PROJECT_SOURCE_DIR) + "/src/shader/quad.vs").c_str(),
        (std::string(PROJECT_SOURCE_DIR) + "/src/shader/quad.fs").c_str());

    diffuseShader = std::make_unique<Shader>(
        (std::string(PROJECT_SOURCE_DIR) + "/src/shader/diffuse.vs").c_str(),
        (std::string(PROJECT_SOURCE_DIR) + "/src/shader/diffuse.fs").c_str());
    dfShdrMatPos = glGetUniformLocation(diffuseShader->ID, "matrix");
    assert(dfShdrMatPos != -1);

    diffuseDepShader = std::make_unique<Shader>(
        (std::string(PROJECT_SOURCE_DIR) + "/src/shader/diffuse.vs").c_str(),
        (std::string(PROJECT_SOURCE_DIR) + "/src/shader/dep.fs").c_str());
    dfDepShdrMatPos = glGetUniformLocation(diffuseDepShader->ID, "matrix");
    assert(dfDepShdrMatPos != -1);

    glfwApp->SetSubmitTex2(submitEyeFBO2, sharedStates->renderSz);

    {
        CompVolVRRenderer::RendererParam param;
        param.texUnitNum = 6;
        param.texUnitShape = {1024, 1024, 1024};
        renderer = std::make_unique<CompVolVRRenderer>(param);
    }
    {
        VRRenderer::CUDAxGLParam param;
        param.outputTex2 = volRenderTex;
        param.inputDepTex2 = rndrDepTex2;
        param.renderSz = sharedStates->renderSz;
        renderer->SetCUDAxGLParam(param);
    }
    {
        VRRenderer::LightParam param;
        param.ka = 0.5f;
        param.kd = 0.8f;
        param.ks = 0.5f;
        param.shininess = 64.f;
        renderer->SetLightParam(param);
    }

    try {
        VolCfg cfg(std::string(PROJECT_SOURCE_DIR) + "/cfg/vol_cfg.json");
        spaces = cfg.GetSpaces();
        baseRayStep = cfg.GetBaseSpace();
        std::shared_ptr<vs::CompVolume> volume =
            vs::CompVolume::Load(cfg.GetVolumePath().c_str());
        {
            auto blockLenInfo = volume->GetBlockLength();
            blockLength = blockLenInfo[0];
            noPaddingBlockLength = blockLenInfo[0] - 2 * blockLenInfo[1];

            auto blockDimInfo = volume->GetBlockDim();
            sharedStates->blkDim = {blockDimInfo[0][0], blockDimInfo[0][1],
                                    blockDimInfo[0][2]};
            auto volSz = (float)noPaddingBlockLength * spaces *
                         glm::vec3{sharedStates->blkDim};
            hfVolSz = .5f * volSz;

            std::tie(volCubeVAO, volCubeVBO, volCubeEBO) = createCube(hfVolSz);
            positionShader->use();
            glUniform3fv(posShdrMaxPosPos, 1, &volSz[0]);
        }
        static_cast<CompVolVRRenderer *>(renderer.get())
            ->SetVolume(volume, spaces);
        renderer->SetTransferFunction(cfg.GetTFPoints());
    } catch (std::exception &ex) {
        spdlog::critical("App error: {0} caused before line: {1} in file: {2}. "
                         "App is closing.",
                         ex.what(), __LINE__, __FILE__);
        sharedStates->canRun = false;
    }

    prepareMaxVoxPos(SharedStates::MAX_SEARCH_SAMPLE_DIM,
                     SharedStates::MAX_SEARCH_MIN_SCALAR);

    pathRenderer = std::make_unique<GLPathRenderer>(
        std::string(PROJECT_SOURCE_DIR) + "/include/path");
}

static void initSignalAndSlots() {
    statefulSys->Register(
        std::tie(sharedStates->FAVRLvl, sharedStates->maxStepCnt,
                 sharedStates->antiMoireStepMult),
        [&]() {
            VRRenderer::RenderParam param;
            param.FAVRLvl = sharedStates->FAVRLvl;
            param.renderSz = sharedStates->renderSz;
            param.maxStepCnt = sharedStates->maxStepCnt;
            param.step = baseRayStep * sharedStates->antiMoireStepMult;
            renderer->SetRenderParam(param);
        });
    statefulSys->Register(
        std::tie(sharedStates->projection2, sharedStates->preScale,
                 sharedStates->eyeToHeadTranslate2),
        [&]() {
            auto minDist = sharedStates->preScale * noPaddingBlockLength;
            projParam.nearClip =
                sharedStates->preScale * SharedStates::NEAR_CLIP;
            projParam.farClip = sharedStates->preScale * SharedStates::FAR_CLIP;
            if (projParam.farClip - projParam.nearClip < minDist)
                projParam.farClip = projParam.nearClip + minDist;

            projParam.unProjection2[0] =
                Math::InverseProjective(sharedStates->projection2[0]);
            projParam.unProjection2[1] =
                Math::InverseProjective(sharedStates->projection2[1]);
            for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx) {
                projParam.projection222[eyeIdx] =
                    -(sharedStates->FAR_CLIP + sharedStates->NEAR_CLIP) /
                    (sharedStates->FAR_CLIP - sharedStates->NEAR_CLIP);
                projParam.projection223[eyeIdx] =
                    -2.f * sharedStates->preScale * sharedStates->preScale *
                    sharedStates->FAR_CLIP * sharedStates->NEAR_CLIP /
                    (sharedStates->FAR_CLIP - sharedStates->NEAR_CLIP);
            }

            CompVolVRRenderer::CamPyramidParam camPyramidParam;
            for (uint8_t i = 0; i < 4; ++i) {
                auto v4 = projParam.unProjection2[i & 0b1] *
                          glm::vec4{(i & 0b01) == 0 ? -1.f : +1.f,
                                    (i & 0b10) == 0 ? -1.f : +1.f, 1.f, 1.f};
                camPyramidParam.pos4[i] = v4;
                camPyramidParam.pos4[i] =
                    glm::normalize(camPyramidParam.pos4[i]);
                auto t = projParam.nearClip / fabsf(camPyramidParam.pos4[i].z);
                camPyramidParam.pos4[i] *= t;
                camPyramidParam.pos4[i] +=
                    sharedStates->preScale *
                    sharedStates->eyeToHeadTranslate2[i & 0b1];
            }
            for (uint8_t i = 0; i < 3; ++i)
                for (uint8_t xy = 0; xy < 2; ++xy)
                    if (auto tmp = fabsf(camPyramidParam.pos4[i][xy]);
                        camPyramidParam.pos4[3][xy] < tmp)
                        camPyramidParam.pos4[3][xy] = tmp;
            for (uint8_t i = 0; i < 3; ++i)
                camPyramidParam.pos4[i] = {
                    (i & 0b01) == 0 ? -camPyramidParam.pos4[3].x
                                    : +camPyramidParam.pos4[3].x,
                    (i & 0b10) == 0 ? -camPyramidParam.pos4[3].y
                                    : +camPyramidParam.pos4[3].y,
                    camPyramidParam.pos4[3].z};
            dynamic_cast<CompVolVRRenderer *>(renderer.get())
                ->SetCameraPyramidParam(camPyramidParam);
        },
        std::tie(projParam));
    statefulSys->Register(std::tie(projParam),
                          [&]() { renderer->SetProjectionParam(projParam); });
    statefulSys->Register(std::tie(sharedStates->preScale), [&]() {
        pathRenderer->szScale = sharedStates->preScale;
        sharedStates->preTranslateChngStep =
            sharedStates->preScale * std::max({spaces.x, spaces.y, spaces.z}) *
            noPaddingBlockLength;
    });
    statefulSys->Register(
        std::tie(sharedStates->preScale, sharedStates->preTranslate), [&]() {
            W2VR = glm::mat4(sharedStates->preScale, 0.f, 0.f, 0.f, 0.f,
                             sharedStates->preScale, 0.f, 0.f, 0.f, 0.f,
                             sharedStates->preScale, 0.f,
                             sharedStates->preTranslate.x,
                             sharedStates->preTranslate.y,
                             sharedStates->preTranslate.z, 1.f);
            auto invPreScale = 1.f / sharedStates->preScale;
            VR2W = glm::mat4(invPreScale, 0.f, 0.f, 0.f, 0.f, invPreScale, 0.f,
                             0.f, 0.f, 0.f, invPreScale, 0.f,
                             -invPreScale * sharedStates->preTranslate.x,
                             -invPreScale * sharedStates->preTranslate.y,
                             -invPreScale * sharedStates->preTranslate.z, 1.f);
            volCubeM = glm::mat4(
                invPreScale, 0.f, 0.f, 0.f, 0.f, invPreScale, 0.f, 0.f, 0.f,
                0.f, invPreScale, 0.f,
                invPreScale * (hfVolSz.x - sharedStates->preTranslate.x),
                invPreScale * (hfVolSz.y - sharedStates->preTranslate.y),
                invPreScale * (hfVolSz.z - sharedStates->preTranslate.z), 1.f);
        });
    statefulSys->Register(
        std::tie(sharedStates->preScale, sharedStates->preTranslate,
                 sharedStates->camera),
        [&]() {
            auto [R, F, U, PL, PR] = sharedStates->camera.GetRFUP2();
            camParam.headPos =
                (sharedStates->preScale * sharedStates->camera.GetHeadPos()) +
                sharedStates->preTranslate;
            camParam.eyePos2[0] =
                (sharedStates->preScale * PL) + sharedStates->preTranslate;
            camParam.eyePos2[1] =
                (sharedStates->preScale * PR) + sharedStates->preTranslate;
            camParam.rotation = {R, U, -F};

            auto invNBL = 1.f / noPaddingBlockLength;
            for (uint8_t xyz = 0; xyz < 3; ++xyz)
                sharedStates->camInBlk[xyz] =
                    (glm::uint)(camParam.headPos[xyz] * invNBL / spaces[xyz]);
        },
        std::tie(camParam));
    statefulSys->Register(std::tie(camParam),
                          [&]() { renderer->SetCameraParam(camParam); });
    statefulSys->Register(
        std::tie(sharedStates->handStates2[VRApp::PathInteractHndIdx].clicked),
        [&]() {
            if (sharedStates->IsGUIBlockInteraction())
                return;

            switch (sharedStates->interactiveMode) {
            case InteractionMode::SelectVert:
                break;
            case InteractionMode::AddPath:
                if (isInteractingPosFound()) {
                    auto &col = colTbl.NextColor();
                    auto pos = W2VR * glm::vec4{refInteractingPos(), 1.f};
                    auto id = pathRenderer->AddPath(col, pos);

                    pathRenderer->StartPath(id);

                    sharedStates->interactiveMode = InteractionMode::AddVert;
                    sharedStates->guiIntrctModePageStates.ChangeSelected(
                        sharedStates->interactiveMode);
                }
                break;
            case InteractionMode::AddVert:
                if (isInteractingPosFound()) {
                    if (pathRenderer->GetSelectedSubPathID() ==
                        GLPathRenderer::NONE) {
                        auto id = pathRenderer->AddSubPath();
                        pathRenderer->StartSubPath(id);
                    }

                    auto pos = W2VR * glm::vec4{refInteractingPos(), 1.f};
                    auto id = pathRenderer->AddVertex(pos);

                    pathRenderer->StartVertex(id);
                }
                break;
            }
        });
    statefulSys->Register(
        std::tie(sharedStates->handStates2[VRApp::PathInteractHndIdx].pressed),
        [&]() {
            if (sharedStates->IsGUIBlockInteraction())
                return;

            switch (sharedStates->interactiveMode) {
            case InteractionMode::SelectVert:
                break;
            case InteractionMode::AddVert:
                if (isInteractingPosFound()) {
                    if (pathRenderer->GetSelectedSubPathID() ==
                        GLPathRenderer::NONE)
                        return;
                    auto lastID = pathRenderer->GetSelectedVertID();
                    if (lastID == GLPathRenderer::NONE)
                        return;

                    auto pos =
                        glm::vec3{W2VR * glm::vec4{refInteractingPos(), 1.f}};
                    auto lastPos = pathRenderer->GetVertexPositions()[lastID];
                    if (glm::distance(pos, lastPos) <
                        sharedStates->preScale *
                            SharedStates::CONSECUTIVE_PATH_VERTS_MIN_DIST)
                        return;

                    auto id = pathRenderer->AddVertex(pos);

                    pathRenderer->StartVertex(id);
                }
                break;
            }
        });
    statefulSys->Register(
        std::tie(sharedStates->guiIntrctModePageStates.selectedIdx), [&]() {
            sharedStates->interactiveMode = static_cast<InteractionMode>(
                sharedStates->guiIntrctModePageStates.selectedIdx);
        });
}

static void processAppInput() {
    glfwApp->ProcessInput();
    vrApp->ProcessInput();

    statefulSys->Update();
}

static void outputToApp() {
    glfwApp->ProcessOutput();
    vrApp->ProcessOutput();
}

static void render() {
    std::array<glm::mat4, 2> VP2;
    for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx)
        VP2[eyeIdx] = sharedStates->projection2[eyeIdx] *
                      sharedStates->camera.GetViewMat(eyeIdx);

    glm::mat4 intrctCubeM = glm::identity<glm::mat4>();
    if (sharedStates->handStates2[VRApp::PathInteractHndIdx].show)
        intrctCubeM[3] = {
            interactingCubeCntrPos =
                glm::vec3{sharedStates->handStates2[VRApp::PathInteractHndIdx]
                              .pose[3]} -
                SharedStates::INTERACT_CUBE_CNTR_TO_HND_DIST *
                    glm::vec3{
                        sharedStates->handStates2[VRApp::PathInteractHndIdx]
                            .pose[2]},
            1.f};

    std::array<glm::mat4, 2> volCubeMVP2;
    std::array<glm::mat4, 2> intrctCubeMVP2;
    std::array<std::array<glm::mat4, 2>, 2> hndMVP2x2;
    for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx) {
        volCubeMVP2[eyeIdx] = VP2[eyeIdx] * volCubeM;
        intrctCubeMVP2[eyeIdx] = VP2[eyeIdx] * intrctCubeM;
        for (uint8_t hndIdx = 0; hndIdx < 2; ++hndIdx)
            hndMVP2x2[eyeIdx][hndIdx] =
                VP2[eyeIdx] * sharedStates->handStates2[hndIdx].pose;
    }

    auto shouldCalcInteractingPos = [&]() {
        return (sharedStates->interactiveMode == InteractionMode::AddPath ||
                sharedStates->interactiveMode == InteractionMode::AddVert) &&
               sharedStates->handStates2[VRApp::PathInteractHndIdx].show;
    };
    auto shouldShowInteractingPos = [&]() {
        return shouldCalcInteractingPos() && isInteractingPosFound();
    };

    static bool calculatingInteractingPos = false;
    if (shouldCalcInteractingPos()) {
        if (calculatingInteractingPos) {
            interactingPos = fetchMaxVoxPos();
            if (isCalcInteractingPosReady()) {
                glm::vec4 tmp{refInteractingPos(), 1.f};
                tmp = VR2W * tmp;
                refInteractingPos() = tmp;
                calculatingInteractingPos = false;
            }
        } else {
            glm::vec4 cubeMin{interactingCubeCntrPos, 1.f};
            glm::vec4 cubeMax{cubeMin};
            cubeMin.x -= SharedStates::INTERACT_CUBE_HF_WID;
            cubeMin.y -= SharedStates::INTERACT_CUBE_HF_WID;
            cubeMin.z -= SharedStates::INTERACT_CUBE_HF_WID;
            cubeMin = W2VR * cubeMin;
            cubeMax.x += SharedStates::INTERACT_CUBE_HF_WID;
            cubeMax.y += SharedStates::INTERACT_CUBE_HF_WID;
            cubeMax.z += SharedStates::INTERACT_CUBE_HF_WID;
            cubeMax = W2VR * cubeMax;
            execMaxVoxPos(cubeMin, cubeMax);
            calculatingInteractingPos = true;
        }
    }

    if (shouldShowInteractingPos())
        pathRenderer->PrepareExtraVertex(refInteractingPos());

    auto renderHands = [&](uint8_t eyeIdx, bool renderDep) {
        if (renderDep)
            diffuseDepShader->use();
        else
            diffuseShader->use();
        for (uint8_t hndIdx = 0; hndIdx < 2; ++hndIdx)
            if (sharedStates->handStates2[hndIdx].show) {
                glUniformMatrix4fv(renderDep ? dfDepShdrMatPos : dfShdrMatPos,
                                   1, GL_FALSE,
                                   (GLfloat *)&hndMVP2x2[eyeIdx][hndIdx]);
                sharedStates->handStates2[hndIdx].model->draw();
            }

        if (sharedStates->handStates2[VRApp::PathInteractHndIdx].show) {
            if (renderDep)
                colorDepShader->use();
            else {
                colorShader->use();
                glUniform3fv(colShdrColPos, 1,
                             &SharedStates::INTERACT_CUBE_COL[0]);
            }
            glUniformMatrix4fv(renderDep ? colDepShdrMatPos : colShdrMatPos, 1,
                               GL_FALSE, (GLfloat *)&intrctCubeMVP2[eyeIdx]);
            glBindVertexArray(intrctCubeVAO);
            glDrawElements(GL_LINES, 24, GL_UNSIGNED_SHORT, (const void *)0);
            glBindVertexArray(0);
        }
    };

    auto renderVolCube = [&](uint8_t eyeIdx, bool renderDep) {
        GL_CHECK;
        if (renderDep)
            colorDepShader->use();
        else {
            positionShader->use();
            glUniformMatrix4fv(posShdrMPos, 1, GL_FALSE, (GLfloat *)&volCubeM);
        }
        glUniformMatrix4fv(renderDep ? colDepShdrMatPos : posShdrMatPos, 1,
                           GL_FALSE, (GLfloat *)&volCubeMVP2[eyeIdx]);
        glBindVertexArray(volCubeVAO);
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_SHORT, (const void *)0);
        glBindVertexArray(0);
        GL_CHECK;
    };

    glEnable(GL_MULTISAMPLE);
    glEnable(GL_DEPTH_TEST);
    glCullFace(GL_BACK);

    // Pixels without fragments are set to FarClip
    glClearColor(1.f, 1.f, 1.f, 1.f);
    for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx) {
        glBindFramebuffer(GL_FRAMEBUFFER, rndrDepFBO2[eyeIdx]);
        glViewport(0, 0, sharedStates->renderSz.x, sharedStates->renderSz.y);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glEnable(GL_CULL_FACE);
        if (shouldShowInteractingPos())
            pathRenderer->DrawExtraVertexDepth(
                SharedStates::INTERACT_VERT_HF_WID, VP2[eyeIdx]);
        pathRenderer->DrawDepth(VR2W, VP2[eyeIdx]);
        renderHands(eyeIdx, true);

        glDisable(GL_CULL_FACE);
        renderVolCube(eyeIdx, true);
    }

    renderer->Render(sharedStates->renderTarget);

    glClearColor(0, 0, 0, 1.f); // background color

    for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx) {
        glBindFramebuffer(GL_FRAMEBUFFER, renderFBO2[eyeIdx]);
        glViewport(0, 0, sharedStates->renderSz.x, sharedStates->renderSz.y);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glEnable(GL_CULL_FACE);
        if (shouldShowInteractingPos())
            pathRenderer->DrawExtraVertex(SharedStates::INTERACT_VERT_HF_WID,
                                          VP2[eyeIdx],
                                          SharedStates::INTERACT_VERT_COL);
        pathRenderer->Draw(VR2W, VP2[eyeIdx]);
        renderHands(eyeIdx, false);

        glDisable(GL_CULL_FACE);
        renderVolCube(eyeIdx, false);
    }

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    quadShader->use();
    glBindVertexArray(quadVAO);
    for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx) {
        glBindFramebuffer(GL_FRAMEBUFFER, renderFBO2[eyeIdx]);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, volRenderTex[eyeIdx]);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, nullptr);
    }
    glBindVertexArray(0);

    glDisable(GL_MULTISAMPLE);
    glDisable(GL_BLEND);

    glBindFramebuffer(GL_FRAMEBUFFER,
                      renderFBO2[sharedStates->GetGUIRenderEyeIdx()]);
    glfwApp->RenderGUI();

    for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx) {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, renderFBO2[eyeIdx]);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, submitEyeFBO2[eyeIdx]);
        glBlitFramebuffer(0, 0, sharedStates->renderSz.x,
                          sharedStates->renderSz.y, 0, 0,
                          sharedStates->renderSz.x, sharedStates->renderSz.y,
                          GL_COLOR_BUFFER_BIT, GL_LINEAR);
    }
}

int main(int argc, char **argv) {
    statefulSys = std::make_shared<StatefulSystem>();
    sharedStates = std::make_shared<SharedStates>();

    glfwApp = std::make_unique<GLFWApp>(sharedStates, statefulSys);
    if (!sharedStates->canRun) {
        spdlog::critical("App error: GLFW or GL init failed. caused before "
                         "line: {0} in file: {1}."
                         "App is closing.",
                         __LINE__, __FILE__);
        return 1;
    }

    vrApp = std::make_unique<VRApp>(sharedStates, statefulSys);

    SetCUDACtx(0); // init CUDA context

    initRender();
    initSignalAndSlots();

    while (sharedStates->canRun) {
        processAppInput();
        render();
        outputToApp();
    }

    waitForAllVoxAlgorithms();

    return 0;
}