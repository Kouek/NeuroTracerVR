#include <cmake_in.h>

#include <limits>

#include "glfw_app.h"
#include "vr_app.h"

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
static float rayStep;
static glm::mat4 W2V;
static glm::mat4 V2W;
static uint32_t blockLength, noPaddingBlockLength;
static VRRenderer::CameraParam camParam;

static std::unique_ptr<VRRenderer> renderer;
static std::unique_ptr<GLPathRenderer> pathRenderer;

static ColorTable colTbl;

// 2 pair of FBOs for multisampling
static std::array<GLuint, 2> renderTex2;
static std::array<GLuint, 2> renderDep2;
static std::array<GLuint, 2> renderFBO2;
static std::array<GLuint, 2> submitEyeTex2;
static std::array<GLuint, 2> submitEyeDep2;
static std::array<GLuint, 2> submitEyeFBO2;

// framebuffer's depth buffer format is not supported in CUDA,
// an RGAB8 format one is needed
static std::array<GLuint, 2> rndrDepTex2;
static std::array<GLuint, 2> rndrDepDep2;
static std::array<GLuint, 2> rndrDepFBO2;

static std::array<GLuint, 2> fromRendererEyeTex2;

static GLuint quadVAO, quadVBO, quadEBO;
static std::unique_ptr<Shader> quadShader, diffuseShader, diffuseDepShader;
static GLint dfShdrMatPos, diffuseDepShdrMatPos;

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
        fromRendererEyeTex2[eyeIdx] = createPlainTexture(
            sharedStates->renderSz.x, sharedStates->renderSz.y);
    }
    if (sharedStates->canVRRun)
        vrApp->SetSubmitTex2(submitEyeTex2);

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
        (std::string(PROJECT_SOURCE_DIR) + "/src/shader/diffuse_dep.fs")
            .c_str());
    diffuseDepShdrMatPos = glGetUniformLocation(diffuseDepShader->ID, "matrix");
    assert(diffuseDepShdrMatPos != -1);

    glfwApp->SetSubmitTex2(submitEyeFBO2, sharedStates->renderSz);

    {
        CompVolVRRenderer::RendererParam param;
        param.texUnitNum = 6;
        param.texUnitShape = {1024, 1024, 1024};
        renderer = std::make_unique<CompVolVRRenderer>(param);
    }
    {
        VRRenderer::CUDAxGLParam param;
        param.outputTex2 = fromRendererEyeTex2;
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
        rayStep = cfg.GetBaseSpace() * .3f;
        std::shared_ptr<vs::CompVolume> volume =
            vs::CompVolume::Load(cfg.GetVolumePath().c_str());
        {
            auto blockLenInfo = volume->GetBlockLength();
            blockLength = blockLenInfo[0];
            noPaddingBlockLength = blockLenInfo[0] - 2 * blockLenInfo[1];
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

    pathRenderer = std::make_unique<GLPathRenderer>(
        std::string(PROJECT_SOURCE_DIR) + "/include/path");
}

static void initSignalAndSlots() {
    statefulSys->Register(std::tie(sharedStates->scaleW2V), [&]() {
        spdlog::info("App info: max step count is set to {0}",
                     sharedStates->scaleW2V);
    });
    statefulSys->Register(
        std::tie(sharedStates->FAVRLvl, sharedStates->maxStepCnt), [&]() {
            VRRenderer::RenderParam param;
            param.FAVRLvl = sharedStates->FAVRLvl;
            param.renderSz = sharedStates->renderSz;
            param.maxStepCnt = sharedStates->maxStepCnt;
            param.step = rayStep;
            renderer->SetRenderParam(param);
            spdlog::info("App info: max step count is set to {0}, FAVR level "
                         "is set to {1}.",
                         sharedStates->maxStepCnt, sharedStates->FAVRLvl);
        });
    statefulSys->Register(
        std::tie(sharedStates->projection2, sharedStates->scaleW2V), [&]() {
            VRRenderer::ProjectionParam param;
            param.unProjection2[0] =
                Math::InverseProjective(sharedStates->projection2[0]);
            param.unProjection2[1] =
                Math::InverseProjective(sharedStates->projection2[1]);
            for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx) {
                param.projection22[eyeIdx] =
                    -(sharedStates->FAR_CLIP + sharedStates->NEAR_CLIP) /
                    (sharedStates->FAR_CLIP - sharedStates->NEAR_CLIP);
                param.projection23[eyeIdx] =
                    -2.f * sharedStates->scaleW2V * sharedStates->scaleW2V *
                    sharedStates->FAR_CLIP * sharedStates->NEAR_CLIP /
                    (sharedStates->FAR_CLIP - sharedStates->NEAR_CLIP);
            }
            renderer->SetProjectionParam(param);
        });
    statefulSys->Register(
        std::tie(sharedStates->scaleW2V, sharedStates->translateW2V), [&]() {
            W2V = glm::scale(glm::translate(glm::identity<glm::mat4>(),
                                            sharedStates->translateW2V),
                             glm::vec3{sharedStates->scaleW2V});
            V2W = glm::translate(
                glm::scale(glm::identity<glm::mat4>(),
                           glm::vec3{1.f / sharedStates->scaleW2V}),
                -sharedStates->translateW2V);
        });
    statefulSys->Register(
        std::tie(sharedStates->scaleW2V, sharedStates->translateW2V,
                 sharedStates->camera),
        [&]() {
            auto [R, F, U, PL, PR] = sharedStates->camera.GetRFUP2();
            camParam.pos2[0] =
                (sharedStates->scaleW2V * PL) + sharedStates->translateW2V;
            camParam.pos2[1] =
                (sharedStates->scaleW2V * PR) + sharedStates->translateW2V;
            camParam.rotation = {R, U, -F};
        },
        std::tie(camParam));
    statefulSys->Register(
        std::tie(sharedStates->scaleW2V),
        [&]() {
            camParam.OBBBorderDistToEyeCntr =
                sharedStates->scaleW2V * SharedStates::NEAR_CLIP;
            camParam.OBBHfSz.x = .25f * noPaddingBlockLength * spaces.x;
            camParam.OBBHfSz.y = .25f * noPaddingBlockLength * spaces.y;
            camParam.OBBHfSz.z = .125f * noPaddingBlockLength * spaces.z;
        },
        std::tie(camParam));
    statefulSys->Register(std::tie(camParam),
                          [&]() { renderer->SetCameraParam(camParam); });
    statefulSys->Register(
        std::tie(sharedStates->handStates2[VRApp::PathInteractHndIdx].clicked),
        [&]() {
            switch (sharedStates->interactiveMode) {
            case InteractionMode::SelectVert:
                break;
            case InteractionMode::AddPath: {
                auto &col = colTbl.NextColor();
                auto pos =
                    W2V * sharedStates->handStates2[VRApp::PathInteractHndIdx]
                              .pose[3];
                auto id = pathRenderer->AddPath(col, pos);

                pathRenderer->StartPath(id);
                sharedStates->interactiveMode = InteractionMode::AddVert;
            } break;
            case InteractionMode::AddVert: {
                if (pathRenderer->GetSelectedSubPathID() ==
                    GLPathRenderer::NONE) {
                    auto id = pathRenderer->AddSubPath();
                    pathRenderer->StartSubPath(id);
                }

                auto pos =
                    W2V * sharedStates->handStates2[VRApp::PathInteractHndIdx]
                              .pose[3];
                auto id = pathRenderer->AddVertex(pos);
                pathRenderer->StartVertex(id);
            } break;
            }
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
    pathRenderer->PrepareExtraVertex(
        sharedStates->handStates2[VRApp::PathInteractHndIdx].pose[3]);

    std::array<glm::mat4, 2> VP2;
    for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx)
        VP2[eyeIdx] = sharedStates->projection2[eyeIdx] *
                      sharedStates->camera.GetViewMat(eyeIdx);
    std::array<std::array<glm::mat4, 2>, 2> hndMVP2x2;
    for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx)
        for (uint8_t hndIdx = 0; hndIdx < 2; ++hndIdx)
            hndMVP2x2[eyeIdx][hndIdx] =
                VP2[eyeIdx] * sharedStates->handStates2[hndIdx].pose;

    auto renderHands = [&](uint8_t eyeIdx, GLint matPos) {
        for (uint8_t hndIdx = 0; hndIdx < 2; ++hndIdx)
            if (sharedStates->handStates2[hndIdx].show) {
                glUniformMatrix4fv(matPos, 1, GL_FALSE,
                                   (GLfloat *)&hndMVP2x2[eyeIdx][hndIdx]);
                sharedStates->handStates2[hndIdx].model->draw();
            }
    };

    glEnable(GL_MULTISAMPLE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glClearColor(1.f, 1.f, 1.f, 1.f); // area without fragments set to FarClip
    for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx) {
        glBindFramebuffer(GL_FRAMEBUFFER, rndrDepFBO2[eyeIdx]);
        glViewport(0, 0, sharedStates->renderSz.x, sharedStates->renderSz.y);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        pathRenderer->DrawExtraVertexDepth(GLPathRenderer::selectedVertSize,
                                           VP2[eyeIdx]);
        pathRenderer->DrawDepth(V2W, VP2[eyeIdx]);

        diffuseDepShader->use();
        renderHands(eyeIdx, diffuseDepShdrMatPos);
    }

    renderer->Render(sharedStates->renderTarget);

    glClearColor(0, 0, 0, 1.f); // background

    for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx) {
        glBindFramebuffer(GL_FRAMEBUFFER, renderFBO2[eyeIdx]);
        glViewport(0, 0, sharedStates->renderSz.x, sharedStates->renderSz.y);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        pathRenderer->DrawExtraVertex(GLPathRenderer::selectedVertSize,
                                      VP2[eyeIdx],
                                      GLPathRenderer::selectedVertColor);
        pathRenderer->Draw(V2W, VP2[eyeIdx]);

        diffuseShader->use();
        renderHands(eyeIdx, dfShdrMatPos);
    }

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    quadShader->use();
    glBindVertexArray(quadVAO);
    for (uint8_t eyeIdx = 0; eyeIdx < 2; ++eyeIdx) {
        glBindFramebuffer(GL_FRAMEBUFFER, renderFBO2[eyeIdx]);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, fromRendererEyeTex2[eyeIdx]);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, nullptr);
    }
    glBindVertexArray(0);

    glDisable(GL_MULTISAMPLE);
    glDisable(GL_BLEND);

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
    sharedStates = std::make_shared<SharedStates>(*statefulSys.get());

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

    return 0;
}