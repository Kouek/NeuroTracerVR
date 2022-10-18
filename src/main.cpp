#include <cmake_in.h>

#include "renderer/comp/comp_vol_vr_renderer.h"
#include "shader/gl_helper.hpp"

#include "qt_app.h"
#include "vr_app.h"

#include <spdlog/spdlog.h>

#include <util/math.hpp>
#include <util/shader.hpp>
#include <util/vol_cfg.hpp>

using namespace kouek;

static std::shared_ptr<StatefulSystem> statefulSys;
static std::shared_ptr<SharedStates> sharedStates;

static std::unique_ptr<VRApp> vrApp;
static std::unique_ptr<QtApp> qtApp;

static glm::vec3 spaces;
static float rayStep;
static uint32_t blockLength, noPaddingBlockLength;
static VRRenderer::CameraParam camParam;
static std::unique_ptr<VRRenderer> renderer;

// 2 pair of FBOs for multisampling
static std::array<GLuint, 2> eyeTex2;
static std::array<GLuint, 2> eyeDep2;
static std::array<GLuint, 2> eyeFBO2;
static std::array<GLuint, 2> submitEyeTex2;
static std::array<GLuint, 2> submitEyeDep2;
static std::array<GLuint, 2> submitEyeFBO2;

static std::array<GLuint, 2> fromRendererEyeTex2;

static GLuint quadVAO, quadVBO, quadEBO;
static std::unique_ptr<Shader> quadShader;

static void initRender() {
    for (uint8_t lr = 0; lr < 2; ++lr) {
        std::tie(eyeFBO2[lr], eyeTex2[lr], eyeDep2[lr]) = createFrambuffer(
            sharedStates->renderSz.x, sharedStates->renderSz.y, true);
        std::tie(submitEyeFBO2[lr], submitEyeTex2[lr], submitEyeDep2[lr]) =
            createFrambuffer(sharedStates->renderSz.x, sharedStates->renderSz.y);
        fromRendererEyeTex2[lr] = createPlainTexture(sharedStates->renderSz.x,
                                                     sharedStates->renderSz.y);
    }
    if (sharedStates->canVRRun)
        vrApp->SetSubmitTex2(submitEyeTex2);

    std::tie(quadVAO, quadVBO, quadEBO) = createQuad();
    quadShader = std::make_unique<Shader>(
        (std::string(PROJECT_SOURCE_DIR) + "/src/shader/quad.vs").c_str(),
        (std::string(PROJECT_SOURCE_DIR) + "/src/shader/quad.fs").c_str());

    qtApp->SetOutputFBO2(submitEyeFBO2, sharedStates->renderSz);

    {
        CompVolVRRenderer::RendererParam param;
        param.texUnitNum = 6;
        param.texUnitShape = {1024, 1024, 1024};
        renderer = std::make_unique<CompVolVRRenderer>(param);
    }
    {
        VRRenderer::CUDAxGLParam param;
        param.outputTex2 = fromRendererEyeTex2;
        param.inputDep2 = eyeDep2;
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
}

static void initSignalAndSlots() {
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
        std::tie(sharedStates->preScale, sharedStates->projection2), [&]() {
            VRRenderer::ProjectionParam param;
            param.unProjection2[0] =
                Math::InverseProjective(sharedStates->projection2[0]);
            param.unProjection2[1] =
                Math::InverseProjective(sharedStates->projection2[1]);
            renderer->SetProjectionParam(param);
        });
    statefulSys->Register(
        std::tie(sharedStates->preScale, sharedStates->preTranslate,
                 sharedStates->camera),
        [&]() {
            auto [R, F, U, PL, PR] = sharedStates->camera.GetRFUP2();
            camParam.pos2[0] =
                (sharedStates->preScale * PL) + sharedStates->preTranslate;
            camParam.pos2[1] =
                (sharedStates->preScale * PR) + sharedStates->preTranslate;
            camParam.rotation = {R, U, -F};
        },
        std::tie(camParam));
    statefulSys->Register(
        std::tie(sharedStates->preScale),
        [&]() {
            camParam.OBBBorderDistToEyeCntr =
                sharedStates->preScale * SharedStates::NEAR_CLIP;
            camParam.OBBHfSz.x = .25f * noPaddingBlockLength * spaces.x;
            camParam.OBBHfSz.y = .25f * noPaddingBlockLength * spaces.y;
            camParam.OBBHfSz.z = .125f * noPaddingBlockLength * spaces.z;
        },
        std::tie(camParam));
    statefulSys->Register(std::tie(camParam),
                          [&]() { renderer->SetCameraParam(camParam); });
}

static void processAppInput() {
    qtApp->ProcessInput();
    vrApp->ProcessInput();

    statefulSys->Update();
}

static void outputToApp() {
    qtApp->ProcessOutput();
    vrApp->ProcessOutput();
}

static void render() {
    glEnable(GL_MULTISAMPLE);

    glClearColor(.1f, .2f, 1.f, 1.f);
    for (uint8_t lr = 0; lr < 2; ++lr) {
        glBindFramebuffer(GL_FRAMEBUFFER, eyeFBO2[lr]);
        glViewport(0, 0, sharedStates->renderSz.x, sharedStates->renderSz.y);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    renderer->Render(sharedStates->renderTarget);

    quadShader->use();
    glBindVertexArray(quadVAO);
    for (uint8_t lr = 0; lr < 2; ++lr) {
        glBindFramebuffer(GL_FRAMEBUFFER, eyeFBO2[lr]);
        glBindTexture(GL_TEXTURE_2D, fromRendererEyeTex2[lr]);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, nullptr);
    }

    glDisable(GL_MULTISAMPLE);

    for (uint8_t lr = 0; lr < 2; ++lr) {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, eyeFBO2[lr]);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, submitEyeFBO2[lr]);
        glBlitFramebuffer(0, 0, sharedStates->renderSz.x,
                          sharedStates->renderSz.y, 0, 0,
                          sharedStates->renderSz.x, sharedStates->renderSz.y,
                          GL_COLOR_BUFFER_BIT, GL_LINEAR);
    }
}

int main(int argc, char **argv) {
    statefulSys = std::make_shared<StatefulSystem>();
    sharedStates = std::make_shared<SharedStates>(*statefulSys.get());
    vrApp = std::make_unique<VRApp>(sharedStates, statefulSys);
    qtApp = std::make_unique<QtApp>(argc, argv, sharedStates, statefulSys);

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