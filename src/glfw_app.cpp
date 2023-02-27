#include "glfw_app.h"

#include <imgui/imgui_impl_opengl3.h>

#include <spdlog/spdlog.h>

using namespace kouek;

static SharedStates *sharedStatesPtr = nullptr;
static glm::uvec2 *windowSzPtr = nullptr;
static int glbKey, glbAction;

static void keyCallback(GLFWwindow *window, int key, int scancode, int action,
                        int mods) {
    switch (key) {
    case GLFW_KEY_ESCAPE:
        if (action == GLFW_RELEASE)
            sharedStatesPtr->canRun = false;
        break;
    default:
        glbAction = action;
        glbKey = key;
    }
}

static void windowSizeCallback(GLFWwindow *window, int width, int height) {
    windowSzPtr->x = width;
    windowSzPtr->y = height;
}

kouek::GLFWApp::GLFWApp(std::shared_ptr<SharedStates> sharedStates,
                        std::shared_ptr<StatefulSystem> statefulSys)
    : sharedStates(sharedStates), statefulSys(statefulSys) {
    sharedStatesPtr = sharedStates.get();
    windowSzPtr = &windowSz;

    // Create GLFW context
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(windowSz.x, windowSz.y, "Neuron Tracer", nullptr,
                              nullptr);
    if (window == NULL) {
        sharedStates->canRun = false;
        return;
    }

    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // Load GL functions
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        sharedStates->canRun = false;
        return;
    }

    // Init ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplOpenGL3_Init();

    initSignalAndSlots();
}

kouek::GLFWApp::~GLFWApp() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();
}

void kouek::GLFWApp::SetSubmitTex2(const std::array<GLuint, 2> &FBO2,
                                   const glm::uvec2 &renderSz) {
    submitFBO2 = FBO2;
    submitSz = renderSz;

    guiSz.x = (float)((renderSz.x >> 3) + (renderSz.x >> 4));
    guiSz.y = (float)((renderSz.y >> 3) + (renderSz.y >> 4));
    guiPos.x = (float)(renderSz.x >> 1) - guiSz.x * .5f;
    guiPos.y = (float)(renderSz.y >> 1) - guiSz.y * .5f;

    ImGui::GetIO().DisplaySize = ImVec2(renderSz.x, renderSz.y);
}

void kouek::GLFWApp::ProcessInput() {
    glfwPollEvents();
    if (glfwWindowShouldClose(window) != GLFW_FALSE)
        sharedStates->canRun = false;
}

void kouek::GLFWApp::ProcessOutput() {
    glm::uint halfWid = windowSz.x >> 1;
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, submitFBO2[0]);
    glBlitFramebuffer(0, 0, submitSz.x, submitSz.y, 0, 0, halfWid, windowSz.y,
                      GL_COLOR_BUFFER_BIT, GL_LINEAR);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, submitFBO2[1]);
    glBlitFramebuffer(0, 0, submitSz.x, submitSz.y, halfWid, 0, windowSz[0],
                      windowSz[1], GL_COLOR_BUFFER_BIT, GL_LINEAR);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

    glfwSwapBuffers(window);
}

void kouek::GLFWApp::RenderGUI() {
    if (sharedStates->guiPage == GUIPage::None)
        return;

    ImGui_ImplOpenGL3_NewFrame();
    ImGui::NewFrame();

    auto biasPos = [&]() {
        // According to the feature of VR display device,
        // GUI rendered in left eye should has a right bias,
        // while the one rendered in right eye should has a left bias.
        auto ret(guiPos);
        switch (sharedStates->guiPage) {
        case GUIPage::IntrctModePage:
            ret.x -= 50.f;
            break;
        }
        return ret;
    }();

    ImGui::SetNextWindowPos(biasPos);
    ImGui::SetNextWindowSize(guiSz);
    ImGui::SetNextWindowBgAlpha(1.f);

    if (!ImGui::Begin(GUIPageNames[static_cast<uint8_t>(sharedStates->guiPage)],
                      false, ImGuiWindowFlags_NoResize))
        goto TERMINAL;

    auto contentSz = [&]() {
        auto tmp = ImGui::GetWindowContentRegionMax();
        auto min = ImGui::GetWindowContentRegionMin();
        tmp.x -= min.x;
        tmp.y -= min.y;
        return std::min(tmp.x, tmp.y);
    }();
    auto drawIntrctModePage = [&]() {
        ImVec2 btnSz{contentSz / GUIIntrctModePageStates::COL_NUM,
                     contentSz / GUIIntrctModePageStates::ROW_NUM};

        ImGui::PushStyleVar(ImGuiStyleVar_SelectableTextAlign,
                            ImVec2(0.5f, 0.5f));
        for (uint8_t row = 0; row < GUIIntrctModePageStates::ROW_NUM; ++row)
            for (uint8_t col = 0; col < GUIIntrctModePageStates::COL_NUM;
                 ++col) {
                uint8_t id = row * GUIIntrctModePageStates::COL_NUM + col;
                if (id >= static_cast<uint8_t>(InteractionMode::End))
                    break;
                if (col > 0)
                    ImGui::SameLine();
                ImGui::Selectable(
                    InteractionModeNames[id],
                    sharedStates->guiIntrctModePageStates.selecteds[id], 0,
                    btnSz);
            }
        ImGui::PopStyleVar();
    };

    switch (sharedStates->guiPage) {
    case GUIPage::IntrctModePage:
        drawIntrctModePage();
        break;
    }

TERMINAL:
    ImGui::End();

    ImGui::EndFrame();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void kouek::GLFWApp::initSignalAndSlots() {
    // This function map GLFW inputs to sharedStates

    statefulSys->Register(
        [&]() {
            switch (glbKey) {
            case GLFW_KEY_HOME:
                if (glbAction == GLFW_RELEASE &&
                    static_cast<uint8_t>(sharedStates->renderTarget) != 0)
                    sharedStates->renderTarget =
                        static_cast<VRRenderer::RenderTarget>(
                            static_cast<uint8_t>(sharedStates->renderTarget) -
                            1);
                break;
            case GLFW_KEY_END:
                if (glbAction == GLFW_RELEASE &&
                    static_cast<uint8_t>(sharedStates->renderTarget) !=
                        static_cast<uint8_t>(VRRenderer::RenderTarget::End) - 1)
                    sharedStates->renderTarget =
                        static_cast<VRRenderer::RenderTarget>(
                            static_cast<uint8_t>(sharedStates->renderTarget) +
                            1);
                break;
            }
            glbAction = glbKey = -1;
        },
        std::tie(sharedStates->renderTarget));
}
