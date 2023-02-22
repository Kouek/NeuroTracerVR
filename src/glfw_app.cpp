#include "glfw_app.h"

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

    initSignalAndSlots();
}

kouek::GLFWApp::~GLFWApp() { glfwTerminate(); }

void kouek::GLFWApp::SetSubmitTex2(const std::array<GLuint, 2> &FBO2,
                                   const glm::uvec2 &renderSz) {
    submitFBO2 = FBO2;
    submitSz = renderSz;
}

void kouek::GLFWApp::ProcessInput() {
    glfwPollEvents();
    if (glfwWindowShouldClose(window) != GLFW_FALSE)
        sharedStates->canRun = false;
}

void kouek::GLFWApp::ProcessOutput() {
    auto hfW = windowSz.x / 2;
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, submitFBO2[0]);
    glBlitFramebuffer(0, 0, submitSz.x, submitSz.y, 0, 0, hfW, windowSz.y,
                      GL_COLOR_BUFFER_BIT, GL_LINEAR);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, submitFBO2[1]);
    glBlitFramebuffer(0, 0, submitSz.x, submitSz.y, hfW, 0, windowSz[0],
                      windowSz[1], GL_COLOR_BUFFER_BIT, GL_LINEAR);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

    glfwSwapBuffers(window);
}

void kouek::GLFWApp::initSignalAndSlots() {
    // This function map GLFW inputs to sharedStates

    statefulSys->Register(
        [&]() {
            sharedStates->renderTarget =
                glbKey == GLFW_KEY_END
                    ? static_cast<VRRenderer::RenderTarget>(
                          static_cast<uint8_t>(sharedStates->renderTarget) + 1)
                    : static_cast<VRRenderer::RenderTarget>(
                          static_cast<uint8_t>(sharedStates->renderTarget) - 1);
            glbAction = glbKey = -1;
        },
        std::tie(sharedStates->renderTarget),
        [&]() {
            return glbAction == GLFW_RELEASE &&
                   ((glbKey == GLFW_KEY_END &&
                     static_cast<uint8_t>(sharedStates->renderTarget) !=
                         static_cast<uint8_t>(VRRenderer::RenderTarget::End) -
                             1) ||
                    (glbKey == GLFW_KEY_HOME &&
                     static_cast<uint8_t>(sharedStates->renderTarget) != 0));
        });
}
