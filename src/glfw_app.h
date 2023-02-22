#ifndef KOUEK_GLFW_APP_H
#define KOUEK_GLFW_APP_H

#include <array>
#include <memory>

#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include <stateful/stateful.hpp>

#include "shared_states.h"

namespace kouek {
class GLFWApp {
  private:
    std::array<GLuint, 2> submitFBO2;
    glm::uvec2 submitSz;
    glm::uvec2 windowSz{1024, 512};
    GLFWwindow *window = nullptr;

    std::shared_ptr<SharedStates> sharedStates;
    std::shared_ptr<StatefulSystem> statefulSys;

  public:
    GLFWApp(std::shared_ptr<SharedStates> sharedStates,
            std::shared_ptr<StatefulSystem> statefulSys);
    ~GLFWApp();
    void SetSubmitTex2(const std::array<GLuint, 2> &FBO2,
                       const glm::uvec2 &renderSz);
    void ProcessInput();
    void ProcessOutput();

    private:
    void initSignalAndSlots();
};
} // namespace kouek

#endif // !KOUEK_GLFW_APP_H
