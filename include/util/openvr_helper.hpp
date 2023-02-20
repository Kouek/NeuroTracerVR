#ifndef KOUEK_OPEN_VR_HELPER
#define KOUEK_OPEN_VR_HELPER

#include <functional>
#include <stdexcept>
#include <thread>

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <openvr.h>

namespace kouek {

static inline glm::mat4x3
SteamVRMat34ToGLMMat43(const vr::HmdMatrix34_t &stMat34) {
    return {stMat34.m[0][0], stMat34.m[1][0], stMat34.m[2][0], stMat34.m[0][1],
            stMat34.m[1][1], stMat34.m[2][1], stMat34.m[0][2], stMat34.m[1][2],
            stMat34.m[2][2], stMat34.m[0][3], stMat34.m[1][3], stMat34.m[2][3]};
}

static inline glm::mat4
SteamVRMat34ToGLMMat4(const vr::HmdMatrix34_t &stMat34) {
    return {stMat34.m[0][0], stMat34.m[1][0], stMat34.m[2][0], 0,
            stMat34.m[0][1], stMat34.m[1][1], stMat34.m[2][1], 0,
            stMat34.m[0][2], stMat34.m[1][2], stMat34.m[2][2], 0,
            stMat34.m[0][3], stMat34.m[1][3], stMat34.m[2][3], 1.f};
}

static inline glm::mat4
SteamVRMat44ToGLMMat4(const vr::HmdMatrix44_t &stMat44) {
    return {stMat44.m[0][0], stMat44.m[1][0], stMat44.m[2][0], stMat44.m[3][0],
            stMat44.m[0][1], stMat44.m[1][1], stMat44.m[2][1], stMat44.m[3][1],
            stMat44.m[0][2], stMat44.m[1][2], stMat44.m[2][2], stMat44.m[3][2],
            stMat44.m[0][3], stMat44.m[1][3], stMat44.m[2][3], stMat44.m[3][3]};
}

static inline std::tuple<vr::RenderModel_t *, vr::RenderModel_TextureMap_t *>
GetRenderModelAndTex(const char *modelName) {
    vr::RenderModel_t *model = nullptr;
    vr::EVRRenderModelError error;
    while (true) {
        error = vr::VRRenderModels()->LoadRenderModel_Async(modelName, &model);
        if (error != vr::VRRenderModelError_Loading)
            break;
        // block until loading model finished
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (error != vr::VRRenderModelError_None)
        throw std::runtime_error(
            std::string("Load render model FAILED: Model Name: ") + modelName +
            ", " +
            vr::VRRenderModels()->GetRenderModelErrorNameFromEnum(error));

    vr::RenderModel_TextureMap_t *tex;
    while (1) {
        error = vr::VRRenderModels()->LoadTexture_Async(model->diffuseTextureId,
                                                        &tex);
        if (error != vr::VRRenderModelError_Loading)
            break;
        // block until loading texture finished
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (error != vr::VRRenderModelError_None)
        throw std::runtime_error(
            std ::string("Load render model's texture FAILED: Model Name: ") +
            modelName +
            ", Texture ID: " + std::to_string(model->diffuseTextureId) + ", " +
            vr::VRRenderModels()->GetRenderModelErrorNameFromEnum(error));
    return {model, tex};
}

static inline std::string
GetTrackedDeviceString(vr::TrackedDeviceIndex_t unDevice,
                       vr::TrackedDeviceProperty prop,
                       vr::TrackedPropertyError *peError = NULL) {
    uint32_t unRequiredBufferLen =
        vr::VRSystem()->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0,
                                                       peError);
    if (unRequiredBufferLen == 0)
        return "";

    std::string sResult(unRequiredBufferLen, '\0');
    unRequiredBufferLen = vr::VRSystem()->GetStringTrackedDeviceProperty(
        unDevice, prop, sResult.data(), unRequiredBufferLen, peError);
    return sResult;
}

class VRRenderModel {
  private:
    GLuint VBO = 0;
    GLuint EBO = 0;
    GLuint VAO = 0;
    GLuint tex = 0;
    GLsizei vertCnt = 0;

  public:
    VRRenderModel(const vr::RenderModel_t &vrModel,
                  const vr::RenderModel_TextureMap_t &vrDiffuseTexture) {
        glGenVertexArrays(1, &VAO);
        glBindVertexArray(VAO);
        {
            glGenBuffers(1, &VBO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER,
                         sizeof(vr::RenderModel_Vertex_t) *
                             vrModel.unVertexCount,
                         vrModel.rVertexData, GL_STATIC_DRAW);

            glEnableVertexAttribArray(0);
            glVertexAttribPointer(
                0, 3, GL_FLOAT, GL_FALSE, sizeof(vr::RenderModel_Vertex_t),
                (void *)offsetof(vr::RenderModel_Vertex_t, vPosition));
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(
                1, 3, GL_FLOAT, GL_FALSE, sizeof(vr::RenderModel_Vertex_t),
                (void *)offsetof(vr::RenderModel_Vertex_t, vNormal));
            glEnableVertexAttribArray(2);
            glVertexAttribPointer(
                2, 2, GL_FLOAT, GL_FALSE, sizeof(vr::RenderModel_Vertex_t),
                (void *)offsetof(vr::RenderModel_Vertex_t, rfTextureCoord));

            glGenBuffers(1, &EBO);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                         sizeof(GLushort) * vrModel.unTriangleCount * 3,
                         vrModel.rIndexData, GL_STATIC_DRAW);
        }
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, vrDiffuseTexture.unWidth,
                         vrDiffuseTexture.unHeight, 0, GL_RGBA,
                         GL_UNSIGNED_BYTE, vrDiffuseTexture.rubTextureMapData);

            glGenerateMipmap(GL_TEXTURE_2D);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                            GL_LINEAR_MIPMAP_LINEAR);

            GLfloat largest;
            glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &largest);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT,
                            largest);
        }
        glBindTexture(GL_TEXTURE_2D, 0);

        vertCnt = vrModel.unTriangleCount * 3;
    }
    VRRenderModel(const VRRenderModel &) = delete;
    ~VRRenderModel() {
        if (VAO != 0)
            glDeleteVertexArrays(1, &VAO);
        if (VBO != 0)
            glDeleteBuffers(1, &VBO);
        if (EBO != 0)
            glDeleteBuffers(1, &EBO);
        if (tex != 0)
            glDeleteTextures(1, &tex);
    }
    void draw() {
        glBindVertexArray(VAO);
        {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, tex);
            glDrawElements(GL_TRIANGLES, vertCnt, GL_UNSIGNED_SHORT,
                           (const void *)0);
            glBindTexture(GL_TEXTURE_2D, 0);
        }
        glBindVertexArray(0);
    }
};

} // namespace kouek
#endif // !KOUEK_OPEN_VR_HELPER
