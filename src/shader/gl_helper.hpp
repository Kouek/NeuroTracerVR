#ifndef KOUEK_GL_HELPER_H
#define KOUEK_GL_HELPER_H

#include <tuple>

#include <glad/glad.h>

#include <spdlog/spdlog.h>

#define GL_CHECK                                                               \
    {                                                                          \
        GLenum gl_err;                                                         \
        if ((gl_err = glGetError()) != GL_NO_ERROR) {                          \
            spdlog::error(                                                     \
                "OpenGL error: {0} caused before line {1} of file:{2}",        \
                static_cast<unsigned int>(gl_err), __LINE__, __FILE__);        \
        }                                                                      \
    }

namespace kouek {
static GLuint createPlainTexture(uint32_t w, uint32_t h) {
    GLuint tex;
    glGenTextures(1, &tex);

    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 (const void *)0);
    glBindTexture(GL_TEXTURE_2D, 0);

    return tex;
}

inline static std::tuple<GLuint, GLuint, GLuint>
createFrambuffer(uint32_t w, uint32_t h, bool useMultiSample = false) {
    GLuint FBO, colorTex, depthRBO;

    glGenFramebuffers(1, &FBO);
    glGenTextures(1, &colorTex);
    glGenRenderbuffers(1, &depthRBO);

    if (useMultiSample) {
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, colorTex);
        glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_RGBA8, w, h,
                                true);
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);

        glBindRenderbuffer(GL_RENDERBUFFER, depthRBO);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH_COMPONENT,
                                         w, h);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);

        glBindFramebuffer(GL_FRAMEBUFFER, FBO);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D_MULTISAMPLE, colorTex, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                  GL_RENDERBUFFER, depthRBO);
    } else {
        colorTex = createPlainTexture(w, h);

        glBindRenderbuffer(GL_RENDERBUFFER, depthRBO);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, w, h);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);

        glBindFramebuffer(GL_FRAMEBUFFER, FBO);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, colorTex, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                  GL_RENDERBUFFER, depthRBO);
    }

    int FBOStatus = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    assert(FBOStatus == GL_FRAMEBUFFER_COMPLETE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return {FBO, colorTex, depthRBO};
}

inline static std::tuple<GLuint, GLuint, GLuint> createQuad() {
    GLuint VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    {
        std::array<std::array<GLfloat, 5>, 4> verts{
            std::array{-1.f, -1.f, +0.f, +0.f, +0.f},
            std::array{+1.f, -1.f, +0.f, +1.f, +0.f},
            std::array{-1.f, +1.f, +0.f, +0.f, +1.f},
            std::array{+1.f, +1.f, +0.f, +1.f, +1.f}};

        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(GLfloat) * verts[0].size() * verts.size(),
                     verts.data()->data(), GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                              sizeof(GLfloat) * verts[0].size(),
                              (const void *)(0));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE,
                              sizeof(GLfloat) * verts[0].size(),
                              (const void *)(sizeof(GLfloat) * 3));
    }

    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    {
        GLushort indices[] = {0, 1, 3, 0, 3, 2};
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLushort) * 6, indices,
                     GL_STATIC_DRAW);
    }
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    return {VAO, VBO, EBO};
}

inline static std::tuple<GLuint, GLuint, GLuint>
createCubeFrame(GLfloat cubeHfWid) {
    GLuint VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    {
        std::array<std::array<GLfloat, 3>, 8> verts;
        for (uint8_t k = 0; k < 8; ++k)
            verts[k] = {(k & 0x1) == 0 ? -cubeHfWid : +cubeHfWid,
                        (k & 0x2) == 0 ? -cubeHfWid : +cubeHfWid,
                        (k & 0x4) == 0 ? -cubeHfWid : +cubeHfWid};

        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(GLfloat) * verts[0].size() * verts.size(),
                     verts.data()->data(), GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                              sizeof(GLfloat) * verts[0].size(),
                              (const void *)(0));
    }

    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    {
        std::array<GLushort, 24> indices{// -Z
                                         0, 1, 1, 3, 3, 2, 2, 0,
                                         // +Z
                                         4, 5, 5, 7, 7, 6, 6, 4,
                                         // other 4 edges
                                         0, 4, 1, 5, 2, 6, 3, 7};
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLushort) * indices.size(),
                     indices.data(), GL_STATIC_DRAW);
    }
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    return {VAO, VBO, EBO};
}

inline static std::tuple<GLuint, GLuint, GLuint>
createCube(const glm::vec3 &hdSz3) {
    GLuint VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    {
        std::array<std::array<GLfloat, 3>, 8> verts;
        for (uint8_t k = 0; k < 8; ++k)
            verts[k] = {(k & 0x1) == 0 ? -hdSz3.x : +hdSz3.x,
                        (k & 0x2) == 0 ? -hdSz3.y : +hdSz3.y,
                        (k & 0x4) == 0 ? -hdSz3.z : +hdSz3.z};

        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(GLfloat) * verts[0].size() * verts.size(),
                     verts.data()->data(), GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                              sizeof(GLfloat) * verts[0].size(),
                              (const void *)(0));
    }

    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    {
        std::array<GLushort, 36> indices{// -Z
                                         1, 0, 2, 2, 3, 1,
                                         // +Z
                                         4, 5, 7, 7, 6, 4,
                                         // -Y
                                         0, 1, 5, 5, 4, 0,
                                         // +Y
                                         6, 7, 3, 3, 2, 6,
                                         // -X
                                         0, 4, 6, 6, 2, 0,
                                         // +X
                                         5, 1, 3, 3, 7, 5};
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLushort) * indices.size(),
                     indices.data(), GL_STATIC_DRAW);
    }
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    return {VAO, VBO, EBO};
}

} // namespace kouek

#endif // !KOUEK_GL_HELPER_H
