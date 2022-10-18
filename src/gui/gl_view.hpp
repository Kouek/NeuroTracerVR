#ifndef KOUEK_GL_VIEW_H
#define KOUEK_GL_VIEW_H

#include <array>
#include <cassert>

#include <glad/glad.h>

#include <QtGui/qevent.h>
#include <QtWidgets/qopenglwidget.h>

#include <glm/gtc/matrix_transform.hpp>

namespace kouek {
class GLView : public QOpenGLWidget {
    Q_OBJECT

  private:
    int functionKey = -1;

    std::array<GLuint, 2> FBO2{0, 0};
    glm::uvec2 renderSz{0};

  public:
    explicit GLView(QWidget *parent = Q_NULLPTR) : QOpenGLWidget(parent) {
        setFocusPolicy(Qt::StrongFocus);
        setCursor(Qt::CrossCursor);

        QSurfaceFormat surfaceFmt;
        surfaceFmt.setDepthBufferSize(24);
        surfaceFmt.setStencilBufferSize(8);
        surfaceFmt.setVersion(4, 5);
        surfaceFmt.setProfile(QSurfaceFormat::CoreProfile);
        setFormat(surfaceFmt);
    }
    inline void SetOutputFBO2(const std::array<GLuint, 2> &FBO2,
                              const glm::uvec2 &renderSz) {
        this->FBO2 = FBO2;
        this->renderSz = renderSz;
    }

  signals:
    void KeyPressed(int key, int functionKey);

  protected:
    void initializeGL() override {
        int hasInit = gladLoadGL();
        assert(hasInit);
    }
    void paintGL() override {
        std::array<int, 2> qOGLRenderSize = {width(), height()};
        auto hfW = qOGLRenderSize[0] / 2;
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, defaultFramebufferObject());
        glBindFramebuffer(GL_READ_FRAMEBUFFER, FBO2[0]);
        glBlitFramebuffer(0, 0, renderSz.x, renderSz.y, 0, 0, hfW,
                          qOGLRenderSize[1], GL_COLOR_BUFFER_BIT, GL_LINEAR);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, FBO2[1]);
        glBlitFramebuffer(0, 0, renderSz.x, renderSz.y, hfW, 0,
                          qOGLRenderSize[0], qOGLRenderSize[1],
                          GL_COLOR_BUFFER_BIT, GL_LINEAR);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    }

    void keyPressEvent(QKeyEvent *e) override {
        switch (e->key()) {
        case Qt::Key_Shift:
        case Qt::Key_Control:
            functionKey = e->key();
            break;
        default:
            emit KeyPressed(e->key(), functionKey);
            break;
        }
    }
    void keyReleaseEvent(QKeyEvent *e) override {
        switch (e->key()) {
        case Qt::Key_Shift:
        case Qt::Key_Control:
            functionKey = -1;
            break;
        default:
            break;
        }
    }
};
} // namespace kouek

#endif // !KOUEK_GL_VIEW_H
