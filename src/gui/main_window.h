#ifndef KOUEK_MAIN_WINDOW_H
#define KOUEK_MAIN_WINDOW_H

#include <QtWidgets/qmainwindow.h>

#include "gl_view.hpp"

namespace Ui {
class MainWindow;
}

namespace kouek {
class MainWindow : public QMainWindow {
    Q_OBJECT

  private:
    Ui::MainWindow *ui;
    GLView *glView;

  public:
    explicit MainWindow(QWidget *parent = Q_NULLPTR);
    inline void SetOutputFBO2(const std::array<GLuint, 2> &FBO2,
                              const glm::uvec2 &renderSz) {
        glView->SetOutputFBO2(FBO2, renderSz);
    }
    inline const auto GetGLView() { return glView; };

  protected:
    void closeEvent(QCloseEvent *e) { emit Closed(); }

  signals:
    void Closed();
};
} // namespace kouek

#endif // !KOUEK_MAIN_WINDOW_H
