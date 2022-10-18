#ifndef KOUEK_QT_APP_H
#define KOUEK_QT_APP_H

#include <memory>
#include <vector>

#include <QtWidgets/qapplication.h>

#include "shared_states.h"

#include "gui/main_window.h"

namespace kouek {
class QtApp : public QObject {
    Q_OBJECT

  private:
    std::shared_ptr<SharedStates> sharedStates;
    std::shared_ptr<StatefulSystem> statefulSys;

    MainWindow *mainWindow;

    QApplication app;

  public:
    QtApp(int argc, char **argv, std::shared_ptr<SharedStates> sharedStates,
          std::shared_ptr<StatefulSystem> statefulSys);
    void ProcessInput();
    void ProcessOutput();
    inline void SetOutputFBO2(const std::array<GLuint, 2> &FBO2,
                              const glm::uvec2 &renderSz) {
        mainWindow->SetOutputFBO2(FBO2, renderSz);
    }

  private:
    void initSignalSlots();
    };
} // namespace kouek

#endif // !KOUEK_QT_APP_H
