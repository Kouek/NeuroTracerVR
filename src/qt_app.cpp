#include "qt_app.h"

kouek::QtApp::QtApp(int argc, char **argv,
                    std::shared_ptr<SharedStates> sharedStates,
                    std::shared_ptr<StatefulSystem> statefulSys)
    : sharedStates(sharedStates), statefulSys(statefulSys), app(argc, argv) {
    mainWindow = new MainWindow;
    initSignalSlots();
    mainWindow->showMaximized();
}

void kouek::QtApp::ProcessInput() {
    app.processEvents();
    mainWindow->GetGLView()->makeCurrent();
}

void kouek::QtApp::ProcessOutput() { mainWindow->GetGLView()->update(); }

void kouek::QtApp::initSignalSlots() {
    connect(mainWindow, &MainWindow::Closed,
            [&]() { sharedStates->canRun = false; });

    static int key = -1, functionKey = -1;
    // key -> camera
    static constexpr auto MOVE_SENSITY = .01f;
    static constexpr auto ROT_SENSITY = glm::radians(10.f);
    statefulSys->Register(
        [&]() {
            switch (functionKey) {
            case -1:
                sharedStates->camera.Move(key == Qt::Key_Right  ? +MOVE_SENSITY
                                          : key == Qt::Key_Left ? -MOVE_SENSITY
                                                                : 0,
                                          0,
                                          key == Qt::Key_Up     ? +MOVE_SENSITY
                                          : key == Qt::Key_Down ? -MOVE_SENSITY
                                                                : 0);
                break;
            case Qt::Key_Shift:
                sharedStates->camera.Move(key == Qt::Key_Right  ? +MOVE_SENSITY
                                          : key == Qt::Key_Left ? -MOVE_SENSITY
                                                                : 0,
                                          key == Qt::Key_Up     ? +MOVE_SENSITY
                                          : key == Qt::Key_Down ? -MOVE_SENSITY
                                                                : 0,
                                          0);
                break;
            }
            key = functionKey = -1;
        },
        std::tie(sharedStates->camera),
        [&]() {
            return key == Qt::Key_Up || key == Qt::Key_Down ||
                   key == Qt::Key_Left || key == Qt::Key_Right;
        });
    // key to maxStepCnt
    static constexpr auto STEP_CNT_SENSITY = 100;
    statefulSys->Register(
        [&]() {
            sharedStates->maxStepCnt +=
                key == Qt::Key_Plus ? +STEP_CNT_SENSITY : -STEP_CNT_SENSITY;
            key = functionKey = -1;
        },
        std::tie(sharedStates->maxStepCnt),
        [&]() {
            return (key == Qt::Key_Plus &&
                    sharedStates->maxStepCnt <
                        SharedStates::MAX_STEP_CNT - STEP_CNT_SENSITY) ||
                   (key == Qt::Key_Minus &&
                    sharedStates->maxStepCnt >
                        SharedStates::MIN_STEP_CNT + STEP_CNT_SENSITY);
        });
    // key to renderTarget
    statefulSys->Register(
        [&]() {
            sharedStates->renderTarget =
                key == Qt::Key_PageUp
                    ? static_cast<VRRenderer::RenderTarget>(
                          static_cast<uint8_t>(sharedStates->renderTarget) + 1)
                    : static_cast<VRRenderer::RenderTarget>(
                          static_cast<uint8_t>(sharedStates->renderTarget) - 1);
            key = functionKey = -1;
        },
        std::tie(sharedStates->renderTarget),
        [&]() {
            return (key == Qt::Key_PageUp &&
                    static_cast<uint8_t>(sharedStates->renderTarget) !=
                        static_cast<uint8_t>(VRRenderer::RenderTarget::End) -
                            1) ||
                   (key == Qt::Key_PageDown &&
                    static_cast<uint8_t>(sharedStates->renderTarget) != 0);
        });
    // key to FAVRLvl
    statefulSys->Register(
        [&]() {
            if (key == Qt::Key_Home)
                ++sharedStates->FAVRLvl;
            else
                --sharedStates->FAVRLvl;
            key = functionKey = -1;
        },
        std::tie(sharedStates->FAVRLvl),
        [&]() {
            return (key == Qt::Key_Home &&
                    sharedStates->FAVRLvl != VRRenderer::MAX_SUBSAMPLE_LEVEL) ||
                   (key == Qt::Key_End && sharedStates->FAVRLvl != 1);
        });

    connect(mainWindow->GetGLView(), &GLView::KeyPressed,
            [&](int keyIn, int functionKeyIn) {
                key = keyIn;
                functionKey = functionKeyIn;
            });
}
