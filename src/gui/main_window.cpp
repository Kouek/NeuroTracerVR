#include "main_window.h"
#include "ui_main_window.h"

#include <QtWidgets/qstackedlayout.h>

kouek::MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    glView = new GLView;
    ui->groupBoxView->setLayout(new QStackedLayout);
    ui->groupBoxView->layout()->addWidget(glView);
}
