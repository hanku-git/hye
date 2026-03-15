#include "mainwindow_widgets_camera_view.hpp"
#include "ui_mainwindow_widgets_camera_view.h"


MainWindow_widgetsCameraView::MainWindow_widgetsCameraView(MainWindow_node * taskWindow, QWidget* parent)
    : _taskWindow(taskWindow)
    , QMainWindow(parent)
    , ui(new Ui::MainWindow_widgetsCameraView)
{
    ui->setupUi(this);
    ////////////////////////////// definition groupboxes, layouts

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow_widgetsCameraView::timerCallback);
    timer_->start(33);

}

MainWindow_widgetsCameraView::~MainWindow_widgetsCameraView()
{
    delete ui;
    rclcpp::shutdown();
}

void MainWindow_widgetsCameraView::timerCallback() {
    // QImage image = _taskWindow->getTaskManager()->_qnode->getCameraImage();
    // QPixmap pix;
    // // initialize raw image to qpixmap
    // pix = QPixmap::fromImage(image, Qt::AutoColor);
    // // show image on qlabel
    // ui->label_image_viewer->setPixmap(pix);
}

void MainWindow_widgetsCameraView::statusTimerCallback() {


}

void MainWindow_widgetsCameraView::testBtnCallback() {
    QImage image = _taskWindow->getTaskManager()->_qnode->getCameraImage();
    QPixmap pix;
    // initialize raw image to qpixmap
    pix = QPixmap::fromImage(image, Qt::AutoColor);
    // show image on qlabel
    ui->label_image_viewer->setPixmap(pix);
    ui->label_image_viewer_2->setPixmap(pix);
}
