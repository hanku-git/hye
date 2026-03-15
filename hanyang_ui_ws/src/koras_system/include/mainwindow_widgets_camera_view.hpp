#ifndef MAINWINDOW_WIDGETS_CAMERA_VIEW_H
#define MAINWINDOW_WIDGETS_CAMERA_VIEW_H

#include "ui_mainwindow_widgets_camera_view.h"
#include <QtCore/QTimer>

#include <QMainWindow>
#include <QScreen>
#include <QLabel>
#include <QSlider>
#include <QWidget>
#include <QGridLayout>
#include <QProcess>
#include <QPainter>
#include <QPainterPath>
#include <QDebug>
#include <QMovie>
#include <QTextEdit>
#include <QDate>
#include <QTime>
#include <QTimer>
#include <QLCDNumber>
#include <QComboBox>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
#include "rclcpp/clock.hpp"
#include "mainwindow_node.hpp"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow_widgetsCameraView; }
QT_END_NAMESPACE
class MainWindow_node;


class MainWindow_widgetsCameraView : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow_widgetsCameraView(MainWindow_node * taskWindow, QWidget *parent = 0);
    ~MainWindow_widgetsCameraView();



private:
    Ui::MainWindow_widgetsCameraView *ui;
    QTimer *timer_;
    QTimer *status_update_timer_;



private Q_SLOTS:
    void timerCallback();
    void testBtnCallback();
    void statusTimerCallback();

  private:

    QTimer* elapsed_timer;

  public:
    MainWindow_node* _taskWindow;
};

#endif // MAINWINDOW_WIDGETS_CAMERA_VIEW_H
