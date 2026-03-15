#ifndef MAINWINDOW_COOKING_AW_H
#define MAINWINDOW_COOKING_AW_H

#include "ui_mainwindow_cooking_aw.h"
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
namespace Ui { class MainWindow_cookingAW; }
QT_END_NAMESPACE
class MainWindow_node;


class MainWindow_cookingAW : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow_cookingAW(QNode* qnode, QWidget *parent = 0);
    ~MainWindow_cookingAW();



private:
    Ui::MainWindow_cookingAW *ui;
    QTimer *timer_;
    QTimer *status_update_timer_;
    QNode *qnode_;
    py::object ros_thread;



private Q_SLOTS:
    void timerCallback();
    void testBtnCallback();
    void statusTimerCallback();

    void pushButtonGraspClickedCallback();
    void pushButtonStopClickedCallback();
    void pushButtonStartClickedCallback();

  private:

    QTimer* elapsed_timer;

  public:
    MainWindow_node* _taskWindow;
};

#endif // MAINWINDOW_COOKING_AW_H
