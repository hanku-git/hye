#ifndef MAINWINDOW_WIDGETS_MONITORING_H
#define MAINWINDOW_WIDGETS_MONITORING_H

#include "ui_mainwindow_widgets_monitoring.h"
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
#include <QCloseEvent>
#include <chrono>
#include <thread>
#include <QCheckBox>
#include <QKeyEvent>

#include "rclcpp/rclcpp.hpp"
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
#include "rclcpp/clock.hpp"
#include "mainwindow_node.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow_widgetsMonitoring; }
QT_END_NAMESPACE
class MainWindow_node;


class MainWindow_widgetsMonitoring : public QMainWindow
{
    Q_OBJECT

Q_SIGNALS:
    void openDeveloperWindow();
    void openIRLDeveloperWindow();

public:
    MainWindow_widgetsMonitoring(MainWindow_node * taskWindow, QWidget *parent = 0);
    ~MainWindow_widgetsMonitoring();

// public Q_SLOTS:
  // void receiveTaskData();

public Q_SLOTS:
    void sendLogMessageStr(std::string &str);

private:
    Ui::MainWindow_widgetsMonitoring *ui;
    QTimer *timer_;
    QTimer *status_update_timer_;
    QCheckBox* checkBox_autoMode;
    bool isRecording;
    void closeEvent(QCloseEvent *bar);

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;  // keyReleaseEvent 함수 선언 추가

private Q_SLOTS:
    void testBtnCallback();
    void statusTimerCallback();

private:
  std::vector<std::string> listNames;

  bool isTaskPaused;
  bool enabled;
  int monitor_task_index;

  int elapsed_task_index;
  QLabel* elapsed_task_label;
  QLabel* started_time_label;
  QLabel* elapsed_time_label;

  QString elapsed_time;

  QTimer* elapsed_timer;

public:
  MainWindow_node* _taskWindow;


public:

    bool is_placement_task = false;
    int current_gripper = 1; // 현재 장착된 그리퍼 (기본값: 2지 그리퍼)
    int current_tip = 1; // default: regular tip
    int current_obj = 36; // striker
    string target_name = "striker";
    string llm_text;

    int tool_change_vel;
    int grasping_vel;
    int passing_vel;
    int loading_vel;

    int blue_stack = 0;
    int yellow_stack = 0;

    JsDouble grp_home_pose_;
    JsDouble home_pose_;
    JsDouble mid_pose_;
    JsDouble scan_pose_;

public Q_SLOTS: // 슬롯을 정의하는 영역
    void UpdateLLMText(const QString &target, const QString &goal, const QString &passes);
    void UpdateVelocites(const QString &velocities);
    void UpdateGoalPose(const geometry_msgs::msg::Pose2D &goal_pose);
private Q_SLOTS:
    void timerCallback();
    void TargetObjectParamSetting();
    // void RecordingCallback();
    void SetPoseCallback(const std::string& position);
    void GraspingTargetObject();
    void GripperAttachCallback();
    void GripperDetachCallback();
    void llm_node_Callback(LLM_PARAM &llm_param);
    void target_obj_Callback();
    void BoxSelectionCallback();
    void UpdateImage(const QImage &image, const QString &current_pass);
    void UpdateWaypoints(const QString &coordinates, const QString &current_pass);
    void RecordingStartCallback();  // 추가된 슬롯
    void RecordingStopCallback();   // 추가된 슬롯
    void LLMNodeReadyCallback();
    void onFocusChanged(QWidget* old, QWidget* now);



    // 기존 이미지 및 좌표 업데이트 슬롯 제거
private:
    QList <QRadioButton *> target_object_list;
    QList <QLineEdit *> waypoint_list;
    QList <QLineEdit *> pass_list;

};

#endif // MAINWINDOW_WIDGETS_MONITORING_H
