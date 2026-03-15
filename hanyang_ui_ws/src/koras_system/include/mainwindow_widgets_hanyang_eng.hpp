#ifndef MAINWINDOW_WIDGETS_HANYANG_ENG_H
#define MAINWINDOW_WIDGETS_HANYANG_ENG_H

#include "ui_mainwindow_widgets_hanyang_eng.h"
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
namespace Ui { class MainWindow_widgetsHanyangEng; }
QT_END_NAMESPACE
class MainWindow_node;


class MainWindow_widgetsHanyangEng : public QMainWindow
{
    Q_OBJECT

Q_SIGNALS:
    void openDeveloperWindow();
    void openIRLDeveloperWindow();

public:
    MainWindow_widgetsHanyangEng(MainWindow_node * taskWindow, QWidget *parent = 0);
    ~MainWindow_widgetsHanyangEng();

public:
    void initializeDialog();
    void uploadImage();

    //// PLC
    void monitorPLCStatus();
    void checkPLCDoosanRobotCollision();

public Q_SLOTS:
    void timerCallback();
    void sendLogMessageStr(std::string &str);
    void pushButtonSTOPAllCallback();
    void rightPushButtonDoDrumTotalTasksCallback();
    void leftPushButtonDoDrumTotalTasksCallback();



    void pushButtonDoTasksAMRReadyPoseCallback();
    void leftPushButtonDoTasksDrumHomeposeCallback();
    void rightPushButtonDoTasksDrumHomeposeCallback();


    void pushButtonDoTasksCapDetachLeftCallback();
    void pushButtonDoTasksCapDetachRightCallback();
    void pushButtonDoTasksCapScrewingLeftCallback();
    void pushButtonDoTasksCapScrewingRightCallback();
    void pushButtonDoTasksCouplerDetachLeftCallback();
    void pushButtonDoTasksCouplerDetachRightCallback();
    void pushButtonDoTasksCouplerScrewingLeftCallback();
    void pushButtonDoTasksCouplerScrewingRightCallback();

    void pushButtonDoTasksRotationAngleDetectionCallback();
    void pushButtonDoTasksBarcodeDetectionCallback();

    void pushButtonDoTasksCapDetachAndCouplerScrewingLeftCallback();
    void pushButtonDoTasksCapDetachAndCouplerScrewingRightCallback();
    void pushButtonDoTasksCouplerDetachAndCapScrewingLeftCallback();
    void pushButtonDoTasksCouplerDetachAndCapScrewingRightCallback();

    ////////// right drum
    void rightPushButtonDrumSubTask1Callback();
    void rightPushButtonDrumSubTask2Callback();
    void rightPushButtonDrumSubTask3Callback();
    void rightPushButtonDrumSubTask4Callback();
    void rightPushButtonDrumSubTask5Callback();
    void rightPushButtonDrumSubTask6Callback();
    void rightPushButtonDrumSubTask7Callback();
    void rightPushButtonDrumSubTask9Callback();
    void rightPushButtonDrumSubTask10Callback();
    void rightPushButtonDrumSubTask11Callback();

    ////// 로봇 기능 테스트 위젯
    void rightPushButtonRobotAutoMode();
    void rightPushButtonRobotManualMode();
    void rightPushButtonKeyCodeTest();
    void rightPushButtonKeyCodeUnscrewing();

    /// @brief /// 250916 한양이엔지 호스 위치 체결 테스트
    void rightPushButtonHanyangHoseTest1();
    void rightPushButtonHanyangHoseTest2();
    void rightPushButtonHanyangHoseTest3();
    void rightPushButtonHanyangHoseTest4();

    /// 250918 한양이엔지 전체 동작 테스트
    void rightPushButtonHanyang0918Test1();
    void rightPushButtonHanyang0918Test2();
    void rightPushButtonHanyang0918Test3();
    void rightPushButtonHanyang0918Test4();
    void rightPushButtonHanyang0918Test5();
    void rightPushButtonHanyang0918Test6();
    void rightPushButtonHanyang0918Test7();
    void rightPushButtonHanyang0918Test8();
    

    ////////// left drum
    void leftPushButtonDrumSubTask1Callback();
    void leftPushButtonDrumSubTask2Callback();
    void leftPushButtonDrumSubTask3Callback();
    void leftPushButtonDrumSubTask4Callback();
    void leftPushButtonDrumSubTask5Callback();
    void leftPushButtonDrumSubTask6Callback();
    void leftPushButtonDrumSubTask7Callback();

    void pushButtonDrumScanTask1Callback();
    void pushButtonDrumScanTask2Callback();
    void pushButtonDrumScanTask3Callback();
    void pushButtonDrumScanTask4Callback();
    void pushButtonDrumScanTask5Callback();
    void pushButtonDrumScanTask6Callback();
    void pushButtonDrumScanTask7Callback();
    void pushButtonDrumScanTask8Callback();
    void pushButtonDrumScanTask9Callback();

    
    void pushButtonSetUIJSPosition11ClickedCallback();
    void pushButtonSetUIJSPosition12ClickedCallback();
    void pushButtonSetUIJSPosition13ClickedCallback();
    void pushButtonSetUIJSPosition14ClickedCallback();
    void pushButtonSetUIJSPosition15ClickedCallback();
    void pushButtonSetUIJSPosition16ClickedCallback();
    void pushButtonSetUIJSPosition17ClickedCallback();

    void pushButtonSetUIJSPosition18ClickedCallback();
    void pushButtonSetUIJSPosition19ClickedCallback();
    void pushButtonSetUIJSPosition20ClickedCallback();
    void pushButtonSetUIJSPosition21ClickedCallback();
    void pushButtonSetUIJSPosition22ClickedCallback();


    void pauseTaskBtnCallback();
    void resumeTaskBtnCallback();

    void pushButtonDoKorasChemicalGripperInitializeCallback();
    void pushButtonDoKorasChemicalGripperOpenCallback();
    void pushButtonDoKorasChemicalGripperCloseCallback();
    void pushButtonDoKorasChemicalGripperScrewingPoseCallback();
    void pushButtonDoKorasChemicalGripperUnscrewingPoseCallback();
    void pushButtonDoKorasChemicalGripperGraspingPoseCallback();
    void pushButtonDoKorasChemicalGripperExitPoseCallback();
    void pushButtonDoKorasChemicalGripperPlusMotorCtrlCallback();
    void pushButtonDoKorasChemicalGripperMinusMotorCtrlCallback();

    void pushButtonDoScanningZIVIDClickedCallback();
    void pushButtonDoTemplateMatchingBinPickingClickedCallback();
    void pushButtonSelectTargetObjectClickedCallback();


    void pushButtonComplianceCallback();
    void pushButtonComplianceOffCallback();
    void pushButtonSetToolWeight();
    void pushButtonMakeToolWeight();
    void pushButtonAddTcpPresetClickedCallback();
    void pushButtonGetTorqueClickedCallback();
    void pushButtonRobotOFFCallback();
    void pushButtonRobotONCallback();

    //// PLC
    void pushButtonSetPLCSimulationFlagClickedCallback();
    void pushButtonSelectPlcWriteAddressClickedCallback();
    void pushButtonSelectPlcSimulationTaskFlagClickedCallback();
    void pushButtonSelectPlcReadAddressClickedCallback();
    void pushButtonModbusWriteDataBoolType();
    void pushButtonModbusWriteDataInt16Type();
    void pushButtonModbusWriteDataASCIIType();

    void pushButtonModbusReadDataBoolType();
    void pushButtonModbusMonitoringPLC();
    void pushButtonModbusMonitoringPLCReadAMR2RobotStatus();
    void pushButtonModbusInitializePLCWrite41000Signals();
    void pushButtonModbusInitializePLCWrite41002Signals();
    void pushButtonModbusInitializePLCWrite41003Signals();
    void pushButtonModbusSendPLCWrite41000Bit14Signals();
    void pushButtonModbusTaskPLCFlagOnOff();
    //// 
    void PLCCommTaskCapDetach();
    void PLCCommTaskCouplerScrewing();
    void PLCCommTaskCouplerDetach();
    void PLCCommTaskCapScrewing();
    void PLCCommTaskCapDetachAndCouplerScrewingLeft();
    void PLCCommTaskCapDetachAndCouplerScrewingRight();
    void PLCCommTaskCouplerDetachAndCapScrewingLeft();
    void PLCCommTaskCouplerDetachAndCapScrewingRight();
    void PLCCommTaskRotationAngleDetection();
    void PLCCommTaskBarcodeReading();

private Q_SLOTS:
    void testBtnCallback();
    void statusTimerCallback();

    void UpdateImage(const QImage &image, const QString &current_pass);
    void changeRobotTCP(unsigned int tcp_idx);


private:
    Ui::MainWindow_widgetsHanyangEng *ui;
    QTimer *timer_;
    QTimer *status_update_timer_;
    QCheckBox* checkBox_autoMode;
    bool isRecording;
    void closeEvent(QCloseEvent *bar);

    void setupPlanner(); // TaskPlannerCooking 초기화 함수
    bool doTargetTask(const std::vector<UnitTask> &target_task);
    TaskPlannerHanyangEng *hanyangEngPlanner_;  // 전역 변수로 선언


protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;  // keyReleaseEvent 함수 선언 추가

private:
  std::vector<std::string> listNames;

  bool isTaskPaused;
  bool enabled;
  bool is_already_initialized_ = false;
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

    int current_gripper = 1; // 현재 장착된 그리퍼 (기본값: 2지 그리퍼)
    int current_tip = 1; // default: regular tip
    int current_obj = 36; // striker
    string target_name = "striker";

    JsDouble grp_home_pose_;
    JsDouble home_pose_;
    JsDouble mid_pose_;
    JsDouble scan_pose_;


    // 기존 이미지 및 좌표 업데이트 슬롯 제거
private:
    QList <QRadioButton *> target_object_list;
    QList <QRadioButton *> bin_picking_target_object_list_;
    QList <QCheckBox *> plc_hanyang_task_flag_check_box_41000_;
    QList <QCheckBox *> plc_hanyang_task_flag_check_box_41002_;
    QList <QCheckBox *> plc_hanyang_task_flag_check_box_41003_;
    QList <QCheckBox *> plc_hanyang_task_flag_check_box_42000_;
    QList <QCheckBox *> plc_hanyang_task_flag_check_box_42002_;
    

};

#endif // MAINWINDOW_WIDGETS_HANYANG_ENG_H
