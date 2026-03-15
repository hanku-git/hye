#ifndef MAINWINDOW_WIDGETS_HANYANG_LAUNCH_PACKAGE_H
#define MAINWINDOW_WIDGETS_HANYANG_LAUNCH_PACKAGE_H

#include "ui_mainwindow_widgets_hanyang_launch_package.h"
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
#include <QProgressDialog>
#include <QApplication>
#include <QDesktopWidget>
#include <QTimer>
#include <QLCDNumber>
#include <QComboBox>
#include <QCloseEvent>
#include <chrono>
#include <thread>
#include <QCheckBox>
#include <QKeyEvent>
#include <QButtonGroup>
#include <QMap>

#include "rclcpp/rclcpp.hpp"
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
#include "rclcpp/clock.hpp"
#include "mainwindow_node.hpp"

#include <unistd.h>
#include <signal.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow_widgetsHanyangLaunchPackage; }
QT_END_NAMESPACE
class MainWindow_node;


class MainWindow_widgetsHanyangLaunchPackage : public QMainWindow
{
    Q_OBJECT

Q_SIGNALS:
    void openDeveloperWindow();
    void openIRLDeveloperWindow();

public:
    MainWindow_widgetsHanyangLaunchPackage(MainWindow_node * taskWindow, QWidget *parent = 0);
    ~MainWindow_widgetsHanyangLaunchPackage();

public:
    void initializeDialog();
    // Image update to match HanyangEng behavior (shows image when available, otherwise "No image")
    void UpdateImage(const QImage &image, const QString &current_pass);

public Q_SLOTS:
    void timerCallback();
    void registerManagedProcess(QProcess*& processRef);
    void runGenericNode(QProcess*& processRef,
        QPushButton* button,
        QComboBox* comboBox,
        QTextEdit* logView,
        const QMap<QString, QString>& commandMap,
        const QString& processType);

    void pushButtonRunRobotController();
    void pushButtonRunGripperNode();
    void pushButtonRun3DScannerNode();
    void pushButtonRunMaskDetectionNode();
    void pushButtonRunRvizViewNode();
    void pushButtonRunScanNode();
    void pushButtonRunMatchingNode();
    void pushButtonRunRobotHandling();
    void pushButtonRunKeyCode();
    void pushButtonRunKeyCodeHolder();

    void on_pushButton_robot_auto_setup_clicked();
    void on_pushButton_robot_auto_off_clicked();
    void bringUIToFront();
    
    void updateProcessStatusIcon(bool isRunning, const QString& target);
    void rosNodeKillByName(const QString& rosNodeName);
    void rosNodeKillByNameList(const QStringList& nodeNames);
    void terminateQProcessAndRosNode();

    void setStatusIconLabel(const QString& name, QLabel* label);

private Q_SLOTS:
    void statusTimerCallback();
    void on_pushButton_makeToolWeight_clicked();
    void on_pushButton_setToolWeight_clicked();
    void on_pushButton_addTcpPreset_clicked();
    // --- PLC 기능 주석 처리 (mainwindow_hanyang_eng에서만 사용) ---
    // void on_pushButton_MODBUS_MonitoringPLC_clicked();
    // void on_pushButton_MODBUS_MonitoringPLCReadAMR2RobotStatus_clicked();
    void on_pushButton_auto_mode_clicked();
    void on_pushButton_manual_mode_clicked();




    // --- PLC 제어 함수들 (주석 처리) ---
    // void pushButtonSelectPlcWriteAddressClickedCallback();
    // void pushButtonSelectPlcReadAddressClickedCallback();
    // void pushButtonModbusWriteDataBoolType();
    // void pushButtonModbusReadDataBoolType();


private:
    Ui::MainWindow_widgetsHanyangLaunchPackage *ui;
    QTimer *timer_;
    QTimer *status_update_timer_;

    QList<QProcess**> managedProcesses;

    QProcess* robotControlNodeProcess = nullptr;
    QProcess* gripperNodeProcess = nullptr;
    QProcess* scannerCommNodeProcess = nullptr;
    QProcess* maskDetectionNodeProcess = nullptr;
    QProcess* rvizViewNodeProcess = nullptr;
    QProcess* scanNodeProcess = nullptr;
    QProcess* matchingNodeProcess = nullptr;
    QProcess* keyCodeNodeProcess = nullptr;
    QProcess* keyCodeHolderNodeProcess = nullptr;

    void closeEvent(QCloseEvent *bar);


private:
  QString elapsed_time;
  QTimer* elapsed_timer;
  QMap<QString, QLabel*> statusIconLabelMap;

public:
  MainWindow_node* _taskWindow;

private:
    void sendLogMessageStr(std::string &str);
    // --- PLC 체크박스 연동용 멤버 추가 (주석 처리) ---
    // QList<QCheckBox*> plc_launch_task_flag_check_box_41000_;
    // QList<QCheckBox*> plc_launch_task_flag_check_box_41002_;
    // QList<QCheckBox*> plc_launch_task_flag_check_box_42000_;
    // QList<QCheckBox*> plc_launch_task_flag_check_box_42002_;
    
    // --- Modbus Status 저장용 멤버 추가 (주석 처리) ---
    // int modbus_status_ = -1;

    // --- Task indicator update cache to avoid redundant redraws ---
    QString prev_task_log_;
    bool prev_is_task_mode_ = false;
    
    // --- Robot Auto Setup Progress Dialog ---
    QProgressDialog* setupProgressDialog;
};

#endif // MAINWINDOW_WIDGETS_HANYANG_LAUNCH_PACKAGE_H 