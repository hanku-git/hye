#ifndef MAINWINDOW_WIDGETS_LAUNCH_PACKAGE_H
#define MAINWINDOW_WIDGETS_LAUNCH_PACKAGE_H

#include "ui_mainwindow_widgets_launch_package.h"
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
#include <QButtonGroup>

#include "rclcpp/rclcpp.hpp"
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
#include "rclcpp/clock.hpp"
#include "mainwindow_node.hpp"

#include <unistd.h>
#include <signal.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow_widgetsLaunchPackage; }
QT_END_NAMESPACE
class MainWindow_node;


class MainWindow_widgetsLaunchPackage : public QMainWindow
{
    Q_OBJECT

Q_SIGNALS:
    void openDeveloperWindow();
    void openIRLDeveloperWindow();

public:
    MainWindow_widgetsLaunchPackage(MainWindow_node * taskWindow, QWidget *parent = 0);
    ~MainWindow_widgetsLaunchPackage();

public:
    void initializeDialog();

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

    void updateProcessStatusIcon(bool isRunning, const QString& target);
    void rosNodeKillByName(const QString& rosNodeName);
    void rosNodeKillByNameList(const QStringList& nodeNames);
    void terminateQProcessAndRosNode();

    void pushButtonSTOPAllCallback();


    void setStatusIconLabel(const QString& name, QLabel* label);

private Q_SLOTS:
    void statusTimerCallback();


private:
    Ui::MainWindow_widgetsLaunchPackage *ui;
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

    void closeEvent(QCloseEvent *bar);



private:
  QString elapsed_time;
  QTimer* elapsed_timer;
  QMap<QString, QLabel*> statusIconLabelMap;

public:
  MainWindow_node* _taskWindow;



};

#endif // MAINWINDOW_WIDGETS_LAUNCH_PACKAGE_H
