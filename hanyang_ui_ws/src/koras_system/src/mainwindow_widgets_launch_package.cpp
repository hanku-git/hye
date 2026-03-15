#include "mainwindow_widgets_launch_package.hpp"
#include "ui_mainwindow_widgets_launch_package.h"
#include <QKeyEvent>
#include <QApplication>
MainWindow_widgetsLaunchPackage::MainWindow_widgetsLaunchPackage(MainWindow_node * taskWindow, QWidget* parent)
    : _taskWindow(taskWindow)
    , QMainWindow(parent)
    , ui(new Ui::MainWindow_widgetsLaunchPackage)
{
    ui->setupUi(this);


    ////////////////////////////////////////////////////////////
    ////////////////////////////// Add connect functions to ui components


    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Robot Controller Start Buttons
    QObject::connect(ui->pushButton_doRobotConnect        , SIGNAL(clicked()), this, SLOT(pushButtonRunRobotController()));
    QObject::connect(ui->pushButton_doGripperConnect        , SIGNAL(clicked()), this, SLOT(pushButtonRunGripperNode()));

    //// Vision Module Start Buttons
    QObject::connect(ui->pushButton_do3DScannerNode        , SIGNAL(clicked()), this, SLOT(pushButtonRun3DScannerNode()));
    QObject::connect(ui->pushButton_doMaskDetectionNode        , SIGNAL(clicked()), this, SLOT(pushButtonRunMaskDetectionNode()));
    QObject::connect(ui->pushButton_doRvizViewNode        , SIGNAL(clicked()), this, SLOT(pushButtonRunRvizViewNode()));

    //// Pose Estimation Start Buttons
    QObject::connect(ui->pushButton_doScanNode        , SIGNAL(clicked()), this, SLOT(pushButtonRunScanNode()));
    QObject::connect(ui->pushButton_doMatchingNode        , SIGNAL(clicked()), this, SLOT(pushButtonRunMatchingNode()));

    QObject::connect(ui->pushButton_terminateAllNode        , SIGNAL(clicked()), this, SLOT(terminateQProcessAndRosNode()));




    updateProcessStatusIcon(false, "robot");
    updateProcessStatusIcon(false, "gripper");
    updateProcessStatusIcon(false, "3d scanner");
    updateProcessStatusIcon(false, "mask detection");
    updateProcessStatusIcon(false, "rviz view");
    updateProcessStatusIcon(false, "scan node");
    updateProcessStatusIcon(false, "matching node");

    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    //// Qt Main timer
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow_widgetsLaunchPackage::timerCallback);
    timer_->start(100);

    //// Qt sub timer
    status_update_timer_ = new QTimer(this);
    connect(status_update_timer_, &QTimer::timeout, this, &MainWindow_widgetsLaunchPackage::statusTimerCallback);
    // status_update_timer_->start(200);
    status_update_timer_->start(1000);

}


MainWindow_widgetsLaunchPackage::~MainWindow_widgetsLaunchPackage()
{
    // qDebug() << "[Destructor] MainWindow_widgetsLaunchPackage called";
    terminateQProcessAndRosNode();
    delete ui;
    rclcpp::shutdown();
}

void MainWindow_widgetsLaunchPackage::closeEvent(QCloseEvent *event)
{
    if (QApplication::closingDown()) {
        // 프로그램 전체 종료 중 (ex: Ctrl+C, app.quit() 등)
        ROS_LOG_WARN("QApplication::closingDown");
        event->accept();  // 정상 종료 허용
    } else {
        // 사용자가 서브윈도우만 닫으려는 경우: 막기
        ROS_LOG_WARN("IGNORE");
        event->ignore();
    }
}

void MainWindow_widgetsLaunchPackage::initializeDialog() {
    updateProcessStatusIcon(false, "robot");
    updateProcessStatusIcon(false, "gripper");
    updateProcessStatusIcon(false, "3d scanner");
    updateProcessStatusIcon(false, "mask detection");
    updateProcessStatusIcon(false, "rviz view");
    updateProcessStatusIcon(false, "scan node");
    updateProcessStatusIcon(false, "matching node");
}

void MainWindow_widgetsLaunchPackage::timerCallback() {


}

void MainWindow_widgetsLaunchPackage::statusTimerCallback() {


}

void MainWindow_widgetsLaunchPackage::registerManagedProcess(QProcess*& processRef)
{
    if (!managedProcesses.contains(&processRef)) {
        managedProcesses.append(&processRef);
    }
}

void MainWindow_widgetsLaunchPackage::runGenericNode(
    QProcess*& processRef,
    QPushButton* button,
    QComboBox* comboBox,
    QTextEdit* logView,
    const QMap<QString, QString>& commandMap,
    const QString& processType)
{
    _taskWindow->getTaskManager()->_qnode->is_package_launched_ = true;

    QString command;
    QString nodeName;

    if (processRef) {
        if (processRef->state() != QProcess::NotRunning) {
            logView->append("Terminating current node...");
            processRef->kill();
            ::kill(-processRef->processId(), SIGKILL);
            processRef->waitForFinished(1000);
            logView->append("Terminated by user.");
        } else {
            logView->append("Process already not running.");
        }

        button->setChecked(false);
        logView->moveCursor(QTextCursor::End);
        updateProcessStatusIcon(false, processType);
        return;
    }

    if (button->isChecked()) {
        QString selectedText = comboBox ? comboBox->currentText() : "none";
        if (commandMap.contains(selectedText)) {
            command = commandMap[selectedText];
            nodeName = selectedText;
            ROS_LOG_WARN("%s selected", selectedText.toStdString().c_str());
        } else {
            ROS_LOG_WARN("Unknown selection");
        }
    }

    if (!command.isEmpty()) {
        logView->clear();
        logView->append("[" + nodeName + "] Node started...\n");

        processRef = new QProcess(this);
        registerManagedProcess(processRef); // QProcess 등록, 프로그램 종료 시 사용
        processRef->setProcessChannelMode(QProcess::MergedChannels);

        connect(processRef, &QProcess::readyReadStandardOutput, this, [=]() {
            QString output = processRef->readAllStandardOutput();
            logView->append(output.trimmed());
            logView->moveCursor(QTextCursor::End);
        });

        connect(processRef, &QProcess::readyReadStandardError, this, [=]() {
            QString error = processRef->readAllStandardError();
            logView->append("[ERROR] " + error.trimmed());
            logView->moveCursor(QTextCursor::End);
        });

        connect(processRef, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                this, [this, &processRef, nodeName, logView, processType](int exitCode, QProcess::ExitStatus) {
            logView->append("[" + nodeName + "] controller exited (code: " + QString::number(exitCode) + ")");
            logView->moveCursor(QTextCursor::End);

            // 💡 여러 ROS 노드 강제 종료
            QMap<QString, QStringList> nodeToRosNameList = {
                // Robot Node
                {"Universal Robots UR10e", {"/UR10e"}},
                {"Fairino FR5", {"/FR5"}}, // TODO: 노드명 확인
                {"Doosan M1013", {"/dsr01/controller_manager", "/dsr01/dsr_hw_interface2", "/dsr01/robot_state_publisher", "/dsr01/spawner_dsr_controller2", "/dsr01/spawner_joint_state_broadcaster", "/dsr01/virtual_node"}},
                {"Doosan E0509", {"/dsr01/controller_manager", "/dsr01/dsr_hw_interface2", "/dsr01/robot_state_publisher", "/dsr01/spawner_dsr_controller2", "/dsr01/spawner_joint_state_broadcaster", "/dsr01/virtual_node"}},
                // Gripper Node
                {"Koras Gripper", {"/DATC_Control_Interface"}},
                // 3D Scanner Comm. Node
                {"[Scanner Type] Zivid2+ MR60", {"/scanner_node", "/scanner_node_sub"}},
                {"[Scanner Type] Zivid2+ M130", {"/scanner_node", "/scanner_node_sub"}},
                {"[Scanner Type] Rvbust RVC I2370", {"/scanner_node", "/scanner_node_sub"}},
                {"[Scanner Type] Rvbust RVC P31300", {"/scanner_node", "/scanner_node_sub"}},
                // Marker Detection Node
                {"AI Module (SAM)", {"/sam_zivid"}},
                {"No AI Module", {"/sam_zivid"}},
                // Rviz View Node
                {"Visualize all", {"/rviz2_v1", "/rviz2_v2", "/rviz2_v5", "/rviz2_v8"}},
                // Scan Node
                {"[Scan Node] Zivid2+ MR60", {"/zivid_scan"}},
                {"[Scan Node] Zivid2+ M130", {"/zivid_scan"}},
                {"[Scan Node] Rvbust RVC I2370", {"/zivid_scan"}},
                {"[Scan Node] Rvbust RVC P31300", {"/zivid_scan"}},
                // Matching Node
                {"Matching", {"/template_matching_node"}}
            };

            if (nodeToRosNameList.contains(nodeName)) {
                rosNodeKillByNameList(nodeToRosNameList[nodeName]);
            } else {
                qDebug() << "[runGenericNode] Unknown node name for ROS kill:" << nodeName;
            }

            processRef->deleteLater();
            processRef = nullptr;
            updateProcessStatusIcon(false, processType);
        });


        processRef->start("bash", QStringList() << "-c" << "setsid bash -c '" + command + "'");
        updateProcessStatusIcon(true, processType);
    }
}



void MainWindow_widgetsLaunchPackage::pushButtonRunRobotController() {
    QMap<QString, QString> command_list = {
        {"Universal Robots UR10e", "source ~/robot_control_ws/install/setup.bash && ros2 run ur10e_controller ur10e_controller 192.168.0.12"},
        {"Fairino FR5", "source ~/robot_control_ws/install/setup.bash && export LD_LIBRARY_PATH=~/robot_control_ws/src/fr5_control_package/fr_controller/include/fr_controller/libfairino/lib/libfairino:$LD_LIBRARY_PATH && ros2 run fr_controller fr_controller"},
        {"Doosan M1013", "source ~/doosan_ros2_ws/install/setup.bash && ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.100.108 port:=12345 model:=m1013"},
        {"Doosan E0509", "source ~/robot_control_ws/install/setup.bash && ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.100.108 port:=12345 model:=e0509"}
    };

    runGenericNode(
        robotControlNodeProcess,
        ui->pushButton_doRobotConnect,
        ui->comboBox_RobotLists,
        ui->textEdit_log_robotController,
        command_list,
        "robot"
    );
}
void MainWindow_widgetsLaunchPackage::pushButtonRunGripperNode() {
    QMap<QString, QString> command_list = {
        {"Koras Gripper", "source ~/gripper_ws/install/setup.bash && ros2 run kr_gcs_ui kr_gcs_ui --no-ui"}
    };
    runGenericNode(
        gripperNodeProcess,
        ui->pushButton_doGripperConnect,
        ui->comboBox_GripperLists,
        ui->textEdit_log_gripperNode,
        command_list,
        "gripper"
    );
}   


void MainWindow_widgetsLaunchPackage::pushButtonRun3DScannerNode() {
    QMap<QString, QString> command_list = {
        {"[Scanner Type] Zivid2+ MR60", "source ~/scanner_ws/install/setup.bash && python3 -u ~/scanner_ws/install/scanner_node/lib/scanner_node/zivid2mr60_topic_node.py"},
        {"[Scanner Type] Zivid2+ M130", "source ~/scanner_ws/install/setup.bash && ros2 run scanner_node zivid2m130_topic_node.py"},
        {"[Scanner Type] Rvbust RVC I2370", "source ~/scanner_ws/install/setup.bash && ros2 run scanner_node zivid2mr60_topic_node.py"},
        {"[Scanner Type] Rvbust RVC P31300", "source ~/scanner_ws/install/setup.bash && ros2 run scanner_node zivid2mr60_topic_node.py"}
    };

    runGenericNode(
        scannerCommNodeProcess,
        ui->pushButton_do3DScannerNode,
        ui->comboBox_3DScannerLists,
        ui->textEdit_log_3DScannerLogView,
        command_list,
        "3d scanner"
    );
}


void MainWindow_widgetsLaunchPackage::pushButtonRunMaskDetectionNode() {
    QMap<QString, QString> command_list = {
        {"AI Module (SAM)", "source ~/sam_ws/install/setup.bash && ros2 run sam_ros sam_node.py"},
        {"No AI Module", "source ~/sam_ws/install/setup.bash && ros2 run sam_ros sam_node.py"}
    };

    runGenericNode(
        maskDetectionNodeProcess,
        ui->pushButton_doMaskDetectionNode,
        ui->comboBox_MaskDetectionLists,
        ui->textEdit_log_MaskDetectionLogView,
        command_list,
        "mask detection"
    );
}

void MainWindow_widgetsLaunchPackage::pushButtonRunRvizViewNode() {
    QMap<QString, QString> command_list = {
        {"Visualize all", "source ~/bin_picking_ws/install/setup.bash && ros2 launch bin_picking_visualizer visualize_zvd_cad_grasp.xml"}
    };

    runGenericNode(
        rvizViewNodeProcess,
        ui->pushButton_doRvizViewNode,
        ui->comboBox_RvizViewLists, // nullptr(콤보박스 없는 경우)
        ui->textEdit_log_RvizView,
        command_list,
        "rviz view"
    );
}

void MainWindow_widgetsLaunchPackage::pushButtonRunScanNode() {
    QMap<QString, QString> command_list = {
        {"[Scan Node] Zivid2+ MR60", "source ~/bin_picking_ws/install/setup.bash && ros2 run bin_picking_process zivid_scan_node"},
        {"[Scan Node] Zivid2+ M130", "source ~/bin_picking_ws/install/setup.bash && ros2 run bin_picking_process zivid_scan_node"},
        {"[Scan Node] Rvbust RVC I2370", "source ~/bin_picking_ws/install/setup.bash && ros2 run bin_picking_process zivid_scan_node"},
        {"[Scan Node] Rvbust RVC P31300", "source ~/bin_picking_ws/install/setup.bash && ros2 run bin_picking_process zivid_scan_node"}
    };

    runGenericNode(
        scanNodeProcess,
        ui->pushButton_doScanNode,
        ui->comboBox_scanNodeLists,
        ui->textEdit_log_ScanNodeLogView,
        command_list,
        "scan node"
    );
}

void MainWindow_widgetsLaunchPackage::pushButtonRunMatchingNode() {
    QMap<QString, QString> command_list = {
        {"Matching", "source ~/bin_picking_ws/install/setup.bash && ros2 run bin_picking_process matching_node"}
    };

    runGenericNode(
        matchingNodeProcess,
        ui->pushButton_doMatchingNode,
        ui->comboBox_matchingNodeLists, // nullptr(콤보박스 없는 경우)
        ui->textEdit_log_MatchingNodeLogView,
        command_list,
        "matching node"
    );
}


// void MainWindow_widgetsLaunchPackage::updateProcessStatusIcon(bool isRunning, const QString& target)
// {
//     int size = 30;
//     QPixmap pixmap(size, size);
//     pixmap.fill(Qt::transparent);

//     QPainter painter(&pixmap);
//     painter.setRenderHint(QPainter::Antialiasing);
//     QColor color = isRunning ? QColor("green") : QColor("red");
//     painter.setBrush(color);
//     painter.setPen(Qt::NoPen);
//     painter.drawEllipse(0, 0, size, size);

//     QLabel* label = nullptr;

//     if (target == "robot")
//         label = ui->label_robotStatusIcon;
//     else if (target == "gripper")
//         label = ui->label_gripperStatusIcon;
//     else if (target == "3d scanner")
//         label = ui->label_3DScannerStatusIcon;
//     else if (target == "mask detection")
//         label = ui->label_maskDetectionStatusIcon;
//     else if (target == "rviz view")
//         label = ui->label_rvizViewStatusIcon;
//     else if (target == "scan node")
//         label = ui->label_scanNodeStatusIcon;
//     else if (target == "matching node")
//         label = ui->label_matchingNodeStatusIcon;

//     if (label) {
//         label->setPixmap(pixmap);
//         label->setFixedSize(size, size);
//     }
// }

void MainWindow_widgetsLaunchPackage::setStatusIconLabel(const QString& name, QLabel* label)
{
    statusIconLabelMap[name] = label;
}

void MainWindow_widgetsLaunchPackage::updateProcessStatusIcon(bool isRunning, const QString& target)
{
    if (!statusIconLabelMap.contains(target)) {
        qWarning() << "[updateProcessStatusIcon] Unknown target:" << target;
        return;
    }

    QLabel* label = statusIconLabelMap[target];
    if (!label) return;

    int size = 30;
    QPixmap pixmap(size, size);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing);
    QColor color = isRunning ? QColor("green") : QColor("red");
    painter.setBrush(color);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(0, 0, size, size);

    label->setPixmap(pixmap);
    label->setFixedSize(size, size);
}


void MainWindow_widgetsLaunchPackage::rosNodeKillByName(const QString& rosNodeName)
{
    // 1. 현재 노드 목록 확인
    QProcess nodeListProcess;
    nodeListProcess.start("bash", QStringList() << "-c" << "ros2 node list");
    nodeListProcess.waitForFinished(1000);
    QStringList nodeList = QString(nodeListProcess.readAllStandardOutput()).split('\n', Qt::SkipEmptyParts);

    // 2. 존재 여부 확인
    bool found = false;
    for (const QString& node : nodeList) {
        if (node.trimmed() == rosNodeName) {
            found = true;
            break;
        }
    }
    if (!found) {
        // qDebug() << "[rosNodeKillByName] Node not found:" << rosNodeName;
        return;
    }

    // 3. 노드 이름 → 실행 파일 이름 추정 (간접 매핑)
    QMap<QString, QString> hintMap = {
        {"/UR10e", "ur10e_controller"},
        {"/DATC_Control_Interface", "kr_gcs_ui"},
        {"/rviz2_v1", "rviz2_v1"},
        {"/rviz2_v2", "rviz2_v2"},
        {"/rviz2_v5", "rviz2_v5"},
        {"/rviz2_v8", "rviz2_v8"},
        {"/scanner_node", "scanner_node"},
        {"/scanner_node_sub", "scanner_node"},
        {"/zivid_scan", "zivid_scan_node"},
        {"/template_matching_node", "matching_node"}
    };
    QString searchKeyword = hintMap.value(rosNodeName, rosNodeName);  // fallback to node name

    // 4. ps aux | grep [실행 파일 이름] 으로 PID 탐색
    QString grepCommand = QString("ps aux | grep %1 | grep -v grep | awk '{print $2}'").arg(searchKeyword);
    QProcess pidProcess;
    pidProcess.start("bash", QStringList() << "-c" << grepCommand);
    pidProcess.waitForFinished(1000);
    QStringList pidList = QString(pidProcess.readAllStandardOutput()).split('\n', Qt::SkipEmptyParts);

    if (pidList.isEmpty()) {
        // qDebug() << "[rosNodeKillByName] No running PID found for node:" << rosNodeName;
        return;
    }

    for (const QString& pid : pidList) {
        QString cmd = "kill -9 " + pid;
        QProcess::execute("bash", QStringList() << "-c" << cmd);
        qDebug() << "[rosNodeKillByName] Killed PID:" << pid << "for node:" << rosNodeName;
    }
}

void MainWindow_widgetsLaunchPackage::rosNodeKillByNameList(const QStringList& nodeNames)
{
    // 1. ros2 node list 가져오기
    QProcess nodeListProcess;
    nodeListProcess.start("bash", QStringList() << "-c" << "ros2 node list");
    nodeListProcess.waitForFinished(300);
    QStringList activeNodes = QString(nodeListProcess.readAllStandardOutput()).split('\n', Qt::SkipEmptyParts);

    for (const QString& rosNodeName : nodeNames) {
        // 2. 현재 노드가 존재하는지 확인
        bool found = false;
        for (const QString& n : activeNodes) {
            if (n.trimmed() == rosNodeName.trimmed()) {
                found = true;
                break;
            }
        }

        if (!found) {
            // qDebug() << "[rosNodeKillByNameList] Node not found:" << rosNodeName;
            continue;
        }

        // 3. 실행 중인 프로세스 찾기 (실행 명령어 기반)
        QString grepCommand = QString("ps aux | grep %1 | grep -v grep | awk '{print $2}'").arg(rosNodeName.section('/', 1, 1));
        QProcess findPidProcess;
        findPidProcess.start("bash", QStringList() << "-c" << grepCommand);
        findPidProcess.waitForFinished(300);

        QStringList pidList = QString(findPidProcess.readAllStandardOutput()).split('\n', Qt::SkipEmptyParts);

        if (pidList.isEmpty()) {
            // qDebug() << "[rosNodeKillByNameList] No running PID found for node:" << rosNodeName;
            continue;
        }

        for (const QString& pid : pidList) {
            QString killCmd = "kill -9 " + pid;
            QProcess::execute("bash", QStringList() << "-c" << killCmd);
            qDebug() << "[rosNodeKillByNameList] Killed PID:" << pid << "for node:" << rosNodeName;
        }
    }
}

void MainWindow_widgetsLaunchPackage::terminateQProcessAndRosNode()
{
#if !IS_RELEASE_VERSION
    ROS_LOG_WARN("[DEVELOPER VERSION] ROS2 processes are not terminated!");
    ROS_LOG_WARN("[DEVELOPER VERSION] ROS2 processes are not terminated!");
    ROS_LOG_WARN("[DEVELOPER VERSION] ROS2 processes are not terminated!");
    ROS_LOG_WARN("[DEVELOPER VERSION] ROS2 processes are not terminated!");
    ROS_LOG_WARN("[DEVELOPER VERSION] ROS2 processes are not terminated!");
    return;

#endif


    if(!_taskWindow->getTaskManager()->_qnode->is_package_launched_) {
        return;
    }

    auto tryKill = [](QProcess*& proc, const QString& name) {
        if (proc && proc->state() != QProcess::NotRunning) {
            qDebug() << "[Destructor] Killing process:" << name << "(PID:" << proc->processId() << ")";
            proc->kill();
            ::kill(-proc->processId(), SIGKILL);
            proc->waitForFinished(1000);
            delete proc;
            proc = nullptr;
        } else {
            // qDebug() << "[Destructor] Process already not running:" << name;
        }
    };

    ////////////////////////////////////////////////////////////
    //// QProcess Kill
    tryKill(robotControlNodeProcess, "robotControlNodeProcess");
    tryKill(gripperNodeProcess, "gripperNodeProcess");
    tryKill(scannerCommNodeProcess, "scannerCommNodeProcess");
    tryKill(maskDetectionNodeProcess, "maskDetectionNodeProcess");
    tryKill(rvizViewNodeProcess, "rvizViewNodeProcess");
    tryKill(scanNodeProcess, "scanNodeProcess");
    tryKill(matchingNodeProcess, "matchingNodeProcess");
    ////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////
    //// ROS Node Kill
    // Robot Node
    rosNodeKillByName("/UR10e");
    rosNodeKillByName("/dsr01/controller_manager");
    rosNodeKillByName("/dsr01/dsr_hw_interface2");
    rosNodeKillByName("/dsr01/robot_state_publisher");
    rosNodeKillByName("/dsr01/spawner_dsr_controller2");
    rosNodeKillByName("/dsr01/spawner_joint_state_broadcaster");
    rosNodeKillByName("/dsr01/virtual_node");
    // Gripper Node
    rosNodeKillByName("/DATC_Control_Interface");
    // 3D Scanner Comm. Node
    rosNodeKillByName("/scanner_node");
    rosNodeKillByName("/scanner_node_sub");
    // Mask Detection Node
    rosNodeKillByName("/sam_zivid");
    // Rviz View Node
    rosNodeKillByName("/rviz2_v1");
    rosNodeKillByName("/rviz2_v2");
    rosNodeKillByName("/rviz2_v5");
    rosNodeKillByName("/rviz2_v8");
    // Scan Node
    rosNodeKillByName("/zivid_scan");
    // Matching Node
    rosNodeKillByName("/template_matching_node");

    //// 아래는 hanyang_eng_koras_system 패키지에 속하는 노드
    // rosNodeKillByName("/Gripper_setting_node");
    // rosNodeKillByName("/rviz_render_node");
    // rosNodeKillByName("bin_picking_node");
    // rosNodeKillByName("UR10e_koras_system");
    ////////////////////////////////////////////////////////////


    ui->pushButton_doRobotConnect->setChecked(false);
    ui->pushButton_doGripperConnect->setChecked(false);
    ui->pushButton_do3DScannerNode->setChecked(false);
    ui->pushButton_doMaskDetectionNode->setChecked(false);
    ui->pushButton_doRvizViewNode->setChecked(false);
    ui->pushButton_doScanNode->setChecked(false);
    ui->pushButton_doMatchingNode->setChecked(false);

}



void MainWindow_widgetsLaunchPackage::pushButtonSTOPAllCallback() {
    _taskWindow->getTaskManager()->_qnode->is_llm_task_fin = true;
    _taskWindow->getTaskManager()->_qnode->is_task_mode_ = false;

    //// PLC FLAG
    // _taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_ = false; // AMR로부터 PLC 수신을 위한 모니터링 정지
    // ui->pushButton_MODBUS_MonitoringPLC->setChecked(false); // 버튼 누른 상태 활성화
    // _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;


#if DRFL_CONTROL
    _taskWindow->getTaskManager()->_qnode->drflMoveStop(2);
#else
    _taskWindow->getTaskManager()->_qnode->stopRobot();
#endif
}
