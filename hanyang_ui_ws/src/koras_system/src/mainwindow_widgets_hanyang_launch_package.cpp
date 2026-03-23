#include "mainwindow_widgets_hanyang_launch_package.hpp"
#include "ui_mainwindow_widgets_hanyang_launch_package.h"
#include <QKeyEvent>
#include <QApplication>
#include "modbus/modbusTcpDefine.hpp"
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QDateTime>

MainWindow_widgetsHanyangLaunchPackage::MainWindow_widgetsHanyangLaunchPackage(MainWindow_node * taskWindow, QWidget* parent)
    : _taskWindow(taskWindow)
    , QMainWindow(parent)
    , ui(new Ui::MainWindow_widgetsHanyangLaunchPackage)
{
    ui->setupUi(this);
    
    // 윈도우를 항상 맨 위에 표시
    setWindowFlags(windowFlags() | Qt::WindowStaysOnTopHint);
    
    // 윈도우 타이틀 설정 (선택사항)
    setWindowTitle("Hanyang Launch Package - Always On Top");
    
    // 프로그레스 다이얼로그 초기화
    setupProgressDialog = nullptr;

    // 로그 창 설정
    ui->textEdit_log->setReadOnly(true);
    QFont logFont;
    logFont.setPointSize(18);
    ui->textEdit_log->setFont(logFont);
    ui->textEdit_log->setMaximumHeight(500);

    // label_image_viewer 초기화 - "No image" 표시
    ui->label_image_viewer->setText("No image");
    ui->label_image_viewer->setAlignment(Qt::AlignCenter);
    ui->label_image_viewer->setStyleSheet("QLabel { color: #666; font-size: 50px; background-color: #f8f9fa; border: 2px dashed #dee2e6; border-radius: 8px; }");

    // Configuration 탭의 7개 버튼 자동 활성화
    ui->pushButton_doRobotConnect->setChecked(true);
    ui->pushButton_doGripperConnect->setChecked(true);
    ui->pushButton_do3DScannerNode->setChecked(true);
    ui->pushButton_doMaskDetectionNode->setChecked(true);
    ui->pushButton_doRvizViewNode->setChecked(false);
    ui->pushButton_doScanNode->setChecked(true);
    ui->pushButton_doMatchingNode->setChecked(true);
    ui->pushButton_doKeyCode->setChecked(true);
    ui->pushButton_doKeyCodeHolder->setChecked(true);

     // UI 시작과 함께 프로그레스 다이얼로그 표시
     QTimer::singleShot(500, this, [this]() {
         // 프로그레스 다이얼로그 생성
         setupProgressDialog = new QProgressDialog("Robot Auto Setup in progress...", "Cancel", 0, 100, this);
         setupProgressDialog->setWindowTitle("Robot Auto Setup");
         setupProgressDialog->setWindowModality(Qt::ApplicationModal);  // 모달 다이얼로그
         setupProgressDialog->setCancelButton(nullptr);  // 취소 버튼 제거
         setupProgressDialog->setAutoClose(false);
         setupProgressDialog->setAutoReset(false);
         setupProgressDialog->setValue(0);
         
         // 다이얼로그 크기 및 위치 설정
         setupProgressDialog->setMinimumSize(600, 200);  // 최소 크기 설정
         setupProgressDialog->resize(1200, 600);          // 기본 크기 설정
         
         // 스타일시트 설정 - 글자를 흰색으로
         setupProgressDialog->setStyleSheet(
             "QProgressDialog {"
             "    background-color: #2b2b2b;"
             "    color: white;"
             "    font-size: 16px;"
             "    font-weight: bold;"
             "}"
             "QProgressDialog QLabel {"
             "    color: white;"
             "    font-size: 18px;"
             "    font-weight: bold;"
             "}"
             "QProgressBar {"
             "    background-color: #404040;"
             "    border: 2px solid #555555;"
             "    border-radius: 5px;"
             "    text-align: center;"
             "    color: white;"
             "    font-weight: bold;"
             "}"
             "QProgressBar::chunk {"
             "    background-color: #4CAF50;"
             "    border-radius: 3px;"
             "}"
         );
         
         // 항상 맨 앞에 표시되도록 설정
         setupProgressDialog->setWindowFlags(Qt::Dialog | Qt::WindowStaysOnTopHint | Qt::WindowTitleHint);
         setupProgressDialog->setWindowModality(Qt::ApplicationModal);
         
         // 화면 중앙에 위치시키기
         QRect screenGeometry = QApplication::desktop()->screenGeometry();
         int x = (screenGeometry.width() - setupProgressDialog->width()) / 2;
         int y = (screenGeometry.height() - setupProgressDialog->height()) / 2;
         setupProgressDialog->move(x, y);
         
         setupProgressDialog->show();
         setupProgressDialog->raise();       // 맨 앞으로
         setupProgressDialog->activateWindow(); // 활성화
         
         ROS_LOG_WARN("[%s] Robot Auto Setup started automatically", __func__);
     });

     // 각 버튼의 클릭 이벤트 트리거 (프로그레스와 함께)
     QTimer::singleShot(600, this, [this]() {
         setupProgressDialog->setLabelText("Starting Robot Controller...");
         setupProgressDialog->setValue(10);
         setupProgressDialog->raise();       // 맨 앞으로
         setupProgressDialog->activateWindow(); // 활성화
         pushButtonRunRobotController();
     });
     
     QTimer::singleShot(700, this, [this]() {
         setupProgressDialog->setLabelText("Starting Gripper Node...");
         setupProgressDialog->setValue(20);
         setupProgressDialog->raise();       // 그리퍼 UI가 뜰 수 있으므로 맨 앞으로
         setupProgressDialog->activateWindow(); // 활성화
         pushButtonRunGripperNode();
     });
     
     QTimer::singleShot(800, this, [this]() {
         setupProgressDialog->setLabelText("Starting 3D Scanner Node...");
         setupProgressDialog->setValue(30);
         pushButtonRun3DScannerNode();
     });
     
     QTimer::singleShot(900, this, [this]() {
         setupProgressDialog->setLabelText("Starting Mask Detection Node...");
         setupProgressDialog->setValue(40);
         pushButtonRunMaskDetectionNode();
     });
     
     QTimer::singleShot(1000, this, [this]() {
         setupProgressDialog->setLabelText("Starting RViz View Node...");
         setupProgressDialog->setValue(50);
         setupProgressDialog->raise();       // RViz가 뜰 수 있으므로 맨 앞으로
         setupProgressDialog->activateWindow(); // 활성화
         pushButtonRunRvizViewNode();
     });
     
     QTimer::singleShot(1100, this, [this]() {
         setupProgressDialog->setLabelText("Starting Scan Node...");
         setupProgressDialog->setValue(60);
         pushButtonRunScanNode();
     });
     
     QTimer::singleShot(1200, this, [this]() {
         setupProgressDialog->setLabelText("Starting Matching Node...");
         setupProgressDialog->setValue(60);
         setupProgressDialog->raise();       // 맨 앞으로
         setupProgressDialog->activateWindow(); // 활성화
         pushButtonRunMatchingNode();
         pushButtonRunKeyCode();
         pushButtonRunKeyCodeHolder();
     });
     
     // 중간중간 다이얼로그가 맨 앞에 유지되도록 추가 보장
     QTimer::singleShot(2000, this, [this]() {
         if (setupProgressDialog) {
             setupProgressDialog->raise();
             setupProgressDialog->activateWindow();
         }
     });
     
     QTimer::singleShot(4000, this, [this]() {
         if (setupProgressDialog) {
             setupProgressDialog->raise();
             setupProgressDialog->activateWindow();
         }
     });
     
     QTimer::singleShot(6000, this, [this]() {
         if (setupProgressDialog) {
             setupProgressDialog->raise();
             setupProgressDialog->activateWindow();
         }
     });

    // 순차적 버튼 클릭을 위한 타이머 설정
#if IS_PLC_COMMUNICATION
    // REAL 모드: 기존 타이밍 그대로
    QTimer::singleShot(8000, this, [this]() {
        setupProgressDialog->setLabelText("Stopping Robot...");
        setupProgressDialog->setValue(70);
        ui->pushButton_stop->click();
        ROS_LOG_WARN("[%s] Stop button auto-clicked", __func__);
    });

    QTimer::singleShot(10000, this, [this]() {
        setupProgressDialog->setLabelText("Making Tool Weight...");
        setupProgressDialog->setValue(75);
        ui->pushButton_MakeToolWeight->click();
        ROS_LOG_WARN("[%s] Make Tool Weight button auto-clicked", __func__);
    });

    QTimer::singleShot(11000, this, [this]() {
        setupProgressDialog->setLabelText("Setting Tool Weight...");
        setupProgressDialog->setValue(80);
        ui->pushButton_SetToolWeight->click();
        ROS_LOG_WARN("[%s] Set Tool Weight button auto-clicked", __func__);
    });

    QTimer::singleShot(12000, this, [this]() {
        setupProgressDialog->setLabelText("Adding TCP Preset...");
        setupProgressDialog->setValue(90);
        ui->pushButtonAddTcpPreset->click();
        ROS_LOG_WARN("[%s] Add TCP Preset button auto-clicked", __func__);
    });

    QTimer::singleShot(14000, this, [this]() {
        setupProgressDialog->setLabelText("Setting Auto Mode...");
        setupProgressDialog->setValue(95);
        ui->pushButton_auto_mode->click();
        ROS_LOG_WARN("[%s] Auto mode button auto-clicked", __func__);
    });

    QTimer::singleShot(18000, this, [this]() {
        bringUIToFront();
    });

    QTimer::singleShot(19000, this, [this]() {
        setupProgressDialog->setLabelText("Setting Auto Mode...");
        setupProgressDialog->setValue(100);
        ui->pushButton_stop->click();
        QTimer::singleShot(1000, this, [this]() {
            if (setupProgressDialog) {
                setupProgressDialog->close();
                delete setupProgressDialog;
                setupProgressDialog = nullptr;
            }
            ROS_LOG_WARN("[%s] Robot Auto Setup completed successfully", __func__);
        });
    });
#else
    // SIM 모드: ToolWeight/TCP 스킵, 타이밍 단축
    QTimer::singleShot(3000, this, [this]() {
        setupProgressDialog->setLabelText("Stopping Robot...");
        setupProgressDialog->setValue(70);
        ui->pushButton_stop->click();
        ROS_LOG_WARN("[%s] Stop button auto-clicked", __func__);
    });

    QTimer::singleShot(4000, this, [this]() {
        setupProgressDialog->setLabelText("Setting Auto Mode...");
        setupProgressDialog->setValue(95);
        ui->pushButton_auto_mode->click();
        ROS_LOG_WARN("[%s] Auto mode button auto-clicked", __func__);
    });

    QTimer::singleShot(5000, this, [this]() {
        bringUIToFront();
        setupProgressDialog->setLabelText("Setup Complete");
        setupProgressDialog->setValue(100);
        QTimer::singleShot(500, this, [this]() {
            if (setupProgressDialog) {
                setupProgressDialog->close();
                delete setupProgressDialog;
                setupProgressDialog = nullptr;
            }
            ROS_LOG_WARN("[%s] Robot Auto Setup completed successfully", __func__);
        });
    });
#endif

    // Robot ON/OFF 버튼 연결
    QObject::connect(ui->pushButton_RobotON, &QPushButton::clicked, [this]() {
#if DRFL_CONTROL
        _taskWindow->getTaskManager()->_qnode->drflSetRobotControl(3);
        ROS_LOG_WARN("ROBOT ON!!", __func__);
#endif
    });

    QObject::connect(ui->pushButton_RobotOFF, &QPushButton::clicked, [this]() {
#if DRFL_CONTROL
        _taskWindow->getTaskManager()->_qnode->drflServoOff(1);
        ROS_LOG_WARN("ROBOT OFF!!", __func__);
#endif
    });

    // Tool Weight 관련 버튼 연결
    QObject::connect(ui->pushButton_MakeToolWeight, &QPushButton::clicked, this, &MainWindow_widgetsHanyangLaunchPackage::on_pushButton_makeToolWeight_clicked);
    QObject::connect(ui->pushButton_SetToolWeight, &QPushButton::clicked, this, &MainWindow_widgetsHanyangLaunchPackage::on_pushButton_setToolWeight_clicked);

    // TCP Preset 버튼 연결
    QObject::connect(ui->pushButtonAddTcpPreset, &QPushButton::clicked, this, &MainWindow_widgetsHanyangLaunchPackage::on_pushButton_addTcpPreset_clicked);

    // --- PLC Monitoring 버튼 연결 (주석 처리) ---
    // QObject::connect(ui->pushButton_MODBUS_MonitoringPLC, &QPushButton::clicked, this, &MainWindow_widgetsHanyangLaunchPackage::on_pushButton_MODBUS_MonitoringPLC_clicked);
    // QObject::connect(ui->pushButton_MODBUS_MonitoringPLCReadAMR2RobotStatus, &QPushButton::clicked, this, &MainWindow_widgetsHanyangLaunchPackage::on_pushButton_MODBUS_MonitoringPLCReadAMR2RobotStatus_clicked);

    // 로그 메시지 연결
    connect(_taskWindow->getTaskManager()->_qnode, &QNode::sendLogMessage, this, [this](LogInfo logInfo) {
        if (ui->textEdit_log->textCursor().position() > 500) {
            ui->textEdit_log->clear();
        }
        QString line = QString::fromStdString(logInfo._log_message);
        QTextCursor cursor = ui->textEdit_log->textCursor();
        cursor.movePosition(QTextCursor::End);
        ui->textEdit_log->setTextCursor(cursor);
        QFont font = ui->textEdit_log->font();

        switch (logInfo._log_level) {
            case LogLevel::INFO: {
                QString infoHtml = "<font color=\"Blue\">";
                line = infoHtml + line;
                break;
            }
            case LogLevel::WARN: {
                QString warnHtml = "<font color=\"Orange\">";
                line = warnHtml + line;
                break;
            }
            case LogLevel::ERROR: {
                QString errorHtml = "<font color=\"Red\">";
                line = errorHtml + line;
                break;
            }
            default:
                std::cout << "no text";
                break;
        }
        QString endHtml = "</font><br>";
        line = line + endHtml;
        ui->textEdit_log->insertHtml(line);
        cursor.movePosition(QTextCursor::End);
        ui->textEdit_log->setTextCursor(cursor);
    });

    // 문자열 로그 메시지 연결
    QObject::connect(_taskWindow->getTaskManager()->_qnode, &QNode::sendLogMessageStr, this, &MainWindow_widgetsHanyangLaunchPackage::sendLogMessageStr);

    // 이미지 업데이트 연결
    connect(_taskWindow->getTaskManager()->_qnode, &QNode::UpdateImage, this, &MainWindow_widgetsHanyangLaunchPackage::UpdateImage);

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
    
    //// KeyCode 관련 버튼 추가
    QObject::connect(ui->pushButton_doKeyCode        , SIGNAL(clicked()), this, SLOT(pushButtonRunKeyCode()));
    QObject::connect(ui->pushButton_doKeyCodeHolder        , SIGNAL(clicked()), this, SLOT(pushButtonRunKeyCodeHolder()));

    // Robot Jog 버튼 연결에 디버깅 로그 추가
    ROS_LOG_WARN("[%s] Connecting Robot Jog button signal", __func__);
    QObject::connect(ui->pushButton_doRobotHandling        , SIGNAL(clicked()), this, SLOT(pushButtonRunRobotHandling()));
    ROS_LOG_WARN("[%s] Robot Jog button signal connected", __func__);

    QObject::connect(ui->pushButton_terminateAllNode        , SIGNAL(clicked()), this, SLOT(terminateQProcessAndRosNode()));


    updateProcessStatusIcon(false, "robot");
    updateProcessStatusIcon(false, "gripper");
    updateProcessStatusIcon(false, "3d scanner");
    updateProcessStatusIcon(false, "mask detection");
    updateProcessStatusIcon(false, "rviz view");
    updateProcessStatusIcon(false, "scan node");
    updateProcessStatusIcon(false, "matching node");
    updateProcessStatusIcon(false, "keycode");
    updateProcessStatusIcon(false, "keycode holder");

    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    //// Qt Main timer
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow_widgetsHanyangLaunchPackage::timerCallback);
    timer_->start(100);

    //// Qt sub timer
    status_update_timer_ = new QTimer(this);
    connect(status_update_timer_, &QTimer::timeout, this, &MainWindow_widgetsHanyangLaunchPackage::statusTimerCallback);
    // status_update_timer_->start(200);
    status_update_timer_->start(1000);

    ROS_LOG_WARN("[%s] Button connections initialized", __func__);

    // Stop 버튼 연결
    QObject::connect(ui->pushButton_stop, &QPushButton::clicked, [this]() {
        ROS_LOG_WARN("[%s] Stop button clicked", __func__);
        _taskWindow->getTaskManager()->_qnode->is_llm_task_fin = true;
        _taskWindow->getTaskManager()->_qnode->is_task_mode_ = false;
#if DRFL_CONTROL
        _taskWindow->getTaskManager()->_qnode->drflMoveStop(2);  // 2: STOP_TYPE_QUICK
#else
        _taskWindow->getTaskManager()->_qnode->stopRobot();
#endif
        ROS_LOG_WARN("[%s] Robot stopped", __func__);
    });

    // 자동/수동 모드 버튼 연결
    QObject::connect(ui->pushButton_auto_mode, &QPushButton::clicked, this, &MainWindow_widgetsHanyangLaunchPackage::on_pushButton_auto_mode_clicked);
    QObject::connect(ui->pushButton_manual_mode, &QPushButton::clicked, this, &MainWindow_widgetsHanyangLaunchPackage::on_pushButton_manual_mode_clicked);
  
    /// 로봇 세팅 자동 모드 on/off
    QObject::connect(ui->pushButton_robot_auto_setup, &QPushButton::clicked, this, &MainWindow_widgetsHanyangLaunchPackage::on_pushButton_robot_auto_setup_clicked);
    QObject::connect(ui->pushButton_robot_auto_off, &QPushButton::clicked, this, &MainWindow_widgetsHanyangLaunchPackage::on_pushButton_robot_auto_off_clicked);

    // --- PLC 체크박스 연동용 배열 초기화 (주석 처리) ---
    /*
    plc_launch_task_flag_check_box_42000_
        << ui->checkBox_42000_1
        << ui->checkBox_42000_2
        << ui->checkBox_42000_3
        << ui->checkBox_42000_4
        << ui->checkBox_42000_5
        << ui->checkBox_42000_6
        << ui->checkBox_42000_7
        << ui->checkBox_42000_8
        << ui->checkBox_42000_9
        << ui->checkBox_42000_10
        << ui->checkBox_42000_11
        << ui->checkBox_42000_12
        << ui->checkBox_42000_13
        << ui->checkBox_42000_14
        << ui->checkBox_42000_15
        << ui->checkBox_42000_16;
    plc_launch_task_flag_check_box_42002_
        << ui->checkBox_42002_1
        << ui->checkBox_42002_2
        << ui->checkBox_42002_3
        << ui->checkBox_42002_4
        << ui->checkBox_42002_5
        << ui->checkBox_42002_6
        << ui->checkBox_42002_7
        << ui->checkBox_42002_8
        << ui->checkBox_42002_9
        << ui->checkBox_42002_10
        << ui->checkBox_42002_11
        << ui->checkBox_42002_12
        << ui->checkBox_42002_13
        << ui->checkBox_42002_14
        << ui->checkBox_42002_15
        << ui->checkBox_42002_16;
    plc_launch_task_flag_check_box_41000_
        << ui->checkBox_41000_1
        << ui->checkBox_41000_2
        << ui->checkBox_41000_3
        << ui->checkBox_41000_4
        << ui->checkBox_41000_5
        << ui->checkBox_41000_6
        << ui->checkBox_41000_7
        << ui->checkBox_41000_8
        << ui->checkBox_41000_9
        << ui->checkBox_41000_10
        << ui->checkBox_41000_11
        << ui->checkBox_41000_12
        << ui->checkBox_41000_13
        << ui->checkBox_41000_14
        << ui->checkBox_41000_15
        << ui->checkBox_41000_16;

    plc_launch_task_flag_check_box_41002_
        << ui->checkBox_41002_1
        << ui->checkBox_41002_2
        << ui->checkBox_41002_3
        << ui->checkBox_41002_4
        << ui->checkBox_41002_5
        << ui->checkBox_41002_6
        << ui->checkBox_41002_7
        << ui->checkBox_41002_8
        << ui->checkBox_41002_9
        << ui->checkBox_41002_10
        << ui->checkBox_41002_11
        << ui->checkBox_41002_12
        << ui->checkBox_41002_13
        << ui->checkBox_41002_14
        << ui->checkBox_41002_15
        << ui->checkBox_41002_16;

    // --- PLC 제어 기능 연결 (주석 처리) ---
    QObject::connect(ui->radioButton_select_plc_address_write_1     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcWriteAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_write_3     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcWriteAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_write_4     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcWriteAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_write_5     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcWriteAddressClickedCallback()));

    QObject::connect(ui->radioButton_select_plc_address_read_1     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcReadAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_read_3     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcReadAddressClickedCallback()));

    QObject::connect(ui->pushButton_MODBUS_WRITE_DATA_BOOL_TYPE, SIGNAL(clicked()), this, SLOT(pushButtonModbusWriteDataBoolType()));
    QObject::connect(ui->pushButton_MODBUS_READ_DATA_BOOL_TYPE, SIGNAL(clicked()), this, SLOT(pushButtonModbusReadDataBoolType()));
    */

    // Enlarge fonts for task text labels and initialize indicator inactive styles
    {
        QFont taskLabelFont;
        taskLabelFont.setPointSize(14);
        taskLabelFont.setBold(true);

        auto setLabelFontIfExists = [&](QLabel* label){ if (label) label->setFont(taskLabelFont); };
        setLabelFontIfExists(ui->label_text_right_detach_coupler);
        setLabelFontIfExists(ui->label_text_right_attach_lid_cap);
        setLabelFontIfExists(ui->label_text_right_detach_lid_cap);
        setLabelFontIfExists(ui->label_text_right_attach_coupler);
        setLabelFontIfExists(ui->label_text_left_detach_coupler);
        setLabelFontIfExists(ui->label_text_left_attach_lid_cap);
        setLabelFontIfExists(ui->label_text_left_detach_lid_cap);
        setLabelFontIfExists(ui->label_text_left_attach_coupler);

        const QString indicatorInactiveStyle = "background-color: gray; border: 3px solid darkgray; border-radius: 16px;";
        auto setIndicatorInactive = [&](QLabel* label){ if (label) label->setStyleSheet(indicatorInactiveStyle); };
        setIndicatorInactive(ui->label_status_right_detach_coupler);
        setIndicatorInactive(ui->label_status_right_attach_lid_cap);
        setIndicatorInactive(ui->label_status_right_detach_lid_cap);
        setIndicatorInactive(ui->label_status_right_attach_coupler);
        setIndicatorInactive(ui->label_status_left_detach_coupler);
        setIndicatorInactive(ui->label_status_left_attach_lid_cap);
        setIndicatorInactive(ui->label_status_left_detach_lid_cap);
        setIndicatorInactive(ui->label_status_left_attach_coupler);
    }
}


MainWindow_widgetsHanyangLaunchPackage::~MainWindow_widgetsHanyangLaunchPackage()
{
    // qDebug() << "[Destructor] MainWindow_widgetsHanyangLaunchPackage called";
    
    // 프로그레스 다이얼로그 정리
    if (setupProgressDialog) {
        delete setupProgressDialog;
        setupProgressDialog = nullptr;
    }
    
    terminateQProcessAndRosNode();
    delete ui;
    rclcpp::shutdown();
}

void MainWindow_widgetsHanyangLaunchPackage::closeEvent(QCloseEvent *event)
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

void MainWindow_widgetsHanyangLaunchPackage::initializeDialog() {
    updateProcessStatusIcon(false, "robot");
    updateProcessStatusIcon(false, "gripper");
    updateProcessStatusIcon(false, "3d scanner");
    updateProcessStatusIcon(false, "mask detection");
    updateProcessStatusIcon(false, "rviz view");
    updateProcessStatusIcon(false, "scan node");
    updateProcessStatusIcon(false, "matching node");
    updateProcessStatusIcon(false, "keycode");
    updateProcessStatusIcon(false, "keycode holder");
}

void MainWindow_widgetsHanyangLaunchPackage::timerCallback() {
    // --- PLC 신호와 체크박스 연동 (주석 처리) ---
    /*
    if (_taskWindow && _taskWindow->getTaskManager() && _taskWindow->getTaskManager()->_qnode) {
        for (int i = 0; i < plc_launch_task_flag_check_box_42000_.size(); ++i) {
            plc_launch_task_flag_check_box_42000_[i]->setChecked(_taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[i]);
        }
        for (int i = 0; i < plc_launch_task_flag_check_box_42002_.size(); ++i) {
            plc_launch_task_flag_check_box_42002_[i]->setChecked(_taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[i]);
        }
        for (int i = 0; i < plc_launch_task_flag_check_box_41000_.size(); ++i) {
            plc_launch_task_flag_check_box_41000_[i]->setChecked(_taskWindow->getTaskManager()->_qnode->plc_status_word_41000_[i]);
        }
        for (int i = 0; i < plc_launch_task_flag_check_box_41002_.size(); ++i) {
            plc_launch_task_flag_check_box_41002_[i]->setChecked(_taskWindow->getTaskManager()->_qnode->plc_status_word_41002_[i]);
        }
    }
    */
}

void MainWindow_widgetsHanyangLaunchPackage::statusTimerCallback() {
    // --- PLC 연결 상태 확인 (주석 처리 - PLC 연결 확인 없이 UI 켜짐) ---
    /*
    if (_taskWindow && _taskWindow->getTaskManager() && _taskWindow->getTaskManager()->_qnode && _taskWindow->getTaskManager()->_qnode->_modbus) {
        if (modbus_status_ == -1) {
            modbus_status_ = 0;
        }
    } else {
        modbus_status_ = -1;
    }

    QString styleSheet;
    if (modbus_status_ == 0) {
        styleSheet = "background-color: green; border-radius: 15px; border: 2px solid darkgreen;";
    } else if (modbus_status_ == -1) {
        styleSheet = "background-color: red; border-radius: 15px; border: 2px solid darkred;";
    } else {
        styleSheet = "background-color: red; border-radius: 15px; border: 2px solid darkred;";
    }

    if (ui->label_modbus_status) {
        ui->label_modbus_status->setStyleSheet(styleSheet);
    } else {
        qDebug() << "ERROR: ui->label_modbus_status가 nullptr입니다!";
    }
    */

    // --- Task 진행 상태 인디케이터 업데이트 (변화 감지 시에만) ---
    const QString activeStyle = "background-color: #1e90ff; border: 3px solid #104e8b; border-radius: 16px;"; // 파란불
    const QString inactiveStyle = "background-color: gray; border: 3px solid darkgray; border-radius: 16px;";      // 회색

    if (!(_taskWindow && _taskWindow->getTaskManager() && _taskWindow->getTaskManager()->_qnode)) return;

    const bool isTaskRunning = _taskWindow->getTaskManager()->_qnode->is_task_mode_;
    const QString currentTask = QString::fromStdString(_taskWindow->getTaskManager()->_qnode->current_task_log_);

    if (isTaskRunning == prev_is_task_mode_ && currentTask == prev_task_log_) {
        return; // 변화 없음: 갱신 불필요
    }

    auto setInactiveAll = [this, &inactiveStyle]() {
        if (ui->label_status_right_detach_coupler) ui->label_status_right_detach_coupler->setStyleSheet(inactiveStyle);
        if (ui->label_status_right_attach_lid_cap) ui->label_status_right_attach_lid_cap->setStyleSheet(inactiveStyle);
        if (ui->label_status_right_detach_lid_cap) ui->label_status_right_detach_lid_cap->setStyleSheet(inactiveStyle);
        if (ui->label_status_right_attach_coupler) ui->label_status_right_attach_coupler->setStyleSheet(inactiveStyle);
        if (ui->label_status_left_detach_coupler)  ui->label_status_left_detach_coupler->setStyleSheet(inactiveStyle);
        if (ui->label_status_left_attach_lid_cap)  ui->label_status_left_attach_lid_cap->setStyleSheet(inactiveStyle);
        if (ui->label_status_left_detach_lid_cap)  ui->label_status_left_detach_lid_cap->setStyleSheet(inactiveStyle);
        if (ui->label_status_left_attach_coupler)  ui->label_status_left_attach_coupler->setStyleSheet(inactiveStyle);
    };

    setInactiveAll();

    if (isTaskRunning) {
        if (currentTask == "right_task_detach_coupler" && ui->label_status_right_detach_coupler) {
            ui->label_status_right_detach_coupler->setStyleSheet(activeStyle);
        } else if (currentTask == "right_task_attach_lid_cap" && ui->label_status_right_attach_lid_cap) {
            ui->label_status_right_attach_lid_cap->setStyleSheet(activeStyle);
        } else if (currentTask == "right_task_detach_lid_cap" && ui->label_status_right_detach_lid_cap) {
            ui->label_status_right_detach_lid_cap->setStyleSheet(activeStyle);
        } else if (currentTask == "right_task_attach_coupler" && ui->label_status_right_attach_coupler) {
            ui->label_status_right_attach_coupler->setStyleSheet(activeStyle);
        } else if (currentTask == "left_task_detach_coupler" && ui->label_status_left_detach_coupler) {
            ui->label_status_left_detach_coupler->setStyleSheet(activeStyle);
        } else if (currentTask == "left_task_attach_lid_cap" && ui->label_status_left_attach_lid_cap) {
            ui->label_status_left_attach_lid_cap->setStyleSheet(activeStyle);
        } else if (currentTask == "left_task_detach_lid_cap" && ui->label_status_left_detach_lid_cap) {
            ui->label_status_left_detach_lid_cap->setStyleSheet(activeStyle);
        } else if (currentTask == "left_task_attach_coupler" && ui->label_status_left_attach_coupler) {
            ui->label_status_left_attach_coupler->setStyleSheet(activeStyle);
        }
    }

    prev_is_task_mode_ = isTaskRunning;
    prev_task_log_ = currentTask;
}


void MainWindow_widgetsHanyangLaunchPackage::registerManagedProcess(QProcess*& processRef)
{
    if (!managedProcesses.contains(&processRef)) {
        managedProcesses.append(&processRef);
    }
}

void MainWindow_widgetsHanyangLaunchPackage::runGenericNode(
    QProcess*& processRef,
    QPushButton* button,
    QComboBox* comboBox,
    QTextEdit* logView,
    const QMap<QString, QString>& commandMap,
    const QString& processType)
{
    _taskWindow->getTaskManager()->_qnode->is_package_launched_ = true;

    static QString sessionTime;  // 실행 세션 폴더 이름 (첫 실행 시 고정)
    if (sessionTime.isEmpty()) {
        sessionTime = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    }

    QString command;
    QString nodeName;

    // 이미 실행 중이면 종료
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
        if (statusIconLabelMap.contains(processType)) {
            updateProcessStatusIcon(false, processType);
        }
        return;
    }

    // 실행 준비
    if (button->isChecked()) {
        QString selectedText = comboBox ? comboBox->currentText() : "none";
        if (commandMap.contains(selectedText)) {
            command = commandMap[selectedText];
            nodeName = selectedText;
            if (selectedText != "none") {
                ROS_LOG_WARN("%s selected", selectedText.toStdString().c_str());
            } else {
                // ROS_LOG_INFO("Starting %s", processType.toStdString().c_str());
            }
        } else {
            ROS_LOG_WARN("Unknown selection");
        }
    }

    if (!command.isEmpty()) {
        logView->clear();
        logView->append("[" + nodeName + "] Node started...\n");

        processRef = new QProcess(this);
        registerManagedProcess(processRef);
        processRef->setProcessChannelMode(QProcess::MergedChannels);

        // 로그 파일 경로: ws/src/Developer_log/세션시간/
        QString wsRoot = QDir::homePath() + "/hanyang_ui_ws";
        QString logDir = wsRoot + "/src/Developer_log/" + sessionTime;
        QDir().mkpath(logDir);

        QString logFileName = QString("%1/%2.log").arg(logDir, processType);

        QFile* logFile = new QFile(logFileName, this);
        QTextStream* logStream = nullptr;
        if (logFile->open(QIODevice::Append | QIODevice::Text)) {
            logStream = new QTextStream(logFile);

            // 노드 시작 구분선 기록
            (*logStream) << "========== [Node Started] " 
                         << processType 
                         << " @ " 
                         << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss") 
                         << " ==========\n";
            logStream->flush();
        } else {
            logView->append("[ERROR] Cannot open log file: " + logFileName);
            delete logFile;
            logFile = nullptr;
        }

        // STDOUT
        connect(processRef, &QProcess::readyReadStandardOutput, this, [=]() mutable {
            QString output = processRef->readAllStandardOutput();
            QString line = QString("[%1] %2")
                               .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"))
                               .arg(output.trimmed());
            logView->append(line);
            logView->moveCursor(QTextCursor::End);
            if (logStream) {
                (*logStream) << line << "\n";
                logStream->flush();
            }
        });

        // STDERR
        connect(processRef, &QProcess::readyReadStandardError, this, [=]() mutable {
            QString error = processRef->readAllStandardError();
            QString line = QString("[%1] [ERROR] %2")
                               .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"))
                               .arg(error.trimmed());
            logView->append(line);
            logView->moveCursor(QTextCursor::End);
            if (logStream) {
                (*logStream) << line << "\n";
                logStream->flush();
            }
        });

        // 종료 처리
        connect(processRef, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                this, [this, &processRef, nodeName, logView, processType, logFile, logStream]
                (int exitCode, QProcess::ExitStatus) mutable {
            QString msg = QString("[%1] controller exited (code: %2)")
                              .arg(nodeName)
                              .arg(exitCode);
            logView->append(msg);
            logView->moveCursor(QTextCursor::End);

            if (logStream) {
                // 노드 종료 구분선 기록
                (*logStream) << "========== [Node Exited] " 
                             << processType 
                             << " @ " 
                             << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss") 
                             << " (code: " << exitCode << ") ==========\n";
                logStream->flush();
                delete logStream;
            }
            if (logFile) {
                logFile->close();
                delete logFile;
            }

            // 기존 강제 종료 로직
            QMap<QString, QStringList> nodeToRosNameList = {
                {"Universal Robots UR10e", {"/UR10e"}},
                {"Fairino FR5", {"/FR5"}},
                {"Doosan M1013", {"/dsr01/controller_manager", "/dsr01/dsr_hw_interface2", "/dsr01/robot_state_publisher", "/dsr01/spawner_dsr_controller2", "/dsr01/spawner_joint_state_broadcaster", "/dsr01/virtual_node"}},
                {"Doosan E0509", {"/dsr01/controller_manager", "/dsr01/dsr_hw_interface2", "/dsr01/robot_state_publisher", "/dsr01/spawner_dsr_controller2", "/dsr01/spawner_joint_state_broadcaster", "/dsr01/virtual_node"}},
                {"Koras Gripper", {"/DATC_Control_Interface"}},
                {"[Scanner Type] Zivid2+ MR60", {"/scanner_node", "/scanner_node_sub"}},
                {"[Scanner Type] Zivid2+ M130", {"/scanner_node", "/scanner_node_sub"}},
                {"[Scanner Type] Rvbust RVC I2370", {"/scanner_node", "/scanner_node_sub"}},
                {"[Scanner Type] Rvbust RVC P31300", {"/scanner_node", "/scanner_node_sub"}},
                {"AI Module (SAM)", {"/sam_zivid"}},
                {"No AI Module", {"/sam_zivid"}},
                {"Visualize all", {"/rviz2_v1", "/rviz2_v2", "/rviz2_v5", "/rviz2_v8"}},
                {"[Scan Node] Zivid2+ MR60", {"/zivid_scan"}},
                {"[Scan Node] Zivid2+ M130", {"/zivid_scan"}},
                {"[Scan Node] Rvbust RVC I2370", {"/zivid_scan"}},
                {"[Scan Node] Rvbust RVC P31300", {"/zivid_scan"}},
                {"Matching", {"/template_matching_node"}},
                {"keycode", {"/detect_red_dot_node"}},
                {"keycode holder", {"/detect_red_dot_node_for_holder"}}
            };

            if (nodeToRosNameList.contains(nodeName)) {
                rosNodeKillByNameList(nodeToRosNameList[nodeName]);
            } else {
                qDebug() << "[runGenericNode] Unknown node name for ROS kill:" << nodeName;
            }

            processRef->deleteLater();
            processRef = nullptr;
            if (statusIconLabelMap.contains(processType)) {
            updateProcessStatusIcon(false, processType);
        }
        });

        // 실행 시작
        processRef->start("bash", QStringList() << "-c" << "setsid bash -c '" + command + "'");
        if (statusIconLabelMap.contains(processType)) {
            updateProcessStatusIcon(true, processType);
        }
    }
}


void MainWindow_widgetsHanyangLaunchPackage::pushButtonRunRobotController() {
    QMap<QString, QString> command_list = {
        {"Universal Robots UR10e", "source ~/robot_control_ws/install/setup.bash && ros2 run ur10e_controller ur10e_controller 192.168.0.12"},
        {"Fairino FR5", "source ~/robot_control_ws/install/setup.bash && export LD_LIBRARY_PATH=~/robot_control_ws/src/fr5_control_package/fr_controller/include/fr_controller/libfairino/lib/libfairino:$LD_LIBRARY_PATH && ros2 run fr_controller fr_controller"},
        {"Doosan M1013", "source $ROBOT_BASE_DIR/doosan_ros2_ws/install/setup.bash && ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m1013 gui:=false"},
        {"Doosan E0509", "source $ROBOT_BASE_DIR/doosan_ros2_ws/install/setup.bash && ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=e0509"}
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
void MainWindow_widgetsHanyangLaunchPackage::pushButtonRunGripperNode() {
    QMap<QString, QString> command_list = {
        {"Koras Gripper", "source $ROBOT_BASE_DIR/gripper_ws/install/setup.bash && ros2 run kr_gcs_ui kr_gcs_ui --no-ui"}
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


void MainWindow_widgetsHanyangLaunchPackage::pushButtonRun3DScannerNode() {
    QMap<QString, QString> command_list = {
        {"[Scanner Type] Zivid2+ MR60", "source $ROBOT_BASE_DIR/hanyang_matching_ws/install/setup.bash && ros2 run hanyang_zivid_scanner_node zivid_topic_node.py"},
        {"[Scanner Type] Zivid2+ M130", "source $ROBOT_BASE_DIR/hanyang_matching_ws/install/setup.bash && ros2 run hanyang_zivid_scanner_node zivid_topic_node.py"},
        {"[Scanner Type] Rvbust RVC I2370", "source $ROBOT_BASE_DIR/hanyang_matching_ws/install/setup.bash && ros2 run scanner_node zivid2mr60_topic_node.py"},
        {"[Scanner Type] Rvbust RVC P31300", "source $ROBOT_BASE_DIR/hanyang_matching_ws/install/setup.bash && ros2 run scanner_node zivid2mr60_topic_node.py"}
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


void MainWindow_widgetsHanyangLaunchPackage::pushButtonRunMaskDetectionNode() {
    QMap<QString, QString> command_list = {
        {"AI Module (SAM)", "source $ROBOT_BASE_DIR/hanyang_matching_ws/install/setup.bash && ros2 run hanyang_sam_node sam_node.py"},
        {"No AI Module", "source $ROBOT_BASE_DIR/hanyang_matching_ws/install/setup.bash && ros2 run hanyang_sam_node sam_node.py"}
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

void MainWindow_widgetsHanyangLaunchPackage::pushButtonRunRvizViewNode() {
    QMap<QString, QString> command_list = {
        {"Visualize all", "source $ROBOT_BASE_DIR/hanyang_matching_ws/install/setup.bash && ros2 launch hanyang_matching_visualizer visualize_zvd_cad_grasp.xml"}
    };

    runGenericNode(
        rvizViewNodeProcess,
        ui->pushButton_doRvizViewNode,
        ui->comboBox_RvizViewLists,
        ui->textEdit_log_RvizView,
        command_list,
        "rviz view"
    );
}

void MainWindow_widgetsHanyangLaunchPackage::pushButtonRunScanNode() {
    QMap<QString, QString> command_list = {
        {"[Scan Node] Zivid2+ MR60", "source $ROBOT_BASE_DIR/hanyang_matching_ws/install/setup.bash && ros2 run hanyang_matching_process zivid_scan_node"},
        {"[Scan Node] Zivid2+ M130", "source $ROBOT_BASE_DIR/hanyang_matching_ws/install/setup.bash && ros2 run hanyang_matching_process zivid_scan_node"},
        {"[Scan Node] Rvbust RVC I2370", "source $ROBOT_BASE_DIR/bin_picking_ws/install/setup.bash && ros2 run bin_picking_process zivid_scan_node"},
        {"[Scan Node] Rvbust RVC P31300", "source $ROBOT_BASE_DIR/bin_picking_ws/install/setup.bash && ros2 run bin_picking_process zivid_scan_node"}
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

void MainWindow_widgetsHanyangLaunchPackage::pushButtonRunMatchingNode() {
    QMap<QString, QString> command_list = {
        {"Matching", "source $ROBOT_BASE_DIR/hanyang_matching_ws/install/setup.bash && ros2 run hanyang_matching_process matching_node"}
    };

    runGenericNode(
        matchingNodeProcess,
        ui->pushButton_doMatchingNode,
        ui->comboBox_matchingNodeLists,
        ui->textEdit_log_MatchingNodeLogView,
        command_list,
        "matching node"
    );
}

void MainWindow_widgetsHanyangLaunchPackage::pushButtonRunKeyCode() {
    QMap<QString, QString> command_list = {
        {"none", "source $ROBOT_BASE_DIR/keycode_ws/install/setup.bash && ros2 run keyring_detection detect_red_dot_node_for_drum"}
    };

    runGenericNode(
        keyCodeNodeProcess,
        ui->pushButton_doKeyCode,
        nullptr,  // 콤보박스 없음
        ui->textEdit_log_KeyCodeView,
        command_list,
        "keycode"
    );
}

void MainWindow_widgetsHanyangLaunchPackage::pushButtonRunKeyCodeHolder() {
    QMap<QString, QString> command_list = {
        {"none", "source $ROBOT_BASE_DIR/keycode_ws/install/setup.bash && ros2 run keyring_detection detect_red_dot_node_for_holder"}
    };

    runGenericNode(
        keyCodeHolderNodeProcess,
        ui->pushButton_doKeyCodeHolder,
        nullptr,  // 콤보박스 없음
        ui->textEdit_log_KeyCodeHolderView,
        command_list,
        "keycode holder"
    );
}



void MainWindow_widgetsHanyangLaunchPackage::pushButtonRunRobotHandling() {
    ROS_LOG_WARN("[%s] Robot Jog button clicked - function entered", __func__);
    
    // 디버깅을 위한 버튼 상태 확인
    ROS_LOG_WARN("[%s] Button enabled: %s", __func__, ui->pushButton_doRobotHandling->isEnabled() ? "true" : "false");
    ROS_LOG_WARN("[%s] Button visible: %s", __func__, ui->pushButton_doRobotHandling->isVisible() ? "true" : "false");
    ROS_LOG_WARN("[%s] Button checkable: %s", __func__, ui->pushButton_doRobotHandling->isCheckable() ? "true" : "false");
    ROS_LOG_WARN("[%s] Button checked: %s", __func__, ui->pushButton_doRobotHandling->isChecked() ? "true" : "false");
    
    // checkable 속성을 제거했으므로 항상 실행
    _taskWindow->getTaskManager()->_qnode->is_package_launched_ = true;
    ROS_LOG_WARN("[%s] Setting is_package_launched_ to true", __func__);
    
    emit openDeveloperWindow();
    ROS_LOG_WARN("[%s] Emitted openDeveloperWindow signal", __func__);
}


// void MainWindow_widgetsHanyangLaunchPackage::updateProcessStatusIcon(bool isRunning, const QString& target)
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

void MainWindow_widgetsHanyangLaunchPackage::setStatusIconLabel(const QString& name, QLabel* label)
{
    statusIconLabelMap[name] = label;
}

void MainWindow_widgetsHanyangLaunchPackage::updateProcessStatusIcon(bool isRunning, const QString& target)
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


void MainWindow_widgetsHanyangLaunchPackage::rosNodeKillByName(const QString& rosNodeName)
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
        {"/template_matching_node", "matching_node"},
        {"/detect_red_dot_node", "detect_red_dot_node"},
        {"/detect_red_dot_node_for_holder", "detect_red_dot_node_for_holder"}
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

void MainWindow_widgetsHanyangLaunchPackage::rosNodeKillByNameList(const QStringList& nodeNames)
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

void MainWindow_widgetsHanyangLaunchPackage::terminateQProcessAndRosNode()
{
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
    tryKill(keyCodeNodeProcess, "keyCodeNodeProcess");
    tryKill(keyCodeHolderNodeProcess, "keyCodeHolderNodeProcess");
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
    // KeyCode Nodes
    rosNodeKillByName("/detect_red_dot_node");
    rosNodeKillByName("/detect_red_dot_node_for_holder");

    //// 아래는 koras_system 패키지에 속하는 노드
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
    ui->pushButton_doKeyCode->setChecked(false);
    ui->pushButton_doKeyCodeHolder->setChecked(false);

}

void MainWindow_widgetsHanyangLaunchPackage::sendLogMessageStr(std::string &str) {
    QString line = QString::fromStdString(str);
    QTextCursor cursor = ui->textEdit_log->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->textEdit_log->setTextCursor(cursor);
    QFont font = ui->textEdit_log->font();

    QString infoHtml = "<font color=\"Blue\">";
    line = infoHtml + line;

    QString endHtml = "</font><br>";
    line = line + endHtml;
    ui->textEdit_log->insertHtml(line);
    cursor.movePosition(QTextCursor::End);
    ui->textEdit_log->setTextCursor(cursor);
}

void MainWindow_widgetsHanyangLaunchPackage::on_pushButton_makeToolWeight_clicked()
{
    ROS_LOG_WARN("[%s] Make Tool Weight button clicked", __func__);
    
    // 레이블 색상 변경
    ui->label_makeToolWeightStatus->setStyleSheet("background-color: green; border-radius: 10px;");
    
#if DRFL_CONTROL
    _taskWindow->getTaskManager()->_qnode->drflSetRobotMode(0);

    bool success = _taskWindow->getTaskManager()->_qnode->drflConfigCreateTool(
        "t1",        // tool name
        3.920,            // tool weight in kg
        // 2.520,            // tool weight in kg
        std::array<double, 3>{0.015820, -0.035930, 0.093440},   // center of gravity (COG) in meters
        std::array<double, 6>{0.00, 0.00, 0.00, 0.00, 0.00, 0.00} // tool inertia (kg·m²) - 예시 값
    );


    if (success) {
        ROS_LOG_WARN("[%s] drflConfigCreateTool Completed!", __func__);
    } else {
        ROS_LOG_ERROR("[%s] Failed to Create Tool Weight", __func__);
        ui->label_makeToolWeightStatus->setStyleSheet("background-color: red; border-radius: 10px;");
    }
    _taskWindow->getTaskManager()->_qnode->drflSetRobotMode(1);
#endif
}

void MainWindow_widgetsHanyangLaunchPackage::on_pushButton_setToolWeight_clicked()
{
    ROS_LOG_WARN("[%s] Set Tool Weight button clicked", __func__);
    
    // 레이블 색상 변경
    ui->label_setToolWeightStatus->setStyleSheet("background-color: green; border-radius: 10px;");
    
#if DRFL_CONTROL
    _taskWindow->getTaskManager()->_qnode->drflSetRobotMode(0);

    _taskWindow->getTaskManager()->_qnode->is_task_mode_ = false;
    bool success = _taskWindow->getTaskManager()->_qnode->drflSetCurrentTool("t1");
    if (success) {
        ROS_LOG_INFO("Tool Weight SET! /// 5KG tool(0, 0.030 , 0.132)");
    } else {
        ROS_LOG_ERROR("Failed to Set Tool Weight");
    }

    _taskWindow->getTaskManager()->_qnode->drflChangeCollisionSensitivity(1);
    if (success) {
        ROS_LOG_INFO("Collision sensitivity changed successfully.");
    } else {
        ROS_LOG_ERROR("Failed to change collision sensitivity.");
    }

    _taskWindow->getTaskManager()->_qnode->drflSetRobotMode(1);
    ROS_LOG_WARN("[%s]drflSetCurrentTool Completed!", __func__);
#endif
}

void MainWindow_widgetsHanyangLaunchPackage::on_pushButton_addTcpPreset_clicked()
{
    ROS_LOG_WARN("[%s] Add TCP Preset button clicked", __func__);
    
    // 레이블 색상 변경
    ui->label_addTcpPresetStatus->setStyleSheet("background-color: green; border-radius: 10px;");
    
#if DRFL_CONTROL
    _taskWindow->getTaskManager()->_qnode->drflSetRobotMode(0);

    auto createTCP = [&] (std::string tcp_name, std::vector<double> tcp_vec) {
        CsDouble tcp;
        vec2Arr(tcp_vec, tcp);
        bool success = _taskWindow->getTaskManager()->_qnode->drflCreateTcp(tcp_name, tcp);
        if(success) {
            ROS_LOG_WARN("[%s] Successfully Registered %s", __func__, tcp_name.c_str());
            _taskWindow->getTaskManager()->_qnode->list_tcp[tcp_name] = tcp;
        } else {
            ROS_LOG_WARN("[%s] TCP Registration Failed for %s!", __func__, tcp_name.c_str());
        }
        return success;
    };

    bool allSuccess = true;
    allSuccess &= createTCP("tcp00", {0, 0, 0, 0, 0, 0}); // Flange
    allSuccess &= createTCP("tcp01", {0, 40.0, 185.0, 0, 0, 90.0}); // legacy gripper
    allSuccess &= createTCP("tcp02", {0, 0, 0, 0, 0, 90.0}); //
    allSuccess &= createTCP("tcp03", {0, 45.0, 199.0, 0, 0, 90.0}); // new gripper
    allSuccess &= createTCP("tcp04", {0, 0.0, 265.0, 0, 0, 90.0}); // star gripper
    allSuccess &= createTCP("tcp05", {0.0, 0, 265.0, 0, 0, 90.0}); // star gripper (only rotation)
    allSuccess &= createTCP("tcp06", {0, 30.0, 203.5, 0, 0, 90.0}); // chemical coupler grasping tcp
    allSuccess &= createTCP("tcp07", {0.0, 0, 265.0, 0, 0, -90.0}); // chemical coupler grasping tcp
    allSuccess &= createTCP("tcp08", {0, 47.5, 265.0, 0, 0, -90.0}); // tcp for KeyRing (기존 보다 y + 66 , z + 25)

    _taskWindow->getTaskManager()->_qnode->drflSetRobotMode(1);
    
    if (!allSuccess) {
        ROS_LOG_ERROR("[%s] Some TCP presets failed to create", __func__);
        ui->label_addTcpPresetStatus->setStyleSheet("background-color: red; border-radius: 10px;");
    }
#endif
}

// --- PLC Monitoring 함수들 (주석 처리) ---
/*
void MainWindow_widgetsHanyangLaunchPackage::on_pushButton_MODBUS_MonitoringPLC_clicked()
{
    ROS_LOG_WARN("[%s] Monitoring PLC button clicked", __func__);
    const bool enable = ui->pushButton_MODBUS_MonitoringPLC->isChecked();
    _taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_ = enable;

    if (enable)
    {
        ui->textEdit_log->clear();  // Clear Log

        // PLC 시뮬레이션 모드라면 4100X 워드 초기화
        if (_taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_)
        {
            for (size_t i = 0; i < 16; ++i) // QNode의 plc_status_word_41000_와 41002_는 크기가 16으로 초기화됩니다.
            {
                _taskWindow->getTaskManager()->_qnode->plc_status_word_41000_[i] = false;
                _taskWindow->getTaskManager()->_qnode->plc_status_word_41002_[i] = false;
            }
            ROS_LOG_WARN("[%s] PLC simulation mode detected. Initialized PLC status words 41000 and 41002.", __func__);
        }
    }
}

void MainWindow_widgetsHanyangLaunchPackage::on_pushButton_MODBUS_MonitoringPLCReadAMR2RobotStatus_clicked()
{
    ROS_LOG_WARN("[%s] Monitoring PLC (Read AMR) button clicked", __func__);
    if (ui->pushButton_MODBUS_MonitoringPLCReadAMR2RobotStatus->isChecked())
    {
        _taskWindow->getTaskManager()->_qnode->is_monitoring_plc_read_amr_to_robot_status_ = true;
    }
    else
    {
        _taskWindow->getTaskManager()->_qnode->is_monitoring_plc_read_amr_to_robot_status_ = false;
    }
}
*/

void MainWindow_widgetsHanyangLaunchPackage::on_pushButton_auto_mode_clicked()
{
    ROS_LOG_WARN("[%s] Auto mode button clicked", __func__);
    
    // --- PLC 모니터링 기능 주석 처리 ---
    // Configuration 탭의 PLC 모니터링 버튼들을 자동으로 체크
    // ui->pushButton_MODBUS_MonitoringPLC->setChecked(true);
    // ui->pushButton_MODBUS_MonitoringPLCReadAMR2RobotStatus->setChecked(true);
    
    // PLC 모니터링 기능 활성화
    // _taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_ = true;
    // _taskWindow->getTaskManager()->_qnode->is_monitoring_plc_read_amr_to_robot_status_ = true;
    
    // 자동 모드 활성화
    _taskWindow->getTaskManager()->_qnode->is_task_mode_ = true;
    
    ROS_LOG_WARN("[%s] Auto mode activated (PLC monitoring disabled in launch package)", __func__);
}

void MainWindow_widgetsHanyangLaunchPackage::on_pushButton_manual_mode_clicked()
{
    ROS_LOG_WARN("[%s] Manual mode button clicked", __func__);
    
    // --- PLC 모니터링 기능 주석 처리 ---
    // Configuration 탭의 PLC 모니터링 버튼들을 자동으로 체크 해제
    // ui->pushButton_MODBUS_MonitoringPLC->setChecked(false);
    // ui->pushButton_MODBUS_MonitoringPLCReadAMR2RobotStatus->setChecked(false);
    
    // PLC 모니터링 기능 비활성화
    // _taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_ = false;
    // _taskWindow->getTaskManager()->_qnode->is_monitoring_plc_read_amr_to_robot_status_ = false;
    
    // 수동 모드 활성화
    _taskWindow->getTaskManager()->_qnode->is_task_mode_ = false;
    
    ROS_LOG_WARN("[%s] Manual mode activated (PLC monitoring disabled in launch package)", __func__);
}

void MainWindow_widgetsHanyangLaunchPackage::on_pushButton_robot_auto_setup_clicked()
{    
    // ROS_LOG_WARN("[%s] Robot Auto Setup button clicked manually", __func__);
    
    // // Configuration 탭의 버튼들 자동 활성화
    // ui->pushButton_doRobotConnect->setChecked(true);
    // ui->pushButton_doGripperConnect->setChecked(true);
    // ui->pushButton_do3DScannerNode->setChecked(true);
    // ui->pushButton_doMaskDetectionNode->setChecked(true);
    // ui->pushButton_doRvizViewNode->setChecked(false);
    // ui->pushButton_doScanNode->setChecked(true);
    // ui->pushButton_doMatchingNode->setChecked(true);
    // ui->pushButton_doKeyCode->setChecked(true);
    // ui->pushButton_doKeyCodeHolder->setChecked(true);
    
    // // UI 시작과 함께 프로그레스 다이얼로그 표시
    // QTimer::singleShot(500, this, [this]() {
    //     // 프로그레스 다이얼로그 생성
    //     setupProgressDialog = new QProgressDialog("Robot Auto Setup in progress...", "Cancel", 0, 100, this);
    //     setupProgressDialog->setWindowTitle("Robot Auto Setup");
    //     setupProgressDialog->setWindowModality(Qt::ApplicationModal);  // 모달 다이얼로그
    //     setupProgressDialog->setCancelButton(nullptr);  // 취소 버튼 제거
    //     setupProgressDialog->setAutoClose(false);
    //     setupProgressDialog->setAutoReset(false);
    //     setupProgressDialog->setValue(0);
        
    //     // 다이얼로그 크기 및 위치 설정
    //     setupProgressDialog->setMinimumSize(600, 200);  // 최소 크기 설정
    //     setupProgressDialog->resize(1200, 600);          // 기본 크기 설정
        
    //     // 스타일시트 설정 - 글자를 흰색으로
    //     setupProgressDialog->setStyleSheet(
    //         "QProgressDialog {"
    //         "    background-color: #2b2b2b;"
    //         "    color: white;"
    //         "    font-size: 16px;"
    //         "    font-weight: bold;"
    //         "}"
    //         "QProgressDialog QLabel {"
    //         "    color: white;"
    //         "    font-size: 18px;"
    //         "    font-weight: bold;"
    //         "}"
    //         "QProgressBar {"
    //         "    background-color: #404040;"
    //         "    border: 2px solid #555555;"
    //         "    border-radius: 5px;"
    //         "    text-align: center;"
    //         "    color: white;"
    //         "    font-weight: bold;"
    //         "}"
    //         "QProgressBar::chunk {"
    //         "    background-color: #4CAF50;"
    //         "    border-radius: 3px;"
    //         "}"
    //     );
        
    //     // 항상 맨 앞에 표시되도록 설정
    //     setupProgressDialog->setWindowFlags(Qt::Dialog | Qt::WindowStaysOnTopHint | Qt::WindowTitleHint);
    //     setupProgressDialog->setWindowModality(Qt::ApplicationModal);
        
    //     // 화면 중앙에 위치시키기
    //     QRect screenGeometry = QApplication::desktop()->screenGeometry();
    //     int x = (screenGeometry.width() - setupProgressDialog->width()) / 2;
    //     int y = (screenGeometry.height() - setupProgressDialog->height()) / 2;
    //     setupProgressDialog->move(x, y);
        
    //     setupProgressDialog->show();
    //     setupProgressDialog->raise();       // 맨 앞으로
    //     setupProgressDialog->activateWindow(); // 활성화
        
    //     ROS_LOG_WARN("[%s] Robot Auto Setup started", __func__);
    // });

    // // 각 버튼의 클릭 이벤트 트리거 (프로그레스와 함께)
    // QTimer::singleShot(600, this, [this]() {
    //     setupProgressDialog->setLabelText("Starting Robot Controller...");
    //     setupProgressDialog->setValue(10);
    //     setupProgressDialog->raise();       // 맨 앞으로
    //     setupProgressDialog->activateWindow(); // 활성화
    //     pushButtonRunRobotController();
    // });
    
    // QTimer::singleShot(700, this, [this]() {
    //     setupProgressDialog->setLabelText("Starting Gripper Node...");
    //     setupProgressDialog->setValue(20);
    //     setupProgressDialog->raise();       // 그리퍼 UI가 뜰 수 있으므로 맨 앞으로
    //     setupProgressDialog->activateWindow(); // 활성화
    //     pushButtonRunGripperNode();
    // });
    
    // QTimer::singleShot(800, this, [this]() {
    //     setupProgressDialog->setLabelText("Starting 3D Scanner Node...");
    //     setupProgressDialog->setValue(30);
    //     pushButtonRun3DScannerNode();
    // });
    
    // QTimer::singleShot(900, this, [this]() {
    //     setupProgressDialog->setLabelText("Starting Mask Detection Node...");
    //     setupProgressDialog->setValue(40);
    //     pushButtonRunMaskDetectionNode();
    // });
    
    // QTimer::singleShot(1000, this, [this]() {
    //     setupProgressDialog->setLabelText("Starting RViz View Node...");
    //     setupProgressDialog->setValue(50);
    //     setupProgressDialog->raise();       // RViz가 뜰 수 있으므로 맨 앞으로
    //     setupProgressDialog->activateWindow(); // 활성화
    //     pushButtonRunRvizViewNode();
    // });
    
    // QTimer::singleShot(1100, this, [this]() {
    //     setupProgressDialog->setLabelText("Starting Scan Node...");
    //     setupProgressDialog->setValue(60);
    //     pushButtonRunScanNode();
    // });
    
    // QTimer::singleShot(1200, this, [this]() {
    //     setupProgressDialog->setLabelText("Starting Matching Node...");
    //     setupProgressDialog->setValue(60);
    //     setupProgressDialog->raise();       // 맨 앞으로
    //     setupProgressDialog->activateWindow(); // 활성화
    //     pushButtonRunMatchingNode();
    //     pushButtonRunKeyCode();
    //     pushButtonRunKeyCodeHolder();
    // });
    
    // // 중간중간 다이얼로그가 맨 앞에 유지되도록 추가 보장
    // QTimer::singleShot(2000, this, [this]() {
    //     if (setupProgressDialog) {
    //         setupProgressDialog->raise();
    //         setupProgressDialog->activateWindow();
    //     }
    // });
    
    // QTimer::singleShot(4000, this, [this]() {
    //     if (setupProgressDialog) {
    //         setupProgressDialog->raise();
    //         setupProgressDialog->activateWindow();
    //     }
    // });
    
    // QTimer::singleShot(6000, this, [this]() {
    //     if (setupProgressDialog) {
    //         setupProgressDialog->raise();
    //         setupProgressDialog->activateWindow();
    //     }
    // });

    // // 순차적 버튼 클릭을 위한 타이머 설정 (프로그레스와 함께)
    // QTimer::singleShot(8000, this, [this]() {
    //     setupProgressDialog->setLabelText("Stopping Robot...");
    //     setupProgressDialog->setValue(70);
    //     ui->pushButton_stop->click();
    //     ROS_LOG_WARN("[%s] Stop button auto-clicked", __func__);
    // });

    // QTimer::singleShot(10000, this, [this]() {
    //     setupProgressDialog->setLabelText("Making Tool Weight...");
    //     setupProgressDialog->setValue(75);
    //     ui->pushButton_MakeToolWeight->click();
    //     ROS_LOG_WARN("[%s] Make Tool Weight button auto-clicked", __func__);
    // });

    // QTimer::singleShot(11000, this, [this]() {
    //     setupProgressDialog->setLabelText("Setting Tool Weight...");
    //     setupProgressDialog->setValue(80);
    //     ui->pushButton_SetToolWeight->click();
    //     ROS_LOG_WARN("[%s] Set Tool Weight button auto-clicked", __func__);
    // });

    // QTimer::singleShot(12000, this, [this]() {
    //     setupProgressDialog->setLabelText("Adding TCP Preset...");
    //     setupProgressDialog->setValue(90);
    //     ui->pushButtonAddTcpPreset->click();
    //     ROS_LOG_WARN("[%s] Add TCP Preset button auto-clicked", __func__);
    // });

    // QTimer::singleShot(14000, this, [this]() {
    //     setupProgressDialog->setLabelText("Setting Auto Mode...");
    //     setupProgressDialog->setValue(95);
    //     ui->pushButton_auto_mode->click();
    //     ROS_LOG_WARN("[%s] Auto mode button auto-clicked", __func__);

    // });

    // QTimer::singleShot(18000, this, [this]() {
    //     bringUIToFront();
    // });

    // QTimer::singleShot(19000, this, [this]() {
    //     setupProgressDialog->setLabelText("Setting Auto Mode...");
    //     setupProgressDialog->setValue(100);
    //     ui->pushButton_stop->click();

    //     // 완료 후 다이얼로그 닫기
    //     QTimer::singleShot(1000, this, [this]() {
    //         if (setupProgressDialog) {
    //             setupProgressDialog->close();
    //             delete setupProgressDialog;
    //             setupProgressDialog = nullptr;
    //         }
    //         ROS_LOG_WARN("[%s] Robot Auto Setup completed successfully", __func__);
    //     });
    // });
}

void MainWindow_widgetsHanyangLaunchPackage::on_pushButton_robot_auto_off_clicked()
{
    terminateQProcessAndRosNode();
    // delete ui;
    // rclcpp::shutdown();
}

void MainWindow_widgetsHanyangLaunchPackage::bringUIToFront()
{
    // 윈도우를 앞으로 가져오는 여러 방법을 시도
    this->raise();                    // 윈도우를 맨 앞으로
    this->activateWindow();           // 윈도우 활성화
    this->showNormal();              // 정상 상태로 표시
    this->setFocus();                // 포커스 설정
    
    // Linux에서 추가적인 방법 (X11 환경)
    #ifdef Q_OS_LINUX
    this->show();
    this->setWindowState(Qt::WindowActive);
    #endif
    
    ROS_LOG_INFO("[%s] UI brought to front", __func__);
}

// --- PLC 제어 함수들 (모두 주석 처리) ---
/*
void MainWindow_widgetsHanyangLaunchPackage::pushButtonSelectPlcWriteAddressClickedCallback() {

#if IS_PLC_LS_XG5000
    //// write
    if(ui->radioButton_select_plc_address_write_1->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x41000");
    } else if(ui->radioButton_select_plc_address_write_3->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x41002");
    } else if(ui->radioButton_select_plc_address_write_4->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x41003");
    } else if(ui->radioButton_select_plc_address_write_5->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x41004");
    }
#else
    //// write
    if(ui->radioButton_select_plc_address_write_1->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("1000");
    } else if(ui->radioButton_select_plc_address_write_3->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("1002");
    } else if(ui->radioButton_select_plc_address_write_4->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("1003");
    } else if(ui->radioButton_select_plc_address_write_5->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("1004");
    }
#endif
}

void MainWindow_widgetsHanyangLaunchPackage::pushButtonSelectPlcReadAddressClickedCallback() {

#if IS_PLC_LS_XG5000
    //// read
    if(ui->radioButton_select_plc_address_read_1->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x42000");
    } else if(ui->radioButton_select_plc_address_read_3->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x42002");
    }
#else
    //// read
    if(ui->radioButton_select_plc_address_read_1->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("2000");
    } else if(ui->radioButton_select_plc_address_read_3->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("2002");
    }
#endif
}

/// @brief PLC 관련 함수: Write
void MainWindow_widgetsHanyangLaunchPackage::pushButtonModbusWriteDataBoolType() {
    bool ok;
    // 16진수 "0x40005"도 변환 가능하도록 처리
    uint16_t word_address = static_cast<uint16_t>(ui->lineEdit_MODBUS_WORD_ADDRESS->text().toUInt(&ok, 0)); // 0을 전달하면 10진수/16진수 자동 인식
    if (!ok) {
        ROS_LOG_WARN("Error: Invalid MODBUS WORD ADDRESS! [%s]", __func__);
        return;
    }

    uint16_t bit_address = static_cast<uint16_t>(ui->lineEdit_MODBUS_BIT_ADDRESS->text().toUShort(&ok));
    if (!ok) {
        ROS_LOG_WARN("Error: Invalid MODBUS BIT ADDRESS! [%s]", __func__);
        return;
    }

    // qnode의 modbus 객체를 사용하여 직접 호출
    if (_taskWindow && _taskWindow->getTaskManager() && _taskWindow->getTaskManager()->_qnode && _taskWindow->getTaskManager()->_qnode->_modbus) {
        uint16_t current_value;
        int modbus_status;

        // 현재 레지스터 값 읽기
#if IS_PLC_LS_XG5000
        modbus_status = _taskWindow->getTaskManager()->_qnode->_modbus->modbus_read_input_registers(word_address, 1, &current_value);
#else
        modbus_status = _taskWindow->getTaskManager()->_qnode->_modbus->modbus_read_holding_registers(word_address, 1, &current_value);
#endif

        if (modbus_status != 0) {
            ROS_LOG_INFO("[%s] Failed to read PLC register, error code: %i", __func__, modbus_status);
            modbus_status_ = modbus_status;
            _taskWindow->getTaskManager()->_qnode->setMODBUSComm(true);
            return;
        }

        // 비트 위치 계산 (LSB 방식)
        uint16_t actual_bit_pos = bit_address;

        // 비트 변경: ON이면 1, OFF이면 0으로 설정
        bool status = ui->pushButton_MODBUS_WRITE_DATA_BOOL_TYPE->isChecked();
        if (status) {
            current_value |= (1 << actual_bit_pos);  // 해당 비트 ON
        } else {
            current_value &= ~(1 << actual_bit_pos); // 해당 비트 OFF
        }

        // 수정된 값 다시 쓰기
        modbus_status = _taskWindow->getTaskManager()->_qnode->_modbus->modbus_write_registers(word_address, 1, &current_value);

        if (modbus_status == 0) {
            ROS_LOG_INFO("[%s] PLC Task Status Updated: Bit %d -> %s", __func__, bit_address, status ? "ON" : "OFF");
            modbus_status_ = 0;
            qDebug() << "PLC Write 성공 - modbus_status_ = 0";
        } else {
            ROS_LOG_INFO("[%s] Failed to update PLC Task Status, error code: %i", __func__, modbus_status);
            modbus_status_ = modbus_status;
            qDebug() << "PLC Write 실패 - modbus_status_ =" << modbus_status;
            _taskWindow->getTaskManager()->_qnode->setMODBUSComm(true);
        }
    } else {
        ROS_LOG_WARN("[%s] Modbus connection not available", __func__);
        modbus_status_ = -1;
        qDebug() << "PLC Write 연결 없음 - modbus_status_ = -1";
    }
}

/// @brief PLC 관련 함수 : Read
void MainWindow_widgetsHanyangLaunchPackage::pushButtonModbusReadDataBoolType() {
    bool ok;
    uint16_t word_address = static_cast<uint16_t>(ui->lineEdit_MODBUS_WORD_ADDRESS->text().toUInt(&ok, 0)); // 0을 전달하면 10진수/16진수 자동 인식
    if (!ok) {
        ROS_LOG_WARN("Error: Invalid MODBUS WORD ADDRESS! [%s]", __func__);
        return;
    }

    uint16_t bit_address = static_cast<uint16_t>(ui->lineEdit_MODBUS_BIT_ADDRESS->text().toUShort(&ok));
    if (!ok) {
        ROS_LOG_WARN("Error: Invalid MODBUS BIT ADDRESS! [%s]", __func__);
        return;
    }

    // qnode의 modbus 객체를 사용하여 직접 호출
    if (_taskWindow && _taskWindow->getTaskManager() && _taskWindow->getTaskManager()->_qnode && _taskWindow->getTaskManager()->_qnode->_modbus) {
        uint16_t current_value;
        int modbus_status;

        // 현재 레지스터 값 읽기
        modbus_status = _taskWindow->getTaskManager()->_qnode->_modbus->modbus_read_input_registers(word_address, 1, &current_value);

        if (modbus_status != 0) {
            ROS_LOG_INFO("[%s] Failed to read PLC register, error code: %i", __func__, modbus_status);
            modbus_status_ = modbus_status;
            qDebug() << "PLC Read 실패 - modbus_status_ =" << modbus_status;
            _taskWindow->getTaskManager()->_qnode->setMODBUSComm(true);
            return;
        }

        // 특정 비트 추출
        bool bit_status = (current_value >> bit_address) & 0x01;
        
        ROS_LOG_INFO("[%s] Register: 0x%X, Bit: %u, Value: %u", __func__, word_address, bit_address, bit_status);
        
        if (bit_status) {
            ROS_LOG_WARN("[%s] MODBUS Read - Address: 0x%X, Bit: %u → Flag: TRUE", __func__, word_address, bit_address);
        } else {
            ROS_LOG_WARN("[%s] MODBUS Read - Address: 0x%X, Bit: %u → Flag: FALSE", __func__, word_address, bit_address);
        }
        
        modbus_status_ = 0;
        qDebug() << "PLC Read 성공 - modbus_status_ = 0";
    } else {
        ROS_LOG_WARN("[%s] MODBUS Read failed - Address: 0x%X, Bit: %u", __func__, word_address, bit_address);
        modbus_status_ = -1;
        qDebug() << "PLC Read 연결 없음 - modbus_status_ = -1";
    }
}
*/

/// @brief label_image_viewer 이미지 업데이트 함수
void MainWindow_widgetsHanyangLaunchPackage::UpdateImage(const QImage &image, const QString &current_pass) {
    if (!image.isNull()) {
        QPixmap scaledPixmap = QPixmap::fromImage(image).scaled(
            ui->label_image_viewer->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation
        );
        ui->label_image_viewer->setPixmap(scaledPixmap);
        ui->label_image_viewer->setText("");
        ui->label_image_viewer->setStyleSheet("QLabel { background-color: white; border: 2px solid #e2e8f0; border-radius: 8px; }");
    } else {
        ui->label_image_viewer->setPixmap(QPixmap());
        ui->label_image_viewer->setText("No image");
        ui->label_image_viewer->setAlignment(Qt::AlignCenter);
        ui->label_image_viewer->setStyleSheet("QLabel { color: #666; font-size: 16px; background-color: #f8f9fa; border: 2px dashed #dee2e6; border-radius: 8px; }");
    }
}