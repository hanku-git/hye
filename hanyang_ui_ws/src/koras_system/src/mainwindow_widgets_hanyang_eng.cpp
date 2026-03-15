#include "mainwindow_widgets_hanyang_eng.hpp"
#include "ui_mainwindow_widgets_hanyang_eng.h"
#include <QKeyEvent>
#include <QApplication>
#include <typeinfo>
MainWindow_widgetsHanyangEng::MainWindow_widgetsHanyangEng(MainWindow_node * taskWindow, QWidget* parent)
    : _taskWindow(taskWindow)
    , QMainWindow(parent)
    , ui(new Ui::MainWindow_widgetsHanyangEng)
    , isRecording(false)
{
    ui->setupUi(this);
    ///////////////////////////////////////////
    setupPlanner(); // TaskPlanner(HanyangEng) 초기화
    ///////////////////////////////////////////

    //set log font

    ui->textEdit_log->setReadOnly(true);

    QFont logFont;
    logFont.setPointSize(18);
    ui->textEdit_log->setFont(logFont);

    ui->textEdit_log->setMaximumHeight(500);


    ROS_LOG_WARN("THIS 2!");

    ////////////////////////////////////////////////////////////
    ////////////////////////////// Add connect functions to ui components


    connect(ui->pushButton_enable_disable, &QPushButton::clicked, this, [this] {
        _taskWindow->getTaskManager()->_qnode->setEnable(!_taskWindow->getTaskManager()->_qnode->params_.status.is_enable);
    });

    connect(ui->pushButton_KorasGripperInitialize, &QPushButton::clicked, this, [this] {
        uint16_t position = 0;
        _taskWindow->getTaskManager()->_qnode->setGrpInitialize();
// #if NEW_VER_KORAS_GRIPPER_PACKAGE
//         _taskWindow->getTaskManager()->_qnode->setGrpInitialize();
// #else
//         _taskWindow->getTaskManager()->_qnode->setGrp((uint16_t)KR_GRP::INIT, position, _taskWindow->getTaskManager()->_qnode->grp_driver_address_);
// #end
    });

    connect(ui->pushButton_KorasGripperOpen, &QPushButton::clicked, this, [this] {
        uint16_t position = 0;
        _taskWindow->getTaskManager()->_qnode->setGrpOpen();
// #if NEW_VER_KORAS_GRIPPER_PACKAGE
//         _taskWindow->getTaskManager()->_qnode->setGrpOpen();
// #else
//         _taskWindow->getTaskManager()->_qnode->setGrp((uint16_t)KR_GRP::OPEN, position, _taskWindow->getTaskManager()->_qnode->grp_driver_address_);
// #end
    });

    connect(ui->pushButton_KorasGripperClose, &QPushButton::clicked, this, [this] {
        uint16_t position = 10000;
        _taskWindow->getTaskManager()->_qnode->setGrpClose();
// #if NEW_VER_KORAS_GRIPPER_PACKAGE
//         _taskWindow->getTaskManager()->_qnode->setGrpClose();
// #else
//         _taskWindow->getTaskManager()->_qnode->setGrp((uint16_t)KR_GRP::CLOSE, position, _taskWindow->getTaskManager()->_qnode->grp_driver_address_);
// #end
    });

    connect(ui->pushButton_KorasGripperPosCtrl, &QPushButton::clicked, this, [this] {
        uint16_t position = ui->task_spinBox_gripperPosValue->value(); // (scale: 0~10000)
        _taskWindow->getTaskManager()->_qnode->setGrp((uint16_t)KR_GRP::POS_CTRL, position, _taskWindow->getTaskManager()->_qnode->grp_driver_address_);
    });

    connect(ui->pushButton_openDeveloperWindow_1, &QPushButton::clicked, this, [this] {
        // if(_taskWindow->getTaskManager()->_qnode->do_open_developer_window_) {
            emit openDeveloperWindow();
        // }
    });
    connect(ui->pushButton_openDeveloperWindow_5, &QPushButton::clicked, this, [this] {
        emit openIRLDeveloperWindow();
    });

    connect(ui->pushButton_startTask, &QPushButton::clicked, this, [this] {
        ui->pushButton_startTask->setEnabled(false);
        ROS_LOG_WARN("***** DO TASK! *****");
        //// Step 1) Generate Target Tasks
        hanyangEngPlanner_->generateHanyangEngTask();
        //// Step 2) Do Target Tasks
        bool ok = doTargetTask(hanyangEngPlanner_->doosan_module_task_);
        if (ok) {
            ROS_LOG_WARN("DO TARGET TASK!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
        ui->pushButton_startTask->setEnabled(true);
    });

    qRegisterMetaType<LogInfo>("LogInfo");
    connect(_taskWindow->getTaskManager()->_qnode, &QNode::sendLogMessage, this, [this](LogInfo logInfo) {
        if (ui->textEdit_log->textCursor().position() > 500) {
            // ui->textEdit_log->textCursor().deletePreviousChar();
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


    // QObject::connect(bin_matching_dialog, &BinMatchingDialog::setCustomTrasnformationPose , this, &MainWindow_widgetsHanyangEng::setCustomTrasnformationPose);

    QObject::connect(_taskWindow->getTaskManager()->_qnode, &QNode::sendLogMessageStr , this, &MainWindow_widgetsHanyangEng::sendLogMessageStr);

    QObject::connect(_taskWindow->getTaskManager()->_qnode, &QNode::taskStartedSignal, this, [this]() {
        // total = 0;
        // auto task_list = _taskWindow->getTaskManager()->_qnode->getTaskInfo().taskInfo;
        // tagCounts.resize(task_list.size(), 0);

        // for (size_t i = 0; i < task_list.size(); i++)
        // {
        //     tagCounts[i] = task_list[i].size();
        //     total += task_list[i].size();
        //     // std::cout << "tagCounts:: " << tagCounts[i] << " \n";
        // }
    });


    QObject::connect(_taskWindow->getTaskManager()->_qnode, &QNode::pauseLogger, this, [this]() {
        //POP UP MESSAGE
        auto nodeId_task = _taskWindow->getTaskManager()->task_nodes[_taskWindow->getTaskManager()->task_count];
        if (_taskWindow->getTaskManager()->_graphmodel.nodeData(nodeId_task, NodeRole::Type) == "POPUP MESSAGE") {

            QMessageBox *msg = new QMessageBox();
            QMessageBox::StandardButton reply;
            msg->setDefaultButton(reply);
            reply = QMessageBox::critical(this, "[Teaching Process] Task pause", "Go to next task",
                                            QMessageBox::Yes|QMessageBox::No);
            if (reply == QMessageBox::Yes) {
                _taskWindow->getTaskManager()->_qnode->resumeTask();
            } else {
            }
        }
    });


    connect(_taskWindow->getTaskManager()->_qnode, &QNode::taskEndLogger, this, [this]() {

        // for (size_t i = 0; i < progress_widget->buttons.size(); i++)
        // {
        //     progress_widget->setButtonStopEffect(i);
        // }
        // QTextCursor cursor = ui->textEdit_log->textCursor();
        // cursor.movePosition(QTextCursor::End);
        // ui->textEdit_log->setTextCursor(cursor);
        // QFont font = ui->textEdit_log->font();
        // // QString endHtml = "<font color=\"Blue\">Task" + QString::number(monitor_task_index + 1) + " has been completed</font><br>";
        // QString endHtml = "<font color=\"Blue\">Task complete!</font><br>";

        // ui->textEdit_log->insertHtml(endHtml);
        // cursor.movePosition(QTextCursor::End);
        // ui->textEdit_log->setTextCursor(cursor);
    });

    ROS_LOG_WARN("[%s] ...", __func__);


    //// Bin-picking target object select
    bin_picking_target_object_list_ << ui->radioButton_graspingUI_object_11
                                    << ui->radioButton_graspingUI_object_12
                                    << ui->radioButton_graspingUI_object_13
                                    << ui->radioButton_graspingUI_object_14
                                    << ui->radioButton_graspingUI_object_15
                                    << ui->radioButton_graspingUI_object_16
                                    << ui->radioButton_graspingUI_object_17
                                    << ui->radioButton_graspingUI_object_18
                                    << ui->radioButton_graspingUI_object_19
                                    << ui->radioButton_graspingUI_object_20
                                    << ui->radioButton_graspingUI_object_21
                                    << ui->radioButton_graspingUI_object_22
                                    << ui->radioButton_graspingUI_object_23
                                    << ui->radioButton_graspingUI_object_24
                                    << ui->radioButton_graspingUI_object_25
                                    << ui->radioButton_graspingUI_object_26
                                    << ui->radioButton_graspingUI_object_27
                                    << ui->radioButton_graspingUI_object_28
                                    << ui->radioButton_graspingUI_object_29
                                    << ui->radioButton_graspingUI_object_30
                                    << ui->radioButton_graspingUI_object_31;

    //// bin picking target object list
    for(int i=0; i<bin_picking_target_object_list_.size(); i++) {
        QObject::connect(bin_picking_target_object_list_[i]     , SIGNAL(clicked()), this, SLOT(pushButtonSelectTargetObjectClickedCallback()));
    }

    ////

    //// Button
    connect(ui->pushButton_exit_program, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::testBtnCallback);
    connect(ui->pushButton_stop_all, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonSTOPAllCallback);

    connect(ui->pushButton_exit_program_2, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::testBtnCallback);
    connect(ui->pushButton_stop_all_2, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonSTOPAllCallback);
        // QNode emits requestStopAll when internal safety wants STOP button behavior
    connect(_taskWindow->getTaskManager()->_qnode, &QNode::requestStopAll, this, &MainWindow_widgetsHanyangEng::pushButtonSTOPAllCallback);

    //// PLC Task with ServoStar
    connect(ui->pushButton_doTask_plc_comm_amrReadyPose, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDoTasksAMRReadyPoseCallback);
    connect(ui->pushButton_doTask_drum_homeposeLeft, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::leftPushButtonDoTasksDrumHomeposeCallback);
    connect(ui->pushButton_doTask_drum_homeposeRight, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonDoTasksDrumHomeposeCallback);

    connect(ui->pushButton_doTask_plc_comm_capDetachLeft, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDoTasksCapDetachLeftCallback);
    connect(ui->pushButton_doTask_plc_comm_capDetachRight, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDoTasksCapDetachRightCallback);
    connect(ui->pushButton_doTask_plc_comm_capScrewingLeft, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDoTasksCapScrewingLeftCallback);
    connect(ui->pushButton_doTask_plc_comm_capScrewingRight, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDoTasksCapScrewingRightCallback);
    connect(ui->pushButton_doTask_plc_comm_couplerDetachLeft, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDoTasksCouplerDetachLeftCallback);
    connect(ui->pushButton_doTask_plc_comm_couplerDetachRight, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDoTasksCouplerDetachRightCallback);
    connect(ui->pushButton_doTask_plc_comm_couplerScrewingLeft, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDoTasksCouplerScrewingLeftCallback);
    connect(ui->pushButton_doTask_plc_comm_couplerScrewingRight, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDoTasksCouplerScrewingRightCallback);

    connect(ui->pushButton_doTask_plc_comm_rotationAngleDetection, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDoTasksRotationAngleDetectionCallback);
    connect(ui->pushButton_doTask_plc_comm_barcodeDetection, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDoTasksBarcodeDetectionCallback);




    connect(ui->pushButton_doTask_plc_comm_capDetachAndCouplerScrewingLeft, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDoTasksCapDetachAndCouplerScrewingLeftCallback);
    connect(ui->pushButton_doTask_plc_comm_capDetachAndCouplerScrewingRight, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDoTasksCapDetachAndCouplerScrewingRightCallback);
    connect(ui->pushButton_doTask_plc_comm_couplerDetachAndCapScrewingLeft, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDoTasksCouplerDetachAndCapScrewingLeftCallback);
    connect(ui->pushButton_doTask_plc_comm_couplerDetachAndCapScrewingRight, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDoTasksCouplerDetachAndCapScrewingRightCallback);



    
        
    



    // Total task
    connect(ui->rightPushButton_doTask_drum_total_task, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonDoDrumTotalTasksCallback);
    connect(ui->leftPushButton_doTask_drum_total_task, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::leftPushButtonDoDrumTotalTasksCallback);

    // Sub task
    connect(ui->rightPushButton_doTask_drum_sub_task_1, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask1Callback);
    connect(ui->rightPushButton_doTask_drum_sub_task_2, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask2Callback);
    connect(ui->rightPushButton_doTask_drum_sub_task_3, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask3Callback);
    connect(ui->rightPushButton_doTask_drum_sub_task_4, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask4Callback);
    connect(ui->rightPushButton_doTask_drum_sub_task_5, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask5Callback);
    connect(ui->rightPushButton_doTask_drum_sub_task_6, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask6Callback);
    connect(ui->rightPushButton_doTask_drum_sub_task_7, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask7Callback);
    connect(ui->rightPushButton_doTask_drum_sub_task_8, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask7Callback);
    connect(ui->rightPushButton_doTask_drum_sub_task_9, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask9Callback);
    connect(ui->rightPushButton_doTask_drum_sub_task_10, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask10Callback);
    connect(ui->rightPushButton_doTask_drum_sub_task_11, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask11Callback);

    /// 로봇 기능 테스트 위젯
    connect(ui->rightPushButton_auto_mode, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonRobotAutoMode);
    connect(ui->rightPushButton_manual_mode, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonRobotManualMode);
    connect(ui->rightPushButton_keycode_test, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonKeyCodeTest);
    connect(ui->rightPushButton_keycode_unscrewing, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonKeyCodeUnscrewing);

    /// 250916 한양이엔지 호스 위치 확인 테스트
    /// (1) holder with lid cap 파지 자세 테스트
    /// (2) holder with key code 파지 자세 테스트
    /// (3) holder with coupler 파지 자세 테스트
    /// (4) coupler unscrewing 파지 자세 테스트
    connect(ui->rightPushButton_hanyang_hose_test1, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonHanyangHoseTest1);
    connect(ui->rightPushButton_hanyang_hose_test2, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonHanyangHoseTest2);
    connect(ui->rightPushButton_hanyang_hose_test3, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonHanyangHoseTest3);
    connect(ui->rightPushButton_hanyang_hose_test4, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonHanyangHoseTest4);

    /// 250918 한양 이엔지 전체 동작 테스트
    /// (1) 드럼에서 lid cap 해제 테스트
    /// (2) 리드캡 거치대에 거치대 거치 테스트
    /// (3) 커플러 거치대에서 커플러 파지 테스트
    /// (4) 드럼에 커플러 체결 테스트
    /// (5) 드럼에서 커플러 해제 테스트
    /// (6) 커플러 거치대에 커플러 거치 테스트
    /// (7) 리드캡 거치대에서 리드캡 파지 테스트
    /// (8) 드럼에 리드캡 체결 테스트
    connect(ui->rightPushButton_hanyang_0918_test1, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonHanyang0918Test1);
    connect(ui->rightPushButton_hanyang_0918_test2, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonHanyang0918Test2);
    connect(ui->rightPushButton_hanyang_0918_test3, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonHanyang0918Test3);
    connect(ui->rightPushButton_hanyang_0918_test4, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonHanyang0918Test4);
    connect(ui->rightPushButton_hanyang_0918_test5, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonHanyang0918Test5);
    connect(ui->rightPushButton_hanyang_0918_test6, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonHanyang0918Test6);
    connect(ui->rightPushButton_hanyang_0918_test7, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonHanyang0918Test7);
    connect(ui->rightPushButton_hanyang_0918_test8, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::rightPushButtonHanyang0918Test8);



    connect(ui->leftPushButton_doTask_drum_sub_task_1, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::leftPushButtonDrumSubTask1Callback);
    connect(ui->leftPushButton_doTask_drum_sub_task_2, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::leftPushButtonDrumSubTask2Callback);
    connect(ui->leftPushButton_doTask_drum_sub_task_3, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::leftPushButtonDrumSubTask3Callback);
    connect(ui->leftPushButton_doTask_drum_sub_task_4, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::leftPushButtonDrumSubTask4Callback);
    connect(ui->leftPushButton_doTask_drum_sub_task_5, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::leftPushButtonDrumSubTask5Callback);
    connect(ui->leftPushButton_doTask_drum_sub_task_6, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::leftPushButtonDrumSubTask6Callback);
    connect(ui->leftPushButton_doTask_drum_sub_task_7, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::leftPushButtonDrumSubTask7Callback);


    // Scan Task
    connect(ui->pushButton_doTask_drum_scan_task_1, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDrumScanTask1Callback);
    connect(ui->pushButton_doTask_drum_scan_task_2, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDrumScanTask2Callback);
    connect(ui->pushButton_doTask_drum_scan_task_3, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDrumScanTask3Callback);
    connect(ui->pushButton_doTask_drum_scan_task_4, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDrumScanTask4Callback);
    connect(ui->pushButton_doTask_drum_scan_task_5, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDrumScanTask5Callback);
    connect(ui->pushButton_doTask_drum_scan_task_6, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDrumScanTask6Callback);
    connect(ui->pushButton_doTask_drum_scan_task_7, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDrumScanTask7Callback);
    connect(ui->pushButton_doTask_drum_scan_task_8, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDrumScanTask8Callback);
    connect(ui->pushButton_doTask_drum_scan_task_9, &QPushButton::clicked, this, &MainWindow_widgetsHanyangEng::pushButtonDrumScanTask9Callback);



    // RIGHT side
    QObject::connect(ui->pushButton_setUIJSPosition_11     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition11ClickedCallback()));
    QObject::connect(ui->pushButton_setUIJSPosition_12     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition12ClickedCallback()));
    QObject::connect(ui->pushButton_setUIJSPosition_13     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition13ClickedCallback()));
    QObject::connect(ui->pushButton_setUIJSPosition_14     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition14ClickedCallback()));
    QObject::connect(ui->pushButton_setUIJSPosition_15     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition15ClickedCallback()));
    QObject::connect(ui->pushButton_setUIJSPosition_16     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition16ClickedCallback()));
    QObject::connect(ui->pushButton_setUIJSPosition_17     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition17ClickedCallback()));

    // LEFT side
    QObject::connect(ui->pushButton_setUIJSPosition_18     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition18ClickedCallback()));
    QObject::connect(ui->pushButton_setUIJSPosition_19     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition19ClickedCallback()));
    QObject::connect(ui->pushButton_setUIJSPosition_20     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition20ClickedCallback()));
    QObject::connect(ui->pushButton_setUIJSPosition_21     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition21ClickedCallback()));
    QObject::connect(ui->pushButton_setUIJSPosition_22     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition22ClickedCallback()));

    QObject::connect(ui->pushButton_setUIJSPosition_25     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition19ClickedCallback()));
    QObject::connect(ui->pushButton_setUIJSPosition_26     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition11ClickedCallback()));


    QObject::connect(ui->pushButton_Compliance        , SIGNAL(clicked()), this, SLOT(pushButtonComplianceCallback()));
    QObject::connect(ui->pushButton_ComplianceOFF        , SIGNAL(clicked()), this, SLOT(pushButtonComplianceOffCallback()));

    QObject::connect(ui->pushButton_RobotOFF        , SIGNAL(clicked()), this, SLOT(pushButtonRobotOFFCallback()));
    QObject::connect(ui->pushButton_RobotON        , SIGNAL(clicked()), this, SLOT(pushButtonRobotONCallback()));

    QObject::connect(ui->pushButton_RobotOFF_2        , SIGNAL(clicked()), this, SLOT(pushButtonRobotOFFCallback()));
    QObject::connect(ui->pushButton_RobotON_2        , SIGNAL(clicked()), this, SLOT(pushButtonRobotONCallback()));

   

    QObject::connect(ui->pushButton_SetToolWeight        , SIGNAL(clicked()), this, SLOT(pushButtonSetToolWeight()));
    QObject::connect(ui->pushButton_SetToolWeight_2        , SIGNAL(clicked()), this, SLOT(pushButtonSetToolWeight()));
    QObject::connect(ui->pushButton_MakeToolWeight        , SIGNAL(clicked()), this, SLOT(pushButtonMakeToolWeight()));
    QObject::connect(ui->pushButton_MakeToolWeight_2       , SIGNAL(clicked()), this, SLOT(pushButtonMakeToolWeight()));
    QObject::connect(ui->pushButtonAddTcpPreset        , SIGNAL(clicked()), this, SLOT(pushButtonAddTcpPresetClickedCallback()));
    QObject::connect(ui->pushButtonAddTcpPreset_2        , SIGNAL(clicked()), this, SLOT(pushButtonAddTcpPresetClickedCallback()));
    QObject::connect(ui->pushButtonGetTorque        , SIGNAL(clicked()), this, SLOT(pushButtonGetTorqueClickedCallback()));


    QObject::connect(ui->pushButton_task_pause         , SIGNAL(clicked()), this, SLOT(pauseTaskBtnCallback()));
    QObject::connect(ui->pushButton_task_resume         , SIGNAL(clicked()), this, SLOT(resumeTaskBtnCallback()));

    QObject::connect(ui->pushButton_task_pause_2         , SIGNAL(clicked()), this, SLOT(pauseTaskBtnCallback()));
    QObject::connect(ui->pushButton_task_resume_2         , SIGNAL(clicked()), this, SLOT(resumeTaskBtnCallback()));

    QObject::connect(ui->pushButton_KorasGripperChemicalGripperInitialize     , SIGNAL(clicked()), this, SLOT(pushButtonDoKorasChemicalGripperInitializeCallback()));

    QObject::connect(ui->pushButton_KorasGripperChemicalGripperOpen     , SIGNAL(clicked()), this, SLOT(pushButtonDoKorasChemicalGripperOpenCallback()));
    QObject::connect(ui->pushButton_KorasGripperChemicalGripperClose     , SIGNAL(clicked()), this, SLOT(pushButtonDoKorasChemicalGripperCloseCallback()));


    QObject::connect(ui->pushButton_KorasGripperChemicalGripperGraspingPose     , SIGNAL(clicked()), this, SLOT(pushButtonDoKorasChemicalGripperGraspingPoseCallback()));
    QObject::connect(ui->pushButton_KorasGripperChemicalGripperScrewingPose     , SIGNAL(clicked()), this, SLOT(pushButtonDoKorasChemicalGripperScrewingPoseCallback()));
    QObject::connect(ui->pushButton_KorasGripperChemicalGripperUnscrewingPose     , SIGNAL(clicked()), this, SLOT(pushButtonDoKorasChemicalGripperUnscrewingPoseCallback()));
    QObject::connect(ui->pushButton_KorasGripperChemicalGripperExitPose     , SIGNAL(clicked()), this, SLOT(pushButtonDoKorasChemicalGripperExitPoseCallback()));
    QObject::connect(ui->pushButton_KorasGripperChemicalGripperPlusMotorCtrl     , SIGNAL(clicked()), this, SLOT(pushButtonDoKorasChemicalGripperPlusMotorCtrlCallback()));
    QObject::connect(ui->pushButton_KorasGripperChemicalGripperMinusMotorCtrl     , SIGNAL(clicked()), this, SLOT(pushButtonDoKorasChemicalGripperMinusMotorCtrlCallback()));


    QObject::connect(ui->pushButton_doScan_ZIVID     , SIGNAL(clicked()), this, SLOT(pushButtonDoScanningZIVIDClickedCallback()));
    QObject::connect(ui->pushButton_doTemplateMatchingBinPicking     , SIGNAL(clicked()), this, SLOT(pushButtonDoTemplateMatchingBinPickingClickedCallback()));


    connect(_taskWindow->getTaskManager()->_qnode, &QNode::UpdateImage, this, &MainWindow_widgetsHanyangEng::UpdateImage);


    //// PLC

    QObject::connect(ui->radioButton_PLCSimulationFlag     , SIGNAL(clicked()), this, SLOT(pushButtonSetPLCSimulationFlagClickedCallback()));


    QObject::connect(ui->radioButton_select_plc_address_write_1     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcWriteAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_write_2     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcWriteAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_write_3     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcWriteAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_write_4     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcWriteAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_write_5     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcWriteAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_write_6     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcWriteAddressClickedCallback()));

    QObject::connect(ui->radioButton_select_plc_address_read_1     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcReadAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_read_2     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcReadAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_read_3     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcReadAddressClickedCallback()));

    QObject::connect(ui->pushButton_MODBUS_WRITE_DATA_BOOL_TYPE, SIGNAL(clicked()), this, SLOT(pushButtonModbusWriteDataBoolType()));
    QObject::connect(ui->pushButton_MODBUS_WRITE_DATA_ROTATION_ANGLE, SIGNAL(clicked()), this, SLOT(pushButtonModbusWriteDataInt16Type()));
    QObject::connect(ui->pushButton_MODBUS_WRITE_DATA_ASCII_BARCODE, SIGNAL(clicked()), this, SLOT(pushButtonModbusWriteDataASCIIType()));

    QObject::connect(ui->pushButton_MODBUS_READ_DATA_BOOL_TYPE, SIGNAL(clicked()), this, SLOT(pushButtonModbusReadDataBoolType()));
    QObject::connect(ui->pushButton_MODBUS_MonitoringPLC, SIGNAL(clicked()), this, SLOT(pushButtonModbusMonitoringPLC()));
    QObject::connect(ui->pushButton_MODBUS_MonitoringPLCReadAMR2RobotStatus, SIGNAL(clicked()), this, SLOT(pushButtonModbusMonitoringPLCReadAMR2RobotStatus()));

    QObject::connect(ui->pushButton_MODBUS_MonitoringPLC_2, SIGNAL(clicked()), this, SLOT(pushButtonModbusMonitoringPLC()));
    QObject::connect(ui->pushButton_MODBUS_MonitoringPLCReadAMR2RobotStatus_2, SIGNAL(clicked()), this, SLOT(pushButtonModbusMonitoringPLCReadAMR2RobotStatus()));


    QObject::connect(ui->pushButton_MODBUS_initializePLCWrite41000Signals, SIGNAL(clicked()), this, SLOT(pushButtonModbusInitializePLCWrite41000Signals()));
    QObject::connect(ui->pushButton_MODBUS_initializePLCWrite41002Signals, SIGNAL(clicked()), this, SLOT(pushButtonModbusInitializePLCWrite41002Signals()));
    QObject::connect(ui->pushButton_MODBUS_initializePLCWrite41003Signals, SIGNAL(clicked()), this, SLOT(pushButtonModbusInitializePLCWrite41003Signals()));
    QObject::connect(ui->pushButton_MODBUS_sendPLCWrite41000Bit14Signals, SIGNAL(clicked()), this, SLOT(pushButtonModbusSendPLCWrite41000Bit14Signals()));


    QObject::connect(ui->pushButton_MODBUS_TaskPLCFlagOnOff, SIGNAL(clicked()), this, SLOT(pushButtonModbusTaskPLCFlagOnOff()));


    plc_hanyang_task_flag_check_box_41000_
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

    plc_hanyang_task_flag_check_box_41002_
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

    plc_hanyang_task_flag_check_box_41003_
    << ui->checkBox_41003_1
    << ui->checkBox_41003_2
    << ui->checkBox_41003_3
    << ui->checkBox_41003_4
    << ui->checkBox_41003_5
    << ui->checkBox_41003_6
    << ui->checkBox_41003_7
    << ui->checkBox_41003_8
    << ui->checkBox_41003_9
    << ui->checkBox_41003_10
    << ui->checkBox_41003_11
    << ui->checkBox_41003_12
    << ui->checkBox_41003_13
    << ui->checkBox_41003_14
    << ui->checkBox_41003_15
    << ui->checkBox_41003_16;

    plc_hanyang_task_flag_check_box_42000_
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

    plc_hanyang_task_flag_check_box_42002_
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

    for(int i = 0; i < plc_hanyang_task_flag_check_box_41000_.size(); ++i) {
        QObject::connect(plc_hanyang_task_flag_check_box_41000_[i]     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcSimulationTaskFlagClickedCallback()));
        QObject::connect(plc_hanyang_task_flag_check_box_41002_[i]     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcSimulationTaskFlagClickedCallback()));
        QObject::connect(plc_hanyang_task_flag_check_box_41003_[i]     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcSimulationTaskFlagClickedCallback()));
        QObject::connect(plc_hanyang_task_flag_check_box_42000_[i]     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcSimulationTaskFlagClickedCallback()));
        QObject::connect(plc_hanyang_task_flag_check_box_42002_[i]     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcSimulationTaskFlagClickedCallback()));
    }

    if(1) {
        //// TODO: 250515, Matching Node Start Button 클릭 시에 bin_picking_node관련된 부분만 Qnode 함수로 변경하기
        ROS_LOG_WARN("[initializeDialog] Start!");
        initializeDialog();
        ROS_LOG_WARN("[initializeDialog] Completed!");
    }

    if(0) {
        ROS_LOG_WARN("[Doosan Robot Initial Setting] Start!");
        pushButtonMakeToolWeight(); // 1) Tool Make
        pushButtonSetToolWeight(); // 2) Tool Set
        pushButtonAddTcpPresetClickedCallback(); // 3) TCPMake
        _taskWindow->getTaskManager()->_qnode->drflSetTcp("tcp00"); // 4) Set TCP

        ROS_LOG_WARN("[Doosan Robot Initial Setting] Completed!");
    }

    //// Qt Main timer
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow_widgetsHanyangEng::timerCallback);
    timer_->start(1000);

    //// Qt sub timer
    status_update_timer_ = new QTimer(this);
    connect(status_update_timer_, &QTimer::timeout, this, &MainWindow_widgetsHanyangEng::statusTimerCallback);
    // status_update_timer_->start(200);
    status_update_timer_->start(1000);

}



MainWindow_widgetsHanyangEng::~MainWindow_widgetsHanyangEng()
{
    delete ui;
    rclcpp::shutdown();
}

void MainWindow_widgetsHanyangEng::closeEvent(QCloseEvent *bar)
{
    bar->ignore();
    return;
}

double computeVectorNormTmp(const std::vector<double>& v1, const std::vector<double>& v2) {
    if (v1.size() != v2.size()) {
        throw std::invalid_argument("두 벡터의 크기가 동일해야 합니다.");
    }

    double sum = 0.0;
    for (size_t i = 0; i < v1.size(); ++i) {
        double diff = v1[i] - v2[i];
        sum += diff * diff;
    }

    return std::sqrt(sum);
}

void MainWindow_widgetsHanyangEng::initializeDialog() {

    ////////////////////////////////////////////////////////
    //// Target object 데모 순서 정의
    _taskWindow->getTaskManager()->_qnode->cnt_selected_object_ = 0; // initialize

    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.clear();
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_CHEMICAL_COUPLER_HOLDER);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_HOLE_SURFACE);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_COUPLER_UNSCREWING);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_LID_CAP_UNSCREWING);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_LID_CAP_HOLDER);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_LID_CAP_HOLDER_EMPTY);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_CHEMICAL_COUPLER_HOLDER_EMPTY);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_LID_CAP_SCREWING);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_DRUM_LID_TOTAL);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_CHEMICAL_COUPLER_HOLDER);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_HOLE_SURFACE);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_COUPLER_UNSCREWING);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_LID_CAP_UNSCREWING);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_LID_CAP_HOLDER);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_LID_CAP_HOLDER_EMPTY);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_CHEMICAL_COUPLER_HOLDER_EMPTY);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_LID_CAP_SCREWING);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_HOLDER_WITH_KEY_CODE);
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.push_back(TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_HOLDER_WITHOUT_KEY_CODE);
    ////////////////////////////////////////////////////////


    //// Object data pointer initialize
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_bin_picking_node_target_object_list.clear();
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list.clear();
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_bin_picking_node_target_object_list.resize(bin_picking_target_object_list_.size());
    for (size_t i = 0; i < _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_bin_picking_node_target_object_list.size(); i++)
    {
        CTARGET_OBJECT_DATA tmp;
        _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_bin_picking_node_target_object_list[i] = tmp;
        CTARGET_OBJECT_DATA* ptr_object_data = &_taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_bin_picking_node_target_object_list[i];
        _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list.push_back(ptr_object_data);
    }

    for (size_t i = 0; i < bin_picking_target_object_list_.size(); i++)
    {
        ROS_LOG_WARN("[pushButtonSelectTargetObjectClickedCallback] bin_picking_target_object_list_ #%zu", i);
        // Ensure only one radio is checked at a time so the selected index is deterministic
        for (size_t j = 0; j < bin_picking_target_object_list_.size(); j++) {
            bin_picking_target_object_list_[j]->setChecked(false);
        }
        bin_picking_target_object_list_[i]->setChecked(true);
        _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[i]->is_template_initialized_ = true; // CAD는 matching node에서 초기화하므로 여기선 그냥 true
        pushButtonSelectTargetObjectClickedCallback();
    }

    // Rebuild name->index map from actual UI order
    _taskWindow->getTaskManager()->_qnode->rebuildTargetObjectIndexMap();


    ////////////////////////////////////////////////////////////
    //// KUAIS
    bin_picking_target_object_list_[0]->setChecked(true);
    pushButtonSelectTargetObjectClickedCallback();
    ////////////////////////////////////////////////////////////

    for (size_t i = 0; i < _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_.size(); i++)
    {
        size_t l_idx_now = _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_selected_object_index_[i];
        if (l_idx_now >= _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list.size()) {
            ROS_LOG_ERROR("[Object #%zu] index out of range: %zu >= %zu", l_idx_now+1, l_idx_now, _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list.size());
            continue;
        }
        ROS_LOG_WARN("[Object #%zu] tcp_changing_id: #%zu", l_idx_now+1, _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[l_idx_now]->tcp_changing_id);
        ROS_LOG_WARN("              target_object_: %s", _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[l_idx_now]->m_scan_parameter.target_name.c_str());
    }


    //// TODO: Modbus TCP 추가
}

void MainWindow_widgetsHanyangEng::sendLogMessageStr(std::string &str) {
    // if (ui->textEdit_log->textCursor().position() > 10000) {
    //     ui->textEdit_log->textCursor().deletePreviousChar();
    // }
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

// void MainWindow_widgetsHanyangEng::timerCallback() {

//     pushButtonGetTorqueClickedCallback();
// }

void MainWindow_widgetsHanyangEng::timerCallback() {
    // 토크 폴링이 활성화되어 있을 때만 1초마다 토크 서비스 호출
    if (_taskWindow->getTaskManager()->_qnode->getTorquePollingEnabled()) {
        pushButtonGetTorqueClickedCallback();
    }
}

void MainWindow_widgetsHanyangEng::statusTimerCallback() {


    if (!_taskWindow->getTaskManager()->_qnode->params_.status.is_enable) {
        ui->lineEdit_status_enable->setText("Disabled");
    } else {
        ui->lineEdit_status_enable->setText("Enabled");
    }

    if (_taskWindow->getTaskManager()->_qnode->is_task_mode_) {
        ui->lineEdit_is_task_in_progress->setText("Task in progress...");
        ui->lineEdit_is_task_in_progress_2->setText("Task in progress...");
    } else {
        ui->lineEdit_is_task_in_progress->setText("Ready");
        ui->lineEdit_is_task_in_progress_2->setText("Ready");
    }

    if (_taskWindow->getTaskManager()->_qnode->is_doosan_robot_collision_) {
        ui->lineEdit_is_task_in_progress->setText("Collision!!");
        ui->lineEdit_is_task_in_progress_2->setText("Collision!!");
    }

    // // Image Viewer
    // uploadImage();


#if IS_PLC_COMMUNICATION


    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    //// 250804
    if(_taskWindow->getTaskManager()->_qnode->is_monitoring_plc_read_amr_to_robot_status_) {
        monitorPLCStatus();
    }

    //// 250804
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// PLC Status Monitoring
    //// WORD: 42000
    // [0]: 로봇 작업 시작 (브릿지 다운 후 공병 배출 시작 전)
    // [1]: 로봇 작업 시작 (실병 반입 작업 완료 후)
    // [2]: 공급 설비 Port1 작업 (Left)
    // [3]: 공급 설비 Port2 작업 (Right)
    // [4]: 커플러 "풀기 작업" 시작 응답 (설비에 도착하여 Loading 작업 이전에)
    // [5]: 커플러 "풀기 작업" 완료 응답 (설비에 도착하여 Loading 작업 이전에)
    // [6]: 캡 "체결 작업" 시작 응답 (설비에 도착하여 Loading 작업 이전에)
    // [7]: 캡 "체결 작업" 완료 응답 (설비에 도착하여 Loading 작업 이전에)
    // [8]: 캡 "풀기 작업" 시작 응답 (설비에 도착하여 UnLoading 작업 후)
    // [9]: 캡 "풀기 작업" 완료 응답 (설비에 도착하여 UnLoading 작업 후)
    // [10]: 커플러 "체결 작업" 시작 응답 (설비에 도착하여 UnLoading 작업 후)
    // [11]: 커플러 "체결 작업" 완료 응답 (설비에 도착하여 UnLoading 작업 후)
    // [12]: Barcode Reading OK

    //// AMR to Robot Read 후에, 작업 시작 플래그를 Robot to AMR로 Write

    //// 41000 (AMR Ready PLC Signal)
    // Eigen::VectorXf current_js_postion(_taskWindow->getTaskManager()->_qnode->params_.meas.q[0], _taskWindow->getTaskManager()->_qnode->params_.meas.q[1], _taskWindow->getTaskManager()->_qnode->params_.meas.q[2], _taskWindow->getTaskManager()->_qnode->params_.meas.q[3], _taskWindow->getTaskManager()->_qnode->params_.meas.q[4], _taskWindow->getTaskManager()->_qnode->params_.meas.q[5]);



    //// 42000
    bool is_drum_loading = _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[0];
    bool is_drum_unloading = _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[1];
    bool is_left_drum_task = _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[2];
    bool is_right_drum_task = _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[3];
    bool is_coupler_detach_start_ack = _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[4];
    bool is_coupler_detach_finished_ack = _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[5];
    bool is_cap_screwing_start_ack = _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[6];
    bool is_cap_screwing_finished_ack = _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[7];
    bool is_cap_detach_start_ack = _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[8];
    bool is_cap_detach_finished_ack = _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[9];
    bool is_coupler_screwing_start_ack = _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[10];
    bool is_coupler_screwing_finished_ack = _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[11];
    bool is_barcode_reading_ok = _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[12];

    //// NOTICE: 42000 BIT 13: 41000 BIT 14에 대한 응답 신호
    bool is_robot_task_finished_from_42000_bit13 = _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[13];


    //// 42002
    bool is_rotation_angle_detection_start_ack = _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[0];
    bool is_rotation_angle_detection_finished_ack = _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[1];
    bool is_rotation_angle_detection_ok_ack = _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[2];
    bool is_rotation_angle_detection_ng_ack = _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[3];

    bool is_turntable_robot_task_start = _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[4];
    bool is_barcode_robot_task_start = _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[5];
    bool is_barcode_complete = _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[6];

    bool is_amr_robot_emergency_clear = _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[7];

    //// NOTICE: QNode의 is_monitoring_plc_status_이 true이어야만 실행되도록 처리되어 있음.
    //// NOTICE: 아래의 작업 중에 어느 하나라도 실행되고 나면, is_plc_task_mode_monitoring_ = false로 처리됨.
    //// NOTICE: 다시 어떤 작업이든 PLC 신호에 따라 실행하려고 하면 is_plc_task_mode_monitoring_ = true로 할당해야함.
    //// NOTICE: UI에서 monitoring plc 버튼 클릭하면 true


    bool is_plc_task_in_progress_read_from_4200x_1 = is_coupler_detach_start_ack ||
                                                    is_coupler_detach_finished_ack ||
                                                    is_cap_screwing_start_ack ||
                                                    is_cap_screwing_finished_ack ||
                                                    is_cap_detach_start_ack ||
                                                    is_cap_detach_finished_ack ||
                                                    is_coupler_screwing_start_ack ||
                                                    is_coupler_screwing_finished_ack;

    bool is_plc_task_in_progress_read_from_4200x_2 = is_barcode_reading_ok ||
                                                    is_rotation_angle_detection_start_ack ||
                                                    is_rotation_angle_detection_finished_ack ||
                                                    is_rotation_angle_detection_ok_ack ||
                                                    is_rotation_angle_detection_ng_ack ||
                                                    is_barcode_complete ||
                                                    is_amr_robot_emergency_clear ;


    if (is_amr_robot_emergency_clear == true) {
        // 로봇 정지
        // ROS_LOG_WARN("emergency clear 111111");
        _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1002), true, static_cast<uint16_t>(7), true);
        _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1002), true, static_cast<uint16_t>(7), true);
        _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1002), true, static_cast<uint16_t>(7), true);
        // ROS_LOG_WARN("emergency clear 222222");

        pushButtonSTOPAllCallback();
        ROS_LOG_WARN("PLC Reading..., ADDRESS 2002 - BIT 7: TRUE");

        // 로봇쪽 모든 신호를 false로 초기화 (1000 주소)
        for(int i = 5; i <= 12; ++i) {
            _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, static_cast<uint16_t>(i), false);
        }
        // ROS_LOG_WARN("emergency clear 3333333");
        
        // 비상 정지 응답 신호 전송 (1002 주소의 7번 비트)
        _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1002), true, static_cast<uint16_t>(7), true);

        // ROS_LOG_WARN("emergency clear 444444");

        _taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_ = true;

        // ROS_LOG_WARN("emergency clear 5555555");

        _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;


        // 잠시 후 응답 신호 해제 (필요한 경우)
        // _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1002), true, static_cast<uint16_t>(7), false);
    }


    // if (is_amr_robot_emergency_clear == true) {
    //     ROS_LOG_WARN("amr emergency");
    //     ROS_LOG_WARN("amr emergency");

    //     // 비상 정지 응답 신호 전송 (1002 주소의 7번 비트)
    //     _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1002), true, 7, true);
    //     // 잠시 후 응답 신호 해제 (필요한 경우)
    //     _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1002), true, 7, false);
    // }


#if IS_PLC_LS_XG5000
    uint16_t word_address_in = static_cast<uint16_t>(0x41000);
    uint16_t word_address_41002 = static_cast<uint16_t>(0x41002);
    uint16_t word_address_41003 = static_cast<uint16_t>(0x41003);
#else
    uint16_t word_address_in = static_cast<uint16_t>(1000);
    uint16_t word_address_41002 = static_cast<uint16_t>(1002);
    uint16_t word_address_41003 = static_cast<uint16_t>(1003);
#endif

    bool is_plc_signal_test = false;

    if(_taskWindow->getTaskManager()->_qnode->is_monitoring_plc_read_amr_to_robot_status_) {
        checkPLCDoosanRobotCollision();
    }




    uint16_t bit_address_in = static_cast<uint16_t>(13);


#if DO_NOT_TRY_HANYANG_ENG_TASK

    //// Monitoring
    if(_taskWindow->getTaskManager()->_qnode->is_monitoring_plc_read_amr_to_robot_status_) { // 모니터링 버튼 클릭 시에만 출력
        if(is_barcode_reading_ok) {
            ROS_LOG_WARN("PLC Reading..., ADDRESS 2000 - BIT 12: TRUE");

        } else {
            ROS_LOG_WARN("PLC Reading..., ADDRESS 2000 - BIT 12: FALSE");
        }
    }



#else
    ROS_LOG_WARN("HANYANG TASK ENABLED!!!");
    // if (_taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_) {
        if(!_taskWindow->getTaskManager()->_qnode->is_task_mode_) { // 작업 모드가 아닌 경우에만 모니터링 하도록 처리
            std::vector<double> current_js_postion;
            arr2Vec(_taskWindow->getTaskManager()->_qnode->params_.meas.q, current_js_postion);
            if(1) {
                // Left Home Pose
                std::vector<double> left_home_position = {-0.143, 18.822, -105.523, 179.931, 93.345, -89.115};
                double norm_left = computeVectorNormTmp(current_js_postion, left_home_position);
                // Right Home Pose
                std::vector<double> right_home_position = {-0.000, -18.713, 105.394, -0.139, 93.361, -90.833};
                double norm_right = computeVectorNormTmp(current_js_postion, right_home_position);
                // if(norm_left < 1e-3) {
                if(norm_left < 1) {
                    ROS_LOG_WARN("LEFT AMR HOME READY!");
                    if(!is_plc_signal_test) {
                        // if(is_robot_task_finished_from_42000_bit13) {
                        if(1) {
                            _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_in, true, 0, false);
                            _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_41002, true, 0, false);
                        }
                        ROS_LOG_INFO("[%s] lines: %d", __func__, __LINE__);
                        _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(word_address_in, true, bit_address_in, true);
                        ROS_LOG_INFO("[%s] lines: %d", __func__, __LINE__);

                    }
                // } else if(norm_right < 1e-3) {
                } else if(norm_right < 1) {
                    ROS_LOG_WARN("RIGHT AMR HOME READY!");
                    if(!is_plc_signal_test) {
                        // if(is_robot_task_finished_from_42000_bit13) {
                        if(1) {
                            _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_in, true, 0, false);
                            _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_41002, true, 0, false);
                        }
                        // ROS_LOG_INFO("[%s] lines: %d", __func__, __LINE__);
                        _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(word_address_in, true, bit_address_in, true);
                        // ROS_LOG_INFO("[%s] lines: %d", __func__, __LINE__);
                    }
                } else {
                    if(!is_plc_signal_test) {
                        ROS_LOG_INFO("[%s] lines: %d", __func__, __LINE__);
                        _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(word_address_in, true, bit_address_in, false);
                        ROS_LOG_INFO("[%s] lines: %d", __func__, __LINE__);
                    }
                }
            }
        }
    // }

    if(0) {
        if(is_robot_task_finished_from_42000_bit13 && !is_already_initialized_) {

            _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;

            is_already_initialized_ = true;
            is_robot_task_finished_from_42000_bit13 = false;
            ROS_LOG_WARN("TASK FINISHED FLAG RECEIVED!");
            ROS_LOG_WARN("TASK FINISHED FLAG RECEIVED!");
            ROS_LOG_WARN("TASK FINISHED FLAG RECEIVED!");
            ROS_LOG_WARN("TASK FINISHED FLAG RECEIVED!");
            ROS_LOG_WARN("TASK FINISHED FLAG RECEIVED!");
            ROS_LOG_WARN("TASK FINISHED FLAG RECEIVED!");
            ROS_LOG_WARN("TASK FINISHED FLAG RECEIVED!");
            ROS_LOG_WARN("TASK FINISHED FLAG RECEIVED!");
            ROS_LOG_WARN("TASK FINISHED FLAG RECEIVED!");
            _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_in, true, 0, false);
            // for(int i = 0; i < plc_hanyang_task_flag_check_box_41000_.size(); ++i) {
            //     if(!_taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_) {
            //         // _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(1000, true, i, false);
            //     }
            //     _taskWindow->getTaskManager()->_qnode->plc_status_word_41000_[i] = false; // is_rotation_angle_detection_finished_ack
            // }
        }


        if((is_barcode_complete || is_rotation_angle_detection_finished_ack) && !is_already_initialized_) { // WRITE 신호 모두 끊기
            is_already_initialized_ = true;
            _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
            _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_41002, true, 0, false);
        }

        //////////////////////////////////////////////////////////////////////
        //// is_plc_task_in_progress_ 플래그 초기화
        if(is_robot_task_finished_from_42000_bit13) { // WRITE 신호 모두 끊기
            _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
        }
        //////////////////////////////////////////////////////////////////////
    }



    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// 캡 풀기 & 커플러 체결
    // bool is_cap_detach_or_coupler_screwing_task = is_cap_detach_start_ack && is_coupler_screwing_start_ack;

    // if(is_drum_unloading && !is_drum_loading && !is_plc_task_in_progress_read_from_4200x_1) {
    if(is_drum_unloading && !is_drum_loading) {

        // Port 1 (Left): 캡 풀기 & 커플러 체결
        if(is_left_drum_task && !is_right_drum_task) {
            // _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
            is_already_initialized_ = false;
            PLCCommTaskCapDetachAndCouplerScrewingLeft();
        }

        // Port 2 (Right): 캡 풀기 & 커플러 체결
        if(is_right_drum_task && !is_left_drum_task) {
            // _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
            is_already_initialized_ = false;
            PLCCommTaskCapDetachAndCouplerScrewingRight();
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// 커플러 풀기 & 캡 체결
    // bool is_coupler_detach_or_cap_screwing_task = is_coupler_detach_start_ack && is_cap_screwing_start_ack;
    // if(is_drum_unloading && !is_drum_loading && is_coupler_detach_or_cap_screwing_task) {
    if(!is_drum_unloading && is_drum_loading && !is_plc_task_in_progress_read_from_4200x_1) {

        // Port 1 (Left): 커플러 풀기 & 캡 체결
        if(is_left_drum_task && !is_right_drum_task) {
            // _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
            is_already_initialized_ = false;
            PLCCommTaskCouplerDetachAndCapScrewingLeft();
        }

        // Port 2 (Right): 커플러 풀기 & 캡 체결
        if(is_right_drum_task && !is_left_drum_task) {
            // _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
            is_already_initialized_ = false;
            PLCCommTaskCouplerDetachAndCapScrewingRight();
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////


    if(1) {
        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        //// 회전각도 인식 작업
        if(is_turntable_robot_task_start && !is_plc_task_in_progress_read_from_4200x_2) {

            // _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
            is_already_initialized_ = false;
            // if(is_rotation_angle_detection_mode) {
                // _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[1] = false; // is_rotation_angle_detection_finished_ack
                PLCCommTaskRotationAngleDetection();
            // }
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////

    }

    if(0) {
        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        //// 바코드 인식 작업
        if(is_barcode_robot_task_start && !is_plc_task_in_progress_read_from_4200x_2) {

            // _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
            is_already_initialized_ = false;
            // if(is_rotation_angle_detection_mode) {
                // _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[1] = false; // is_rotation_angle_detection_finished_ack
                PLCCommTaskBarcodeReading();
            // }
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////

    }
#endif

    //// PLC SIMULATION WRITE TEST
    // if(_taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_) {
    if(1) {
        for(int i = 0; i < plc_hanyang_task_flag_check_box_41000_.size(); ++i) {
            plc_hanyang_task_flag_check_box_41000_[i]->setChecked(_taskWindow->getTaskManager()->_qnode->plc_status_word_41000_[i]);
            plc_hanyang_task_flag_check_box_41002_[i]->setChecked(_taskWindow->getTaskManager()->_qnode->plc_status_word_41002_[i]);
            plc_hanyang_task_flag_check_box_41003_[i]->setChecked(_taskWindow->getTaskManager()->_qnode->plc_status_word_41002_[i]);
        }
    }

    //// PLC SIMULATION READ TEST
    if(!_taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_) {
        for(int i = 0; i < plc_hanyang_task_flag_check_box_42000_.size(); ++i) {
            plc_hanyang_task_flag_check_box_42000_[i]->setChecked(_taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[i]);
            plc_hanyang_task_flag_check_box_42002_[i]->setChecked(_taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[i]);
        }
    }
#endif

}


void MainWindow_widgetsHanyangEng::PLCCommTaskCapDetach() {

}

void MainWindow_widgetsHanyangEng::PLCCommTaskCouplerScrewing() {

}

void MainWindow_widgetsHanyangEng::PLCCommTaskCouplerDetach() {

}

void MainWindow_widgetsHanyangEng::PLCCommTaskCapScrewing() {

}

void MainWindow_widgetsHanyangEng::PLCCommTaskCapDetachAndCouplerScrewingLeft() {
    if (_taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_) {
        if (!_taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_) {

            bool is_robot_ready = !_taskWindow->getTaskManager()->_qnode->is_task_mode_;
            if(is_robot_ready) {
                //// Port 1 (Left)
                // _taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_ = false; // AMR로부터 PLC 수신을 위한 모니터링 정지
                _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = true; // AMR로부터 PLC 수신을 위한 모니터링 정지
                // ui->pushButton_MODBUS_MonitoringPLC->setChecked(false); // 버튼 해제 상태 활성화

                if(_taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_) {
                    _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[9] = false; // is_cap_detach_finished_ack
                    _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[11] = false; // is_coupler_screwing_finished_ack
                }


                ROS_LOG_WARN("*****************************************************");
                ROS_LOG_WARN("********** Port 1 - Left Drum Task Start! **********");
                ROS_LOG_WARN("*****************************************************");

                // //// NOTICE: From robot to AMR PLC Write (작업 시작 플래그)
                // _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(0x41000, true, 0, true);

                ui->pushButton_doTask_plc_comm_capDetachAndCouplerScrewingLeft->setChecked(true); // 버튼 누른 상태 활성화
                pushButtonDoTasksCapDetachAndCouplerScrewingLeftCallback();
            }

        } else {
            //// Port 1 (Left)
            if(ui->pushButton_doTask_plc_comm_capDetachAndCouplerScrewingLeft->isChecked())
            {
                if(!_taskWindow->getTaskManager()->_qnode->is_task_mode_) // 작업이 끝난 경우
                {
                    ROS_LOG_WARN("*****************************************************");
                    ROS_LOG_WARN("********* Port 1 - Left Drum Task Finished! ********");
                    ROS_LOG_WARN("*****************************************************");
                    pushButtonSTOPAllCallback();
                    ui->pushButton_doTask_plc_comm_capDetachAndCouplerScrewingLeft->setChecked(false);

                    if(_taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_) {
                        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[8] = false; // is_cap_detach_start_ack
                        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[9] = true; // is_cap_detach_finished_ack
                        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[10] = false; // is_coupler_screwing_start_ack
                        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[11] = true; // is_coupler_screwing_finished_ack
                    }
                }
            }
        }
    }
}

void MainWindow_widgetsHanyangEng::PLCCommTaskCapDetachAndCouplerScrewingRight() {
    if (_taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_) {
        if (!_taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_) {
            bool is_robot_ready = !_taskWindow->getTaskManager()->_qnode->is_task_mode_;
            if(is_robot_ready) {
                //// Port 2 (Right)
                // _taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_ = false; // AMR로부터 PLC 수신을 위한 모니터링 정지
                _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = true; // AMR로부터 PLC 수신을 위한 모니터링 정지
                // ui->pushButton_MODBUS_MonitoringPLC->setChecked(false); // 버튼 누른 상태 활성화

                if(_taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_) {
                    _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[9] = false; // is_cap_detach_finished_ack
                    _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[11] = false; // is_coupler_screwing_finished_ack
                }
                ROS_LOG_WARN("*****************************************************");
                ROS_LOG_WARN("********** Port 2 - Right Drum Task Start! **********");
                ROS_LOG_WARN("*****************************************************");

                // //// NOTICE: From robot to AMR PLC Write (작업 시작 플래그)
                // _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(0x41000, true, 0, true);

                ui->pushButton_doTask_plc_comm_capDetachAndCouplerScrewingRight->setChecked(true); // 버튼 누른 상태 활성화
                pushButtonDoTasksCapDetachAndCouplerScrewingRightCallback();
            }

        } else {
            //// Port 2 (Right)
            if(ui->pushButton_doTask_plc_comm_capDetachAndCouplerScrewingRight->isChecked())
            {
                if(!_taskWindow->getTaskManager()->_qnode->is_task_mode_) // 작업이 끝난 경우
                {
                    ROS_LOG_WARN("*****************************************************");
                    ROS_LOG_WARN("********* Port 2 - Right Drum Task Finished! ********");
                    ROS_LOG_WARN("*****************************************************");
                    pushButtonSTOPAllCallback();
                    ui->pushButton_doTask_plc_comm_capDetachAndCouplerScrewingRight->setChecked(false);

                    if(_taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_) {
                        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[8] = false; // is_cap_detach_start_ack
                        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[9] = true; // is_cap_detach_finished_ack
                        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[10] = false; // is_coupler_screwing_start_ack
                        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[11] = true; // is_coupler_screwing_finished_ack
                    }
                }
            }
        }
    }
}

void MainWindow_widgetsHanyangEng::PLCCommTaskCouplerDetachAndCapScrewingLeft() {
    if (_taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_) {
        if (!_taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_) {
            bool is_robot_ready = !_taskWindow->getTaskManager()->_qnode->is_task_mode_;
            if(is_robot_ready) {
                //// Port 1 (Left) 커플러 풀기, 캡 체결
                // _taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_ = false; // AMR로부터 PLC 수신을 위한 모니터링 정지
                _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = true; // AMR로부터 PLC 수신을 위한 모니터링 정지
                // ui->pushButton_MODBUS_MonitoringPLC->setChecked(false); // 버튼 누른 상태 활성화

                if(_taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_) {
                    _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[5] = false; // is_coupler_detach_finished_ack
                    _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[7] = false; // is_cap_screwing_finished_ack
                }
                ROS_LOG_WARN("*****************************************************");
                ROS_LOG_WARN("********** Port 1 - Left Drum Task Start! **********");
                ROS_LOG_WARN("*****************************************************");

                // //// NOTICE: From robot to AMR PLC Write (작업 시작 플래그)
                // _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(0x41000, true, 0, true);

                ui->pushButton_doTask_plc_comm_couplerDetachAndCapScrewingLeft->setChecked(true); // 버튼 누른 상태 활성화
                pushButtonDoTasksCouplerDetachAndCapScrewingLeftCallback();
            }
        } else {
            //// Port 1 (Left) 커플러 풀기, 캡 체결
            if(ui->pushButton_doTask_plc_comm_couplerDetachAndCapScrewingLeft->isChecked())
            {
                if(!_taskWindow->getTaskManager()->_qnode->is_task_mode_) // 작업이 끝난 경우
                {
                    ROS_LOG_WARN("*****************************************************");
                    ROS_LOG_WARN("********* Port 1 - Left Drum Task Finished! ********");
                    ROS_LOG_WARN("*****************************************************");
                    pushButtonSTOPAllCallback();
                    ui->pushButton_doTask_plc_comm_couplerDetachAndCapScrewingLeft->setChecked(false);

                    if(_taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_) {

                        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[4] = false; // is_coupler_detach_start_ack
                        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[5] = true; // is_coupler_detach_finished_ack
                        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[6] = false; // is_cap_screwing_start_ack
                        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[7] = true; // is_cap_screwing_finished_ack
                    }
                }
            }
        }
    }
}

// is_plc_task_mode_monitoring_ = false 면 작업 실행 안됨, 작업 하나 끝나면 바로 false 처리
// is_plc_task_in_progress_ = true 면 작업 실행 안됨 
void MainWindow_widgetsHanyangEng::PLCCommTaskCouplerDetachAndCapScrewingRight() {
    if (_taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_) {
        if (!_taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_) {
            bool is_robot_ready = !_taskWindow->getTaskManager()->_qnode->is_task_mode_;
            if(is_robot_ready) {
                //// Port 2 (Right)
                // _taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_ = false; // AMR로부터 PLC 수신을 위한 모니터링 정지
                _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = true; // AMR로부터 PLC 수신을 위한 모니터링 정지
                // ui->pushButton_MODBUS_MonitoringPLC->setChecked(false); // 버튼 누른 상태 활성화

                if(_taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_) {
                    _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[5] = false; // is_coupler_detach_finished_ack
                    _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[7] = false; // is_cap_screwing_finished_ack
                }
                ROS_LOG_WARN("*****************************************************");
                ROS_LOG_WARN("********** Port 2 - Right Drum Task Start! **********");
                ROS_LOG_WARN("*****************************************************");

                // //// NOTICE: From robot to AMR PLC Write (작업 시작 플래그)
                // _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(0x41000, true, 0, true);

                ui->pushButton_doTask_plc_comm_couplerDetachAndCapScrewingRight->setChecked(true); // 버튼 누른 상태 활성화
                pushButtonDoTasksCouplerDetachAndCapScrewingRightCallback();
            }

        } else {
            //// Port 2 (Right)
            if(ui->pushButton_doTask_plc_comm_couplerDetachAndCapScrewingRight->isChecked())
            {
                if(!_taskWindow->getTaskManager()->_qnode->is_task_mode_) // 작업이 끝난 경우
                {
                    ROS_LOG_WARN("*****************************************************");
                    ROS_LOG_WARN("********* Port 2 - Right Drum Task Finished! ********");
                    ROS_LOG_WARN("*****************************************************");
                    pushButtonSTOPAllCallback();
                    ui->pushButton_doTask_plc_comm_couplerDetachAndCapScrewingRight->setChecked(false);
                    if(_taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_) {
                        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[4] = false; // is_coupler_detach_start_ack
                        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[5] = true; // is_coupler_detach_finished_ack
                        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[6] = false; // is_cap_screwing_start_ack
                        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[7] = true; // is_cap_screwing_finished_ack
                    }


                }
            }
        }
    }
}

void MainWindow_widgetsHanyangEng::PLCCommTaskRotationAngleDetection() {
    if (_taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_) {
        if (!_taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_) {

            bool is_robot_ready = !_taskWindow->getTaskManager()->_qnode->is_task_mode_;
            if(is_robot_ready) {

                // _taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_ = false; // AMR로부터 PLC 수신을 위한 모니터링 정지
                _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = true; // AMR로부터 PLC 수신을 위한 모니터링 정지
                // ui->pushButton_MODBUS_MonitoringPLC->setChecked(false); // 버튼 누른 상태 활성화

                if(_taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_) {
                    _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[1] = false; // is_rotation_angle_detection_finished_ack
                }
                ROS_LOG_WARN("*****************************************************");
                ROS_LOG_WARN("********** Rotation Angle Detection Task! **********");
                ROS_LOG_WARN("*****************************************************");

                // //// NOTICE: From robot to AMR PLC Write (작업 시작 플래그)
                // _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(0x41000, true, 0, true);

                ui->pushButton_doTask_plc_comm_rotationAngleDetection->setChecked(true); // 버튼 누른 상태 활성화
                pushButtonDoTasksRotationAngleDetectionCallback();
            }

        } else {

            if(ui->pushButton_doTask_plc_comm_rotationAngleDetection->isChecked())
            {
                if(!_taskWindow->getTaskManager()->_qnode->is_task_mode_) // 작업이 끝난 경우
                {
                    ROS_LOG_WARN("*****************************************************");
                    ROS_LOG_WARN("****** Rotation Angle Detection Task Finished! ******");
                    ROS_LOG_WARN("*****************************************************");
                    pushButtonSTOPAllCallback();
                    ui->pushButton_doTask_plc_comm_rotationAngleDetection->setChecked(false);

                    _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[0] = false; // is_rotation_angle_detection_start_ack
                    _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[1] = true; // is_rotation_angle_detection_finished_ack

                }
            }
        }
    }
}


void MainWindow_widgetsHanyangEng::PLCCommTaskBarcodeReading() {
    if (_taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_) {
        if (!_taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_) {

            bool is_robot_ready = !_taskWindow->getTaskManager()->_qnode->is_task_mode_;
            if(is_robot_ready) {

                // _taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_ = false; // AMR로부터 PLC 수신을 위한 모니터링 정지
                _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = true; // AMR로부터 PLC 수신을 위한 모니터링 정지
                // ui->pushButton_MODBUS_MonitoringPLC->setChecked(false); // 버튼 누른 상태 활성화

                if(_taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_) {
                    _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[1] = false; // is_rotation_angle_detection_finished_ack
                }
                ROS_LOG_WARN("*****************************************************");
                ROS_LOG_WARN("********** Barcode Detection Task! **********");
                ROS_LOG_WARN("*****************************************************");

                // //// NOTICE: From robot to AMR PLC Write (작업 시작 플래그)
                // _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(0x41000, true, 0, true);

                ui->pushButton_doTask_plc_comm_barcodeDetection->setChecked(true); // 버튼 누른 상태 활성화
                pushButtonDoTasksBarcodeDetectionCallback();
            }

        } else {

            if(ui->pushButton_doTask_plc_comm_barcodeDetection->isChecked())
            {
                if(!_taskWindow->getTaskManager()->_qnode->is_task_mode_) // 작업이 끝난 경우
                {
                    ROS_LOG_WARN("*****************************************************");
                    ROS_LOG_WARN("****** Barcode Detection Task Finished! ******");
                    ROS_LOG_WARN("*****************************************************");
                    pushButtonSTOPAllCallback();
                    ui->pushButton_doTask_plc_comm_barcodeDetection->setChecked(false);

                    // _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[0] = false; // is_rotation_angle_detection_start_ack
                    // _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[1] = true; // is_rotation_angle_detection_finished_ack

                }
            }
        }
    }
}

////
void MainWindow_widgetsHanyangEng::testBtnCallback() {

    int ret = QMessageBox::warning(this,
                                   tr("Exit Program"),
                                   tr("Are you sure you want to exit?"),
                                   QMessageBox::Ok | QMessageBox::Cancel,
                                   QMessageBox::Ok); //종료시 메세지 박스 출력...
    switch(ret)
    {
        case QMessageBox::Ok:
        {
            for(int i = 0; i < 3; i++) {
                ROS_LOG_WARN("OK");
                // QProcess::execute("/bin/bash", {"-c", "bash -c \"killall ur_controller && exit\""});
                // QProcess::execute("/bin/bash", {"-c", "bash -c \"killall grp_control && exit\""});
                // QProcess::execute("/bin/bash", {"-c", "bash -c \"killall zivid_scan_node && exit\""});
                // QProcess::execute("/bin/bash", {"-c", "bash -c \"killall matching_node && exit\""});

                QProcess::execute("/bin/bash", {"-c", "bash -c \"killall kr_sys && exit\""});
            }
            break;
        }
        case QMessageBox::Cancel:
        {
            ROS_LOG_WARN("Cancel");
            return;
        }
    }
}

void MainWindow_widgetsHanyangEng::keyPressEvent(QKeyEvent *event) {
    // if (event->key() == Qt::Key_Space && !isRecording && _taskWindow->getTaskManager()->_qnode->is_llm_task_fin) {
    //     isRecording = true;
    //     ui->pushButton_recording->setDown(true);
    //     // RecordingStartCallback();
    // }
}

void MainWindow_widgetsHanyangEng::keyReleaseEvent(QKeyEvent *event) {
    // if (event->key() == Qt::Key_Space && isRecording && _taskWindow->getTaskManager()->_qnode->is_llm_task_fin) {
    //     isRecording = false;
    //     ui->pushButton_recording->setDown(false);
    //     // RecordingStopCallback();
    // }
}


void MainWindow_widgetsHanyangEng::setupPlanner() {
    try {
        std::cout << "setupPlanner() called" << std::endl;
        
        if (!_taskWindow) {
            std::cerr << "_taskWindow is null!" << std::endl;
            ROS_LOG_ERROR("_taskWindow is null");
            hanyangEngPlanner_ = nullptr;
            return;
        }
        
        if (!_taskWindow->getTaskManager()) {
            std::cerr << "getTaskManager() returned null!" << std::endl;
            ROS_LOG_ERROR("getTaskManager() returned null");
            hanyangEngPlanner_ = nullptr;
            return;
        }
        
        if (!_taskWindow->getTaskManager()->_qnode) {
            std::cerr << "_qnode is null!" << std::endl;
            ROS_LOG_ERROR("_qnode is null");
            hanyangEngPlanner_ = nullptr;
            return;
        }
        
        if (!_taskWindow->getTaskManager()->_qnode->task_planner_) {
            std::cerr << "task_planner_ is null!" << std::endl;
            ROS_LOG_ERROR("task_planner_ is null");
            hanyangEngPlanner_ = nullptr;
            return;
        }
        
        std::cout << "Attempting to cast task_planner_ to TaskPlannerHanyangEng..." << std::endl;
        std::cout << "task_planner_ type: " << typeid(*_taskWindow->getTaskManager()->_qnode->task_planner_.get()).name() << std::endl;
        
        hanyangEngPlanner_ = dynamic_cast<TaskPlannerHanyangEng*>(_taskWindow->getTaskManager()->_qnode->task_planner_.get());
        if (!hanyangEngPlanner_) {
            std::cerr << "Failed to cast task_planner_ to TaskPlannerHanyangEng!" << std::endl;
            ROS_LOG_ERROR("Failed to cast task_planner_ to TaskPlannerHanyangEng");
            
            // 대안: 직접 TaskPlannerHanyangEng 생성
            std::cout << "Creating new TaskPlannerHanyangEng instance..." << std::endl;
            try {
                hanyangEngPlanner_ = new TaskPlannerHanyangEng();
                std::cout << "Successfully created new TaskPlannerHanyangEng instance" << std::endl;
                ROS_LOG_INFO("Created new TaskPlannerHanyangEng instance");
            } catch (const std::exception& e) {
                std::cerr << "Failed to create TaskPlannerHanyangEng: " << e.what() << std::endl;
                ROS_LOG_ERROR("Failed to create TaskPlannerHanyangEng: %s", e.what());
                hanyangEngPlanner_ = nullptr;
            }
        } else {
            std::cout << "TaskPlannerHanyangEng successfully initialized via dynamic_cast" << std::endl;
            ROS_LOG_INFO("TaskPlannerHanyangEng successfully initialized via dynamic_cast");
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception during planner setup: " << e.what() << std::endl;
        ROS_LOG_ERROR("Exception during planner setup: %s", e.what());
        hanyangEngPlanner_ = nullptr;
    } catch (...) {
        std::cerr << "Unknown exception during planner setup!" << std::endl;
        ROS_LOG_ERROR("Unknown exception during planner setup");
        hanyangEngPlanner_ = nullptr;
    }
}

void MainWindow_widgetsHanyangEng::pushButtonSTOPAllCallback() {
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

bool MainWindow_widgetsHanyangEng::doTargetTask(const std::vector<UnitTask> &target_task) {
    if (target_task.size() > 0) {
        //// Parameters setting
        _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[_taskWindow->getTaskManager()->_qnode->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_save_data = ui->radioButton_assemblyUI_scanDataSave->isChecked();
        _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[_taskWindow->getTaskManager()->_qnode->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_image_processing = ui->radioButton_assemblyUI_scanDoImageProcessing->isChecked();
        _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[_taskWindow->getTaskManager()->_qnode->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching = false; // matching topic

        //// Target task
        _taskWindow->getTaskManager()->_qnode->current_task_list_ = target_task;
#if BIN_PICKING_FLAG
        ROS_LOG_WARN("[%s] BIN_PICKING_FLAG: TRUE", __func__);
        _taskWindow->getTaskManager()->_qnode->beforeBinPickingTaskStart(true);
#else
        ROS_LOG_WARN("[%s] BIN_PICKING_FLAG: FALSE", __func__);
        _taskWindow->getTaskManager()->_qnode->beforeTaskStart();
#endif
        _taskWindow->getTaskManager()->_qnode->task_cycle_ = 0;
        return true;
    } else {
        ROS_LOG_WARN("[%s] target_task.size() == 0", __func__);
        return false;
    }
}





void MainWindow_widgetsHanyangEng::rightPushButtonDoDrumTotalTasksCallback() {
    if(ui->rightPushButton_doTask_drum_total_task->isChecked()) {
        // 포인터 검증 추가
        if (!hanyangEngPlanner_) {
            ROS_LOG_ERROR("hanyangEngPlanner_ is null! Attempting to reinitialize...");
            setupPlanner();
            if (!hanyangEngPlanner_) {
                QMessageBox::critical(this, "Error", "Failed to initialize TaskPlannerHanyangEng!");
                ui->rightPushButton_doTask_drum_total_task->setChecked(false);
                return;
            }
        }
        
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        try {
            hanyangEngPlanner_->rightGenDrumTask();
            ROS_LOG_WARN("TASK GENERATED!");
            //// Step 2) Do Target Tasks
            if (doTargetTask(hanyangEngPlanner_->right_drum_total_task_)) {
                ROS_LOG_WARN("DO CHEMICAL TOTAL TASK!");
            } else {
                QMessageBox::information(this, "Task Execution", "No Tasks Exist");
            }
        } catch (const std::exception& e) {
            ROS_LOG_ERROR("Exception in rightGenDrumTask: %s", e.what());
            QMessageBox::critical(this, "Error", QString("Task generation failed: %1").arg(e.what()));
            ui->rightPushButton_doTask_drum_total_task->setChecked(false);
        } catch (...) {
            ROS_LOG_ERROR("Unknown exception in rightGenDrumTask");
            QMessageBox::critical(this, "Error", "Unknown error occurred during task generation");
            ui->rightPushButton_doTask_drum_total_task->setChecked(false);
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}


void MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask1Callback() {
    if(ui->rightPushButton_doTask_drum_sub_task_1->isChecked()) {
        // 포인터 검증 추가
        if (!hanyangEngPlanner_) {
            ROS_LOG_ERROR("hanyangEngPlanner_ is null! Attempting to reinitialize...");
            setupPlanner();
            if (!hanyangEngPlanner_) {
                QMessageBox::critical(this, "Error", "Failed to initialize TaskPlannerHanyangEng!");
                ui->rightPushButton_doTask_drum_sub_task_1->setChecked(false);
                return;
            }
        }
        
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 1! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        try {
            hanyangEngPlanner_->rightGenDrumTask();
            ROS_LOG_WARN("TASK GENERATED!");
            //// Step 2) Do Target Tasks
            if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_1_)) {
                ROS_LOG_WARN("DO CHEMICAL SUB TASK 1!");
            } else {
                QMessageBox::information(this, "Task Execution", "No Tasks Exist");
            }
        } catch (const std::exception& e) {
            ROS_LOG_ERROR("Exception in rightGenDrumTask: %s", e.what());
            QMessageBox::critical(this, "Error", QString("Task generation failed: %1").arg(e.what()));
            ui->rightPushButton_doTask_drum_sub_task_1->setChecked(false);
        } catch (...) {
            ROS_LOG_ERROR("Unknown exception in rightGenDrumTask");
            QMessageBox::critical(this, "Error", "Unknown error occurred during task generation");
            ui->rightPushButton_doTask_drum_sub_task_1->setChecked(false);
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask2Callback() {
    if(ui->rightPushButton_doTask_drum_sub_task_2->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 2! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_2_)) {
            ROS_LOG_WARN("DO CHEMICAL SUB TASK 2!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask3Callback() {
    if(ui->rightPushButton_doTask_drum_sub_task_3->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 3! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_3_)) {
            ROS_LOG_WARN("DO CHEMICAL SUB TASK 3!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask4Callback() {
    if(ui->rightPushButton_doTask_drum_sub_task_4->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 4! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_4_)) {
            ROS_LOG_WARN("DO CHEMICAL SUB TASK 4!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask5Callback() {
    if(ui->rightPushButton_doTask_drum_sub_task_5->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 5! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_5_)) {
            ROS_LOG_WARN("DO CHEMICAL SUB TASK 5!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask6Callback() {
    if(ui->rightPushButton_doTask_drum_sub_task_6->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 6! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_6_)) {
            ROS_LOG_WARN("DO CHEMICAL SUB TASK 6!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask7Callback() {
    if(ui->rightPushButton_doTask_drum_sub_task_8->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 7! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_7_)) {
            ROS_LOG_WARN("DO CHEMICAL SUB TASK 7!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask9Callback() {
    if(ui->rightPushButton_doTask_drum_sub_task_9->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 7! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_8_)) {
            ROS_LOG_WARN("DO CHEMICAL SUB TASK 7!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask10Callback() {
    if(ui->rightPushButton_doTask_drum_sub_task_10->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 7! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_9_)) {
            ROS_LOG_WARN("DO CHEMICAL SUB TASK 7!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonDrumSubTask11Callback() {
    if(ui->rightPushButton_doTask_drum_sub_task_11->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 7! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_10_)) {
            ROS_LOG_WARN("DO CHEMICAL SUB TASK 7!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

//// 로봇 기능 테스트 위젯
void MainWindow_widgetsHanyangEng::rightPushButtonRobotManualMode(){
#if DRFL_CONTROL
    _taskWindow->getTaskManager()->_qnode->drflSetRobotMode(0);
#endif
}
void MainWindow_widgetsHanyangEng::rightPushButtonRobotAutoMode(){
#if DRFL_CONTROL
    _taskWindow->getTaskManager()->_qnode->drflSetRobotMode(1);
#endif
}

void MainWindow_widgetsHanyangEng::rightPushButtonKeyCodeTest() {
    if(ui->rightPushButton_keycode_test->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("KeyCode Task ...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_11_)) {
            ROS_LOG_WARN("DO CHEMICAL TOTAL TASK!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonKeyCodeUnscrewing() {
    if(ui->rightPushButton_keycode_unscrewing->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("KeyCode Task ...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_12_)) {
            ROS_LOG_WARN("DO CHEMICAL TOTAL TASK!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonHanyangHoseTest1() {
    if(ui->rightPushButton_hanyang_hose_test1->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("KeyCode Task ...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_13_)) {
            ROS_LOG_WARN("DO CHEMICAL TOTAL TASK!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonHanyangHoseTest2() {
    if(ui->rightPushButton_hanyang_hose_test2->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("KeyCode Task ...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_14_)) {
            ROS_LOG_WARN("DO CHEMICAL TOTAL TASK!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonHanyangHoseTest3() {
    if(ui->rightPushButton_hanyang_hose_test3->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("KeyCode Task ...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_15_)) {
            ROS_LOG_WARN("DO CHEMICAL TOTAL TASK!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonHanyangHoseTest4() {
    if(ui->rightPushButton_hanyang_hose_test4->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("KeyCode Task ...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_16_)) {
            ROS_LOG_WARN("DO CHEMICAL TOTAL TASK!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonHanyang0918Test1() {
    if(ui->rightPushButton_hanyang_0918_test1->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("KeyCode Task ...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_17_)) {
            ROS_LOG_WARN("DO CHEMICAL TOTAL TASK!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonHanyang0918Test2() {
    if(ui->rightPushButton_hanyang_0918_test2->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("KeyCode Task ...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_18_)) {
            ROS_LOG_WARN("DO CHEMICAL TOTAL TASK!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonHanyang0918Test3() {
    if(ui->rightPushButton_hanyang_0918_test3->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("KeyCode Task ...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_19_)) {
            ROS_LOG_WARN("DO CHEMICAL TOTAL TASK!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonHanyang0918Test4() {
    if(ui->rightPushButton_hanyang_0918_test4->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("KeyCode Task ...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_20_)) {
            ROS_LOG_WARN("DO CHEMICAL TOTAL TASK!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonHanyang0918Test5() {
    if(ui->rightPushButton_hanyang_0918_test5->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("KeyCode Task ...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_21_)) {
            ROS_LOG_WARN("DO CHEMICAL TOTAL TASK!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonHanyang0918Test6() {
    if(ui->rightPushButton_hanyang_0918_test6->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("KeyCode Task ...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_22_)) {
            ROS_LOG_WARN("DO CHEMICAL TOTAL TASK!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonHanyang0918Test7() {
    if(ui->rightPushButton_hanyang_0918_test7->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("KeyCode Task ...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_23_)) {
            ROS_LOG_WARN("DO CHEMICAL TOTAL TASK!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonHanyang0918Test8() {
    if(ui->rightPushButton_hanyang_0918_test8->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("KeyCode Task ...");
        hanyangEngPlanner_->rightGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_sub_task_24_)) {
            ROS_LOG_WARN("DO CHEMICAL TOTAL TASK!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::leftPushButtonDoDrumTotalTasksCallback() {
    if(ui->leftPushButton_doTask_drum_total_task->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->leftGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->left_drum_total_task_)) {
            ROS_LOG_WARN("DO CHEMICAL TOTAL TASK!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::leftPushButtonDrumSubTask1Callback() {
    if(ui->leftPushButton_doTask_drum_sub_task_1->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 8! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->leftGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->left_drum_sub_task_1_)) {
            ROS_LOG_WARN("DO CHEMICAL SUB TASK 8!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::leftPushButtonDrumSubTask2Callback() {
    if(ui->leftPushButton_doTask_drum_sub_task_2->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 8! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->leftGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->left_drum_sub_task_2_)) {
            ROS_LOG_WARN("DO CHEMICAL SUB TASK 8!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::leftPushButtonDrumSubTask3Callback() {
    if(ui->leftPushButton_doTask_drum_sub_task_3->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 8! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->leftGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->left_drum_sub_task_3_)) {
            ROS_LOG_WARN("DO CHEMICAL SUB TASK 8!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::leftPushButtonDrumSubTask4Callback() {
    if(ui->leftPushButton_doTask_drum_sub_task_4->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 8! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->leftGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->left_drum_sub_task_4_)) {
            ROS_LOG_WARN("DO CHEMICAL SUB TASK 8!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::leftPushButtonDrumSubTask5Callback() {
    if(ui->leftPushButton_doTask_drum_sub_task_5->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 8! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->leftGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->left_drum_sub_task_5_)) {
            ROS_LOG_WARN("DO CHEMICAL SUB TASK 8!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::leftPushButtonDrumSubTask6Callback() {
    if(ui->leftPushButton_doTask_drum_sub_task_6->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 8! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->leftGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->left_drum_sub_task_6_)) {
            ROS_LOG_WARN("DO CHEMICAL SUB TASK 8!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::leftPushButtonDrumSubTask7Callback() {
    if(ui->leftPushButton_doTask_drum_sub_task_7->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SUB TASK 8! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->leftGenDrumTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->left_drum_sub_task_7_)) {
            ROS_LOG_WARN("DO CHEMICAL SUB TASK 8!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDrumScanTask1Callback() {
    if(ui->pushButton_doTask_drum_scan_task_1->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SCAN TASK 1! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genScanTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->drum_scan_task_1_)) {
            ROS_LOG_WARN("DO CHEMICAL SCAN TASK 1!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDrumScanTask2Callback() {
    if(ui->pushButton_doTask_drum_scan_task_2->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SCAN TASK 2! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genScanTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->drum_scan_task_2_)) {
            ROS_LOG_WARN("DO CHEMICAL SCAN TASK 2!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDrumScanTask3Callback() {
    if(ui->pushButton_doTask_drum_scan_task_3->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SCAN TASK 3! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genScanTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->drum_scan_task_3_)) {
            ROS_LOG_WARN("DO CHEMICAL SCAN TASK 3!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDrumScanTask4Callback() {
    if(ui->pushButton_doTask_drum_scan_task_4->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SCAN TASK 4! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genScanTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->drum_scan_task_4_)) {
            ROS_LOG_WARN("DO CHEMICAL SCAN TASK 4!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDrumScanTask5Callback() {
    if(ui->pushButton_doTask_drum_scan_task_5->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SCAN TASK 5! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genScanTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->drum_scan_task_5_)) {
            ROS_LOG_WARN("DO CHEMICAL SCAN TASK 5!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDrumScanTask6Callback() {
    if(ui->pushButton_doTask_drum_scan_task_6->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SCAN TASK 6! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genScanTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->drum_scan_task_6_)) {
            ROS_LOG_WARN("DO CHEMICAL SCAN TASK 6!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDrumScanTask7Callback() {
    if(ui->pushButton_doTask_drum_scan_task_7->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SCAN TASK 7! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genScanTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->drum_scan_task_7_)) {
            ROS_LOG_WARN("DO CHEMICAL SCAN TASK 7!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDrumScanTask8Callback() {
    if(ui->pushButton_doTask_drum_scan_task_8->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SCAN TASK 8! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genScanTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->drum_scan_task_8_)) {
            ROS_LOG_WARN("DO CHEMICAL SCAN TASK 8!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDrumScanTask9Callback() {
    if(ui->pushButton_doTask_drum_scan_task_9->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] SCAN TASK 9! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genScanTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->drum_scan_task_9_)) {
            ROS_LOG_WARN("DO CHEMICAL SCAN TASK 9!");
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pauseTaskBtnCallback() {
    _taskWindow->getTaskManager()->_qnode->pauseTask();
}

void MainWindow_widgetsHanyangEng::resumeTaskBtnCallback() {
    _taskWindow->getTaskManager()->_qnode->resumeTask();
}


void MainWindow_widgetsHanyangEng::pushButtonDoKorasChemicalGripperInitializeCallback() {
    ROS_LOG_WARN("***** [CHEMICAL] Gripper Initialize! *****");
    //// Step 1) Generate Target Tasks
    ROS_LOG_WARN("TASK GENERATING...");
    hanyangEngPlanner_->genGripperTask();
    ROS_LOG_WARN("TASK GENERATED!");
    //// Step 2) Do Target Tasks
    if (doTargetTask(hanyangEngPlanner_->drum_grp_initialize_task_)) {
        ROS_LOG_WARN("DO GRIPPER INITIALIZE!");
    } else {
        QMessageBox::information(this, "Task Execution", "No Tasks Exist");
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoKorasChemicalGripperOpenCallback() {
    ROS_LOG_WARN("***** [CHEMICAL] GRIPPER OPEN FIRST CAP! *****");
    //// Step 1) Generate Target Tasks
    ROS_LOG_WARN("TASK GENERATING...");
    hanyangEngPlanner_->genGripperTask();
    ROS_LOG_WARN("TASK GENERATED!");
    //// Step 2) Do Target Tasks
    if (doTargetTask(hanyangEngPlanner_->drum_grp_lid_cap_open_)) {
        ROS_LOG_WARN("DO GRIPPER OPEN FIRST CAP!");
    } else {
        QMessageBox::information(this, "Task Execution", "No Tasks Exist");
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoKorasChemicalGripperCloseCallback() {
    ROS_LOG_WARN("***** [CHEMICAL] GRIPPER CLOSE FIRST CAP! *****");
    //// Step 1) Generate Target Tasks
    ROS_LOG_WARN("TASK GENERATING...");
    hanyangEngPlanner_->genGripperTask();
    ROS_LOG_WARN("TASK GENERATED!");
    //// Step 2) Do Target Tasks
    if (doTargetTask(hanyangEngPlanner_->drum_grp_lid_cap_close_)) {
        ROS_LOG_WARN("DO GRIPPER CLOSE FIRST CAP!");
    } else {
        QMessageBox::information(this, "Task Execution", "No Tasks Exist");
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoKorasChemicalGripperGraspingPoseCallback() {
    ROS_LOG_WARN("***** [CHEMICAL] GRIPPER GRASPING! *****");
    //// Step 1) Generate Target Tasks
    ROS_LOG_WARN("TASK GENERATING...");
    hanyangEngPlanner_->genGripperTask();
    ROS_LOG_WARN("TASK GENERATED!");
    //// Step 2) Do Target Tasks
    if (doTargetTask(hanyangEngPlanner_->drum_grp_grasping_pose_task_)) {
        ROS_LOG_WARN("DO GRIPPER GRASPING POSE!");
    } else {
        QMessageBox::information(this, "Task Execution", "No Tasks Exist");
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoKorasChemicalGripperScrewingPoseCallback() {
    ROS_LOG_WARN("***** [CHEMICAL] GRIPPER SCREWING! *****");
    //// Step 1) Generate Target Tasks
    ROS_LOG_WARN("TASK GENERATING...");
    hanyangEngPlanner_->genGripperTask();
    ROS_LOG_WARN("TASK GENERATED!");
    //// Step 2) Do Target Tasks
    if (doTargetTask(hanyangEngPlanner_->drum_grp_screwing_pose_task_)) {
        ROS_LOG_WARN("DO GRIPPER SCREWING!");
    } else {
        QMessageBox::information(this, "Task Execution", "No Tasks Exist");
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoKorasChemicalGripperUnscrewingPoseCallback() {
    ROS_LOG_WARN("***** [CHEMICAL] GRIPPER UNSCREWING! *****");
    //// Step 1) Generate Target Tasks
    ROS_LOG_WARN("TASK GENERATING...");
    hanyangEngPlanner_->genGripperTask();
    ROS_LOG_WARN("TASK GENERATED!");
    //// Step 2) Do Target Tasks
    if (doTargetTask(hanyangEngPlanner_->drum_grp_unscrewing_pose_task_)) {
        ROS_LOG_WARN("DO GRIPPER UNSCREWING!");
    } else {
        QMessageBox::information(this, "Task Execution", "No Tasks Exist");
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoKorasChemicalGripperExitPoseCallback() {
    ROS_LOG_WARN("***** [CHEMICAL] GRIPPER EXIT POSE! *****");
    //// Step 1) Generate Target Tasks
    ROS_LOG_WARN("TASK GENERATING...");
    hanyangEngPlanner_->genGripperTask();
    ROS_LOG_WARN("TASK GENERATED!");
    //// Step 2) Do Target Tasks
    if (doTargetTask(hanyangEngPlanner_->drum_grp_exit_pose_task_)) {
        ROS_LOG_WARN("DO GRIPPER EXIT POSE!");
    } else {
        QMessageBox::information(this, "Task Execution", "No Tasks Exist");
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoKorasChemicalGripperPlusMotorCtrlCallback() {
    ROS_LOG_WARN("***** [CHEMICAL] GRIPPER EXIT POSE! *****");
    //// Step 1) Generate Target Tasks
    ROS_LOG_WARN("TASK GENERATING...");
    hanyangEngPlanner_->genGripperTask();
    ROS_LOG_WARN("TASK GENERATED!");
    //// Step 2) Do Target Tasks
    if (doTargetTask(hanyangEngPlanner_->drum_grp_plus_motor_ctrl_task_)) {
        ROS_LOG_WARN("DO GRIPPER MINUS MOTOR CTRL!");
    } else {
        QMessageBox::information(this, "Task Execution", "No Tasks Exist");
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoKorasChemicalGripperMinusMotorCtrlCallback() {
    ROS_LOG_WARN("***** [CHEMICAL] GRIPPER EXIT POSE! *****");
    //// Step 1) Generate Target Tasks
    ROS_LOG_WARN("TASK GENERATING...");
    hanyangEngPlanner_->genGripperTask();
    ROS_LOG_WARN("TASK GENERATED!");
    //// Step 2) Do Target Tasks
    if (doTargetTask(hanyangEngPlanner_->drum_grp_minus_motor_ctrl_task_)) {
        ROS_LOG_WARN("DO GRIPPER PLUS MOTOR CTRL!");
    } else {
        QMessageBox::information(this, "Task Execution", "No Tasks Exist");
    }
}



void MainWindow_widgetsHanyangEng::pushButtonDoScanningZIVIDClickedCallback() {
#if BIN_PICKING_FLAG

#if DRFL_CONTROL
    /////////////////////////////////////////////////
    // _taskWindow->getTaskManager()->_qnode->getDrflCurrentPose(); //// 두산 로봇 CS pose 할당
    /////////////////////////////////////////////////
#endif

    taskScanningParameter scan_parameter;
    scan_parameter.target_id = ui->lineEdit_targetId->text().toInt();
    scan_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
    scan_parameter.is_mask_pixel_fixed = ui->radioButton_assemblyUI_isMaskFixed->isChecked();
    std::vector<int> mask_pixel_list(4,10); // to be deprecated
    mask_pixel_list[0] = 0;
    mask_pixel_list[1] = 0;
    mask_pixel_list[2] = 1920;
    mask_pixel_list[3] = 1200;
    scan_parameter.mask_pixel_list = mask_pixel_list;
    scan_parameter.sampling_num = ui->lineEdit_scanSamplingNum->text().toInt();
    scan_parameter.is_base_frame_unknown = false;
    scan_parameter.do_scan_sampling = ui->radioButton_assemblyUI_scanSampling->isChecked();
    scan_parameter.do_save_data = ui->radioButton_assemblyUI_scanDataSave->isChecked();
    scan_parameter.do_image_processing = ui->radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    scan_parameter.do_single_matching = true; // matching service
    scan_parameter.do_not_scan_do_load_data = ui->radioButton_assemblyUI_scanDataLoad->isChecked();
    scan_parameter.skip_detection_mask = ui->radioButton_assemblyUI_skipDetectionMask->isChecked();


    //// 241125 추가
    scan_parameter.robot_dh_vec = _taskWindow->getTaskManager()->_qnode->getRobotDHParameters(); // [m], [deg]
    scan_parameter.robot_tcp_default = _taskWindow->getTaskManager()->_qnode->getRobotDefaultTCP(); // [m], [deg]
    scan_parameter.robot_tcp = _taskWindow->getTaskManager()->_qnode->getRobotTCP(); // [m], [deg]



    arr2Vec(_taskWindow->getTaskManager()->_qnode->params_.meas.q , scan_parameter.scan_position); // scanning joint position
    std::vector<double> scan_position_q = scan_parameter.scan_position;
    ROS_LOG_WARN("Current JS position [deg]: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", scan_position_q[0], scan_position_q[1], scan_position_q[2], scan_position_q[3], scan_position_q[4], scan_position_q[5]);


    // // dlgDoScanning(scan_parameter); // signal to MainWindow_widgetsHanyangEng
    // if(scan_parameter.target_id >= 31 && scan_parameter.target_id < 45) {
    //     _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->scanZIVID(scan_parameter);
    // } else {
    //     ROS_LOG_INFO("Wrong target id!");
    // }
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->scanZIVID(scan_parameter);

#endif
}

void MainWindow_widgetsHanyangEng::pushButtonDoTemplateMatchingBinPickingClickedCallback() { // bin picking
#if BIN_PICKING_FLAG

#if DRFL_CONTROL
     /////////////////////////////////////////////////
    //  _taskWindow->getTaskManager()->_qnode->getDrflCurrentPose(); //// 두산 로봇 CS pose 할당
     /////////////////////////////////////////////////
#endif
    taskTemplateMatchingParameter matching_parameter;
    matching_parameter.debug_mode = true;
    matching_parameter.is_base_frame_unknown = false;
    matching_parameter.is_symmetric = ui->radioButton_isSymmetricCAD->isChecked();
    matching_parameter.do_scan_sampling = ui->radioButton_assemblyUI_scanSampling->isChecked();
    matching_parameter.sampling_num = ui->lineEdit_scanSamplingNum->text().toInt();
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->doCADMatchingService(matching_parameter);
#endif
}

void MainWindow_widgetsHanyangEng::pushButtonSelectTargetObjectClickedCallback() {
#if BIN_PICKING_FLAG

    // double voxel_downsampling_size = ui->task_doubleSpinBox_featureVoxelDownsamplingSize->value();

    size_t target_object_idx = 0;
    for(int i=0; i<bin_picking_target_object_list_.size(); i++) {
        if(bin_picking_target_object_list_[i]->isChecked()) { target_object_idx = i; }
    }

    ROS_LOG_WARN("[pushButtonSelectTargetObjectClickedCallback] target_object_idx #%zu", target_object_idx);


    // if(bin_picking_target_object_list_[10]->isChecked() || bin_picking_target_object_list_[11]->isChecked()) {
    //     _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->current_target_object_idx_ = target_object_idx - 10;
    //     _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->before_target_object_idx_ = target_object_idx - 10;
    // } else {
    //     _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->current_target_object_idx_ = target_object_idx;
    //     _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->before_target_object_idx_ = target_object_idx;
    // }
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->current_target_object_idx_ = target_object_idx;
    _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->before_target_object_idx_ = target_object_idx;

    CTARGET_OBJECT_DATA* ptr_object_now = _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[_taskWindow->getTaskManager()->_qnode->m_bin_picking_node->current_target_object_idx_];


    //// # of gripper driver: 2
    ptr_object_now->grp_initial_min_position.resize(2);
    ptr_object_now->grp_initial_max_position.resize(2);
    ptr_object_now->is_grp_set_min_value_finished.resize(2);
    ptr_object_now->is_grp_set_max_value_finished.resize(2);

    for(int i=0; i<2; i++)
    {
        ptr_object_now->grp_initial_min_position[i] = 0;
        ptr_object_now->grp_initial_max_position[i] = 1;
        ptr_object_now->is_grp_set_min_value_finished[i] = false;
        ptr_object_now->is_grp_set_max_value_finished[i] = false;
    }

    uint16_t gripper_close_length = 0;
    taskScanningParameter ui_cmd_parameter;

    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    ///// Default///////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////

    ui->radioButton_assemblyUI_isMaskFixed->setChecked(false);
    ui->task_spinBox_gripperPosValue->setValue(7500); // tip ver. 3
    gripper_close_length = 8250;

    _taskWindow->getTaskManager()->_qnode->grp_do_initialize_ = true;
    _taskWindow->getTaskManager()->_qnode->grp_is_vacuum_ = false;
    _taskWindow->getTaskManager()->_qnode->grp_is_vacuum_before_ = false;
    _taskWindow->getTaskManager()->_qnode->grp_init_2_grp_cmd_ = 4953; // for initialize_2

    //// Tip index setting
    ptr_object_now->m_grasping_parameter.gripper_tip_index = 1; // default index: 1
    ptr_object_now->m_grasping_parameter.is_tip_changing_applied = false;

    //// 파지 성공/실패 여부 확인을 위한 변수
    //// TODO: JSON 파일로 설정하기
    // Gripper #1
    ptr_object_now->grp_initial_min_position[0] = -4735; // [deg]
    ptr_object_now->grp_initial_max_position[0] = 44; // [deg]
    ptr_object_now->is_grp_set_min_value_finished[0] = true;
    ptr_object_now->is_grp_set_max_value_finished[0] = true;
    // Gripper #2
    ptr_object_now->grp_initial_min_position[1] = -4557; // [deg]
    ptr_object_now->grp_initial_max_position[1] = 187; // [deg]
    ptr_object_now->is_grp_set_min_value_finished[1] = true;
    ptr_object_now->is_grp_set_max_value_finished[1] = true;

#if DRFL_CONTROL

#else

    // ////
    // // Target mrcnn weight number
    // if(ui->radioButton_use_automatic_mrcnn_weight_1->isChecked()) ui->lineEdit_targetWeightNumber->setText("udr");
    // else if(ui->radioButton_use_automatic_mrcnn_weight_2->isChecked()) ui->lineEdit_targetWeightNumber->setText("udrv");
    // else ui->lineEdit_targetWeightNumber->setText("29");
    // //// Tool changing slave id
    // ptr_object_now->tool_changing_attach_id = 2; // # slave #1, 2, ...
    // ptr_object_now->tool_changing_detach_id = 2; // # slave #1, 2, ...
    // _taskWindow->getTaskManager()->_qnode->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
    // _taskWindow->getTaskManager()->_qnode->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

    // ui->task_spinBox_toolIndex->setValue(_taskWindow->getTaskManager()->_qnode->tool_changing_attach_id); // tool slot #1, 2, ...

    // //// Tip changing id
    // ptr_object_now->is_tip_changing_allowed_ = true;
    // ptr_object_now->tip_changing_attach_id = 2; // tip #1, 2, ...
    // ptr_object_now->tip_changing_detach_id = 2; // tip #1, 2, ...
    // _taskWindow->getTaskManager()->_qnode->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
    // _taskWindow->getTaskManager()->_qnode->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

    // ui->task_spinBox_tipIndex->setValue(_taskWindow->getTaskManager()->_qnode->tip_changing_attach_id); // tool slot #1, 2, ...
#endif
    //// Detaching count initialize
    //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
    ptr_object_now->detaching_cnt_ = 0;
    ptr_object_now->stacking_z_idx_ = 0;
    _taskWindow->getTaskManager()->_qnode->test_cnt_ = 0;

    std::vector<double> stacking_trans_scale = {0.225, 0.050, 0.045}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
    ptr_object_now->stacking_single_stack_num_ = 4; // 1개씩 놓는 진행 방향의 물체 개수
    ptr_object_now->stacking_line_stack_num_ = 1; // stacking_single_stack_num_만큼 놓는 선의 개수
    ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
    ptr_object_now->stacking_mode_ = StackingType::CYLINDER_STACKING;

    //// 1단의 실린더 개수
    ptr_object_now->stack_part_cnt_ = ((ptr_object_now->stacking_single_stack_num_ * (ptr_object_now->stacking_single_stack_num_ + 1)) / 2) * ptr_object_now->stacking_line_stack_num_;
    //// counting 최대 개수
    ptr_object_now->max_cnt_detaching_ = ptr_object_now->stack_part_cnt_; // 한 단의 실린더 스택, ex) 한 개의 단은 10개씩, 두 단은 총 20개
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////

    switch(target_object_idx)
    {
        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_CHEMICAL_COUPLER_HOLDER :
        {
            ROS_LOG_WARN("Target grasping object: RIGHT CHEMICAL_COUPLER_HOLDER!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_RIGHT_CHEMICAL_COUPLER_HOLDER));
            ui->lineEdit_targetName->setText("right_holder_with_coupler");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("6");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_HOLE_SURFACE :
        {
            ROS_LOG_WARN("Target grasping object: RIGHT DRUM HOLE SURFACE!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_RIGHT_DRUM_HOLE_SURFACE));
            ui->lineEdit_targetName->setText("right_drum_hole_surface");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_COUPLER_UNSCREWING :
        {
            ROS_LOG_WARN("Target grasping object: RIGHT DRUM_COUPLER_UNSCREWING!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_RIGHT_DRUM_COUPLER_UNSCREWING));
            ui->lineEdit_targetName->setText("right_drum_coupler_unscrewing");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_LID_CAP_UNSCREWING :
        {
            ROS_LOG_WARN("Target grasping object: RIGHT_DRUM_LID_CAP_UNSCREWING!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_RIGHT_DRUM_LID_CAP_UNSCREWING));
            ui->lineEdit_targetName->setText("right_drum_lid_cap_unscrewing");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_LID_CAP_HOLDER :
        {
            ROS_LOG_WARN("Target grasping object: RIGHT_DRUM_LID_HOLDER!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_RIGHT_DRUM_LID_CAP_HOLDER));
            ui->lineEdit_targetName->setText("right_holder_with_lid_cap");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_LID_CAP_HOLDER_EMPTY :
        {
            ROS_LOG_WARN("Target grasping object: RIGHT_DRUM_LID_CAP_HOLDER_EMPTY!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_RIGHT_DRUM_LID_CAP_HOLDER_EMPTY));
            ui->lineEdit_targetName->setText("right_holder_without_lid_cap");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_CHEMICAL_COUPLER_HOLDER_EMPTY :
        {
            ROS_LOG_WARN("Target grasping object: RIGHT_CHEMICAL_COUPLER_HOLDER_EMPTY!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_RIGHT_CHEMICAL_COUPLER_HOLDER_EMPTY));
            ui->lineEdit_targetName->setText("right_holder_without_coupler");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_LID_CAP_SCREWING :
        {
            ROS_LOG_WARN("Target grasping object: RIGHT_DRUM_LID_CAP_SCREWING!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_RIGHT_DRUM_LID_CAP_SCREWING));
            ui->lineEdit_targetName->setText("right_drum_lid_cap_screwing");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_DRUM_LID_TOTAL :
        {
            ROS_LOG_WARN("Target grasping object: DRUM_LID_TOTAL!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_DRUM_LID_TOTAL));
            ui->lineEdit_targetName->setText("drum_lid_total");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_KEY_CODE :
        {
            ROS_LOG_WARN("Target grasping object: RIGHT_DRUM_KEY_CODE!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_RIGHT_DRUM_KEY_CODE));
            ui->lineEdit_targetName->setText("right_drum_key_code");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_HOLDER_WITH_KEY_CODE :
        {
            ROS_LOG_WARN("Target grasping object: RIGHT_DRUM_HOLDER_WITH_KEY_CODE!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_RIGHT_DRUM_HOLDER_WITH_KEY_CODE));
            ui->lineEdit_targetName->setText("right_drum_holder_with_key_code");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1);
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true);
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_HOLDER_WITHOUT_KEY_CODE :
        {
            ROS_LOG_WARN("Target grasping object: RIGHT_DRUM_HOLDER_WITHOUT_KEY_CODE!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_RIGHT_DRUM_HOLDER_WITHOUT_KEY_CODE));
            ui->lineEdit_targetName->setText("right_drum_holder_without_key_code");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1);
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true);
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_KEY_CODE_UNSCREWING :
        {
            ROS_LOG_WARN("Target grasping object: RIGHT_DRUM_KEY_CODE_UNSCREWING!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_RIGHT_DRUM_KEY_CODE_UNSCREWING));
            ui->lineEdit_targetName->setText("right_drum_key_code_unscrewing");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1);
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true);
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;


        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_CHEMICAL_COUPLER_HOLDER :
        {
            ROS_LOG_WARN("Target grasping object: LEFT CHEMICAL_COUPLER_HOLDER!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_LEFT_CHEMICAL_COUPLER_HOLDER));
            ui->lineEdit_targetName->setText("left_holder_with_coupler");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("6");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_HOLE_SURFACE :
        {
            ROS_LOG_WARN("Target grasping object: LEFT DRUM HOLE SURFACE!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_LEFT_DRUM_HOLE_SURFACE));
            ui->lineEdit_targetName->setText("left_drum_hole_surface");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_COUPLER_UNSCREWING :
        {
            ROS_LOG_WARN("Target grasping object: LEFT DRUM_COUPLER_UNSCREWING!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_LEFT_DRUM_COUPLER_UNSCREWING));
            ui->lineEdit_targetName->setText("left_drum_coupler_unscrewing");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_LID_CAP_UNSCREWING :
        {
            ROS_LOG_WARN("Target grasping object: LEFT_DRUM_LID_CAP_UNSCREWING!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_LEFT_DRUM_LID_CAP_UNSCREWING));
            ui->lineEdit_targetName->setText("left_drum_lid_cap_unscrewing");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_LID_CAP_HOLDER :
        {
            ROS_LOG_WARN("Target grasping object: LEFT_DRUM_LID_HOLDER!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_LEFT_DRUM_LID_CAP_HOLDER));
            ui->lineEdit_targetName->setText("left_holder_with_lid_cap");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_LID_CAP_HOLDER_EMPTY :
        {
            ROS_LOG_WARN("Target grasping object: LEFT_DRUM_LID_CAP_HOLDER_EMPTY!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_LEFT_DRUM_LID_CAP_HOLDER_EMPTY));
            ui->lineEdit_targetName->setText("left_holder_without_lid_cap");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_CHEMICAL_COUPLER_HOLDER_EMPTY :
        {
            ROS_LOG_WARN("Target grasping object: LEFT_CHEMICAL_COUPLER_HOLDER_EMPTY!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_LEFT_CHEMICAL_COUPLER_HOLDER_EMPTY));
            ui->lineEdit_targetName->setText("left_holder_without_coupler");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_LID_CAP_SCREWING :
        {
            ROS_LOG_WARN("Target grasping object: LEFT_DRUM_LID_CAP_SCREWING!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_LEFT_DRUM_LID_CAP_SCREWING));
            ui->lineEdit_targetName->setText("left_drum_lid_cap_screwing");
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
#if !DRFL_CONTROL
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;
            ui->radioButton_assemblyUI_scanSampling->setChecked(true); // Scan sampling setting
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;


        default: {
            ROS_LOG_WARN("Target grasping object: Default!");
            ui->lineEdit_targetId->setText(QString::number(ON_PART_RIGHT_DRUM_HOLE_SURFACE));
            ui->lineEdit_targetName->setText("right_drum_hole_surface");
            //// load mrcnn weight
            ui_cmd_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = 39;
            // ui_cmd_parameter.is_sam_mean_size_assigned = false; // SAM paramter
            // ui_cmd_parameter.sam_mean_size = 0; // SAM paramter
            // ui_cmd_parameter.sam_mask_min_area = 6000;
            // ui_cmd_parameter.sam_mask_max_area = 300000;
            // _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            //// 드럼통 과제
#if DRFL_CONTROL

#else
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;
#endif
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;

            //// Scan sampling setting
            ui->radioButton_assemblyUI_scanSampling->setChecked(true);
            ui->lineEdit_scanSamplingNum->setText("8");
        }
        break;
        

    }
#if DRFL_CONTROL

#else
    // //// 초기 TCP는 tool master 기준
    if(!_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(7); // TCP #7 // Robot end-effector
    // // if(!_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(1); // TCP #1, 2지그리퍼
    // // if(!_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(8); // TCP #8 --> Robot Calibration
    // // if(!_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(9); // TCP #9 --> (0.0, 0.0, 0.0)
    // // if(!_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) changeRobotTCP(10); // TCP #10, Connector gripper
#endif

    // Set scanning parameters
    ptr_object_now->m_scan_parameter.target_id = ui->lineEdit_targetId->text().toInt();
    ptr_object_now->m_scan_parameter.target_name = ui->lineEdit_targetName->text().toStdString();

    // Removed forced remap to preserve original key_code at index 17

    ptr_object_now->m_scan_parameter.do_scan_sampling = ui->radioButton_assemblyUI_scanSampling->isChecked();
    ptr_object_now->m_scan_parameter.sampling_num = ui->lineEdit_scanSamplingNum->text().toInt();
    ptr_object_now->m_scan_parameter.is_mask_pixel_fixed = ui->radioButton_assemblyUI_isMaskFixed->isChecked();
    ptr_object_now->m_scan_parameter.do_save_data = false;
    ptr_object_now->m_scan_parameter.do_not_scan_do_load_data = false;
    ptr_object_now->m_scan_parameter.skip_detection_mask = false;

    ptr_object_now->m_scan_parameter.do_single_matching = false; // matching topic


    // SAM,
    // 추후 아래 파라미터는 삭제
    ptr_object_now->m_scan_parameter.weight_number = ui_cmd_parameter.weight_number;
    ptr_object_now->m_scan_parameter.is_sam_mean_size_assigned = ui_cmd_parameter.is_sam_mean_size_assigned;
    ptr_object_now->m_scan_parameter.sam_mean_size = ui_cmd_parameter.sam_mean_size;
    ptr_object_now->m_scan_parameter.sam_mask_min_area = ui_cmd_parameter.sam_mask_min_area;
    ptr_object_now->m_scan_parameter.sam_mask_max_area = ui_cmd_parameter.sam_mask_max_area;


    // Set matching parameters
    ptr_object_now->m_matching_parameter.debug_mode = true;
    ptr_object_now->m_matching_parameter.is_base_frame_unknown = false;
    ptr_object_now->m_matching_parameter.do_scan_sampling = ui->radioButton_assemblyUI_scanSampling->isChecked();
    ptr_object_now->m_matching_parameter.sampling_num = ui->lineEdit_scanSamplingNum->text().toInt();
    // Set grasping parameters
    ptr_object_now->m_grasping_parameter.gripper_open_length = ui->task_spinBox_gripperPosValue->value();
    ptr_object_now->m_grasping_parameter.gripper_close_length = gripper_close_length;

    std::vector<int> mask_pixel_list(4,10); // to be deprecated
    mask_pixel_list[0] = 0;
    mask_pixel_list[1] = 0;
    mask_pixel_list[2] = 1920;
    mask_pixel_list[3] = 1200;

    ptr_object_now->m_scan_parameter.mask_pixel_list = mask_pixel_list;

    ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////
    //// NOTICE: 240220, 실행 초기에만 실행되는 부분
    //// NOTICE: 240302, 파라미터 세팅으로만 설정하도록 항상 is_template_initialized_ = true
    //// Template JSON import
    taskTemplateMatchingParameter template_parameter;
    template_parameter.target_id = ui->lineEdit_targetId->text().toInt();
    template_parameter.target_name = ui->lineEdit_targetName->text().toStdString();
    template_parameter.robot_dh_vec = _taskWindow->getTaskManager()->_qnode->getRobotDHParameters();
    template_parameter.robot_tcp_default = _taskWindow->getTaskManager()->_qnode->getRobotDefaultTCP();
    template_parameter.robot_tcp = _taskWindow->getTaskManager()->_qnode->getRobotTCP();

    if(!ptr_object_now->is_template_initialized_) { // CAD importing
        ROS_LOG_WARN("***** Template_initialized *****");
        ptr_object_now->is_template_initialized_ = true;
        bool do_set_only_parameters = false;
        bool do_overwrite_JSON = true;
        // _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->doTemplateInitialize(template_parameter, do_set_only_parameters, do_overwrite_JSON);
    } else { // only set parameters for JSON
        bool do_set_only_parameters = true;
        bool do_overwrite_JSON = true;
        // _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->doTemplateInitialize(template_parameter, do_set_only_parameters, do_overwrite_JSON);
    }

    //// Drum 과제
    bool do_set_only_parameters = true;
    bool do_overwrite_JSON = true;
    if(target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_CHEMICAL_COUPLER_HOLDER
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_HOLE_SURFACE
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_COUPLER_UNSCREWING
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_LID_CAP_UNSCREWING
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_LID_CAP_HOLDER
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_LID_CAP_HOLDER_EMPTY
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_CHEMICAL_COUPLER_HOLDER_EMPTY
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_LID_CAP_SCREWING

            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_DRUM_LID_TOTAL
        
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_CHEMICAL_COUPLER_HOLDER
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_HOLE_SURFACE
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_COUPLER_UNSCREWING
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_LID_CAP_UNSCREWING
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_LID_CAP_HOLDER
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_LID_CAP_HOLDER_EMPTY
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_CHEMICAL_COUPLER_HOLDER_EMPTY
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_LEFT_DRUM_LID_CAP_SCREWING
            
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_HOLDER_WITH_KEY_CODE
            || target_object_idx == TargetObjectHanyangEng::OBJECT_HANYANG_ENG_RIGHT_DRUM_HOLDER_WITHOUT_KEY_CODE
        
        ) {
        _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter); // sam

        if(0) { // 250408, 사용하지 않도록 임시 처리--> 필요 없는 부분인지 확인 필요
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->doTemplateInitialize(template_parameter, do_set_only_parameters, do_overwrite_JSON); // matching_node
        }
    }
    ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////

#endif
}


void MainWindow_widgetsHanyangEng::pushButtonSetUIJSPosition11ClickedCallback() { // Move Home JS position
    std::vector<double> position_q;
    if (std::string(ROBOT_NAME) == "DS_M1013") {
        std::vector<double> home_q = {-0.000, -18.713, 105.394, -0.139, 93.361, -90.833};
        position_q = home_q;
    } else if (std::string(ROBOT_NAME) == "UR10e") {
        std::vector<double> home_q = {0.0, -69.226, -88.043, -112.723, 90.500, 0.0};
        position_q = home_q;
    }

    // Move
    double qd  = 10.0;
    double qdd = 20.0;
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    _taskWindow->getTaskManager()->_qnode->moveQ(q, qd, qdd, is_relative);
}

void MainWindow_widgetsHanyangEng::pushButtonSetUIJSPosition12ClickedCallback() {
    //// Drum hole surface
    std::vector<double> position_q;
    if (std::string(ROBOT_NAME) == "DS_M1013") {
        std::vector<double> home_q = {31.644, -11.474, 99.474, 33.409, 73.069, -153.435};
        position_q = home_q;
    } else if (std::string(ROBOT_NAME) == "UR10e") {
        std::vector<double> home_q = {0.014, -76.136, -75.924, -94.832, 89.954, 0.125};
        position_q = home_q;
    }

    // Move
    double qd  = 10.0;
    double qdd = 20.0;
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    _taskWindow->getTaskManager()->_qnode->moveQ(q, qd, qdd, is_relative);
}

void MainWindow_widgetsHanyangEng::pushButtonSetUIJSPosition13ClickedCallback() {
    //// Robot-Camera Calibration JS Position
    std::vector<double> position_q = {90.0, -16.934, 118.429, 0.085, 113.505, -90.0};
    // Move
    double qd  = 10.0;
    double qdd = 20.0;
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    _taskWindow->getTaskManager()->_qnode->moveQ(q, qd, qdd, is_relative);
}

void MainWindow_widgetsHanyangEng::pushButtonSetUIJSPosition14ClickedCallback() {
    //// Scan JS Position (Lid Cap Holder)
    std::vector<double> position_q = {70.094, -11.916, 114.557, -59.840, 29.435, -15.609};
    // Move
    double qd  = 10.0;
    double qdd = 20.0;
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    _taskWindow->getTaskManager()->_qnode->moveQ(q, qd, qdd, is_relative);
}

void MainWindow_widgetsHanyangEng::pushButtonSetUIJSPosition15ClickedCallback() {
    //// Scan JS Position (Chemical Coupler Holder)
    std::vector<double> position_q = {5.755, -8.142, 78.586, 32.294, 83.469, -206.805};
    // Move
    double qd  = 10.0;
    double qdd = 20.0;
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    _taskWindow->getTaskManager()->_qnode->moveQ(q, qd, qdd, is_relative);
}

void MainWindow_widgetsHanyangEng::pushButtonSetUIJSPosition16ClickedCallback() {
    //// Scan JS Position (Drum Coupler Unscrewing)
    std::vector<double> position_q = {9.969, -5.642, 88.461, 32.060, 70.802, -209.990};
    // Move
    double qd  = 10.0;
    double qdd = 20.0;
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    _taskWindow->getTaskManager()->_qnode->moveQ(q, qd, qdd, is_relative);
}

void MainWindow_widgetsHanyangEng::pushButtonSetUIJSPosition17ClickedCallback() {
    //// Scan JS Position (Drum Lid Total Scan, Rotation Angle Gen.)
    std::vector<double> position_q = {50.071, -10.607, 69.057, 16.046, 101.500, -123.910};
    // Move
    double qd  = 10.0;
    double qdd = 20.0;
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    _taskWindow->getTaskManager()->_qnode->moveQ(q, qd, qdd, is_relative);
}

void MainWindow_widgetsHanyangEng::pushButtonSetUIJSPosition18ClickedCallback() {
    //// Scan JS Position (Drum Lid Total Scan, Rotation Angle Gen.)
    std::vector<double> position_q = {-26.866, 8.048, -99.158, 142.843, 72.921, -19.598};
    // Move
    double qd  = 10.0;
    double qdd = 20.0;
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    _taskWindow->getTaskManager()->_qnode->moveQ(q, qd, qdd, is_relative);
}

void MainWindow_widgetsHanyangEng::pushButtonSetUIJSPosition19ClickedCallback() {
    //// Scan JS Position (Drum Lid Total Scan, Rotation Angle Gen.)
    std::vector<double> position_q = {-0.143, 18.822, -105.523, 179.931, 93.345, -89.115};
    // Move
    double qd  = 10.0;
    double qdd = 20.0;
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    _taskWindow->getTaskManager()->_qnode->moveQ(q, qd, qdd, is_relative);
}

void MainWindow_widgetsHanyangEng::pushButtonSetUIJSPosition20ClickedCallback() {
    //// Scan JS Position (Drum Lid Total Scan, Rotation Angle Gen.)
    std::vector<double> position_q = {-77.845, -0.167, -101.713, 60.562, -55.476, 9.974};
    // Move
    double qd  = 10.0;
    double qdd = 20.0;
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    _taskWindow->getTaskManager()->_qnode->moveQ(q, qd, qdd, is_relative);
}

void MainWindow_widgetsHanyangEng::pushButtonSetUIJSPosition21ClickedCallback() {
    //// Scan JS Position (Drum Lid Total Scan, Rotation Angle Gen.)
    std::vector<double> position_q = {-7.343, -17.846, -59.621, 137.056, 116.486, -58.113};
    // Move
    double qd  = 10.0;
    double qdd = 20.0;
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    _taskWindow->getTaskManager()->_qnode->moveQ(q, qd, qdd, is_relative);
}

void MainWindow_widgetsHanyangEng::pushButtonSetUIJSPosition22ClickedCallback() {
    //// Scan JS Position (Drum Lid Total Scan, Rotation Angle Gen.)
    std::vector<double> position_q = {-45.378, 21.125, -96.397, 159.281, 82.962, -46.578};
    // Move
    double qd  = 10.0;
    double qdd = 20.0;
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    _taskWindow->getTaskManager()->_qnode->moveQ(q, qd, qdd, is_relative);
}

void MainWindow_widgetsHanyangEng::pushButtonComplianceCallback() {
#if DRFL_CONTROL
    _taskWindow->getTaskManager()->_qnode->drflSetRobotMode(0);

    _taskWindow->getTaskManager()->_qnode->is_task_mode_ = false;
    bool success = _taskWindow->getTaskManager()->_qnode->drflTaskComplianceCtrl();
    if (success) {
        ROS_LOG_INFO("Compliance control enabled.");
    } else {
        ROS_LOG_ERROR("Failed to enable compliance control.");
    }

    _taskWindow->getTaskManager()->_qnode->drflSetRobotMode(1);

#endif
}

void MainWindow_widgetsHanyangEng::pushButtonComplianceOffCallback() {
#if DRFL_CONTROL
    _taskWindow->getTaskManager()->_qnode->drflSetRobotMode(0);

    _taskWindow->getTaskManager()->_qnode->is_task_mode_ = false;
    bool success = _taskWindow->getTaskManager()->_qnode->drflReleaseComplianceCtrl();
    if (success) {
        ROS_LOG_INFO("Compliance control enabled.");
    } else {
        ROS_LOG_ERROR("Failed to enable compliance control.");
    }

    _taskWindow->getTaskManager()->_qnode->drflSetRobotMode(1);
#endif
}

void MainWindow_widgetsHanyangEng::pushButtonSetToolWeight() {
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

void MainWindow_widgetsHanyangEng::pushButtonMakeToolWeight() {
#if DRFL_CONTROL
    _taskWindow->getTaskManager()->_qnode->drflSetRobotMode(0);

    _taskWindow->getTaskManager()->_qnode->is_task_mode_ = false;

    //// 8kg
    // bool success = _taskWindow->getTaskManager()->_qnode->drflConfigCreateTool(
    //     "t1",        // tool name
    //     8.00,            // tool weight in kg
    //     std::array<double, 3>{-1.00, 0.0177, 0.095},   // center of gravity (COG) in meters
    //     std::array<double, 6>{0.00, 0.00, 0.00, 0.00, 0.00, 0.00} // tool inertia (kg·m²) - 예시 값
    // );

    //// 5kg
    bool success = _taskWindow->getTaskManager()->_qnode->drflConfigCreateTool(
        "t1",        // tool name
        3.920,            // tool weight in kg
        std::array<double, 3>{0.015820, -0.035930, 0.093440},   // center of gravity (COG) in meters
        std::array<double, 6>{0.00, 0.00, 0.00, 0.00, 0.00, 0.00} // tool inertia (kg·m²) - 예시 값
    );

    if (success) {
        ROS_LOG_WARN("Tool Weight SET! /// 5KG tool(0, 0.030 , 0.132)");
    } else {
        ROS_LOG_ERROR("Failed to Set Tool Weight");
    }

    _taskWindow->getTaskManager()->_qnode->drflSetRobotMode(1);
    ROS_LOG_WARN("[%s]drflConfigCreateTool Completed!", __func__);
#endif
}

void MainWindow_widgetsHanyangEng::pushButtonAddTcpPresetClickedCallback() {
#if DRFL_CONTROL

    // 로봇 모드를 비활성화합니다.
    _taskWindow->getTaskManager()->_qnode->drflSetRobotMode(0);

    auto createTCP = [&] (std::string tcp_name, std::vector<double> tcp_vec) {

        CsDouble tcp;
        vec2Arr(tcp_vec, tcp);
        bool success = _taskWindow->getTaskManager()->_qnode->drflCreateTcp(tcp_name, tcp);
        if(success) {
            ROS_LOG_WARN("[%s] Successfully Registered", tcp_name.c_str());
            // 내부 TCP 목록에 저장 (UI 업데이트 없이)
            _taskWindow->getTaskManager()->_qnode->list_tcp[tcp_name] = tcp;
        } else {
            ROS_LOG_WARN("[%s] TCP Registration Failed!", tcp_name.c_str());
        }
    };

    // [mm], [deg]
    createTCP("tcp00", {0, 0, 0, 0, 0, 0}); // Flange
    createTCP("tcp01", {0, 40.0, 185.0, 0, 0, 90.0}); // legacy gripper
    createTCP("tcp02", {0, 0, 0, 0, 0, 90.0}); //
    // createTCP("tcp03", {0, 45.0, 197.5, 0, 0, 90.0}); // new gripper
    createTCP("tcp03", {0, 45.0, 199.0, 0, 0, 90.0}); // new gripper
    /// 동심 맞춘 그리퍼 버전
    createTCP("tcp04", {0, 0, 265.0, 0, 0, 90.0}); // star gripper

    createTCP("tcp05", {0.0, 0, 265.0, 0, 0, 90.0}); // star gripper (only rotation)
    createTCP("tcp06", {0, 30.0, 203.5, 0, 0, 90.0}); // chemical coupler grasping tcp
    // left cap
    createTCP("tcp07", {0.0, 0, 265.0, 0, 0, -90.0}); // chemical coupler grasping tcp
    createTCP("tcp08", {0, 47.5, 265.0, 0, 0, -90.0}); // tcp for KeyRing 



    // // Preset TCP (flange) 등록
    // std::string tcp_name0 = "tcp00";
    // CsDouble tcp0;
    // tcp0[0] = 0.0;  // X (mm)
    // tcp0[1] = 0.0;  // Y (mm)
    // tcp0[2] = 0.0;  // Z (mm)
    // tcp0[3] = 0.0;   // Rx (deg)
    // tcp0[4] = 0.0;   // Ry (deg)
    // tcp0[5] = 0.0;   // Rz (deg)
    // bool success0 = _taskWindow->getTaskManager()->_qnode->drflCreateTcp(tcp_name0, tcp0);
    // // 내부 TCP 목록에 저장 (UI 업데이트 없이)
    // _taskWindow->getTaskManager()->_qnode->list_tcp[tcp_name0] = tcp0;

    // // Preset TCP 1 등록
    // std::string tcp_name1 = "tcp01"; // 이전 그리퍼
    // CsDouble tcp1;
    // tcp1[0] = 0.0;  // X (mm)
    // tcp1[1] = 40.0;  // Y (mm)
    // tcp1[2] = 185.0;  // Z (mm)
    // tcp1[3] = 0.0;   // Rx (deg)
    // tcp1[4] = 0.0;   // Ry (deg)
    // tcp1[5] = 90.0;   // Rz (deg)
    // bool success1 = _taskWindow->getTaskManager()->_qnode->drflCreateTcp(tcp_name1, tcp1);
    // // 내부 TCP 목록에 저장 (UI 업데이트 없이)
    // _taskWindow->getTaskManager()->_qnode->list_tcp[tcp_name1] = tcp1;

    // // Preset TCP 2 등록
    // std::string tcp_name2 = "tcp02";
    // CsDouble tcp2;
    // tcp2[0] = 0.0;  // X (mm)
    // tcp2[1] = 0.0;  // Y (mm)
    // tcp2[2] = 0.0;  // Z (mm)
    // tcp2[3] = 0.0;   // Rx (deg)
    // tcp2[4] = 0.0;   // Ry (deg)
    // tcp2[5] = 90.0;   // Rz (deg)
    // bool success2 = _taskWindow->getTaskManager()->_qnode->drflCreateTcp(tcp_name2, tcp2);
    // _taskWindow->getTaskManager()->_qnode->list_tcp[tcp_name2] = tcp2;

    // // Preset TCP 3 등록 (new gripper)
    // std::string tcp_name3 = "tcp03";
    // CsDouble tcp3;
    // tcp3[0] = 0.0;  // X (mm)
    // tcp3[1] = 45.0;  // Y (mm)
    // tcp3[2] = 197.5;  // Z (mm)
    // tcp3[3] = 0.0;   // Rx (deg)
    // tcp3[4] = 0.0;   // Ry (deg)
    // tcp3[5] = 90.0;   // Rz (deg)
    // bool success3 = _taskWindow->getTaskManager()->_qnode->drflCreateTcp(tcp_name3, tcp3);
    // // 내부 TCP 목록에 저장 (UI 업데이트 없이)
    // _taskWindow->getTaskManager()->_qnode->list_tcp[tcp_name3] = tcp3;


    // 로봇 모드를 다시 활성화합니다.
    _taskWindow->getTaskManager()->_qnode->drflSetRobotMode(1);

    ROS_LOG_WARN("[%s]drflCreateTcp Completed!", __func__);
#endif
}

void MainWindow_widgetsHanyangEng::pushButtonGetTorqueClickedCallback() {
#if DRFL_CONTROL
    _taskWindow->getTaskManager()->_qnode->drflGetJointTorque();
#endif
}

void MainWindow_widgetsHanyangEng::pushButtonRobotOFFCallback() {
#if DRFL_CONTROL
    _taskWindow->getTaskManager()->_qnode->drflServoOff(1);
    ROS_LOG_WARN("ROBOT OFF!!", __func__);
#endif
}

void MainWindow_widgetsHanyangEng::pushButtonRobotONCallback() {
#if DRFL_CONTROL
    _taskWindow->getTaskManager()->_qnode->drflSetRobotControl(3);
    ROS_LOG_WARN("ROBOT ON!!", __func__);
#endif
}

void MainWindow_widgetsHanyangEng::uploadImage() {
    // QImage image = _taskWindow->getTaskManager()->_qnode->getCameraImage();
    // QPixmap pix;
    // // initialize raw image to qpixmap
    // pix = QPixmap::fromImage(image, Qt::AutoColor);
    // // show image on qlabel
    // ui->label_image_viewer->setPixmap(pix);
}

void MainWindow_widgetsHanyangEng::UpdateImage(const QImage &image, const QString &current_pass) {
    if (current_pass == "zivid" && !image.isNull()) {
        QPixmap scaledPixmap = QPixmap::fromImage(image).scaled(
            ui->label_image_viewer->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation
        );
        ui->label_image_viewer->setPixmap(scaledPixmap);
        
        ui->label_image_viewer_2->setPixmap(QPixmap::fromImage(image).scaled(
            ui->label_image_viewer_2->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation
        ));
    }
}

void MainWindow_widgetsHanyangEng::pushButtonSetPLCSimulationFlagClickedCallback() {


    _taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_ = ui->radioButton_PLCSimulationFlag->isChecked() ? true:false;

}
void MainWindow_widgetsHanyangEng::pushButtonSelectPlcWriteAddressClickedCallback() {

#if IS_PLC_LS_XG5000
    //// write
    if(ui->radioButton_select_plc_address_write_1->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x41000");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_write_2->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x41001");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("0.0");
    } else if(ui->radioButton_select_plc_address_write_3->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x41002");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_write_4->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x41003");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_write_5->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x41004");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_write_6->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x41005");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("0123456789ABCDEFGHIJ");
    }
#else
    //// write
    if(ui->radioButton_select_plc_address_write_1->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("1000");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_write_2->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("1001");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("0.0");
    } else if(ui->radioButton_select_plc_address_write_3->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("1002");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_write_4->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("1003");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_write_5->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("1004");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_write_6->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("1005");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("0123456789ABCDEFGHIJ");
    }
#endif

}

void MainWindow_widgetsHanyangEng::pushButtonSelectPlcSimulationTaskFlagClickedCallback() {

    if(!_taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_) {
        std::string log = "PLC SIMULATION MODE SHOULD BE TURNED ON!";
        sendLogMessageStr(log);
        return;
    }

    // for(int i = 0; i < plc_hanyang_task_flag_check_box_41000_.size(); ++i) {
    //     _taskWindow->getTaskManager()->_qnode->plc_status_word_41000_[i] = plc_hanyang_task_flag_check_box_41000_[i]->isChecked() ? true:false;
    // }
    // for(int i = 0; i < plc_hanyang_task_flag_check_box_41002_.size(); ++i) {
    //     _taskWindow->getTaskManager()->_qnode->plc_status_word_41002_[i] = plc_hanyang_task_flag_check_box_41002_[i]->isChecked() ? true:false;
    // }
    for(int i = 0; i < plc_hanyang_task_flag_check_box_42000_.size(); ++i) {
        _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[i] = plc_hanyang_task_flag_check_box_42000_[i]->isChecked() ? true:false;
    }
    for(int i = 0; i < plc_hanyang_task_flag_check_box_42002_.size(); ++i) {
        _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[i] = plc_hanyang_task_flag_check_box_42002_[i]->isChecked() ? true:false;
    }
}

void MainWindow_widgetsHanyangEng::pushButtonSelectPlcReadAddressClickedCallback() {

#if IS_PLC_LS_XG5000
    //// read
    if(ui->radioButton_select_plc_address_read_1->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x42000");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_read_2->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x42001");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_read_3->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x42002");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    }
#else
    //// read
    if(ui->radioButton_select_plc_address_read_1->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("2000");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_read_2->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("2001");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_read_3->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("2002");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    }
#endif
}

/// @brief PLC 관련 함수: Write
void MainWindow_widgetsHanyangEng::pushButtonModbusWriteDataBoolType() {
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
    if(ui->pushButton_MODBUS_WRITE_DATA_BOOL_TYPE->isChecked()) {
        _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(word_address, true, bit_address, true);
    } else {
        _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(word_address, true, bit_address, false);
    }
}

void MainWindow_widgetsHanyangEng::pushButtonModbusWriteDataInt16Type() {

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

    ////
    double angle = ui->lineEdit_MODBUS_DATA_TO_SEND->text().toDouble(&ok);
    if (!ok) {
        ROS_LOG_WARN("Error: Invalid TARGET ROTATION ANGLE! [%s]", __func__);
        return;
    }

    _taskWindow->getTaskManager()->_qnode->setPLCModbusRotationAngle(word_address, bit_address, angle);
}




void MainWindow_widgetsHanyangEng::pushButtonModbusWriteDataASCIIType() {

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

    ////
    std::string str_data = ui->lineEdit_MODBUS_DATA_TO_SEND->text().toStdString();
    if (!ok) {
        ROS_LOG_WARN("Error: Invalid TARGET ROTATION ANGLE! [%s]", __func__);
        return;
    }

    _taskWindow->getTaskManager()->_qnode->setPLCModbusBarcodeWrite(word_address, bit_address, str_data);
}



/// @brief PLC 관련 함수 : Read
/// @brief PLC 관련 함수 : Read
void MainWindow_widgetsHanyangEng::pushButtonModbusReadDataBoolType() {
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

    bool status_read = false;
    if (_taskWindow->getTaskManager()->_qnode->PLCModbusReadStatusFromAMR(word_address, bit_address, status_read)) {
        if (status_read) {
            ROS_LOG_WARN("[%s] MODBUS Read - Address: 0x%X, Bit: %u → Flag: TRUE", __func__, word_address, bit_address);
        } else {
            ROS_LOG_WARN("[%s] MODBUS Read - Address: 0x%X, Bit: %u → Flag: FALSE", __func__, word_address, bit_address);
        }
    } else {
        ROS_LOG_WARN("[%s] MODBUS Read failed - Address: 0x%X, Bit: %u", __func__, word_address, bit_address);
    }
}

void MainWindow_widgetsHanyangEng::pushButtonModbusMonitoringPLC()
{
    const bool enable =
        ui->pushButton_MODBUS_MonitoringPLC->isChecked() ||
        ui->pushButton_MODBUS_MonitoringPLC_2->isChecked();

    _taskWindow->getTaskManager()->_qnode->is_plc_task_mode_monitoring_ = true;

    if (true)         
    {
        ui->textEdit_log->clear();  // Clear Log

        /* PLC 시뮬레이션 모드라면 4100X 워드 초기화 */
        if (_taskWindow->getTaskManager()->_qnode->is_plc_simulation_mode_)
        {
            for (size_t i = 0;
                 i < plc_hanyang_task_flag_check_box_41000_.size();
                 ++i)
            {
                _taskWindow->getTaskManager()->_qnode->plc_status_word_41000_[i] = false;
                _taskWindow->getTaskManager()->_qnode->plc_status_word_41002_[i] = false;
            }
        }
    }
}



void MainWindow_widgetsHanyangEng::pushButtonModbusMonitoringPLCReadAMR2RobotStatus()
{
    if ( ui->pushButton_MODBUS_MonitoringPLCReadAMR2RobotStatus->isChecked() ||
         ui->pushButton_MODBUS_MonitoringPLCReadAMR2RobotStatus_2->isChecked() )
    {
        _taskWindow->getTaskManager()->_qnode->is_monitoring_plc_read_amr_to_robot_status_ = true;
    }
    else
    {
        _taskWindow->getTaskManager()->_qnode->is_monitoring_plc_read_amr_to_robot_status_ = false;
    }
}


void MainWindow_widgetsHanyangEng::pushButtonModbusInitializePLCWrite41000Signals() {

#if IS_PLC_LS_XG5000
    uint16_t word_address_in = static_cast<uint16_t>(0x41000);
#else
    uint16_t word_address_in = static_cast<uint16_t>(1000);
#endif

    _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_in, true, 0, false);
}

void MainWindow_widgetsHanyangEng::pushButtonModbusInitializePLCWrite41002Signals() {
#if IS_PLC_LS_XG5000
    uint16_t word_address_in = static_cast<uint16_t>(0x41002);
#else
    uint16_t word_address_in = static_cast<uint16_t>(1002);
#endif
    _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_in, true, 0, false);
}

void MainWindow_widgetsHanyangEng::pushButtonModbusInitializePLCWrite41003Signals() {
#if IS_PLC_LS_XG5000
    uint16_t word_address_in = static_cast<uint16_t>(0x41003);
#else
    uint16_t word_address_in = static_cast<uint16_t>(1003);
#endif
    _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_in, true, 0, false);
}


void MainWindow_widgetsHanyangEng::pushButtonModbusSendPLCWrite41000Bit14Signals() {
#if IS_PLC_LS_XG5000
    uint16_t word_address_in = static_cast<uint16_t>(0x41000);
#else
    uint16_t word_address_in = static_cast<uint16_t>(1000);
#endif
    uint16_t bit_address_in = static_cast<uint16_t>(14); // 로봇 작업 완료 신호
    bool status = true;
    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(word_address_in, true, bit_address_in, status);
}


void MainWindow_widgetsHanyangEng::pushButtonModbusTaskPLCFlagOnOff() {
    if(ui->pushButton_MODBUS_TaskPLCFlagOnOff->isChecked()) {
        _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = true;
    } else {
        _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoTasksAMRReadyPoseCallback() {
    if(ui->pushButton_doTask_plc_comm_amrReadyPose->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->amr_ready_pose_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}


void MainWindow_widgetsHanyangEng::leftPushButtonDoTasksDrumHomeposeCallback() {
    if(ui->pushButton_doTask_drum_homeposeLeft->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->left_drum_home_pose_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::rightPushButtonDoTasksDrumHomeposeCallback() {
    if(ui->pushButton_doTask_drum_homeposeRight->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_home_pose_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}


void MainWindow_widgetsHanyangEng::pushButtonDoTasksRotationAngleDetectionCallback() {
    if(ui->pushButton_doTask_plc_comm_rotationAngleDetection->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->drum_rotating_angle_detection_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoTasksBarcodeDetectionCallback() {
    if(ui->pushButton_doTask_plc_comm_barcodeDetection->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->drum_barcode_read_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}


void MainWindow_widgetsHanyangEng::pushButtonDoTasksCapDetachAndCouplerScrewingLeftCallback() {
    if(ui->pushButton_doTask_plc_comm_capDetachAndCouplerScrewingLeft->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->left_drum_cap_detach_coupler_screwing_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoTasksCapDetachLeftCallback() {
    if(ui->pushButton_doTask_plc_comm_capDetachLeft->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] PLC COMM. TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->left_drum_cap_detach_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoTasksCapDetachRightCallback() {
    if(ui->pushButton_doTask_plc_comm_capDetachRight->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] PLC COMM. TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_cap_detach_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoTasksCouplerScrewingLeftCallback() {
    if(ui->pushButton_doTask_plc_comm_couplerScrewingLeft->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] PLC COMM. TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->left_drum_coupler_screwing_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoTasksCouplerScrewingRightCallback() {
    if(ui->pushButton_doTask_plc_comm_couplerScrewingRight->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] PLC COMM. TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_coupler_screwing_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}


void MainWindow_widgetsHanyangEng::pushButtonDoTasksCouplerDetachLeftCallback() {
    if(ui->pushButton_doTask_plc_comm_couplerDetachLeft->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] PLC COMM. TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->left_drum_coupler_detach_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoTasksCouplerDetachRightCallback() {
    if(ui->pushButton_doTask_plc_comm_couplerDetachRight->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] PLC COMM. TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_coupler_detach_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}


void MainWindow_widgetsHanyangEng::pushButtonDoTasksCapScrewingLeftCallback() {
    if(ui->pushButton_doTask_plc_comm_capScrewingLeft->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] PLC COMM. TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->left_drum_cap_screwing_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoTasksCapScrewingRightCallback() {
    if(ui->pushButton_doTask_plc_comm_capScrewingRight->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] PLC COMM. TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_cap_screwing_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}


void MainWindow_widgetsHanyangEng::pushButtonDoTasksCapDetachAndCouplerScrewingRightCallback() {
    if(ui->pushButton_doTask_plc_comm_capDetachAndCouplerScrewingRight->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_cap_detach_coupler_screwing_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}


void MainWindow_widgetsHanyangEng::pushButtonDoTasksCouplerDetachAndCapScrewingLeftCallback() {
    if(ui->pushButton_doTask_plc_comm_couplerDetachAndCapScrewingLeft->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->left_drum_coupler_detach_cap_screwing_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::pushButtonDoTasksCouplerDetachAndCapScrewingRightCallback() {
    if(ui->pushButton_doTask_plc_comm_couplerDetachAndCapScrewingRight->isChecked()) {
        ROS_LOG_WARN("***** [CHEMICAL] TOTAL TASK! *****");
        //// Step 1) Generate Target Tasks
        ROS_LOG_WARN("TASK GENERATING...");
        hanyangEngPlanner_->genPLCCommTask();
        ROS_LOG_WARN("TASK GENERATED!");
        //// Step 2) Do Target Tasks
        if (doTargetTask(hanyangEngPlanner_->right_drum_coupler_detach_cap_screwing_plc_comm_task_)) {
            ROS_LOG_WARN("%s", __func__);
        } else {
            QMessageBox::information(this, "Task Execution", "No Tasks Exist");
        }
    } else {
        pushButtonSTOPAllCallback();
    }
}

void MainWindow_widgetsHanyangEng::changeRobotTCP(unsigned int tcp_idx) {
#if BIN_PICKING_FLAG
    _taskWindow->getTaskManager()->_qnode->sendChangeRobotTCPCommand(tcp_idx);
#endif
}
    

//// PLC Read 2000~2002 (AMR to Robot)
void MainWindow_widgetsHanyangEng::monitorPLCStatus() {
#if IS_PLC_LS_XG5000
        uint16_t word_address_42000 = static_cast<uint16_t>(0x42000);
        uint16_t word_address_42001 = static_cast<uint16_t>(0x42001);
        uint16_t word_address_42002 = static_cast<uint16_t>(0x42002);
#else
        uint16_t word_address_42000 = static_cast<uint16_t>(2000);
        uint16_t word_address_42001 = static_cast<uint16_t>(2001);
        uint16_t word_address_42002 = static_cast<uint16_t>(2002);
#endif

    ROS_LOG_WARN("[monitorPLCStatus]...");

    if(1) { // 2000

        uint16_t current_value_42000 = 0;
        _taskWindow->getTaskManager()->_qnode->PLCModbusReadStatusFromAMR2(word_address_42000, current_value_42000);
        ////
        for(int i = 0; i < 16; ++i) {
            // 특정 비트 추출
            uint16_t bit_address = static_cast<uint16_t>(i);
            bool bit_status = (current_value_42000 >> bit_address) & 0x01;
            // D 레지스터 번호 변환
#if IS_PLC_LS_XG5000
            uint16_t d_register_number = word_address_42000 - 0x40000;
#else
            uint16_t d_register_number = word_address_42000;
#endif
            // flag update
            if (_taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[i] != bit_status) {
                _taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[i] = bit_status;
                // ROS_LOG_WARN("[%s] Word address: (0x4%X), Register: 0x%X (D%05u), Bit address: %u, Value: %u [Modified]",
                //                 __func__, word_address_42000, word_address_42000, d_register_number, bit_address, bit_status);
            } else {
                // ROS_LOG_INFO("[%s] Word address: (0x4%X), Register: 0x%X (D%05u), Bit address: %u, Value: %u",
                //                 __func__, word_address_42000, word_address_42000, d_register_number, bit_address, bit_status);
            }
        }
    }

//     if(0) { // 2001
//         uint16_t current_value_42001 = 0;
//         _taskWindow->getTaskManager()->_qnode->PLCModbusReadStatusFromAMR2(word_address_42001, current_value_42001);
//         for(int i = 0; i < 16; ++i) { // 0~13
//             // 특정 비트 추출
//             uint16_t bit_address = static_cast<uint16_t>(i);
//             bool bit_status = (current_value_42001 >> bit_address) & 0x01;
//             // D 레지스터 번호 변환
// #if IS_PLC_LS_XG5000
//             uint16_t d_register_number = word_address_42001 - 0x40000;
// #else
//             uint16_t d_register_number = word_address_42001;
// #endif
//             // flag update
//             if (_taskWindow->getTaskManager()->_qnode->plc_status_word_42001_[i] != bit_status) {
//                 _taskWindow->getTaskManager()->_qnode->plc_status_word_42001_[i] = bit_status;
//                 // ROS_LOG_WARN("[%s] Word address: (0x4%X), Register: 0x%X (D%05u), Bit address: %u, Value: %u [Modified]",
//                 //                 __func__, word_address_42001, word_address_42001, d_register_number, bit_address, bit_status);
//             } else {
//                 // ROS_LOG_INFO("[%s] Word address: (0x4%X), Register: 0x%X (D%05u), Bit address: %u, Value: %u",
//                 //                 __func__, word_address_42001, word_address_42001, d_register_number, bit_address, bit_status);
//             }
//         }
//     }
    ////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////
    if(1) { // 2002
        uint16_t current_value_42002 = 0;
        _taskWindow->getTaskManager()->_qnode->PLCModbusReadStatusFromAMR2(word_address_42002, current_value_42002);
        for(int i = 0; i < 16; ++i) { // 0~13
            // 특정 비트 추출
            uint16_t bit_address = static_cast<uint16_t>(i);
            bool bit_status = (current_value_42002 >> bit_address) & 0x01;
            // D 레지스터 번호 변환
#if IS_PLC_LS_XG5000
            uint16_t d_register_number = word_address_42002 - 0x40000;
#else
            uint16_t d_register_number = word_address_42002;
#endif
            // flag update
            if (_taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[i] != bit_status) {
                _taskWindow->getTaskManager()->_qnode->plc_status_word_42002_[i] = bit_status;
                // ROS_LOG_WARN("[%s] Word address: (0x4%X), Register: 0x%X (D%05u), Bit address: %u, Value: %u [Modified]",
                //                 __func__, word_address_42002, word_address_42002, d_register_number, bit_address, bit_status);
            } else {
                // ROS_LOG_INFO("[%s] Word address: (0x4%X), Register: 0x%X (D%05u), Bit address: %u, Value: %u",
                //                 __func__, word_address_42002, word_address_42002, d_register_number, bit_address, bit_status);
            }
        }
    }
    ////////////////////////////////////////////////////////////////////////
}



void MainWindow_widgetsHanyangEng::checkPLCDoosanRobotCollision() {

#if IS_PLC_LS_XG5000
    uint16_t word_address_41000 = static_cast<uint16_t>(0x41000);
    uint16_t word_address_41002 = static_cast<uint16_t>(0x41002);
    uint16_t word_address_41003 = static_cast<uint16_t>(0x41003);
#else
    uint16_t word_address_41000 = static_cast<uint16_t>(1000);
    uint16_t word_address_41002 = static_cast<uint16_t>(1002);
    uint16_t word_address_41003 = static_cast<uint16_t>(1003);
#endif

    if (!_taskWindow->getTaskManager()->_qnode->is_doosan_robot_collision_) { 
        // for (uint16_t i = 0; i <= 15; ++i) {
        //     _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1003), true, i, false);
        // }
        // ROS_LOG_WARN("Not Collision ----");
        // ROS_LOG_WARN("Not Collision ----");
        // ROS_LOG_WARN("Not Collision ----");

        _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_41003, true, 0, false);


    } else { // Collision
        pushButtonSTOPAllCallback();
        pushButtonSTOPAllCallback();



            //수정중. amr 통신 테스트 필요. 안될 경우 우선 748~755줄 사용. 250730 경태
        {

            if(1) {

                //Port1 (왼쪽) "커플러 풀기" 중 충돌 알람
                if(_taskWindow->getTaskManager()->_qnode->current_task_log_ == "right_task_detach_coupler") {
                    _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
                    // _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_41003, true, 0, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 0, true);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 1, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 2, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 3, false);

                //Port1 (왼쪽) "캡 체결" 중 충돌 알람
                } else if(_taskWindow->getTaskManager()->_qnode->current_task_log_ == "right_task_attach_lid_cap") {
                    _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
                    
                    // _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_41003, true, 0, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 0, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 1, true);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 2, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 3, false);


                //Port1 (왼쪽) "캡 풀기" 중 충돌 알람
                } else if(_taskWindow->getTaskManager()->_qnode->current_task_log_ == "right_task_detach_lid_cap") {
                    _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
                    
                    // _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_41003, true, 0, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 0, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 1, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 2, true);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 3, false);


                //Port1 (왼쪽) "커플러 체결" 중 충돌 알람
                } else if(_taskWindow->getTaskManager()->_qnode->current_task_log_ == "right_task_attach_coupler") {
                    _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
                    
                    // _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_41003, true, 0, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 0, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 1, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 2, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 3, true);
                }

                /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //Port1 (왼쪽) "커플러 풀기" 중 충돌 알람
                if(_taskWindow->getTaskManager()->_qnode->current_task_log_ == "left_task_detach_coupler") {
                    _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
                    
                    // _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_41003, true, 0, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 4, true);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 5, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 6, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 7, false);

                //Port1 (왼쪽) "캡 체결" 중 충돌 알람
                } else if(_taskWindow->getTaskManager()->_qnode->current_task_log_ == "left_task_attach_lid_cap") {
                    _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
                    
                    // _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_41003, true, 0, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 4, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 5, true);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 6, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 7, false);


                //Port1 (왼쪽) "캡 풀기" 중 충돌 알람
                } else if(_taskWindow->getTaskManager()->_qnode->current_task_log_ == "left_task_detach_lid_cap") {
                    _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
                    
                    // _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_41003, true, 0, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 4, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 5, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 6, true);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 7, false);


                //Port1 (왼쪽) "커플러 체결" 중 충돌 알람
                } else if(_taskWindow->getTaskManager()->_qnode->current_task_log_ == "left_task_attach_coupler") {
                    _taskWindow->getTaskManager()->_qnode->is_plc_task_in_progress_ = false;
                    
                    // _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_41003, true, 0, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 4, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 5, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 6, false);
                    _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 7, true);
                }
    
    
            } else {

                _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 8, true);

            }

        }


        // if (_taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[3]== true && _taskWindow->getTaskManager()->_qnode->is_doosan_robot_collision_) {

        // ROS_LOG_WARN("RIGHT -- It's Collision ----33");
        // ROS_LOG_WARN("RIGHT -- It's Collision ----33");

        //     //Port2 (오른쪽) "커플러 풀기" 중 충돌 알람
        //     if(_taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, 4, true) && 
        //     _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, 6, false)) 
        //     {
        //         ROS_LOG_WARN("RIGHT -- It's Collision ----44");
        //         ROS_LOG_WARN("RIGHT -- It's Collision ----44");
        //         // _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_41003, true, 0, false);
        //         _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 4, true);
        //     }

        //     //Port2 (오른쪽) "캡 체결" 중 충돌 알람
        //     else if(_taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, 4, true) && 
        //     _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, 6, true)) 
        //     {
        //         ROS_LOG_WARN("RIGHT -- It's Collision ----55");
        //         ROS_LOG_WARN("RIGHT -- It's Collision ----55");
        //         // _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_41003, true, 0, false);
        //         _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 5, true);
        //     }

        //     //Port2 (오른쪽) "캡 풀기" 중 충돌 알람
        //     else if(_taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, 8, true) && 
        //     _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, 10, false)) 
        //     {
        //         ROS_LOG_WARN("RIGHT -- It's Collision ----66");
        //         ROS_LOG_WARN("RIGHT -- It's Collision ----66");
        //         // _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_41003, true, 0, false);
        //         _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 6, true);
        //     }

        //     //Port2 (오른쪽) "커플러 체결" 중 충돌 알람
        //     else if(_taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, 8, true) && 
        //     _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, 10, true)) 
        //     {
        //         ROS_LOG_WARN("RIGHT -- It's Collision ----77");
        //         ROS_LOG_WARN("RIGHT -- It's Collision ----77");
        //         // _taskWindow->getTaskManager()->_qnode->PLCModbusInitializeStatusFromRobotToAMR(word_address_41003, true, 0, false);
        //         _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 7, true);
        //     }

        // ROS_LOG_WARN("RIGHT -- It's Collision ----8888");
        // ROS_LOG_WARN("RIGHT -- It's Collision ----8888");
        // }
    }

    
    // if (_taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[2]== true &&
    //     _taskWindow->getTaskManager()->_qnode->is_doosan_robot_collision_) {

            
    //         //Port1 (왼쪽) "캡 분리" 중 충돌 알람
    //         if(_taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, 6, true)) {
    //             _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 0, true);
    //             ROS_LOG_WARN("1111");
    //         }
            
    //         //Port1 (왼쪽) "캡 체결" 중 충돌 알람
    //         else if(_taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, 4, true)) {
    //             _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 1, true);
    //             ROS_LOG_WARN("2222");

    //         }

    //         //Port1 (왼쪽) "커플러 분리" 중 충돌 알람
    //         else if(_taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, 2, true)) {
    //             _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 2, true);
    //             ROS_LOG_WARN("3333");

    //         }

    //         //Port1 (왼쪽) "커플러 체결" 중 충돌 알람
    //         else if(_taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, 8, true)) {
    //             _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 3, true);
    //             ROS_LOG_WARN("4444");

    //         }
    //     }


    // if (_taskWindow->getTaskManager()->_qnode->plc_status_word_42000_[3]== true &&
    //     _taskWindow->getTaskManager()->_qnode->is_doosan_robot_collision_) {

            
    //         //Port2 (오른쪽) "캡 분리" 중 충돌 알람
    //         if(_taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, 6, true)) {
    //             _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 4, true);
    //             ROS_LOG_WARN("5555");
    //         }
            
    //         //Port2 (오른쪽) "캡 체결" 중 충돌 알람
    //         else if(_taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, 4, true)) {
    //             _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 5, true);
    //             ROS_LOG_WARN("6666");

    //         }

    //         //Port2 (오른쪽) "커플러 분리" 중 충돌 알람
    //         else if(_taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, 2, true)) {
    //             _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 6, true);
    //             ROS_LOG_WARN("7777");


    //         }

    //         //Port2 (오른쪽) "커플러 체결" 중 충돌 알람
    //         else if(_taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(1000), true, 8, true)) {
    //             _taskWindow->getTaskManager()->_qnode->PLCModbusWriteStatusFromRobotToAMR(static_cast<uint16_t>(word_address_41003), true, 7, true);
    //             ROS_LOG_WARN("8888");
    //         }
    //     }
}
