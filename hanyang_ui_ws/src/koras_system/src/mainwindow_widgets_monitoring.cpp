#include "mainwindow_widgets_monitoring.hpp"
#include "ui_mainwindow_widgets_monitoring.h"
#include <QKeyEvent>
#include <QApplication>
MainWindow_widgetsMonitoring::MainWindow_widgetsMonitoring(MainWindow_node * taskWindow, QWidget* parent)
    : _taskWindow(taskWindow)
    , QMainWindow(parent)
    , ui(new Ui::MainWindow_widgetsMonitoring)
    , isRecording(false)
{
    ui->setupUi(this);
    ////////////////////////////// definition groupboxes, layouts


    ROS_LOG_WARN("THIS 1!");
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

    connect(ui->pushButton_stop_all, &QPushButton::clicked, this, [this] {
        _taskWindow->getTaskManager()->_qnode->is_llm_task_fin = true;
        _taskWindow->getTaskManager()->_qnode->is_task_mode_ = false;
        _taskWindow->getTaskManager()->_qnode->stopRobot();
    });

    connect(ui->pushButton_KorasGripperInitialize, &QPushButton::clicked, this, [this] {
        uint16_t position = 0;
        _taskWindow->getTaskManager()->_qnode->setGrp((uint16_t)KR_GRP::INIT, position, _taskWindow->getTaskManager()->_qnode->grp_driver_address_);
    });

    connect(ui->pushButton_KorasGripperOpen, &QPushButton::clicked, this, [this] {
        uint16_t position = 0;
        _taskWindow->getTaskManager()->_qnode->setGrp((uint16_t)KR_GRP::OPEN, position, _taskWindow->getTaskManager()->_qnode->grp_driver_address_);
    });

    connect(ui->pushButton_KorasGripperClose, &QPushButton::clicked, this, [this] {
        uint16_t position = 10000;
        _taskWindow->getTaskManager()->_qnode->setGrp((uint16_t)KR_GRP::CLOSE, position, _taskWindow->getTaskManager()->_qnode->grp_driver_address_);
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


    qRegisterMetaType<LogInfo>("LogInfo");
    connect(_taskWindow->getTaskManager()->_qnode, &QNode::sendLogMessage, this, [this](LogInfo logInfo) {
        if (ui->textEdit_log->textCursor().position() > 50) {
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


    // QObject::connect(bin_matching_dialog, &BinMatchingDialog::setCustomTrasnformationPose , this, &BinPickingDialog::setCustomTrasnformationPose);

    QObject::connect(_taskWindow->getTaskManager()->_qnode, &QNode::sendLogMessageStr , this, &MainWindow_widgetsMonitoring::sendLogMessageStr);

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
    ROS_LOG_WARN("THIS 3!");

    //// Button
    connect(ui->pushButton_exit_program, &QPushButton::clicked, this,
            &MainWindow_widgetsMonitoring::testBtnCallback);


    status_update_timer_ = new QTimer(this);
    connect(status_update_timer_, &QTimer::timeout, this, &MainWindow_widgetsMonitoring::statusTimerCallback);
    status_update_timer_->start(200);

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

    /////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// LLM Initialize ///////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////

    grp_home_pose_ = {-61.332, -61.761, -95.944, -22.196, 151.207, 0.011};
    home_pose_ = {-80.641, -54.269, -129.186, -87.611, 89.384, -82.897};
    mid_pose_ = {-177.303, -87.894, -107.568, -74.470, 89.329, -84.873};
    scan_pose_ = {-80.641, -54.269, -129.186, -87.611, 89.384, -82.897};
    tool_change_vel = ui->lineEdit_velocity_1->text().toInt();
    grasping_vel = ui-> lineEdit_velocity_2->text().toInt();
    passing_vel = ui-> lineEdit_velocity_3->text().toInt();
    loading_vel = ui-> lineEdit_velocity_4->text().toInt();

    target_object_list  << ui->radioButton_striker
                        << ui->radioButton_scu
                        << ui->radioButton_grill
                        << ui->radioButton_hinge
                        << ui->radioButton_bolt;

    waypoint_list << ui->lineEdit_A_x << ui->lineEdit_A_y << ui->lineEdit_A_z
                  << ui->lineEdit_B_x << ui->lineEdit_B_y << ui->lineEdit_B_z
                  << ui->lineEdit_C_x << ui->lineEdit_C_y << ui->lineEdit_C_z;

    pass_list << ui->lineEdit_pass_1 << ui->lineEdit_pass_2 << ui->lineEdit_pass_3;

    connect(ui->pushButton_recording, &QPushButton::pressed, this, &MainWindow_widgetsMonitoring::RecordingStartCallback);
    connect(ui->pushButton_recording, &QPushButton::released, this, &MainWindow_widgetsMonitoring::RecordingStopCallback);
    QObject::connect(QApplication::instance(), SIGNAL(focusChanged(QWidget*, QWidget*)), this, SLOT(onFocusChanged(QWidget*, QWidget*)));

	// connect(ui->pushButton_recording, &QPushButton::clicked, this,
    //         &MainWindow_widgetsMonitoring::RecordingCallback);

    connect(ui->pushButton_home, &QPushButton::clicked, this,
            std::bind(&MainWindow_widgetsMonitoring::SetPoseCallback, this, "home"));
    connect(ui->pushButton_gr_home, &QPushButton::clicked, this,
            std::bind(&MainWindow_widgetsMonitoring::SetPoseCallback, this, "grp"));
    connect(ui->pushButton_mid, &QPushButton::clicked, this,
            std::bind(&MainWindow_widgetsMonitoring::SetPoseCallback, this, "mid"));
    connect(ui->pushButton_scaning_pose, &QPushButton::clicked, this,
            std::bind(&MainWindow_widgetsMonitoring::SetPoseCallback, this, "scan"));

    connect(ui->radioButton_striker, &QRadioButton::clicked, this,
            &MainWindow_widgetsMonitoring::target_obj_Callback);
    connect(ui->radioButton_scu, &QRadioButton::clicked, this,
            &MainWindow_widgetsMonitoring::target_obj_Callback);
    connect(ui->radioButton_grill, &QRadioButton::clicked, this,
            &MainWindow_widgetsMonitoring::target_obj_Callback);
    connect(ui->radioButton_hinge, &QRadioButton::clicked, this,
            &MainWindow_widgetsMonitoring::target_obj_Callback);
    connect(ui->radioButton_bolt, &QRadioButton::clicked, this,
            &MainWindow_widgetsMonitoring::target_obj_Callback);

	connect(ui->pushButton_test1_in, &QPushButton::clicked, this,
            &MainWindow_widgetsMonitoring::GripperAttachCallback);
    connect(ui->pushButton_test1_out, &QPushButton::clicked, this,
            &MainWindow_widgetsMonitoring::GripperDetachCallback);
    connect(ui->pushButton_test2_in, &QPushButton::clicked, this,
            &MainWindow_widgetsMonitoring::GripperAttachCallback);
    connect(ui->pushButton_test2_out, &QPushButton::clicked, this,
             &MainWindow_widgetsMonitoring::GripperDetachCallback);
    connect(ui->pushButton_test3_in, &QPushButton::clicked, this,
            &MainWindow_widgetsMonitoring::GripperAttachCallback);
    connect(ui->pushButton_test3_out, &QPushButton::clicked, this,
             &MainWindow_widgetsMonitoring::GripperDetachCallback);
    connect(ui->pushButton_tip1_in, &QPushButton::clicked, this,
            &MainWindow_widgetsMonitoring::GripperAttachCallback);
    connect(ui->pushButton_tip1_out, &QPushButton::clicked, this,
            &MainWindow_widgetsMonitoring::GripperDetachCallback);
    connect(ui->pushButton_tip2_in, &QPushButton::clicked, this,
            &MainWindow_widgetsMonitoring::GripperAttachCallback);
    connect(ui->pushButton_tip2_out, &QPushButton::clicked, this,
            &MainWindow_widgetsMonitoring::GripperDetachCallback);

    // connect(ui->pushButton_pal, &QPushButton::clicked, this,
    //          &MainWindow_widgetsMonitoring::BoxSelectionCallback);
    connect(ui->pushButton_blue, &QPushButton::clicked, this,
             &MainWindow_widgetsMonitoring::BoxSelectionCallback);
    connect(ui->pushButton_yellow, &QPushButton::clicked, this,
             &MainWindow_widgetsMonitoring::BoxSelectionCallback);

    connect(_taskWindow->getTaskManager()->_qnode, &QNode::UpdateLLMText, this, &MainWindow_widgetsMonitoring::UpdateLLMText);
    connect(_taskWindow->getTaskManager()->_qnode, &QNode::UpdateVelocites, this, &MainWindow_widgetsMonitoring::UpdateVelocites);
    connect(_taskWindow->getTaskManager()->_qnode, &QNode::UpdateImage, this, &MainWindow_widgetsMonitoring::UpdateImage);
    connect(_taskWindow->getTaskManager()->_qnode, &QNode::UpdateWaypoints, this, &MainWindow_widgetsMonitoring::UpdateWaypoints);
    connect(_taskWindow->getTaskManager()->_qnode, &QNode::UpdateGoalPose, this, &MainWindow_widgetsMonitoring::UpdateGoalPose);
    connect(_taskWindow->getTaskManager()->_qnode, &QNode::LLMNodeReady, this, &MainWindow_widgetsMonitoring::LLMNodeReadyCallback);


    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow_widgetsMonitoring::timerCallback);
    timer_->start(100);
}



MainWindow_widgetsMonitoring::~MainWindow_widgetsMonitoring()
{
    delete ui;
    rclcpp::shutdown();
}

void MainWindow_widgetsMonitoring::closeEvent(QCloseEvent *bar)
{
    bar->ignore();
    return;
}

void MainWindow_widgetsMonitoring::sendLogMessageStr(std::string &str) {
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

void MainWindow_widgetsMonitoring::timerCallback() {
    if (is_placement_task && _taskWindow->getTaskManager()->_qnode->task_param_.is_robot_move_fin)
    {
        string goal_info = _taskWindow->getTaskManager()->_qnode->params_.llm_param.goal;
        if((goal_info == "yellow" || goal_info == "blue") && _taskWindow->getTaskManager()->_qnode->is_box_pose_received_)
        {
            is_placement_task = false;
            GraspingTargetObject();
            _taskWindow->getTaskManager()->_qnode->is_box_pose_received_ = false;
        }
        else if(!(goal_info == "yellow" || goal_info == "blue"))
        {
            is_placement_task = false;
            GraspingTargetObject();
            _taskWindow->getTaskManager()->_qnode->is_box_pose_received_ = false;
        }
    }
    else if (_taskWindow->getTaskManager()->_qnode->params_.llm_param.llm_text_received)
    {
        llm_node_Callback(_taskWindow->getTaskManager()->_qnode->params_.llm_param);
        _taskWindow->getTaskManager()->_qnode->params_.llm_param.llm_text_received = false;

    }
    if (_taskWindow->getTaskManager()->_qnode->is_llm_task_fin){
        ui->pushButton_recording->setText("Recording");
        // ui->pushButton_recording->setEnabled(true);
        ui->pushButton_recording->setFocus();
    }
    else{
        ui->pushButton_recording->setText("Robot is Working");
        // ui->pushButton_recording->setEnabled(false);
    }
}

void MainWindow_widgetsMonitoring::statusTimerCallback() {

    if (_taskWindow->getTaskManager()->_qnode->params_.status.is_enable) {
        ui->lineEdit_status_enable->setText("Enabled");
    } else {
        ui->lineEdit_status_enable->setText("Disabled");
    }

    if (_taskWindow->getTaskManager()->_qnode->is_task_mode_) {
        ui->lineEdit_is_task_in_progress->setText("Task in progress...");
        ui->pushButton_recording->setEnabled(false);
    } else {
        ui->lineEdit_is_task_in_progress->setText("Ready to start recording.");
        ui->pushButton_recording->setEnabled(true);
    }
}

void MainWindow_widgetsMonitoring::testBtnCallback() {

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

void MainWindow_widgetsMonitoring::keyPressEvent(QKeyEvent *event) {
    // if (event->key() == Qt::Key_Space && !isRecording && _taskWindow->getTaskManager()->_qnode->is_llm_task_fin) {
    //     isRecording = true;
    //     ui->pushButton_recording->setDown(true);
    //     // RecordingStartCallback();
    // }
}

void MainWindow_widgetsMonitoring::keyReleaseEvent(QKeyEvent *event) {
    // if (event->key() == Qt::Key_Space && isRecording && _taskWindow->getTaskManager()->_qnode->is_llm_task_fin) {
    //     isRecording = false;
    //     ui->pushButton_recording->setDown(false);
    //     // RecordingStopCallback();
    // }
}

void MainWindow_widgetsMonitoring::RecordingStartCallback() {
    ui->lineEdit_recording->setText("recording start");
    _taskWindow->getTaskManager()->_qnode->LLMStartRecording();
}

void MainWindow_widgetsMonitoring::RecordingStopCallback() {
    ui->lineEdit_recording->setText("recording end.");
    _taskWindow->getTaskManager()->_qnode->LLMStopRecording();
}

void MainWindow_widgetsMonitoring::LLMNodeReadyCallback() {
    ui->lineEdit_recording->setText("recording ready..");
}

// 포커스가 변경될 때마다 호출되는 슬롯
void MainWindow_widgetsMonitoring::onFocusChanged(QWidget* old, QWidget* now) {
    if (now != ui->pushButton_recording && _taskWindow->getTaskManager()->_qnode->is_llm_task_fin) {
        ui->pushButton_recording->setFocus();  // 포커스를 녹음 버튼으로 이동
    }
    else if(!(_taskWindow->getTaskManager()->_qnode->is_llm_task_fin)){
        ui->lineEdit_recording->setFocus();
    }
}

// UI pushbutton 입력으로 저장된 Pose로 이동
void MainWindow_widgetsMonitoring::SetPoseCallback(const std::string& position) {
    double qd = 30;
    double qdd = 60;
    if      (position == "home"){_taskWindow->getTaskManager()->_qnode->moveQ(home_pose_, qd, qdd, false);}
    else if (position == "grp") {_taskWindow->getTaskManager()->_qnode->moveQ(grp_home_pose_, qd, qdd, false);}
    else if (position == "mid") {_taskWindow->getTaskManager()->_qnode->moveQ(mid_pose_, qd, qdd, false);}
    else if (position == "scan"){_taskWindow->getTaskManager()->_qnode->moveQ(scan_pose_, qd, qdd, false);}
}

// LLM node 입력에 따라 UI radio button
void MainWindow_widgetsMonitoring::llm_node_Callback(LLM_PARAM &llm_param) {
    try {
        if (llm_param.task == "placement") {

            if (llm_param.target == "striker") {
                cout << "LLM node input: grasping striker" << endl;
                ui->radioButton_striker->setChecked(true);
            }
            else if (llm_param.target == "scu") {
                cout << "LLM node input: grasping SCU" << endl;
            ui->radioButton_scu->setChecked(true);
            }
            else if (llm_param.target == "grill") {
                cout << "LLM node input: grasping grill" << endl;
                ui->radioButton_grill->setChecked(true);
            }
            else if (llm_param.target == "hinge") {
                cout << "LLM node input: grasping hinge" << endl;
                ui->radioButton_hinge->setChecked(true);
            }
            else if (llm_param.target == "bolt") {
                cout << "LLM node input: grasping bolt" << endl;
                ui->radioButton_bolt->setChecked(true);
            }
            is_placement_task = true;
            target_obj_Callback();
        }
        else if (llm_param.task == "velocity") {
            tool_change_vel = ui->lineEdit_velocity_1->text().toInt();
            grasping_vel = ui-> lineEdit_velocity_2->text().toInt();
            passing_vel = ui-> lineEdit_velocity_3->text().toInt();
            loading_vel = ui-> lineEdit_velocity_4->text().toInt();
        }
        else if (llm_param.task == "move") {
            std::vector<double> pose{0, 0, 0, 0, 0, 0};
            double xd  = 0.1;
            double xdd = 0.2;
            pose[stoi(llm_param.target)] = stod(llm_param.goal);
            CsDouble x;
            vec2Arr(pose, x);
            _taskWindow->getTaskManager()->_qnode->moveX(x, xd, xdd, false, true);
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in llm_node_Callback: " << e.what() << std::endl;
    }
}

// UI Radio button 상태에 따른 object prameter setting 및 gripper setting
void MainWindow_widgetsMonitoring::target_obj_Callback() {
    int new_gripper = 1;
    int new_tip = 1;

    // 현재 선택된 객체에 따라 새로운 객체를 설정
    if (ui->radioButton_striker->isChecked()) {
        target_name = "striker";
        current_obj = 39;
        new_gripper = 1; // 2지 그리퍼
        new_tip = 1;
    } else if (ui->radioButton_scu->isChecked()) {
        target_name = "scu";
        current_obj = 40;
        new_gripper = 2; // 흡착 그리퍼
    } else if (ui->radioButton_grill->isChecked()) {
        target_name = "grill";
        current_obj = 41;
        new_gripper = 1; // 2지 그리퍼
        new_tip = 1;
    } else if (ui->radioButton_bolt->isChecked()) {
        target_name = "bolt";
        current_obj = 37;
        new_gripper = 1; // 2지 그리퍼
        new_tip = 2;    // bolt tip
    } else if (ui->radioButton_hinge->isChecked()) {
        target_name = "hinge";
        current_obj = 38;
        new_gripper = 1; // 2지 그리퍼
        new_tip = 1;
    } else {
        std::cerr << "Unknown target object." << std::endl;
        return;
    }
    std::cout << "Target object: " << target_name << ", new gripper: " << new_gripper << std::endl;

    _taskWindow->getTaskManager()->_qnode->current_task_list_.clear();
    _taskWindow->getTaskManager()->_qnode->task_planner_->taskPushBack_llm_start(_taskWindow->getTaskManager()->_qnode->current_task_list_);
    // 현재 장착된 그리퍼와 새로운 그리퍼가 다른 경우 그리퍼를 변경
    if (current_gripper != new_gripper) {

        std::cout << "Detaching current gripper: " << current_gripper << std::endl;
        _taskWindow->getTaskManager()->_qnode->task_planner_->LLMDetachGripper(current_gripper, tool_change_vel);
        _taskWindow->getTaskManager()->_qnode->current_task_list_.insert(_taskWindow->getTaskManager()->_qnode->current_task_list_.end(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_detach_.begin(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_detach_.end());

        std::cout << "Attaching new gripper: " << new_gripper << std::endl;
        _taskWindow->getTaskManager()->_qnode->task_planner_->LLMAttachGripper(new_gripper, tool_change_vel);
        _taskWindow->getTaskManager()->_qnode->current_task_list_.insert(_taskWindow->getTaskManager()->_qnode->current_task_list_.end(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_attach_.begin(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_attach_.end());

        std::cout << "Moving to gripper home position." << std::endl;
        _taskWindow->getTaskManager()->_qnode->task_planner_->taskPushBack_jsMove(_taskWindow->getTaskManager()->_qnode->current_task_list_, grp_home_pose_, false);
        current_gripper = new_gripper;
    }
    // 현재 장착된 팁과 새로운 팁이 다른 경우 팁을 변경
    if (current_tip != new_tip && new_gripper == 1) {
        std::cout << "Detaching current tip: " << current_tip << std::endl;
        _taskWindow->getTaskManager()->_qnode->task_planner_->LLMDetachTip(current_tip, tool_change_vel);
        _taskWindow->getTaskManager()->_qnode->current_task_list_.insert(
            _taskWindow->getTaskManager()->_qnode->current_task_list_.end(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_detach_tip_.begin(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_detach_tip_.end());

        std::cout << "Attaching current tip: " << current_tip << std::endl;
        _taskWindow->getTaskManager()->_qnode->task_planner_->LLMAttachTip(new_tip, tool_change_vel);
        _taskWindow->getTaskManager()->_qnode->current_task_list_.insert(
            _taskWindow->getTaskManager()->_qnode->current_task_list_.end(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_attach_tip_.begin(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_attach_tip_.end());

        std::cout << "Moving to gripper home position." << std::endl;
        _taskWindow->getTaskManager()->_qnode->task_planner_->taskPushBack_jsMove(_taskWindow->getTaskManager()->_qnode->current_task_list_, grp_home_pose_, false);
        current_tip = new_tip;
    }
    if (current_gripper != new_gripper) {
        _taskWindow->getTaskManager()->_qnode->task_planner_->taskPushBack_irl_grp(_taskWindow->getTaskManager()->_qnode->current_task_list_, KR_GRP::INIT, 0, 1);
    }
    // 홈 위치로 이동 (세팅 후 기본 위치로 돌아가기 위해)
    _taskWindow->getTaskManager()->_qnode->task_planner_->taskPushBack_jsMove(_taskWindow->getTaskManager()->_qnode->current_task_list_, home_pose_, false);
    _taskWindow->getTaskManager()->_qnode->beforeTaskStart();
}

// object parameter setting 후 kr_sys process 진행
void MainWindow_widgetsMonitoring::GraspingTargetObject() {
    TargetObjectParamSetting(); // scan/matching 위한 object parameter setting
    vector<vector<double>> waypoints;
    _taskWindow->getTaskManager()->_qnode->current_task_list_.clear();

    ////////////////////////////1. Matchig&Grasping////////////////////////////////

    _taskWindow->getTaskManager()->_qnode->task_planner_->LLMScanMatchingGrasping(current_obj, grasping_vel);
    _taskWindow->getTaskManager()->_qnode->current_task_list_.insert(_taskWindow->getTaskManager()->_qnode->current_task_list_.end(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_zivid_.begin(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_zivid_.end());

    ////////////////////////////2. passing waypoint////////////////////////////////
    printf("debug taek\n");

    for (int i = 0; i < 3; ++i) {
        string current_pass = pass_list[i]->text().toStdString();
        int idx_ = (current_pass == "A") ? 0 : (current_pass == "B") ? 3 : (current_pass == "C") ? 6 : 9;
        if (idx_ < 9){
            double waypoint_x = waypoint_list[idx_] -> text().toDouble();
            double waypoint_y = waypoint_list[idx_+1] -> text().toDouble();
            double waypoint_z = waypoint_list[idx_+2] -> text().toDouble();

            vector<double> waypoint = {waypoint_x, waypoint_y, waypoint_z};
            double vecsum = accumulate(waypoint.begin(), waypoint.end(), double(0));
            if (vecsum != 0){
                printf("Using stored data for pass point %s: x=%f, y=%f, z=%f \n", current_pass.c_str(), waypoint[0], waypoint[1], waypoint[2]);
                waypoints.push_back(waypoint);
            }
            else{
                printf("No stored data for pass point: %s \n", current_pass.c_str());
            }
        }
    }

    if (waypoints.size() != 0){
        _taskWindow->getTaskManager()->_qnode->task_planner_->LLMMoveWayPoints(waypoints, passing_vel);
        _taskWindow->getTaskManager()->_qnode->current_task_list_.insert(_taskWindow->getTaskManager()->_qnode->current_task_list_.end(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_waypoints_.begin(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_waypoints_.end());
    }

    ////////////////////////////3. loading color box////////////////////////////////
    int boxNumber = 2; // Yellow box
    int palette_num = 5; // palette center
    int boxStack = 0;
    string placement_goal = _taskWindow->getTaskManager()->_qnode->params_.llm_param.goal;

    if(placement_goal == "blue"){
        cout << "LLM node input: move to blue box" << endl;
        boxNumber = 1;
        blue_stack += 1;
        if (blue_stack > 2) {blue_stack = 1;}
        ui->lineEdit_blue_stack->setText(QString::number(blue_stack));
        boxStack = blue_stack;
    }
    else if(placement_goal == "yellow"){
        cout << "LLM node input: move to yellow box" << endl;
        boxNumber = 2;
        yellow_stack += 1;
        if (yellow_stack > 2) {yellow_stack = 1;}
        ui->lineEdit_yellow_stack->setText(QString::number(yellow_stack));
        boxStack = yellow_stack;
    }
    else if(placement_goal.substr(0,7) == "palette") {
        boxNumber = 3;
        palette_num = stoi(placement_goal.substr(8));
        cout << "LLM node input: move to palette_" << palette_num << endl;
    }
    else if(target_name == "bolt") {
        cout << "LLM node input: move to bolt aligner" << endl;
        boxNumber = 4;
    }
    else{
        cout << "LLM node input: unknown goal location" << endl;
    }
    _taskWindow->getTaskManager()->_qnode->task_planner_->LLMMoveBox(boxNumber, boxStack, current_obj, loading_vel, palette_num);
    _taskWindow->getTaskManager()->_qnode->current_task_list_.insert(_taskWindow->getTaskManager()->_qnode->current_task_list_.end(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_box_.begin(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_box_.end());
    _taskWindow->getTaskManager()->_qnode->task_planner_->taskPushBack_llm_fin(_taskWindow->getTaskManager()->_qnode->current_task_list_);
    _taskWindow->getTaskManager()->_qnode->beforeTaskStart();
}

void MainWindow_widgetsMonitoring::GripperAttachCallback() {
    QPushButton* button = qobject_cast<QPushButton*>(sender());
    int gripper_type = 0; // 기본값
    int tip_type = 0;

    if (button == ui->pushButton_test1_in) {
        gripper_type = 1;
        tip_type = 0;
    } else if (button == ui->pushButton_test2_in) {
        gripper_type = 2;
        tip_type = 0;
    } else if (button == ui->pushButton_test3_in) {
        gripper_type = 3;
        tip_type = 0;
    } else if (button == ui->pushButton_tip1_in) {
        gripper_type = 0;
        tip_type = 1;
    } else if (button == ui->pushButton_tip2_in) {
        gripper_type = 0;
        tip_type = 2;
    } else {
        std::cerr << "Unknown button pressed." << std::endl;
    }
    _taskWindow->getTaskManager()->_qnode->task_planner_->LLMAttachGripper(gripper_type, tool_change_vel);
    _taskWindow->getTaskManager()->_qnode->task_planner_->LLMAttachTip(tip_type, tool_change_vel);
    _taskWindow->getTaskManager()->_qnode->current_task_list_ = _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_attach_;
    _taskWindow->getTaskManager()->_qnode->current_task_list_.insert(_taskWindow->getTaskManager()->_qnode->current_task_list_.end(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_attach_tip_.begin(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_attach_tip_.end());

    _taskWindow->getTaskManager()->_qnode->beforeTaskStart();
}

void MainWindow_widgetsMonitoring::GripperDetachCallback() {
    QPushButton* button = qobject_cast<QPushButton*>(sender());
    int gripper_type = 0; // 기본값
    int tip_type = 0;

    if (button == ui->pushButton_test1_out) {
        gripper_type = 1;
        tip_type = 0;
    } else if (button == ui->pushButton_test2_out) {
        gripper_type = 2;
        tip_type = 0;
    } else if (button == ui->pushButton_test3_out) {
        gripper_type = 3;
        tip_type = 0;
    } else if (button == ui->pushButton_tip1_out) {
        gripper_type = 0;
        tip_type = 1;
    } else if (button == ui->pushButton_tip2_out) {
        gripper_type = 0;
        tip_type = 2;
    } else {
        std::cerr << "Unknown button pressed." << std::endl;
    }

    _taskWindow->getTaskManager()->_qnode->task_planner_->LLMDetachTip(tip_type, tool_change_vel);
    _taskWindow->getTaskManager()->_qnode->task_planner_->LLMDetachGripper(gripper_type, tool_change_vel);
    _taskWindow->getTaskManager()->_qnode->current_task_list_ = _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_detach_;
    _taskWindow->getTaskManager()->_qnode->current_task_list_.insert(_taskWindow->getTaskManager()->_qnode->current_task_list_.end(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_detach_tip_.begin(), _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_detach_tip_.end());

    _taskWindow->getTaskManager()->_qnode->beforeTaskStart();
}

void MainWindow_widgetsMonitoring::BoxSelectionCallback() {
    QPushButton* button = qobject_cast<QPushButton*>(sender());
    int boxNumber = 1; // 기본값
    int palette_num = 5;

    if (button == ui->pushButton_blue) {
        boxNumber = 1;
    } else if (button == ui->pushButton_yellow) {
        boxNumber = 2;
    // } else if (button == ui->pushButton_pal) {
    //     boxNumber = 3;
    } else {
        std::cerr << "Unknown button pressed. Defaulting to red box." << std::endl;
    }

    _taskWindow->getTaskManager()->_qnode->task_planner_->LLMMoveBox(boxNumber, 1, current_obj, loading_vel, palette_num);
    _taskWindow->getTaskManager()->_qnode->current_task_list_ = _taskWindow->getTaskManager()->_qnode->task_planner_->llm_task_box_;
    _taskWindow->getTaskManager()->_qnode->beforeTaskStart();
}

void MainWindow_widgetsMonitoring::UpdateImage(const QImage &image, const QString &current_pass) {
    if (current_pass == "A") {
        ui->label_pass_A->setPixmap(QPixmap::fromImage(image).scaled(ui->label_pass_A->size(), Qt::KeepAspectRatio));
    } else if (current_pass == "B") {
        ui->label_pass_B->setPixmap(QPixmap::fromImage(image).scaled(ui->label_pass_B->size(), Qt::KeepAspectRatio));
    } else if (current_pass == "C") {
        ui->label_pass_C->setPixmap(QPixmap::fromImage(image).scaled(ui->label_pass_C->size(), Qt::KeepAspectRatio));
    }
}

void MainWindow_widgetsMonitoring::UpdateWaypoints(const QString &coordinates,  const QString &current_pass) {
    QStringList coordList = coordinates.split(",");
    if (coordList.size() == 3) {
        int idx_ = (current_pass == "A") ? 0 : (current_pass == "B") ? 3 : (current_pass == "C") ? 6 : 9;
        waypoint_list[idx_] -> setText(QString::number(coordList[0].split(":")[1].trimmed().toFloat()));
        waypoint_list[idx_+1] -> setText(QString::number(coordList[1].split(":")[1].trimmed().toFloat()));
        waypoint_list[idx_+2] -> setText(QString::number(coordList[2].split(":")[1].trimmed().toFloat()));
    }
}

void MainWindow_widgetsMonitoring::UpdateLLMText(const QString &target, const QString &goal, const QString &passes) {
    ui->lineEdit_target->setText(target);
    ui->lineEdit_goal->setText(goal);
    // ui->lineEdit_passPoint->setText(passes);
    if (passes.size()>0){
        QStringList qpasslist = passes.split(" ");
        for (int i = 0; i < qpasslist.size()-1; ++i){
            pass_list[i]->setText(qpasslist[i]);
        }
    } else {
        for (int i = 0; i < 3; ++i){
            pass_list[i]->setText(" ");
        }
    }
}

void MainWindow_widgetsMonitoring::UpdateVelocites(const QString &velocities) {
    QStringList velocityList = velocities.split(" ");
    if (velocityList.size() >= 4) {
        ui->lineEdit_velocity_1->setText(velocityList[0]);
        ui->lineEdit_velocity_2->setText(velocityList[1]);
        ui->lineEdit_velocity_3->setText(velocityList[2]);
        ui->lineEdit_velocity_4->setText(velocityList[3]);
    } else {
        ui->lineEdit_velocity_1->setText("3");
        ui->lineEdit_velocity_2->setText("3");
        ui->lineEdit_velocity_3->setText("3");
        ui->lineEdit_velocity_4->setText("3");
    }
}

void MainWindow_widgetsMonitoring::UpdateGoalPose(const geometry_msgs::msg::Pose2D &goal_pose) {
    _taskWindow->getTaskManager()->_qnode->task_planner_->UpdateBoxPose(
        goal_pose.x, goal_pose.y, goal_pose.theta);
}

// Bin picking parameter setting each object
void MainWindow_widgetsMonitoring::TargetObjectParamSetting() {
    double voxel_downsampling_size = 5;

    size_t target_object_idx = 0;
    for(int i=0; i<target_object_list.size(); i++) {
        if(target_object_list[i]->isChecked()) { target_object_idx = i; }
    }

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
    switch(target_object_idx)
    {
        case TargetObject::OBJECT_STRIKER :
        {
            ROS_LOG_WARN("Target grasping object: Striker!");
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

            //// load mrcnn weight
            ui_cmd_parameter.target_name = target_name;
            // ui_cmd_parameter.weight_number = to_string(current_obj);
            // ui_cmd_parameter.is_sam_mean_size_assigned = false; // SAM paramter
            // ui_cmd_parameter.sam_mean_size = 0; // SAM paramter
            // ui_cmd_parameter.sam_mask_min_area = 6000;
            // ui_cmd_parameter.sam_mask_max_area = 300000;
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            //// [2F 그리퍼] - before
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) _taskWindow->getTaskManager()->_qnode->sendChangeRobotTCPCommand(13); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 13;

            // //// [2F 그리퍼] - new short version
            // if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) _taskWindow->getTaskManager()->_qnode->sendChangeRobotTCPCommand(18); // TCP #18(tip changing - round tip)
            // ptr_object_now->tcp_changing_id = 18;


            //// Tool changing slave id
            ptr_object_now->tool_changing_attach_id = 2; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 2; // # slave #1, 2, ...

            _taskWindow->getTaskManager()->_qnode->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            _taskWindow->getTaskManager()->_qnode->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = true;
            ptr_object_now->tip_changing_attach_id = 2; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 2; // tip #1, 2, ...
            _taskWindow->getTaskManager()->_qnode->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            _taskWindow->getTaskManager()->_qnode->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            //// [흡착 그리퍼]
            // if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) _taskWindow->getTaskManager()->_qnode->sendChangeRobotTCPCommand(16); // TCP #11, square peg vacuum
            // //// Tool changing slave id
            // _taskWindow->getTaskManager()->_qnode->tool_changing_attach_id = 4; // # slave #1, 2, ...
            // _taskWindow->getTaskManager()->_qnode->tool_changing_detach_id = 4; // # slave #1, 2, ...


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

            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;

            //// Scanning sampling number
            ptr_object_now->m_scan_parameter.sampling_num = 4;
            ptr_object_now->m_matching_parameter.sampling_num = 4;
        }
        break;

        case TargetObject::OBJECT_SCU :
        {

            ROS_LOG_WARN("Target grasping object: SCU!");

            gripper_close_length = 9000;

            _taskWindow->getTaskManager()->_qnode->grp_do_initialize_ = false;
            _taskWindow->getTaskManager()->_qnode->grp_is_vacuum_ = true;
            _taskWindow->getTaskManager()->_qnode->grp_is_vacuum_before_ = true;
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
            ////

            //// load mrcnn weight
            ui_cmd_parameter.target_name = target_name;
            // ui_cmd_parameter.weight_number = to_string(current_obj);
            // ui_cmd_parameter.is_sam_mean_size_assigned = false; // SAM paramter
            // ui_cmd_parameter.sam_mean_size = 35736; // SAM paramter
            // ui_cmd_parameter.sam_mask_min_area = 29000;
            // ui_cmd_parameter.sam_mask_max_area = 43000;
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) _taskWindow->getTaskManager()->_qnode->sendChangeRobotTCPCommand(11); // TCP #11, square peg vacuum
            ptr_object_now->tcp_changing_id = 11;

            //// Tool changing slave id
            // _taskWindow->getTaskManager()->_qnode->is_tool_attached = false;
            ptr_object_now->tool_changing_attach_id = 1; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 1; // # slave #1, 2, ...

            _taskWindow->getTaskManager()->_qnode->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            _taskWindow->getTaskManager()->_qnode->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = false;
            ptr_object_now->tip_changing_attach_id = 1; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 1; // tip #1, 2, ...
            _taskWindow->getTaskManager()->_qnode->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            _taskWindow->getTaskManager()->_qnode->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            _taskWindow->getTaskManager()->_qnode->test_cnt_ = 0;

            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = 2;

            // std::vector<double> stacking_trans_scale = {0.0, 0.14, 0.082}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	// ptr_object_now->stacking_single_stack_num_ = 2; // 1개씩 놓는 진행 방향의 물체 개수
	        // ptr_object_now->stacking_line_stack_num_ = 1; // stacking_single_stack_num_만큼 놓는 선의 개수
            // ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            // ptr_object_now->stacking_mode_ = StackingType::GRID_STACKING;

            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;

            //// Scanning sampling number
            ptr_object_now->m_scan_parameter.sampling_num = 8;
            ptr_object_now->m_matching_parameter.sampling_num = 8;
        }
        break;

        case TargetObject::OBJECT_GRILL :
        {
            ROS_LOG_WARN("Target grasping object: Grill!");
            gripper_close_length = 5000;

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
            ////

            //// load mrcnn weight
            ui_cmd_parameter.target_name = target_name;
            // ui_cmd_parameter.weight_number = to_string(current_obj);
            // ui_cmd_parameter.is_sam_mean_size_assigned = true; // SAM paramter
            // ui_cmd_parameter.sam_mean_size = 26000; // SAM paramter
            // ui_cmd_parameter.sam_mask_min_area = 15000;
            // ui_cmd_parameter.sam_mask_max_area = 38000;
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) _taskWindow->getTaskManager()->_qnode->sendChangeRobotTCPCommand(13); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 13;

            //// Tool changing slave id
            // _taskWindow->getTaskManager()->_qnode->is_tool_attached = false;
            ptr_object_now->tool_changing_attach_id = 2; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 2; // # slave #1, 2, ...

            _taskWindow->getTaskManager()->_qnode->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            _taskWindow->getTaskManager()->_qnode->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = false;
            ptr_object_now->tip_changing_attach_id = 2; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 2; // tip #1, 2, ...
            _taskWindow->getTaskManager()->_qnode->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            _taskWindow->getTaskManager()->_qnode->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            _taskWindow->getTaskManager()->_qnode->test_cnt_ = 0;

            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = 3;

            std::vector<double> stacking_trans_scale = {0.0, 0.100, 0.02}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	ptr_object_now->stacking_single_stack_num_ = 3; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 1; // stacking_single_stack_num_만큼 놓는 선의 개수
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::GRID_STACKING;

            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;

            //// Scanning sampling number
            ptr_object_now->m_scan_parameter.sampling_num = 4;
            ptr_object_now->m_matching_parameter.sampling_num = 4;
        }
        break;

        case TargetObject::OBJECT_BOLT :
        {
            ROS_LOG_WARN("Target grasping object: Bolt!");

            gripper_close_length = 1000;

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
            ////

            //// load mrcnn weight
            ui_cmd_parameter.target_name = target_name;
            // ui_cmd_parameter.weight_number = to_string(current_obj);
            // ui_cmd_parameter.is_sam_mean_size_assigned = false; // SAM paramter
            // ui_cmd_parameter.sam_mean_size = 0; // SAM paramter
            // ui_cmd_parameter.sam_mask_min_area = 6000;
            // ui_cmd_parameter.sam_mask_max_area = 300000;
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            //// 2F gripper - before
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) _taskWindow->getTaskManager()->_qnode->sendChangeRobotTCPCommand(12); // TCP #3
            ptr_object_now->tcp_changing_id = 12;

            // 2F gripper - new short version
            // if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) _taskWindow->getTaskManager()->_qnode->sendChangeRobotTCPCommand(17); // TCP #17
            // ptr_object_now->tcp_changing_id = 17;


            //// Vacuum 1cup
            // if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) _taskWindow->getTaskManager()->_qnode->sendChangeRobotTCPCommand(15); // TCP #15

            //// Tool changing slave id
            // _taskWindow->getTaskManager()->_qnode->is_tool_attached = false;
            ptr_object_now->tool_changing_attach_id = 1; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 1; // # slave #1, 2, ...

            _taskWindow->getTaskManager()->_qnode->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            _taskWindow->getTaskManager()->_qnode->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = true;
            ptr_object_now->tip_changing_attach_id = 1; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 1; // tip #1, 2, ...
            _taskWindow->getTaskManager()->_qnode->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            _taskWindow->getTaskManager()->_qnode->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            _taskWindow->getTaskManager()->_qnode->test_cnt_ = 0;

            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = 2;

            std::vector<double> stacking_trans_scale = {0.0, 0.125, 0.082}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	ptr_object_now->stacking_single_stack_num_ = 2; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 1; // stacking_single_stack_num_만큼 놓는 선의 개수
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::GRID_STACKING;

            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;

            //// Scanning sampling number
            ptr_object_now->m_scan_parameter.sampling_num = 1;
            ptr_object_now->m_matching_parameter.sampling_num = 1;
        }
        break;

        case TargetObject::OBJECT_HINGE :
        {
            ROS_LOG_WARN("Target grasping object: Welding hinge!");

            gripper_close_length = 2000;

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
            ////

            //// load mrcnn weight
            ui_cmd_parameter.target_name = target_name;
            ui_cmd_parameter.weight_number = to_string(current_obj);
            ui_cmd_parameter.is_sam_mean_size_assigned = false; // SAM paramter
            ui_cmd_parameter.sam_mean_size = 0; // SAM paramter
            ui_cmd_parameter.sam_mask_min_area = 6000;
            ui_cmd_parameter.sam_mask_max_area = 300000;
            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            if(_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) _taskWindow->getTaskManager()->_qnode->sendChangeRobotTCPCommand(13); // TCP #2
            ptr_object_now->tcp_changing_id = 13;

            //// Tool changing slave id
            // _taskWindow->getTaskManager()->_qnode->is_tool_attached = false;
            ptr_object_now->tool_changing_attach_id = 2; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 2; // # slave #1, 2, ...

            _taskWindow->getTaskManager()->_qnode->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            _taskWindow->getTaskManager()->_qnode->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = false;
            ptr_object_now->tip_changing_attach_id = 2; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 2; // tip #1, 2, ...
            _taskWindow->getTaskManager()->_qnode->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            _taskWindow->getTaskManager()->_qnode->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            _taskWindow->getTaskManager()->_qnode->test_cnt_ = 0;

            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = 9;

            std::vector<double> stacking_trans_scale = {0.14, 0.12, 0.082}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	ptr_object_now->stacking_single_stack_num_ = 2; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 3; // stacking_single_stack_num_만큼 놓는 선의 개수
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::GRID_STACKING;

            _taskWindow->getTaskManager()->_qnode->m_bin_picking_node->is_task_in_progress = false;

            //// Scanning sampling number
            ptr_object_now->m_scan_parameter.sampling_num = 2;
            ptr_object_now->m_matching_parameter.sampling_num = 2;

        }
        break;

    }

    //// 초기 TCP는 tool master 기준
    if(!_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) _taskWindow->getTaskManager()->_qnode->sendChangeRobotTCPCommand(7);; // TCP #7 // Robot end-effector
    // if(!_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) _taskWindow->getTaskManager()->_qnode->sendChangeRobotTCPCommand(1); // TCP #1, 2지그리퍼
    // if(!_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) _taskWindow->getTaskManager()->_qnode->sendChangeRobotTCPCommand(8); // TCP #8 --> Robot Calibration
    // if(!_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) _taskWindow->getTaskManager()->_qnode->sendChangeRobotTCPCommand(9); // TCP #9 --> (0.0, 0.0, 0.0)
    // if(!_taskWindow->getTaskManager()->_qnode->is_tcp_initialized) _taskWindow->getTaskManager()->_qnode->sendChangeRobotTCPCommand(10); // TCP #10, Connector gripper

    // Set scanning parameters
    ptr_object_now->m_scan_parameter.target_id = current_obj;
    ptr_object_now->m_scan_parameter.target_name = target_name;
    ptr_object_now->m_scan_parameter.do_scan_sampling = false;
    ptr_object_now->m_scan_parameter.is_mask_pixel_fixed = false;
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
    ptr_object_now->m_matching_parameter.do_scan_sampling = true;
    // Set grasping parameters
    ptr_object_now->m_grasping_parameter.gripper_open_length = 5000;
    ptr_object_now->m_grasping_parameter.gripper_close_length = gripper_close_length;

    std::vector<int> mask_pixel_list(4,10);
    for(int i=0; i<4; i++) mask_pixel_list[i] = 0;
    ptr_object_now->m_scan_parameter.mask_pixel_list = mask_pixel_list;

    ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////

    //// Enable task button
    if(_taskWindow->getTaskManager()->_qnode->is_task_generated_) {
        // ui.pushButton_doTask_binPicking->setEnabled(true);
        // ui.pushButton_doTask_binPicking_sequentialDemo->setEnabled(true);
    }
}

