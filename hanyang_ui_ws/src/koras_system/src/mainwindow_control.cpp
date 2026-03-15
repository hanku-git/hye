#include "mainwindow_control.hpp"

#include <QtWidgets/QMessageBox>

MainWindow_control::MainWindow_control(int argc, char** argv, QNode *qnode, QWidget *parent)
        : init_argc(argc), init_argv(argv), ui(new Ui::MainWindowDesign), qnode_(qnode) {
    ui->setupUi(this);
    setWindowIcon(QIcon(":/images/icon.png"));

    connect(qnode_, &QNode::rosShutdown, this, &MainWindow_control::close);

    ROS_LOG_WARN("[%s] [line: %d]", __func__, __LINE__);


    list_q_meas_ << ui->lineEdit_q_meas_1 << ui->lineEdit_q_meas_2 << ui->lineEdit_q_meas_3
                 << ui->lineEdit_q_meas_4 << ui->lineEdit_q_meas_5 << ui->lineEdit_q_meas_6
                 << ui->lineEdit_q_meas_7;

    list_q_target_ << ui->lineEdit_q_target_1 << ui->lineEdit_q_target_2
                   << ui->lineEdit_q_target_3 << ui->lineEdit_q_target_4
                   << ui->lineEdit_q_target_5 << ui->lineEdit_q_target_6
                   << ui->lineEdit_q_target_7;

    list_torque_meas_ << ui->lineEdit_torque_meas_1 << ui->lineEdit_torque_meas_2
                      << ui->lineEdit_torque_meas_3 << ui->lineEdit_torque_meas_4
                      << ui->lineEdit_torque_meas_5 << ui->lineEdit_torque_meas_6
                      << ui->lineEdit_torque_meas_7;

    list_x_meas_ << ui->lineEdit_x_meas_1 << ui->lineEdit_x_meas_2 << ui->lineEdit_x_meas_3
                 << ui->lineEdit_x_meas_4 << ui->lineEdit_x_meas_5 << ui->lineEdit_x_meas_6
                 << ui->lineEdit_x_meas_7;

    list_x_target_ << ui->lineEdit_x_target_1 << ui->lineEdit_x_target_2 << ui->lineEdit_x_target_3
                   << ui->lineEdit_x_target_4 << ui->lineEdit_x_target_5 << ui->lineEdit_x_target_6
                   << ui->lineEdit_x_target_7;

    list_force_meas_ << ui->lineEdit_force_meas_1 << ui->lineEdit_force_meas_2
                     << ui->lineEdit_force_meas_3 << ui->lineEdit_force_meas_4
                     << ui->lineEdit_force_meas_5 << ui->lineEdit_force_meas_6;

    list_jog_q_plus_ << ui->pushButton_jog_j1_plus << ui->pushButton_jog_j2_plus
                     << ui->pushButton_jog_j3_plus << ui->pushButton_jog_j4_plus
                     << ui->pushButton_jog_j5_plus << ui->pushButton_jog_j6_plus
                     << ui->pushButton_jog_j7_plus;

    list_jog_q_minus_ << ui->pushButton_jog_j1_minus << ui->pushButton_jog_j2_minus
                      << ui->pushButton_jog_j3_minus << ui->pushButton_jog_j4_minus
                      << ui->pushButton_jog_j5_minus << ui->pushButton_jog_j6_minus
                      << ui->pushButton_jog_j7_minus;

    list_jog_x_plus_ << ui->pushButton_jog_x1_plus << ui->pushButton_jog_x2_plus
                     << ui->pushButton_jog_x3_plus << ui->pushButton_jog_x4_plus
                     << ui->pushButton_jog_x5_plus << ui->pushButton_jog_x6_plus
                     << ui->pushButton_jog_x7_plus;

    list_jog_x_minus_ << ui->pushButton_jog_x1_minus << ui->pushButton_jog_x2_minus
                      << ui->pushButton_jog_x3_minus << ui->pushButton_jog_x4_minus
                      << ui->pushButton_jog_x5_minus << ui->pushButton_jog_x6_minus
                      << ui->pushButton_jog_x7_minus;

    list_step_move_q_plus_ << ui->pushButton_step_move_js_plus_1 << ui->pushButton_step_move_js_plus_2
                           << ui->pushButton_step_move_js_plus_3 << ui->pushButton_step_move_js_plus_4
                           << ui->pushButton_step_move_js_plus_5 << ui->pushButton_step_move_js_plus_6
                           << ui->pushButton_step_move_js_plus_7;

    list_step_move_q_minus_ << ui->pushButton_step_move_js_minus_1 << ui->pushButton_step_move_js_minus_2
                            << ui->pushButton_step_move_js_minus_3 << ui->pushButton_step_move_js_minus_4
                            << ui->pushButton_step_move_js_minus_5 << ui->pushButton_step_move_js_minus_6
                            << ui->pushButton_step_move_js_minus_7;

    list_step_move_x_plus_ << ui->pushButton_step_move_cs_plus_1 << ui->pushButton_step_move_cs_plus_2
                           << ui->pushButton_step_move_cs_plus_3 << ui->pushButton_step_move_cs_plus_4
                           << ui->pushButton_step_move_cs_plus_5 << ui->pushButton_step_move_cs_plus_6
                           << ui->pushButton_step_move_cs_plus_7;

    list_step_move_x_minus_ << ui->pushButton_step_move_cs_minus_1 << ui->pushButton_step_move_cs_minus_2
                            << ui->pushButton_step_move_cs_minus_3 << ui->pushButton_step_move_cs_minus_4
                            << ui->pushButton_step_move_cs_minus_5 << ui->pushButton_step_move_cs_minus_6
                            << ui->pushButton_step_move_cs_minus_7;

    list_dtc_mode_ << ui->checkBox_dtc_1 << ui->checkBox_dtc_2
                   << ui->checkBox_dtc_3 << ui->checkBox_dtc_4
                   << ui->checkBox_dtc_5 << ui->checkBox_dtc_6
                   << ui->checkBox_dtc_7;

    list_dtc_input_ << ui->doubleSpinBox_dtc_1 << ui->doubleSpinBox_dtc_2
                    << ui->doubleSpinBox_dtc_3 << ui->doubleSpinBox_dtc_4
                    << ui->doubleSpinBox_dtc_5 << ui->doubleSpinBox_dtc_6
                    << ui->doubleSpinBox_dtc_7;

    list_col_mal_index_js_ << ui->lineEdit_index_js_1 << ui->lineEdit_index_js_2
                           << ui->lineEdit_index_js_3 << ui->lineEdit_index_js_4
                           << ui->lineEdit_index_js_5 << ui->lineEdit_index_js_6
                           << ui->lineEdit_index_js_7;

    list_col_mal_index_cs_ << ui->lineEdit_index_cs_1 << ui->lineEdit_index_cs_2
                           << ui->lineEdit_index_cs_3 << ui->lineEdit_index_cs_4
                           << ui->lineEdit_index_cs_5 << ui->lineEdit_index_cs_6;

    list_tcp_ << ui->lineEdit_tcp_x_1 << ui->lineEdit_tcp_x_2 << ui->lineEdit_tcp_x_3
              << ui->lineEdit_tcp_x_4 << ui->lineEdit_tcp_x_5 << ui->lineEdit_tcp_x_6;
    list_com_ << ui->lineEdit_com_x << ui->lineEdit_com_y << ui->lineEdit_com_z;

    list_impedance_selection_ << ui->radioButton_imped_selection_1
                              << ui->radioButton_imped_selection_2
                              << ui->radioButton_imped_selection_3
                              << ui->radioButton_imped_selection_4
                              << ui->radioButton_imped_selection_5
                              << ui->radioButton_imped_selection_6;

    list_force_selection_ << ui->radioButton_force_selection_1
                          << ui->radioButton_force_selection_2
                          << ui->radioButton_force_selection_3
                          << ui->radioButton_force_selection_4
                          << ui->radioButton_force_selection_5
                          << ui->radioButton_force_selection_6;

    list_pos_selection_ << ui->radioButton_pos_selection_1
                        << ui->radioButton_pos_selection_2
                        << ui->radioButton_pos_selection_3
                        << ui->radioButton_pos_selection_4
                        << ui->radioButton_pos_selection_5
                        << ui->radioButton_pos_selection_6;

    list_impedance_m_ << ui->lineEdit_impedance_m_1 << ui->lineEdit_impedance_m_2
                      << ui->lineEdit_impedance_m_3 << ui->lineEdit_impedance_m_4
                      << ui->lineEdit_impedance_m_5 << ui->lineEdit_impedance_m_6;

    list_impedance_b_ << ui->lineEdit_impedance_b_1 << ui->lineEdit_impedance_b_2
                      << ui->lineEdit_impedance_b_3 << ui->lineEdit_impedance_b_4
                      << ui->lineEdit_impedance_b_5 << ui->lineEdit_impedance_b_6;

    list_impedance_k_ << ui->lineEdit_impedance_k_1 << ui->lineEdit_impedance_k_2
                      << ui->lineEdit_impedance_k_3 << ui->lineEdit_impedance_k_4
                      << ui->lineEdit_impedance_k_5 << ui->lineEdit_impedance_k_6;

    list_impedance_force_limit_ << ui->lineEdit_impedance_force_limit_1
                                << ui->lineEdit_impedance_force_limit_2
                                << ui->lineEdit_impedance_force_limit_3
                                << ui->lineEdit_impedance_force_limit_4
                                << ui->lineEdit_impedance_force_limit_5
                                << ui->lineEdit_impedance_force_limit_6;

    list_impedance_force_des_ << ui->lineEdit_impedance_force_des_1
                              << ui->lineEdit_impedance_force_des_2
                              << ui->lineEdit_impedance_force_des_3
                              << ui->lineEdit_impedance_force_des_4
                              << ui->lineEdit_impedance_force_des_5
                              << ui->lineEdit_impedance_force_des_6;


    // // ZIVID_Scanning
    // taskTargetParts_ << ui->lineEdit_targetParts_1
    //                  << ui->lineEdit_targetParts_2;

    // fixedMaskPixelList_ << ui->lineEdit_pixelList_1
    //                     << ui->lineEdit_pixelList_2
    //                     << ui->lineEdit_pixelList_3
    //                     << ui->lineEdit_pixelList_4;

    // //// Bin-picking target object select
    // bin_picking_target_object_list_ << ui->radioButton_graspingUI_object_1
    //                                 << ui->radioButton_graspingUI_object_2
    //                                 << ui->radioButton_graspingUI_object_3
    //                                 << ui->radioButton_graspingUI_object_4
    //                                 << ui->radioButton_graspingUI_object_5
    //                                 << ui->radioButton_graspingUI_object_6;


    /// Button connect to function
    // Joint space & Cartesian space control button
    connect(ui->pushButton_set_home_q   , &QPushButton::clicked, this,
            &MainWindow_control::setHomeQBtnCallback);
    connect(ui->pushButton_set_task_q   , &QPushButton::clicked, this,
            &MainWindow_control::setTaskQBtnCallback);
    connect(ui->pushButton_set_task_x   , &QPushButton::clicked, this,
            &MainWindow_control::setTaskXBtnCallback);
    connect(ui->pushButton_set_current_q, &QPushButton::clicked, this,
            &MainWindow_control::setCurrentQBtnCallback);
    connect(ui->pushButton_set_current_x, &QPushButton::clicked, this,
            &MainWindow_control::setCurrentXBtnCallback);
    connect(ui->pushButton_move_q       , &QPushButton::clicked, this,
            &MainWindow_control::moveQBtnCallback);
    connect(ui->pushButton_move_x       , &QPushButton::clicked, this,
            &MainWindow_control::moveXBtnCallback);

    // Step move button
    for (int i = 0; i < 7; i++) {
        connect(list_step_move_q_plus_[i], &QPushButton::clicked, this, [=] () {
            stepMoveQBtnCallback(i, true);
        });
        connect(list_step_move_q_minus_[i], &QPushButton::clicked, this, [=] () {
            stepMoveQBtnCallback(i, false);
        });
        connect(list_step_move_x_plus_[i], &QPushButton::clicked, this, [=] () {
            stepMoveXBtnCallback(i, true);
        });
        connect(list_step_move_x_minus_[i], &QPushButton::clicked, this, [=] () {
            stepMoveXBtnCallback(i, false);
        });
    }

    // Jog button
    connect(ui->pushButton_jog_start, &QPushButton::clicked, this, &MainWindow_control::jogStartBtnCallback);
    connect(ui->pushButton_jog_end  , &QPushButton::clicked, this, &MainWindow_control::jogEndBtnCallback);

    for (int i = 0; i < 7; i++) {
        connect(list_jog_q_plus_[i], &QPushButton::pressed, this, [=] () {
            jogBtnPressed(i, true, true);
        });
        connect(list_jog_q_minus_[i], &QPushButton::pressed, this, [=] () {
            jogBtnPressed(i, false, true);
        });
        connect(list_jog_x_plus_[i], &QPushButton::pressed, this, [=] () {
            jogBtnPressed(i, true, false);
        });
        connect(list_jog_x_minus_[i], &QPushButton::pressed, this, [=] () {
            jogBtnPressed(i, false, false);
        });

        connect(list_jog_q_plus_[i], &QPushButton::released, this, [=] () {
            jogBtnReleased();
        });
        connect(list_jog_q_minus_[i], &QPushButton::released, this, [=] () {
            jogBtnReleased();
        });
        connect(list_jog_x_plus_[i], &QPushButton::released, this, [=] () {
            jogBtnReleased();
        });
        connect(list_jog_x_minus_[i], &QPushButton::released, this, [=] () {
            jogBtnReleased();
        });
    }

// Direct torque control button
    for (int i = 0; i < 7; i++) {
        connect(list_dtc_mode_[i], &QCheckBox::stateChanged, this, [=] () {
            setDtcMode();
        });
    }
    connect(ui->pushButton_dtc, &QCheckBox::clicked, this,
            &MainWindow_control::setDtcInput);

    // Robot setting tab buton
    connect(ui->pushButton_power_on_off, &QPushButton::clicked, [=] () {
        setPowerBtnCallback();
    });
    connect(ui->pushButton_enable_disable  , &QPushButton::clicked, this,
            &MainWindow_control::setEnableBtnCallback);
    connect(ui->pushButton_jamming_state_enable, &QPushButton::clicked, this,
            &MainWindow_control::jammingStateEnableBtnCallback);
    connect(ui->pushButton_friction_model  , &QPushButton::clicked, this,
            &MainWindow_control::setFrictionObserverBtnCallback);
    connect(ui->pushButton_hand_guide      , &QPushButton::clicked, this,
            &MainWindow_control::setHandGuideBtnCallback);
    connect(ui->pushButton_collision_detect, &QPushButton::clicked, this,
            &MainWindow_control::setCollisionDetectionBtnCallback);
    connect(ui->pushButton_error_clear     , &QPushButton::clicked, this,
            &MainWindow_control::clearErrorBtnCallback);
    connect(ui->pushButton_error_clear_2   , &QPushButton::clicked, this,
            &MainWindow_control::clearErrorBtnCallback);
    connect(ui->pushButton_jts_bias        , &QPushButton::clicked, this,
            &MainWindow_control::jtsBiasBtnCallback);
    connect(ui->pushButton_force_bias      , &QPushButton::clicked, this,
            &MainWindow_control::forceBiasBtnCallback);
    connect(ui->pushButton_fts_bias        , &QPushButton::clicked, this,
            &MainWindow_control::ftsBiasBtnCallback);
    connect(ui->pushButton_set_tcp         , &QPushButton::clicked, this,
            &MainWindow_control::setTcpBtnCallback);
    connect(ui->pushButton_set_payload     , &QPushButton::clicked, this,
            &MainWindow_control::setPayloadBtnCallback);
    connect(ui->pushButton_collision_demo_mode, &QPushButton::clicked, this,
            &MainWindow_control::setCollisionDemoModeBtnCallback);
    connect(ui->pushButton_mode_csp         , &QPushButton::clicked, [=] () {
        setDriverCtrlModeBtnCallback(DriverCtrlMode::CSP);
    });
    connect(ui->pushButton_mode_csv         , &QPushButton::clicked, [=] () {
        setDriverCtrlModeBtnCallback(DriverCtrlMode::CSV);
    });
    connect(ui->pushButton_mode_cst         , &QPushButton::clicked, [=] () {
        setDriverCtrlModeBtnCallback(DriverCtrlMode::CST);
    });

    // Brake on / off button
    connect(ui->pushButton_brake_on , &QPushButton::clicked, [=] () {
        setBrakeBtnCallback(true);
    });
    connect(ui->pushButton_brake_off, &QPushButton::clicked, [=] () {
        setBrakeBtnCallback(false);
    });

    // Impedance & gripper tab
    connect(ui->pushButton_impedance_on, &QPushButton::clicked, [=] () {
        impedanceBtnCallback(true);
    });
    connect(ui->pushButton_impedance_off, &QPushButton::clicked, [=] () {
        impedanceBtnCallback(false);
    });
    connect(ui->pushButton_set_led, &QPushButton::clicked, [=] () {
        setLedBtnCallback();
    });
    connect(ui->pushButton_set_grp, &QPushButton::clicked, [=] () {
        setGrpBtnCallback();
    });
    connect(ui->pushButton_grp_ctrl, &QPushButton::clicked, [=] () {
        grpCtrlWordCallback();
    });


    // Task planner tab
    auto taskCallbackFn = [=] (vector<UnitTask> task) {
        qnode_->current_task_list_ = task;
        qnode_->beforeTaskStart();
        qnode_->task_cycle_ = 0;
    };

    connect(ui->pushButton_task_gen_whole_tasks, &QPushButton::clicked, this,
            &MainWindow_control::genTaskBtnCallback);
    connect(ui->pushButton_task_test_1, &QPushButton::clicked,
            [=] () {taskCallbackFn(qnode_->task_planner_->module_task_test1_);});
    connect(ui->pushButton_task_test_2, &QPushButton::clicked,
            [=] () {taskCallbackFn(qnode_->task_planner_->module_task_test2_);});
    connect(ui->pushButton_task_test_3, &QPushButton::clicked,
            [=] () {taskCallbackFn(qnode_->task_planner_->module_task_test3_);});
    connect(ui->pushButton_task_test_4, &QPushButton::clicked,
            [=] () {taskCallbackFn(qnode_->task_planner_->module_task_test4_);});
    connect(ui->pushButton_task_test_5, &QPushButton::clicked,
            [=] () {taskCallbackFn(qnode_->task_planner_->module_task_test5_);});
    connect(ui->pushButton_load_identification, &QPushButton::clicked,
            [=] () {taskCallbackFn(qnode_->task_planner_->load_id_task_);});

    // Tool change task
    connect(ui->pushButton_task_grp_1_attach, &QPushButton::clicked,
            [=] () {taskCallbackFn(qnode_->task_planner_->module_task_tool1_attach_);});
    connect(ui->pushButton_task_grp_1_detach, &QPushButton::clicked,
            [=] () {taskCallbackFn(qnode_->task_planner_->module_task_tool1_detach_);});
    connect(ui->pushButton_task_grp_2_attach, &QPushButton::clicked,
            [=] () {taskCallbackFn(qnode_->task_planner_->module_task_tool2_attach_);});
    connect(ui->pushButton_task_grp_2_detach, &QPushButton::clicked,
            [=] () {taskCallbackFn(qnode_->task_planner_->module_task_tool2_detach_);});
    connect(ui->pushButton_task_grp_3_attach, &QPushButton::clicked,
            [=] () {taskCallbackFn(qnode_->task_planner_->module_task_tool3_attach_);});
    connect(ui->pushButton_task_grp_3_detach, &QPushButton::clicked,
            [=] () {taskCallbackFn(qnode_->task_planner_->module_task_tool3_detach_);});

    // Development tab
    connect(ui->pushButton_update_json_params  , &QPushButton::clicked, this,
            &MainWindow_control::updateJsonParamsCallback);
    connect(ui->pushButton_rollback_json_params, &QPushButton::clicked, this,
            &MainWindow_control::rollbackJsonParamsCallback);

    // Recording button
    connect(ui->pushButton_recording_start  , &QPushButton::clicked, this,
            &MainWindow_control::recordingStartBtnCallback);
    connect(ui->pushButton_recording_end, &QPushButton::clicked, this,
            &MainWindow_control::recordingEndBtnCallback);

    // Stop button
    connect(ui->pushButton_stop_robot, &QPushButton::clicked, this,
            &MainWindow_control::stopRobotBtnCallback);
    connect(ui->pushButton_stop_all  , &QPushButton::clicked, this,
            &MainWindow_control::stopAllBtnCallback);
    connect(ui->pushButton_stop_task , &QPushButton::clicked, this,
            &MainWindow_control::stopTaskBtnCallback);
    connect(ui->pushButton_task_pause, &QPushButton::clicked, this,
            &MainWindow_control::pauseTaskBtnCallback);
    connect(ui->pushButton_task_resume, &QPushButton::clicked, this,
            &MainWindow_control::resumeTaskBtnCallback);


    //plc
    connect(ui->pushButton_set_step_motor_position, &QPushButton::clicked, this,
            &MainWindow_control::SetStepMotorPosition);
    connect(ui->pushButton_set_step_motor_speed, &QPushButton::clicked, this,
            &MainWindow_control::SetStepMotorVelocity);


    //// Robot calibration dialog

#if CALIBRATION_SKIP
#else
    cal_dialog_ = new cal_dialog(this, qnode_);
    connect(ui->pushButton_cal_dialog, &QPushButton::clicked, this,
            &MainWindow_control::OpenDialogCallback);
#endif

    // llm_dialog_ = new llm_dialog(this, qnode_);
    // connect(ui->pushButton_llm_dialog, &QPushButton::clicked, this,
    //         &MainWindow_control::OpenLLMDialog);

#if BIN_PICKING_FLAG
    //// Bin picking node
    qnode_->m_bin_picking_node = new BinPickingNode(init_argc, init_argv);

    //// Bin picking dialog
    bin_picking_dialog_ = new BinPickingDialog(this);
    bin_picking_dialog_->setModal(false);
    bin_picking_dialog_->setWindowTitle(tr("Bin Picking Manager"));
    connect(ui->pushButton_bin_picking_dialog, &QPushButton::clicked, this,
        &MainWindow_control::openBinPickingDialog);
#endif

    ////////////////////////////////
    //// qnode_ pointer allocation
    initNodePtr();
    ////////////////////////////////


#if defined(JS_VEL) && defined(JS_ACC) && defined(CS_VEL) && defined(CS_ACC)
    ui->doubleSpinBox_qd_target ->setValue(JS_VEL);
    ui->doubleSpinBox_qdd_target->setValue(JS_ACC);
    ui->doubleSpinBox_xd_target ->setValue(CS_VEL);
    ui->doubleSpinBox_xdd_target->setValue(CS_ACC);
#endif

#if defined(JS_VEL_TASK) && defined(JS_ACC_TASK) && defined(CS_VEL_TASK) && defined(CS_ACC_TASK)
    ui->doubleSpinBox_task_qd ->setValue(JS_VEL_TASK);
    ui->doubleSpinBox_task_qdd->setValue(JS_ACC_TASK);
    ui->doubleSpinBox_task_xd ->setValue(CS_VEL_TASK);
    ui->doubleSpinBox_task_xdd->setValue(CS_ACC_TASK);
#endif

#ifdef DEFAULT_TCP
    CsDouble default_tcp = DEFAULT_TCP;

    for (int i = 0; i < CS_DOF; i++) {
        list_tcp_[i]->setText(QString::number(default_tcp[i], 'f', 5));
    }
#endif

#if defined(DEFAULT_PAYLOAD) && defined(DEFAULT_COM)
    double default_com[3] = DEFAULT_COM;

    ui->lineEdit_payload->setText(QString::number(DEFAULT_PAYLOAD, 'f', 5));

    for (int i = 0; i < 3; i++) {
        list_com_[i]->setText(QString::number(default_com[i], 'f', 5));
    }
#endif

    qnode_->init();

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow_control::timerCallback);
    timer_->start(100);

    // ui->groupBoxTorqueDirectInput->setVisible(false);

    for (int i = 6; i > JS_DOF - 1; i--) {
        list_jog_q_plus_ [i]->setEnabled(false);
        list_jog_q_minus_[i]->setEnabled(false);
        list_torque_meas_[i]->setEnabled(false);

        list_q_meas_  [i]->setEnabled(false);
        list_q_target_[i]->setEnabled(false);

        list_col_mal_index_js_[i]->setEnabled(false);

        list_step_move_q_plus_ [i]->setEnabled(false);
        list_step_move_q_minus_[i]->setEnabled(false);

        list_dtc_mode_[i]->setEnabled(false);
        list_dtc_input_[i]->setEnabled(false);
    }

    if (JS_DOF < 7) {
        list_jog_x_plus_ [6]->setEnabled(false);
        list_jog_x_minus_[6]->setEnabled(false);
        list_x_meas_  [6]->setEnabled(false);
        list_x_target_[6]->setEnabled(false);

        list_step_move_x_plus_ [6]->setEnabled(false);
        list_step_move_x_minus_[6]->setEnabled(false);

        list_dtc_mode_[6]->setEnabled(false);
        list_dtc_input_[6]->setEnabled(false);
    }

    ui->tabWidget->setCurrentIndex(0);
    ui->tabWidget_2->setCurrentIndex(0);

    // #if BIN_PICKING_FLAG
    //     //// Parameter initialize
    //     bin_picking_dialog_->initializeDialog();
    // #endif
    //// Bin Picking Parameter Initialize
    if (SW_MODE_HD_LLM) {
        bin_picking_dialog_->initializeDialog();
    } else if (SW_MODE_COOKING) {

    } else if (SW_MODE_HANYANG_ENG) {
        // bin_picking_dialog_->initializeDialog();
    } else {
        // bin_picking_dialog_->initializeDialog();
    }

    connect(ui->pushButton_exit_IRLdeveloperWindow, &QPushButton::clicked, this, [this] {
        emit closeIRLDeveloperWindow();
    });

}

MainWindow_control::~MainWindow_control() {
    delete ui;
    //delete qnode_;
    // delete cal_dialog_;
#if BIN_PICKING_FLAG
    delete bin_picking_dialog_;
#endif
}

// step motor plc
   // STEP_MOTOR_POSITION_CONTROL, 12
   // STEP_MOTOR_SET_VELOCITY 13
void MainWindow_control::SetStepMotorPosition() {
    int value = ui->lineEdit_pulse_position->text().toInt();
    std::cout << "text value " << value << std::endl;
    qnode_->setPLCModbusCommandWriteRegister(12, value); // 위치 값 저장
    //위 함수에 이미 포함됨 --> 실행 구동 명령 qnode_->setPLCModbusCommand(0);
}
void MainWindow_control::SetStepMotorVelocity() {
    int value = ui->lineEdit_pulse_velocity->text().toInt();
    std::cout << "text value " << value << std::endl;
    qnode_->setPLCModbusCommandWriteRegister(13, value);
}


void MainWindow_control::timerCallback() {
    if (JS_DOF == 7 && qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION) {
        list_x_meas_[6]->setEnabled(true);
        list_x_meas_[6]->setText(QString::number(qnode_->params_.meas.q[2], 'f', 5));;

        list_step_move_x_plus_ [6]->setEnabled(true);
        list_step_move_x_minus_[6]->setEnabled(true);
    } else if (JS_DOF == 7 && qnode_->params_.mode.ik_method == InvKineMethod::CLOSED_FORM) {
        list_x_meas_[6]->setEnabled(false);
    } else {
        list_x_meas_[6]->setEnabled(false);
    }

    for (size_t i = 0; i < JS_DOF; i++) {
        list_q_meas_[i]->setText(QString::number(qnode_->params_.meas.q[i], 'f', 5));
        list_torque_meas_[i]->setText(QString::number(qnode_->params_.meas.torque_jts[i], 'f', 5));
        list_col_mal_index_js_[i]->setText(qnode_->params_.col.flag[i] ||
                                           qnode_->params_.malfunction.flag[i] ||
                                           qnode_->params_.torque_limit.flag[i] ? "X" : "O");
    }

    for (size_t i = 0; i < CS_DOF; i++) {
        list_x_meas_[i]->setText(QString::number(qnode_->params_.meas.x[i], 'f', 5));
        list_force_meas_[i]->setText(QString::number(qnode_->params_.meas.force[i], 'f', 5));
        list_col_mal_index_cs_[i]->setText("O"); // TODO: 조립이나 힘 관련 작업 시의 상태를 반영할 수 있도록 변경
    }

    if (qnode_->rsc_params_.is_joint_connected) {
        ui->lineEdit_status_power_state->setText("Power on");
    } else {
        ui->lineEdit_status_power_state->setText("Power off");
    }

    if (qnode_->params_.status.is_enable) {
        ui->lineEdit_status_enable->setText("Enabled");
    } else {
        ui->lineEdit_status_enable->setText("Disabled");

        // if (qnode_->is_task_mode_) {
        //     ROS_LOG_ERROR("Robot is disabled: End task mode");
        //     qnode_->is_task_mode_ = false;
        // }
    }

    if (qnode_->params_.mode.is_friction_observer_mode) {
        ui->lineEdit_status_friction_model->setText("Observer");
    } else {
        ui->lineEdit_status_friction_model->setText("Model");
    }

    if (qnode_->params_.mode.is_teaching_mode) {
        ui->lineEdit_status_hand_guide->setText("On");
    } else {
        ui->lineEdit_status_hand_guide->setText("Off");
    }

    if (qnode_->params_.mode.is_collision_detection_mode) {
        ui->lineEdit_status_collision_detect->setText("On");
    } else {
        ui->lineEdit_status_collision_detect->setText("Off");
    }

    if (qnode_->params_.mode.is_collision_demo_mode) {
        ui->lineEdit_status_collision_demo_mode->setText("On");
    } else {
        ui->lineEdit_status_collision_demo_mode->setText("Off");
    }

    if (qnode_->params_.malfunction.status && qnode_->params_.col.check_flag) {
        ui->lineEdit_status_error  ->setText("Mal/Col");
        ui->lineEdit_status_error_2->setText("Mal/Col");
    } else if (qnode_->params_.malfunction.status && !qnode_->params_.col.check_flag) {
        ui->lineEdit_status_error  ->setText("Malfunction");
        ui->lineEdit_status_error_2->setText("Malfunction");
    } else if (!qnode_->params_.malfunction.status && qnode_->params_.col.check_flag) {
        ui->lineEdit_status_error  ->setText("Collision");
        ui->lineEdit_status_error_2->setText("Collision");
    } else if (!qnode_->params_.malfunction.status && !qnode_->params_.col.check_flag) {
        ui->lineEdit_status_error  ->setText("Normal");
        ui->lineEdit_status_error_2->setText("Normal");
    }

    if (qnode_->params_.torque_limit.status) {
        ui->lineEdit_status_error  ->setText("Torque limit");
        ui->lineEdit_status_error_2->setText("Torque limit");
    }

    // Malfunction 상태에서 task mode 종료
    if (qnode_->params_.malfunction.status) {
        qnode_->is_task_mode_ = false;
    }

    // ui->lineEdit_status_brake; // TODO: brake 상태 변수 추가 혹은 on/off 버튼으로 변경

    // Relative 버튼 on/off 여부를 통해 target value 리셋
    static bool is_relative_js_prev = false;
    static bool is_relative_cs_prev = false;
    static bool is_tcp_move_prev    = false;

    bool is_relative_js = ui->checkBox_is_relative_move_js->isChecked();
    bool is_relative_cs = ui->checkBox_is_relative_move_cs->isChecked();
    bool is_tcp_move = ui->checkBox_is_ee_frame->isChecked();

    if (is_relative_js && !is_relative_js_prev) {
        // check
        for (size_t i = 0; i < JS_DOF; i++) {
            list_q_target_[i]->setText(QString::number(0.0, 'f', 1));
        }
    } else if (!is_relative_js && is_relative_js_prev) {
        // uncheck
        for (size_t i = 0; i < JS_DOF; i++) {
            list_q_target_[i]->setText(QString::number(qnode_->params_.meas.q[i], 'f', 3));
        }
    }

    if ((is_relative_cs && !is_relative_cs_prev) || (is_tcp_move && !is_tcp_move_prev)) {
        // check
        for (size_t i = 0; i < CS_DOF; i++) {
            list_x_target_[i]->setText(QString::number(0.0, 'f', 1));
        }

        if (JS_DOF == 7 && qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION) {
            list_x_target_[6]->setText(QString::number(0.0, 'f', 1));;
        } else if (JS_DOF == 7 && qnode_->params_.mode.ik_method == InvKineMethod::CLOSED_FORM) {
            list_x_target_[6]->setEnabled(false);
        } else {
            list_x_target_[6]->setEnabled(false);
        }
    } else if ((!is_relative_cs && is_relative_cs_prev) || (!is_tcp_move && is_tcp_move_prev)) {
        // uncheck
        for (size_t i = 0; i < CS_DOF; i++) {
            list_x_target_[i]->setText(QString::number(qnode_->params_.meas.x[i], 'f', 5));
        }

        if (JS_DOF == 7 && qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION) {
            list_x_target_[6]->setText(QString::number(qnode_->params_.meas.q[2], 'f', 5));;
        } else if (JS_DOF == 7 && qnode_->params_.mode.ik_method == InvKineMethod::CLOSED_FORM) {
            list_x_target_[6]->setEnabled(false);
        } else {
            list_x_target_[6]->setEnabled(false);
        }
    }

    is_relative_js_prev = is_relative_js;
    is_relative_cs_prev = is_relative_cs;
    is_tcp_move_prev    = is_tcp_move;

#if BIN_PICKING_FLAG
    bin_picking_dialog_->timerCallback();
    bin_picking_dialog_->timerSubCallback();

#endif
}

// Joint space & Cartesian space control button
void MainWindow_control::setHomeQBtnCallback() {
    ui->checkBox_is_relative_move_js->setChecked(false);

    timerCallback();

    JsDouble q_home = kQHome;

    for (size_t i = 0; i < JS_DOF; i++) {
        list_q_target_[i]->setText(QString::number(q_home[i], 'f', 3));
    }
}

void MainWindow_control::setTaskQBtnCallback() {
    ui->checkBox_is_relative_move_js->setChecked(false);

    timerCallback();

    JsDouble q_task = kQTask;

    for (size_t i = 0; i < JS_DOF; i++) {
        list_q_target_[i]->setText(QString::number(q_task[i], 'f', 3));
    }
}

void MainWindow_control::setTaskXBtnCallback() {
    ui->checkBox_is_relative_move_cs->setChecked(false);
    ui->checkBox_is_ee_frame->setChecked(false);

    timerCallback();

    CsDouble x_task = kXTask;

    for (size_t i = 0; i < CS_DOF; i++) {
        list_x_target_[i]->setText(QString::number(x_task[i], 'f', 5));
    }

    if (JS_DOF == 7 && qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION) {
        list_x_target_[6]->setText(QString::number(0, 'f', 1));
    }
}

void MainWindow_control::setCurrentQBtnCallback() {
    ui->checkBox_is_relative_move_js->setChecked(false);

    timerCallback();

    JsDouble q_current = qnode_->params_.meas.q;

    for (size_t i = 0; i < JS_DOF; i++) {
        list_q_target_[i]->setText(QString::number(q_current[i], 'f', 3));
    }
}

void MainWindow_control::setCurrentXBtnCallback() {
    ui->checkBox_is_relative_move_cs->setChecked(false);
    ui->checkBox_is_ee_frame->setChecked(false);

    timerCallback();

    CsDouble x_current = qnode_->params_.meas.x;

    for (size_t i = 0; i < CS_DOF; i++) {
        list_x_target_[i]->setText(QString::number(x_current[i], 'f', 5));
    }

    if (JS_DOF == 7 && qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION) {
        list_x_target_[6]->setText(QString::number(qnode_->params_.meas.q[2], 'f', 5));
    }
}

void MainWindow_control::moveQBtnCallback() {
    double qd  = ui->doubleSpinBox_qd_target->value();
    double qdd = ui->doubleSpinBox_qdd_target->value();

    bool is_relative = ui->checkBox_is_relative_move_js->isChecked();

    JsDouble q;

    for (int i = 0; i < JS_DOF; i++) {
        q[i] = list_q_target_[i]->text().toDouble();
    }

    qnode_->moveQ(q, qd, qdd, is_relative);
}

void MainWindow_control::moveXBtnCallback() {
    double xd  = ui->doubleSpinBox_xd_target->value();
    double xdd = ui->doubleSpinBox_xdd_target->value();

    bool is_relative   = ui->checkBox_is_relative_move_cs->isChecked();
    bool is_base_frame = !ui->checkBox_is_ee_frame->isChecked();

    CsDouble x;

    for (int i = 0; i < CS_DOF; i++) {
        x[i] = list_x_target_[i]->text().toDouble();
    }

    if (JS_DOF == 6 || ((JS_DOF == 7) && !(qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION))) {
        qnode_->moveX(x, xd, xdd, is_base_frame, is_relative);
    } else if ((JS_DOF == 7) && (qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION)) {
        double q_redundant = list_x_target_[6]->text().toDouble();
        qnode_->moveX(x, xd, xdd, is_base_frame, is_relative, q_redundant);
    }
}

void MainWindow_control::stepMoveQBtnCallback(uint index, bool is_plus) {
    JsDouble q;

    for (int i = 0; i < JS_DOF; i++) {
        q[i] = 0;
    }

    q[index] = is_plus ? ui->doubleSpinBox_step_move_q->value() :
               ui->doubleSpinBox_step_move_q->value() * -1;

    double qd  = ui->doubleSpinBox_step_move_qd ->value();
    double qdd = ui->doubleSpinBox_step_move_qdd->value();

    qnode_->moveQ(q, qd, qdd, true);
}

void MainWindow_control::stepMoveXBtnCallback(uint index, bool is_plus) {
    CsDouble x;
    double q_redundant;

    for (int i = 0; i < CS_DOF; i++) {
        x[i] = 0;
    }

    if (index == 6) {
        // JS_DOF = 7 case
        q_redundant = is_plus ? ui->doubleSpinBox_step_move_q->value() :
                      ui->doubleSpinBox_step_move_q->value() * -1;
    } else if (index < 3) {
        x[index] = is_plus ? ui->doubleSpinBox_step_move_x->value() :
                   ui->doubleSpinBox_step_move_x->value() * -1;
    } else {
        x[index] = is_plus ? ui->doubleSpinBox_step_move_q->value() :
                   ui->doubleSpinBox_step_move_q->value() * -1;
    }

    double xd  = ui->doubleSpinBox_step_move_xd ->value();
    double xdd = ui->doubleSpinBox_step_move_xdd->value();

    bool is_base_frame = !ui->checkBox_step_move_is_tcp_move->isChecked();

    if (JS_DOF == 6) {
        qnode_->moveX(x, xd, xdd, is_base_frame, true);
    } else if ((JS_DOF == 7) && (qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION)) {
        qnode_->moveX(x, xd, xdd, is_base_frame, true, q_redundant);
    }
}

// Jog button
void MainWindow_control::jogBtnCallback() {

}

void MainWindow_control::jogBtnPressed(uint index, bool is_plus, bool is_joint_space) {
    if (qnode_->is_jog_mode_) {
        if ((qnode_->jog_space_ == JS_JOG && is_joint_space) || (qnode_->jog_space_ == CS_JOG && !is_joint_space)) {
            double vel, acc;

            if (!is_joint_space && index < 3) {
                vel = ui->doubleSpinBox_jog_xd->value();
                vel *= is_plus ? +1 : -1;
                acc = fabs(vel) * 2.0;
            } else {
                vel = ui->doubleSpinBox_jog_qd->value();
                vel *= is_plus ? +1 : -1;
                acc = fabs(vel) * 2.0;
            }

            qnode_->jogStart(index, vel, acc, is_joint_space, !ui->checkBox_is_ee_frame_jog->isChecked());
        }
    }
}

void MainWindow_control::jogBtnReleased() {
    qnode_->jogEnd(0.5);
}

void MainWindow_control::jogStartBtnCallback() {
    if (ui->radioButton_jog_q->isChecked()) {
        qnode_->jog_space_ = JS_JOG;
    } else if (ui->radioButton_jog_x->isChecked()) {
        qnode_->jog_space_ = CS_JOG;
    } else if (ui->radioButton_jog_pad->isChecked()) {
        qnode_->jog_space_ = XPAD_JOG;
        qnode_->jog_xpad_trans_ = ui->doubleSpinBox_jog_xd->value();
        qnode_->jog_xpad_ori_   = ui->doubleSpinBox_jog_qd->value();
    }

    qnode_->is_jog_mode_ = true;
}

void MainWindow_control::jogEndBtnCallback() {
    qnode_->is_jog_mode_ = false;
}

// Direct torque control button
void MainWindow_control::setDtcMode() {
    JsBool dtc_mode;
    for (int i = 0; i < JS_DOF; i++) {
        dtc_mode[i] = list_dtc_mode_[i]->isChecked();
    }
    qnode_->setDtcMode(dtc_mode);
}

void MainWindow_control::setDtcInput() {
    JsDouble dtc_input;
    for (int i = 0; i < JS_DOF; i++) {
        dtc_input[i] = list_dtc_input_[i]->value();
    }
    qnode_->setDtcInput(dtc_input);
}

/// Robot setting tab button
void MainWindow_control::setPowerBtnCallback() {
    qnode_->setPower(!qnode_->rsc_params_.is_joint_connected);
}

void MainWindow_control::setEnableBtnCallback() {
    qnode_->setEnable(!qnode_->params_.status.is_enable);
}

void MainWindow_control::jammingStateEnableBtnCallback() {
    qnode_->jammingStateEnable();
}

void MainWindow_control::setFrictionObserverBtnCallback() {
    qnode_->setFrictionObserver(!qnode_->params_.mode.is_friction_observer_mode);
}

void MainWindow_control::setHandGuideBtnCallback() {
    qnode_->setHandGuide(!qnode_->params_.mode.is_teaching_mode);
}

void MainWindow_control::setCollisionDetectionBtnCallback() {
    qnode_->setCollisionDetection(!qnode_->params_.mode.is_collision_detection_mode);
}

void MainWindow_control::setCollisionDemoModeBtnCallback() {
    qnode_->setCollisionDemoMode(!qnode_->params_.mode.is_collision_demo_mode);
}

void MainWindow_control::setInvKineMethodBtnCallback(InvKineMethod ik_method) {
    if (ik_method == InvKineMethod::CLOSED_FORM) {
        qnode_->setInvKineMethod("closed");
    } else if (ik_method == InvKineMethod::JACOBIAN) {
        qnode_->setInvKineMethod("jacobian");
    } else if (ik_method == InvKineMethod::OPTIMIZATION) {
        qnode_->setInvKineMethod("optimization");
    }
}

void MainWindow_control::setDriverCtrlModeBtnCallback(DriverCtrlMode driver_ctrl_mode) {
    if (driver_ctrl_mode == DriverCtrlMode::CSP) {
        qnode_->setDriverCtrlMode("csp_ctrl_mode");
    } else if (driver_ctrl_mode == DriverCtrlMode::CSV) {
        qnode_->setDriverCtrlMode("csv_ctrl_mode");
    } else if (driver_ctrl_mode == DriverCtrlMode::CST) {
        qnode_->setDriverCtrlMode("cst_ctrl_mode");
    }
}

void MainWindow_control::setBrakeBtnCallback(bool flag) {
    if (flag) {
        qnode_->setCommand("brake_on");
    } else {
        qnode_->setCommand("brake_off");
    }
}

void MainWindow_control::clearErrorBtnCallback() {
    qnode_->clearError();
}

void MainWindow_control::jtsBiasBtnCallback() {
    qnode_->setCommand("jts_bias");
}

void MainWindow_control::ftsBiasBtnCallback() {
    qnode_->setCommand("fts_bias");
}

void MainWindow_control::forceBiasBtnCallback() {
    qnode_->setCommand("force_bias");
}

void MainWindow_control::setTcpBtnCallback() {
    CsDouble tcp;

    for (int i = 0; i < CS_DOF; i++) {
        tcp[i] = list_tcp_[i]->text().toDouble();
    }

    qnode_->setTcp(tcp);
}

void MainWindow_control::setPayloadBtnCallback() {
    std::vector<double> com(3);
    for (int i = 0; i < 3; i++) {
        com[i] = list_com_[i]->text().toDouble();
    }
    qnode_->setPayload(ui->lineEdit_payload->text().toDouble(), com);
}

/// Impedance & gripper tab
// Impedance button
void MainWindow_control::impedanceBtnCallback(bool flag) {
    Impedance imped;
if (ui->radioButton_impedance_tool->isChecked()) {
        imped.is_tool_frame = true;
    } else if (ui->radioButton_impedance_base->isChecked()) {
        imped.is_tool_frame = false;
    }

    for (int i = 0; i < CS_DOF; i++) {
        imped.m[i] = list_impedance_m_[i]->text().toDouble();
        imped.b[i] = list_impedance_b_[i]->text().toDouble();
        imped.k[i] = list_impedance_k_[i]->text().toDouble();

        imped.force_limit[i] = list_impedance_force_limit_[i]->text().toDouble();
        imped.force_des[i]   = list_impedance_force_des_  [i]->text().toDouble();

        imped.imped_selection[i] = list_impedance_selection_[i]->isChecked();
        imped.force_selection[i] = list_force_selection_[i]->isChecked();
        imped.pos_selection[i]   = list_pos_selection_[i]->isChecked();
    }

    qnode_->setImpedance(imped, flag);
}

// Gripper button
void MainWindow_control::grpEnableBtnCallback() {

}

void MainWindow_control::grpResetBtnCallback() {

}

void MainWindow_control::grpGraspBtnCallback() {

}

void MainWindow_control::grpReleaseBtnCallback() {

}

void MainWindow_control::grpMoveBtnCallback() {

}

void MainWindow_control::setLedBtnCallback() {
    uint16_t command, color;
    command = ui->spinBox_led_switch->value();
    color   = ui->spinBox_led_color->value();
    qnode_->setLed(command, color);
}

void MainWindow_control::setGrpBtnCallback() {
    uint16_t command, value, address;
    command = ui->spinBox_grp_cmd->value();
    value   = ui->spinBox_grp_val->value();
    address = ui->spinBox_grp_ctrl->value();
    qnode_->setGrp(command, value, address);
}

void MainWindow_control::grpCtrlWordCallback() {
    uint16_t grp_ctrl_word;
    grp_ctrl_word = ui->spinBox_grp_ctrl->value();
    qnode_->setGrpCtrlWord(grp_ctrl_word);
}

/// Task planner tab
// Task related button
void MainWindow_control::resetTaskParamsBtnCallback() {

}

void MainWindow_control::genTaskBtnCallback() {
    qnode_->task_planner_->ref_unit_task_.vel_js = ui->doubleSpinBox_task_qd->value();
    qnode_->task_planner_->ref_unit_task_.acc_js = ui->doubleSpinBox_task_qdd->value();
    qnode_->task_planner_->ref_unit_task_.vel_cs = ui->doubleSpinBox_task_xd->value();
    qnode_->task_planner_->ref_unit_task_.acc_cs = ui->doubleSpinBox_task_xdd->value();

    for (int i = 0; i < CS_DOF; i++) {
        qnode_->task_planner_->impedance_default_.m[i] = list_impedance_m_[i]->text().toDouble();
        qnode_->task_planner_->impedance_default_.b[i] = list_impedance_b_[i]->text().toDouble();
        qnode_->task_planner_->impedance_default_.k[i] = list_impedance_k_[i]->text().toDouble();

        qnode_->task_planner_->impedance_default_.force_limit[i]     = list_impedance_force_limit_[i]->text().toDouble();
        qnode_->task_planner_->impedance_default_.force_selection[i] = list_impedance_selection_[i]->isChecked();
    }

    qnode_->task_planner_->makeTaskList();
}

/// Development tab
void MainWindow_control::updateJsonParamsCallback() {
    qnode_->updateParam();
}

void MainWindow_control::rollbackJsonParamsCallback() {
    qnode_->rollbackParam();
}

// Recording button
void MainWindow_control::recordingStartBtnCallback() {
    qnode_->recordingStart(ui->lineEdit_save_file_name->text().toStdString());
}

void MainWindow_control::recordingEndBtnCallback() {
    qnode_->recordingEnd();
}

// Stop button
void MainWindow_control::stopAllBtnCallback() {
    qnode_->is_task_mode_ = false;
    qnode_->stopRobot();

}

void MainWindow_control::stopRobotBtnCallback() {
    qnode_->stopRobot();
}

void MainWindow_control::stopGrpBtnCallback() {

}

void MainWindow_control::stopTaskBtnCallback() {
    qnode_->is_task_mode_ = false;
}

void MainWindow_control::pauseTaskBtnCallback() {
    qnode_->pauseTask();
}

void MainWindow_control::resumeTaskBtnCallback() {
    qnode_->resumeTask();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Bin Picking //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#if BIN_PICKING_FLAG

void MainWindow_control::openBinPickingDialog() {
    // bin_picking_dialog_->exec(); // dialog open
    bin_picking_dialog_->show(); // dialog open
}


void MainWindow_control::initNodePtr() {
    // ROS_LOG_INFO("Initialize qnode pointer!");
    bin_picking_dialog_->setNodePtr(qnode_);
}

#endif

// void MainWindow_control::pushButtonDoScanningZIVIDClickedCallback() {
//     taskScanningParameter scan_parameter;
//     scan_parameter.target_id = taskTargetParts_[0]->text().toInt();
//     scan_parameter.target_name = taskTargetParts_[1]->text().toStdString();
//     scan_parameter.is_mask_pixel_fixed = ui->radioButton_assemblyUI_isMaskFixed->isChecked();
//     std::vector<int> mask_pixel_list(4,10);
//     mask_pixel_list[0] = fixedMaskPixelList_[0]->text().toInt();
//     mask_pixel_list[1] = fixedMaskPixelList_[1]->text().toInt();
//     mask_pixel_list[2] = fixedMaskPixelList_[2]->text().toInt();
//     mask_pixel_list[3] = fixedMaskPixelList_[3]->text().toInt();
//     scan_parameter.mask_pixel_list = mask_pixel_list;
//     scan_parameter.sampling_num = ui->lineEdit_scanSamplingNum->text().toInt();
//     scan_parameter.is_base_frame_unknown = false;
//     scan_parameter.do_scan_sampling = ui->radioButton_assemblyUI_scanSampling->isChecked();
//     scan_parameter.do_save_data = ui->radioButton_assemblyUI_scanDataSave->isChecked();
//     scan_parameter.do_image_processing = ui->radioButton_assemblyUI_scanDoImageProcessing->isChecked();
//     scan_parameter.do_single_matching = true; // matching service
//     scan_parameter.do_not_scan_do_load_data = ui->radioButton_assemblyUI_scanDataLoad->isChecked();
//     scan_parameter.skip_detection_mask = false;

//     // ROS_LOG_INFO("test id: %d", target_id);
//     // if(target_id > 0 && target_id < 8 || target_id==13 || target_id==21 || target_id==31) // 1~7
//     if(scan_parameter.target_id >= 31 && scan_parameter.target_id < 37) // bin picking(31~36)
//     {
//         qnode_->m_bin_picking_node->scanZIVID(scan_parameter);
//     }
//     else
//     {
//         ROS_LOG_INFO("Wrong target id!");
//     }
// }

// void MainWindow_control::pushButtonSetPreDetectedPoseClickedCallback() {
//     std::vector<double> detected_pose = qnode_->m_bin_picking_node->getDetectedPose();
//     std::vector<double> tool_relative_pose(6, 0.0);
//     double approach_distance = qnode_->m_bin_picking_node->getApproachDistance(); // [m]
//     tool_relative_pose[2] = -approach_distance; // Tool 기준이므로 부호 반대
//     // gripper open length
//     uint16_t gripper_open_length = qnode_->m_bin_picking_node->getGripperOpenLength();
//     ui->task_spinBox_gripperPosValue->setValue(gripper_open_length);

//     std::vector<double> current_pose = detected_pose;
//     std::vector<double> approach_pose;
//     // detected_pose로부터 Tool frame 기준의 상대 이동 (approach distance 고려)
//     for (std::size_t i = 0; i < 3; i++) { current_pose[i + 3] = current_pose[i + 3] * kDeg2Rad; }
//     qnode_->m_bin_picking_node->m_bp_math.transformToolFrame(tool_relative_pose, current_pose, approach_pose); // [m], [rad]
//     for (std::size_t i = 0; i < 3; i++) { approach_pose[i + 3] = approach_pose[i + 3] * kRad2Deg; }
//     for (std::size_t i = 0; i < detected_pose.size(); i++)
//     {
//         if(ui->pushButton_setPreDetectedPose->isChecked())
//         {
//             robotATargetXList_[i]->setText(QString::number(approach_pose[i], 'f', 5)); // Approach distance 고려
//         }
//         else
//         {
//             robotATargetXList_[i]->setText(QString::number(detected_pose[i], 'f', 5)); // detected grasping pose
//         }
//     }
// }

// void MainWindow_control::pushButtonDoTemplateMatchingBinPickingClickedCallback() { // bin picking
//     taskTemplateMatchingParameter matching_parameter;
//     matching_parameter.debug_mode = true;
//     matching_parameter.is_base_frame_unknown = false;
//     matching_parameter.do_scan_sampling = ui->radioButton_assemblyUI_scanSampling->isChecked();
//     matching_parameter.sampling_num = ui->lineEdit_scanSamplingNum->text().toInt();
//     qnode_->m_bin_picking_node->doCADMatchingService(matching_parameter);
//     pushButtonSetPreDetectedPoseClickedCallback();
// }
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Robot calibration //////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
int MainWindow_control::OpenDialogCallback() {
    // cal_dialog cal_dialog(this, qnode_);

#ifndef CALIBRATION_SKIP
    cal_dialog_->show();
    return cal_dialog_->exec();
#endif
}

int MainWindow_control::OpenLLMDialog() {
    // llm_dialog_->show();
    // return llm_dialog_->exec();
    return 0;
}
// void MainWindow_control::OpenFolderCallback() {
//     QString folder_name = QFileDialog::getExistingDirectory(this, tr("Open folder"),"./");
//     // printf(filename);
//     ui->label_cal_folder_name->setText(folder_name);
// }

// void MainWindow_control::CalGenerateCallback() {
//     std::vector<double> cal_pose(6);
//     for (int i = 0; i < 6; i++){

//         if (list_cal_pose_[i]->text().isEmpty()){
//             list_cal_pose_[i]->setStyleSheet("font-size: 15px; background-color:rgb(255,200,200);");
//             break;
//         }
//         else{
//             cal_pose[i]  = list_cal_pose_[i]->text().toDouble();
//             list_cal_pose_[i]->setStyleSheet("font-size: 15px; background-color:rgb(255,255,255);");
//             }
//     int num_of_pose = ui->lineEdit_cal_num_of_pose->text().toInt();
//     }

//     // Connect to Qnode and Save the Pose.csv and Pose.ply in save folder
// }

// void MainWindow_control::CalibrateCallback(){
//     printf("\n");
//     printf("%s\n", ui->label_cal_folder_name->text().toStdString().c_str());
//     const char* folder_name = ui->label_cal_folder_name->text().toStdString().c_str();
// }
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


// // robot status
// void MainWindow_control::servo_on_button_clicked() {
//     bool enable_state = qnode_->getEnableState();
//     qnode_->setEnable(!enable_state);
// }

// void MainWindow_control::jsctrl_button_clicked() {
//     uint32_t command = JSCTRL_START;
//     qnode_->setCommand(command);
// }

// void MainWindow_control::csctrl_button_clicked() {
//     uint32_t command = CSCTRL_START;
//     qnode_->setCommand(command);
// }

// void MainWindow_control::friction_mode_button_clicked() {
//     uint32_t command = friction_mode_ ? FRICTION_MODEL : FRICTION_OBSERVER;
//     qnode_->setCommand(command);
//     friction_mode_ = !friction_mode_;
//     ROS_LOG_INFO("kcr_control_ui: %s %d", __FUNCTION__, command);
// }

// void MainWindow_control::handguide_button_clicked() {
//     uint32_t command = handguide_mode_ ? JS_HG_OFF : JS_HG_ON;
//     qnode_->setCommand(command);
//     handguide_mode_ = !handguide_mode_;
//     ROS_LOG_INFO("kcr_control_ui: %s %d", __FUNCTION__, command);
// }

// void MainWindow_control::collision_detection_button_clicked() {
//     uint32_t command = collision_detection_mode_ ? CD_OFF : CD_ON;
//     qnode_->setCommand(command);
//     collision_detection_mode_ = !collision_detection_mode_;
//     ROS_LOG_INFO("kcr_control_ui: %s %d", __FUNCTION__, command);
// }

// void MainWindow_control::impedance_button_clicked() {
//     uint32_t command = impedance_mode_ ? CS_IMPEDANCE_CTRL_OFF : CS_IMPEDANCE_CTRL_ON;
//     qnode_->setCommand(command);
//     impedance_mode_ = !impedance_mode_;
//     ROS_LOG_INFO("kcr_control_ui: %s %d", __FUNCTION__, command);
// }

// void MainWindow_control::brake_off_button_clicked() {
//     uint32_t command = BRAKE_OFF;
//     qnode_->setCommand(command);
//     ROS_LOG_INFO("kcr_control_ui: %s %d", __FUNCTION__, command);
// }

// void MainWindow_control::jts_bias_button_clicked() {
//     uint32_t command = JTS_BIAS;
//     qnode_->setCommand(command);
//     ROS_LOG_INFO("kcr_control_ui: %s %d", __FUNCTION__, command);
// }

// void MainWindow_control::error_clear_button_clicked() {
//     uint32_t command = CLEAR_MALFUNCTION;
//     qnode_->setCommand(command);
//     ROS_LOG_INFO("kcr_control_ui: %s %d", __FUNCTION__, command);
// }

// void MainWindow_control::home_pose_button_clicked() {
// #if JS_DOF == 6
//     vector<double> target_q = {0.0, 90.0, 0.0, 90.0, 0.0, 0.0};
// #elif JS_DOF == 7
//     vector<double> target_q = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// #endif
//     move_joint_space(target_q);
//     ROS_LOG_INFO("kcr_control_ui: %s", __FUNCTION__);
// }

// void MainWindow_control::task_pose_button_clicked() {
// #if JS_DOF == 6
//     vector<double> target_q = {0.0, 88.0, -110.0, 110.0, -90.0, 0.0};
// #elif JS_DOF == 7
//     vector<double> target_q = {0.0, -15.0, 0.0, -70.0, 0.0, -95.0, 0.0};
// #endif
//     move_joint_space(target_q);
//     ROS_LOG_INFO("kcr_control_ui: %s", __FUNCTION__);
// }

// void MainWindow_control::robot_stop_button_clicked() {
//     double stop_time = 0.2;
//     qnode_->stopRobot(stop_time);
//     ROS_LOG_INFO("kcr_control_ui: %s %lf", __FUNCTION__, stop_time);
// }

// void MainWindow_control::joint_angle_button_clicked(int index, int sign) {
//     vector<double> target_q = qnode_->getDesiredQ();
//     double step = ui->lineEditJointStep->text().toDouble() * sign;

//     target_q[index] += step;

//     move_joint_space(target_q);

//     ROS_LOG_INFO("kcr_control_ui: %s index: %d, step: %lf", __FUNCTION__, index, step);
// }

// void MainWindow_control::pose_button_clicked(int index, int sign) {
//     vector<double> target_x = qnode_->getDesiredX();
//     double step = ui->lineEditPositionStep->text().toDouble() * sign;
//     uint8_t frame_type = ui->comboBoxOrientation->currentIndex();

//     if (index > 2) {
//         step = ui->lineEditOrientationStep->text().toDouble() * sign;
//     }

//     if (frame_type == 1) {
//         // base
//         for (int i = 0; i < 3; i++) {
//             target_x[i+3] = 0.0;
//         }
//     } else if (frame_type == 2) {
//         // tool
//         for (int i = 0; i < 6; i++) {
//             target_x[i] = 0.0;
//         }
//     }

//     target_x[index] += step;

//     move_cartesian_space(target_x, frame_type);

//     ROS_LOG_INFO("kcr_control_ui: %s index: %d, step: %lf", __FUNCTION__, index, step);
// }

// void MainWindow_control::set_tcp_payload_button_clicked() {
//     vector<double> tcp = {
//         ui->doubleSpinBoxTcpX->value(),
//         ui->doubleSpinBoxTcpY->value(),
//         ui->doubleSpinBoxTcpZ->value(),
//         ui->doubleSpinBoxTcpRx->value(),
//         ui->doubleSpinBoxTcpRy->value(),
//         ui->doubleSpinBoxTcpRx->value()
//     };
//     double payload = ui->doubleSpinBoxPayload->value();

//     uint32_t error = qnode_->setTcpPayload(tcp, payload);

//     ROS_LOG_INFO("kcr_control_ui: %s", __FUNCTION__);

//     if (error) {
//         QString error_msg = "";
//         if (error & 0x01) {
//             error_msg += "tool center point size error\n";
//         }
//         if (error & 0x02) {
//             error_msg += "over the payload(3kg) error\n";
//         }
//         QMessageBox::warning(this, "KCR Control UI", error_msg);
//     }
// }

// void MainWindow_control::set_pid_button_clicked() {
//     int joint = ui->comboBoxPidJoint->currentIndex();
//     double p_gain = ui->lineEditPGain->text().toDouble();
//     double i_gain = ui->lineEditIGain->text().toDouble();
//     double d_gain = ui->lineEditDGain->text().toDouble();
//     double observer_gain = ui->lineEditObsGain->text().toDouble();

//     if (joint < JS_DOF) {
//         qnode_->setPidParameter(joint, p_gain, i_gain, d_gain, observer_gain);
//     }
// }

// void MainWindow_control::set_impedance_button_clicked() {
//     int axis = ui->comboBoxImpedanceAxis->currentIndex();

//     double p_gain = ui->lineEditPGain->text().toDouble();
//     double i_gain = ui->lineEditIGain->text().toDouble();
//     double d_gain = ui->lineEditDGain->text().toDouble();
//     double observer_gain = ui->lineEditObsGain->text().toDouble();

//     qnode_->setPidParameter(axis, p_gain, i_gain, d_gain, observer_gain);
// }

// void MainWindow_control::get_pid_button_clicked() {
//     int joint = ui->comboBoxPidJoint->currentIndex();

//     if (joint < JS_DOF) {
//         QTimer::singleShot(10, this, [ = ]() {
//             double p_gain = 0.0;
//             double i_gain = 0.0;
//             double d_gain = 0.0;
//             double observer_gain = 0.0;

//             qnode_->getPidParameter(joint, &p_gain, &i_gain, &d_gain, &observer_gain);

//             ui->lineEditPGain->setText(QString::number(p_gain, 'f', 2));
//             ui->lineEditIGain->setText(QString::number(i_gain, 'f', 2));
//             ui->lineEditDGain->setText(QString::number(d_gain, 'f', 2));
//             ui->lineEditObsGain->setText(QString::number(observer_gain, 'f', 2));
//         });
//     }
// }

// void MainWindow_control::get_impedance_button_clicked() {
//     int axis = ui->comboBoxImpedanceAxis->currentIndex();

//     QTimer::singleShot(10, this, [ = ]() {
//         double m_gain = 0.0;
//         double d_gain = 0.0;
//         double k_gain = 0.0;

//         qnode_->getImpedanceParameter(axis, &m_gain, &d_gain, &k_gain);

//         if (axis < 3) {
//             ui->lineEditMass->setText(QString::number(m_gain, 'f', 2));
//             ui->lineEditDamper->setText(QString::number(d_gain, 'f', 2));
//             ui->lineEditSpring->setText(QString::number(k_gain, 'f', 2));
//         } else {
//             ui->lineEditMass->setText(QString::number(m_gain, 'f', 3));
//             ui->lineEditDamper->setText(QString::number(d_gain, 'f', 3));
//             ui->lineEditSpring->setText(QString::number(k_gain, 'f', 3));
//         }
//     });
// }

// void MainWindow_control::move_joint_space(vector<double> target_q) {
//     double speed = ui->lineEditJointSpeed->text().toDouble();
//     double acc_time = ui->lineEditJointAccTime->text().toDouble();
//     double max_velocity = kQdDefault * speed / 100.0;
//     double acceleration = max_velocity / acc_time;

//     uint32_t error = qnode_->setTargetJointPosition(target_q, max_velocity, acceleration);

//     ROS_LOG_INFO("kcr_control_ui: %s %d", __FUNCTION__, error);

//     if (error) {
//         QString error_msg = "";
//         if (error & 0x01) {
//             error_msg += "target joint position size error\n";
//         }
//         if (error & 0x02) {
//             error_msg += "velocity value error\n";
//         }
//         if (error & 0x04) {
//             error_msg += "acceleration value error\n";
//         }
//         if (error & 0x08) {
//             error_msg += "enable first\n";
//         }
//         if (error & (0x10 | 0x20)) {
//             error_msg += "clear error first\n";
//         }

//         QMessageBox::warning(this, "KCR Control UI", error_msg);
//     }
// }

// void MainWindow_control::move_cartesian_space(vector<double> target_x, uint8_t frame_type) {
//     double speed = ui->lineEditSpeed->text().toDouble();
//     double acc_time = ui->lineEditAccTime->text().toDouble();
//     double max_velocity = kXdDefault * speed / 100.0;
//     double acceleration = max_velocity / acc_time;

//     uint32_t error = qnode_->setTargetPose(target_x, max_velocity, acceleration, frame_type);

//     ROS_LOG_INFO("kcr_control_ui: %s %d", __FUNCTION__, error);

//     if (error) {
//         QString error_msg = "";
//         if (error & 0x01) {
//             error_msg += "target pose position size error\n";
//         }
//         if (error & 0x02) {
//             error_msg += "velocity value error\n";
//         }
//         if (error & 0x04) {
//             error_msg += "acceleration value error\n";
//         }
//         if (error & 0x08) {
//             error_msg += "enable first\n";
//         }
//         if (error & (0x10 | 0x20)) {
//             error_msg += "clear error first\n";
//         }
//         QMessageBox::warning(this, "KCR Control UI", error_msg);
//     }
// }