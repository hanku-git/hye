#include "llm_control/dialog/llm_dialog.hpp"
#include "ui_llm_dialog.h"

llm_dialog::llm_dialog(QMainWindow *parent, QNode *qnode_)
    : QDialog(parent)
    , llm_ui(new Ui::llm_dialog)
{
    qnode_llm = qnode_;
    llm_ui->setupUi(this);

    grp_home_pose_ = {-137.618, -122.956, 118.660, -173.172, -43.526, -0.261};
    home_pose_ = {-105.007, -111.778, 127.924, -106.086, -90.272, -105.255};
    mid_pose_ = {-187.328, -129.881, 129.327, -89.363, -90.314, -97.518};
    scan_pose_ = {-89.925, -142.042, 133.435, -81.286, -90.334, -90.083};
    tool_change_vel = llm_ui->lineEdit_velocity_1->text().toInt();
    grasping_vel = llm_ui -> lineEdit_velocity_2->text().toInt();
    passing_vel = llm_ui -> lineEdit_velocity_3->text().toInt();
    loading_vel = llm_ui -> lineEdit_velocity_4->text().toInt();

    target_object_list  << llm_ui->radioButton_striker
                        << llm_ui->radioButton_scu
                        << llm_ui->radioButton_grill
                        << llm_ui->radioButton_hinge
                        << llm_ui->radioButton_bolt;

    waypoint_list << llm_ui->lineEdit_A_x << llm_ui->lineEdit_A_y << llm_ui->lineEdit_A_z
                  << llm_ui->lineEdit_B_x << llm_ui->lineEdit_B_y << llm_ui->lineEdit_B_z
                  << llm_ui->lineEdit_C_x << llm_ui->lineEdit_C_y << llm_ui->lineEdit_C_z;

    pass_list << llm_ui->lineEdit_pass_1 << llm_ui->lineEdit_pass_2 << llm_ui->lineEdit_pass_3;

	connect(llm_ui->pushButton_recording, &QPushButton::clicked, this,
            &llm_dialog::RecordingCallback);

    connect(llm_ui->pushButton_home, &QPushButton::clicked, this,
            std::bind(&llm_dialog::SetPoseCallback, this, "home"));
    connect(llm_ui->pushButton_gr_home, &QPushButton::clicked, this,
            std::bind(&llm_dialog::SetPoseCallback, this, "grp"));
    connect(llm_ui->pushButton_mid, &QPushButton::clicked, this,
            std::bind(&llm_dialog::SetPoseCallback, this, "mid"));
    connect(llm_ui->pushButton_scaning_pose, &QPushButton::clicked, this,
            std::bind(&llm_dialog::SetPoseCallback, this, "scan"));

    connect(llm_ui->radioButton_striker, &QRadioButton::clicked, this,
            &llm_dialog::target_obj_Callback);
    connect(llm_ui->radioButton_scu, &QRadioButton::clicked, this,
            &llm_dialog::target_obj_Callback);
    connect(llm_ui->radioButton_grill, &QRadioButton::clicked, this,
            &llm_dialog::target_obj_Callback);
    connect(llm_ui->radioButton_hinge, &QRadioButton::clicked, this,
            &llm_dialog::target_obj_Callback);
    connect(llm_ui->radioButton_bolt, &QRadioButton::clicked, this,
            &llm_dialog::target_obj_Callback);

	connect(llm_ui->pushButton_test1_in, &QPushButton::clicked, this,
            &llm_dialog::GripperAttachCallback);
    connect(llm_ui->pushButton_test1_out, &QPushButton::clicked, this,
            &llm_dialog::GripperDetachCallback);
    connect(llm_ui->pushButton_test2_in, &QPushButton::clicked, this,
            &llm_dialog::GripperAttachCallback);
    connect(llm_ui->pushButton_test2_out, &QPushButton::clicked, this,
             &llm_dialog::GripperDetachCallback);
    connect(llm_ui->pushButton_test3_in, &QPushButton::clicked, this,
            &llm_dialog::GripperAttachCallback);
    connect(llm_ui->pushButton_test3_out, &QPushButton::clicked, this,
             &llm_dialog::GripperDetachCallback);
    connect(llm_ui->pushButton_tip1_in, &QPushButton::clicked, this,
            &llm_dialog::GripperAttachCallback);
    connect(llm_ui->pushButton_tip1_out, &QPushButton::clicked, this,
            &llm_dialog::GripperDetachCallback);
    connect(llm_ui->pushButton_tip2_in, &QPushButton::clicked, this,
            &llm_dialog::GripperAttachCallback);
    connect(llm_ui->pushButton_tip2_out, &QPushButton::clicked, this,
            &llm_dialog::GripperDetachCallback);

    connect(llm_ui->pushButton_pal, &QPushButton::clicked, this,
             &llm_dialog::BoxSelectionCallback);
    connect(llm_ui->pushButton_blue, &QPushButton::clicked, this,
             &llm_dialog::BoxSelectionCallback);
    connect(llm_ui->pushButton_yellow, &QPushButton::clicked, this,
             &llm_dialog::BoxSelectionCallback);

    connect(qnode_llm, &QNode::UpdateLLMText, this, &llm_dialog::updateLLMText);
    connect(qnode_llm, &QNode::UpdateVelocites, this, &llm_dialog::UpdateVelocites);
    connect(qnode_llm, &QNode::UpdateImage, this, &llm_dialog::onUpdateImage);
    connect(qnode_llm, &QNode::UpdateWaypoints, this, &llm_dialog::onUpdate3DCoordinates);

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &llm_dialog::timerCallback);
    timer_->start(100);
}

llm_dialog::~llm_dialog()
{
	rclcpp::shutdown();
    delete llm_ui;
}

// qnode watchdog, 작업 진행 및 flag 관리
void llm_dialog::timerCallback() {
    if (is_placement_task && qnode_llm->task_param_.is_robot_move_fin) {
        is_placement_task = false;
        GraspingTargetObject();
    }
    else if (qnode_llm->params_.llm_param.llm_text_received) {
        llm_node_Callback(qnode_llm->params_.llm_param);
        qnode_llm->params_.llm_param.llm_text_received = false;
    }
}

void llm_dialog::RecordingCallback() {
    qnode_llm->LLMRecording();
}

// UI pushbutton 입력으로 저장된 Pose로 이동
void llm_dialog::SetPoseCallback(const std::string& position) {
    double qd = 40;
    double qdd = 80;
    if      (position == "home"){qnode_llm->moveQ(home_pose_, qd, qdd, false);}
    else if (position == "grp") {qnode_llm->moveQ(grp_home_pose_, qd, qdd, false);}
    else if (position == "mid") {qnode_llm->moveQ(mid_pose_, qd, qdd, false);}
    else if (position == "scan"){qnode_llm->moveQ(scan_pose_, qd, qdd, false);}
}

// LLM node 입력에 따라 UI radio button
void llm_dialog::llm_node_Callback(LLM_PARAM &llm_param) {
    try {
        if (llm_param.task == "placement") {
            if (llm_param.target == "striker") {
                cout << "LLM node input: grasping striker" << endl;
                llm_ui->radioButton_striker->setChecked(true);
            }
            else if (llm_param.target == "scu") {
                cout << "LLM node input: grasping SCU" << endl;
            llm_ui->radioButton_scu->setChecked(true);
            }
            else if (llm_param.target == "grill") {
                cout << "LLM node input: grasping grill" << endl;
                llm_ui->radioButton_grill->setChecked(true);
            }
            else if (llm_param.target == "hinge") {
                cout << "LLM node input: grasping hinge" << endl;
                llm_ui->radioButton_hinge->setChecked(true);
            }
            else if (llm_param.target == "bolt") {
                cout << "LLM node input: grasping bolt" << endl;
                llm_ui->radioButton_bolt->setChecked(true);
            }
            is_placement_task = true;
            target_obj_Callback();
        }
        else if (llm_param.task == "velocity") {
            tool_change_vel = llm_ui->lineEdit_velocity_1->text().toInt();
            grasping_vel = llm_ui -> lineEdit_velocity_2->text().toInt();
            passing_vel = llm_ui -> lineEdit_velocity_3->text().toInt();
            loading_vel = llm_ui -> lineEdit_velocity_4->text().toInt();
        }
        else if (llm_param.task == "move") {
            std::vector<double> pose{0, 0, 0, 0, 0, 0};
            double xd  = 0.1;
            double xdd = 0.2;
            pose[stoi(llm_param.target)] = stod(llm_param.goal);
            CsDouble x;
            vec2Arr(pose, x);
            qnode_llm->moveX(x, xd, xdd, false, true);
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in llm_node_Callback: " << e.what() << std::endl;
    }
}

// UI Radio button 상태에 따른 object prameter setting 및 gripper setting
void llm_dialog::target_obj_Callback() {
    int new_gripper = 1;
    int new_tip = 1;

    // 현재 선택된 객체에 따라 새로운 객체를 설정
    if (llm_ui->radioButton_striker->isChecked()) {
        target_name = "striker";
        current_obj = 39;
        new_gripper = 1; // 2지 그리퍼
        new_tip = 1;
    } else if (llm_ui->radioButton_scu->isChecked()) {
        target_name = "scu";
        current_obj = 40;
        new_gripper = 2; // 흡착 그리퍼
    } else if (llm_ui->radioButton_grill->isChecked()) {
        target_name = "grill";
        current_obj = 41;
        new_gripper = 1; // 2지 그리퍼
        new_tip = 1;
    } else if (llm_ui->radioButton_bolt->isChecked()) {
        target_name = "bolt";
        current_obj = 37;
        new_gripper = 1; // 2지 그리퍼
        new_tip = 2;    // bolt tip
    } else if (llm_ui->radioButton_hinge->isChecked()) {
        target_name = "hinge";
        current_obj = 38;
        new_gripper = 1; // 2지 그리퍼
        new_tip = 1;
    } else {
        std::cerr << "Unknown target object." << std::endl;
        return;
    }
    std::cout << "Target object: " << target_name << ", new gripper: " << new_gripper << std::endl;

    qnode_llm->current_task_list_.clear();
    // 현재 장착된 그리퍼와 새로운 그리퍼가 다른 경우 그리퍼를 변경
    if (current_gripper != new_gripper) {

        std::cout << "Detaching current gripper: " << current_gripper << std::endl;
        qnode_llm->task_planner_.LLMDetachGripper(current_gripper, tool_change_vel);
        qnode_llm->current_task_list_.insert(qnode_llm->current_task_list_.end(), qnode_llm->task_planner_.llm_task_detach_.begin(), qnode_llm->task_planner_.llm_task_detach_.end());

        std::cout << "Attaching new gripper: " << new_gripper << std::endl;
        qnode_llm->task_planner_.LLMAttachGripper(new_gripper, tool_change_vel);
        qnode_llm->current_task_list_.insert(qnode_llm->current_task_list_.end(), qnode_llm->task_planner_.llm_task_attach_.begin(), qnode_llm->task_planner_.llm_task_attach_.end());

        std::cout << "Moving to gripper home position." << std::endl;
        qnode_llm->task_planner_.taskPushBack_jsMove(qnode_llm->current_task_list_, grp_home_pose_, false);
        current_gripper = new_gripper;
    }
    // 현재 장착된 팁과 새로운 팁이 다른 경우 팁을 변경
    if (current_tip != new_tip && new_gripper == 1) {
        std::cout << "Detaching current tip: " << current_tip << std::endl;
        qnode_llm->task_planner_.LLMDetachTip(current_tip, tool_change_vel);
        qnode_llm->current_task_list_.insert(
            qnode_llm->current_task_list_.end(), qnode_llm->task_planner_.llm_task_detach_tip_.begin(), qnode_llm->task_planner_.llm_task_detach_tip_.end());

        std::cout << "Attaching current tip: " << current_tip << std::endl;
        qnode_llm->task_planner_.LLMAttachTip(new_tip, tool_change_vel);
        qnode_llm->current_task_list_.insert(
            qnode_llm->current_task_list_.end(), qnode_llm->task_planner_.llm_task_attach_tip_.begin(), qnode_llm->task_planner_.llm_task_attach_tip_.end());

        std::cout << "Moving to gripper home position." << std::endl;
        qnode_llm->task_planner_.taskPushBack_jsMove(qnode_llm->current_task_list_, grp_home_pose_, false);
        current_tip = new_tip;
    }
    // 홈 위치로 이동 (세팅 후 기본 위치로 돌아가기 위해)
    qnode_llm->task_planner_.taskPushBack_jsMove(qnode_llm->current_task_list_, home_pose_, false);
    qnode_llm->beforeTaskStart();
}

// object parameter setting 후 kr_sys process 진행
void llm_dialog::GraspingTargetObject() {
    TargetObjectParamSetting(); // scan/matching 위한 object parameter setting
    vector<vector<double>> waypoints;
    qnode_llm->current_task_list_.clear();

    ////////////////////////////1. Matchig&Grasping////////////////////////////////

    qnode_llm->task_planner_.LLMScanMatchingGrasping(current_obj, grasping_vel);
    qnode_llm->current_task_list_.insert(qnode_llm->current_task_list_.end(), qnode_llm->task_planner_.llm_task_zivid_.begin(), qnode_llm->task_planner_.llm_task_zivid_.end());

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
        qnode_llm->task_planner_.LLMMoveWayPoints(waypoints, passing_vel);
        qnode_llm->current_task_list_.insert(qnode_llm->current_task_list_.end(), qnode_llm->task_planner_.llm_task_waypoints_.begin(), qnode_llm->task_planner_.llm_task_waypoints_.end());
    }

    ////////////////////////////3. loading color box////////////////////////////////
    int boxNumber = 2; // Yellow box
    int palette_num = 5; // palette center
    string placement_goal = qnode_llm->params_.llm_param.goal;

    if(placement_goal == "blue"){
        cout << "LLM node input: move to blue box" << endl;
        boxNumber = 1;
    }
    else if(placement_goal == "yellow"){
        cout << "LLM node input: move to yellow box" << endl;
        boxNumber = 2;
    }
    else if(placement_goal.substr(0,7) == "palette") {
        boxNumber = 3;
        palette_num = stoi(placement_goal.substr(8));
        cout << "LLM node input: move to palette_" << palette_num << endl;
    }
    qnode_llm->task_planner_.LLMMoveBox(boxNumber, 1, current_obj, loading_vel, palette_num);
    qnode_llm->current_task_list_.insert(qnode_llm->current_task_list_.end(), qnode_llm->task_planner_.llm_task_box_.begin(), qnode_llm->task_planner_.llm_task_box_.end());

    qnode_llm->beforeTaskStart();
}

void llm_dialog::GripperAttachCallback() {
    QPushButton* button = qobject_cast<QPushButton*>(sender());
    int gripper_type = 0; // 기본값
    int tip_type = 0;

    if (button == llm_ui->pushButton_test1_in) {
        gripper_type = 1;
        tip_type = 0;
    } else if (button == llm_ui->pushButton_test2_in) {
        gripper_type = 2;
        tip_type = 0;
    } else if (button == llm_ui->pushButton_test3_in) {
        gripper_type = 3;
        tip_type = 0;
    } else if (button == llm_ui->pushButton_tip1_in) {
        gripper_type = 0;
        tip_type = 1;
    } else if (button == llm_ui->pushButton_tip2_in) {
        gripper_type = 0;
        tip_type = 2;
    } else {
        std::cerr << "Unknown button pressed." << std::endl;
    }
    qnode_llm->task_planner_.LLMAttachGripper(gripper_type, tool_change_vel);
    qnode_llm->task_planner_.LLMAttachTip(tip_type, tool_change_vel);
    qnode_llm->current_task_list_ = qnode_llm->task_planner_.llm_task_attach_;
    qnode_llm->current_task_list_.insert(qnode_llm->current_task_list_.end(), qnode_llm->task_planner_.llm_task_attach_tip_.begin(), qnode_llm->task_planner_.llm_task_attach_tip_.end());

    qnode_llm->beforeTaskStart();
}

void llm_dialog::GripperDetachCallback() {
    QPushButton* button = qobject_cast<QPushButton*>(sender());
    int gripper_type = 0; // 기본값
    int tip_type = 0;

    if (button == llm_ui->pushButton_test1_out) {
        gripper_type = 1;
        tip_type = 0;
    } else if (button == llm_ui->pushButton_test2_out) {
        gripper_type = 2;
        tip_type = 0;
    } else if (button == llm_ui->pushButton_test3_out) {
        gripper_type = 3;
        tip_type = 0;
    } else if (button == llm_ui->pushButton_tip1_out) {
        gripper_type = 0;
        tip_type = 1;
    } else if (button == llm_ui->pushButton_tip2_out) {
        gripper_type = 0;
        tip_type = 2;
    } else {
        std::cerr << "Unknown button pressed." << std::endl;
    }

    qnode_llm->task_planner_.LLMDetachTip(tip_type, tool_change_vel);
    qnode_llm->task_planner_.LLMDetachGripper(gripper_type, tool_change_vel);
    qnode_llm->current_task_list_ = qnode_llm->task_planner_.llm_task_detach_;
    qnode_llm->current_task_list_.insert(qnode_llm->current_task_list_.end(), qnode_llm->task_planner_.llm_task_detach_tip_.begin(), qnode_llm->task_planner_.llm_task_detach_tip_.end());

    qnode_llm->beforeTaskStart();
}

void llm_dialog::BoxSelectionCallback() {
    QPushButton* button = qobject_cast<QPushButton*>(sender());
    int boxNumber = 1; // 기본값
    int palette_num = 5;

    if (button == llm_ui->pushButton_blue) {
        boxNumber = 1;
    } else if (button == llm_ui->pushButton_yellow) {
        boxNumber = 2;
    } else if (button == llm_ui->pushButton_pal) {
        boxNumber = 3;
    } else {
        std::cerr << "Unknown button pressed. Defaulting to red box." << std::endl;
    }

    qnode_llm->task_planner_.LLMMoveBox(boxNumber, 1, current_obj, loading_vel, palette_num);
    qnode_llm->current_task_list_ = qnode_llm->task_planner_.llm_task_box_;
    qnode_llm->beforeTaskStart();
}

void llm_dialog::onUpdateImage(const QImage &image, const QString &current_pass) {
    if (current_pass == "A") {
        llm_ui->label_pass_A->setPixmap(QPixmap::fromImage(image).scaled(llm_ui->label_pass_A->size(), Qt::KeepAspectRatio));
    } else if (current_pass == "B") {
        llm_ui->label_pass_B->setPixmap(QPixmap::fromImage(image).scaled(llm_ui->label_pass_B->size(), Qt::KeepAspectRatio));
    } else if (current_pass == "C") {
        llm_ui->label_pass_C->setPixmap(QPixmap::fromImage(image).scaled(llm_ui->label_pass_C->size(), Qt::KeepAspectRatio));
    }
}

void llm_dialog::onUpdate3DCoordinates(const QString &coordinates,  const QString &current_pass) {
    QStringList coordList = coordinates.split(",");
    if (coordList.size() == 3) {
        int idx_ = (current_pass == "A") ? 0 : (current_pass == "B") ? 3 : (current_pass == "C") ? 6 : 9;
        waypoint_list[idx_] -> setText(QString::number(coordList[0].split(":")[1].trimmed().toFloat()));
        waypoint_list[idx_+1] -> setText(QString::number(coordList[1].split(":")[1].trimmed().toFloat()));
        waypoint_list[idx_+2] -> setText(QString::number(coordList[2].split(":")[1].trimmed().toFloat()));
    }
}

void llm_dialog::updateLLMText(const QString &target, const QString &goal, const QString &passes) {
    llm_ui->lineEdit_target->setText(target);
    llm_ui->lineEdit_goal->setText(goal);
    // llm_ui->lineEdit_passPoint->setText(passes);

    if (passes.size()>0){
        QStringList qpasslist = passes.split(" ");
        for (int i = 0; i < qpasslist.size(); ++i){
            pass_list[i]->setText(qpasslist[i]);
        }
    } else {
        for (int i = 0; i < 3; ++i){
            pass_list[i]->setText(" ");
        }
    }
}

void llm_dialog::UpdateVelocites(const QString &velocities) {
    QStringList velocityList = velocities.split(" ");
    if (velocityList.size() >= 4) {
        llm_ui->lineEdit_velocity_1->setText(velocityList[0]);
        llm_ui->lineEdit_velocity_2->setText(velocityList[1]);
        llm_ui->lineEdit_velocity_3->setText(velocityList[2]);
        llm_ui->lineEdit_velocity_4->setText(velocityList[3]);
    } else {
        llm_ui->lineEdit_velocity_1->setText("3");
        llm_ui->lineEdit_velocity_2->setText("3");
        llm_ui->lineEdit_velocity_3->setText("3");
        llm_ui->lineEdit_velocity_4->setText("3");
    }
}

// Bin picking parameter setting each object
void llm_dialog::TargetObjectParamSetting() {
    double voxel_downsampling_size = 5;

    size_t target_object_idx = 0;
    for(int i=0; i<target_object_list.size(); i++) {
        if(target_object_list[i]->isChecked()) { target_object_idx = i; }
    }

    qnode_llm->m_bin_picking_node->current_target_object_idx_ = target_object_idx;
    qnode_llm->m_bin_picking_node->before_target_object_idx_ = target_object_idx;

    CTARGET_OBJECT_DATA* ptr_object_now = qnode_llm->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_llm->m_bin_picking_node->current_target_object_idx_];

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

            qnode_llm->grp_do_initialize_ = true;
            qnode_llm->grp_is_vacuum_ = false;
            qnode_llm->grp_is_vacuum_before_ = false;
            qnode_llm->grp_init_2_grp_cmd_ = 4953; // for initialize_2

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
            qnode_llm->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            //// [2F 그리퍼] - before
            if(qnode_llm->is_tcp_initialized) qnode_llm->sendChangeRobotTCPCommand(13); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 13;

            // //// [2F 그리퍼] - new short version
            // if(qnode_llm->is_tcp_initialized) qnode_llm->sendChangeRobotTCPCommand(18); // TCP #18(tip changing - round tip)
            // ptr_object_now->tcp_changing_id = 18;


            //// Tool changing slave id
            ptr_object_now->tool_changing_attach_id = 2; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 2; // # slave #1, 2, ...

            qnode_llm->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            qnode_llm->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = true;
            ptr_object_now->tip_changing_attach_id = 2; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 2; // tip #1, 2, ...
            qnode_llm->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            qnode_llm->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            //// [흡착 그리퍼]
            // if(qnode_llm->is_tcp_initialized) qnode_llm->sendChangeRobotTCPCommand(16); // TCP #11, square peg vacuum
            // //// Tool changing slave id
            // qnode_llm->tool_changing_attach_id = 4; // # slave #1, 2, ...
            // qnode_llm->tool_changing_detach_id = 4; // # slave #1, 2, ...


            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            qnode_llm->test_cnt_ = 0;

            std::vector<double> stacking_trans_scale = {0.225, 0.050, 0.045}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	ptr_object_now->stacking_single_stack_num_ = 4; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 1; // stacking_single_stack_num_만큼 놓는 선의 개수
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::CYLINDER_STACKING;

            //// 1단의 실린더 개수
            ptr_object_now->stack_part_cnt_ = ((ptr_object_now->stacking_single_stack_num_ * (ptr_object_now->stacking_single_stack_num_ + 1)) / 2) * ptr_object_now->stacking_line_stack_num_;
            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = ptr_object_now->stack_part_cnt_; // 한 단의 실린더 스택, ex) 한 개의 단은 10개씩, 두 단은 총 20개

            qnode_llm->m_bin_picking_node->is_task_in_progress = false;

            //// Scanning sampling number
            ptr_object_now->m_scan_parameter.sampling_num = 4;
            ptr_object_now->m_matching_parameter.sampling_num = 4;
        }
        break;

        case TargetObject::OBJECT_SCU :
        {

            ROS_LOG_WARN("Target grasping object: SCU!");

            gripper_close_length = 9000;

            qnode_llm->grp_do_initialize_ = false;
            qnode_llm->grp_is_vacuum_ = true;
            qnode_llm->grp_is_vacuum_before_ = true;
            qnode_llm->grp_init_2_grp_cmd_ = 4953; // for initialize_2

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
            qnode_llm->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            if(qnode_llm->is_tcp_initialized) qnode_llm->sendChangeRobotTCPCommand(11); // TCP #11, square peg vacuum
            ptr_object_now->tcp_changing_id = 11;

            //// Tool changing slave id
            // qnode_llm->is_tool_attached = false;
            ptr_object_now->tool_changing_attach_id = 1; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 1; // # slave #1, 2, ...

            qnode_llm->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            qnode_llm->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = false;
            ptr_object_now->tip_changing_attach_id = 1; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 1; // tip #1, 2, ...
            qnode_llm->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            qnode_llm->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            qnode_llm->test_cnt_ = 0;

            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = 2;

            // std::vector<double> stacking_trans_scale = {0.0, 0.14, 0.082}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	// ptr_object_now->stacking_single_stack_num_ = 2; // 1개씩 놓는 진행 방향의 물체 개수
	        // ptr_object_now->stacking_line_stack_num_ = 1; // stacking_single_stack_num_만큼 놓는 선의 개수
            // ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            // ptr_object_now->stacking_mode_ = StackingType::GRID_STACKING;

            qnode_llm->m_bin_picking_node->is_task_in_progress = false;

            //// Scanning sampling number
            ptr_object_now->m_scan_parameter.sampling_num = 8;
            ptr_object_now->m_matching_parameter.sampling_num = 8;
        }
        break;

        case TargetObject::OBJECT_GRILL :
        {
            ROS_LOG_WARN("Target grasping object: Grill!");
            gripper_close_length = 5000;

            qnode_llm->grp_do_initialize_ = true;
            qnode_llm->grp_is_vacuum_ = false;
            qnode_llm->grp_is_vacuum_before_ = false;
            qnode_llm->grp_init_2_grp_cmd_ = 4953; // for initialize_2

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
            qnode_llm->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            if(qnode_llm->is_tcp_initialized) qnode_llm->sendChangeRobotTCPCommand(13); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 13;

            //// Tool changing slave id
            // qnode_llm->is_tool_attached = false;
            ptr_object_now->tool_changing_attach_id = 2; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 2; // # slave #1, 2, ...

            qnode_llm->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            qnode_llm->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = false;
            ptr_object_now->tip_changing_attach_id = 2; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 2; // tip #1, 2, ...
            qnode_llm->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            qnode_llm->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            qnode_llm->test_cnt_ = 0;

            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = 3;

            std::vector<double> stacking_trans_scale = {0.0, 0.100, 0.02}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	ptr_object_now->stacking_single_stack_num_ = 3; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 1; // stacking_single_stack_num_만큼 놓는 선의 개수
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::GRID_STACKING;

            qnode_llm->m_bin_picking_node->is_task_in_progress = false;

            //// Scanning sampling number
            ptr_object_now->m_scan_parameter.sampling_num = 4;
            ptr_object_now->m_matching_parameter.sampling_num = 4;
        }
        break;

        case TargetObject::OBJECT_BOLT :
        {
            ROS_LOG_WARN("Target grasping object: Welding T joint (2F Gripper)!");

            gripper_close_length = 1000;

            qnode_llm->grp_do_initialize_ = true;
            qnode_llm->grp_is_vacuum_ = false;
            qnode_llm->grp_is_vacuum_before_ = false;
            qnode_llm->grp_init_2_grp_cmd_ = 4953; // for initialize_2

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
            qnode_llm->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            //// 2F gripper - before
            if(qnode_llm->is_tcp_initialized) qnode_llm->sendChangeRobotTCPCommand(12); // TCP #3
            ptr_object_now->tcp_changing_id = 12;

            // 2F gripper - new short version
            // if(qnode_llm->is_tcp_initialized) qnode_llm->sendChangeRobotTCPCommand(17); // TCP #17
            // ptr_object_now->tcp_changing_id = 17;


            //// Vacuum 1cup
            // if(qnode_llm->is_tcp_initialized) qnode_llm->sendChangeRobotTCPCommand(15); // TCP #15

            //// Tool changing slave id
            // qnode_llm->is_tool_attached = false;
            ptr_object_now->tool_changing_attach_id = 2; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 2; // # slave #1, 2, ...

            qnode_llm->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            qnode_llm->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = true;
            ptr_object_now->tip_changing_attach_id = 1; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 1; // tip #1, 2, ...
            qnode_llm->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            qnode_llm->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            qnode_llm->test_cnt_ = 0;

            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = 2;

            std::vector<double> stacking_trans_scale = {0.0, 0.125, 0.082}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	ptr_object_now->stacking_single_stack_num_ = 2; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 1; // stacking_single_stack_num_만큼 놓는 선의 개수
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::GRID_STACKING;

            qnode_llm->m_bin_picking_node->is_task_in_progress = false;

            //// Scanning sampling number
            ptr_object_now->m_scan_parameter.sampling_num = 1;
            ptr_object_now->m_matching_parameter.sampling_num = 1;
        }
        break;

        case TargetObject::OBJECT_HINGE :
        {
            ROS_LOG_WARN("Target grasping object: Welding hinge!");

            gripper_close_length = 2000;

            qnode_llm->grp_do_initialize_ = true;
            qnode_llm->grp_is_vacuum_ = false;
            qnode_llm->grp_is_vacuum_before_ = false;
            qnode_llm->grp_init_2_grp_cmd_ = 4953; // for initialize_2

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
            qnode_llm->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            if(qnode_llm->is_tcp_initialized) qnode_llm->sendChangeRobotTCPCommand(13); // TCP #2
            ptr_object_now->tcp_changing_id = 13;

            //// Tool changing slave id
            // qnode_llm->is_tool_attached = false;
            ptr_object_now->tool_changing_attach_id = 2; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 2; // # slave #1, 2, ...

            qnode_llm->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            qnode_llm->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = false;
            ptr_object_now->tip_changing_attach_id = 2; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 2; // tip #1, 2, ...
            qnode_llm->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            qnode_llm->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            qnode_llm->test_cnt_ = 0;

            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = 9;

            std::vector<double> stacking_trans_scale = {0.14, 0.12, 0.082}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	ptr_object_now->stacking_single_stack_num_ = 2; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 3; // stacking_single_stack_num_만큼 놓는 선의 개수
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::GRID_STACKING;

            qnode_llm->m_bin_picking_node->is_task_in_progress = false;

            //// Scanning sampling number
            ptr_object_now->m_scan_parameter.sampling_num = 2;
            ptr_object_now->m_matching_parameter.sampling_num = 2;

        }
        break;

    }

    //// 초기 TCP는 tool master 기준
    if(!qnode_llm->is_tcp_initialized) qnode_llm->sendChangeRobotTCPCommand(7);; // TCP #7 // Robot end-effector
    // if(!qnode_llm->is_tcp_initialized) qnode_llm->sendChangeRobotTCPCommand(1); // TCP #1, 2지그리퍼
    // if(!qnode_llm->is_tcp_initialized) qnode_llm->sendChangeRobotTCPCommand(8); // TCP #8 --> Robot Calibration
    // if(!qnode_llm->is_tcp_initialized) qnode_llm->sendChangeRobotTCPCommand(9); // TCP #9 --> (0.0, 0.0, 0.0)
    // if(!qnode_llm->is_tcp_initialized) qnode_llm->sendChangeRobotTCPCommand(10); // TCP #10, Connector gripper

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
    if(qnode_llm->is_task_generated_) {
        // ui.pushButton_doTask_binPicking->setEnabled(true);
        // ui.pushButton_doTask_binPicking_sequentialDemo->setEnabled(true);
    }
}
