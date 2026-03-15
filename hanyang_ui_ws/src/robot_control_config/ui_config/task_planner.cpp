#include "task_planner.hpp"
#include <iostream>

TaskPlanner::TaskPlanner() : TaskPlannerDefault() {
    current_directory_ = std::filesystem::current_path();

    void_task_list_.clear();

    // void task
    ref_unit_task_.task_mode = TASK_DEFAULT;
    void_task_list_.push_back(ref_unit_task_);

    setPosePosition();
}

TaskPlanner::~TaskPlanner() {

}

void TaskPlanner::setPosePosition() {
    setPosePositionDefault();

    // Task pose/position setting
    task_q_default_ = kQTask;
    task_x_default_ = kXTask;
}

bool TaskPlanner::makeTaskDefaultSetup() {

    return true;
}

bool TaskPlanner::makeTaskList() {
    makeTaskListDefault();

    module_task_test1_.clear();
    module_task_test2_.clear();
    module_task_test3_.clear();
    module_task_test4_.clear();
    module_task_test5_.clear();

    // plc
    // 영점 킬때 --> XG 5000 화면 -> 접속 -> 모니터 모드 켜기 -> M00101 더블클릭 켰다 끄기 -> 모니터모드 끄기
    // taskPushBackPLCModbusCmdWriteRegister(module_task_test1_, 13, 500);
    //set position 12
    // taskPushBackPLCModbusCmdWriteRegister(module_task_test1_, 12, 300);
    // taskPushBack_delay_1ms(module_task_test1_, 100);
    // taskPushBackPLCModbusCmdWriteRegister(module_task_test1_, 12, 0);


    CsDouble tcp_default  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // tip change grp

    JsDouble task_position_initial_position_change = {-90.000, 110.000, -210.000, 0.000, -80.000, 0.000};
    JsDouble task_position_middle_position_change = {0.000, 106.716, -200.916, 0.145, -85.800, 0.085};
    JsDouble task_position_tip_change = {6.906, 93.257, -133.665, -59.803, -127.052, -38.632};

    CsDouble task_pose_tip_change_right;
    CsDouble task_pose_tip_change_center;
    CsDouble task_pose_tip_change_left;

    task_pose_tip_change_right  = {0.42384, -0.26246, 0.78601, -136.92125, -2.76712, -177.08};
    task_pose_tip_change_center = {0.32764, -0.26298, 0.78789, -136.92125, -2.76712, -177.08};
    task_pose_tip_change_left   = {0.22676, -0.25593, 0.78750, -137.12123, -0.76715, -177.08};

    CsDouble task_pose_tip_change_right_offset_y  = arraySumCS(task_pose_tip_change_right, {0, 0.05, 0, 0, 0, 0});
    CsDouble task_pose_tip_change_center_offset_y = arraySumCS(task_pose_tip_change_center, {0, 0.05, 0, 0, 0, 0});
    CsDouble task_pose_tip_change_left_offset_y   = arraySumCS(task_pose_tip_change_left, {0, 0.05, 0, 0, 0, 0});

    CsDouble task_pose_tip_change_right_up  = arraySumCS(task_pose_tip_change_right, {0, 0, 0.06, 0, 0, 0});
    CsDouble task_pose_tip_change_center_up = arraySumCS(task_pose_tip_change_center, {0, 0, 0.06, 0, 0, 0});
    CsDouble task_pose_tip_change_left_up   = arraySumCS(task_pose_tip_change_left, {0, 0, 0.06, 0, 0, 0});

    CsDouble task_pose_tip_change_right_up_offset_y  = arraySumCS(task_pose_tip_change_right_up, {0, 0.15, 0, 0, 0, 0});
    CsDouble task_pose_tip_change_center_up_offset_y = arraySumCS(task_pose_tip_change_center_up, {0, 0.15, 0, 0, 0, 0});
    CsDouble task_pose_tip_change_left_up_offset_y   = arraySumCS(task_pose_tip_change_left_up, {0, 0.2, 0, 0, 0, 0});

    int sub_grp  = 1;

    // right tip change
    // attach
    taskPushBack_setTCP (module_task_test2_, tcp_default);
    taskPushBack_jsMove(module_task_test2_, task_position_initial_position_change);
    taskPushBack_irl_grp(module_task_test2_, KR_GRP::CLOSE, 0, sub_grp);
    taskPushBack_jsMove(module_task_test2_, task_position_middle_position_change);
    taskPushBack_jsMove(module_task_test2_, task_position_tip_change);
    taskPushBack_csMove(module_task_test2_, task_pose_tip_change_right_offset_y);
    ref_unit_task_.vel_cs /= 5;
    ref_unit_task_.acc_cs /= 5;
    taskPushBack_csMove(module_task_test2_, task_pose_tip_change_right);
    ref_unit_task_.vel_cs *= 5;
    ref_unit_task_.acc_cs *= 5;
    taskPushBack_csMove(module_task_test2_, task_pose_tip_change_right_up);
    taskPushBack_csMove(module_task_test2_, task_pose_tip_change_right_up_offset_y);
    // taskPushBack_jsMove(module_task_test2_, kQTask);
    taskPushBack_jsMove(module_task_test2_, task_position_middle_position_change);
    taskPushBack_jsMove(module_task_test2_, task_position_initial_position_change);

    // detach
    taskPushBack_setTCP (module_task_test3_, tcp_default);
    taskPushBack_jsMove(module_task_test3_, task_position_initial_position_change);
    taskPushBack_irl_grp(module_task_test3_, KR_GRP::CLOSE, 0, sub_grp);
    taskPushBack_jsMove(module_task_test3_, task_position_middle_position_change);
    taskPushBack_jsMove(module_task_test3_, task_position_tip_change);
    taskPushBack_csMove(module_task_test3_, task_pose_tip_change_right_up_offset_y);
    taskPushBack_csMove(module_task_test3_, task_pose_tip_change_right_up);
    ref_unit_task_.vel_cs /= 5;
    ref_unit_task_.acc_cs /= 5;
    taskPushBack_csMove(module_task_test3_, task_pose_tip_change_right);
    ref_unit_task_.vel_cs *= 5;
    ref_unit_task_.acc_cs *= 5;
    taskPushBack_csMove(module_task_test3_, task_pose_tip_change_right_offset_y);
    taskPushBack_jsMove(module_task_test3_, task_position_tip_change);
    // taskPushBack_jsMove(module_task_test3_, kQTask);
    taskPushBack_jsMove(module_task_test3_, task_position_middle_position_change);
    taskPushBack_jsMove(module_task_test3_, task_position_initial_position_change);

    // center tip change
    // attach
    taskPushBack_setTCP (module_task_test4_, tcp_default);
    taskPushBack_jsMove(module_task_test4_, task_position_initial_position_change);
    taskPushBack_irl_grp(module_task_test4_, KR_GRP::CLOSE, 0, sub_grp);
    taskPushBack_jsMove(module_task_test4_, task_position_middle_position_change);
    taskPushBack_jsMove(module_task_test4_, task_position_tip_change);
    taskPushBack_csMove(module_task_test4_, task_pose_tip_change_center_offset_y);
    ref_unit_task_.vel_cs /= 5;
    ref_unit_task_.acc_cs /= 5;
    taskPushBack_csMove(module_task_test4_, task_pose_tip_change_center);
    ref_unit_task_.vel_cs *= 5;
    ref_unit_task_.acc_cs *= 5;
    taskPushBack_csMove(module_task_test4_, task_pose_tip_change_center_up);
    taskPushBack_csMove(module_task_test4_, task_pose_tip_change_center_up_offset_y);
    // taskPushBack_jsMove(module_task_test4_, kQTask);
    taskPushBack_jsMove(module_task_test4_, task_position_middle_position_change);
    taskPushBack_jsMove(module_task_test4_, task_position_initial_position_change);

    // detach
    taskPushBack_setTCP (module_task_test5_, tcp_default);
    taskPushBack_jsMove(module_task_test5_, task_position_initial_position_change);
    taskPushBack_irl_grp(module_task_test5_, KR_GRP::CLOSE, 0, sub_grp);
    taskPushBack_jsMove(module_task_test5_, task_position_middle_position_change);
    taskPushBack_jsMove(module_task_test5_, task_position_tip_change);
    taskPushBack_csMove(module_task_test5_, task_pose_tip_change_center_up_offset_y);
    taskPushBack_csMove(module_task_test5_, task_pose_tip_change_center_up);
    ref_unit_task_.vel_cs /= 5;
    ref_unit_task_.acc_cs /= 5;
    taskPushBack_csMove(module_task_test5_, task_pose_tip_change_center);
    ref_unit_task_.vel_cs *= 5;
    ref_unit_task_.acc_cs *= 5;
    taskPushBack_csMove(module_task_test5_, task_pose_tip_change_center_offset_y);
    taskPushBack_jsMove(module_task_test5_, task_position_tip_change);
    // taskPushBack_jsMove(module_task_test5_,gb kQTask);
    taskPushBack_jsMove(module_task_test5_, task_position_middle_position_change);
    taskPushBack_jsMove(module_task_test5_, task_position_initial_position_change);

    std::cout << "\n----------- [The task has been created successfully] -----------" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////// LLM TASKS /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TaskPlanner::LLMScanMatchingGrasping(int target_id, int vel_level) {
    llm_task_zivid_.clear();

    JsDouble llm_scan_pose;
    llm_scan_pose = {-80.641, -54.269, -129.186, -87.611, 89.384, -82.897};

    // 속도 level에 따른 실제 속도 가중치
    double vel_cs = 0.6 * (0.25*vel_level + 0.25);
    double acc_cs = 0.9 * (0.25*vel_level + 0.25); // [m/s^2]
    ref_unit_task_.vel_js = 30 * (0.25*vel_level + 0.25);
    ref_unit_task_.acc_js = 60 * (0.25*vel_level + 0.25);

    if(target_id !=40){ //SCU (vacuum gripper)
        taskPushBack_irl_grp(llm_task_zivid_, KR_GRP::CLOSE);
    }

    taskPushBack_jsMove(llm_task_zivid_, llm_scan_pose);
    taskPushBack_delay_1ms(llm_task_zivid_, 100);

    if(target_id == 40){ //SCU (vacuum gripper)
        // taskPushBackKORASGripperCmd(llm_task_zivid_, (uint16_t)KR_GRP::VACUUM_OFF);
        taskPushBack_irl_grp(llm_task_zivid_, KR_GRP::VACUUM_OFF, 0, 1);
    }
    else if(target_id==39){ //striker
        taskPushBack_irl_grp(llm_task_zivid_, KR_GRP::POS_CTRL, 1000, 1);
    }
    else if(target_id==37){ //bolt
        taskPushBack_irl_grp(llm_task_zivid_, KR_GRP::POS_CTRL, 300, 1);
    }
    else if(target_id==41){ // grill
        taskPushBack_irl_grp(llm_task_zivid_, KR_GRP::POS_CTRL, 2000, 1);
    }
    else{
        taskPushBack_irl_grp(llm_task_zivid_, KR_GRP::POS_CTRL, 3500, 1);
    }
    taskPushBack_delay_1ms(llm_task_zivid_, 60);

    //// Scanning
    taskPushBack_taskRecog_3DScanning_sharedTask(llm_task_zivid_); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(llm_task_zivid_, 8000); // 10초
    //// Matching
    taskPushBack_taskRecog_matching_sharedTask(llm_task_zivid_, false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(llm_task_zivid_, 200);
    //// Grasping
    taskPushBack_doGrasping_sharedTask(llm_task_zivid_, 0.2, 0.1); // Move to grasping pose, (acc, vel)

    //// Tool frame 기준 approach 자세
    taskPushBack_csMoveToolFrame(llm_task_zivid_, {0, 0, 0.08, 0, 0, 0}, acc_cs*0.3, vel_cs*0.3, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(llm_task_zivid_, 50);

    if(target_id ==40){ //SCU (vacuum gripper)
         //// Gripper
        // taskPushBackKORASGripperCmd(llm_task_zivid_, (uint16_t)KR_GRP::VACUUM_ON);
        taskPushBack_irl_grp(llm_task_zivid_, KR_GRP::VACUUM_ON, 0, 1);
        taskPushBack_delay_1ms(llm_task_zivid_, 2500);

        taskPushBack_csMove2(llm_task_zivid_, {0, 0, 0.080, 0, 0, 0}, acc_cs*0.1, vel_cs*0.1, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_zivid_, 10);

        taskPushBack_csMove2(llm_task_zivid_, {0, 0, 0.150, 0, 0, 0}, acc_cs, vel_cs, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_zivid_, 10);
    }
    else if(target_id==35){ ///bolt bush (parallel gripper)
            //// Gripper Close
        taskPushBackKORASGripperCmd(llm_task_zivid_, (uint16_t)KR_GRP::OPEN);
        taskPushBack_delay_1ms(llm_task_zivid_, 1000);

        taskPushBack_csMove2(llm_task_zivid_, {0, 0, 0.08, 0, 0, 0}, acc_cs*0.3, vel_cs*0.3, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_zivid_, 50);

        //// Gripper additional close command 1
        taskPushBackKORASGripperCmd(llm_task_zivid_, (uint16_t)KR_GRP::OPEN);
        taskPushBack_delay_1ms(llm_task_zivid_, 10);

        taskPushBack_csMove2(llm_task_zivid_, {0, 0, 0.080, 0, 0, 0}, acc_cs, vel_cs, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_zivid_, 10);

        //// Gripper additional close command 1
        taskPushBackKORASGripperCmd(llm_task_zivid_, (uint16_t)KR_GRP::OPEN);
        taskPushBack_delay_1ms(llm_task_zivid_, 10);
    }
    else{
        //// Gripper Close
        //taskPushBackKORASGripperCmd(llm_task_zivid_, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_irl_grp(llm_task_zivid_, KR_GRP::CLOSE, 0, 1);
        taskPushBack_delay_1ms(llm_task_zivid_, 1000);

        taskPushBack_csMove2(llm_task_zivid_, {0, 0, 0.08, 0, 0, 0}, acc_cs*0.3, vel_cs*0.3, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_zivid_, 50);

        //// Gripper additional close command 1
        // taskPushBackKORASGripperCmd(llm_task_zivid_, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_irl_grp(llm_task_zivid_, KR_GRP::CLOSE, 0, 1);
        taskPushBack_delay_1ms(llm_task_zivid_, 10);

        taskPushBack_csMove2(llm_task_zivid_, {0, 0, 0.150, 0, 0, 0}, acc_cs, vel_cs, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_zivid_, 10);

        //// Gripper additional close command 1
        // taskPushBackKORASGripperCmd(llm_task_zivid_, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_irl_grp(llm_task_zivid_, KR_GRP::CLOSE, 0, 1);
        taskPushBack_delay_1ms(llm_task_zivid_, 10);
    }

}

void TaskPlanner::LLMMoveWayPoints(std::vector<vector<double>> waypoints, int vel_level) {
    llm_task_waypoints_.clear();

    // 속도 level에 따른 실제 속도 가중치
    double vel_cs = 0.3 * (0.25*vel_level + 0.25);
    double acc_cs = 0.6 * (0.25*vel_level + 0.25); // [m/s^2]
    taskPushBack_setTCP(llm_task_waypoints_, {0, 0, 0.24500, 0, 0, 0});
    for (int i = 0; i < waypoints.size(); ++i) {
        for (auto& value : waypoints[i]) {
            value *= 0.001;
        }
        waypoints[i].insert(waypoints[i].end(), {180, 0, 0});
        taskPushBack_csMove2(llm_task_waypoints_, waypoints[i], acc_cs, vel_cs, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_waypoints_, 200);
    }
}

void TaskPlanner::LLMAttachGripper(int current_gripper, int vel_level) {
    llm_task_attach_.clear();

    double tool_changing_insertion_distance = 0.0505; // 툴 장착위치로부터 떨어진 거리
    double tool_changing_exit_from_tool_home_distance = 0.08; // 툴 거치대의 내부에서 입장하기 위한 거리

    CsDouble tool_changing_tcp = {0, 0, 0, 0, 0, 90};
    JsDouble grp_home_pose = {-61.332, -61.761, -95.944, -22.196, 151.207, 0.011};
    JsDouble gripper_2F_attach_pose = {-8.560, -61.503, -106.459, -11.882, 98.459, -0.131};
    JsDouble gripper_VC_attach_pose = {29.479, -52.862, -109.794, -17.066, 60.397, -0.283};
    JsDouble gripper_PR_attach_pose = {-147.026, -96.593, 121.808, -206.193, -123.766, 0.973}; // 평행 그리퍼

    JsDouble selected_gripper_pose;

    switch (current_gripper) {
        case 1:
            selected_gripper_pose = gripper_2F_attach_pose;
            break;
        case 2:
            selected_gripper_pose = gripper_VC_attach_pose;
            break;
        case 3:
            selected_gripper_pose = gripper_PR_attach_pose;
            break;
        default:
            std::cerr << "Don't change the gripper" << std::endl;
            selected_gripper_pose = {};
            break;
    }

    std::cout << "LLMTEST Move Gripper Attach to position: " << selected_gripper_pose[0] << ", " << selected_gripper_pose[1] << ", "
              << selected_gripper_pose[2] << ", " << selected_gripper_pose[3] << ", " << selected_gripper_pose[4] << ", "
              << selected_gripper_pose[5] << std::endl;

    // 속도 level에 따른 실제 속도 가중치
    double vel_cs = 0.3 * (0.25*vel_level + 0.25);
    double acc_cs = 0.6 * (0.25*vel_level + 0.25); // [m/s^2]
    ref_unit_task_.vel_js = 40 * (0.25*vel_level + 0.25);
    ref_unit_task_.acc_js = 70 * (0.25*vel_level + 0.25);

    taskPushBack_setTCP(llm_task_attach_, tool_changing_tcp);
    if(std::any_of(selected_gripper_pose.begin(), selected_gripper_pose.end(), [](double val) { return val != 0.0; })) {
        // gripper changing
        taskPushBack_jsMove(llm_task_attach_, selected_gripper_pose, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_attach_, 60);

        taskPushBack_csMoveToolFrame(llm_task_attach_, {0, 0, +tool_changing_insertion_distance, 0, 0, 0}, acc_cs, vel_cs, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_attach_, 60);

        taskPushBack_csMoveToolFrame(llm_task_attach_, {-tool_changing_exit_from_tool_home_distance, 0, 0, 0, 0, 0}, acc_cs, vel_cs, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_attach_, 60);

        taskPushBack_irl_grp(llm_task_attach_, KR_GRP::CLOSE, 0, 1);
    }
}

void TaskPlanner::LLMDetachGripper(int current_gripper, int vel_level) {
    llm_task_detach_.clear();

    double tool_changing_insertion_distance = 0.05; // 툴 장착위치로부터 떨어진 거리
    double tool_changing_exit_from_tool_home_distance = 0.08; // 툴 거치대의 내부에서 입장하기 위한 거리

    CsDouble tool_changing_tcp = {0, 0, 0, 0, 0, 90};
    JsDouble grp_home_pose = {-61.332, -61.761, -95.944, -22.196, 151.207, 0.011};
    JsDouble gripper_2F_detach_pose = {-7.208, -67.554, -94.967, -17.324, 97.099, -0.120};
    JsDouble gripper_VC_detach_pose = {23.198, -61.223, -98.077, -20.458, 66.682, -0.233};
    JsDouble gripper_PR_detach_pose = {-150.497, -97.041, 112.476, -196.362, -120.316, 1.031};
    JsDouble selected_gripper_pose;

    switch (current_gripper) {
        case 1:
            selected_gripper_pose = gripper_2F_detach_pose;
            break;
        case 2:
            selected_gripper_pose = gripper_VC_detach_pose;
            break;
        case 3:
            selected_gripper_pose = gripper_PR_detach_pose;
            break;
        default:
            std::cerr << "Don't change the gripper" << std::endl;
            selected_gripper_pose = {};
            break;
    }

    std::cout << "LLMTEST Move Gripper Detach to position: " << selected_gripper_pose[0] << ", " << selected_gripper_pose[1] << ", "
              << selected_gripper_pose[2] << ", " << selected_gripper_pose[3] << ", " << selected_gripper_pose[4] << ", "
              << selected_gripper_pose[5] << std::endl;

    // 속도 level에 따른 실제 속도 가중치
    double vel_cs = 0.3 * (0.25*vel_level + 0.25);
    double acc_cs = 0.6 * (0.25*vel_level + 0.25); // [m/s^2]
    ref_unit_task_.vel_js = 40 * (0.25*vel_level + 0.25);
    ref_unit_task_.acc_js = 70 * (0.25*vel_level + 0.25);

    taskPushBack_setTCP(llm_task_detach_, tool_changing_tcp);
    if(std::any_of(selected_gripper_pose.begin(), selected_gripper_pose.end(), [](double val) { return val != 0.0; })){
        taskPushBack_jsMove(llm_task_detach_, grp_home_pose, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_detach_, 60);
        // gripper changing
        taskPushBack_jsMove(llm_task_detach_, selected_gripper_pose, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_detach_, 60);

        taskPushBack_csMoveToolFrame(llm_task_detach_, {tool_changing_exit_from_tool_home_distance, 0, 0, 0, 0, 0}, acc_cs, vel_cs, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_detach_, 60);

        taskPushBack_csMoveToolFrame(llm_task_detach_, {0, 0, -tool_changing_insertion_distance, 0, 0, 0}, acc_cs, vel_cs, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_detach_, 60);
    }
}

void TaskPlanner::LLMAttachTip(int current_tip, int vel_level) {
    llm_task_attach_tip_.clear();

    double tip_changing_insertion_distance = 0.0505;
    double tip_changing_exit_distance = 0.08;

    CsDouble tool_changing_tcp = {0, 0, 0, 0, 0, 90};
    JsDouble grp_home_pose = {-61.332, -61.761, -95.944, -22.196, 151.207, 0.011};
    JsDouble tip_R_attach_pose = {-45.192, -68.078, -106.126, -5.084, 135.100, 0.049};
    JsDouble tip_S_attach_pose = {-57.266, -81.809, -96.161, -1.084, 147.184, 0.372};

    JsDouble selected_tip_pose;

    switch (current_tip) {
        case 1:
            selected_tip_pose = tip_R_attach_pose;
            break;
        case 2:
            selected_tip_pose = tip_S_attach_pose;
            break;
        default:
            std::cerr << "Don't change the tip" << std::endl;
            selected_tip_pose = {};
            break;
    }

    std::cout << "LLMTEST Move Tip Attach to position: " << selected_tip_pose[0] << ", " << selected_tip_pose[1] << ", "
              << selected_tip_pose[2] << ", " << selected_tip_pose[3] << ", " << selected_tip_pose[4] << ", "
              << selected_tip_pose[5] << std::endl;

    // 속도 level에 따른 실제 속도 가중치
    double vel_cs = 0.3 * (0.25*vel_level + 0.25);
    double acc_cs = 0.6 * (0.25*vel_level + 0.25); // [m/s^2]
    ref_unit_task_.vel_js = 40 * (0.25*vel_level + 0.25);
    ref_unit_task_.acc_js = 70 * (0.25*vel_level + 0.25);

    taskPushBack_setTCP(llm_task_attach_tip_, tool_changing_tcp);
    if(std::any_of(selected_tip_pose.begin(), selected_tip_pose.end(), [](double val) { return val != 0.0; })) {
        // tip changing
        taskPushBack_jsMove(llm_task_attach_tip_, selected_tip_pose, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_attach_tip_, 60);

        taskPushBack_csMoveToolFrame(llm_task_attach_tip_, {0, 0, tip_changing_insertion_distance, 0, 0, 0}, acc_cs, vel_cs, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_attach_tip_, 60);

        taskPushBack_csMoveToolFrame(llm_task_attach_tip_, {-tip_changing_exit_distance, 0, 0, 0, 0, 0}, acc_cs, vel_cs, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_attach_tip_, 60);
        taskPushBack_irl_grp(llm_task_attach_, KR_GRP::SET_SPEED, 100, 1);
    }
}

void TaskPlanner::LLMDetachTip(int current_tip, int vel_level) {
    llm_task_detach_tip_.clear();

    double tip_changing_insertion_distance = 0.08;
    double tip_changing_exit_distance = 0.05;

    CsDouble tool_changing_tcp = {0, 0, 0, 0, 0, 90};
    JsDouble grp_home_pose = {-61.332, -61.761, -95.944, -22.196, 151.207, 0.011};
    JsDouble tip_R_detach_pose = {-39.582, -71.152, -96.005, -12.170, 129.477, -0.030};
    JsDouble tip_S_detach_pose = {-52.390, -83.734, -86.181, -9.232, 142.289, 0.250};
    JsDouble selected_tip_pose;

    switch (current_tip) {
        case 1:
            selected_tip_pose = tip_R_detach_pose;
            break;
        case 2:
            selected_tip_pose = tip_S_detach_pose;
            break;
        default:
            std::cerr << "Don't change tip" << std::endl;
            selected_tip_pose = {};
            break;
    }

    std::cout << "LLMTEST Move Gripper Detach to position: " << selected_tip_pose[0] << ", " << selected_tip_pose[1] << ", "
              << selected_tip_pose[2] << ", " << selected_tip_pose[3] << ", " << selected_tip_pose[4] << ", "
              << selected_tip_pose[5] << std::endl;

    // 속도 level에 따른 실제 속도 가중치
    double vel_cs = 0.3 * (0.25*vel_level + 0.25);
    double acc_cs = 0.6 * (0.25*vel_level + 0.25); // [m/s^2]
    ref_unit_task_.vel_js = 40 * (0.25*vel_level + 0.25);
    ref_unit_task_.acc_js = 70 * (0.25*vel_level + 0.25);

    taskPushBack_setTCP(llm_task_detach_tip_, tool_changing_tcp);
    // taskPushBackKORASGripperCmd(llm_task_detach_tip_, (uint16_t)KR_GRP::CLOSE);
    taskPushBack_irl_grp(llm_task_detach_tip_, KR_GRP::CLOSE, 0, 1);
    taskPushBack_delay_1ms(llm_task_detach_tip_, 500);
    if(std::any_of(selected_tip_pose.begin(), selected_tip_pose.end(), [](double val) { return val != 0.0; })) {

        taskPushBack_jsMove(llm_task_detach_tip_, grp_home_pose, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_detach_tip_, 60);
        // tip changing
        taskPushBack_jsMove(llm_task_detach_tip_, selected_tip_pose, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_detach_tip_, 60);

        taskPushBack_csMoveToolFrame(llm_task_detach_tip_, {tip_changing_insertion_distance, 0, 0, 0, 0, 0}, acc_cs, vel_cs, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_detach_tip_, 60);

        taskPushBack_csMoveToolFrame(llm_task_detach_tip_, {0, 0, -tip_changing_exit_distance, 0, 0, 0}, acc_cs, vel_cs, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_detach_tip_, 60);
    }
}

void TaskPlanner::UpdateBoxPose(double x, double y, double rz) {
    double center_x = x + (-120 * cos(rz * M_PI/180) - 200 * sin(rz * M_PI/180));
    double center_y = y + (-120 * sin(rz * M_PI/180) + 200 * cos(rz * M_PI/180));
    current_box_pose_ = {center_x * 0.001 , center_y * 0.001, 0.45, 179.9, 0, rz};
    std::cout << "LLM Move Object to Color Box: " << current_box_pose_[0] << ", " << current_box_pose_[1] << ", "
              << current_box_pose_[2] << ", " << current_box_pose_[3] << ", " << current_box_pose_[4] << ", "
              << current_box_pose_[5] << std::endl;
}

void TaskPlanner::LLMMoveBox(int boxNumber, int boxStack, int current_obj, int vel_level, int palette_num) {
    llm_task_box_.clear();
    auto grp_release = KR_GRP::OPEN;
    auto grp_default = KR_GRP::CLOSE;
    double object_insertion_distance;

    // 속도 level에 따른 실제 속도 가중치
    double vel_cs = 0.3 * (0.25*vel_level + 0.25);
    double acc_cs = 0.6 * (0.25*vel_level + 0.25); // [m/s^2]
    ref_unit_task_.vel_js = 40 * (0.25*vel_level + 0.25);
    ref_unit_task_.acc_js = 70 * (0.25*vel_level + 0.25);

    // current_obj에 따른 object_insertion_distance 설정
    if (current_obj == 36) {
        object_insertion_distance = 0.16; // 툴 장착위치로부터 떨어진 거리 for peg
        grp_release = KR_GRP::OPEN;
        grp_default = KR_GRP::CLOSE;
    } else if (current_obj == 40) {
        object_insertion_distance = 0.23; // 예시 거리 for scu
        grp_release = KR_GRP::VACUUM_OFF;
        grp_default = KR_GRP::VACUUM_OFF;
    } else if (current_obj == 35) {
        object_insertion_distance = 0.2; // 예시 거리 for bolt_bush
        grp_release = KR_GRP::CLOSE;
        grp_default = KR_GRP::CLOSE;
    } else if (current_obj == 37) {
        object_insertion_distance = 0.05; // 예시 거리 for bolt
        grp_release = KR_GRP::POS_CTRL;
        grp_default = KR_GRP::CLOSE;
    } else {
        std::cerr << "Unknown current_obj value. Defaulting insertion_distance." << std::endl;
        object_insertion_distance = 0.23; // 거의 바닥에 가까운 위치
    }

    CsDouble tool_changing_tcp = {0, 0, 0, 0, 0, 90};
    JsDouble home_pose = {-80.641, -54.269, -129.186, -87.611, 89.384, -82.897};
    JsDouble mid_pose = {-122.683, -89.656, -105.906, -74.103, 89.543, -122.477};
    JsDouble box_base_pose = {-177.303, -87.894, -107.568, -74.470, 89.329, -84.873};
    JsDouble blue_in_pose = {-254.967, -98.295, 122.002, -113.553, -90.272, -75.236};
    JsDouble yellow_in_pose = {-291.132, -114.252, 132.860, -108.583, -90.304, -111.379};
    // JsDouble palette_in_pose = {-134.184, -109.790, -102.297, -58.196, 89.232, -47.418}; // The palette_5
    JsDouble palette_in_pose = {-104.107, -82.816, -113.895, -73.918, 89.478, -17.336}; // The palette_5
    JsDouble bolt_in_pose = {-128.714, -109.745, -106.828, -27.675, 109.400, -215.674};

    // if (boxNumber == 1) {
    //     // selected_box_pose = blue_in_pose;
    //     CsDouble selected_box_pose;
    //     selected_box_pose = current_box_pose_;
    // } else if (boxNumber == 2) {
    //     // selected_box_pose = yellow_in_pose;
    //     CsDouble selected_box_pose;
    //     selected_box_pose = current_box_pose_;
    // } else if (boxNumber == 3) {
    //     JsDouble selected_box_pose;
    //     selected_box_pose = palette_in_pose;
    // } else {
    //     std::cerr << "Invalid. Defaulting to yellow box." << std::endl;
    //     JsDouble selected_box_pose;
    //     selected_box_pose = yellow_in_pose;
    // }

    // std::cout << "LLM Move Object to Color Box: " << selected_box_pose[0] << ", " << selected_box_pose[1] << ", "
    //           << selected_box_pose[2] << ", " << selected_box_pose[3] << ", " << selected_box_pose[4] << ", "
    //           << selected_box_pose[5] << std::endl;

    taskPushBack_setTCP(llm_task_box_, tool_changing_tcp);
    taskPushBack_delay_1ms(llm_task_box_, 10);

    if (boxNumber == 3){
        // taskPushBack_jsMove(llm_task_box_, mid_pose, false); // i: index, false: absolute, true: relative
        // taskPushBack_delay_1ms(llm_task_box_, 10);

        taskPushBack_jsMove(llm_task_box_, palette_in_pose, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_box_, 10);
    }
    else if (boxNumber ==4) {
        taskPushBack_jsMove(llm_task_box_, mid_pose, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_box_, 10);

        taskPushBack_jsMove(llm_task_box_, bolt_in_pose, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_box_, 10);
    }
    else{
        taskPushBack_jsMove(llm_task_box_, box_base_pose, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_box_, 10);

        std::vector<double> current_box_pose(std::begin(current_box_pose_), std::end(current_box_pose_));
        taskPushBack_csMove2(llm_task_box_, current_box_pose, acc_cs, vel_cs, false);
        taskPushBack_delay_1ms(llm_task_box_, 60);
        object_insertion_distance += (1 - boxStack) * 0.1;
    }

    if (boxNumber == 3){
        double x_offset = 0.0;
        double y_offset = 0.0;
        object_insertion_distance = 0.03;
        if (palette_num < 4){
            y_offset = -0.138;
            x_offset = 0.105 * (palette_num - 2);
        }
        else if (palette_num < 7){
            y_offset = 0.0;
            x_offset = 0.105 * (palette_num - 5);
        }
        else if (palette_num < 10){
            y_offset = 0.138;
            x_offset = 0.105 * (palette_num - 8);
        }


        std::vector<double> target_hole = {-0.69121, -0.45910, 0.33648, -179.688, -0.755, 93.237};

        target_hole[0] += y_offset;
        target_hole[1] += x_offset;

        taskPushBack_csMove2(llm_task_box_, target_hole, acc_cs, vel_cs, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_box_, 50);

        // taskPushBack_csMoveToolFrame(llm_task_box_, {x_offset, y_offset, 0, 0, 0, 0}, acc_cs, vel_cs, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(llm_task_box_, 60);
    }

    taskPushBack_csMoveToolFrame(llm_task_box_, {0, 0, +object_insertion_distance, 0, 0, 0}, acc_cs, vel_cs, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(llm_task_box_, 60);

    taskPushBack_irl_grp(llm_task_box_, grp_release, 1000, 1);
    taskPushBack_delay_1ms(llm_task_box_, 1000);

    taskPushBack_csMoveToolFrame(llm_task_box_, {0, 0, -object_insertion_distance, 0, 0, 0}, acc_cs, vel_cs, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(llm_task_box_, 60);

    taskPushBack_irl_grp(llm_task_box_, grp_default, 0, 1);
    taskPushBack_delay_1ms(llm_task_box_, 10);

    if (boxNumber == 3){
        taskPushBack_jsMove(llm_task_box_, mid_pose, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_box_, 10);
    }
    else if (boxNumber == 4){
        taskPushBack_jsMove(llm_task_box_, mid_pose, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_box_, 10);
    }
    else{
        taskPushBack_jsMove(llm_task_box_, box_base_pose, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(llm_task_box_, 60);
    }
    taskPushBack_jsMove(llm_task_box_, home_pose, false); // i: index, false: absolute, true: relative
    taskPushBack_delay_1ms(llm_task_box_, 60);

    // taskPushBack_csMoveToolFrame(llm_task_list4_, {-tool_changing_exit_from_tool_home_distance, 0, 0, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
    // taskPushBack_delay_1ms(llm_task_list4_, 60);

    // taskPushBack_irl_grp(llm_task_list4_, KR_GRP::INIT, 0, 1);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// ROBOT CALIBRATION TASKS ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void TaskPlanner::CalGenerationJSTask(std::vector<vector<double>> pose_set, double qd, double qdd, std::string folder_name) {
    fname_ = folder_name;
    cal_mode_ = "JS";
    JsDouble qpose;
    cal_task_list_.clear();

    ref_unit_task_.vel_js = qd;
    ref_unit_task_.acc_js = qdd;

    for(int i=0; i<pose_set.size(); i++) {
        std::cout<<"qpose"<<i+1<<": ";
        for (int j = 0; j < 6; j++) {
            qpose[j] = pose_set[i][j];
            std::cout<<qpose[j]<<", ";
        }
        std::cout<<std::endl;
        taskPushBack_jsMove(cal_task_list_, qpose);
        taskPushBack_delay(cal_task_list_, 100);
        taskPushBack_save_ply(cal_task_list_);
        taskPushBack_delay(cal_task_list_, 100);
    }
}

void TaskPlanner::CalGenerationCSTask(std::vector<vector<double>> pose_set, double xd, double xdd, std::string folder_name) {
    fname_ = folder_name;
    cal_mode_ = "CS";
    CsDouble xpose;
    cal_task_list_.clear();

    ref_unit_task_.vel_cs = xd;
    ref_unit_task_.acc_cs = xdd;

    for(int i=0; i<pose_set.size(); i++) {
        std::cout<<"xpose"<<i+1<<": ";
        for (int j = 0; j < 6; j++) {
            xpose[j] = pose_set[i][j];
            std::cout<<xpose[j]<<", ";
        }
        std::cout<<std::endl;
        taskPushBack_csMove(cal_task_list_, xpose);
        taskPushBack_delay(cal_task_list_, 100);
        taskPushBack_save_ply(cal_task_list_);
        taskPushBack_delay(cal_task_list_, 100);
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// MAKE MODULE TASKS ///////////////////////////////////////////////

void TaskPlanner::readAssemParam() {
    // std::string path_pin     = "../catkin_ws/src/Furniture_Assembly_Project/furniture_assemble_ui/config/fastener_assem_param/pin_parameter.json";
    // std::string path_bolt    = "../catkin_ws/src/Furniture_Assembly_Project/furniture_assemble_ui/config/fastener_assem_param/bolt_parameter.json";
    // std::string path_bracket = "../catkin_ws/src/Furniture_Assembly_Project/furniture_assemble_ui/config/fastener_assem_param/bracket_parameter.json";
    // std::string path_screw   = "../catkin_ws/src/Furniture_Assembly_Project/furniture_assemble_ui/config/fastener_assem_param/screw_parameter.json";
    // std::string path_part    = "../catkin_ws/src/Furniture_Assembly_Project/furniture_assemble_ui/config/fastener_assem_param/part_parameter.json";

    // std::string root_input_pin     = "pinAssemParam";
    // std::string root_input_bolt    = "boltAssemParam";
    // std::string root_input_bracket = "bracketAssemParam";
    // std::string root_input_screw   = "screwAssemParam";
    // std::string root_input_part    = "partAssemParam";

    // auto readParamFunction = [] (Assem &assemblyParam, std::string path, std::string rootInput) {
    //     std::ifstream ifs;

    //     Json::Value param;
    //     Json::Value root;

    //     ifs.open(path);
    //     ifs >> root;
    //     param = root[rootInput];

    //     for (unsigned int i = 0; i < 6; i++) {
    //         assemblyParam.impedance.nf.at(i)          = param["nf"]        [i].asDouble();
    //         assemblyParam.impedance.zeta.at(i)        = param["zeta"]      [i].asDouble();
    //         assemblyParam.impedance.stiffness.at(i)   = param["stiffness"] [i].asDouble();
    //         assemblyParam.impedance.force_limit.at(i) = param["forceLimit"][i].asDouble();

    //         assemblyParam.target_force .at(i) = param["targetForce"] [i].asDouble();
    //         assemblyParam.contact_force.at(i) = param["contactForce"][i].asDouble();

    //         assemblyParam.move_vel   .at(i) = param["moveVelocity"]   [i].asDouble();
    //         assemblyParam.insert_vel .at(i) = param["insertVelocity"] [i].asDouble();
    //         assemblyParam.contact_vel.at(i) = param["contactVelocity"][i].asDouble();
    //     }

    //     assemblyParam.insertion_depth = param["insertionDepth"].asDouble();
    // };
}


/////////////////////////////////////// Bin picking task
void TaskPlanner::makeRecogOnlyGraspingTaskList(bool is_slow_mode) {

    //// MACHINE TENDING TASK
    module_task_bin_picking_single_machine_tending_lathe_.clear();
    module_task_bin_picking_single_machine_tending_milling_.clear();
    // module_task_bin_picking_dual_machine_tending_.clear();
    module_task_bin_picking_machine_tending_bolt_bush_.clear();

    //// PICK AND PLACE TASK
    module_task_bin_picking_pick_and_place_.clear();
    module_task_bin_picking_pick_and_place_eb_joint_.clear();
    module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_.clear();
    module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_.clear();
    module_task_bin_picking_pick_and_place_bolt_bush_.clear();
    module_task_bin_picking_pick_and_place_cylinder_.clear();
    module_task_bin_picking_pick_and_place_square_peg_.clear();

    //// Module task - grasping
    auto makeInitializePnematicChuck = [&] (std::vector<UnitTask>& targetModuleTask) {
        //// Cylinder demo set
        // taskPushBack_gripper_pne1(targetModuleTask, false); // Vertical pnematic OFF
        taskPushBackGripperRelayCmd(targetModuleTask, PNE1_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBack_gripper_pne2(targetModuleTask, false); // Horizontal pnematic ON
        taskPushBackGripperRelayCmd(targetModuleTask, PNE2_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };

    //// Module task - grasping
    auto makeInitializePLCChuck = [&] (std::vector<UnitTask>& targetModuleTask) {
        taskPushBackPLCModbusCmd(targetModuleTask, LATHE_CHUCK_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, MILLING_CHUCK_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };

    //// Module task - grasping
    auto makeInitializeTCP = [&] (std::vector<UnitTask>& targetModuleTask, unsigned int tcp_idx) {
        // Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6)
        taskPushBack_selectTCP(targetModuleTask, tcp_idx); // Left hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };

    //// Module task - grasping
    auto makeInitializeTCPAndGripper = [&] (std::vector<UnitTask>& targetModuleTask, unsigned int tcp_idx, unsigned int grp_driver_idx) {
        // Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6)
        taskPushBack_selectTCP(targetModuleTask, tcp_idx); // Left hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// Gripper driver change(#1 or #2)
        taskPushBack_setGripperDriver(targetModuleTask, grp_driver_idx); // Driver #1 Left hand (for loading)
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };

    auto makeMoveJSDefaultPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        //// Single gripper
        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);
    };

    //// Module task - grasping
    auto makeMoveJSMiddlePosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        taskPushBack_jsMove(targetModuleTask, {230.626, -116.719, 121.618, -95.213, -89.574, -31.555}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);
    };

    auto makeRegraspingTaskPnematicChuck = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {

        std::vector<double> regrasping_pose_1, regrasping_pose_2;
        switch(demo_tag) { // JSON input
            case BPDemoType::DEMO_BP_CNC_LATHE_CYLINDER: {
                std::vector<double> vec_tmp_1 = {0.21726, -0.49994, 0.03426, 179.335, 0.230, -179.331};
                regrasping_pose_1 = vec_tmp_1;
                std::vector<double> vec_tmp_2 = {0.20629, -0.49918, 0.08973, -179.838, 0.055, 178.263};
                regrasping_pose_2 = vec_tmp_2;
                break;
            }
            case BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG: {
                std::vector<double> vec_tmp_1 = {0.21726, -0.49994, 0.13426, 179.335, 0.230, -179.331};
                regrasping_pose_1 = vec_tmp_1;
                std::vector<double> vec_tmp_2 = {0.20629, -0.49918, 0.18973, -179.838, 0.055, 178.263};
                regrasping_pose_2 = vec_tmp_2;
                break;
            }
            default: {
                assert(false);
                break;
            }
        }

        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 2.0; vel_move = 1.0;
            acc_contact = 0.2; vel_contact = 0.1;
        }
        taskPushBackGripperRelayCmd(targetModuleTask, PNE1_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackGripperRelayCmd(targetModuleTask, PNE2_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.05, 0, 0, 0}, acc_move, vel_move, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // KORAS gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBackGripperRelayCmd(targetModuleTask, PNE2_ON);
        taskPushBack_delay_1ms(targetModuleTask, 1500);

        //// code here
        // JS: 94.424, -91.897, 134.768, -132.072, -89.926, -85.823
        taskPushBack_csMove2(targetModuleTask, regrasping_pose_1, acc_move, vel_move, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Gripper close 1
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        taskPushBackGripperRelayCmd(targetModuleTask, PNE2_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// Gripper close 2
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        taskPushBack_csMoveToolFrame(targetModuleTask, {0.005, 0, -0.005, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        taskPushBack_csMove2(targetModuleTask, regrasping_pose_2, acc_move, vel_move, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Gripper close 3
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 20);

    };

    auto makeRegraspingTaskZig = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 2.0; vel_move = 1.0;
            acc_contact = 0.2; vel_contact = 0.1;
        }

        taskPushBack_taskJSON_csMoveMotionTag(targetModuleTask, {0, 0, 0, 0, 0, 0}, demo_tag, MotionTag::MOTION_TAG_JSON_CSMOVE_REGRASP_PLACE);
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // KORAS gripper Open
        if(demo_tag != BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) {
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 60);
        } else {
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1500);
        }

        taskPushBack_taskJSON_csMoveMotionTag(targetModuleTask, {0, 0, 0, 0, 0, 0}, demo_tag, MotionTag::MOTION_TAG_JSON_CSMOVE_REGRASP_PICK);
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper close 1
        if(demo_tag != BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) {
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1000);
        } else {
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
            taskPushBack_delay_1ms(targetModuleTask, 1500);
        }
        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.005, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// Gripper close 2
        if(demo_tag != BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) {
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        }
    };

    //// Module task - grasping
    //// 기존: template matching - 서비스
    auto makeZIVIDScanningTask = [&] (std::vector<UnitTask>& targetModuleTask) {
        taskPushBack_taskRecog_3DScanning(targetModuleTask); // ZIVID Scanning with task recognition
        taskPushBack_delayScanning(targetModuleTask, 2000);
        taskPushBack_templateMatching(targetModuleTask, false, 8); // false: No sampling, true: Sampling, sampling number
        taskPushBack_delayScanning(targetModuleTask, 200);
    };

    //// 변경: template matching - 토픽 // ZIVID scanning은 서비스
    auto makeZIVIDScanningAndMatchingTask = [&] (std::vector<UnitTask>& targetModuleTask) {

        taskPushBack_taskRecog_3DScanningAndMatching(targetModuleTask, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };

    //// 변경: template matching - 토픽 // ZIVID scanning은 서비스
    auto makeCheckMatchingFinishedTaskVer2 = [&] (std::vector<UnitTask>& targetModuleTask) {
        taskPushBack_taskRecog_checkMatchingFinished(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };

    //// Module task - grasping (inner plane grasping)
    auto makeInnerGraspingTask = [&] (std::vector<UnitTask>& targetModuleTask) {
        //// Gripper initialize - KORAS gripper Close
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        taskPushBack_doGrasping_binPicking(targetModuleTask, 0.1, 0.1); // Move to grasping pose, (acc, vel)

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        //// TODO: JSON파일의 z margin과 연동해야 함.
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.04, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.050, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 200);

        // //// Gripper additional close command
        // taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        // taskPushBack_delay_1ms(targetModuleTask, 400);

        // Pick and Place - Middle position
        taskPushBack_jsMove(targetModuleTask, {93.600, -83.712, -100.319, -89.281, 88.459, 175.313}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    //// 파지
    //// Grasping with blending
    //// Module task - grasping
    auto makeSingleGripperGraspingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        taskPushBack_doGrasping_binPicking(targetModuleTask, 0.5, 0.25); // Move to grasping pose, (acc, vel)

        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.04, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Gripper Close
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.040, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Gripper additional close command 1
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 50);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.080, 0, 0, 0}, 0.75, 0.5, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Gripper additional close command 1
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 50);

        // taskPushBack_jsMove(targetModuleTask, {-59.528, 118.811, -200.909, -43.154, -117.118, 17.200}, false); // i: index, false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 100);

        // //// Gripper additional close command 2
        // taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        // taskPushBack_delay_1ms(targetModuleTask, 50);

        // //// Check grasping success/failure
        // taskPushBack_taskRecog_KORASGripperCheckGrasping(targetModuleTask);
        // taskPushBack_delay_1ms(targetModuleTask, 20);

    };

    //// Module task - grasping
    auto makeSingleVacuumGripperGraspingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {


        //// Gripper Open
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        taskPushBack_doGrasping_binPicking(targetModuleTask, 0.5, 0.25); // Move to grasping pose, (acc, vel)

        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.04, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);


        //// Gripper
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
        taskPushBack_delay_1ms(targetModuleTask, 1000);


        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.040, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.080, 0, 0, 0}, 0.75, 0.5, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);
    };

    //// Grasping with blending
    auto makeSingleGripperGraspingBlendingTask = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;

        //// Gripper - ungrasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        } else { // 2-finger grasping - Open
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        }

        ////////////////////////////////////////////////////////////////
        //// Blending motion, MotionTag::MOTION_TAG_BLENDING_HOME_TO_GRASP_APPROACH_POSE
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_HOME_TO_GRASP_APPROACH_POSE);
        taskPushBack_delay_1ms(targetModuleTask, 100);
        ////////////////////////////////////////////////////////////////
        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        //// TODO: JSON파일의 z margin과 연동해야 함.
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.04, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Gripper - grasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::OPEN);
            taskPushBack_delay_1ms(targetModuleTask, 1000);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - ON
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
            taskPushBack_delay_1ms(targetModuleTask, 1000);
        } else { // 2-finger grasping - close
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1000);
        }

        //// Tool frame 기준 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -0.04, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Gripper additional close command 1
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping


        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping
            //// TODO: 흡착 그리퍼가 파지를 성공했는 지 여부?

        } else { // 2-finger grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 20);
            // //// Check grasping success/failure
            // taskPushBack_taskRecog_KORASGripperCheckGrasping(targetModuleTask);
            // taskPushBack_delay_1ms(targetModuleTask, 20);
        }

        ////////////////////////////////////////////////////////////////
        //// Blending motion, MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_REGRASPING_ZIG
        // if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
        //     taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_CNC_INNER);
        //     taskPushBack_delay_1ms(targetModuleTask, 100);
        // } else { // Regrasping 수행하는 경우
        //     taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_REGRASPING_ZIG);
        //     taskPushBack_delay_1ms(targetModuleTask, 100);
        // }
        ////////////////////////////////////////////////////////////////
    };

    //// Grasping with blending
    auto makePickAndPlaceGraspingBlendingTaskVer2 = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;

        //// Gripper - ungrasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        } else { // 2-finger grasping - Open
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        }

        ////////////////////////////////////////////////////////////////
        //// Blending motion
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_HOME_TO_GRASP_APPROACH_POSE);
        taskPushBack_delay_1ms(targetModuleTask, 50);
        ////////////////////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG
            || demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH // bolt bush - planar inner grasping
            || demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // planar vacuum grasping
            //// NOTICE: 로봇의 자세 정확도 문제로 인해, 사각펙과 같이 평면 접촉 후, 흡착하는 경우에는 과도한 접촉력이 발생하여 로봇이 뻗는 문제 발생하므로, 적절한 높이는 고정하여 사용
            // taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_PLANAR_VACUUM_Z_FIXED_DETECTED_GRASPING_POSE);
            taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_DETECTED_GRASPING_POSE);
        } else { // default
            //// approach distance와 무관하게 수신된 detected_pose로 곧바로 이동, 파지 자세 모션(MOTION_TAG_DETECTED_GRASPING_POSE)
            taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_DETECTED_GRASPING_POSE);
        }
        taskPushBack_delay_1ms(targetModuleTask, 50);
        ////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////
        //// NOTICE: 현재 그리퍼 동작 끄고 테스트 중...
        bool is_test_no_grasping = false;
        ////////////////////////////////////////////////////////////////
        //// Gripper - grasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            if(!is_test_no_grasping) {
                taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            }
            taskPushBack_delay_1ms(targetModuleTask, 1000);

            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.040, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 50);

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) { // inner grasping
            if(!is_test_no_grasping) {
                taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            }
            taskPushBack_delay_1ms(targetModuleTask, 1000);
            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.040, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 50);

        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - ON
            if(!is_test_no_grasping) {
                taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
            }
            taskPushBack_delay_1ms(targetModuleTask, 1000);
            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.040, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 50);

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP) { // vacuum grasping - ON
            if(!is_test_no_grasping) {
                taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
            }
            taskPushBack_delay_1ms(targetModuleTask, 1000);
            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.005, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 500);

            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.035, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 50);

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // vacuum grasping - ON
            if(!is_test_no_grasping) {
                taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
            }
            taskPushBack_delay_1ms(targetModuleTask, 1000);
            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.005, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 500);

            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.035, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 50);

        } else { // 2-finger grasping - close

            if(!is_test_no_grasping) {
                taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            }
            taskPushBack_delay_1ms(targetModuleTask, 1000);

            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.005, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 50);
            if(!is_test_no_grasping) {
                taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            }
            taskPushBack_delay_1ms(targetModuleTask, 50);
            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.035, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 50);
        }



        //// Gripper additional close command 1
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) { // inner grasping

        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP) { // vacuum grasping

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // vacuum grasping

        } else { // 2-finger grasping
            if(!is_test_no_grasping) {
                taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            }
            taskPushBack_delay_1ms(targetModuleTask, 20);
            // //// Check grasping success/failure
            // taskPushBack_taskRecog_KORASGripperCheckGrasping(targetModuleTask);
            // taskPushBack_delay_1ms(targetModuleTask, 20);
        }


        // taskPushBack_csMove2(targetModuleTask, {0, 0, 0.080, 0, 0, 0}, 0.75, 0.5, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 50);
        // //// Gripper additional close command 1
        // if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping

        // } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) { // inner grasping

        // } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping

        // } else { // 2-finger grasping
        //     taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        //     taskPushBack_delay_1ms(targetModuleTask, 20);
        //     // //// Check grasping success/failure
        //     // taskPushBack_taskRecog_KORASGripperCheckGrasping(targetModuleTask);
        //     // taskPushBack_delay_1ms(targetModuleTask, 20);
        // }

    };


    auto makeStackingTask = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        // double acc_move, vel_move, acc_contact, vel_contact;
        // if(slow_mode) {
        //     acc_move = 0.2; vel_move = 0.1;
        //     acc_contact = 0.075; vel_contact = 0.03;
        // } else {
        //     acc_move = 0.6; vel_move = 0.3;
        //     acc_contact = 0.2; vel_contact = 0.1;
        // }

        // double apporach_distance = 0.0;
        // if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) {
        //     apporach_distance = 0.0075;
        // } else if (demo_tag == BPDemoType::DEMO_BP_CNC_LATHE_CYLINDER) {
        //     apporach_distance = 0.042;
        // } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) {
        //     apporach_distance = 0.01;
        // } else { // 2-finger grasping - Open
        //     apporach_distance = 0.01;
        // }

        // // pose, acc(0.05m/s^2), vel(0.0033m/s)
        // //// TODO: JSON파일의 z margin과 연동해야 함.
        // //// Tool frame 기준 approach 자세
        // taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, apporach_distance, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 50);

        // // Gripper - ungrasp
        // if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
        //     taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        //     taskPushBack_delay_1ms(targetModuleTask, 1000);
        // } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
        //     taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
        //     taskPushBack_delay_1ms(targetModuleTask, 1000);
        // } else { // 2-finger grasping - Open
        //     taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        //     taskPushBack_delay_1ms(targetModuleTask, 1000);
        // }
        // taskPushBack_countDetachingPose(targetModuleTask);
        // taskPushBack_delay_1ms(targetModuleTask, 10);

        // taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -apporach_distance, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 50);
        // ////////////////////////////////////////////////////////////////
    };

    auto makePickAndPlaceDoStackingTaskVer2 = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 0.6; vel_move = 0.3;
            acc_contact = 0.2; vel_contact = 0.1;
        }

        double apporach_distance = 0.0;
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) {
            apporach_distance = 0.0075;
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_LATHE_CYLINDER) {
            apporach_distance = 0.035;

        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) {
            apporach_distance = 0.01;

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_2F_GRP) {
            apporach_distance = 0.003;

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP) {
            apporach_distance = 0.01;

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_ELBOW_JOINT) {
            apporach_distance = 0.01;

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_CYLINDER) {
            apporach_distance = 0.035;

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) {
            apporach_distance = 0.0075;

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) {
            apporach_distance = 0.002;

        } else { // 2-finger grasping - Open
            apporach_distance = 0.01;
        }

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        //// TODO: JSON파일의 z margin과 연동해야 함.
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, apporach_distance, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Stacking mode start!
        taskPushBack_startStackingMode(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        ////////////////////////////////////////////////////////////////
        //// Gripper - ungrasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            // taskPushBack_delay_1ms(targetModuleTask, 5000);
            taskPushBack_delay_1ms(targetModuleTask, 1500);
        } else { // 2-finger grasping - Open
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        }

        taskPushBack_countDetachingPose(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 10);

        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -apporach_distance, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // //// J6 0deg --> 블렌딩 모션 반복하다보면 파지 자세에 따라 6축 관절각도 제한을 벗어나는 경우 발생  ? < -360
        // //// absolute J6 motion target --> 6축을 0deg로 명령
        // taskPushBack_jsMoveOnlyJ6(targetModuleTask, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, false); // i: index, false: absolute, true: relative


        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.1, 0, 0, 0}, acc_move, vel_move, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Default JS position
        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);


        //// Stacking mode stop!
        taskPushBack_stopStackingMode(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        ////////////////////////////////////////////////////////////////

    };


    auto makeSingleGripperMachineTendingLatheLoadingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 2.0; vel_move = 1.0;
            acc_contact = 0.2; vel_contact = 0.1;
        }
        //// Cylinder demo - single gripper
        // Feeding position
        // CS: 0.05304, -0.99463, 0.45220, 179.187, -0.172, -179.921
        taskPushBack_jsMove(targetModuleTask, {82.911, -60.350, 49.772, -78.497, -90.467, -96.737}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // Insertion - 투입 직전 위치로부터 30mm 떨어진 곳에서 88mm 삽입 (펙은 58mm 삽입), pnematic chuck
        // Insertion - 투입 직전 위치로부터 30mm 떨어진 곳에서 55mm 삽입 (펙은 25mm 삽입), Lathe Chuck
        //// Tool frame 기준 approach 자세
        // taskPushBack_csMoveToolFrame(targetModuleTask, {-0.088, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        // taskPushBack_csMoveToolFrame(targetModuleTask, {-0.055, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_csMoveToolFrame(targetModuleTask, {-0.055, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // Feeder ON
        // taskPushBackGripperRelayCmd(targetModuleTask, PNE1_ON);
        taskPushBackPLCModbusCmd(targetModuleTask, LATHE_CHUCK_CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 2500);

        // KORAS gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// CNC 문 바깥으로 (빼는 동작 후, tool 기준 +y로 500mm 이동한 CS 자세)
        taskPushBack_csMoveToolFrame(targetModuleTask, {0.120, 0, -0.010, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0.400, 0, 0, 0, 0}, acc_move, vel_move, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);


        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_CLOSE);
        // taskPushBack_delay_1ms(targetModuleTask, 5000);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        // taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        // taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_ON);
        // taskPushBack_delay_1ms(targetModuleTask, 1000);
    };

    auto makeSingleGripperMachingTendingLatheUnloadingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 2.0; vel_move = 1.0;
            acc_contact = 0.2; vel_contact = 0.1;
        }

        taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 5000);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_ON);
        taskPushBack_delay_1ms(targetModuleTask, 1000);


        ////////////////////////////////////////////////
        //// Cylinder demo - Unloading
        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 6000);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// CNC 문 안으로 (tool 기준 -y로 400mm 이동한 CS 자세)
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, -0.400, 0, 0, 0, 0}, acc_move, vel_move, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);
        taskPushBack_csMoveToolFrame(targetModuleTask, {-0.120, 0, 0.010, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper close 1
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        //// Feeder OFF
        // taskPushBackGripperRelayCmd(targetModuleTask, PNE1_OFF);
        taskPushBackPLCModbusCmd(targetModuleTask, LATHE_CHUCK_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 500);

        //// Gripper close 2
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// 배출
        //// Tool frame 기준 approach 자세
        // taskPushBack_csMoveToolFrame(targetModuleTask, {0.055, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_csMoveToolFrame(targetModuleTask, {0.05, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

    };

    auto makeSingleGripperMachineTendingMillingLoadingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 2.0; vel_move = 1.0;
            acc_contact = 0.2; vel_contact = 0.1;
        }
        //// Cylinder demo - single gripper
        // Feeding position
        // CS: -0.26097, -0.99145, 0.19359, 179.834, -0.120, 135.024 (밀링 척 투입 티칭 자세에서 tool기준 z 50mm 위)
        taskPushBack_jsMove(targetModuleTask, {65.319, -53.969, 76.133, -111.814, -90.255, -69.277}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // Insertion - 투입 직전 위치로부터 50mm 떨어진 곳에서 50mm 삽입, Milling Chuck
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.05, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // KORAS gripper Open
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        // Feeder ON
        // taskPushBackGripperRelayCmd(targetModuleTask, PNE1_ON);
        taskPushBackPLCModbusCmd(targetModuleTask, MILLING_CHUCK_CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 2500);

        //// CNC 문 바깥으로 (빼는 동작 후, tool 기준 +y로 500mm 이동한 CS 자세)
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -0.01, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        taskPushBack_csMove2(targetModuleTask, {-0.10788, -0.57674, 0.48707, 179.483, -0.753, 135.090}, acc_move, vel_move, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_CLOSE);
        // taskPushBack_delay_1ms(targetModuleTask, 5000);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        // taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        // taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_ON);
        // taskPushBack_delay_1ms(targetModuleTask, 1000);
    };

    auto makeSingleGripperMachingTendingMillingUnloadingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 2.0; vel_move = 1.0;
            acc_contact = 0.2; vel_contact = 0.1;
        }

        taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 5000);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_ON);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        ////////////////////////////////////////////////
        //// Cylinder demo - Unloading
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 6000);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// CNC 문 안으로 (tool 기준 -y로 400mm 이동한 CS 자세)
        //// TODO: -0.26097, -0.99145, 0.19359, 179.834, -0.120, 135.024
        taskPushBack_csMove2(targetModuleTask, {-0.26408, -0.99122, 0.19363, 179.726, -0.675, 135.073}, acc_move, vel_move, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);
        taskPushBack_csMoveToolFrame(targetModuleTask, {0.004, -0.004, 0.05, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper close
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        //// Feeder OFF
        taskPushBackPLCModbusCmd(targetModuleTask, MILLING_CHUCK_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 1500);

        //// 배출
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {-0.004, +0.004, -0.05, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeSingleGripperMachineTendingBoltBushLoadingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 2.0; vel_move = 1.0;
            acc_contact = 0.2; vel_contact = 0.1;
        }
        //// Cylinder demo - single gripper
        // Feeding position
        // CS: -0.26545, -0.98927, 0.12231, 179.581, -0.726, 135.085
        taskPushBack_jsMove(targetModuleTask, {65.196, -53.404, 76.241, -112.041, -90.736, -69.459}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // Insertion - 투입 직전 위치로부터 30mm 떨어진 곳에서 55mm 삽입 (펙은 25mm 삽입), Lathe Chuck
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.05, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // KORAS gripper - ungrasp, inner grasping
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        // // Feeder ON
        // taskPushBackPLCModbusCmd(targetModuleTask, MILLING_CHUCK_CLOSE);
        // taskPushBack_delay_1ms(targetModuleTask, 2500);

        //// CNC 문 바깥으로 (빼는 동작 후, tool 기준 +y로 500mm 이동한 CS 자세)
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -0.05, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        taskPushBack_csMove2(targetModuleTask, {-0.10788, -0.57674, 0.42857, 179.483, -0.753, 135.090}, acc_move, vel_move, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_CLOSE);
        // taskPushBack_delay_1ms(targetModuleTask, 5000);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        // taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        // taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_ON);
        // taskPushBack_delay_1ms(targetModuleTask, 1000);
    };

    auto makeSingleGripperMachingTendingBoltBushUnloadingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 2.0; vel_move = 1.0;
            acc_contact = 0.2; vel_contact = 0.1;
        }

        taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 5000);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_ON);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        ////////////////////////////////////////////////
        //// Cylinder demo - Unloading
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 6000);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// CNC 문 안으로
        taskPushBack_jsMove(targetModuleTask, {65.196, -53.404, 76.241, -112.041, -90.736, -69.459}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // Insertion - 투입 직전 위치로부터 30mm 떨어진 곳에서 55mm 삽입 (펙은 25mm 삽입), Lathe Chuck
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.05, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper - grasp, inner grasping
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        // //// Feeder OFF
        // taskPushBackPLCModbusCmd(targetModuleTask, MILLING_CHUCK_OPEN);
        // taskPushBack_delay_1ms(targetModuleTask, 1500);

        //// 배출
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -0.05, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };


    auto makeSingleGripperMachingTendingPartDetachingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        ////////////////////////////////////////////////
        //// Cylinder demo - Unloaded parts detaching

        //////// Blending start
        //// CNC 문 바깥
        if(slow_mode) {
            taskPushBack_csMove2(targetModuleTask, {0.15325, -0.44732, 0.39671, 179.286, 1.072, 179.751}, 0.3, 0.15, false); // false: absolute, true: relative
        } else {
            taskPushBack_csMove2(targetModuleTask, {0.15325, -0.44732, 0.39671, 179.286, 1.072, 179.751}, 2.0, 1.0, false); // false: absolute, true: relative
        }
        taskPushBack_delay_1ms(targetModuleTask, 100);
        //// Detaching 위치 (떨어뜨리는 곳)
        if(slow_mode) {
            taskPushBack_csMove2(targetModuleTask, {-0.34293, -0.44248, 0.18734, 179.278, 1.075, 179.735}, 0.3, 0.15, false); // false: absolute, true: relative
        } else {
            taskPushBack_csMove2(targetModuleTask, {-0.34293, -0.44248, 0.18734, 179.278, 1.075, 179.735}, 2.0, 1.0, false); // false: absolute, true: relative
        }
        taskPushBack_delay_1ms(targetModuleTask, 100);
        //////// Blending end

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 1000);
        //////// Blending end

    };

    auto makePickAndPlaceTaskCylinder = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        ////////////////////////////////////////////////
        //// Welding joint demo
        //// Detaching 위치 (떨어뜨리는 곳)
        if(slow_mode) {
            taskPushBack_csMove2(targetModuleTask, {-0.40899, -0.60293, 0.18714, 179.266, 1.080, 179.736}, 0.3, 0.15, false); // false: absolute, true: relative
        } else {
            taskPushBack_csMove2(targetModuleTask, {-0.40899, -0.60293, 0.18714, 179.266, 1.080, 179.736}, 2.0, 1.0, false); // false: absolute, true: relative
        }
        taskPushBack_delay_1ms(targetModuleTask, 100);
        //////// Blending end

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
        //////// Blending end
    };

    auto makeGripperOpenTask = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag) {
        // Gripper ungrasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 60);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 60);
        } else { // 2-finger grasping - Open
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 60);
        }
        taskPushBack_countDetachingPose(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 10);
    };

    auto makePickAndPlaceTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        ////////////////////////////////////////////////
        // taskPushBack_csMove2(targetModuleTask, {0.37444, -0.06028, 0.30000, -179.780, -0.131, 179.575}, 0.75, 0.5, false); // false: absolute, true: relative

        // JS: 200.137, -84.322, 93.815, -99.295, -90.156, 20.559
        // CS: 0.63939, 0.42011, 0.30040, -179.786, -0.127, 179.576
        taskPushBack_csMove2(targetModuleTask, {0.63939, 0.42011, 0.30000, -179.786, -0.127, 179.576}, 0.75, 0.5, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 1200);
    };

    auto makePickAndPlaceP2PBlendingTaskVer2 = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {

        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;

        ////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////
        //// Blending motion, MotionTag::MOTION_TAG_BLENDING_HOME_TO_GRASP_APPROACH_POSE
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_P2P_POSE);
        taskPushBack_delay_1ms(targetModuleTask, 100);
        ////////////////////////////////////////////////////////////////

        //// Gripper - ungrasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 5000);
        } else { // 2-finger grasping - Open
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        }

        //// J6 0deg --> 블렌딩 모션 반복하다보면 파지 자세에 따라 6축 관절각도 제한을 벗어나는 경우 발생  ? < -360
        //// absolute J6 motion target --> 6축을 0deg로 명령
        // taskPushBack_jsMoveOnlyJ6(targetModuleTask, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, false); // i: index, false: absolute, true: relative
        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

    };

    auto makePickAndPlaceP2PBlendingTaskVer2TestToBeDeleted = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {

        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;

        ////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////
        //// Blending motion, MotionTag::MOTION_TAG_BLENDING_HOME_TO_GRASP_APPROACH_POSE
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_P2P_POSE);
        taskPushBack_delay_1ms(targetModuleTask, 100);
        ////////////////////////////////////////////////////////////////

        //// J6 0deg --> 블렌딩 모션 반복하다보면 파지 자세에 따라 6축 관절각도 제한을 벗어나는 경우 발생  ? < -360
        //// absolute J6 motion target --> 6축을 0deg로 명령
        taskPushBack_jsMoveOnlyJ6(targetModuleTask, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, false); // i: index, false: absolute, true: relative


        //// Zig Approach Pose
        taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0.05, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_DETECTED_ZIG_POSE);
        taskPushBack_delay_1ms(targetModuleTask, 50);

        // 티칭 위치보다 위에서 떨어뜨리도록 (다른 자세에서 충돌 발생하는 것을 방지하기 위함)
        //// TODO: 그리퍼와 지그 환경과의 충돌을 방지해야 함.
        taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0.002, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_DETECTED_ZIG_POSE);
        taskPushBack_delay_1ms(targetModuleTask, 50);




        //// Gripper - ungrasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 5000);
        } else { // 2-finger grasping - Open
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        }

        //// Zig Approach Pose
        taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0.05, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_DETECTED_ZIG_POSE);
        taskPushBack_delay_1ms(targetModuleTask, 50);

    };


    auto makePickAndPlaceStackingBlendingTaskVer2 = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, uint16_t motion_tag, bool slow_mode) {

        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;

        ////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////
        //// Blending motion, MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_JSON_POSE
        //// Blending motion, MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_DETECTED_ZIG_POSE
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, motion_tag);
        taskPushBack_delay_1ms(targetModuleTask, 100);
        ////////////////////////////////////////////////////////////////
    };



    auto makePickAndPlaceVacuumTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        ////////////////////////////////////////////////
        taskPushBack_csMove2(targetModuleTask, {0.37444, -0.06028, 0.30000, -179.780, -0.131, 179.575}, 0.75, 0.5, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper Open
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 1200);
    };

    auto makeInnerPickAndPlaceTask = [&] (std::vector<UnitTask>& targetModuleTask) {
        ////////////////////////////////////////////////
        //// Welding joint demo
        // Feeding position
        taskPushBack_jsMove(targetModuleTask, {167.267, -70.608, -110.328, -92.411, 88.511, 176.306}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // KORAS gripper Close
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

    };

    //// Module task - grasping
    auto makeDualGripperGraspingTask = [&] (std::vector<UnitTask>& targetModuleTask) {

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        // 상자 위 투입 직전 position
        taskPushBack_jsMove(targetModuleTask, {65.435, -108.792, -58.861, -146.723, 72.872, 73.338}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_doGrasping_binPicking(targetModuleTask, 0.1, 0.1); // Move to grasping pose, (acc, vel)

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        //// TODO: JSON파일의 z margin과 연동해야 함.
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.04, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.050, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper additional close command 1
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 400);

        // 상자 위 투입 직전 position
        taskPushBack_jsMove(targetModuleTask, {65.435, -108.792, -58.861, -146.723, 72.872, 73.338}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // Gripper #1 default position
        taskPushBack_jsMove(targetModuleTask, {21.594, -92.093, -83.072, -136.916, 102.995, 107.571}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
        //// Gripper additional close command 2
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 400);
        // //// Check grasping success/failure
        // taskPushBack_taskRecog_KORASGripperCheckGrasping(targetModuleTask);
        // taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeDualGripperMachingTendingUnloadingTask = [&] (std::vector<UnitTask>& targetModuleTask) {
        ////////////////////////////////////////////////
        //// Cylinder demo - Unloading
        // Dual gripper pose change - gripper #2 default position
        taskPushBack_jsMove(targetModuleTask, {21.595, -92.094, -83.091, -136.894, 102.985, -73.187}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // Set TCP
        taskPushBack_selectTCP(targetModuleTask, 6); // Right hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper driver change
        taskPushBack_setGripperDriver(targetModuleTask, 2); // Driver #2 Right hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        // Unloading
        taskPushBack_jsMove(targetModuleTask, {17.468, -110.583, -68.974, -134.589, 103.231, -77.610}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_csMove2(targetModuleTask, {0, 0, -0.04, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper close 1
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        //// Feeder OFF
        // taskPushBack_gripper_pne1(targetModuleTask, false);
        taskPushBackGripperRelayCmd(targetModuleTask, PNE1_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper close 2
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// 배출
        // taskPushBack_csMove2(targetModuleTask, {-0.088, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 100);
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0.088, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

    };

    auto makeDualGripperMachingTendingLoadingTask = [&] (std::vector<UnitTask>& targetModuleTask) {
        ////////////////////////////////////////////////
        //// Cylinder demo - Loading
        // Set TCP
        taskPushBack_selectTCP(targetModuleTask, 5); // Left hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper driver change
        taskPushBack_setGripperDriver(targetModuleTask, 1); // Driver #1 Left hand (for loading)
        taskPushBack_delay_1ms(targetModuleTask, 200);


        // Feeding position
        taskPushBack_jsMove(targetModuleTask, {19.469, -103.218, -82.595, -129.301, 104.891, 104.288}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // Insertion - 투입 직전 위치로부터 30mm 떨어진 곳에서 88mm 삽입 (펙은 58mm 삽입)
        // taskPushBack_csMove2(targetModuleTask, {0.088, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 100);
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {-0.088, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // Feeder ON
        // taskPushBack_gripper_pne1(targetModuleTask, true);
        taskPushBackGripperRelayCmd(targetModuleTask, PNE1_ON);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_csMove2(targetModuleTask, {-0.050, 0, 0.060, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 400);

    };

    auto makeDualGripperMachingTendingPartDetachingTask = [&] (std::vector<UnitTask>& targetModuleTask) {
        ////////////////////////////////////////////////
        //// Cylinder demo - Unloaded parts detaching

        // Part detaching position
        taskPushBack_jsMove(targetModuleTask, {-0.510, -113.416, -69.432, -134.141, 92.437, -87.860}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // Set TCP
        taskPushBack_selectTCP(targetModuleTask, 6); // Right hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper driver change
        taskPushBack_setGripperDriver(targetModuleTask, 2); // Driver #2 Right hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // Middle position
        taskPushBack_jsMove(targetModuleTask, {25.644, -77.373, -96.711, -121.404, 49.098, 36.153}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    //// Blending module task
    auto makeBlendingInit2RegraspingZigTask = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            // xd = 0.75;
            // xdd = 1.5;
            // waypoint_xd = 0.5;
            // radius = 30; // radius percent %
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_HOME_TO_REGRASP_ZIG);
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeBlendingRegraspingZig2CNCTask = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            // xd = 0.75;
            // xdd = 1.5;
            // waypoint_xd = 0.25;
            // radius = 30; // radius percent %
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_REGRASP_ZIG_TO_CNC_INDOOR);
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeBlendingCNC2DetachTask = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            xd = 0.75;
            xdd = 1.0;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_CNC_INDOOR_TO_DETACH);
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    //// Module task - grasping
    auto makeBlendingCNCDetach2InitTask = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            // xd = 0.75;
            // xdd = 1.5;
            // waypoint_xd = 0.5;
            // radius = 30; // radius percent %
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_DETACH_TO_HOME_1);
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeBlendingInit2CNCMillingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            // xd = 0.75;
            // xdd = 1.5;
            // waypoint_xd = 0.5;
            // radius = 30; // radius percent %
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG, MotionTag::MOTION_TAG_BLENDING_HOME_TO_CNC_INDOOR);
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeBlendingCNCMillingDetach2InitTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            // xd = 0.75;
            // xdd = 1.5;
            // waypoint_xd = 0.5;
            // radius = 30; // radius percent %
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG, MotionTag::MOTION_TAG_BLENDING_DETACH_TO_HOME_1);
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };


    auto makeIfCheckTipChangingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        float tip_changing_insertion_distance = 0.025; // 팁 장착위치로부터 떨어진 거리
        float tip_changing_exit_from_tip_home_distance = 0.05; // 팁 거치대의 내부에서 퇴장하기 위한 거리

        double acc_target = 0.5;
        double vel_target = 0.25;

        //// Check matching finished and tip changing parameters update
        taskPushBack_checkMatchingFinishedAndTipChanging(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// Task Flag ON
        taskPushBack_setTipChangingFlag(targetModuleTask, true);

        ////////////////////////////
        //// Detaching
        ////////////////////////////
        taskPushBack_taskRecog_TipChangingDefaultDetach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// Gripper
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        ////// a) JS position - 지그 앞 5cm
        taskPushBack_taskRecog_TipChangingDetach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// TCP setting: TCP default - 혼동을 막기 위해, 로봇의 기본으로 세팅된 TCP의 방위를 사용
        // Set TCP (#1~) master(#7)
        taskPushBack_selectTCP(targetModuleTask, 7); // TCP default

        ////// b) 거치대 내부 입장
        //// Tool frame 기준 - 툴 거치대의 내부에 입장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, -tip_changing_exit_from_tip_home_distance, 0, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// Tool frame 기준 - 툴 탈착 후 퇴장
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -tip_changing_insertion_distance, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        ////////////////////////////
        //// Attaching
        ////////////////////////////
        taskPushBack_taskRecog_TipChangingDefaultAttach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
        ////// a) JS position - 장착 위치 앞 2.5cm
        taskPushBack_taskRecog_TipChangingAttach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
        //// Tool frame 기준 - 팁 장착
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, +tip_changing_insertion_distance, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
        ////// b) 장착 후, 거치대 내부 퇴장
        //// Tool frame 기준 - 툴 거치대의 내부에서 퇴장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, tip_changing_exit_from_tip_home_distance, 0, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_taskRecog_TipChangingDefaultDetach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_taskRecog_TipChangingSetTCP(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// Task Flag OFF
        taskPushBack_setTipChangingFlag(targetModuleTask, false);

    };









    // bool is_slow_mode = true;
    //// Machine tending - Lathe, single gripper, CNC Chuck(PLC modbus)
    //// 모든 모션에 대해 블렌딩 적용
    makeInitializeTCP (module_task_bin_picking_single_machine_tending_lathe_, 1);  // tcp#
    taskPushBack_jsMove(module_task_bin_picking_single_machine_tending_lathe_, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
    // KORAS gripper Open
    taskPushBack_taskRecog_KORASGripperCmd(module_task_bin_picking_single_machine_tending_lathe_);
    taskPushBack_delay_1ms(module_task_bin_picking_single_machine_tending_lathe_, 2000);
    taskPushBack_irl_grp(module_task_bin_picking_single_machine_tending_lathe_, KR_GRP::CLOSE, 0, 2);
    taskPushBack_delay_1ms(module_task_bin_picking_single_machine_tending_lathe_, 2000);
    taskPushBack_taskRecog_KORASGripperCmd(module_task_bin_picking_single_machine_tending_lathe_);
    taskPushBack_delay_1ms(module_task_bin_picking_single_machine_tending_lathe_, 600);

    // makeSingleGripperGraspingTask (module_task_bin_picking_single_machine_tending_lathe_, is_slow_mode);

    // //// 파지 이후
    // taskPushBack_irl_grp(module_task_bin_picking_single_machine_tending_lathe_, KR_GRP::CLOSE, 0, main_grp);
    // taskPushBack_delay_1ms(module_task_bin_picking_single_machine_tending_lathe_, 200);
    // // taskPushBack_csMove (module_task_bin_picking_single_machine_tending_lathe_, x1);
    // taskPushBack_csMove (module_task_bin_picking_single_machine_tending_lathe_, x2);
    // taskPushBack_csMove (module_task_bin_picking_single_machine_tending_lathe_, x3);
    // taskPushBack_irl_grp(module_task_bin_picking_single_machine_tending_lathe_, KR_GRP::POS_CTRL, grp_default_width, main_grp);
    // taskPushBack_delay  (module_task_bin_picking_single_machine_tending_lathe_, grp_move_delay);
    // taskPushBack_tcpMove(module_task_bin_picking_single_machine_tending_lathe_, {0, 0, -0.07, 0, 0, 0});
    // taskPushBack_csMove (module_task_bin_picking_single_machine_tending_lathe_, x4);
    // taskPushBack_csMove (module_task_bin_picking_single_machine_tending_lathe_, x5);
    // taskPushBack_irl_grp(module_task_bin_picking_single_machine_tending_lathe_, KR_GRP::CLOSE, 0, main_grp);
    // taskPushBack_delay  (module_task_bin_picking_single_machine_tending_lathe_, grp_move_delay);
    // taskPushBack_csMove (module_task_bin_picking_single_machine_tending_lathe_, {0, 0, 0.15, 0, 0, 0}, true);
    // taskPushBack_csMove (module_task_bin_picking_single_machine_tending_lathe_, {0.05, 0.45, 0.1, 0, 0, 0}, true);
    // taskPushBack_csMove (module_task_bin_picking_single_machine_tending_lathe_, x6);




    //// Machine tending - Milling, single gripper, CNC Chuck(PLC modbus)
    //// 모든 모션에 대해 블렌딩 적용
    // makeInitializeTCP (module_task_bin_picking_single_machine_tending_milling_, 3);  // tcp#
    // makeInitializePLCChuck (module_task_bin_picking_single_machine_tending_milling_);
    // makeZIVIDScanningTask (module_task_bin_picking_single_machine_tending_milling_); // 초기 1회만 수행됨.
    // makeSingleGripperGraspingBlendingTask (module_task_bin_picking_single_machine_tending_milling_, BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG, is_slow_mode);
    // makeRegraspingTaskZig(module_task_bin_picking_single_machine_tending_milling_, BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG, is_slow_mode);
    // makeBlendingRegraspingZig2CNCTask (module_task_bin_picking_single_machine_tending_milling_, BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG, is_slow_mode);
    // makeSingleGripperMachineTendingMillingLoadingTask (module_task_bin_picking_single_machine_tending_milling_, is_slow_mode);
    // makeZIVIDScanningAndMatchingTask (module_task_bin_picking_single_machine_tending_milling_); // Topic
    // makeSingleGripperMachingTendingMillingUnloadingTask (module_task_bin_picking_single_machine_tending_milling_, is_slow_mode);
    // makeBlendingCNC2DetachTask (module_task_bin_picking_single_machine_tending_milling_, BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG, is_slow_mode);
    // makeGripperOpenTask (module_task_bin_picking_single_machine_tending_milling_, BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG);
    //// TODO: 마지막 종료 시에 home 포즈로 돌아오도록 처리
    //// TODO: 마지막 종료 시에 home 포즈로 돌아오도록 처리
    //// TODO: 마지막 종료 시에 home 포즈로 돌아오도록 처리

    //// 로보월드 사각펙 밀링 데모, 초기만 블렌딩 모션
    int main_grp = 2;
    int sub_grp  = 1;
    makeInitializeTCP (module_task_bin_picking_single_machine_tending_milling_, 3);  // tcp#
    makeInitializePLCChuck (module_task_bin_picking_single_machine_tending_milling_);
    makeZIVIDScanningTask (module_task_bin_picking_single_machine_tending_milling_); // 초기 1회만 수행됨.
    makeSingleGripperGraspingBlendingTask (module_task_bin_picking_single_machine_tending_milling_, BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG, is_slow_mode);
    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0, 0, 0.04, 0, 0, 0}, true);
    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0.02087, -0.47059, 0.25000, -180.000, -0.000, 90.000}, false);
    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0.25877, -0.27009, 0.28664, -179.999, -0.000, 90.000}, false);
    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0.41281, -0.10501, 0.25544, 134.624, -3.796, 87.636}, false);
    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0.41281, -0.10501, 0.09967, 134.624, -3.796, 87.636}, false);
    taskPushBack_irl_grp(module_task_bin_picking_single_machine_tending_milling_, KR_GRP::VACUUM_OFF, 0, main_grp);
    taskPushBack_delay_1ms(module_task_bin_picking_single_machine_tending_milling_, 1000);
    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0.40467, -0.10382, 0.08544, 134.071, -1.424, 90.372}, false);
    taskPushBack_irl_grp(module_task_bin_picking_single_machine_tending_milling_, KR_GRP::VACUUM_ON, 0, main_grp);
    taskPushBack_delay_1ms(module_task_bin_picking_single_machine_tending_milling_, 1000);
    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0.40467, -0.10382, 0.25544, 134.071, -1.424, 90.372}, false);
    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0.33176, 0.06407, 0.30315, 180.000, 0.000, 135.000}, false);
    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0.36013, 0.55513, 0.16367, 179.601, -2.639, 135.647}, false);
    makeZIVIDScanningAndMatchingTask (module_task_bin_picking_single_machine_tending_milling_); // Topic

    // 투입 정답
    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0.36012, 0.55513, 0.11260, 179.601, -2.639, 135.647}, false);
    taskPushBack_irl_grp(module_task_bin_picking_single_machine_tending_milling_, KR_GRP::VACUUM_OFF, 0, main_grp);
    taskPushBack_delay_1ms(module_task_bin_picking_single_machine_tending_milling_, 600);
    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0, 0, 0.02, 0, 0, 0}, true);
    taskPushBackPLCModbusCmd(module_task_bin_picking_single_machine_tending_milling_, MILLING_CHUCK_CLOSE);
    taskPushBack_delay_1ms(module_task_bin_picking_single_machine_tending_milling_, 3000);
    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0.33176, 0.06407, 0.30315, 180.000, 0.000, 135.000}, false);

    // 배출 정답
    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0.36013, 0.55513, 0.16367, 179.601, -2.639, 135.647}, false);
    taskPushBackPLCModbusCmd(module_task_bin_picking_single_machine_tending_milling_, MILLING_CHUCK_OPEN);
    taskPushBack_delay_1ms(module_task_bin_picking_single_machine_tending_milling_, 1000);
    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0.36313, 0.55513, 0.11035, 179.601, -2.639, 135.647}, false);
    taskPushBack_irl_grp(module_task_bin_picking_single_machine_tending_milling_, KR_GRP::VACUUM_ON, 0, main_grp);
    taskPushBack_delay_1ms(module_task_bin_picking_single_machine_tending_milling_, 600);

    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0, 0, 0.08, 0, 0, 0}, true);
    // 배출 후 바깥
    taskPushBack_jsMove (module_task_bin_picking_single_machine_tending_milling_, {27.190, 100.178, -181.535, -17.363, -141.008, 61.713}, false);
    // Stacking default
    taskPushBack_jsMove (module_task_bin_picking_single_machine_tending_milling_, {-180, 0, 0, 0, 0, 0}, true);
    taskPushBack_jsMove (module_task_bin_picking_single_machine_tending_milling_, {-181.496, 80.687, -176.091, 31.692, -116.969, 134.821}, false);
    // Stacking
    taskPushBack_taskJSON_csMoveMotionTag(module_task_bin_picking_single_machine_tending_milling_, {0, 0, +0.05, 0, 0, 0}, BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG, MotionTag::MOTION_TAG_JSON_CSMOVE_STACKING);
    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0, 0, -0.05, 0, 0, 0}, true);
    makeStackingTask (module_task_bin_picking_single_machine_tending_milling_, BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG, is_slow_mode);
    taskPushBack_csMove (module_task_bin_picking_single_machine_tending_milling_, {0, 0, 0.05, 0, 0, 0}, true);
    taskPushBack_jsMove (module_task_bin_picking_single_machine_tending_milling_, {-181.496, 80.687, -176.091, 31.692, -116.969, 134.821}, false);
    taskPushBack_jsMove (module_task_bin_picking_single_machine_tending_milling_, {-59.528, 118.811, -200.909, -43.154, -117.118, 17.200}, false);
    ////

    // //// Maching tending (inner plane grasping - bolt bush)
    // makeInitializeTCP (module_task_bin_picking_machine_tending_bolt_bush_, 4);  // tcp#
    // makeZIVIDScanningTask (module_task_bin_picking_machine_tending_bolt_bush_); // 초기 1회만 수행됨.
    // makeSingleGripperGraspingBlendingTask (module_task_bin_picking_machine_tending_bolt_bush_, BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH, is_slow_mode);
    // makeZIVIDScanningAndMatchingTask (module_task_bin_picking_machine_tending_bolt_bush_); // Topic
    // makeSingleGripperMachineTendingBoltBushLoadingTask (module_task_bin_picking_machine_tending_bolt_bush_, is_slow_mode);
    // makeSingleGripperMachingTendingBoltBushUnloadingTask (module_task_bin_picking_machine_tending_bolt_bush_, is_slow_mode);
    // makeBlendingCNC2DetachTask (module_task_bin_picking_machine_tending_bolt_bush_, BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH, is_slow_mode);
    // makeStackingTask (module_task_bin_picking_machine_tending_bolt_bush_, BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH, is_slow_mode);


    //// Pick-and-place - t joint

    // // Pick-and-place - t joint
    // // 2F gripper (No Blending)
    // makeInitializeTCPAndGripper (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_, 3, 1);
    // makeMoveJSDefaultPosition (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_);
    // makeMoveJSMiddlePosition(module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_);
    // makeSingleGripperGraspingTask (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_, is_slow_mode);
    // taskPushBack_jsMove(module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
    // taskPushBack_delay_1ms(module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_, 20);
    // makeZIVIDScanningAndMatchingTask (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_); // Topic
    // makePickAndPlaceTask (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_, is_slow_mode);
    // makeMoveJSDefaultPosition (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_);
    // makeCheckMatchingFinishedTaskVer2 (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_);

    // Pick-and-place - t joint
    // [2F gripper] (With Blending)
    makeInitializeTCPAndGripper (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_, 3, 1);
    makePickAndPlaceGraspingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_, BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_2F_GRP, is_slow_mode);


    if(0) { //// Point 2 Point Task
        makePickAndPlaceP2PBlendingTaskVer2 (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_, BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_2F_GRP, is_slow_mode);
    } else { //// Stacking Task
        uint16_t motion_tag_t_joint = MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_DETECTED_ZIG_POSE;
        makePickAndPlaceStackingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_, BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_2F_GRP, motion_tag_t_joint, is_slow_mode);
        makePickAndPlaceDoStackingTaskVer2 (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_, BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_2F_GRP, is_slow_mode);
    }
    makeCheckMatchingFinishedTaskVer2 (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_);


    // // Vacuum 1cup
    // makeInitializeTCPAndGripper (module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_, 15, 1);
    // makeMoveJSDefaultPosition (module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_);
    // makeMoveJSMiddlePosition(module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_);
    // makeSingleVacuumGripperGraspingTask (module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_, is_slow_mode);
    // taskPushBack_jsMove(module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
    // taskPushBack_delay_1ms(module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_, 100);
    // makeZIVIDScanningAndMatchingTask (module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_); // Topic
    // makePickAndPlaceVacuumTask (module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_, is_slow_mode);
    // makeMoveJSDefaultPosition (module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_);
    // makeCheckMatchingFinishedTaskVer2 (module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_);


    // Pick-and-place - t joint
    // [Vacuum 1cup] (With Blending)
    makeInitializeTCPAndGripper (module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_, 15, 1);
    makePickAndPlaceGraspingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_, BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP, is_slow_mode);
    makePickAndPlaceP2PBlendingTaskVer2 (module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_, BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP, is_slow_mode);
    makeCheckMatchingFinishedTaskVer2 (module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_);

    // Pick-and-place - bolt bush (inner plane grasping)
    makeInitializeTCPAndGripper (module_task_bin_picking_pick_and_place_bolt_bush_, 4, 1);
    makePickAndPlaceGraspingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_bolt_bush_, BPDemoType::DEMO_BP_PICK_BOLT_BUSH, is_slow_mode);
    if(0) { //// Point 2 Point Task
        makePickAndPlaceP2PBlendingTaskVer2 (module_task_bin_picking_pick_and_place_bolt_bush_, BPDemoType::DEMO_BP_PICK_BOLT_BUSH, is_slow_mode);
    } else { //// Stacking Task
        uint16_t motion_tag_bolt_bush = MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_JSON_POSE;
        makePickAndPlaceStackingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_bolt_bush_, BPDemoType::DEMO_BP_PICK_BOLT_BUSH, motion_tag_bolt_bush, is_slow_mode);
        makePickAndPlaceDoStackingTaskVer2 (module_task_bin_picking_pick_and_place_bolt_bush_, BPDemoType::DEMO_BP_PICK_BOLT_BUSH, is_slow_mode);
    }
    makeCheckMatchingFinishedTaskVer2 (module_task_bin_picking_pick_and_place_bolt_bush_);

    //// Pick-and-place - eb joint
    // [2F gripper] (No Blending)
    // makeInitializeTCPAndGripper (module_task_bin_picking_pick_and_place_eb_joint_, 2, 1);
    // makeMoveJSDefaultPosition (module_task_bin_picking_pick_and_place_eb_joint_);
    // makeZIVIDScanningTask (module_task_bin_picking_pick_and_place_eb_joint_);
    // makeSingleGripperGraspingTask (module_task_bin_picking_pick_and_place_eb_joint_, is_slow_mode);
    // makeMoveJSMiddlePosition(module_task_bin_picking_pick_and_place_eb_joint_);
    // makeZIVIDScanningAndMatchingTask (module_task_bin_picking_pick_and_place_eb_joint_); // Topic
    // makePickAndPlaceTask (module_task_bin_picking_pick_and_place_eb_joint_, is_slow_mode);
    // makeMoveJSDefaultPosition (module_task_bin_picking_pick_and_place_eb_joint_);

    //// Pick-and-place - eb joint
    // [2F gripper] (With Blending)
    makeInitializeTCPAndGripper (module_task_bin_picking_pick_and_place_eb_joint_, 2, 1);
    makePickAndPlaceGraspingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_eb_joint_, BPDemoType::DEMO_BP_PICK_WELDING_ELBOW_JOINT, is_slow_mode);
    makePickAndPlaceP2PBlendingTaskVer2 (module_task_bin_picking_pick_and_place_eb_joint_, BPDemoType::DEMO_BP_PICK_WELDING_ELBOW_JOINT, is_slow_mode);
    makeCheckMatchingFinishedTaskVer2 (module_task_bin_picking_pick_and_place_eb_joint_);



    // Pick-and-place - cylinder
    // [2F gripper] (With Blending)
    makeInitializeTCPAndGripper (module_task_bin_picking_pick_and_place_cylinder_, 13, 1); // cylinder with tip-changing (round tip)
    makePickAndPlaceGraspingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_cylinder_, BPDemoType::DEMO_BP_PICK_CYLINDER, is_slow_mode);
    if(1) { //// Point 2 Point Task
        makePickAndPlaceP2PBlendingTaskVer2 (module_task_bin_picking_pick_and_place_cylinder_, BPDemoType::DEMO_BP_PICK_CYLINDER, is_slow_mode);
    } else { //// Stacking Task
        uint16_t motion_tag_cylinder = MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_JSON_POSE;
        makePickAndPlaceStackingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_cylinder_, BPDemoType::DEMO_BP_PICK_CYLINDER, motion_tag_cylinder, is_slow_mode);
        makePickAndPlaceDoStackingTaskVer2 (module_task_bin_picking_pick_and_place_cylinder_, BPDemoType::DEMO_BP_PICK_CYLINDER, is_slow_mode);
    }
    makeCheckMatchingFinishedTaskVer2 (module_task_bin_picking_pick_and_place_cylinder_);



    // Pick-and-place - square peg
    // [Vacuum] (With Blending)
    makeInitializeTCPAndGripper (module_task_bin_picking_pick_and_place_square_peg_, 11, 1); // square peg with vacuum gripper(4 vacuum)
    makePickAndPlaceGraspingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_square_peg_, BPDemoType::DEMO_BP_PICK_SQUARE_PEG, is_slow_mode);

    if(1) { //// Point 2 Point Task
        makePickAndPlaceP2PBlendingTaskVer2 (module_task_bin_picking_pick_and_place_square_peg_, BPDemoType::DEMO_BP_PICK_SQUARE_PEG, is_slow_mode);
    } else { //// Stacking Task
        //// NOTICE: Test Zig Poise p2p
        // makePickAndPlaceP2PBlendingTaskVer2TestToBeDeleted (module_task_bin_picking_pick_and_place_square_peg_, BPDemoType::DEMO_BP_PICK_SQUARE_PEG, is_slow_mode);
        uint16_t motion_tag_square_peg = MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_DETECTED_ZIG_POSE;
        makePickAndPlaceStackingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_square_peg_, BPDemoType::DEMO_BP_PICK_SQUARE_PEG, motion_tag_square_peg, is_slow_mode);
        makePickAndPlaceDoStackingTaskVer2 (module_task_bin_picking_pick_and_place_square_peg_, BPDemoType::DEMO_BP_PICK_SQUARE_PEG, is_slow_mode);
    }
    makeCheckMatchingFinishedTaskVer2 (module_task_bin_picking_pick_and_place_square_peg_);




    // //// Machine tending - dual gripper
    // makeInitializeTCPAndGripper (module_task_bin_picking_dual_machine_tending_, 5, 1);  // tcp#, driver#
    // makeMoveJSDefaultPosition (module_task_bin_picking_dual_machine_tending_);
    // makeZIVIDScanningTask (module_task_bin_picking_dual_machine_tending_);
    // makeDualGripperGraspingTask (module_task_bin_picking_dual_machine_tending_);
    // makeDualGripperMachingTendingUnloadingTask (module_task_bin_picking_dual_machine_tending_);
    // makeDualGripperMachingTendingLoadingTask (module_task_bin_picking_dual_machine_tending_);
    // makeDualGripperMachingTendingPartDetachingTask (module_task_bin_picking_dual_machine_tending_);
    // makeMoveJSDefaultPosition (module_task_bin_picking_dual_machine_tending_);

    makeBinPickingCylinderDualGripperTaskList();

    //// LOG
    ROS_LOG_INFO("[The task has been created successfully]: %s", __FUNCTION__);
}

///////////////////////////////////////
//// Tool Changing
void TaskPlanner::makeToolChangeMotionTaskList() {

    module_task_tool_changing_attach_motion_.clear();
    module_task_tool_changing_detach_motion_.clear();

    module_task_dual_tool_initialize_.clear();


    //// Module task - grasping

    auto makeMoveJSDefaultPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        //// Single gripper
        // taskPushBack_jsMove(targetModuleTask, {-90, 110, -210, 0, -80, 0}, false); // i: index, false: absolute, true: relative
        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeInitializeTCP = [&] (std::vector<UnitTask>& targetModuleTask, unsigned int tcp_idx) {
        // Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6) , master(#7)
        taskPushBack_selectTCP(targetModuleTask, tcp_idx); // Left hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 200);
    };

    auto makeInitializeTCPAndGripper = [&] (std::vector<UnitTask>& targetModuleTask, unsigned int tcp_idx, unsigned int grp_driver_idx) {
        // Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6) , master(#7)
        taskPushBack_selectTCP(targetModuleTask, tcp_idx); // Left hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper driver change(#1 or #2)
        taskPushBack_setGripperDriver(targetModuleTask, grp_driver_idx); // Driver #1 Left hand (for loading)
        taskPushBack_delay_1ms(targetModuleTask, 200);
    };

    auto makeSetToolSlaveTCPAndInitializeGripper = [&] (std::vector<UnitTask>& targetModuleTask) {
        // Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6) , master(#7)
        taskPushBack_taskRecog_selectTCP(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        taskPushBack_initializeGripper(targetModuleTask); // Gripper driver (#1 or #2)
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };


    auto makeMoveAttachingJSDefaultPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        // taskPushBack_jsMove(targetModuleTask, {76.125, -105.396, -109.606, 34.502, 104.972, 88.428}, false); // i: index, false: absolute, true: relative
        taskPushBack_taskRecog_ToolChangingDefaultAttach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    auto makeMoveDetachingJSDefaultPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        // taskPushBack_jsMove(targetModuleTask, {78.279, -95.776, -102.591, -71.229, 88.616, 77.144}, false); // i: index, false: absolute, true: relative
        taskPushBack_taskRecog_ToolChangingDefaultDetach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    auto makeToolAttachingCSTask = [&] (std::vector<UnitTask>& targetModuleTask) {

        // float tool_changing_insertion_distance = 0.04; // 툴 장착위치로부터 떨어진 거리, 0.5mm를 더 삽입하도록 25.5mm 기입
        // float tool_changing_exit_from_tool_home_distance = 0.10; // 툴 거치대의 내부에서 퇴장하기 위한 거리
        std::vector<double> tool_changing_insertion_relative = {0, 0.04, 0, 0 ,0, 0};
        std::vector<double> tool_changing_home = {0, 0, 0, 0, 0, 0};
        std::vector<double> tool_changing_exit_from_tool_home_relative = {-0.1, 0, 0, 0, 0, 0};
        bool is_tool_attaching = true;
        //// a) CS pose - 장착 위치 앞 4.0cm (base 기준 -y방향, tool 기준 +z방향)
        taskPushBack_taskRecog_ToolChangingGoal_csMove(targetModuleTask, is_tool_attaching, tool_changing_insertion_relative);
        taskPushBack_delay_1ms(targetModuleTask, 60);
        //// b) CS pose - 장착 위치 (Goal)
        ref_unit_task_.vel_cs /= 5;
        ref_unit_task_.acc_cs /= 5;
        taskPushBack_taskRecog_ToolChangingGoal_csMove(targetModuleTask, is_tool_attaching, tool_changing_home);
        taskPushBack_delay_1ms(targetModuleTask, 60);
        //// c) 장착 후, 거치대 내부 퇴장 - 장착 위치 옆 10.0cm (base 기준 -x방향, tool 기준 +x방향)
        taskPushBack_taskRecog_ToolChangingGoal_csMove(targetModuleTask, is_tool_attaching, tool_changing_exit_from_tool_home_relative);
        taskPushBack_delay_1ms(targetModuleTask, 60);
        ref_unit_task_.vel_cs *= 5;
        ref_unit_task_.acc_cs *= 5;
    };

    auto makeToolDetachingCSTask = [&] (std::vector<UnitTask>& targetModuleTask) {

        // float tool_changing_insertion_distance = 0.04; // 툴 장착위치로부터 떨어진 거리, 0.5mm를 더 삽입하도록 25.5mm 기입
        // float tool_changing_exit_from_tool_home_distance = 0.10; // 툴 거치대의 내부에서 퇴장하기 위한 거리
        std::vector<double> tool_changing_exit_from_tool_home_relative = {-0.1, 0, 0, 0, 0, 0};
        std::vector<double> tool_changing_home = {0, 0, 0, 0, 0, 0};
        std::vector<double> tool_changing_insertion_relative = {0, 0.04, 0, 0 ,0, 0};
        bool is_tool_attaching = false;
        //// c) 장착 후, 거치대 내부 퇴장 - 장착 위치 옆 10.0cm (base 기준 -x방향, tool 기준 +x방향)
        taskPushBack_taskRecog_ToolChangingGoal_csMove(targetModuleTask, is_tool_attaching, tool_changing_exit_from_tool_home_relative);
        taskPushBack_delay_1ms(targetModuleTask, 60);
        //// b) CS pose - 장착 위치 (Goal)
        ref_unit_task_.vel_cs /= 5;
        ref_unit_task_.acc_cs /= 5;
        taskPushBack_taskRecog_ToolChangingGoal_csMove(targetModuleTask, is_tool_attaching, tool_changing_home);
        taskPushBack_delay_1ms(targetModuleTask, 60);
        //// a) CS pose - 장착 위치 앞 4.0cm (base 기준 -y방향, tool 기준 +z방향)
        taskPushBack_taskRecog_ToolChangingGoal_csMove(targetModuleTask, is_tool_attaching, tool_changing_insertion_relative);
        taskPushBack_delay_1ms(targetModuleTask, 60);
        ref_unit_task_.vel_cs *= 5;
        ref_unit_task_.acc_cs *= 5;
    };

    auto makeToolAttachingTCPMoveTask = [&] (std::vector<UnitTask>& targetModuleTask) {

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        // float tool_changing_insertion_distance = 0.025; // 툴 장착위치로부터 떨어진 거리
        // float tool_changing_insertion_distance = 0.0255; // 툴 장착위치로부터 떨어진 거리, 0.5mm를 더 삽입하도록 25.5mm 기입
        float tool_changing_insertion_distance = 0.0265; // 툴 장착위치로부터 떨어진 거리, 0.5mm를 더 삽입하도록 25.5mm 기입
        float tool_changing_exit_from_tool_home_distance = 0.09; // 툴 거치대의 내부에서 퇴장하기 위한 거리

        // double acc_target = 0.075;
        // double vel_target = 0.030;
        // double acc_target = 0.3;
        // double vel_target = 0.15;
        double acc_target = 0.5;
        double vel_target = 0.25;

        ////// a) JS position - 장착 위치 앞 2.5cm
        taskPushBack_taskRecog_ToolChangingAttach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
        //// Tool frame 기준 - 툴 장착
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, +tool_changing_insertion_distance, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
        ////// b) 장착 후, 거치대 내부 퇴장
        //// Tool frame 기준 - 툴 거치대의 내부에서 퇴장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {-tool_changing_exit_from_tool_home_distance, 0, 0, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    auto makeToolDetachingTCPMoveTask = [&] (std::vector<UnitTask>& targetModuleTask) {

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        float tool_changing_insertion_distance = 0.025; // 툴 장착위치로부터 떨어진 거리
        // float tool_changing_insertion_distance = 0.0255; // 툴 장착위치로부터 떨어진 거리, 0.5mm를 더 삽입하도록 25.5mm 기입
        float tool_changing_exit_from_tool_home_distance = 0.09; // 툴 거치대의 내부에서 입장하기 위한 거리

        // double acc_target = 0.075;
        // double vel_target = 0.030;
        // double acc_target = 0.3;
        // double vel_target = 0.15;
        double acc_target = 0.5;
        double vel_target = 0.25;

        ////// a) JS position - 지그 앞 5cm
        taskPushBack_taskRecog_ToolChangingDetach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        ////// b) 거치대 내부 입장
        //// Tool frame 기준 - 툴 거치대의 내부에 입장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {tool_changing_exit_from_tool_home_distance, 0, 0, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// Tool frame 기준 - 툴 탈착 후 퇴장
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -tool_changing_insertion_distance, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };


    //// Tool Attaching
    // makeMoveJSDefaultPosition (module_task_tool_changing_attach_motion_);
    makeInitializeTCP (module_task_tool_changing_attach_motion_, 7); // tcp #7, {0, 0, 0, 0, 0, 0}
    makeMoveAttachingJSDefaultPosition (module_task_tool_changing_attach_motion_);
    ////////////////////
    //// CS (absolute motion) ////
    // makeToolAttachingCSTask(module_task_tool_changing_attach_motion_);
    //// JS & CS (relative motion) ////
    makeToolAttachingTCPMoveTask(module_task_tool_changing_attach_motion_);
    ////////////////////
    makeMoveDetachingJSDefaultPosition (module_task_tool_changing_attach_motion_);
    // // // makeSetToolSlaveTCPAndInitializeGripper(module_task_tool_changing_attach_motion_);
    makeMoveJSDefaultPosition (module_task_tool_changing_attach_motion_);


    //// Tool Detaching
    makeMoveJSDefaultPosition (module_task_tool_changing_detach_motion_);
    makeInitializeTCP (module_task_tool_changing_detach_motion_, 7); // tcp #7, {0, 0, 0, 0, 0, 0}
    //// Gripper close
    taskPushBackKORASGripperCmd(module_task_tool_changing_detach_motion_, (uint16_t)KR_GRP::CLOSE);
    taskPushBack_delay_1ms(module_task_tool_changing_detach_motion_, 50);
    makeMoveDetachingJSDefaultPosition (module_task_tool_changing_detach_motion_);
    ////////////////////
    //// CS (absolute motion) ////
    // makeToolDetachingCSTask(module_task_tool_changing_detach_motion_);
    //// JS & CS (relative motion) ////
    makeToolDetachingTCPMoveTask(module_task_tool_changing_detach_motion_);
    ////////////////////
    makeMoveAttachingJSDefaultPosition (module_task_tool_changing_detach_motion_);
    // makeMoveJSDefaultPosition (module_task_tool_changing_detach_motion_);

    //// Tool initialize
    int main_grp = 2;
    int sub_grp  = 1;
    taskPushBack_irl_grp(module_task_dual_tool_initialize_, KR_GRP::INIT, 0, main_grp);
    taskPushBack_irl_grp(module_task_dual_tool_initialize_, KR_GRP::INIT, 0, sub_grp);
    taskPushBack_delay_1ms(module_task_dual_tool_initialize_, 1000);

    //// LOG
    ROS_LOG_INFO("[The task has been created successfully]: %s", __FUNCTION__);
}

///////////////////////////////////////
//// Tip Changing
void TaskPlanner::makeTipChangeMotionTaskList() {

    module_ui_task_tip_changing_attach_motion_.clear();
    module_ui_task_tip_changing_detach_motion_.clear();
    module_task_tip_changing_attach_motion_.clear();
    module_task_tip_changing_detach_motion_.clear();

    auto makeMoveJSDefaultPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        //// Single gripper
        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeMoveJSSubPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        //// Single gripper
        taskPushBack_jsMove(targetModuleTask, {-121.127, -113.808, 87.457, -154.681, -55.801, 91.250}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeMoveAttachingJSDefaultPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        taskPushBack_taskRecog_TipChangingDefaultAttach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    auto makeMoveDetachingJSDefaultPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        taskPushBack_taskRecog_TipChangingDefaultDetach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    auto makeTipDetachingTask = [&] (std::vector<UnitTask>& targetModuleTask) {


        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        float tip_changing_insertion_distance = 0.025; // 팁 장착위치로부터 떨어진 거리
        float tip_changing_exit_from_tip_home_distance = 0.05; // 팁 거치대의 내부에서 퇴장하기 위한 거리

        double acc_target = 0.5;
        double vel_target = 0.25;

        ////// a) JS position - 지그 앞 5cm
        taskPushBack_taskRecog_TipChangingDetach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// TCP setting: TCP default - 혼동을 막기 위해, 로봇의 기본으로 세팅된 TCP의 방위를 사용
        // Set TCP (#1~) master(#7)
        taskPushBack_selectTCP(targetModuleTask, 7); // TCP default

        ////// b) 거치대 내부 입장
        //// Tool frame 기준 - 툴 거치대의 내부에 입장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, -tip_changing_exit_from_tip_home_distance, 0, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// Tool frame 기준 - 툴 탈착 후 퇴장
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -tip_changing_insertion_distance, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // //// TCP setting (JSON TCP index)
        // taskPushBack_taskRecog_TipChangingSetTCP(targetModuleTask);
    };


    auto makeTipAttachingTask = [&] (std::vector<UnitTask>& targetModuleTask) {

        //// Gripper
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        // float tip_changing_insertion_distance = 0.025; // 팁 장착위치로부터 떨어진 거리
        float tip_changing_insertion_distance = 0.0255; // 팁 장착위치로부터 떨어진 거리
        float tip_changing_exit_from_tip_home_distance = 0.05; // 팁 거치대의 내부에서 퇴장하기 위한 거리

        double acc_target = 0.5;
        double vel_target = 0.25;

        ////// a) JS position - 장착 위치 앞 2.5cm
        taskPushBack_taskRecog_TipChangingAttach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// TCP setting: TCP default - 혼동을 막기 위해, 로봇의 기본으로 세팅된 TCP의 방위를 사용
        // Set TCP (#1~) master(#7)
        taskPushBack_selectTCP(targetModuleTask, 7); // TCP default

        //// Tool frame 기준 - 팁 장착
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, +tip_changing_insertion_distance, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
        ////// b) 장착 후, 거치대 내부 퇴장
        //// Tool frame 기준 - 툴 거치대의 내부에서 퇴장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, tip_changing_exit_from_tip_home_distance, 0, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// TCP setting (JSON TCP index)
        taskPushBack_taskRecog_TipChangingSetTCP(targetModuleTask);
    };

    ////////////////////////////////////////////////////////////////
    //// [UI Task] Tip changing - attach
    // makeMoveJSSubPosition(module_ui_task_tip_changing_attach_motion_);
    taskPushBack_setGripperDriver(module_ui_task_tip_changing_attach_motion_, 1); // Driver #1
    makeMoveAttachingJSDefaultPosition (module_ui_task_tip_changing_attach_motion_);
    makeTipAttachingTask(module_ui_task_tip_changing_attach_motion_);
    makeMoveDetachingJSDefaultPosition (module_ui_task_tip_changing_attach_motion_);
    // makeMoveJSSubPosition(module_ui_task_tip_changing_attach_motion_);
    // makeMoveJSDefaultPosition (module_task_tip_changing_attach_motion_);
    ////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////
    //// [UI Task] Tip changing - detach
    // makeMoveJSSubPosition(module_ui_task_tip_changing_detach_motion_);
    taskPushBack_setGripperDriver(module_ui_task_tip_changing_detach_motion_, 1); // Driver #1
    //// Gripper close
    taskPushBackKORASGripperCmd(module_ui_task_tip_changing_detach_motion_, (uint16_t)KR_GRP::CLOSE);
    taskPushBack_delay_1ms(module_ui_task_tip_changing_detach_motion_, 50);
    makeMoveDetachingJSDefaultPosition (module_ui_task_tip_changing_detach_motion_);
    makeTipDetachingTask(module_ui_task_tip_changing_detach_motion_);
    // makeMoveAttachingJSDefaultPosition (module_ui_task_tip_changing_detach_motion_);
    ////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////
    //// [User Task] Tip changing - attach
    // makeMoveJSSubPosition(module_task_tip_changing_attach_motion_);
    // taskPushBack_setGripperDriver(module_task_tip_changing_attach_motion_, 1); // Driver #1
    // makeMoveAttachingJSDefaultPosition (module_task_tip_changing_attach_motion_);
    makeTipAttachingTask(module_task_tip_changing_attach_motion_);
    makeMoveDetachingJSDefaultPosition (module_task_tip_changing_attach_motion_);
    makeMoveJSSubPosition(module_task_tip_changing_attach_motion_);
    // makeMoveJSDefaultPosition (module_task_tip_changing_attach_motion_);
    ////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////
    //// [User Task] Tip changing - detach
    makeMoveJSSubPosition(module_task_tip_changing_detach_motion_);
    taskPushBack_setGripperDriver(module_task_tip_changing_detach_motion_, 1); // Driver #1
    //// Gripper close
    taskPushBackKORASGripperCmd(module_task_tip_changing_detach_motion_, (uint16_t)KR_GRP::CLOSE);
    taskPushBack_delay_1ms(module_task_tip_changing_detach_motion_, 50);
    makeMoveDetachingJSDefaultPosition (module_task_tip_changing_detach_motion_);
    makeTipDetachingTask(module_task_tip_changing_detach_motion_);
    makeMoveAttachingJSDefaultPosition (module_task_tip_changing_detach_motion_);
    ////////////////////////////////////////////////////////////////


    //// LOG
    ROS_LOG_INFO("[The task has been created successfully]: %s", __FUNCTION__);
}



void TaskPlanner::makeSetTipIndexTask(size_t current_tip_idx) {
    module_task_tip_changing_set_current_tip_idx_.clear();

    taskPushBack_taskRecog_TipChangingSetCurrentTipIndex(module_task_tip_changing_set_current_tip_idx_, current_tip_idx);
}

void TaskPlanner::makeTipChangeMotionTaskListKR0509() {

    // module_task_tip_changing_attach_motion_.clear();
    // module_task_tip_changing_detach_motion_.clear();

    // JsDouble task_position_tip_change = {6.906, 93.257, -133.665, -59.803, -127.052, -38.632};

    // CsDouble task_pose_tip_change_right;
    // CsDouble task_pose_tip_change_center;
    // CsDouble task_pose_tip_change_left;

    // task_pose_tip_change_right  = {0.42935, -0.25866, 0.78933, -136.92125, -2.76712, -177.08};
    // task_pose_tip_change_center = {0.33319, -0.25984, 0.78967, -136.92125, -2.76712, -177.08};
    // task_pose_tip_change_left   = {0.22676, -0.25593, 0.78750, -137.12123, -0.76715, -177.08};

    // CsDouble task_pose_tip_change_right_offset_y  = arraySumCS(task_pose_tip_change_right, {0, 0.05, 0, 0, 0, 0});
    // CsDouble task_pose_tip_change_center_offset_y = arraySumCS(task_pose_tip_change_center, {0, 0.05, 0, 0, 0, 0});
    // CsDouble task_pose_tip_change_left_offset_y   = arraySumCS(task_pose_tip_change_left, {0, 0.05, 0, 0, 0, 0});

    // CsDouble task_pose_tip_change_right_up  = arraySumCS(task_pose_tip_change_right, {0, 0, 0.06, 0, 0, 0});
    // CsDouble task_pose_tip_change_center_up = arraySumCS(task_pose_tip_change_center, {0, 0, 0.06, 0, 0, 0});
    // CsDouble task_pose_tip_change_left_up   = arraySumCS(task_pose_tip_change_left, {0, 0, 0.06, 0, 0, 0});

    // CsDouble task_pose_tip_change_right_up_offset_y  = arraySumCS(task_pose_tip_change_right_up, {0, 0.15, 0, 0, 0, 0});
    // CsDouble task_pose_tip_change_center_up_offset_y = arraySumCS(task_pose_tip_change_center_up, {0, 0.15, 0, 0, 0, 0});
    // CsDouble task_pose_tip_change_left_up_offset_y   = arraySumCS(task_pose_tip_change_left_up, {0, 0.2, 0, 0, 0, 0});

    // // right tip change
    // taskPushBack_jsMove(module_task_test4_, kQTask);
    // taskPushBack_jsMove(module_task_test4_, task_position_tip_change);
    // taskPushBack_csMove(module_task_test4_, task_pose_tip_change_right_offset_y);
    // ref_unit_task_.vel_cs /= 5;
    // ref_unit_task_.acc_cs /= 5;
    // taskPushBack_csMove(module_task_test4_, task_pose_tip_change_right);
    // ref_unit_task_.vel_cs *= 5;
    // ref_unit_task_.acc_cs *= 5;
    // taskPushBack_csMove(module_task_test4_, task_pose_tip_change_right_up);
    // taskPushBack_csMove(module_task_test4_, task_pose_tip_change_right_up_offset_y);
    // taskPushBack_jsMove(module_task_test4_, kQTask);

    // taskPushBack_jsMove(module_task_test4_, kQTask);
    // taskPushBack_jsMove(module_task_test4_, task_position_tip_change);
    // taskPushBack_csMove(module_task_test4_, task_pose_tip_change_right_up_offset_y);
    // taskPushBack_csMove(module_task_test4_, task_pose_tip_change_right_up);
    // ref_unit_task_.vel_cs /= 5;
    // ref_unit_task_.acc_cs /= 5;
    // taskPushBack_csMove(module_task_test4_, task_pose_tip_change_right);
    // ref_unit_task_.vel_cs *= 5;
    // ref_unit_task_.acc_cs *= 5;
    // taskPushBack_csMove(module_task_test4_, task_pose_tip_change_right_offset_y);
    // taskPushBack_jsMove(module_task_test4_, task_position_tip_change);
    // taskPushBack_jsMove(module_task_test4_, kQTask);

    // // center tip change
    // taskPushBack_jsMove(module_task_test5_, kQTask);
    // taskPushBack_jsMove(module_task_test5_, task_position_tip_change);
    // taskPushBack_csMove(module_task_test5_, task_pose_tip_change_center_offset_y);
    // ref_unit_task_.vel_cs /= 5;
    // ref_unit_task_.acc_cs /= 5;
    // taskPushBack_csMove(module_task_test5_, task_pose_tip_change_center);
    // ref_unit_task_.vel_cs *= 5;
    // ref_unit_task_.acc_cs *= 5;
    // taskPushBack_csMove(module_task_test5_, task_pose_tip_change_center_up);
    // taskPushBack_csMove(module_task_test5_, task_pose_tip_change_center_up_offset_y);
    // taskPushBack_jsMove(module_task_test5_, kQTask);


    // taskPushBack_jsMove(module_task_test5_, kQTask);
    // taskPushBack_jsMove(module_task_test5_, task_position_tip_change);
    // taskPushBack_csMove(module_task_test5_, task_pose_tip_change_center_up_offset_y);
    // taskPushBack_csMove(module_task_test5_, task_pose_tip_change_center_up);
    // ref_unit_task_.vel_cs /= 5;
    // ref_unit_task_.acc_cs /= 5;
    // taskPushBack_csMove(module_task_test5_, task_pose_tip_change_center);
    // ref_unit_task_.vel_cs *= 5;
    // ref_unit_task_.acc_cs *= 5;
    // taskPushBack_csMove(module_task_test5_, task_pose_tip_change_center_offset_y);
    // taskPushBack_jsMove(module_task_test5_, task_position_tip_change);
    // taskPushBack_jsMove(module_task_test5_, kQTask);

    // //// LOG
    // ROS_LOG_INFO("[The task has been created successfully]: %s", __FUNCTION__);
}

///////////////////////////////////////
void TaskPlanner::makeRecogOnlyPoseEstimationTaskList() {

    module_recog_only_grasping_pose_estimation_.clear();

    //// Module task - grasping
    auto makeRecogOnlyPoseEstimationTask_robotA = [&] (std::vector<UnitTask>& targetModuleTask) {

        //// Gripper initialize
        //// KORAS gripper Open
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::POS_CTRL, 0, 1000);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //////////////////////////////////////////////////////////////////////////////
        // JS position 1 - ZIVID Scanning position
        taskPushBack_jsMove(targetModuleTask, {86.555, -63.178, -86.690, -119.222, 75.028, 86.424}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_taskRecog_3DScanning(targetModuleTask); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(targetModuleTask, 2000);
        taskPushBack_poseEstimation(targetModuleTask, false, 8); // false: No sampling, true: Sampling, sampling number
        taskPushBack_delay_1ms(targetModuleTask, 200);
        taskPushBack_doGrasping_binPicking_poseEstResults(targetModuleTask); // Move to grasping pose

        ref_unit_task_.vel_cs /= 12;
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.10, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        ////////////////////////////////////////////////
        //// Cylinder demo
        // taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 2000);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.050, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 200);

        // Middle position
        taskPushBack_jsMove(targetModuleTask, {26.736, -81.762, -105.226, -82.602, 89.808, 27.613}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // Feeding position
        taskPushBack_jsMove(targetModuleTask, {-10.518, -89.093, -108.197, -72.271, 90.097, -9.651}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // KORAS gripper Open
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::POS_CTRL, 0, 1000);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_jsMove(targetModuleTask, {86.555, -63.178, -86.690, -119.222, 75.028, 86.424}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 200);

    };
    makeRecogOnlyPoseEstimationTask_robotA (module_recog_only_grasping_pose_estimation_);
    //// LOG
    ROS_LOG_INFO("[The task has been created successfully]: %s", __FUNCTION__);
}

///////////////////////////////////////
//// Initial scan and matching with tool changing motion
void TaskPlanner::makeInitialScanMatchingWithToolChangingMotionTaskList() {

    module_task_initial_scan_matching_.clear();
    module_task_initial_scan_matching_with_tool_changing_motion_.clear();

    auto makeMoveScanJSPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        //// Single gripper
        // taskPushBack_jsMove(targetModuleTask, {-59.528, 118.811, -200.909, -43.154, -117.118, 17.200}, false); // i: index, false: absolute, true: relative
        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    //// Module task - grasping
    auto makeInitializeTCPAndGripper = [&] (std::vector<UnitTask>& targetModuleTask, unsigned int tcp_idx, unsigned int grp_driver_idx) {
        // Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6) , master(#7)
        taskPushBack_selectTCP(targetModuleTask, tcp_idx); // Left hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper driver change(#1 or #2)
        taskPushBack_setGripperDriver(targetModuleTask, grp_driver_idx); // Driver #1 Left hand (for loading)
        taskPushBack_delay_1ms(targetModuleTask, 200);
    };

    auto makeSetToolSlaveTCPAndInitializeGripper = [&] (std::vector<UnitTask>& targetModuleTask) {
        // Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6) , master(#7)
        taskPushBack_taskRecog_selectTCP(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        // taskPushBack_setGripperDriver(targetModuleTask, 2); // Driver #1 Left hand (for loading)
        // taskPushBack_delay_1ms(targetModuleTask, 20);

        taskPushBack_initializeGripper(targetModuleTask); // Gripper driver (#1 or #2)
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };

    auto makeMoveAttachingJSDefaultPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        // taskPushBack_jsMove(targetModuleTask, {76.125, -105.396, -109.606, 34.502, 104.972, 88.428}, false); // i: index, false: absolute, true: relative
        taskPushBack_taskRecog_ToolChangingDefaultAttach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    auto makeMoveDetachingJSDefaultPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        // taskPushBack_jsMove(targetModuleTask, {78.279, -95.776, -102.591, -71.229, 88.616, 77.144}, false); // i: index, false: absolute, true: relative
        taskPushBack_taskRecog_ToolChangingDefaultDetach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    auto makeCurrentToolDetachingTask = [&] (std::vector<UnitTask>& targetModuleTask) {

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        float tool_changing_insertion_distance = 0.025; // 툴 장착위치로부터 떨어진 거리
        // float tool_changing_insertion_distance = 0.0255; // 툴 장착위치로부터 떨어진 거리, 0.5mm를 더 삽입하도록 25.5mm 기입
        float tool_changing_exit_from_tool_home_distance = 0.09; // 툴 거치대의 내부에서 입장하기 위한 거리

        // double acc_target = 0.075;
        // double vel_target = 0.030;
        // double acc_target = 0.3;
        // double vel_target = 0.15;
        double acc_target = 0.5;
        double vel_target = 0.25;

        ////// a) JS position - 지그 앞 5cm
        taskPushBack_taskRecog_CurrentToolDetaching_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        ////// b) 거치대 내부 입장
        //// Tool frame 기준 - 툴 거치대의 내부에 입장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {tool_changing_exit_from_tool_home_distance, 0, 0, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// Tool frame 기준 - 툴 탈착 후 퇴장
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -tool_changing_insertion_distance, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    auto makeToolAttachingTCPMoveTask = [&] (std::vector<UnitTask>& targetModuleTask) {

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        // float tool_changing_insertion_distance = 0.025; // 툴 장착위치로부터 떨어진 거리
        // float tool_changing_insertion_distance = 0.0255; // 툴 장착위치로부터 떨어진 거리, 0.5mm를 더 삽입하도록 25.5mm 기입
        float tool_changing_insertion_distance = 0.0265; // 툴 장착위치로부터 떨어진 거리, 0.5mm를 더 삽입하도록 25.5mm 기입
        float tool_changing_exit_from_tool_home_distance = 0.09; // 툴 거치대의 내부에서 퇴장하기 위한 거리

        // double acc_target = 0.075;
        // double vel_target = 0.030;
        // double acc_target = 0.3;
        // double vel_target = 0.15;
        double acc_target = 0.5;
        double vel_target = 0.25;

        ////// a) JS position - 장착 위치 앞 2.5cm
        taskPushBack_taskRecog_ToolChangingAttach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
        //// Tool frame 기준 - 툴 장착
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, +tool_changing_insertion_distance, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
        ////// b) 장착 후, 거치대 내부 퇴장
        //// Tool frame 기준 - 툴 거치대의 내부에서 퇴장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {-tool_changing_exit_from_tool_home_distance, 0, 0, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    //// 변경: template matching - 토픽 // ZIVID scanning은 서비스
    auto makeZIVIDScanningAndMatchingTask = [&] (std::vector<UnitTask>& targetModuleTask) {
        taskPushBack_taskRecog_3DScanningAndMatching(targetModuleTask, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };

    //// 변경: template matching - 토픽 // ZIVID scanning은 서비스
    auto makeCheckMatchingFinishedTaskVer2 = [&] (std::vector<UnitTask>& targetModuleTask) {
        taskPushBack_taskRecog_checkMatchingFinished(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };



    auto makeInitializeTCP = [&] (std::vector<UnitTask>& targetModuleTask, unsigned int tcp_idx) {
        // Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6) , master(#7)
        taskPushBack_selectTCP(targetModuleTask, tcp_idx); // Left hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 200);
    };



    ///////////////////////////////////////////////////////////////////////////////////////////////
    //// Initial scan and matching (topic)
    makeMoveScanJSPosition (module_task_initial_scan_matching_); // Default JS Position
    makeZIVIDScanningAndMatchingTask (module_task_initial_scan_matching_); // Scanning and matching (topic)
    //// Check matching finished
    makeCheckMatchingFinishedTaskVer2 (module_task_initial_scan_matching_);
    ///////////////////////////////////////////////////////////////////////////////////////////////

    //// TODO:
    ///////////////////////////////////////////////////////////////////////////////////////////////
    //// Initial scan and matching (topic)
    makeMoveScanJSPosition (module_task_initial_scan_matching_with_tool_changing_motion_); // Default JS Position
    makeZIVIDScanningAndMatchingTask (module_task_initial_scan_matching_with_tool_changing_motion_); // Scanning and matching (topic)
    //// Tool detaching
    makeInitializeTCP (module_task_initial_scan_matching_with_tool_changing_motion_, 7); // tcp#, driver#
    //// Gripper close
    taskPushBackKORASGripperCmd(module_task_initial_scan_matching_with_tool_changing_motion_, (uint16_t)KR_GRP::CLOSE);
    taskPushBack_delay_1ms(module_task_initial_scan_matching_with_tool_changing_motion_, 50);
    makeMoveDetachingJSDefaultPosition (module_task_initial_scan_matching_with_tool_changing_motion_);
    makeCurrentToolDetachingTask(module_task_initial_scan_matching_with_tool_changing_motion_);
    makeMoveAttachingJSDefaultPosition (module_task_initial_scan_matching_with_tool_changing_motion_);
    //// Tool attaching
    makeToolAttachingTCPMoveTask(module_task_initial_scan_matching_with_tool_changing_motion_);
    makeMoveDetachingJSDefaultPosition (module_task_initial_scan_matching_with_tool_changing_motion_);
    makeSetToolSlaveTCPAndInitializeGripper(module_task_initial_scan_matching_with_tool_changing_motion_);




    makeMoveScanJSPosition (module_task_initial_scan_matching_with_tool_changing_motion_);
    //// Check matching finished
    makeCheckMatchingFinishedTaskVer2 (module_task_initial_scan_matching_with_tool_changing_motion_);
    ///////////////////////////////////////////////////////////////////////////////////////////////


    //// LOG
    ROS_LOG_INFO("[The task has been created successfully]: %s", __FUNCTION__);
}

///////////////////////////////////////
//// Initial scan and matching with tool changing motion
void TaskPlanner::makeCurrentToolChangingMotionTaskList() {

    module_task_tool_changing_current_index_detach_and_attach_motion_.clear();

    auto makeMoveScanJSPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        //// Single gripper
        // taskPushBack_jsMove(targetModuleTask, {-59.528, 118.811, -200.909, -43.154, -117.118, 17.200}, false); // i: index, false: absolute, true: relative
        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    //// Module task - grasping
    auto makeInitializeTCPAndGripper = [&] (std::vector<UnitTask>& targetModuleTask, unsigned int tcp_idx, unsigned int grp_driver_idx) {
        // Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6) , master(#7)
        taskPushBack_selectTCP(targetModuleTask, tcp_idx); // Left hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper driver change(#1 or #2)
        taskPushBack_setGripperDriver(targetModuleTask, grp_driver_idx); // Driver #1 Left hand (for loading)
        taskPushBack_delay_1ms(targetModuleTask, 200);
    };

    auto makeSetToolSlaveTCPAndInitializeGripper = [&] (std::vector<UnitTask>& targetModuleTask) {
        // Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6) , master(#7)
        taskPushBack_taskRecog_selectTCP(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        // taskPushBack_setGripperDriver(targetModuleTask, 2); // Driver #1 Left hand (for loading)
        // taskPushBack_delay_1ms(targetModuleTask, 20);

        //// 열고 닫는 초기화 버전1
        // taskPushBack_initializeGripper(targetModuleTask); // Gripper driver (#1 or #2)
        //// 오직 닫기만 하는 초기화 버전2
        taskPushBack_initializeVer2Gripper(targetModuleTask); // Gripper driver (#1 or #2)
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };

    auto makeMoveAttachingJSDefaultPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        // taskPushBack_jsMove(targetModuleTask, {76.125, -105.396, -109.606, 34.502, 104.972, 88.428}, false); // i: index, false: absolute, true: relative
        taskPushBack_taskRecog_ToolChangingDefaultAttach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    auto makeMoveDetachingJSDefaultPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        // taskPushBack_jsMove(targetModuleTask, {78.279, -95.776, -102.591, -71.229, 88.616, 77.144}, false); // i: index, false: absolute, true: relative
        taskPushBack_taskRecog_ToolChangingDefaultDetach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    auto makeCurrentToolDetachingTask = [&] (std::vector<UnitTask>& targetModuleTask) {

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        float tool_changing_insertion_distance = 0.025; // 툴 장착위치로부터 떨어진 거리
        // float tool_changing_insertion_distance = 0.0255; // 툴 장착위치로부터 떨어진 거리, 0.5mm를 더 삽입하도록 25.5mm 기입
        float tool_changing_exit_from_tool_home_distance = 0.09; // 툴 거치대의 내부에서 입장하기 위한 거리

        // double acc_target = 0.075;
        // double vel_target = 0.030;
        // double acc_target = 0.3;
        // double vel_target = 0.15;
        double acc_target = 0.5;
        double vel_target = 0.25;

        ////// a) JS position - 지그 앞 5cm
        taskPushBack_taskRecog_CurrentToolDetaching_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        ////// b) 거치대 내부 입장
        //// Tool frame 기준 - 툴 거치대의 내부에 입장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {tool_changing_exit_from_tool_home_distance, 0, 0, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// Tool frame 기준 - 툴 탈착 후 퇴장
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -tool_changing_insertion_distance, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };


    auto makeToolAttachingTCPMoveTask = [&] (std::vector<UnitTask>& targetModuleTask) {

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        // float tool_changing_insertion_distance = 0.025; // 툴 장착위치로부터 떨어진 거리
        // float tool_changing_insertion_distance = 0.0255; // 툴 장착위치로부터 떨어진 거리, 0.5mm를 더 삽입하도록 25.5mm 기입
        float tool_changing_insertion_distance = 0.0265; // 툴 장착위치로부터 떨어진 거리, 0.5mm를 더 삽입하도록 25.5mm 기입
        float tool_changing_exit_from_tool_home_distance = 0.09; // 툴 거치대의 내부에서 퇴장하기 위한 거리

        // double acc_target = 0.075;
        // double vel_target = 0.030;
        // double acc_target = 0.3;
        // double vel_target = 0.15;
        double acc_target = 0.5;
        double vel_target = 0.25;

        ////// a) JS position - 장착 위치 앞 2.5cm
        taskPushBack_taskRecog_ToolChangingAttach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
        //// Tool frame 기준 - 툴 장착
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, +tool_changing_insertion_distance, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
        ////// b) 장착 후, 거치대 내부 퇴장
        //// Tool frame 기준 - 툴 거치대의 내부에서 퇴장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {-tool_changing_exit_from_tool_home_distance, 0, 0, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    auto makeInitializeTCP = [&] (std::vector<UnitTask>& targetModuleTask, unsigned int tcp_idx) {
        // Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6) , master(#7)
        taskPushBack_selectTCP(targetModuleTask, tcp_idx); // Left hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 200);
    };


    ///////////////////////////////////////////////////////////////////////////////////////////////
    makeMoveScanJSPosition (module_task_tool_changing_current_index_detach_and_attach_motion_); // Default JS Position
    //// Tool detaching
    makeInitializeTCP (module_task_tool_changing_current_index_detach_and_attach_motion_, 7); // tcp#, driver#
    //// Gripper close
    taskPushBackKORASGripperCmdToolDetaching(module_task_tool_changing_current_index_detach_and_attach_motion_, (uint16_t)KR_GRP::CLOSE);
    taskPushBack_delay_1ms(module_task_tool_changing_current_index_detach_and_attach_motion_, 50);
    makeMoveDetachingJSDefaultPosition (module_task_tool_changing_current_index_detach_and_attach_motion_);
    makeCurrentToolDetachingTask(module_task_tool_changing_current_index_detach_and_attach_motion_);
    makeMoveAttachingJSDefaultPosition (module_task_tool_changing_current_index_detach_and_attach_motion_);
    //// Tool attaching
    makeToolAttachingTCPMoveTask(module_task_tool_changing_current_index_detach_and_attach_motion_);
    makeMoveDetachingJSDefaultPosition (module_task_tool_changing_current_index_detach_and_attach_motion_);
    makeSetToolSlaveTCPAndInitializeGripper(module_task_tool_changing_current_index_detach_and_attach_motion_);
    makeMoveScanJSPosition (module_task_tool_changing_current_index_detach_and_attach_motion_);
    ///////////////////////////////////////////////////////////////////////////////////////////////


    //// LOG
    ROS_LOG_INFO("[The task has been created successfully]: %s", __FUNCTION__);
}


void TaskPlanner::makeCurrentTipChangingMotionTaskList() {
    module_task_tip_changing_current_index_detach_and_attach_motion_.clear();

    auto makeMoveJSDefaultPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        //// Single gripper
        // taskPushBack_jsMove(targetModuleTask, {-90, 110, -210, 0, -80, 0}, false); // i: index, false: absolute, true: relative
        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeMoveAttachingJSDefaultPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        taskPushBack_taskRecog_TipChangingDefaultAttach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    auto makeMoveDetachingJSDefaultPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        taskPushBack_taskRecog_TipChangingDefaultDetach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    auto makeCurrentTipDetachingTask = [&] (std::vector<UnitTask>& targetModuleTask) {

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        float tip_changing_insertion_distance = 0.025; // 팁 장착위치로부터 떨어진 거리
        float tip_changing_exit_from_tip_home_distance = 0.05; // 팁 거치대의 내부에서 퇴장하기 위한 거리

        double acc_target = 0.5;
        double vel_target = 0.25;

        ////// a) JS position - 지그 앞 5cm
        taskPushBack_taskRecog_CurrentTipChangingDetach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// TCP setting: TCP default - 혼동을 막기 위해, 로봇의 기본으로 세팅된 TCP의 방위를 사용
        // Set TCP (#1~) master(#7)
        taskPushBack_selectTCP(targetModuleTask, 7); // TCP default

        ////// b) 거치대 내부 입장
        //// Tool frame 기준 - 툴 거치대의 내부에 입장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, -tip_changing_exit_from_tip_home_distance, 0, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// Tool frame 기준 - 툴 탈착 후 퇴장
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -tip_changing_insertion_distance, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // //// TCP setting (JSON TCP index)
        // taskPushBack_taskRecog_TipChangingSetTCP(targetModuleTask);
    };


    auto makeTipAttachingTask = [&] (std::vector<UnitTask>& targetModuleTask) {

        //// Gripper
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        // float tip_changing_insertion_distance = 0.025; // 팁 장착위치로부터 떨어진 거리
        float tip_changing_insertion_distance = 0.0255; // 팁 장착위치로부터 떨어진 거리
        float tip_changing_exit_from_tip_home_distance = 0.05; // 팁 거치대의 내부에서 퇴장하기 위한 거리

        double acc_target = 0.5;
        double vel_target = 0.25;

        ////// a) JS position - 장착 위치 앞 2.5cm
        taskPushBack_taskRecog_TipChangingAttach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// TCP setting: TCP default - 혼동을 막기 위해, 로봇의 기본으로 세팅된 TCP의 방위를 사용
        // Set TCP (#1~) master(#7)
        taskPushBack_selectTCP(targetModuleTask, 7); // TCP default

        //// Tool frame 기준 - 팁 장착
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, +tip_changing_insertion_distance, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
        ////// b) 장착 후, 거치대 내부 퇴장
        //// Tool frame 기준 - 툴 거치대의 내부에서 퇴장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, tip_changing_exit_from_tip_home_distance, 0, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// TCP setting (JSON TCP index)
        taskPushBack_taskRecog_TipChangingSetTCP(targetModuleTask);
    };


    //// Tip changing - detach
    taskPushBack_setGripperDriver(module_task_tip_changing_current_index_detach_and_attach_motion_, 1); // Driver #1
    //// Gripper close
    taskPushBackKORASGripperCmd(module_task_tip_changing_current_index_detach_and_attach_motion_, (uint16_t)KR_GRP::CLOSE);
    taskPushBack_delay_1ms(module_task_tip_changing_current_index_detach_and_attach_motion_, 50);
    makeMoveDetachingJSDefaultPosition (module_task_tip_changing_current_index_detach_and_attach_motion_);
    makeCurrentTipDetachingTask(module_task_tip_changing_current_index_detach_and_attach_motion_);
    makeMoveAttachingJSDefaultPosition (module_task_tip_changing_current_index_detach_and_attach_motion_);
    //// Tip changing - attach
    // taskPushBack_setGripperDriver(module_task_tip_changing_current_index_detach_and_attach_motion_, 1); // Driver #1
    // makeMoveAttachingJSDefaultPosition (module_task_tip_changing_current_index_detach_and_attach_motion_);
    makeTipAttachingTask(module_task_tip_changing_current_index_detach_and_attach_motion_);
    makeMoveDetachingJSDefaultPosition (module_task_tip_changing_current_index_detach_and_attach_motion_);
    makeMoveJSDefaultPosition (module_task_tip_changing_current_index_detach_and_attach_motion_);


    //// LOG
    ROS_LOG_INFO("[The task has been created successfully]: %s", __FUNCTION__);
}

//// KORAS GRIPPER INITIALIZE TASK

/** @brief Korean: KORAS 그리퍼의 최소/최대 거리를 설정한다.
 */
void TaskPlanner::makeKORASGripperSetInitialValueTask() {

    module_koras_gripper_set_initial_value_task_.clear();

    auto makeKORASGripperSetInitialValue = [&] (std::vector<UnitTask>& targetModuleTask) {

        //// Gripper driver change
        taskPushBack_setGripperDriver(targetModuleTask, 1); // Driver #1 Left hand (for loading)
        taskPushBack_delay_1ms(targetModuleTask, 200);
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 4000);
        taskPushBack_setGripperMaxValue(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::POS_CTRL, 10000, 1000);
        taskPushBack_delay_1ms(targetModuleTask, 4000);
        taskPushBack_setGripperMinValue(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 200);


        //// Gripper driver change
        taskPushBack_setGripperDriver(targetModuleTask, 2); // Driver #1 Left hand (for loading)
        taskPushBack_delay_1ms(targetModuleTask, 200);
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 4000);
        taskPushBack_setGripperMaxValue(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 200);
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::POS_CTRL, 10000, 1000);
        taskPushBack_delay_1ms(targetModuleTask, 4000);
        taskPushBack_setGripperMinValue(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 200);
    };
    makeKORASGripperSetInitialValue (module_koras_gripper_set_initial_value_task_);

    //// LOG
    ROS_LOG_INFO("[The task has been created successfully]: %s", __FUNCTION__);
}

//// TEST
void TaskPlanner::makeBinPickingTestTaskList() {

    module_bin_picking_test_.clear();
    module_task_initial_scan_matching_.clear();
    module_task_initial_scan_matching_with_tool_changing_motion_.clear();
    // Bin picking system module task (for test)
    int main_grp = 2;
    int sub_grp  = 1;

    //// Grasping with blending
    auto makeStackingTask = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 0.6; vel_move = 0.3;
            acc_contact = 0.2; vel_contact = 0.1;
        }

        double apporach_distance = 0.0;
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) {
            apporach_distance = 0.0075;
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_LATHE_CYLINDER) {
            apporach_distance = 0.032;
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) {
            apporach_distance = 0.01;
        } else { // 2-finger grasping - Open
            apporach_distance = 0.01;
        }

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        //// TODO: JSON파일의 z margin과 연동해야 함.
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, apporach_distance, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        // Gripper - ungrasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1000);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1000);
        } else { // 2-finger grasping - Open
            taskPushBack_irl_grp(targetModuleTask, KR_GRP::POS_CTRL, 7250, sub_grp);
            taskPushBack_delay_1ms(targetModuleTask, 1000);
        }
        taskPushBack_countDetachingPose(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 10);

        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -apporach_distance, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);
        ////////////////////////////////////////////////////////////////
    };

    bool is_slow_mode = true;
    // // BPDemoType::BPDemoType::DEMO_BP_CNC_LATHE_CYLINDER
    taskPushBack_selectTCP(module_bin_picking_test_, 2); // Left hand (for unloading)
    taskPushBack_delay_1ms(module_bin_picking_test_, 20);
    taskPushBack_jsMove (module_bin_picking_test_, {-197.672, 122.241, -192.982, 50.455, -112.537, -2.032}, false);

    taskPushBack_taskJSON_csMoveMotionTag(module_bin_picking_test_, {0, 0, 0, 0, 0, 0}, BPDemoType::DEMO_BP_CNC_LATHE_CYLINDER, MotionTag::MOTION_TAG_JSON_CSMOVE_STACKING);
    taskPushBack_delay_1ms(module_bin_picking_test_, 100);
    makeStackingTask (module_bin_picking_test_, BPDemoType::DEMO_BP_CNC_LATHE_CYLINDER, is_slow_mode);
    taskPushBack_jsMove (module_bin_picking_test_, {-197.672, 122.241, -192.982, 50.455, -112.537, -2.032}, false);

    // taskPushBack_selectTCP(module_bin_picking_test_, 3); // Left hand (for unloading)
    // taskPushBack_delay_1ms(module_bin_picking_test_, 20);
    // taskPushBack_jsMove (module_bin_picking_test_, {-181.496, 80.687, -176.091, 31.692, -116.969, 134.821}, false);

    // taskPushBack_taskJSON_csMoveMotionTag(module_bin_picking_test_, {0, 0, +0.05, 0, 0, 0}, BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG, MotionTag::MOTION_TAG_JSON_CSMOVE_STACKING);
    // taskPushBack_csMove (module_bin_picking_test_, {0, 0, -0.05, 0, 0, 0}, true);
    // makeStackingTask (module_bin_picking_test_, BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG, is_slow_mode);
    // taskPushBack_csMove (module_bin_picking_test_, {0, 0, 0.05, 0, 0, 0}, true);
    // taskPushBack_jsMove (module_bin_picking_test_, {-181.496, 80.687, -176.091, 31.692, -116.969, 134.821}, false);

    //// LOG
    ROS_LOG_INFO("[The task has been created successfully]: %s", __FUNCTION__);
}

void TaskPlanner::makeBinPickingCylinderDualGripperTaskList()
{
    module_task_bin_picking_dual_machine_tending_.clear();

    // Bin picking system module task (for test)
    int main_grp = 2;
    int sub_grp  = 1;
    int grp_init_delay = 500;
    int grp_move_delay = 200;
    int grp_default_width = 7250;

    JsDouble task_position_right = arraySumJS(kQTask, {-90, 0, 0, 0, 0, 0});
    // CsDouble tcp_main_grp = {-0.000,  0.14985, 0.14985, 0, -45, -90}; // not tip change (short gripper)
    CsDouble tcp_main_grp  = {-0.003, -0.19845, 0.19845, 0,  45, -90}; // not tip change grp
    // CsDouble tcp_sub_grp  = {-0.003, -0.19845, 0.19845, 0,  45, -90}; // tip change grp
    CsDouble tcp_sub_grp  = {-0.003, -0.21178, 0.20562, 0.0, 45.0, -90.0}; // tip change grp


    JsDouble j1 = {-92.114, 83.943, -195.365, -2.138, -22.965, -89.305};
    // tcp_main_grp
    CsDouble x1 = {-0.01215, -0.60397, 0.16846, 178.88298, -0.55384, -91.35799}; // tcp_main_grp !!
    CsDouble x2 = {0.47654, -0.30705, 0.19282, 148.571, 26.052, 115.125};
    CsDouble x3 = {0.47654, -0.30705, 0.07954, 148.57092, 26.05248, 115.12485}; // tcp_main_grp & 재파지 전 놓기 자세

    CsDouble x4 = {0.42467, -0.28208, 0.12396, 149.13535, 24.05487, 116.45753}; // tcp_main_grp & 재파지 전 tcp z -0.07 자세
    CsDouble x5 = {0.46773, -0.28801, 0.06909, 149.13535, 24.05487, 116.45753}; // tcp_main_grp & 재파지 자세
    CsDouble x6 = {0.27090, 0.34325, 0.73414, 92.85377, 51.32366, 92.73173}; // tcp_main_grp & sub grp로 파지 전
    // tcp_sub_grp
    // CsDouble x7 = {-0.02243, 0.60960, 0.40668, 141.35750, -1.78382, -179.49753}; // tcp_sub_grp & 척 파지 자세
    CsDouble x7 = {-0.02684, 0.60935, 0.40489, 141.358, -1.783, -179.497}; // tcp_sub_grp & 척 파지 자세


    // tcp_main_grp
    CsDouble x8 = {-0.02191, 0.61677, 0.41242, -141.35585, 1.87698, 0.57814}; // tcp_main_grp & 척 파지 꽂은 자세
    // tcp_sub_grp
    // JsDouble j2 = {-2.790, 109.531, -209.277, 2.566, -35.377, 86.092}; // 쌓기 자세?
    JsDouble j3 = {50.133, 111.092, -182.854, -4.359, -102.626, -41.671}; // 쌓기 자세?


    //// 파지
    //// Grasping with blending
    //// Module task - grasping
    auto makeSingleGripperGraspingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {

        taskPushBack_setGripperDriver(targetModuleTask, main_grp); // Driver #1 Left hand (for loading)

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        if(slow_mode) {
            taskPushBack_doGrasping_binPicking(targetModuleTask, 0.3, 0.15); // Move to grasping pose, (acc, vel)
        } else {
            taskPushBack_doGrasping_binPicking(targetModuleTask, 0.5, 0.3); // Move to grasping pose, (acc, vel)
        }

        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.04, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Gripper Close
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.080, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Gripper additional close command 1
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 50);

        taskPushBack_jsMove(targetModuleTask, {-59.528, 118.811, -200.909, -43.154, -117.118, 17.200}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper additional close command 2
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 50);

        // //// Check grasping success/failure
        // taskPushBack_taskRecog_KORASGripperCheckGrasping(targetModuleTask);
        // taskPushBack_delay_1ms(targetModuleTask, 20);

    };

    //// template matching - 토픽
    auto makeZIVIDScanningAndMatchingTask = [&] (std::vector<UnitTask>& targetModuleTask) {
        taskPushBack_taskRecog_3DScanningAndMatching(targetModuleTask, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };


    //// Module task - grasping
    auto makeInitializePLCChuck = [&] (std::vector<UnitTask>& targetModuleTask) {
        // taskPushBackPLCModbusCmd(targetModuleTask, LATHE_CHUCK_CLOSE);
        // taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, MILLING_CHUCK_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };

    //// Grasping with blending
    auto makeStackingTask = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 0.6; vel_move = 0.3;
            acc_contact = 0.2; vel_contact = 0.1;
        }

        double apporach_distance = 0.0;
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) {
            apporach_distance = 0.0075;
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_LATHE_CYLINDER) {
            apporach_distance = 0.032;
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) {
            apporach_distance = 0.01;
        } else { // 2-finger grasping - Open
            apporach_distance = 0.01;
        }

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        //// TODO: JSON파일의 z margin과 연동해야 함.
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, apporach_distance, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        // Gripper - ungrasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1000);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1000);
        } else { // 2-finger grasping - Open
            taskPushBack_irl_grp(targetModuleTask, KR_GRP::POS_CTRL, 7250, sub_grp);
            taskPushBack_delay_1ms(targetModuleTask, 1000);
        }
        taskPushBack_countDetachingPose(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 10);

        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -apporach_distance, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);
        ////////////////////////////////////////////////////////////////
    };

    bool is_slow_mode = true;
    // makeInitializePLCChuck(module_task_bin_picking_dual_machine_tending_);
    //// Machine tending - Lathe, single gripper, CNC Chuck(PLC modbus)
    // taskPushBack_jsMove (module_task_bin_picking_dual_machine_tending_, task_position_right);
    taskPushBack_jsMove(module_task_bin_picking_dual_machine_tending_, {-59.528, 118.811, -200.909, -43.154, -117.118, 17.200}, false); // i: index, false: absolute, true: relative
    // taskPushBack_irl_grp(module_task_bin_picking_dual_machine_tending_, KR_GRP::INIT, 0, main_grp);
    // taskPushBack_irl_grp(module_task_bin_picking_dual_machine_tending_, KR_GRP::INIT, 0, sub_grp);
    // taskPushBack_delay  (module_task_bin_picking_dual_machine_tending_, grp_init_delay);
    taskPushBack_irl_grp(module_task_bin_picking_dual_machine_tending_, KR_GRP::POS_CTRL, grp_default_width, main_grp);
    taskPushBack_irl_grp(module_task_bin_picking_dual_machine_tending_, KR_GRP::POS_CTRL, grp_default_width, sub_grp);
    taskPushBack_delay  (module_task_bin_picking_dual_machine_tending_, grp_move_delay);
    taskPushBack_setTCP (module_task_bin_picking_dual_machine_tending_, tcp_main_grp);
    makeSingleGripperGraspingTask (module_task_bin_picking_dual_machine_tending_, is_slow_mode);

    //// 파지 이후
    // taskPushBack_jsMove (module_task_bin_picking_dual_machine_tending_, task_position_right);
    // taskPushBack_jsMove (module_task_bin_picking_dual_machine_tending_, j1);
    taskPushBack_irl_grp(module_task_bin_picking_dual_machine_tending_, KR_GRP::CLOSE, 0, main_grp);
    taskPushBack_delay_1ms(module_task_bin_picking_dual_machine_tending_, 200);

    // tcp change!!
    taskPushBack_setTCP (module_task_bin_picking_dual_machine_tending_, tcp_main_grp);
    // taskPushBack_csMove (module_task_bin_picking_dual_machine_tending_, x1);
    taskPushBack_csMove (module_task_bin_picking_dual_machine_tending_, x2);
    taskPushBack_csMove (module_task_bin_picking_dual_machine_tending_, x3);
    taskPushBack_irl_grp(module_task_bin_picking_dual_machine_tending_, KR_GRP::POS_CTRL, grp_default_width, main_grp);
    taskPushBack_delay  (module_task_bin_picking_dual_machine_tending_, grp_move_delay);
    taskPushBack_tcpMove(module_task_bin_picking_dual_machine_tending_, {0, 0, -0.07, 0, 0, 0});
    taskPushBack_csMove (module_task_bin_picking_dual_machine_tending_, x4);
    taskPushBack_csMove (module_task_bin_picking_dual_machine_tending_, x5);
    taskPushBack_irl_grp(module_task_bin_picking_dual_machine_tending_, KR_GRP::CLOSE, 0, main_grp);
    taskPushBack_delay  (module_task_bin_picking_dual_machine_tending_, grp_move_delay);
    taskPushBack_csMove (module_task_bin_picking_dual_machine_tending_, {0, 0, 0.15, 0, 0, 0}, true);
    taskPushBack_csMove (module_task_bin_picking_dual_machine_tending_, {0.05, 0.45, 0.1, 0, 0, 0}, true);
    taskPushBack_csMove (module_task_bin_picking_dual_machine_tending_, x6);

    // Scanning
    makeZIVIDScanningAndMatchingTask (module_task_bin_picking_dual_machine_tending_); // Topic

    // tcp change!!
    taskPushBack_setTCP (module_task_bin_picking_dual_machine_tending_, tcp_sub_grp);
    taskPushBack_csMove (module_task_bin_picking_dual_machine_tending_, x7);
    taskPushBack_irl_grp(module_task_bin_picking_dual_machine_tending_, KR_GRP::CLOSE, 0, sub_grp);
    taskPushBack_delay  (module_task_bin_picking_dual_machine_tending_, grp_move_delay);
    taskPushBackPLCModbusCmd(module_task_bin_picking_dual_machine_tending_, LATHE_CHUCK_OPEN);
    taskPushBack_delay_1ms(module_task_bin_picking_dual_machine_tending_, 1500);
    taskPushBack_tcpMove(module_task_bin_picking_dual_machine_tending_, {-0.15, 0, 0, 0, 0, 0}); // 150mm 빼기
    taskPushBack_jsMove (module_task_bin_picking_dual_machine_tending_, {0, 0, 0, 0, 0, -180}, true);
    // tcp change!!
    taskPushBack_setTCP (module_task_bin_picking_dual_machine_tending_, tcp_main_grp);
    taskPushBack_csMove (module_task_bin_picking_dual_machine_tending_, arraySumCS(x8, {0.05, 0, 0, 0, 0, 0}));
    taskPushBack_csMove (module_task_bin_picking_dual_machine_tending_, x8);
    taskPushBackPLCModbusCmd(module_task_bin_picking_dual_machine_tending_, LATHE_CHUCK_CLOSE);
    taskPushBack_delay_1ms(module_task_bin_picking_dual_machine_tending_, 2500);
    taskPushBack_irl_grp(module_task_bin_picking_dual_machine_tending_, KR_GRP::OPEN, 0, main_grp);
    taskPushBack_delay  (module_task_bin_picking_dual_machine_tending_, grp_move_delay);
    taskPushBack_tcpMove(module_task_bin_picking_dual_machine_tending_, {0, 0, -0.07, 0, 0, 0});
    taskPushBack_irl_grp(module_task_bin_picking_dual_machine_tending_, KR_GRP::POS_CTRL, grp_default_width, main_grp);

    //// To Stacking station
    taskPushBack_jsMove (module_task_bin_picking_dual_machine_tending_, {50.456, 108.166, -171.484, -4.560, -111.067, -42.045}, false);
    taskPushBack_jsMove (module_task_bin_picking_dual_machine_tending_, {-32.672, 122.241, -192.982, 50.455, -112.537, -2.032}, false);
    taskPushBack_jsMove (module_task_bin_picking_dual_machine_tending_, {-165.0, 0, 0, 0, 0, 0}, true);


    taskPushBack_setTCP (module_task_bin_picking_dual_machine_tending_, tcp_sub_grp);

    // Stacking
    taskPushBack_taskJSON_csMoveMotionTag(module_task_bin_picking_dual_machine_tending_, {0, 0, 0, 0, 0, 0}, BPDemoType::DEMO_BP_CNC_LATHE_CYLINDER, MotionTag::MOTION_TAG_JSON_CSMOVE_STACKING);
    ref_unit_task_.vel_cs /= 5;
    ref_unit_task_.acc_cs /= 5;
    makeStackingTask (module_task_bin_picking_dual_machine_tending_, BPDemoType::DEMO_BP_CNC_LATHE_CYLINDER, is_slow_mode);
    ref_unit_task_.vel_cs *= 5;
    ref_unit_task_.acc_cs *= 5;
    taskPushBack_jsMove (module_task_bin_picking_dual_machine_tending_, {-197.672, 122.241, -192.982, 50.455, -112.537, -2.032}, false);
    //// Scan pose
    taskPushBack_jsMove (module_task_bin_picking_dual_machine_tending_, {-59.528, 118.811, -200.909, -43.154, -117.118, 17.200}, false);
    taskPushBack_setTCP (module_task_bin_picking_dual_machine_tending_, tcp_main_grp);
    // taskPushBack_jsMove (module_task_bin_picking_dual_machine_tending_, {0, 0, 0, 0, 0, 180}, true);
    // tcp change!!
    // taskPushBack_setTCP (module_task_bin_picking_dual_machine_tending_, tcp_sub_grp);
    // taskPushBack_jsMove (module_task_bin_picking_dual_machine_tending_, j2);

}


///////////////////////////////////////


/////////////////////////////////////// Bin picking task
void TaskPlanner::makeTestRecogOnlyGraspingTaskList(bool is_slow_mode) {

    //// MACHINE TENDING TASK
    module_task_bin_picking_single_machine_tending_lathe_.clear();
    module_task_bin_picking_single_machine_tending_milling_.clear();
    module_task_bin_picking_machine_tending_bolt_bush_.clear();

    //// PICK AND PLACE TASK
    module_task_bin_picking_pick_and_place_.clear();
    module_task_bin_picking_pick_and_place_eb_joint_.clear();
    module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_.clear();
    module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_.clear();
    module_task_bin_picking_pick_and_place_bolt_bush_.clear();
    module_task_bin_picking_pick_and_place_cylinder_.clear();
    module_task_bin_picking_pick_and_place_square_peg_.clear();

    //// Graphy Test TASK
    module_task_graphy_stage5_grasping_test_1_.clear();
    module_task_graphy_stage5_only_grasping_test_.clear();
    module_task_graphy_stage5_plate_rotation_test_1_.clear();

    //// Module task - grasping
    auto makeTestInitializePnematicChuck = [&] (std::vector<UnitTask>& targetModuleTask) {
        //// Cylinder demo set
        // taskPushBack_gripper_pne1(targetModuleTask, false); // Vertical pnematic OFF
        taskPushBackGripperRelayCmd(targetModuleTask, PNE1_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBack_gripper_pne2(targetModuleTask, false); // Horizontal pnematic ON
        taskPushBackGripperRelayCmd(targetModuleTask, PNE2_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };

    //// Module task - grasping
    auto makeTestInitializePLCChuck = [&] (std::vector<UnitTask>& targetModuleTask) {
        taskPushBackPLCModbusCmd(targetModuleTask, LATHE_CHUCK_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, MILLING_CHUCK_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };

    //// Module task - grasping
    auto makeTestInitializeTCP = [&] (std::vector<UnitTask>& targetModuleTask, unsigned int tcp_idx) {
        // Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6)
        taskPushBack_selectTCP(targetModuleTask, tcp_idx); // Left hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };

    //// Module task - grasping
    auto makeTestInitializeTCPAndGripper = [&] (std::vector<UnitTask>& targetModuleTask, unsigned int tcp_idx, unsigned int grp_driver_idx) {
        // Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6)
        taskPushBack_selectTCP(targetModuleTask, tcp_idx); // Left hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// Gripper driver change(#1 or #2)
        taskPushBack_setGripperDriver(targetModuleTask, grp_driver_idx); // Driver #1 Left hand (for loading)
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };

    auto makeTestMoveJSDefaultPosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        //// Single gripper
        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);
    };

    //// Module task - grasping
    auto makeTestMoveJSMiddlePosition = [&] (std::vector<UnitTask>& targetModuleTask) {
        taskPushBack_jsMove(targetModuleTask, {230.626, -116.719, 121.618, -95.213, -89.574, -31.555}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);
    };

    auto makeTestRegraspingTaskPnematicChuck = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {

        std::vector<double> regrasping_pose_1, regrasping_pose_2;
        switch(demo_tag) { // JSON input
            case BPDemoType::DEMO_BP_CNC_LATHE_CYLINDER: {
                std::vector<double> vec_tmp_1 = {0.21726, -0.49994, 0.03426, 179.335, 0.230, -179.331};
                regrasping_pose_1 = vec_tmp_1;
                std::vector<double> vec_tmp_2 = {0.20629, -0.49918, 0.08973, -179.838, 0.055, 178.263};
                regrasping_pose_2 = vec_tmp_2;
                break;
            }
            case BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG: {
                std::vector<double> vec_tmp_1 = {0.21726, -0.49994, 0.13426, 179.335, 0.230, -179.331};
                regrasping_pose_1 = vec_tmp_1;
                std::vector<double> vec_tmp_2 = {0.20629, -0.49918, 0.18973, -179.838, 0.055, 178.263};
                regrasping_pose_2 = vec_tmp_2;
                break;
            }
            default: {
                assert(false);
                break;
            }
        }

        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 2.0; vel_move = 1.0;
            acc_contact = 0.2; vel_contact = 0.1;
        }
        taskPushBackGripperRelayCmd(targetModuleTask, PNE1_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackGripperRelayCmd(targetModuleTask, PNE2_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.05, 0, 0, 0}, acc_move, vel_move, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // KORAS gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBackGripperRelayCmd(targetModuleTask, PNE2_ON);
        taskPushBack_delay_1ms(targetModuleTask, 1500);

        //// code here
        // JS: 94.424, -91.897, 134.768, -132.072, -89.926, -85.823
        taskPushBack_csMove2(targetModuleTask, regrasping_pose_1, acc_move, vel_move, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Gripper close 1
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        taskPushBackGripperRelayCmd(targetModuleTask, PNE2_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// Gripper close 2
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        taskPushBack_csMoveToolFrame(targetModuleTask, {0.005, 0, -0.005, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        taskPushBack_csMove2(targetModuleTask, regrasping_pose_2, acc_move, vel_move, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Gripper close 3
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 20);

    };

    auto makeTestRegraspingTaskZig = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 2.0; vel_move = 1.0;
            acc_contact = 0.2; vel_contact = 0.1;
        }

        taskPushBack_taskJSON_csMoveMotionTag(targetModuleTask, {0, 0, 0, 0, 0, 0}, demo_tag, MotionTag::MOTION_TAG_JSON_CSMOVE_REGRASP_PLACE);
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // KORAS gripper Open
        if(demo_tag != BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) {
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 60);
        } else {
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1500);
        }

        taskPushBack_taskJSON_csMoveMotionTag(targetModuleTask, {0, 0, 0, 0, 0, 0}, demo_tag, MotionTag::MOTION_TAG_JSON_CSMOVE_REGRASP_PICK);
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper close 1
        if(demo_tag != BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) {
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1000);
        } else {
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
            taskPushBack_delay_1ms(targetModuleTask, 1500);
        }
        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.005, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// Gripper close 2
        if(demo_tag != BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) {
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        }
    };

    //// Module task - grasping
    //// 기존: template matching - 서비스
    auto makeTestZIVIDScanningTask = [&] (std::vector<UnitTask>& targetModuleTask) {
        taskPushBack_taskRecog_3DScanning(targetModuleTask); // ZIVID Scanning with task recognition
        taskPushBack_delayScanning(targetModuleTask, 2000);
        taskPushBack_templateMatching(targetModuleTask, false, 8); // false: No sampling, true: Sampling, sampling number
        taskPushBack_delayScanning(targetModuleTask, 200);
    };

    //// 변경: template matching - 토픽 // ZIVID scanning은 서비스
    auto makeTestZIVIDScanningAndMatchingTask = [&] (std::vector<UnitTask>& targetModuleTask) {

        taskPushBack_taskRecog_3DScanningAndMatching(targetModuleTask, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };

    //// 변경: template matching - 토픽 // ZIVID scanning은 서비스
    auto makeTestCheckMatchingFinishedTaskVer2 = [&] (std::vector<UnitTask>& targetModuleTask) {
        taskPushBack_taskRecog_checkMatchingFinished(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);
    };

    //// Module task - grasping (inner plane grasping)
    auto makeTestInnerGraspingTask = [&] (std::vector<UnitTask>& targetModuleTask) {
        //// Gripper initialize - KORAS gripper Close
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        taskPushBack_doGrasping_binPicking(targetModuleTask, 0.1, 0.1); // Move to grasping pose, (acc, vel)

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        //// TODO: JSON파일의 z margin과 연동해야 함.
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.04, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.050, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 200);

        // //// Gripper additional close command
        // taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        // taskPushBack_delay_1ms(targetModuleTask, 400);

        // Pick and Place - Middle position
        taskPushBack_jsMove(targetModuleTask, {93.600, -83.712, -100.319, -89.281, 88.459, 175.313}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    //// 파지
    //// Grasping with blending
    //// Module task - grasping
    auto makeTestSingleGripperGraspingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        taskPushBack_doGrasping_binPicking(targetModuleTask, 0.5, 0.25); // Move to grasping pose, (acc, vel)

        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.04, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Gripper Close
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.040, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Gripper additional close command 1
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 50);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.080, 0, 0, 0}, 0.75, 0.5, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Gripper additional close command 1
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 50);

        // taskPushBack_jsMove(targetModuleTask, {-59.528, 118.811, -200.909, -43.154, -117.118, 17.200}, false); // i: index, false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 100);

        // //// Gripper additional close command 2
        // taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        // taskPushBack_delay_1ms(targetModuleTask, 50);

        // //// Check grasping success/failure
        // taskPushBack_taskRecog_KORASGripperCheckGrasping(targetModuleTask);
        // taskPushBack_delay_1ms(targetModuleTask, 20);

    };

    //// Module task - grasping
    auto makeTestSingleVacuumGripperGraspingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {


        //// Gripper Open
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        taskPushBack_doGrasping_binPicking(targetModuleTask, 0.5, 0.25); // Move to grasping pose, (acc, vel)

        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.04, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);


        //// Gripper
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
        taskPushBack_delay_1ms(targetModuleTask, 1000);


        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.040, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.080, 0, 0, 0}, 0.75, 0.5, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);
    };

    //// Grasping with blending
    auto makeTestSingleGripperGraspingBlendingTask = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;

        //// Gripper - ungrasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        } else { // 2-finger grasping - Open
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        }

        ////////////////////////////////////////////////////////////////
        //// Blending motion, MotionTag::MOTION_TAG_BLENDING_HOME_TO_GRASP_APPROACH_POSE
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_HOME_TO_GRASP_APPROACH_POSE);
        taskPushBack_delay_1ms(targetModuleTask, 100);
        ////////////////////////////////////////////////////////////////
        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        //// TODO: JSON파일의 z margin과 연동해야 함.
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.04, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Gripper - grasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::OPEN);
            taskPushBack_delay_1ms(targetModuleTask, 1000);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - ON
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
            taskPushBack_delay_1ms(targetModuleTask, 1000);
        } else { // 2-finger grasping - close
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1000);
        }

        //// Tool frame 기준 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -0.04, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Gripper additional close command 1
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping


        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping
            //// TODO: 흡착 그리퍼가 파지를 성공했는 지 여부?

        } else { // 2-finger grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 20);
            // //// Check grasping success/failure
            // taskPushBack_taskRecog_KORASGripperCheckGrasping(targetModuleTask);
            // taskPushBack_delay_1ms(targetModuleTask, 20);
        }

        ////////////////////////////////////////////////////////////////
        //// Blending motion, MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_REGRASPING_ZIG
        // if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
        //     taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_CNC_INNER);
        //     taskPushBack_delay_1ms(targetModuleTask, 100);
        // } else { // Regrasping 수행하는 경우
        //     taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_REGRASPING_ZIG);
        //     taskPushBack_delay_1ms(targetModuleTask, 100);
        // }
        ////////////////////////////////////////////////////////////////
    };

    //// Grasping with blending
    auto makeTestPickAndPlaceGraspingBlendingTaskVer2 = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;

        //// Gripper - ungrasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        } else { // 2-finger grasping - Open
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 20);
        }

        ////////////////////////////////////////////////////////////////
        //// Blending motion
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_HOME_TO_GRASP_APPROACH_POSE);
        taskPushBack_delay_1ms(targetModuleTask, 50);
        ////////////////////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG
            || demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH // bolt bush - planar inner grasping
            || demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // planar vacuum grasping
            //// NOTICE: 로봇의 자세 정확도 문제로 인해, 사각펙과 같이 평면 접촉 후, 흡착하는 경우에는 과도한 접촉력이 발생하여 로봇이 뻗는 문제 발생하므로, 적절한 높이는 고정하여 사용
            taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_PLANAR_VACUUM_Z_FIXED_DETECTED_GRASPING_POSE);
        } else { // default
            //// approach distance와 무관하게 수신된 detected_pose로 곧바로 이동, 파지 자세 모션(MOTION_TAG_DETECTED_GRASPING_POSE)
            taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_DETECTED_GRASPING_POSE);
        }
        taskPushBack_delay_1ms(targetModuleTask, 50);
        ////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////
        //// NOTICE: 현재 그리퍼 동작 끄고 테스트 중...
        bool is_test_no_grasping = false;
        ////////////////////////////////////////////////////////////////
        //// Gripper - grasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            if(!is_test_no_grasping) {
                taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            }
            taskPushBack_delay_1ms(targetModuleTask, 1000);

            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.040, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 50);

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) { // inner grasping
            if(!is_test_no_grasping) {
                taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            }
            taskPushBack_delay_1ms(targetModuleTask, 1000);
            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.040, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 50);

        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - ON
            if(!is_test_no_grasping) {
                taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
            }
            taskPushBack_delay_1ms(targetModuleTask, 1000);
            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.040, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 50);

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP) { // vacuum grasping - ON
            if(!is_test_no_grasping) {
                taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
            }
            taskPushBack_delay_1ms(targetModuleTask, 1000);
            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.005, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 500);

            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.035, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 50);

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // vacuum grasping - ON
            if(!is_test_no_grasping) {
                taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
            }
            taskPushBack_delay_1ms(targetModuleTask, 1000);
            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.005, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 500);

            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.035, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 50);

        } else { // 2-finger grasping - close

            if(!is_test_no_grasping) {
                taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            }
            taskPushBack_delay_1ms(targetModuleTask, 1000);

            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.005, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 50);
            if(!is_test_no_grasping) {
                taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            }
            taskPushBack_delay_1ms(targetModuleTask, 50);
            taskPushBack_csMove2(targetModuleTask, {0, 0, 0.035, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 50);
        }



        //// Gripper additional close command 1
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) { // inner grasping

        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP) { // vacuum grasping

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // vacuum grasping

        } else { // 2-finger grasping
            if(!is_test_no_grasping) {
                taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            }
            taskPushBack_delay_1ms(targetModuleTask, 20);
            // //// Check grasping success/failure
            // taskPushBack_taskRecog_KORASGripperCheckGrasping(targetModuleTask);
            // taskPushBack_delay_1ms(targetModuleTask, 20);
        }


        // taskPushBack_csMove2(targetModuleTask, {0, 0, 0.080, 0, 0, 0}, 0.75, 0.5, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 50);
        // //// Gripper additional close command 1
        // if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping

        // } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) { // inner grasping

        // } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping

        // } else { // 2-finger grasping
        //     taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        //     taskPushBack_delay_1ms(targetModuleTask, 20);
        //     // //// Check grasping success/failure
        //     // taskPushBack_taskRecog_KORASGripperCheckGrasping(targetModuleTask);
        //     // taskPushBack_delay_1ms(targetModuleTask, 20);
        // }

    };


    auto makeTestStackingTask = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        // double acc_move, vel_move, acc_contact, vel_contact;
        // if(slow_mode) {
        //     acc_move = 0.2; vel_move = 0.1;
        //     acc_contact = 0.075; vel_contact = 0.03;
        // } else {
        //     acc_move = 0.6; vel_move = 0.3;
        //     acc_contact = 0.2; vel_contact = 0.1;
        // }

        // double apporach_distance = 0.0;
        // if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) {
        //     apporach_distance = 0.0075;
        // } else if (demo_tag == BPDemoType::DEMO_BP_CNC_LATHE_CYLINDER) {
        //     apporach_distance = 0.042;
        // } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) {
        //     apporach_distance = 0.01;
        // } else { // 2-finger grasping - Open
        //     apporach_distance = 0.01;
        // }

        // // pose, acc(0.05m/s^2), vel(0.0033m/s)
        // //// TODO: JSON파일의 z margin과 연동해야 함.
        // //// Tool frame 기준 approach 자세
        // taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, apporach_distance, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 50);

        // // Gripper - ungrasp
        // if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
        //     taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        //     taskPushBack_delay_1ms(targetModuleTask, 1000);
        // } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
        //     taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
        //     taskPushBack_delay_1ms(targetModuleTask, 1000);
        // } else { // 2-finger grasping - Open
        //     taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        //     taskPushBack_delay_1ms(targetModuleTask, 1000);
        // }
        // taskPushBack_countDetachingPose(targetModuleTask);
        // taskPushBack_delay_1ms(targetModuleTask, 10);

        // taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -apporach_distance, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 50);
        // ////////////////////////////////////////////////////////////////
    };

    auto makeTestPickAndPlaceDoStackingTaskVer2 = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 0.6; vel_move = 0.3;
            acc_contact = 0.2; vel_contact = 0.1;
        }

        double apporach_distance = 0.0;
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) {
            apporach_distance = 0.0075;
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_LATHE_CYLINDER) {
            apporach_distance = 0.035;

        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) {
            apporach_distance = 0.01;

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_2F_GRP) {
            apporach_distance = 0.003;

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP) {
            apporach_distance = 0.01;

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_ELBOW_JOINT) {
            apporach_distance = 0.01;

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_CYLINDER) {
            apporach_distance = 0.035;

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) {
            apporach_distance = 0.0075;

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) {
            apporach_distance = 0.002;

        } else { // 2-finger grasping - Open
            apporach_distance = 0.01;
        }

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        //// TODO: JSON파일의 z margin과 연동해야 함.
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, apporach_distance, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// Stacking mode start!
        taskPushBack_startStackingMode(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        ////////////////////////////////////////////////////////////////
        //// Gripper - ungrasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            // taskPushBack_delay_1ms(targetModuleTask, 5000);
            taskPushBack_delay_1ms(targetModuleTask, 1500);
        } else { // 2-finger grasping - Open
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        }

        // taskPushBack_countDetachingPose(targetModuleTask);
        // taskPushBack_delay_1ms(targetModuleTask, 10);

        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -apporach_distance, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);



        //// 재파지
        ////////////////////////////////////////////////////////////////
        if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG
            || demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_2F_GRP) { // USING ZIG POSE DETECTION
            //// NOTICE: J6 limit 문제로 인해 추출된 Zig pose의 방위와 동일하게 설정

            taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -0.015, 0, 0, 0}, acc_move, vel_move, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(targetModuleTask, 100);

            taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_REGRASPING_POSE_ORI_DETECTED);

        } else { // default

            taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_REGRASPING_POSE);
        }
        taskPushBack_delay_1ms(targetModuleTask, 50);



        ////////////////////////////////////////////////////////////////


        //// Gripper - grasp [Test]
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) { // inner grasping
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
            // taskPushBack_delay_1ms(targetModuleTask, 5000);
            taskPushBack_delay_1ms(targetModuleTask, 1500);
        } else { // 2-finger grasping - Close
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        }

        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -apporach_distance, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.1, 0, 0, 0}, acc_move, vel_move, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Default JS position
        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);


        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;

        ////////////////////////////////////////////////////////////////
        //// Blending motion
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_HOME_TO_BEFORE_GRASP_APPROACH_POSE);
        taskPushBack_delay_1ms(targetModuleTask, 50);
        ////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG
            || demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH // bolt bush - planar inner grasping
            || demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // planar vacuum grasping
            //// NOTICE: 로봇의 자세 정확도 문제로 인해, 사각펙과 같이 평면 접촉 후, 흡착하는 경우에는 과도한 접촉력이 발생하여 로봇이 뻗는 문제 발생하므로, 적절한 높이는 고정하여 사용
            taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0.005, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_PLANAR_VACUUM_Z_FIXED_BEFORE_DETECTED_GRASPING_POSE);

        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_2F_GRP) {
            taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, -0.01, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_BEFORE_DETECTED_GRASPING_POSE);

        } else { // default
            //// approach distance와 무관하게 수신된 detected_pose로 곧바로 이동, 파지 자세 모션(MOTION_TAG_DETECTED_GRASPING_POSE)
            taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0.005, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_BEFORE_DETECTED_GRASPING_POSE);
        }
        taskPushBack_delay_1ms(targetModuleTask, 50);
        ////////////////////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////
        //// Gripper - ungrasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            // taskPushBack_delay_1ms(targetModuleTask, 5000);
            taskPushBack_delay_1ms(targetModuleTask, 1500);
        } else { // 2-finger grasping - Open
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        }


        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG
            || demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH // bolt bush - planar inner grasping
            || demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // planar vacuum grasping
            //// NOTICE: 로봇의 자세 정확도 문제로 인해, 사각펙과 같이 평면 접촉 후, 흡착하는 경우에는 과도한 접촉력이 발생하여 로봇이 뻗는 문제 발생하므로, 적절한 높이는 고정하여 사용
            taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0.200, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_PLANAR_VACUUM_Z_FIXED_BEFORE_DETECTED_GRASPING_POSE);
        } else { // default
            //// approach distance와 무관하게 수신된 detected_pose로 곧바로 이동, 파지 자세 모션(MOTION_TAG_DETECTED_GRASPING_POSE)
            taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0.200, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_BEFORE_DETECTED_GRASPING_POSE);
        }
        taskPushBack_delay_1ms(targetModuleTask, 50);
        ////////////////////////////////////////////////////////////////

        //// Default JS position
        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);


        //// TODO: here
        //// TODO: here
        //// TODO: here

        //// Stacking mode stop!
        taskPushBack_stopStackingMode(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        ////////////////////////////////////////////////////////////////

    };


    auto makeTestSingleGripperMachineTendingLatheLoadingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 2.0; vel_move = 1.0;
            acc_contact = 0.2; vel_contact = 0.1;
        }
        //// Cylinder demo - single gripper
        // Feeding position
        // CS: 0.05304, -0.99463, 0.45220, 179.187, -0.172, -179.921
        taskPushBack_jsMove(targetModuleTask, {82.911, -60.350, 49.772, -78.497, -90.467, -96.737}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // Insertion - 투입 직전 위치로부터 30mm 떨어진 곳에서 88mm 삽입 (펙은 58mm 삽입), pnematic chuck
        // Insertion - 투입 직전 위치로부터 30mm 떨어진 곳에서 55mm 삽입 (펙은 25mm 삽입), Lathe Chuck
        //// Tool frame 기준 approach 자세
        // taskPushBack_csMoveToolFrame(targetModuleTask, {-0.088, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        // taskPushBack_csMoveToolFrame(targetModuleTask, {-0.055, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_csMoveToolFrame(targetModuleTask, {-0.055, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // Feeder ON
        // taskPushBackGripperRelayCmd(targetModuleTask, PNE1_ON);
        taskPushBackPLCModbusCmd(targetModuleTask, LATHE_CHUCK_CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 2500);

        // KORAS gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// CNC 문 바깥으로 (빼는 동작 후, tool 기준 +y로 500mm 이동한 CS 자세)
        taskPushBack_csMoveToolFrame(targetModuleTask, {0.120, 0, -0.010, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0.400, 0, 0, 0, 0}, acc_move, vel_move, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);


        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_CLOSE);
        // taskPushBack_delay_1ms(targetModuleTask, 5000);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        // taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        // taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_ON);
        // taskPushBack_delay_1ms(targetModuleTask, 1000);
    };

    auto makeTestSingleGripperMachingTendingLatheUnloadingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 2.0; vel_move = 1.0;
            acc_contact = 0.2; vel_contact = 0.1;
        }

        taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 5000);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_ON);
        taskPushBack_delay_1ms(targetModuleTask, 1000);


        ////////////////////////////////////////////////
        //// Cylinder demo - Unloading
        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 6000);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// CNC 문 안으로 (tool 기준 -y로 400mm 이동한 CS 자세)
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, -0.400, 0, 0, 0, 0}, acc_move, vel_move, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);
        taskPushBack_csMoveToolFrame(targetModuleTask, {-0.120, 0, 0.010, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper close 1
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        //// Feeder OFF
        // taskPushBackGripperRelayCmd(targetModuleTask, PNE1_OFF);
        taskPushBackPLCModbusCmd(targetModuleTask, LATHE_CHUCK_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 500);

        //// Gripper close 2
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// 배출
        //// Tool frame 기준 approach 자세
        // taskPushBack_csMoveToolFrame(targetModuleTask, {0.055, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_csMoveToolFrame(targetModuleTask, {0.05, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

    };

    auto makeTestSingleGripperMachineTendingMillingLoadingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 2.0; vel_move = 1.0;
            acc_contact = 0.2; vel_contact = 0.1;
        }
        //// Cylinder demo - single gripper
        // Feeding position
        // CS: -0.26097, -0.99145, 0.19359, 179.834, -0.120, 135.024 (밀링 척 투입 티칭 자세에서 tool기준 z 50mm 위)
        taskPushBack_jsMove(targetModuleTask, {65.319, -53.969, 76.133, -111.814, -90.255, -69.277}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // Insertion - 투입 직전 위치로부터 50mm 떨어진 곳에서 50mm 삽입, Milling Chuck
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.05, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // KORAS gripper Open
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        // Feeder ON
        // taskPushBackGripperRelayCmd(targetModuleTask, PNE1_ON);
        taskPushBackPLCModbusCmd(targetModuleTask, MILLING_CHUCK_CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 2500);

        //// CNC 문 바깥으로 (빼는 동작 후, tool 기준 +y로 500mm 이동한 CS 자세)
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -0.01, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        taskPushBack_csMove2(targetModuleTask, {-0.10788, -0.57674, 0.48707, 179.483, -0.753, 135.090}, acc_move, vel_move, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_CLOSE);
        // taskPushBack_delay_1ms(targetModuleTask, 5000);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        // taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        // taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_ON);
        // taskPushBack_delay_1ms(targetModuleTask, 1000);
    };

    auto makeTestSingleGripperMachingTendingMillingUnloadingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 2.0; vel_move = 1.0;
            acc_contact = 0.2; vel_contact = 0.1;
        }

        taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 5000);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_ON);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        ////////////////////////////////////////////////
        //// Cylinder demo - Unloading
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 6000);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// CNC 문 안으로 (tool 기준 -y로 400mm 이동한 CS 자세)
        //// TODO: -0.26097, -0.99145, 0.19359, 179.834, -0.120, 135.024
        taskPushBack_csMove2(targetModuleTask, {-0.26408, -0.99122, 0.19363, 179.726, -0.675, 135.073}, acc_move, vel_move, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);
        taskPushBack_csMoveToolFrame(targetModuleTask, {0.004, -0.004, 0.05, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper close
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_ON);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        //// Feeder OFF
        taskPushBackPLCModbusCmd(targetModuleTask, MILLING_CHUCK_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 1500);

        //// 배출
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {-0.004, +0.004, -0.05, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeTestSingleGripperMachineTendingBoltBushLoadingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 2.0; vel_move = 1.0;
            acc_contact = 0.2; vel_contact = 0.1;
        }
        //// Cylinder demo - single gripper
        // Feeding position
        // CS: -0.26545, -0.98927, 0.12231, 179.581, -0.726, 135.085
        taskPushBack_jsMove(targetModuleTask, {65.196, -53.404, 76.241, -112.041, -90.736, -69.459}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // Insertion - 투입 직전 위치로부터 30mm 떨어진 곳에서 55mm 삽입 (펙은 25mm 삽입), Lathe Chuck
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.05, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // KORAS gripper - ungrasp, inner grasping
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        // // Feeder ON
        // taskPushBackPLCModbusCmd(targetModuleTask, MILLING_CHUCK_CLOSE);
        // taskPushBack_delay_1ms(targetModuleTask, 2500);

        //// CNC 문 바깥으로 (빼는 동작 후, tool 기준 +y로 500mm 이동한 CS 자세)
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -0.05, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        taskPushBack_csMove2(targetModuleTask, {-0.10788, -0.57674, 0.42857, 179.483, -0.753, 135.090}, acc_move, vel_move, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_CLOSE);
        // taskPushBack_delay_1ms(targetModuleTask, 5000);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        // taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        // taskPushBack_delay_1ms(targetModuleTask, 20);
        // taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_ON);
        // taskPushBack_delay_1ms(targetModuleTask, 1000);
    };

    auto makeTestSingleGripperMachingTendingBoltBushUnloadingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        double acc_move, vel_move, acc_contact, vel_contact;
        if(slow_mode) {
            acc_move = 0.2; vel_move = 0.1;
            acc_contact = 0.075; vel_contact = 0.03;
        } else {
            acc_move = 2.0; vel_move = 1.0;
            acc_contact = 0.2; vel_contact = 0.1;
        }

        taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 5000);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_ON);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        ////////////////////////////////////////////////
        //// Cylinder demo - Unloading
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 6000);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_ON);
        taskPushBack_delay_1ms(targetModuleTask, 20);
        taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// CNC 문 안으로
        taskPushBack_jsMove(targetModuleTask, {65.196, -53.404, 76.241, -112.041, -90.736, -69.459}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // Insertion - 투입 직전 위치로부터 30mm 떨어진 곳에서 55mm 삽입 (펙은 25mm 삽입), Lathe Chuck
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.05, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper - grasp, inner grasping
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::OPEN);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        // //// Feeder OFF
        // taskPushBackPLCModbusCmd(targetModuleTask, MILLING_CHUCK_OPEN);
        // taskPushBack_delay_1ms(targetModuleTask, 1500);

        //// 배출
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -0.05, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeTestSingleGripperMachingTendingPartDetachingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        ////////////////////////////////////////////////
        //// Cylinder demo - Unloaded parts detaching

        //////// Blending start
        //// CNC 문 바깥
        if(slow_mode) {
            taskPushBack_csMove2(targetModuleTask, {0.15325, -0.44732, 0.39671, 179.286, 1.072, 179.751}, 0.3, 0.15, false); // false: absolute, true: relative
        } else {
            taskPushBack_csMove2(targetModuleTask, {0.15325, -0.44732, 0.39671, 179.286, 1.072, 179.751}, 2.0, 1.0, false); // false: absolute, true: relative
        }
        taskPushBack_delay_1ms(targetModuleTask, 100);
        //// Detaching 위치 (떨어뜨리는 곳)
        if(slow_mode) {
            taskPushBack_csMove2(targetModuleTask, {-0.34293, -0.44248, 0.18734, 179.278, 1.075, 179.735}, 0.3, 0.15, false); // false: absolute, true: relative
        } else {
            taskPushBack_csMove2(targetModuleTask, {-0.34293, -0.44248, 0.18734, 179.278, 1.075, 179.735}, 2.0, 1.0, false); // false: absolute, true: relative
        }
        taskPushBack_delay_1ms(targetModuleTask, 100);
        //////// Blending end

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 1000);
        //////// Blending end

    };

    auto makeTestPickAndPlaceTaskCylinder = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        ////////////////////////////////////////////////
        //// Welding joint demo
        //// Detaching 위치 (떨어뜨리는 곳)
        if(slow_mode) {
            taskPushBack_csMove2(targetModuleTask, {-0.40899, -0.60293, 0.18714, 179.266, 1.080, 179.736}, 0.3, 0.15, false); // false: absolute, true: relative
        } else {
            taskPushBack_csMove2(targetModuleTask, {-0.40899, -0.60293, 0.18714, 179.266, 1.080, 179.736}, 2.0, 1.0, false); // false: absolute, true: relative
        }
        taskPushBack_delay_1ms(targetModuleTask, 100);
        //////// Blending end

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
        //////// Blending end
    };

    auto makeTestGripperOpenTask = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag) {
        // Gripper ungrasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 60);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 60);
        } else { // 2-finger grasping - Open
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 60);
        }
        // taskPushBack_countDetachingPose(targetModuleTask);
        // taskPushBack_delay_1ms(targetModuleTask, 10);
    };

    auto makeTestPickAndPlaceTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        ////////////////////////////////////////////////
        // taskPushBack_csMove2(targetModuleTask, {0.37444, -0.06028, 0.30000, -179.780, -0.131, 179.575}, 0.75, 0.5, false); // false: absolute, true: relative

        // JS: 200.137, -84.322, 93.815, -99.295, -90.156, 20.559
        // CS: 0.63939, 0.42011, 0.30040, -179.786, -0.127, 179.576
        taskPushBack_csMove2(targetModuleTask, {0.63939, 0.42011, 0.30000, -179.786, -0.127, 179.576}, 0.75, 0.5, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 1200);
    };

    auto makeTestPickAndPlaceP2PBlendingTaskVer2 = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {

        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;

        ////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////
        //// Blending motion, MotionTag::MOTION_TAG_BLENDING_HOME_TO_GRASP_APPROACH_POSE
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_P2P_POSE);
        taskPushBack_delay_1ms(targetModuleTask, 100);
        ////////////////////////////////////////////////////////////////

        //// Gripper - ungrasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 5000);
        } else { // 2-finger grasping - Open
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        }

        //// J6 0deg --> 블렌딩 모션 반복하다보면 파지 자세에 따라 6축 관절각도 제한을 벗어나는 경우 발생  ? < -360
        //// absolute J6 motion target --> 6축을 0deg로 명령
        taskPushBack_jsMoveOnlyJ6(targetModuleTask, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, false); // i: index, false: absolute, true: relative

    };

    auto makeTestPickAndPlaceP2PBlendingTaskVer2TestToBeDeleted = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {

        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;

        ////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////
        //// Blending motion, MotionTag::MOTION_TAG_BLENDING_HOME_TO_GRASP_APPROACH_POSE
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_P2P_POSE);
        taskPushBack_delay_1ms(targetModuleTask, 100);
        ////////////////////////////////////////////////////////////////

        //// J6 0deg --> 블렌딩 모션 반복하다보면 파지 자세에 따라 6축 관절각도 제한을 벗어나는 경우 발생  ? < -360
        //// absolute J6 motion target --> 6축을 0deg로 명령
        taskPushBack_jsMoveOnlyJ6(targetModuleTask, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, false); // i: index, false: absolute, true: relative


        //// Zig Approach Pose
        taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0.05, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_DETECTED_ZIG_POSE);
        taskPushBack_delay_1ms(targetModuleTask, 50);

        // 티칭 위치보다 위에서 떨어뜨리도록 (다른 자세에서 충돌 발생하는 것을 방지하기 위함)
        //// TODO: 그리퍼와 지그 환경과의 충돌을 방지해야 함.
        taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0.002, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_DETECTED_ZIG_POSE);
        taskPushBack_delay_1ms(targetModuleTask, 50);




        //// Gripper - ungrasp
        if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_BOLT_BUSH) { // inner grasping
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        } else if (demo_tag == BPDemoType::DEMO_BP_PICK_SQUARE_PEG) { // vacuum grasping - OFF
            taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
            taskPushBack_delay_1ms(targetModuleTask, 5000);
        } else { // 2-finger grasping - Open
            taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
            taskPushBack_delay_1ms(targetModuleTask, 1200);
        }

        //// Zig Approach Pose
        taskPushBack_taskRecog_csMoveMotionTag(targetModuleTask, {0, 0, 0.05, 0, 0, 0}, 0.3, 0.15, demo_tag, MotionTag::MOTION_TAG_DETECTED_ZIG_POSE);
        taskPushBack_delay_1ms(targetModuleTask, 50);

    };


    auto makeTestPickAndPlaceStackingBlendingTaskVer2 = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, uint16_t motion_tag, bool slow_mode) {

        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;

        ////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////
        //// Blending motion, MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_JSON_POSE
        //// Blending motion, MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_DETECTED_ZIG_POSE
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, motion_tag);
        taskPushBack_delay_1ms(targetModuleTask, 100);
        ////////////////////////////////////////////////////////////////
    };



    auto makeTestPickAndPlaceVacuumTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        ////////////////////////////////////////////////
        taskPushBack_csMove2(targetModuleTask, {0.37444, -0.06028, 0.30000, -179.780, -0.131, 179.575}, 0.75, 0.5, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper Open
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::VACUUM_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 1200);
    };

    auto makeTestInnerPickAndPlaceTask = [&] (std::vector<UnitTask>& targetModuleTask) {
        ////////////////////////////////////////////////
        //// Welding joint demo
        // Feeding position
        taskPushBack_jsMove(targetModuleTask, {167.267, -70.608, -110.328, -92.411, 88.511, 176.306}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // KORAS gripper Close
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

    };

    //// Module task - grasping
    auto makeTestDualGripperGraspingTask = [&] (std::vector<UnitTask>& targetModuleTask) {

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        // 상자 위 투입 직전 position
        taskPushBack_jsMove(targetModuleTask, {65.435, -108.792, -58.861, -146.723, 72.872, 73.338}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_doGrasping_binPicking(targetModuleTask, 0.1, 0.1); // Move to grasping pose, (acc, vel)

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        //// TODO: JSON파일의 z margin과 연동해야 함.
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, 0.04, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.050, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper additional close command 1
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 400);

        // 상자 위 투입 직전 position
        taskPushBack_jsMove(targetModuleTask, {65.435, -108.792, -58.861, -146.723, 72.872, 73.338}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // Gripper #1 default position
        taskPushBack_jsMove(targetModuleTask, {21.594, -92.093, -83.072, -136.916, 102.995, 107.571}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
        //// Gripper additional close command 2
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 400);
        // //// Check grasping success/failure
        // taskPushBack_taskRecog_KORASGripperCheckGrasping(targetModuleTask);
        // taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeTestDualGripperMachingTendingUnloadingTask = [&] (std::vector<UnitTask>& targetModuleTask) {
        ////////////////////////////////////////////////
        //// Cylinder demo - Unloading
        // Dual gripper pose change - gripper #2 default position
        taskPushBack_jsMove(targetModuleTask, {21.595, -92.094, -83.091, -136.894, 102.985, -73.187}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // Set TCP
        taskPushBack_selectTCP(targetModuleTask, 6); // Right hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper driver change
        taskPushBack_setGripperDriver(targetModuleTask, 2); // Driver #2 Right hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        // Unloading
        taskPushBack_jsMove(targetModuleTask, {17.468, -110.583, -68.974, -134.589, 103.231, -77.610}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_csMove2(targetModuleTask, {0, 0, -0.04, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        //// Gripper close 1
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        //// Feeder OFF
        // taskPushBack_gripper_pne1(targetModuleTask, false);
        taskPushBackGripperRelayCmd(targetModuleTask, PNE1_OFF);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper close 2
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// 배출
        // taskPushBack_csMove2(targetModuleTask, {-0.088, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 100);
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {0.088, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

    };

    auto makeTestDualGripperMachingTendingLoadingTask = [&] (std::vector<UnitTask>& targetModuleTask) {
        ////////////////////////////////////////////////
        //// Cylinder demo - Loading
        // Set TCP
        taskPushBack_selectTCP(targetModuleTask, 5); // Left hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper driver change
        taskPushBack_setGripperDriver(targetModuleTask, 1); // Driver #1 Left hand (for loading)
        taskPushBack_delay_1ms(targetModuleTask, 200);


        // Feeding position
        taskPushBack_jsMove(targetModuleTask, {19.469, -103.218, -82.595, -129.301, 104.891, 104.288}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // Insertion - 투입 직전 위치로부터 30mm 떨어진 곳에서 88mm 삽입 (펙은 58mm 삽입)
        // taskPushBack_csMove2(targetModuleTask, {0.088, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 100);
        //// Tool frame 기준 approach 자세
        taskPushBack_csMoveToolFrame(targetModuleTask, {-0.088, 0, 0, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 100);

        // Feeder ON
        // taskPushBack_gripper_pne1(targetModuleTask, true);
        taskPushBackGripperRelayCmd(targetModuleTask, PNE1_ON);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_csMove2(targetModuleTask, {-0.050, 0, 0.060, 0, 0, 0}, 0.075, 0.03, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 400);

    };

    auto makeTestDualGripperMachingTendingPartDetachingTask = [&] (std::vector<UnitTask>& targetModuleTask) {
        ////////////////////////////////////////////////
        //// Cylinder demo - Unloaded parts detaching

        // Part detaching position
        taskPushBack_jsMove(targetModuleTask, {-0.510, -113.416, -69.432, -134.141, 92.437, -87.860}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // Set TCP
        taskPushBack_selectTCP(targetModuleTask, 6); // Right hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper driver change
        taskPushBack_setGripperDriver(targetModuleTask, 2); // Driver #2 Right hand (for unloading)
        taskPushBack_delay_1ms(targetModuleTask, 200);

        //// Gripper Open
        taskPushBack_taskRecog_KORASGripperCmd(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // Middle position
        taskPushBack_jsMove(targetModuleTask, {25.644, -77.373, -96.711, -121.404, 49.098, 36.153}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
    };

    //// Blending module task
    auto makeTestBlendingInit2RegraspingZigTask = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            // xd = 0.75;
            // xdd = 1.5;
            // waypoint_xd = 0.5;
            // radius = 30; // radius percent %
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_HOME_TO_REGRASP_ZIG);
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeTestBlendingRegraspingZig2CNCTask = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            // xd = 0.75;
            // xdd = 1.5;
            // waypoint_xd = 0.25;
            // radius = 30; // radius percent %
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_REGRASP_ZIG_TO_CNC_INDOOR);
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeTestBlendingCNC2DetachTask = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            xd = 0.75;
            xdd = 1.0;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_CNC_INDOOR_TO_DETACH);
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    //// Module task - grasping
    auto makeTestBlendingCNCDetach2InitTask = [&] (std::vector<UnitTask>& targetModuleTask, uint16_t demo_tag, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            // xd = 0.75;
            // xdd = 1.5;
            // waypoint_xd = 0.5;
            // radius = 30; // radius percent %
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, demo_tag, MotionTag::MOTION_TAG_BLENDING_DETACH_TO_HOME_1);
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeTestBlendingInit2CNCMillingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            // xd = 0.75;
            // xdd = 1.5;
            // waypoint_xd = 0.5;
            // radius = 30; // radius percent %
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG, MotionTag::MOTION_TAG_BLENDING_HOME_TO_CNC_INDOOR);
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };

    auto makeTestBlendingCNCMillingDetach2InitTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            // xd = 0.75;
            // xdd = 1.5;
            // waypoint_xd = 0.5;
            // radius = 30; // radius percent %
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }
        BlendingTraj blending_traj;
        blending_traj.xd = xd;
        blending_traj.xdd = xdd;
        blending_traj.waypoint_xd = waypoint_xd;
        blending_traj.radius = radius;
        taskPushBack_taskRecog_moveBlending(targetModuleTask, blending_traj, BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG, MotionTag::MOTION_TAG_BLENDING_DETACH_TO_HOME_1);
        taskPushBack_delay_1ms(targetModuleTask, 100);
    };


    auto makeTestIfCheckTipChangingTask = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {

        // pose, acc(0.05m/s^2), vel(0.0033m/s)
        float tip_changing_insertion_distance = 0.025; // 팁 장착위치로부터 떨어진 거리
        float tip_changing_exit_from_tip_home_distance = 0.05; // 팁 거치대의 내부에서 퇴장하기 위한 거리

        double acc_target = 0.5;
        double vel_target = 0.25;

        //// Check matching finished and tip changing parameters update
        taskPushBack_checkMatchingFinishedAndTipChanging(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// Task Flag ON
        taskPushBack_setTipChangingFlag(targetModuleTask, true);

        ////////////////////////////
        //// Detaching
        ////////////////////////////
        taskPushBack_taskRecog_TipChangingDefaultDetach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// Gripper
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 200);

        ////// a) JS position - 지그 앞 5cm
        taskPushBack_taskRecog_TipChangingDetach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// TCP setting: TCP default - 혼동을 막기 위해, 로봇의 기본으로 세팅된 TCP의 방위를 사용
        // Set TCP (#1~) master(#7)
        taskPushBack_selectTCP(targetModuleTask, 7); // TCP default

        ////// b) 거치대 내부 입장
        //// Tool frame 기준 - 툴 거치대의 내부에 입장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, -tip_changing_exit_from_tip_home_distance, 0, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        //// Tool frame 기준 - 툴 탈착 후 퇴장
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -tip_changing_insertion_distance, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        ////////////////////////////
        //// Attaching
        ////////////////////////////
        taskPushBack_taskRecog_TipChangingDefaultAttach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
        ////// a) JS position - 장착 위치 앞 2.5cm
        taskPushBack_taskRecog_TipChangingAttach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);
        //// Tool frame 기준 - 팁 장착
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, +tip_changing_insertion_distance, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);
        ////// b) 장착 후, 거치대 내부 퇴장
        //// Tool frame 기준 - 툴 거치대의 내부에서 퇴장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, tip_changing_exit_from_tip_home_distance, 0, 0, 0, 0}, acc_target, vel_target, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_taskRecog_TipChangingDefaultDetach_jsMove(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_taskRecog_TipChangingSetTCP(targetModuleTask);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        //// Task Flag OFF
        taskPushBack_setTipChangingFlag(targetModuleTask, false);

    };

    ////////////////////////////////////////////////////////////////////////////////
    // Pick-and-place - bolt bush (inner plane grasping)
    makeTestInitializeTCPAndGripper (module_task_bin_picking_pick_and_place_bolt_bush_, 4, 1);
    makeTestPickAndPlaceGraspingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_bolt_bush_, BPDemoType::DEMO_BP_PICK_BOLT_BUSH, is_slow_mode);
    if(0) { //// Point 2 Point Task
        makeTestPickAndPlaceP2PBlendingTaskVer2 (module_task_bin_picking_pick_and_place_bolt_bush_, BPDemoType::DEMO_BP_PICK_BOLT_BUSH, is_slow_mode);
    } else { //// Stacking Task
        uint16_t motion_tag_bolt_bush = MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_JSON_POSE;
        makeTestPickAndPlaceStackingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_bolt_bush_, BPDemoType::DEMO_BP_PICK_BOLT_BUSH, motion_tag_bolt_bush, is_slow_mode);
        makeTestPickAndPlaceDoStackingTaskVer2 (module_task_bin_picking_pick_and_place_bolt_bush_, BPDemoType::DEMO_BP_PICK_BOLT_BUSH, is_slow_mode);
    }
    makeTestCheckMatchingFinishedTaskVer2 (module_task_bin_picking_pick_and_place_bolt_bush_);
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////
    // Pick-and-place - square peg
    // [Vacuum] (With Blending)
    makeTestInitializeTCPAndGripper (module_task_bin_picking_pick_and_place_square_peg_, 11, 1); // square peg with vacuum gripper(4 vacuum)
    makeTestPickAndPlaceGraspingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_square_peg_, BPDemoType::DEMO_BP_PICK_SQUARE_PEG, is_slow_mode);

    if(0) { //// Point 2 Point Task
        makeTestPickAndPlaceP2PBlendingTaskVer2 (module_task_bin_picking_pick_and_place_square_peg_, BPDemoType::DEMO_BP_PICK_SQUARE_PEG, is_slow_mode);
    } else { //// Stacking Task
        //// NOTICE: Test Zig Poise p2p
        // makeTestPickAndPlaceP2PBlendingTaskVer2TestToBeDeleted (module_task_bin_picking_pick_and_place_square_peg_, BPDemoType::DEMO_BP_PICK_SQUARE_PEG, is_slow_mode);
        uint16_t motion_tag_square_peg = MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_DETECTED_ZIG_POSE;
        makeTestPickAndPlaceStackingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_square_peg_, BPDemoType::DEMO_BP_PICK_SQUARE_PEG, motion_tag_square_peg, is_slow_mode);
        makeTestPickAndPlaceDoStackingTaskVer2 (module_task_bin_picking_pick_and_place_square_peg_, BPDemoType::DEMO_BP_PICK_SQUARE_PEG, is_slow_mode);
    }
    makeTestCheckMatchingFinishedTaskVer2 (module_task_bin_picking_pick_and_place_square_peg_);
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////
    // Pick-and-place - t joint
    // [2F gripper] (With Blending)
    makeTestInitializeTCPAndGripper (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_, 3, 1);
    makeTestPickAndPlaceGraspingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_, BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_2F_GRP, is_slow_mode);
    if(0) { //// Point 2 Point Task
        makeTestPickAndPlaceP2PBlendingTaskVer2 (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_, BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_2F_GRP, is_slow_mode);
    } else { //// Stacking Task
        uint16_t motion_tag_t_joint = MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_DETECTED_ZIG_POSE;
        makeTestPickAndPlaceStackingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_, BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_2F_GRP, motion_tag_t_joint, is_slow_mode);
        makeTestPickAndPlaceDoStackingTaskVer2 (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_, BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_2F_GRP, is_slow_mode);
    }
    makeTestCheckMatchingFinishedTaskVer2 (module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_);
    ////////////////////////////////////////////////////////////////////////////////



    ////////////////////////////////////////////////////////////////////////////////
    // Pick-and-place - cylinder
    // [2F gripper] (With Blending)
    makeTestInitializeTCPAndGripper (module_task_bin_picking_pick_and_place_cylinder_, 13, 1); // cylinder with tip-changing (round tip)
    makeTestPickAndPlaceGraspingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_cylinder_, BPDemoType::DEMO_BP_PICK_CYLINDER, is_slow_mode);
    if(0) { //// Point 2 Point Task
        makeTestPickAndPlaceP2PBlendingTaskVer2 (module_task_bin_picking_pick_and_place_cylinder_, BPDemoType::DEMO_BP_PICK_CYLINDER, is_slow_mode);
    } else { //// Stacking Task
        uint16_t motion_tag_cylinder = MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_JSON_POSE;
        makeTestPickAndPlaceStackingBlendingTaskVer2 (module_task_bin_picking_pick_and_place_cylinder_, BPDemoType::DEMO_BP_PICK_CYLINDER, motion_tag_cylinder, is_slow_mode);
        makeTestPickAndPlaceDoStackingTaskVer2 (module_task_bin_picking_pick_and_place_cylinder_, BPDemoType::DEMO_BP_PICK_CYLINDER, is_slow_mode);
    }
    makeTestCheckMatchingFinishedTaskVer2 (module_task_bin_picking_pick_and_place_cylinder_);
    ////////////////////////////////////////////////////////////////////////////////


    //// Grasping with blending
    auto makeTestGraphyStage5OnlyPickAndPlace = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }

        //// Gripper - ungrasp
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::POS_CTRL, 9500);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);


        //// NOTICE: THIS TASK ONLY PICK AND PLACE (NO TIP CHANGING)
        //// 파지 자세 1 위
        taskPushBack_csMove2(targetModuleTask, {-0.29109, 0.13255, 0.15421, 179.993, 0.007, -115.306}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// 파지 자세 1
        taskPushBack_csMove2(targetModuleTask, {-0.29109, 0.13255, 0.01508, 179.993, 0.007, -115.306}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        taskPushBack_csMove2(targetModuleTask, {-0.29109, 0.13253, -0.00055, 179.997, 0.004, -115.306}, 0.075, 0.05, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1500);

        //// 파지 자세 1 위
        taskPushBack_csMove2(targetModuleTask, {-0.29109, 0.13255, 0.15421, 179.993, 0.007, -115.306}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);


        // //////////////////////////////////////////////////////////////////////////////////////
        // //// 파지 자세 2 위
        // taskPushBack_csMove2(targetModuleTask, {-0.32219, 0.47538, 0.15426, -179.592, -0.152, -2.871}, 0.3, 0.15, false); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 50);

        // //// 파지 자세 2a
        // taskPushBack_csMove2(targetModuleTask, {-0.26622, 0.47537, 0.12710, -179.323, 14.132, -2.638}, 0.3, 0.15, false); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 50);

        // //// 파지 자세 2b
        // taskPushBack_csMove2(targetModuleTask, {-0.24904, 0.47541, 0.12707, -179.324, 14.128, -2.643}, 0.3, 0.15, false); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 50);
        // //////////////////////////////////////////////////////////////////////////////////////


        //////////////////////////////////////////////////////////////////////////////////////
        //// 파지 자세 2 위
        taskPushBack_csMove2(targetModuleTask, {-0.40342, 0.46761, 0.18000, -179.326, 13.926, 178.364}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// 파지 자세 2a
        taskPushBack_csMove2(targetModuleTask, {-0.40342, 0.46761, 0.12999, -179.326, 13.926, 178.364}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// 파지 자세 2b
        taskPushBack_csMove2(targetModuleTask, {-0.40342, 0.46761, 0.11522, -179.327, 13.925, 178.375}, 0.1, 0.05, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);
        //////////////////////////////////////////////////////////////////////////////////////

        //// Gripper - ungrasp
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::POS_CTRL, 9500);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        //// Tool frame 기준 - 퇴장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -0.05, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.05, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);
        //////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////



        //////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////// 2 ///////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////
        //// Gripper - ungrasp
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::POS_CTRL, 9500);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);


        //// NOTICE: THIS TASK ONLY PICK AND PLACE (NO TIP CHANGING)
        //// 파지 자세 1 위
        taskPushBack_csMove2(targetModuleTask, {-0.29109, 0.13255, 0.15421, 179.993, 0.007, -115.306}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// 파지 자세 1
        taskPushBack_csMove2(targetModuleTask, {-0.29109, 0.13255, 0.01508, 179.993, 0.007, -115.306}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        taskPushBack_csMove2(targetModuleTask, {-0.29109, 0.13253, -0.00055, 179.997, 0.004, -115.306}, 0.075, 0.05, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1500);

        //// 파지 자세 1 위
        taskPushBack_csMove2(targetModuleTask, {-0.29109, 0.13255, 0.15421, 179.993, 0.007, -115.306}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //////////////////////////////////////////////////////////////////////////////////////
        //// 파지 자세 2 위
        taskPushBack_csMove2(targetModuleTask, {-0.35144, 0.43195, 0.18000, -179.327, 13.940, -2.245}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// 파지 자세 2a
        taskPushBack_csMove2(targetModuleTask, {-0.35144, 0.43195, 0.12999, -179.327, 13.940, -2.245}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// 파지 자세 2b
        taskPushBack_csMove2(targetModuleTask, {-0.35144, 0.43195, 0.11522, -179.327, 13.940, -2.245}, 0.1, 0.05, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);
        //////////////////////////////////////////////////////////////////////////////////////

        //// Gripper - ungrasp
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::POS_CTRL, 9500);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        //// Tool frame 기준 - 퇴장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -0.05, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.05, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);
        //////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////


        //////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////// 3 ///////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////
        //// Gripper - ungrasp
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::POS_CTRL, 9500);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);


        //// NOTICE: THIS TASK ONLY PICK AND PLACE (NO TIP CHANGING)
        //// 파지 자세 1 위
        taskPushBack_csMove2(targetModuleTask, {-0.29109, 0.13255, 0.15421, 179.993, 0.007, -115.306}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// 파지 자세 1
        taskPushBack_csMove2(targetModuleTask, {-0.29109, 0.13255, 0.01508, 179.993, 0.007, -115.306}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        taskPushBack_csMove2(targetModuleTask, {-0.29109, 0.13253, -0.00055, 179.997, 0.004, -115.306}, 0.075, 0.05, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1500);

        //// 파지 자세 1 위
        taskPushBack_csMove2(targetModuleTask, {-0.29109, 0.13255, 0.15421, 179.993, 0.007, -115.306}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);


        //////////////////////////////////////////////////////////////////////////////////////
        //// 파지 자세 2 위
        taskPushBack_csMove2(targetModuleTask, {-0.35145, 0.49979, 0.18000, -179.324, 13.939, -2.239}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// 파지 자세 2a
        taskPushBack_csMove2(targetModuleTask, {-0.35145, 0.49979, 0.12999, -179.324, 13.939, -2.239}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// 파지 자세 2b
        taskPushBack_csMove2(targetModuleTask, {-0.35145, 0.49979, 0.11522, -179.324, 13.939, -2.239}, 0.1, 0.05, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);
        //////////////////////////////////////////////////////////////////////////////////////

        //// Gripper - ungrasp
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::POS_CTRL, 9500);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        //// Tool frame 기준 - 퇴장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -0.05, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.05, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);
        //////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////

    };

    //// Grasping with blending
    auto makeTestGraphyStage5PickAndPlace = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode, size_t test_idx) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }

        //// Gripper - ungrasp
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::POS_CTRL, 9500);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// 파지 자세 1 위
        taskPushBack_csMove2(targetModuleTask, {-0.29109, 0.13255, 0.15421, 179.993, 0.007, -115.306}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// 파지 자세 1
        taskPushBack_csMove2(targetModuleTask, {-0.29109, 0.13255, 0.01508, 179.993, 0.007, -115.306}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);


        taskPushBack_csMove2(targetModuleTask, {-0.29109, 0.13253, -0.00055, 179.997, 0.004, -115.306}, 0.075, 0.05, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1500);

        //// 파지 자세 1 위
        taskPushBack_csMove2(targetModuleTask, {-0.29109, 0.13255, 0.15421, 179.993, 0.007, -115.306}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        ////////////////////////////////////////////////////////////////////////////////////////
        // //// 파지 자세 2 위
        // taskPushBack_csMove2(targetModuleTask, {-0.32219, 0.47538, 0.15426, -179.592, -0.152, -2.871}, 0.3, 0.15, false); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 50);

        // //// 파지 자세 2a
        // taskPushBack_csMove2(targetModuleTask, {-0.26622, 0.47537, 0.12710, -179.323, 14.132, -2.638}, 0.3, 0.15, false); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 50);

        // //// 파지 자세 2b
        // taskPushBack_csMove2(targetModuleTask, {-0.24904, 0.47541, 0.12707, -179.324, 14.128, -2.643}, 0.3, 0.15, false); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 50);
        ////////////////////////////////////////////////////////////////////////////////////////

        switch(test_idx)
        {
            case 1:
                ////////////////////////////////////////////////////////////////////////////////////////
                //// 파지 자세 2 위
                taskPushBack_csMove2(targetModuleTask, {-0.06363, 0.40514, 0.17517, -179.769, 0.621, -106.996}, 0.3, 0.15, false); // false: absolute, true: relative
                taskPushBack_delay_1ms(targetModuleTask, 50);

                //// 파지 자세 2a
                taskPushBack_csMove2(targetModuleTask, {-0.06290, 0.40763, 0.06070, -179.772, 0.621, -106.996}, 0.3, 0.15, false); // false: absolute, true: relative
                taskPushBack_delay_1ms(targetModuleTask, 50);

                //// 파지 자세 2b
                taskPushBack_csMove2(targetModuleTask, {-0.06462, 0.39946, 0.04891, -179.620, 23.529, -106.778}, 0.3, 0.15, false); // false: absolute, true: relative
                taskPushBack_delay_1ms(targetModuleTask, 50);
                ////////////////////////////////////////////////////////////////////////////////////////

                break;

            case 2:
                ////////////////////////////////////////////////////////////////////////////////////////
                //// 파지 자세 2 위
                taskPushBack_csMove2(targetModuleTask, {0.01216, 0.43143, 0.17517, -179.321, 14.806, -59.254}, 0.3, 0.15, false); // false: absolute, true: relative
                taskPushBack_delay_1ms(targetModuleTask, 50);

                //// 파지 자세 2a
                taskPushBack_csMove2(targetModuleTask, {0.01216, 0.43143, 0.06070, -179.321, 14.806, -59.254}, 0.3, 0.15, false); // false: absolute, true: relative
                taskPushBack_delay_1ms(targetModuleTask, 50);

                //// 파지 자세 2b
                taskPushBack_csMove2(targetModuleTask, {0.01216, 0.43143, 0.04891, -179.321, 14.806, -59.254}, 0.1, 0.05, false); // false: absolute, true: relative
                taskPushBack_delay_1ms(targetModuleTask, 50);
                ////////////////////////////////////////////////////////////////////////////////////////

                break;
        }

        //// Gripper - ungrasp
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::POS_CTRL, 9500);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        //// Tool frame 기준 - 퇴장하기 위한 거리
        taskPushBack_csMoveToolFrame(targetModuleTask, {0, 0, -0.1, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        // taskPushBack_csMove2(targetModuleTask, {0, 0, 0.05, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

    };



    //// Grasping with blending
    auto makeTestGraphyRotationPlate = [&] (std::vector<UnitTask>& targetModuleTask, bool slow_mode, size_t test_idx) {
        //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
        double xd, xdd, waypoint_xd, radius;
        if(slow_mode) {
            xd = 0.1;
            xdd = 0.2;
            waypoint_xd = 0.1;
            radius = 30; // radius percent %
        } else {
            xd = 0.5;
            xdd = 0.75;
            waypoint_xd = 0.25;
            radius = 30; // radius percent %
        }


        // CsDouble tcp_task  = {0.0, 0.0, 0.26500, 0.0, 0.0, 90.0}; // tip change grp
        // taskPushBack_setTCP (targetModuleTask, tcp_task);

        //// Gripper - ungrasp
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::POS_CTRL, 8500);
        taskPushBack_delay_1ms(targetModuleTask, 20);

        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// 파지 자세 1 위
        taskPushBack_csMove2(targetModuleTask, {-0.05552, 0.45057, 0.18111, -179.768, 0.621, -106.989}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// 파지 자세 1
        taskPushBack_csMove2(targetModuleTask, {-0.05552, 0.45057, 0.08111, -179.768, 0.621, -106.989}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::CLOSE);
        taskPushBack_delay_1ms(targetModuleTask, 1500);

        //// 파지 자세 상승
        taskPushBack_csMove2(targetModuleTask, {0.0, 0.0, 0.01, 0.0, 0.0, 0.0}, 0.1, 0.05, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

        //// 파지 자세 회전 직전 위 1
        taskPushBack_csMove2(targetModuleTask, {-0.04457, 0.46076, 0.09712, -179.766, 0.621, -106.988}, 0.3, 0.15, false); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);


        switch(test_idx)
        {
            case 1:
                taskPushBack_jsMove(targetModuleTask, {0.0, 0.0, 0.0, 0.0, 0.0, 90.0}, true); // i: index, false: absolute, true: relative
                taskPushBack_delay_1ms(targetModuleTask, 50);

                //// 파지 자세 회전 직후 위 1
                taskPushBack_csMove2(targetModuleTask, {-0.05191, 0.45399, 0.09712, 179.572, 0.427, 163.004}, 0.3, 0.15, false); // false: absolute, true: relative
                taskPushBack_delay_1ms(targetModuleTask, 50);

                //// 파지 자세 회전 직후 위 2
                taskPushBack_csMove2(targetModuleTask, {-0.05191, 0.45399, 0.08926, 179.572, 0.427, 163.004}, 0.3, 0.15, false); // false: absolute, true: relative
                taskPushBack_delay_1ms(targetModuleTask, 50);

                //// 파지 자세 회전 직후
                taskPushBack_csMove2(targetModuleTask, {-0.05191, 0.45399, 0.08311, 179.572, 0.427, 163.004}, 0.1, 0.05, false); // false: absolute, true: relative
                taskPushBack_delay_1ms(targetModuleTask, 50);

                break;

            case 2:
                taskPushBack_jsMove(targetModuleTask, {0.0, 0.0, 0.0, 0.0, 0.0, 45.0}, true); // i: index, false: absolute, true: relative
                taskPushBack_delay_1ms(targetModuleTask, 50);

                //// 파지 자세 회전 직후 위 1
                taskPushBack_csMove2(targetModuleTask, {-0.05533, 0.45201, 0.09712, 179.863, 0.664, -151.995}, 0.3, 0.15, false); // false: absolute, true: relative
                taskPushBack_delay_1ms(targetModuleTask, 50);

                //// 파지 자세 회전 직후 위 2
                taskPushBack_csMove2(targetModuleTask, {-0.05533, 0.45201, 0.08926, 179.863, 0.664, -151.995}, 0.3, 0.15, false); // false: absolute, true: relative
                taskPushBack_delay_1ms(targetModuleTask, 50);

                //// 파지 자세 회전 직후
                taskPushBack_csMove2(targetModuleTask, {-0.05533, 0.45201, 0.08311, 179.863, 0.664, -151.995}, 0.1, 0.05, false); // false: absolute, true: relative
                taskPushBack_delay_1ms(targetModuleTask, 50);

                break;
        }

        //// Gripper - ungrasp
        taskPushBackKORASGripperCmd(targetModuleTask, (uint16_t)KR_GRP::POS_CTRL, 8500);
        taskPushBack_delay_1ms(targetModuleTask, 1000);

        taskPushBack_csMove2(targetModuleTask, {0, 0, 0.15, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 60);

        taskPushBack_jsMove(targetModuleTask, {-19.678, -117.392, 105.335, -77.862, -89.799, 160.236}, false); // i: index, false: absolute, true: relative
        taskPushBack_delay_1ms(targetModuleTask, 50);

    };

    ////////////////////////////////////////////////////////////////////////////////
    // 그래피 테스트 1 - 단순 놓기 팁체인징 안함
    makeTestInitializeTCPAndGripper (module_task_graphy_stage5_only_grasping_test_, 13, 1); // cylinder with tip-changing (round tip)
    makeTestGraphyStage5OnlyPickAndPlace (module_task_graphy_stage5_only_grasping_test_, is_slow_mode);
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////
    // 그래피 테스트 2 - 기울여 넣는 작업
    makeTestInitializeTCPAndGripper (module_task_graphy_stage5_grasping_test_1_, 13, 1); // cylinder with tip-changing (round tip)
    makeTestGraphyStage5PickAndPlace (module_task_graphy_stage5_grasping_test_1_, is_slow_mode, 1);

    makeTestInitializeTCPAndGripper (module_task_graphy_stage5_plate_rotation_test_1_, 13, 1); // cylinder with tip-changing (round tip)
    makeTestGraphyRotationPlate (module_task_graphy_stage5_plate_rotation_test_1_, is_slow_mode, 1);
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////
    // 그래피 테스트 3 - 기울여 넣는 작업
    makeTestInitializeTCPAndGripper (module_task_graphy_stage5_grasping_test_2_, 13, 1); // cylinder with tip-changing (round tip)
    makeTestGraphyStage5PickAndPlace (module_task_graphy_stage5_grasping_test_2_, is_slow_mode, 2);

    makeTestInitializeTCPAndGripper (module_task_graphy_stage5_plate_rotation_test_2_, 13, 1); // cylinder with tip-changing (round tip)
    makeTestGraphyRotationPlate (module_task_graphy_stage5_plate_rotation_test_2_, is_slow_mode, 2);
    ////////////////////////////////////////////////////////////////////////////////



    //// LOG
    ROS_LOG_INFO("[The task has been created successfully]: %s", __FUNCTION__);
}
bool TaskPlanner::makeGraphyTaskList() {
    //makeGraphyTaskListDefault();
    module_task_graphy_.clear();
    module_task_graphy_2.clear();
    module_task_graphy_3.clear();
    module_task_graphy_4.clear();
    module_task_graphy_5.clear();
    module_task_graphy_6.clear();
    module_task_graphy_7.clear();
    module_task_graphy_8.clear();
    module_task_graphy_9.clear();
    module_task_graphy_10.clear();
    module_task_graphy_11.clear();
    module_task_graphy_12.clear();
    module_task_graphy_13.clear();
    module_task_graphy_14.clear();
    module_task_graphy_15.clear();
    module_task_graphy_16.clear();
    module_task_graphy_17.clear();
    module_task_graphy_18.clear();
    module_task_graphy_19.clear();
    module_task_graphy_20.clear();
    module_task_graphy_21.clear();
    module_task_graphy_22.clear();
    module_task_graphy_23.clear();
    module_task_graphy_24.clear();
    module_task_graphy_25.clear();
    module_task_graphy_26.clear();
    module_task_graphy_27.clear();
    module_task_graphy_28.clear();
    module_task_graphy_29.clear();
    JsDouble task_position1 = {70.667, -65.546, -95.444, -82.379, 95.778, 88.854};
    JsDouble task_position2 = {60.667, -65.546, -95.444, -82.379, 95.778, 88.864};
    int main_grp  = 1;
    CsDouble tcp_  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    CsDouble tcp_0  = {0.0, 0.0, 0.17, 0.0, 0.0, 0.0};
    CsDouble tcp_1  = {-0.12, 0.0, 0.5, 0.0, 0.0, 0.0};
    double acc_move, vel_move;
    acc_move = 0.25; vel_move = 0.12/2;
    CsDouble tcp_3  = {0.0, 0.0, 0.375, 0.0, 0.0, 0.0};
    CsDouble tcp_4  = {-0.14, 0.0, 0.28, 0.0, 0.0, 0.0};

    std::vector<double> home_pose  = {0.21310, -0.26284, 0.41406, -179.99, 0.001, -179.99};
    std::vector<double> printer_before_hand  = {-0.19171, -0.36427, 0.55059, -179.99+45.0, 0.0, -179.99};
    taskPushBack_setTCP(module_task_graphy_2, tcp_);
    taskPushBack_csMove2(module_task_graphy_2, home_pose, acc_move*2, vel_move*2, false);
    taskPushBack_delay(module_task_graphy_2, 10);
    taskPushBack_setTCP(module_task_graphy_2, tcp_);
    taskPushBack_csMove2(module_task_graphy_2, printer_before_hand, acc_move, vel_move, false);
    taskPushBackKORASGripperCmd(module_task_graphy_2, (uint16_t)KR_GRP::POS_CTRL, 8100, 1000);
    taskPushBack_delay(module_task_graphy_2, 50);
    taskPushBack_csMoveToolFrame(module_task_graphy_2, {0, 0, 0.125, 0, 0, 0}, acc_move, vel_move/2, true);
    taskPushBack_delay(module_task_graphy_2, 5);
    taskPushBack_setTCP(module_task_graphy_2, tcp_0);
    taskPushBack_delay(module_task_graphy_2, 5);
    taskPushBack_csMove2(module_task_graphy_2, {0, 0, 0, +34, 0, 0}, acc_move, vel_move/2, true);
    taskPushBack_delay(module_task_graphy_2, 5);
    taskPushBack_setTCP(module_task_graphy_2, tcp_3);
    taskPushBack_delay(module_task_graphy_2, 5);
    taskPushBack_csMove2(module_task_graphy_2, {0, 0, 0, -60, 0, 0}, acc_move, vel_move/4, true);
    taskPushBack_setTCP(module_task_graphy_2, tcp_0);
    taskPushBack_delay(module_task_graphy_2, 5);
    taskPushBack_csMove2(module_task_graphy_2, {0, 0, 0, +30, 0, 0}, acc_move, vel_move/2, true);
    taskPushBack_delay(module_task_graphy_2, 5);
    taskPushBack_csMove2(module_task_graphy_2, {0, -0.05*1.4, 0.025*1.4, 0, 0, 0}, acc_move, vel_move/2, true);
    taskPushBack_delay(module_task_graphy_2, 5);
    taskPushBack_csMoveToolFrame(module_task_graphy_2, {0, 0, -0.15, 0, 0, 0}, acc_move, vel_move/2, true);
    taskPushBack_delay(module_task_graphy_2, 5);
    std::vector<double> bolt_before_hand0  = {-0.41554,-0.49027+0.2,0.60597,-179.99,-60.00,-90.00};
    std::vector<double> bolt_before_hand  = {-0.41554,-0.49027,0.60597,-179.99,-60.00,-90.00};
    CsDouble tcp_6  = {0.0, 0.0, 0.20, 0.0, 0.0, 0.0};
    taskPushBack_setTCP(module_task_graphy_2, tcp_);
    taskPushBack_csMove2(module_task_graphy_2, home_pose, acc_move*2, vel_move*2, false);
    taskPushBack_delay(module_task_graphy_2, 10);
    taskPushBack_setTCP(module_task_graphy_2, tcp_);
    taskPushBack_csMove2(module_task_graphy_2, bolt_before_hand0, acc_move*2, vel_move*3, false);
    taskPushBackKORASGripperCmd(module_task_graphy_2, (uint16_t)KR_GRP::POS_CTRL, 4500, 1000);
    taskPushBack_delay(module_task_graphy_2, 50);
    taskPushBack_csMove2(module_task_graphy_2, bolt_before_hand, acc_move, vel_move/2, false);
    taskPushBack_delay(module_task_graphy_2, 5);
    taskPushBack_setTCP(module_task_graphy_2, tcp_6);
    taskPushBack_delay(module_task_graphy_2, 5);
    taskPushBack_csMove2(module_task_graphy_2, {0, 0, 0, 0, 0, -30}, acc_move, vel_move/2, true);
    taskPushBack_irl_grp(module_task_graphy_2, KR_GRP::CLOSE, 0, main_grp);
    taskPushBack_delay(module_task_graphy_2, 100);
    taskPushBack_csMove2(module_task_graphy_2, {0, 0, 0, 0, 0, 30}, acc_move, vel_move/2, true);
    taskPushBackKORASGripperCmd(module_task_graphy_2, (uint16_t)KR_GRP::POS_CTRL, 0, 1000);
    taskPushBack_delay(module_task_graphy_2, 50);
    taskPushBack_csMove2(module_task_graphy_2, {0, 0.05, 0, 0, 0, 0}, acc_move, vel_move/2, true);
    taskPushBack_csMove2(module_task_graphy_2, {0, 0, 0, 0, -30, 0}, acc_move, vel_move/2, true);
    taskPushBack_csMove2(module_task_graphy_2, {0, 0, -0.055, 0, 0, 0}, acc_move, vel_move/2, true);
    taskPushBack_csMove2(module_task_graphy_2, {0, -0.07, 0, 0, 0, 0}, acc_move, vel_move/2, true);
    taskPushBack_irl_grp(module_task_graphy_2, KR_GRP::CLOSE, 0, main_grp);
    taskPushBack_delay(module_task_graphy_2, 100);

    taskPushBack_csMove2(module_task_graphy_2, {0, 0.25, 0, 0, 0, 0}, acc_move, vel_move/2, true);
    std::vector<double> motion1  = {0.06723, -0.59239, 0.26379+0.17, -179.99, -89.00, -90.00};
    std::vector<double> motion2  = {0.06723, -0.59239, 0.26379, 90.00, 89.99, 0.00};

    taskPushBack_csMove2(module_task_graphy_2, motion1, acc_move, vel_move/2, false);
    taskPushBack_jsMove(module_task_graphy_2, {0, 0, 0, 0, 0, 180.0}, true);
    taskPushBack_csMove2(module_task_graphy_2, motion2, acc_move, vel_move*2, false);
    taskPushBack_csMove2(module_task_graphy_2, {0.0,0.0,-0.17,0.0,0.0,0.0}, acc_move*3, vel_move/2, true);
    taskPushBackKORASGripperCmd(module_task_graphy_2, (uint16_t)KR_GRP::POS_CTRL, 0, 1000);
    taskPushBack_csMove2(module_task_graphy_2, {0.0, 0.1, 0.0, 0.0, 0.0, 0.0}, acc_move*3, vel_move/2, true);
    taskPushBack_csMove2(module_task_graphy_2, {0.0, 0.0, 0.2, 0.0, 0.0, 0.0}, acc_move*3, vel_move/2, true);

    double tc_cs_vel = 0.04;
    taskPushBack_setTCP(module_task_graphy_, tcp_);
    std::vector<double> tc_pose0 = {-0.18873,-0.38,0.03570+0.2,179.99,0.0,90.0};
    std::vector<double> tc_pose1 = {-0.18873,-0.38,0.03570,179.99,0.0,90.0};
    std::vector<double> tc_pose2 = {-0.18873,-0.31009,0.03570,179.99,0.0,90.0};
    std::vector<double> tc_pose3 = {-0.18873,-0.31009,0.03570+0.05,179.99,0.0,90.0};

    taskPushBack_csMove2(module_task_graphy_3, tc_pose0, acc_move*2, vel_move, false);
    taskPushBack_csMove2(module_task_graphy_3, tc_pose1, acc_move*2, tc_cs_vel, false);
    taskPushBack_csMove2(module_task_graphy_3, tc_pose2, acc_move*2, tc_cs_vel, false);
    taskPushBack_csMove2(module_task_graphy_3, tc_pose3, acc_move*3, vel_move/1.5, false);

    taskPushBack_csMove2(module_task_graphy_4, tc_pose3, acc_move*3, vel_move/1.5, false);
    taskPushBack_csMove2(module_task_graphy_4, tc_pose2, acc_move*2, tc_cs_vel, false);
    taskPushBack_initializeGripper(module_task_graphy_4);
    taskPushBack_csMove2(module_task_graphy_4, tc_pose1, acc_move*2, tc_cs_vel, false);
    taskPushBack_csMove2(module_task_graphy_4, tc_pose0, acc_move*2, vel_move, false);
    taskPushBack_jsMove(module_task_graphy_4, {0, 0, 0, 0, 0, -180.0}, true);

    taskPushBack_setTCP(module_task_graphy_, tcp_);
    std::vector<double> sc_pose0 = {-0.0914,-0.39,0.34,179.99,0.0,90.0};
    std::vector<double> sc_pose1 = {-0.0914,-0.54750,0.31286+0.02,179.99,0.0,90.0};
    taskPushBack_csMove2(module_task_graphy_5, sc_pose0, acc_move*3, vel_move/1.5, false);
    taskPushBack_csMove2(module_task_graphy_5, sc_pose1, acc_move*3, vel_move/1.5, false);
    taskPushBack_csMove2(module_task_graphy_5, {0.0,0.0,-0.02,0.0,0.0,0.0}, acc_move*3, tc_cs_vel/2, true);

    //to do VEL_CTRL VEL_CTRL
    // taskPushBackKORASGripperCmd(module_task_graphy_5, (uint16_t)KR_GRP::VEL_CTRL, -1000, 1000);
    // taskPushBack_delay(module_task_graphy_5, 200);
    // taskPushBackKORASGripperCmd(module_task_graphy_5, (uint16_t)KR_GRP::VEL_CTRL, 0, 0);
    taskPushBack_delay(module_task_graphy_5, 10);
    taskPushBackKORASGripperCmd(module_task_graphy_5, (uint16_t)KR_GRP::VACUUM_OFF);
    taskPushBack_delay(module_task_graphy_5, 10);
    taskPushBack_delay(module_task_graphy_5, 50);

    taskPushBack_csMove2(module_task_graphy_5, {0.0,0.0,0.02,0.0,0.0,0.0}, acc_move*3, tc_cs_vel/2, true);

    taskPushBack_setTCP(module_task_graphy_6, tcp_);
    taskPushBack_csMove2(module_task_graphy_6, home_pose, acc_move*2, vel_move*2, false);
    taskPushBackKORASGripperCmd(module_task_graphy_6, (uint16_t)KR_GRP::POS_CTRL, 8100, 1000);
    taskPushBack_delay(module_task_graphy_6, 50);
    taskPushBack_csMove2(module_task_graphy_6, {-0.0131, -0.033, 0, 0, 0, 22.5}, acc_move, vel_move/2, true);
    taskPushBack_delay(module_task_graphy_6, 5);
    taskPushBack_csMove2(module_task_graphy_6, {0, 0, -0.07, 0, 0, 0}, acc_move, vel_move/2, true);
    taskPushBack_irl_grp(module_task_graphy_6, KR_GRP::CLOSE, 0, main_grp);
    taskPushBack_delay(module_task_graphy_6, 80);
    taskPushBack_csMove2(module_task_graphy_6, {0, 0, 0.23, 0, 0, 0}, acc_move, vel_move/2, true);
    taskPushBack_delay(module_task_graphy_6, 5);
    taskPushBack_csMove2(module_task_graphy_6, {0.373, 0.1558, 0, 0, 0, 0}, acc_move, vel_move/2, true);
    taskPushBack_delay(module_task_graphy_6, 5);
    taskPushBack_csMove2(module_task_graphy_6, {0, 0, -0.22059+0.007, 0, 0, 0}, acc_move, vel_move/3, true);
    taskPushBack_csMove2(module_task_graphy_6, {0, 0, -0.007, 0, 0, 0}, acc_move, vel_move/10, true);
    taskPushBack_csMove2(module_task_graphy_6, {0, 0, 0, 0, 0, 90}, acc_move, vel_move, true);
    taskPushBackKORASGripperCmd(module_task_graphy_6, (uint16_t)KR_GRP::POS_CTRL, 6000, 1000);
    taskPushBack_delay(module_task_graphy_6, 120);
    taskPushBack_csMove2(module_task_graphy_6, {0, 0, 0.22, 0, 0, 0}, acc_move, vel_move/3, true);

    std::vector<double> home_pose2  = {0.21310, -0.26284, 0.53, -179.99, 0.001, -90.0};
    std::vector<double> pick_pose2  = {0.57301, -0.14004, 0.57347, -179.99, 0.001, -90.0};
    taskPushBack_setTCP(module_task_graphy_7, tcp_);
    taskPushBack_csMove2(module_task_graphy_7, pick_pose2, acc_move*2, vel_move*2, false);
    taskPushBackKORASGripperCmd(module_task_graphy_7, (uint16_t)KR_GRP::POS_CTRL, 8100, 1000);
    taskPushBack_setTCP(module_task_graphy_7, tcp_);
    taskPushBack_csMove2(module_task_graphy_7, {0, 0, -0.22, 0, 0, 0}, acc_move, vel_move/3, true);
    taskPushBack_irl_grp(module_task_graphy_7, KR_GRP::CLOSE, 0, main_grp);
    taskPushBack_delay(module_task_graphy_7, 80);
    taskPushBack_csMove2(module_task_graphy_7, {0, 0, 0.22, 0, 0, 0}, acc_move, vel_move/3, true);
    taskPushBack_csMove2(module_task_graphy_7, home_pose2, acc_move*2, vel_move*2, false);
    taskPushBack_csMove2(module_task_graphy_7, home_pose, acc_move*2, vel_move*2, false);
    taskPushBack_csMove2(module_task_graphy_7, {-0.0131, -0.033, 0, 0, 0, -22.5}, acc_move, vel_move/2, true);
    taskPushBack_csMove2(module_task_graphy_7, {0, 0, -0.069, 0, 0, 0}, acc_move, vel_move/2, true);
    taskPushBackKORASGripperCmd(module_task_graphy_7, (uint16_t)KR_GRP::POS_CTRL, 8100, 1000);

    CsDouble tcp_2  = {0.2, -0.2, 0.17, -20.0, 0.0, 0.0};
    std::vector<double> pose3  = {0.33893, 0.29146, 0.45219, -179.99+20.00, 0.00, -90.0};
    taskPushBack_setTCP(module_task_graphy_8, tcp_);
    taskPushBackKORASGripperCmd(module_task_graphy_8, (uint16_t)KR_GRP::POS_CTRL, 8800, 1000);
    taskPushBack_csMove2(module_task_graphy_8, pose3, acc_move, vel_move, false);
    taskPushBack_delay(module_task_graphy_8, 5);
    taskPushBack_csMoveToolFrame(module_task_graphy_8, {0, 0, 0.03, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_delay(module_task_graphy_8, 5);
    taskPushBack_setTCP(module_task_graphy_8, tcp_2);
    taskPushBack_csMove2(module_task_graphy_8, {0, 0, 0, 0, 0, 60}, acc_move, vel_move, true);
    taskPushBack_delay(module_task_graphy_8, 5);
    taskPushBack_csMove2(module_task_graphy_8, {0, 0, 0.13, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_delay(module_task_graphy_8, 5);

    CsDouble tcp_5  = {0.0, 0.0, 0.19, 0.0, 0.0, 0.0};
    std::vector<double> home_pose3  = {0.21310, -0.26284, 0.41406, -179.99, 0.001, -179.99+90};
    taskPushBack_setTCP(module_task_graphy_9, tcp_);
    taskPushBack_csMove2(module_task_graphy_9, home_pose3, acc_move*2, vel_move*2, false);
    taskPushBack_delay(module_task_graphy_9, 10);
    taskPushBackKORASGripperCmd(module_task_graphy_9, (uint16_t)KR_GRP::POS_CTRL, 7500, 1000);
    taskPushBack_csMove2(module_task_graphy_9, {0.244-0.003, -0.234-0.003, 0, 0, 0, 45.0}, acc_move, vel_move, true);
    taskPushBack_delay(module_task_graphy_9, 10);
    taskPushBack_csMoveToolFrame(module_task_graphy_9, {0, 0, 0.05, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_delay(module_task_graphy_9, 10);
    taskPushBack_setTCP(module_task_graphy_9, tcp_5);
    taskPushBack_csMove2(module_task_graphy_9, {0, 0, 0, 0, -60.0, 0}, acc_move, vel_move, true);
    taskPushBack_irl_grp(module_task_graphy_9, KR_GRP::CLOSE, 0, main_grp);
    taskPushBack_delay(module_task_graphy_9, 80);
    taskPushBack_setTCP(module_task_graphy_9, tcp_0);
    taskPushBack_csMove2(module_task_graphy_9, {0, 0, 0.45, 0, 0, 45}, acc_move, vel_move, true);
    taskPushBack_csMove2(module_task_graphy_9, {0.05, 0.75, -0.04, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBackKORASGripperCmd(module_task_graphy_9, (uint16_t)KR_GRP::POS_CTRL, 8000, 1000);
    taskPushBack_delay(module_task_graphy_9, 80);
    taskPushBack_csMove2(module_task_graphy_9, {0.0, 0.0, -0.026, 0, -30.0, 0}, acc_move, vel_move, true);
    taskPushBack_irl_grp(module_task_graphy_9, KR_GRP::CLOSE, 0, main_grp);
    taskPushBack_delay(module_task_graphy_9, 80);
    taskPushBack_csMove2(module_task_graphy_9, {0.0, 0.0, 0.01, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_csMove2(module_task_graphy_9, {0.0, 0.15, 0.0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_csMove2(module_task_graphy_9, {-0.15, 0.0, 0.0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_csMove2(module_task_graphy_9, {0.0, 0.0,-0.333, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_csMove2(module_task_graphy_9, {0.0, -0.13, 0.0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_csMove2(module_task_graphy_9, {0.22, 0.0, 0.0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBackKORASGripperCmd(module_task_graphy_9, (uint16_t)KR_GRP::POS_CTRL, 6000, 1000);
    taskPushBack_delay(module_task_graphy_9, 100);
    taskPushBack_csMove2(module_task_graphy_9, {-0.25, 0, 0.0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_csMove2(module_task_graphy_9, {0.0, 0.1, 0.4, 0, 0, 0}, acc_move, vel_move, true);

    std::vector<double> pose1  = {0.39022, -0.36259, 0.42508, -135.00, 0.00, -90.0};
    taskPushBack_setTCP(module_task_graphy_10, tcp_);
    taskPushBackKORASGripperCmd(module_task_graphy_10, (uint16_t)KR_GRP::POS_CTRL, 8200, 1000);
    taskPushBack_delay(module_task_graphy_10, 5);
    taskPushBack_csMove2(module_task_graphy_10, pose1, acc_move, vel_move, false);
    taskPushBack_delay(module_task_graphy_10, 5);
    taskPushBack_csMoveToolFrame(module_task_graphy_10, {0, 0, 0.07, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_setTCP(module_task_graphy_10, tcp_0);
    uint div = 3;
    uint target_deg = 105;
    double r = 0.192;
    for (uint i=0; i<div; i++){
        taskPushBack_csMove2(module_task_graphy_10, {0.55682+r*(1-cos(M_PI/180*target_deg/div*(i+1))),-0.35984,0.26414+r*sin(M_PI/180*target_deg/div*(i+1)),-135+10.000*(i+1), 0, -90.0}, acc_move*1.2, vel_move*1.2, false);
    }
    taskPushBackKORASGripperCmd(module_task_graphy_10, (uint16_t)KR_GRP::POS_CTRL, 5500, 1000);
    taskPushBack_delay(module_task_graphy_10, 80);
    taskPushBack_csMoveToolFrame(module_task_graphy_10, {0, 0, 0.015, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_10, {0, 0, -0.1, 0, 0, 0}, acc_move, vel_move, true);

    std::vector<double> teeth_  = {0.21141, -0.21617, 0.35498+0.07, -179.99, 0.00, 179.99};
    taskPushBack_setTCP(module_task_graphy_11, tcp_);
    taskPushBack_csMove2(module_task_graphy_11, teeth_, acc_move*2, 0.05, false);
    taskPushBack_csMove2(module_task_graphy_11, {0.0,0.0,-0.078,0.0,0.0,0.0}, acc_move*2, 0.03, true);
    taskPushBackKORASGripperCmd(module_task_graphy_11, (uint16_t)KR_GRP::POS_CTRL, 8000, 1000);
    taskPushBack_delay(module_task_graphy_11, 150);
    taskPushBack_csMove2(module_task_graphy_11, {0.0,-0.01,0.0,0.0,0.0,0.0}, acc_move*2, 0.1, true);
    taskPushBack_csMove2(module_task_graphy_11, {0.0,0.0,0.15,0.0,0.0,0.0}, acc_move*2, 0.1, true);

    std::vector<std::vector<double>> teeth_pose;
    teeth_pose = std::vector<std::vector<double>>(8, std::vector<double>(2, 0.0));
    teeth_pose[0][0] = 0.0;   teeth_pose[0][1] = 0.0;
    teeth_pose[1][0] = 0.0;   teeth_pose[1][1] = -0.04;
    teeth_pose[2][0] = 0.0;   teeth_pose[2][1] = -0.08;
    teeth_pose[3][0] = 0.055; teeth_pose[3][1] = 0.01;
    teeth_pose[4][0] = 0.055; teeth_pose[4][1] = -0.03;
    teeth_pose[5][0] = 0.11;  teeth_pose[5][1] = 0.0;
    teeth_pose[6][0] = 0.11;  teeth_pose[6][1] = -0.04;
    teeth_pose[7][0] = 0.11;  teeth_pose[7][1] = -0.08;
    uint16_t teeth_num = 4;

    std::vector<double> pose_teeth_0  = {0.00955+teeth_pose[teeth_num][0], -0.54099+teeth_pose[teeth_num][1], 0.46910+0.05, 180, 0, 180};
    double vel_test = 0.01;
    double dist_shake = 0.02;
    double value1 = -0.02;
    double value2 = 0.015;
    int grip_pose = 8100;
    if (teeth_num==3||teeth_num==4){
        value1 = -1*value1;
        value2 = -1*value1;
    }
    taskPushBackKORASGripperCmd(module_task_graphy_12, (uint16_t)KR_GRP::POS_CTRL, grip_pose, 1000);
    taskPushBack_setTCP(module_task_graphy_12, tcp_);
    taskPushBack_csMove2(module_task_graphy_12, pose_teeth_0, acc_move*2, vel_test*6, false);
    taskPushBack_csMove2(module_task_graphy_12, {0.0,0.0,-0.05,0.0,0.0,0.0}, acc_move*2, vel_test*6, true);
    taskPushBack_csMove2(module_task_graphy_12, {value1,value2,0.0,0.0,0.0,0.0}, acc_move*2, vel_test, true);
    taskPushBack_csMove2(module_task_graphy_12, {0.0,0.0,-0.02,0.0,0.0,0.0}, acc_move*2, vel_test, true);
    taskPushBack_irl_grp(module_task_graphy_12, KR_GRP::CLOSE, 0, main_grp);
    taskPushBack_delay(module_task_graphy_12, 100);
    taskPushBack_csMove2(module_task_graphy_12, {0.0,dist_shake,0.0,0.0,0.0,0.0}, acc_move*2, vel_test, true);
    taskPushBack_csMove2(module_task_graphy_12, {0.0,-2*dist_shake,0.0,0.0,0.0,0.0}, acc_move*2, vel_test, true);
    taskPushBack_csMove2(module_task_graphy_12, {0.0,dist_shake,0.0,0.0,0.0,0.0}, acc_move*2, vel_test, true);
    taskPushBackKORASGripperCmd(module_task_graphy_12, (uint16_t)KR_GRP::POS_CTRL, grip_pose, 1000);
    taskPushBack_delay(module_task_graphy_12, 100);
    taskPushBack_csMove2(module_task_graphy_12, {0.0,0.0,0.05,0.0,0.0,0.0}, acc_move*2, vel_test*3, true);
    taskPushBack_csMove2(module_task_graphy_12, {-2*value1,0.0,0.0,0.0,0.0,0.0}, acc_move*2, vel_test, true);
    taskPushBack_csMove2(module_task_graphy_12, {0.0,0.0,-0.05,0.0,0.0,0.0}, acc_move*2, vel_test*3, true);
    taskPushBack_irl_grp(module_task_graphy_12, KR_GRP::CLOSE, 0, main_grp);
    taskPushBack_delay(module_task_graphy_12, 100);
    taskPushBack_csMove2(module_task_graphy_12, {0.0,dist_shake,0.0,0.0,0.0,0.0}, acc_move*2, vel_test, true);
    taskPushBack_csMove2(module_task_graphy_12, {0.0,-2*dist_shake,0.0,0.0,0.0,0.0}, acc_move*2, vel_test, true);
    taskPushBack_csMove2(module_task_graphy_12, {0.0,dist_shake,0.0,0.0,0.0,0.0}, acc_move*2, vel_test, true);
    taskPushBackKORASGripperCmd(module_task_graphy_12, (uint16_t)KR_GRP::POS_CTRL, grip_pose, 1000);
    taskPushBack_delay(module_task_graphy_12, 100);
    taskPushBack_csMove2(module_task_graphy_12, {0.0,0.0,0.02,0.0,0.0,0.0}, acc_move*2, vel_test, true);
    taskPushBack_csMove2(module_task_graphy_12, {value1,-1*value2,0.0,0.0,0.0,0.0}, acc_move*2, vel_test, true);
    taskPushBack_irl_grp(module_task_graphy_12, KR_GRP::CLOSE, 0, main_grp);
    taskPushBack_delay(module_task_graphy_12, 100);
    taskPushBack_csMove2(module_task_graphy_12, {0.0,0.0,0.05,0.0,0.0,0.0}, acc_move*2, vel_test*3, true);
    taskPushBack_csMove2(module_task_graphy_12, {0.0,0.2,0.0,0.0,0.0,0.0}, acc_move*2, vel_test*6, true);

    taskPushBack_setTCP(module_task_graphy_13, tcp_0);
    std::vector<double> close1  = {0.79996-0.3, -0.31666, 0.48928, -140.0, 0, -90};
    taskPushBack_irl_grp(module_task_graphy_13, KR_GRP::CLOSE, 0, main_grp);
    taskPushBack_delay(module_task_graphy_13, 100);
    taskPushBack_csMove2(module_task_graphy_13, close1, acc_move*2, 0.15, false);
    taskPushBack_csMove2(module_task_graphy_13, {0.3,0.0,0.0,0.0,0.0,0.0}, acc_move*2, 0.1, true);
    taskPushBack_csMove2(module_task_graphy_13, {0.0,0.0,-0.01,0.0,0.0,0.0}, acc_move*2, 0.05, true);
    taskPushBack_csMove2(module_task_graphy_13, {-0.1,0.0,0.0,0.0,0.0,0.0}, acc_move*2, 0.05, true);

    taskPushBack_setTCP(module_task_graphy_14, tcp_);
    taskPushBack_csMove2(module_task_graphy_14, home_pose3, acc_move*2, vel_move*2, false);
    taskPushBackKORASGripperCmd(module_task_graphy_14, (uint16_t)KR_GRP::POS_CTRL, 8000, 1000);
    taskPushBack_csMove2(module_task_graphy_14, {-0.29695,+0.012,-0.2436+0.05,0.0,0.0,0.0}, acc_move*2, 0.1, true);
    taskPushBack_csMove2(module_task_graphy_14, {0.0,0.0,-0.05,0.0,0.0,0.0}, acc_move*2, 0.05, true);
    taskPushBack_irl_grp(module_task_graphy_14, KR_GRP::CLOSE, 0, main_grp);
    taskPushBack_delay(module_task_graphy_14, 100);
    taskPushBack_csMove2(module_task_graphy_14, {0.0,0.0,0.05,0.0,0.0,0.0}, acc_move*2, 0.05, true);
    std::vector<double> touch1  = {-0.35088-0.005, -0.4245+0.022, 0.26856, -179.99, 0, -90};
    taskPushBack_csMove2(module_task_graphy_14, touch1, acc_move*2, vel_move, false);
    taskPushBack_csMove2(module_task_graphy_14, {0.0,-0.015,0.0,0.0,0.0,0.0}, acc_move*2, 0.06, true);
    taskPushBack_csMove2(module_task_graphy_14, {0.0,0.015,0.023,0.0,0.0,0.0}, acc_move*2, 0.06, true);
    taskPushBack_csMove2(module_task_graphy_14, {0.0,-0.015,0.0,0.0,0.0,0.0}, acc_move*2, 0.06, true);
    taskPushBack_csMove2(module_task_graphy_14, {0.0,0.015,0.023,0.0,0.0,0.0}, acc_move*2, 0.06, true);
    taskPushBack_csMove2(module_task_graphy_14, {0.0,-0.015,0.0,0.0,0.0,0.0}, acc_move*2, 0.06, true);
    taskPushBack_csMove2(module_task_graphy_14, {0.0,0.015,0.023,0.0,0.0,0.0}, acc_move*2, 0.06, true);
    taskPushBack_csMove2(module_task_graphy_14, {0.0,-0.015,0.0,0.0,0.0,0.0}, acc_move*2, 0.06, true);
    taskPushBack_csMove2(module_task_graphy_14, {0.0,0.015,0.023,0.0,0.0,0.0}, acc_move*2, 0.06, true);
    taskPushBack_setTCP(module_task_graphy_16, tcp_);
    taskPushBack_csMove2(module_task_graphy_16, home_pose, acc_move*2, vel_move*2, false);
    taskPushBack_delay(module_task_graphy_16, 10);

//TO DO ~~
    // taskPushBack_aprilTagDetection(module_task_graphy_17, 0);
    // taskPushBack_moveToTag(module_task_graphy_17, 0);

    CsDouble tcp_17  = {0.0, 0.0, 0.19, 0.0, 20.0, 0.0};
    taskPushBack_setTCP(module_task_graphy_18, tcp_);
    taskPushBackKORASGripperCmd(module_task_graphy_18, (uint16_t)KR_GRP::POS_CTRL, 10000, 1000);
    taskPushBack_csMove2(module_task_graphy_18, home_pose, acc_move*2, vel_move*2, false);
    taskPushBack_delay(module_task_graphy_18, 20);
    taskPushBack_csMove2(module_task_graphy_18, {0.28467, -0.14432, 0.16180-0.015, -180, 20, -180}, acc_move*2, vel_move*2, false);
    taskPushBack_csMove2(module_task_graphy_18, {0.008, 0, 0, 0, 0, 0}, acc_move*2, vel_move*2, true);
    taskPushBack_csMove2(module_task_graphy_18, {-0.008, 0, 0, 0, 0, 0}, acc_move*2, vel_move*2, true);
    taskPushBackKORASGripperCmd(module_task_graphy_18, (uint16_t)KR_GRP::POS_CTRL, 3000, 1000);
    taskPushBack_delay(module_task_graphy_18, 200);
    taskPushBack_csMove2(module_task_graphy_18, {0.013, 0, 0, 0, 0, 0}, acc_move*2, vel_move*2, true);
    taskPushBackKORASGripperCmd(module_task_graphy_18, (uint16_t)KR_GRP::POS_CTRL, 500, 1000);
    taskPushBack_delay(module_task_graphy_18, 50);
    taskPushBack_irl_grp(module_task_graphy_18, KR_GRP::CLOSE, 0, main_grp);
    taskPushBack_delay(module_task_graphy_18, 120);
    taskPushBack_setTCP(module_task_graphy_18, tcp_17);
    taskPushBack_csMove2(module_task_graphy_18, {0, 0, 0, -40, 0, 0}, acc_move*2, vel_move/3, true);
    taskPushBackKORASGripperCmd(module_task_graphy_18, (uint16_t)KR_GRP::POS_CTRL, 1500, 1000);
    taskPushBack_delay(module_task_graphy_18, 40);
    taskPushBack_csMove2(module_task_graphy_18, {-0.013, 0, 0, 0, 0, 0}, acc_move*2, vel_move*2, true);
    taskPushBack_csMove2(module_task_graphy_18, {0, 0, 0.35, 0, 0, 0}, acc_move*2, vel_move*2, true);

    uint16_t pick_teeth_num = 4;

    std::vector<double> rot_teeth  = {115.0, 45.0, 45.0, 45.0-360.0,
                                      45.0, 45.0, 45.0, 45.0};

    double plate_r = 9.5;
    std::vector<std::vector<double>> plate_teeth_pose;

    uint16_t plate_teeth_num = 0;
    std::vector<double> resin_plate_center  = {-0.43696, -0.22909, 0.14924, -179.999, 0, -179.999};
    taskPushBack_setTCP(module_task_graphy_19, tcp_);
    taskPushBack_csMove2(module_task_graphy_19, home_pose, acc_move*2, vel_move*2, false);
    taskPushBack_csMove2(module_task_graphy_19, resin_plate_center, acc_move*2, vel_move*2, false);
    taskPushBack_jsMove(module_task_graphy_19, {0, 0, 0, 0, 0, rot_teeth[plate_teeth_num]}, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0.095, 0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0, 0.07, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBackKORASGripperCmd(module_task_graphy_19, (uint16_t)KR_GRP::POS_CTRL, 7500, 1000);
    taskPushBack_delay(module_task_graphy_19, 120);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0, -0.07, 0, 0, 0}, acc_move, vel_move, true);

    plate_teeth_num = 1;
    taskPushBackKORASGripperCmd(module_task_graphy_19, (uint16_t)KR_GRP::POS_CTRL, 10000, 1000);
    taskPushBack_delay(module_task_graphy_19, 120);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, -0.095, 0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_jsMove(module_task_graphy_19, {0, 0, 0, 0, 0, rot_teeth[plate_teeth_num]}, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0.095, 0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0, 0.07, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBackKORASGripperCmd(module_task_graphy_19, (uint16_t)KR_GRP::POS_CTRL, 7500, 1000);
    taskPushBack_delay(module_task_graphy_19, 120);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0, -0.07, 0, 0, 0}, acc_move, vel_move, true);
    plate_teeth_num = 2;
    taskPushBackKORASGripperCmd(module_task_graphy_19, (uint16_t)KR_GRP::POS_CTRL, 10000, 1000);
    taskPushBack_delay(module_task_graphy_19, 120);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, -0.095, 0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_jsMove(module_task_graphy_19, {0, 0, 0, 0, 0, rot_teeth[plate_teeth_num]}, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0.095, 0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0, 0.07, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBackKORASGripperCmd(module_task_graphy_19, (uint16_t)KR_GRP::POS_CTRL, 7500, 1000);
    taskPushBack_delay(module_task_graphy_19, 120);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0, -0.07, 0, 0, 0}, acc_move, vel_move, true);
    plate_teeth_num = 3;
    taskPushBackKORASGripperCmd(module_task_graphy_19, (uint16_t)KR_GRP::POS_CTRL, 10000, 1000);
    taskPushBack_delay(module_task_graphy_19, 120);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, -0.095, 0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_jsMove(module_task_graphy_19, {0, 0, 0, 0, 0, rot_teeth[plate_teeth_num]}, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0.095, 0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0, 0.07, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBackKORASGripperCmd(module_task_graphy_19, (uint16_t)KR_GRP::POS_CTRL, 7500, 1000);
    taskPushBack_delay(module_task_graphy_19, 120);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0, -0.07, 0, 0, 0}, acc_move, vel_move, true);
    plate_teeth_num = 4;
    taskPushBackKORASGripperCmd(module_task_graphy_19, (uint16_t)KR_GRP::POS_CTRL, 10000, 1000);
    taskPushBack_delay(module_task_graphy_19, 120);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, -0.095, 0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_jsMove(module_task_graphy_19, {0, 0, 0, 0, 0, rot_teeth[plate_teeth_num]}, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0.095, 0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0, 0.07, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBackKORASGripperCmd(module_task_graphy_19, (uint16_t)KR_GRP::POS_CTRL, 7500, 1000);
    taskPushBack_delay(module_task_graphy_19, 120);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0, -0.07, 0, 0, 0}, acc_move, vel_move, true);
    plate_teeth_num = 5;
    taskPushBackKORASGripperCmd(module_task_graphy_19, (uint16_t)KR_GRP::POS_CTRL, 10000, 1000);
    taskPushBack_delay(module_task_graphy_19, 120);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, -0.095, 0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_jsMove(module_task_graphy_19, {0, 0, 0, 0, 0, rot_teeth[plate_teeth_num]}, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0.095, 0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0, 0.07, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBackKORASGripperCmd(module_task_graphy_19, (uint16_t)KR_GRP::POS_CTRL, 7500, 1000);
    taskPushBack_delay(module_task_graphy_19, 120);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0, -0.07, 0, 0, 0}, acc_move, vel_move, true);
    plate_teeth_num = 6;
    taskPushBackKORASGripperCmd(module_task_graphy_19, (uint16_t)KR_GRP::POS_CTRL, 10000, 1000);
    taskPushBack_delay(module_task_graphy_19, 120);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, -0.095, 0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_jsMove(module_task_graphy_19, {0, 0, 0, 0, 0, rot_teeth[plate_teeth_num]}, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0.095, 0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0, 0.07, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBackKORASGripperCmd(module_task_graphy_19, (uint16_t)KR_GRP::POS_CTRL, 7500, 1000);
    taskPushBack_delay(module_task_graphy_19, 120);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0, -0.07, 0, 0, 0}, acc_move, vel_move, true);
    plate_teeth_num = 7;
    taskPushBackKORASGripperCmd(module_task_graphy_19, (uint16_t)KR_GRP::POS_CTRL, 10000, 1000);
    taskPushBack_delay(module_task_graphy_19, 120);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, -0.095, 0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_jsMove(module_task_graphy_19, {0, 0, 0, 0, 0, rot_teeth[plate_teeth_num]}, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0.095, 0, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0, 0.07, 0, 0, 0}, acc_move, vel_move, true);
    taskPushBackKORASGripperCmd(module_task_graphy_19, (uint16_t)KR_GRP::POS_CTRL, 7500, 1000);
    taskPushBack_delay(module_task_graphy_19, 120);
    taskPushBack_csMoveToolFrame(module_task_graphy_19, {0, 0, -0.07, 0, 0, 0}, acc_move, vel_move, true);
    std::cout << "\n----------- [The task has been created successfully] -----------" << std::endl;

    module_task_graphy_list_.clear();
    module_task_graphy_list_.push_back(module_task_graphy_);
    module_task_graphy_list_.push_back(module_task_graphy_2);
    module_task_graphy_list_.push_back(module_task_graphy_3);
    module_task_graphy_list_.push_back(module_task_graphy_4);
    module_task_graphy_list_.push_back(module_task_graphy_5);
    module_task_graphy_list_.push_back(module_task_graphy_6);
    module_task_graphy_list_.push_back(module_task_graphy_7);
    module_task_graphy_list_.push_back(module_task_graphy_8);
    module_task_graphy_list_.push_back(module_task_graphy_9);
    module_task_graphy_list_.push_back(module_task_graphy_10);
    module_task_graphy_list_.push_back(module_task_graphy_11);
    module_task_graphy_list_.push_back(module_task_graphy_12);
    module_task_graphy_list_.push_back(module_task_graphy_13);
    module_task_graphy_list_.push_back(module_task_graphy_14);
    module_task_graphy_list_.push_back(module_task_graphy_15);
    module_task_graphy_list_.push_back(module_task_graphy_16);
    module_task_graphy_list_.push_back(module_task_graphy_17);
    module_task_graphy_list_.push_back(module_task_graphy_18);
    module_task_graphy_list_.push_back(module_task_graphy_19);
    module_task_graphy_list_.push_back(module_task_graphy_20);
    module_task_graphy_list_.push_back(module_task_graphy_21);
    module_task_graphy_list_.push_back(module_task_graphy_22);
    module_task_graphy_list_.push_back(module_task_graphy_23);
    module_task_graphy_list_.push_back(module_task_graphy_24);
    module_task_graphy_list_.push_back(module_task_graphy_25);
    module_task_graphy_list_.push_back(module_task_graphy_26);
    module_task_graphy_list_.push_back(module_task_graphy_27);
    module_task_graphy_list_.push_back(module_task_graphy_28);
    module_task_graphy_list_.push_back(module_task_graphy_29);
    return true;
}

void TaskPlanner::LLMTestTask(int target_id, int vel_level) {
    llm_task_test_.clear();

    //// Scanning
    taskPushBack_taskRecog_3DScanning_sharedTask(llm_task_test_); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(llm_task_test_, 600);
    //// Matching
    taskPushBack_taskRecog_matching_sharedTask(llm_task_test_, false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(llm_task_test_, 200);


}

void TaskPlanner::genKUAISTask() {

    kuais_total_task_.clear();

    JsDouble home_pose = {-128.946, -122.367, 103.995, -51.828, -105.286, -36.267};
    taskPushBack_jsMove2(kuais_total_task_, home_pose, 45.0, 30.0, false);
    taskPushBack_delay_1ms(kuais_total_task_, 600);

    taskPushBack_task3DScanning(kuais_total_task_); // 3D Scanning
    taskPushBack_delay_1ms(kuais_total_task_, 3000);

}

/// 두산 로봇 데모
void TaskPlanner::rightGenDrumTask() {
    drum_total_task_.clear();
    drum_grp_initialize_task_.clear();
    drum_grp_grasping_pose_task_.clear();
    drum_grp_screwing_pose_task_.clear();
    drum_grp_unscrewing_pose_task_.clear();
    drum_grp_exit_pose_task_.clear();
    drum_grp_lid_cap_close_.clear();
    drum_grp_lid_cap_open_.clear();

    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    /////////////// 케미컬 그리퍼 초기화
    const int DEFAULT_GRP = 1;
    const int R5_GRP = 2;
    const int R3_GRP = 3;
    const int NEW_GRP = 4;  //New 그리퍼 ///끝에서 끝이 22000 (+-11000)
    int whatGripper; // 기본 감속비
    whatGripper = NEW_GRP;// 1: 기본 감속비, 2: 감속비 (1:5), 3: 감속비 (1:3)


    switch (whatGripper)  //그리퍼 종류 교체
    {
    case DEFAULT_GRP:
        ///// 그리퍼 Initialize
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_ENABLE);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000, 1000);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 1500);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -1400, 1000);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);
        ///// 그리퍼 파지
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_grasping_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -5500, 1000);
        taskPushBack_delay_1ms(drum_grp_grasping_pose_task_, 2000);

        ///// 체결
        taskPushBackKORASGripperCmd(drum_grp_screwing_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_screwing_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_screwing_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -10000, 1000);
        taskPushBack_delay_1ms(drum_grp_screwing_pose_task_, 60);

        //// 체결 해제
        taskPushBackKORASGripperCmd(drum_grp_unscrewing_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_unscrewing_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_unscrewing_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +10000, 300);
        taskPushBack_delay_1ms(drum_grp_unscrewing_pose_task_, 60);

        //// 나올 때
        taskPushBackKORASGripperCmd(drum_grp_exit_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_exit_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_exit_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 400, 1000);
        taskPushBack_delay_1ms(drum_grp_exit_pose_task_, 2000);

        break;

    case R5_GRP:            //// 값 확정 (수정 X)
        ///// 그리퍼 Initialize
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_ENABLE);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -8000, 1000);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 3000);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -4000, 1000);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);

        ///// 그리퍼 파지
        taskPushBackKORASGripperCmd(drum_grp_grasping_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_grasping_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_grasping_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -8000, 1000);
        taskPushBack_delay_1ms(drum_grp_grasping_pose_task_, 3000);

        ///// 체결
        taskPushBackKORASGripperCmd(drum_grp_screwing_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_screwing_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_screwing_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -12000, 1000);
        taskPushBack_delay_1ms(drum_grp_screwing_pose_task_, 3000);

        //// 체결 해제
        taskPushBackKORASGripperCmd(drum_grp_unscrewing_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_unscrewing_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_unscrewing_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 12000, 1000);
        taskPushBack_delay_1ms(drum_grp_unscrewing_pose_task_, 3000);

        //// 나올 때
        taskPushBackKORASGripperCmd(drum_grp_exit_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_exit_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_exit_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 4000, 1000);
        taskPushBack_delay_1ms(drum_grp_exit_pose_task_, 2000);

        break;

    case R3_GRP:                /// 값 확정 (수정 X)
        ///// 그리퍼 Initialize
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_ENABLE);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -4000, 1000);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 3000);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000, 1000);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);

        ///// 그리퍼 파지
        taskPushBackKORASGripperCmd(drum_grp_grasping_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_grasping_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_grasping_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -5500, 1000);
        taskPushBack_delay_1ms(drum_grp_grasping_pose_task_, 2000);

        ///// 체결
        taskPushBackKORASGripperCmd(drum_grp_screwing_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        // taskPushBack_delay_1ms(drum_grp_screwing_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_screwing_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -10000*3, 1000);
        // taskPushBack_delay_1ms(drum_grp_screwing_pose_task_, 60);

        //// 체결 해제
        taskPushBackKORASGripperCmd(drum_grp_unscrewing_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_unscrewing_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_unscrewing_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +10000*3, 300);
        taskPushBack_delay_1ms(drum_grp_unscrewing_pose_task_, 60);

        //// 나올 때
        taskPushBackKORASGripperCmd(drum_grp_exit_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_exit_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_exit_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 1600, 1000);
        taskPushBack_delay_1ms(drum_grp_exit_pose_task_, 2000);

        break;

    case NEW_GRP:                /// 값 확정 (수정 X)
        ///// 그리퍼 Initialize
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_ENABLE);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -20000, 1000);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 5000);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -700, 1000);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);

        ///// 그리퍼 파지
        taskPushBackKORASGripperCmd(drum_grp_grasping_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_grasping_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_grasping_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -13000, 1000);
        taskPushBack_delay_1ms(drum_grp_grasping_pose_task_, 2000);

        ///// 체결
        taskPushBackKORASGripperCmd(drum_grp_screwing_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        // taskPushBack_delay_1ms(drum_grp_screwing_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_screwing_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -25000, 1000);
        // taskPushBack_delay_1ms(drum_grp_screwing_pose_task_, 60);

        //// 체결 해제
        taskPushBackKORASGripperCmd(drum_grp_unscrewing_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_unscrewing_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_unscrewing_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +25000, 1000);
        taskPushBack_delay_1ms(drum_grp_unscrewing_pose_task_, 60);

        //// 나올 때
        taskPushBackKORASGripperCmd(drum_grp_exit_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_exit_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_exit_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 13000, 1000);
        taskPushBack_delay_1ms(drum_grp_exit_pose_task_, 2000);

        //// open jog
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 4000, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_, 500);

        //// close jog
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_open_, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -4000, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_open_, 500);
        break;

    default:
        break;
    }
    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////

    // CsDouble test_cs_pose1 = {0.09163, 0.40109, 0.70622, 81.751, 166.500, -80.510};      //cs 예시
    // taskPushBack_csMove(drum_total_task_, test_cs_pose1, false);

    std::string tcp_name = "tcp01";   // UI에서 미리 등록! 대강 (0, 40, 220)
    taskPushBack_drflSetTCP(drum_total_task_, tcp_name);
    taskPushBack_delay_1ms(drum_total_task_, 60);

    JsDouble home_pose = {-270, 0, 90, 0, 90, 270};
    JsDouble hole_scan_pose = {-288.520, -16.511, 103.542, 2.425, 79.684, 269.132};

    JsDouble middle_pose1 = {-154, 33.46, -99.21, 183.32, 112.38, -98.9};
    JsDouble middle_pose2 = {-320, 0, 0, 10, 90, 270};
    JsDouble middle_pose3 = {-337.774, -12.307, 48.955, 8.205, 108.241, 270.171};
    JsDouble middle_pose4 = {-336.697, 11.398, 28.614, 8.438, 101.810, 272.504};

    JsDouble hose_grapping_pose_top = {-330.541, 15.049, 42.831, -21.415, 82.498, 293.622}; //tool y +160mm
    JsDouble hole_pose_top = {-286.811, -16.089, 121.899, 1.298, -14.174, 270.000};
    JsDouble hole_pose_top_top = {-286.961, -16.010, 126.812, 0.548, -20.162, 268.280};

    JsDouble star_top = {-286.980, 14.514, 73.459, -0.026, 92.027, 356}; // cs z -75mm
    JsDouble star_off = {-350.855, -3.821, 64.940, 4.213, 120.139, 191.755};
    JsDouble star_bottom = {-286.986, 14.807, 81.297, -0.023, 83.896, 270.013};
    JsDouble star_off_off = {-348.492, -11.223, 70.754, 4.351, 121.568, 194.283};


                            //////////////////////////////////////////////
                            //////////////////////////////////////////////
                            //////////////////////////////////////////////
                            //////////////////////////////////////////////


    drum_total_task_.insert(drum_total_task_.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(drum_total_task_, 5000);

    taskPushBack_jsMove2(drum_total_task_, home_pose, 15, 30, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);
    taskPushBack_jsMove2(drum_total_task_, hole_scan_pose, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);



    //// Scanning
    // taskPushBack_taskRecog_3DScanning_sharedTask(drum_total_task_); // ZIVID Scanning with task recognition
    // taskPushBack_delay_1ms(drum_total_task_, 1000);
    // //// Matching
    // // taskPushBack_taskRecog_matching_sharedTask(drum_total_task_, false, 8); // ZIVID Scanning with task recognition
    // // taskPushBack_delay_1ms(drum_total_task_, 200);

    //별모양 뚜껑 따기
    if(0) {

    std::string tcp_name = "tcp01";   // UI에서 미리 등록! 대강 (0, 40, 220)
    taskPushBack_drflSetTCP(drum_total_task_, tcp_name);
    taskPushBack_delay_1ms(drum_total_task_, 60);


    taskPushBack_jsMove2(drum_total_task_, star_top, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);

    for (int cycle = 0; cycle < 3; cycle++) {
        taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, 0.080, 0, 0, 0}, 0.05, 0.1, true); // true: relative
        taskPushBack_delay_1ms(drum_total_task_, 50);
        taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, 0, 0, 0, -30}, 0.02, 0.04, true); // true: relative
        taskPushBack_delay_1ms(drum_total_task_, 60);
        taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, -0.080, 0, 0, 0}, 0.05, 0.1, true); // true: relative
        taskPushBack_delay_1ms(drum_total_task_, 60);
        taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, 0, 0, 0, 30}, 0.02, 0.04, true); // true: relative
        taskPushBack_delay_1ms(drum_total_task_, 60);
    }


    taskPushBack_jsMove2(drum_total_task_, hole_scan_pose, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);
    taskPushBack_jsMove2(drum_total_task_, middle_pose3, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);
    taskPushBack_jsMove2(drum_total_task_, star_off, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);

    taskPushBack_jsMove2(drum_total_task_, star_off_off, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);
    taskPushBack_jsMove2(drum_total_task_, hole_scan_pose, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);
    }


    //호스 파지
    if(1) {
    std::string tcp_name = "test2";   // UI에서 미리 등록! (0, 0, 0)
    taskPushBack_drflSetTCP(drum_total_task_, tcp_name);
    taskPushBack_delay_1ms(drum_total_task_, 60);


    taskPushBack_jsMove2(drum_total_task_, middle_pose3, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);

    taskPushBack_jsMove2(drum_total_task_, middle_pose4, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);

    // taskPushBack_taskRecog_3DScanning_sharedTask(drum_total_task_); // ZIVID Scanning with task recognition
    // taskPushBack_delay_1ms(drum_total_task_, 1000);

    taskPushBack_jsMove2(drum_total_task_, hose_grapping_pose_top, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);

    taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0.110, 0, 0, 0, 0}, 0.005, 0.01, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(drum_total_task_, 200);
    taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0.030, 0, 0, 0, 0}, 0.005, 0.01, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(drum_total_task_, 200);

    taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, 0.002, 0, 0, 0}, 0.005, 0.01, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(drum_total_task_, 200);

    drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
    taskPushBack_delay_1ms(drum_total_task_, 500);
    drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
    taskPushBack_delay_1ms(drum_total_task_, 500);


    taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, -0.210, 0, 0, 0}, 0.05, 0.1, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(drum_total_task_, 50);

    taskPushBack_jsMove2(drum_total_task_, hole_scan_pose, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);
    }

    // //// Grasping
    // // taskPushBack_doGrasping_sharedTask(drum_total_task_, 0.1, 0.05, "drum_task"); // Move to grasping pose, (acc, vel)

    // //// TODO: 추후 dq반영하도록 처리
    // // taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, -0.210, 0, 0, 0}, 0.05, 0.1, true); // false: absolute, true: relative
    // // taskPushBack_delay_1ms(drum_total_task_, 50);

    //호스 체결
    if(1){
    taskPushBack_jsMove2(drum_total_task_, hole_pose_top, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);
    // 총 200 mm down
    taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0.15, 0, 0, 0, 0}, 0.06, 0.1, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(drum_total_task_, 60);
    taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0.04, 0, 0, 0, 0}, 0.01, 0.02, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(drum_total_task_, 500);
    taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0.005, 0, 0, 0, 0}, 0.005, 0.01, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(drum_total_task_, 500);

    drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
    taskPushBack_delay_1ms(drum_total_task_, 4000);

    for (int cycle = 0; cycle < 3; cycle++) {
    drum_total_task_.insert(drum_total_task_.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    taskPushBack_delay_1ms(drum_total_task_, 100);
    }

    taskPushBack_delay_1ms(drum_total_task_, 4000);

    drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
    taskPushBack_delay_1ms(drum_total_task_, 2000);


        //// 케미컬 호스 체결 작업 (총 0.009 가량 down)
    auto subTaskScrewingChemicalHose = [&] (std::vector<UnitTask>& drum_total_task_, int trial_cnt) {

        drum_total_task_.insert(drum_total_task_.end(), drum_grp_screwing_pose_task_.begin(), drum_grp_screwing_pose_task_.end());

        if (trial_cnt % 3 == 0) {
            double z_scale_down = 0.009 / 40;         ///// 총 횟수%2

            taskPushBack_csMoveToolFrame(drum_total_task_, {0, z_scale_down, 0, 0, 0, 0}, 0.1, 0.05, true); // z축 이동
                //taskPushBack_delay_1ms(sub_task, 1000);
        }
        ROS_LOG_WARN("Chemical Hose Screwing - Trial: %d / 60", trial_cnt + 1);

    };

    for (int k = 0; k < 120; k++) {
        subTaskScrewingChemicalHose(drum_total_task_, k);
    }

    taskPushBack_delay_1ms(drum_total_task_, 10000);

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
    auto subTaskUnScrewingChemicalHose = [&] (std::vector<UnitTask>& drum_total_task_, int trial_cnt) {

        drum_total_task_.insert(drum_total_task_.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());

        if (trial_cnt % 2 == 0) {
            double z_scale_down = 0.009 / 30;         ///// 총 횟수%2

            taskPushBack_csMoveToolFrame(drum_total_task_, {0, -z_scale_down, 0, 0, 0, 0}, 0.1, 0.05, true); // z축 이동
                //taskPushBack_delay_1ms(sub_task, 1000);
        }

        ROS_LOG_WARN("Chemical Hose UnScrewing - Trial: %d / 60", trial_cnt + 1);

    };

    for (int k = 0; k < 50; k++) {
        subTaskUnScrewingChemicalHose(drum_total_task_, k);
    }

    taskPushBack_delay_1ms(drum_total_task_, 5000);


    drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
    taskPushBack_delay_1ms(drum_total_task_, 2000);

    taskPushBack_csMoveToolFrame(drum_total_task_, {0, -0.20 , 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(drum_total_task_, 50);

    taskPushBack_jsMove2(drum_total_task_, home_pose, 15, 30, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

    }

    //호스 체결 해제
    if(0) {
    taskPushBack_jsMove2(drum_total_task_, hole_pose_top_top, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);

    taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0.15 , 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(drum_total_task_, 50);
    taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0.022 , 0, 0, 0, 0}, 0.05, 0.1, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(drum_total_task_, 50);

    taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, 0.011, 0, 0, 0}, 0.01, 0.02, true); // false: absolute, true: relative

    auto subTaskUnScrewingChemicalHose = [&] (std::vector<UnitTask>& drum_total_task_, int trial_cnt) {

        drum_total_task_.insert(drum_total_task_.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());

        if (trial_cnt % 2 == 0) {
            double z_scale_down = 0.009 / 30;         ///// 총 횟수%2

            taskPushBack_csMoveToolFrame(drum_total_task_, {0, -z_scale_down, 0, 0, 0, 0}, 0.1, 0.05, true); // z축 이동
                //taskPushBack_delay_1ms(sub_task, 1000);
        }

        ROS_LOG_WARN("Chemical Hose UnScrewing - Trial: %d / 60", trial_cnt + 1);

    };

    for (int k = 0; k < 60; k++) {
        subTaskUnScrewingChemicalHose(drum_total_task_, k);
    }

    taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0.011 , 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(drum_total_task_, 50);

    drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
    taskPushBack_delay_1ms(drum_total_task_, 100);

    taskPushBack_csMoveToolFrame(drum_total_task_, {0, -0.15 , 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(drum_total_task_, 50);

    taskPushBack_jsMove2(drum_total_task_, home_pose, 15, 30, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);
    }

    // //호스 원위치
    if(0){
    JsDouble hose_back = {-313.262, -2.109, 57.689, -31.603, 91.007, 305.551};
    taskPushBack_jsMove2(drum_total_task_, hose_back, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);
    taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, 0.210, 0, 0, 0}, 0.03, 0.06, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(drum_total_task_, 50);

    drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
    taskPushBack_delay_1ms(drum_total_task_, 100);
    taskPushBack_csMoveToolFrame(drum_total_task_, {0, -0.160, 0, 0, 0, 0}, 0.05, 0.1, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(drum_total_task_, 50);
    taskPushBack_jsMove2(drum_total_task_, middle_pose3, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);
    }


    // CsDouble test_cs_pose1 = {-0.07764, -0.49377, 0.57596, 85.853, -66.065, 85.812};
    // taskPushBack_csMove(drum_total_task_, test_cs_pose1, false);
    // taskPushBack_delay_1ms(drum_total_task_, 60);


    // // 별 모양 뚜껑 원위치
    if(0) {
    JsDouble jog1_move = {20, 0, 0, 0, 0, 0};

    taskPushBack_jsMove2(drum_total_task_, middle_pose3, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);

    taskPushBack_jsMove2(drum_total_task_, star_off_off, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);

    taskPushBack_jsMove2(drum_total_task_, star_off, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);

    taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, 0.030, 0, 0, 0}, 0.02, 0.04, true); // true: relative
    taskPushBack_delay_1ms(drum_total_task_, 50);

    drum_total_task_.insert(drum_total_task_.end(), drum_grp_lid_cap_close_.begin(), drum_grp_lid_cap_close_.end());
    taskPushBack_delay_1ms(drum_total_task_, 5000);

    taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, -0.030, 0, 0, 0}, 0.02, 0.04, true); // true: relative
    taskPushBack_delay_1ms(drum_total_task_, 50);

    taskPushBack_jsMove2(drum_total_task_, star_off_off, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);

    taskPushBack_jsMove2(drum_total_task_, jog1_move, 10, 20, true);
    taskPushBack_delay_1ms(drum_total_task_, 60);

    taskPushBack_jsMove2(drum_total_task_, hole_scan_pose, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);

    taskPushBack_jsMove2(drum_total_task_, star_top, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);

    taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, 0, 0, 0, -30}, 0.02, 0.04, true); // true: relative
    taskPushBack_delay_1ms(drum_total_task_, 60);

    for (int cycle = 0; cycle < 3; cycle++) {
        taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, 0.080, 0, 0, 0}, 0.05, 0.1, true); // true: relative
        taskPushBack_delay_1ms(drum_total_task_, 50);
        taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, 0, 0, 0, 30}, 0.02, 0.04, true); // true: relative
        taskPushBack_delay_1ms(drum_total_task_, 60);
        taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, -0.080, 0, 0, 0}, 0.05, 0.1, true); // true: relative
        taskPushBack_delay_1ms(drum_total_task_, 60);
        taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, 0, 0, 0, -30}, 0.02, 0.04, true); // true: relative
        taskPushBack_delay_1ms(drum_total_task_, 60);
    }

    taskPushBack_jsMove2(drum_total_task_, hole_scan_pose, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);

    taskPushBack_jsMove2(drum_total_task_, hole_scan_pose, 10, 20, false);
    taskPushBack_delay_1ms(drum_total_task_, 60);

    }
}







//     //// UR10e, 241202
//     JsDouble drum_home_pose = {94.273, -68.145, -88.882, -112.889, 90.362, 3.922};
//     JsDouble ready_pose_grasping_hose = {98.193, -61.121, -119.278, -44.839, 83.722, 6.157};
//     JsDouble ready_pose_screwing_drum_hole = {-86.540, -68.144, -88.879, -112.888, 90.360, 3.926};

//     JsDouble grasping_scan_pose = {100.259, -71.777, -96.719, -98.787, 89.457, 13.185};   //호스 파지 스캐닝 자세
//     JsDouble ready_pose_grasping_hose_new = {97.925, -81.156, -122.759, -21.096, 85.347, 3.879};    //호스 파지 초기자세
//     JsDouble ready_pose_grasping_hose_new_middle = {-1.456, -68.146, -69.552, -132.738, 90.360, 3.927};         //호스 파지 과정에서, 경유지1
//     JsDouble ready_pose_grasping_hose_new_middle2 = {90.513, -42.797, -96.441, -130.675, 90.878, 4.372};        //호스 파지 과정에서, 경유지2

//     JsDouble unscrewing_scanning_pose = {-127.512, -44.991, -107.654, -82.569, 113.781, -29.264};   //체결해제를 위한


//     JsDouble ready_pose_ungrasping_first_cap = {67.892, -68.144, -88.878, -112.883, 90.363, 3.927};


//     //// TODO: 아래 자세는 측정 자세로, 나민우가 처리 예정
//     // JsDouble scanning_pose_grasping_hose = {103.298, -42.450, -135.403, -57.847, 82.598, 11.343};
//     // JsDouble scanning_pose_screwing_drum_hole = {-86.540, -68.144, -88.879, -112.888, 90.360, 3.926};

//     JsDouble approaching_pose_screwing_drum_hole_real = {-112.762, -63.835, -125.147, 8.196, 108.738, -0.867};
//     /// **************************************************************************** ///
//     /// **************************************************************************** ///
//     /// **************************************************************************** ///



//     /// **************************************************************************** ///
//     /// **************************************************************************** ///
//     /// **************************************************************************** ///
//     /////////////////////////////////////////////
//     //// Total Task
//     /////////////////////////////////////////////

//     drum_total_task_.clear();

//     //// TCP Setting
//     CsDouble tcp_drum_task_grasping_first_cap  = {0, 0.040, 0.28500, 0,  0, 90.0};
//     taskPushBack_setTCP (drum_total_task_, tcp_drum_task_grasping_first_cap);
//     taskPushBack_delay_1ms(drum_total_task_, 60);

//     ////////////////////////////////////////////////////////////
//     //// [작업 1] 뚜껑 열기
//     ////////////////////////////////////////////////////////////
//     if(1)
//     {
//         // taskPushBack_jsMove2(drum_total_task_, drum_home_pose, 45.0, 30.0, false);
//         // taskPushBack_delay_1ms(drum_total_task_, 600);

//         ////////////////////////////////////////////////////////////
//         /////////////// 케미컬 그리퍼 열기
//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_lid_cap_close_.begin(), drum_grp_lid_cap_close_.end());
//         ///////////////////////////////////////////

//         taskPushBack_jsMove2(drum_total_task_, ready_pose_screwing_drum_hole, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 600);


//         auto subTaskUnscrewingFirstCap = [&] (std::vector<UnitTask>& sub_task, int trial_cnt) {


//             double z_scale_up = static_cast<double>(trial_cnt);
//             // z_scale_up *= 0.0004;
//             z_scale_up *= 0.0012;
//             if(trial_cnt == 0) {
//                 // 접근 직전 자세
//                 taskPushBack_csMove2(sub_task, {-0.28195, -0.58280, 0.39271 + 0.04, -179.999, 1.110, 90.000}, 0.2, 0.1, false); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);

//                 // 정답 자세
//                 taskPushBack_csMove2(sub_task, {-0.28195, -0.58280, 0.39271 + z_scale_up, -179.999, 1.110, 90.000}, 0.1, 0.05, false); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//             } else {
//                 // -0.28195, -0.58280, 0.43529, 179.998, 1.111, -45.001
//                 double rz_rotation = 135.0;
//                 // 접근 직전 자세
//                 taskPushBack_csMove2(sub_task, {-0.28195, -0.58280, 0.39271 + 0.04, -179.999, 1.110, 90.000 - rz_rotation}, 0.2, 0.1, false); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);

//                 // 정답 자세
//                 taskPushBack_csMove2(sub_task, {-0.28195, -0.58280, 0.39271 + z_scale_up, -179.999, 1.110, 90.000 - rz_rotation}, 0.1, 0.05, false); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//             }

//             ////////////////////////////////////////////////////////////
//             /////////////// 케미컬 그리퍼 닫기
//             sub_task.insert(sub_task.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
//             ////////////////////////////////////////////////////////////
//             taskPushBack_delay_1ms(sub_task, 600);

//             //// 45 (47.5) + 180 + 180 + 135 = 총 540도 회전
//             if(trial_cnt == 0) {
//                 taskPushBack_csMove2(sub_task, {0, 0, 0, 0, 0, 52.5}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//             } else if(trial_cnt == 3) {
//                 taskPushBack_csMove2(sub_task, {0, 0, 0, 0, 0, 135.0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//             } else {
//                 taskPushBack_csMove2(sub_task, {0, 0, 0, 0, 0, 3.5}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);

//                 taskPushBack_csMove2(sub_task, {0, 0, 0, 0, 0, 180.0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//             }

//             ////////////////////////////////////////////////////////////
//             /////////////// 케미컬 그리퍼 열기

//             if(trial_cnt != 3) {
//                 sub_task.insert(sub_task.end(), drum_grp_lid_cap_close_.begin(), drum_grp_lid_cap_close_.end());
//                 taskPushBack_delay_1ms(sub_task, 1000);
//                 taskPushBack_csMove2(sub_task, {0, 0, 0.04, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);

//                 taskPushBack_csMove2(sub_task, {0, 0, 0, 0, 0, -135.0}, 0.2, 0.1, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);

//             } else {
//                 sub_task.insert(sub_task.end(), drum_grp_lid_cap_close_.begin(), drum_grp_lid_cap_close_.end());
//                 taskPushBack_delay_1ms(sub_task, 600);

//                 taskPushBack_csMove2(sub_task, {0, 0, 0, 0, 0, -1.0}, 0.2, 0.1, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);

//                 sub_task.insert(sub_task.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
//                 taskPushBack_delay_1ms(sub_task, 600);

//                 sub_task.insert(sub_task.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
//                 taskPushBack_delay_1ms(sub_task, 600);

//                 sub_task.insert(sub_task.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
//                 taskPushBack_delay_1ms(sub_task, 600);

//             }
//         };

//         //// Trial 4, 이 값을 바꾸는 경우, 나민우에게 연락
//         //// Trial 4, 이 값을 바꾸는 경우, 나민우에게 연락
//         //// Trial 4, 이 값을 바꾸는 경우, 나민우에게 연락
//         for(int k = 0; k < 4; k++) {
//             subTaskUnscrewingFirstCap(drum_total_task_, k);
//         }

//         // ////////////// 못 잡는 것 방지 --> 추후 삭제
//         // taskPushBack_csMove2(drum_total_task_, {0.0005, 0.0005, 0, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_total_task_, 50);
//         // taskPushBack_csMove2(drum_total_task_, {-0.0005, -0.0005, 0, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_total_task_, 50);
//         // taskPushBack_csMove2(drum_total_task_, {0.00025, 0.00025, 0, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_total_task_, 50);
//         // drum_total_task_.insert(drum_total_task_.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
//         // taskPushBack_delay_1ms(drum_total_task_, 1000);
//         // drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//         // taskPushBack_delay_1ms(drum_total_task_, 1000);
//         // drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         // taskPushBack_delay_1ms(drum_total_task_, 1000);
//         // taskPushBack_csMove2(drum_total_task_, {0, 0, 0.001, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_total_task_, 50);
//         // drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         // taskPushBack_delay_1ms(drum_total_task_, 1000);
//         // taskPushBack_csMove2(drum_total_task_, {0, 0, 0.001, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_total_task_, 50);
//         // drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         // taskPushBack_delay_1ms(drum_total_task_, 1000);
//         // taskPushBack_csMove2(drum_total_task_, {0, 0, 0.001, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_total_task_, 50);
//         // ////////////// 못 잡는 것 방지 --> 추후 삭제

//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 1000);
//         taskPushBack_csMove2(drum_total_task_, {0, 0, 0.04, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         taskPushBack_jsMove2(drum_total_task_, ready_pose_screwing_drum_hole, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 200);

//     }

//     //// [작업 2] 뚜껑 3차원 출력물 거치대 위에 놓기
//     if(1) {
//         taskPushBack_jsMove2(drum_total_task_, ready_pose_ungrasping_first_cap, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 200);



//         ////////////////////////////////////////////////////////////////////////////////
//         //// 티칭 자세 1: 뚜껑을 떨어뜨리는 허공 자세
//         std::vector<double> l_teaching_pose_1 = {0.18250, 0.38850, 0.05491, 179.989, 0.004, -180.000};
//         ////////////////////////////////////////////////////////////////////////////////

//         std::vector<double> l_teaching_approach_pose = l_teaching_pose_1;
//         l_teaching_approach_pose[2] += 0.14;
//         taskPushBack_csMove2(drum_total_task_, l_teaching_approach_pose, 0.2, 0.1, false); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 60);
//         taskPushBack_csMove2(drum_total_task_, l_teaching_pose_1, 0.2, 0.1, false); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 60);

//         /////////////// 케미컬 그리퍼 열기
//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_lid_cap_close_.begin(), drum_grp_lid_cap_close_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 100);


//         ////////////////////////////////////////////////////////////////////////////////
//         //// 티칭 자세 2: 뚜껑을 거치대에 잘 안착하도록 꾹 누르는 자세
//         std::vector<double> l_teaching_pose_2 = {0.18251, 0.38849, 0.02446, 179.993, 0.004, -180.000};
//         ////////////////////////////////////////////////////////////////////////////////

//         taskPushBack_csMove2(drum_total_task_, l_teaching_pose_2, 0.1, 0.05, false); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 600);

//         taskPushBack_csMoveToolFrame(drum_total_task_, {0.02, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         taskPushBack_csMove2(drum_total_task_, l_teaching_approach_pose, 0.2, 0.1, false); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 60);

//         taskPushBack_jsMove2(drum_total_task_, ready_pose_ungrasping_first_cap, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 200);

//     }


//     //// TCP Setting
//     CsDouble tcp_drum_task_grasping_hose  = {0, 0, 0.252, 0,  0, 90.0}; //0.028 (28mm)
//     taskPushBack_setTCP (drum_total_task_, tcp_drum_task_grasping_hose);
//     taskPushBack_delay_1ms(drum_total_task_, 60);

//     //// [작업 3] 케미컬 호스 파지 작업
//     if(1) {
//         ////////////////////////////////////////////////////////////
//         /////////////// 케미컬 그리퍼 초기화
//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
//         ////////////////////////////////////////////////////////////

//         taskPushBack_jsMove2(drum_total_task_, ready_pose_grasping_hose, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 600);


//         // //// Scanning - cylinder with hose
//         // taskPushBack_task3DScanning(drum_total_task_); // ZIVID Scanning with task recognition
//         // taskPushBack_delay_1ms(drum_total_task_, 3000);



//         /////////////////////////// TODO: 파지 자세
//         /////////////////////////// TODO: 파지 자세
//         /////////////////////////// TODO: 파지 자세
//         // # 케미컬 호스 파지 초기 자세
//         taskPushBack_csMove2(drum_total_task_, {0.07460, 0.79532, 0.39246, 179.991, 45.018, -90.005}, 0.3, 0.15, false); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         // # 케미컬 호스 파지 입장 바로 직전
//         taskPushBack_csMove2(drum_total_task_, {0.07462, 0.70069, 0.29772, 179.994, 45.008, -90.003}, 0.3, 0.15, false); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         // # 케미컬 호스 파지 직전 자세  (정답 직전, 기어 맞물리기 직전)
//         taskPushBack_csMove2(drum_total_task_, {0.07460, 0.65621, 0.25325, 179.993, 45.010, -90.003}, 0.1, 0.05, false); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         // # 케미컬 호스 파지 자세 (정답)
//         taskPushBack_csMove2(drum_total_task_, {0.07462, 0.65831, 0.25109, 179.996, 45.007, -90.003}, 0.1, 0.05, false); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         // 호스를 파지한 자세
//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 1000);

//         //////////////////////////////////////////////////////////
//         //////////////////////////////////////////////////////////

//         taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, -0.1, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         taskPushBack_csMove2(drum_total_task_, {0, 0, 0.15, 0, 0, 0}, 0.3, 0.15, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         taskPushBack_jsMove2(drum_total_task_, ready_pose_grasping_hose, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 200);
//     }

//     ///////////////////////////////////////////////////////////////////////////////////////////////////////
//     ///////////////////////////////////////////////////////////////////////////////////////////////////////
//     ///////////////////////////////////////////////////////////////////////////////////////////////////////
//     ////////////////////////////////////////////////////////////

//     ////////////////////////////////////////////////////////////

//     ///////////////////////////////////////////////////////////////////////////////////////////////////////
//     ///////////////////////////////////////////////////////////////////////////////////////////////////////
//     ///////////////////////////////////////////////////////////////////////////////////////////////////////



//     //// [작업 4-1] 주입구 스캔 및 인식
//     if(0) {

//         CsDouble tcp_drum_task_detection  = {0, 0.10000, 0.25000, 0,  0, 90.0};
//         taskPushBack_setTCP (drum_total_task_, tcp_drum_task_detection);
//         taskPushBack_delay_1ms(drum_total_task_, 60);

//         taskPushBack_jsMove2(drum_total_task_, ready_pose_screwing_drum_hole, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 200);

//         JsDouble scan_pose_detection_drum_hole = {-105.429, -65.126, -93.997, -100.813, 93.297, -14.272};
//         taskPushBack_jsMove2(drum_total_task_, scan_pose_detection_drum_hole, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 600);

//         //// Scanning - drum hole
//         // taskPushBack_task3DScanning(drum_total_task_); // ZIVID Scanning with task recognition
//         // taskPushBack_delay_1ms(drum_total_task_, 2000);

//         //// Scanning
//         taskPushBack_taskRecog_3DScanning_sharedTask(drum_total_task_); // ZIVID Scanning with task recognition
//         taskPushBack_delay_1ms(drum_total_task_, 200); // 10초
//         //// Matching
//         taskPushBack_taskRecog_matching_sharedTask(drum_total_task_, false, 8); // ZIVID Scanning with task recognition
//         taskPushBack_delay_1ms(drum_total_task_, 200);
//         //// Grasping
//         taskPushBack_doGrasping_sharedTask(drum_total_task_, 0.1, 0.05, "drum_task"); // Move to grasping pose, (acc, vel)

//         //// TODO: 추후 dq반영하도록 처리
//         // "theta": [-1.9120197260298286,0.3181669112305074,-0.36863361159210223,0.0033912181153434107,0.7058301826036172,-0.08962205198438197]
//         taskPushBack_jsMove2(drum_total_task_, {1.9120197260298286, -0.3181669112305074, 0.36863361159210223, -0.0033912181153434107, -0.7058301826036172, 0.08962205198438197}, 15.0, 10.0, true);
//         taskPushBack_delay_1ms(drum_total_task_, 200);


//         //// TODO: 오차를 줄이기 위한 캘리브레이션 방안 필요
//         // taskPushBack_csMoveToolFrame(drum_total_task_, {0, -0.005, 0.003, 0, 0, 0}, 0.02, 0.01, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_total_task_, 50);

//         //// Tool frame 기준 approach 자세
//         taskPushBack_csMoveToolFrame(drum_total_task_, {0.08, -0.005, 0.003, 0, 0, 0}, 0.08, 0.04, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         //// 최종 삽입
//         taskPushBack_csMove2(drum_total_task_, {0, 0, -0.0215, 0, 0, 0}, 0.02, 0.01, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);


//         ////////////////////////////////////////////////////////////////////////////////////////////
//         //// 케미컬 호스 체결 작업
//         auto subTaskScrewingChemicalHose = [&] (std::vector<UnitTask>& sub_task, int trial_cnt) {

//             sub_task.insert(sub_task.end(), drum_grp_screwing_pose_task_.begin(), drum_grp_screwing_pose_task_.end());
//             taskPushBack_delay_1ms(sub_task, 1000);

//             // 정답 자세
//             // double z_scale_down = static_cast<double>(trial_cnt);
//             // z_scale_down *= 0.0005; // 0.025mm --> 40x0.025mm = 1mm

//             double z_scale_down = 0.00025;
//             double x_scale = 0.00025;
//             double y_scale = 0.00025;

//             if(trial_cnt < 10) {
//                 // if(trial_cnt % 2 == 1) {


//                 // } else {

//                 // }
//                 taskPushBack_csMove2(sub_task, {0, 0, -z_scale_down, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//                 taskPushBack_csMove2(sub_task, {x_scale, y_scale, 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//                 taskPushBack_csMove2(sub_task, {-x_scale, -y_scale, 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//                 taskPushBack_csMove2(sub_task, {x_scale, y_scale, 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//                 taskPushBack_csMove2(sub_task, {-x_scale, -y_scale, 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//             }

//         };

//         for(int k = 0; k < 30; k++) {
//             subTaskScrewingChemicalHose(drum_total_task_, k);
//         }
//         ////////////////////////////////////////////////////////////////////////////////////////////

//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 1000);

//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 1000);

//         //// 기어가 빠지도록 툴 기준 3mm 후퇴
//         taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, -0.003, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 1000);

//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 1000);

//         // taskPushBack_csMove2(drum_total_task_, {0, 0, 0.005, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_total_task_, 50);

//         // drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//         // taskPushBack_delay_1ms(drum_total_task_, 1500);

//         // drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         // taskPushBack_delay_1ms(drum_total_task_, 1500);

//         taskPushBack_csMove2(drum_total_task_, {0, 0, 0.200, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         //// 작업 끝날 때까지 대기 (딜레이)
//         //// TODO: 로봇을 뺐다가 다시 넣는 작업 추가하도록(대표 지시사항)
//         taskPushBack_delay_1ms(drum_total_task_, 5000);



//     }


//     //// TCP Setting
//     taskPushBack_setTCP (drum_total_task_, tcp_drum_task_grasping_hose);
//     taskPushBack_delay_1ms(drum_total_task_, 60);

//     //// [작업 4] 케미컬 호스 체결 작업
//     if(1) {
//         taskPushBack_jsMove2(drum_total_task_, ready_pose_screwing_drum_hole, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 200);


//         taskPushBack_jsMove2(drum_total_task_, approaching_pose_screwing_drum_hole_real, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 600);

//         //////////////////////////////////////////////////////
//         //// 정답자세 (이 자세를 티칭, 진입 직전 +z 방향 25mm위)
//         //// TODO: 스캔 포즈로 대체
//         JsDouble screwing_pose_drum_hole_real = {-112.768, -80.532, -145.352, 44.986, 108.677, -1.231};
//         taskPushBack_jsMove2(drum_total_task_, screwing_pose_drum_hole_real, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 600);
//         //////////////////////////////////////////////////////

//         //// 최종 삽입
//         //// -0.28549, -0.58879, 0.46456, -162.341, 89.280, 104.850
//         taskPushBack_csMove2(drum_total_task_, {0, 0, -0.0285, 0, 0, 0}, 0.03, 0.015, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         ////////////////////////////////////////////////////////////////////////////////////////////
//         //// 케미컬 호스 체결 작업
//         auto subTaskScrewingChemicalHose = [&] (std::vector<UnitTask>& sub_task, int trial_cnt) {

//             sub_task.insert(sub_task.end(), drum_grp_screwing_pose_task_.begin(), drum_grp_screwing_pose_task_.end());
//             taskPushBack_delay_1ms(sub_task, 1000);

//             // 정답 자세
//             // double z_scale_down = static_cast<double>(trial_cnt);
//             // z_scale_down *= 0.0005; // 0.025mm --> 40x0.025mm = 1mm

//             double z_scale_down = 0.00025;
//             double x_scale = 0.00025;
//             double y_scale = 0.00025;

//             if(trial_cnt < 10) {
//                 // if(trial_cnt % 2 == 1) {


//                 // } else {

//                 // }
//                 taskPushBack_csMove2(sub_task, {0, 0, -z_scale_down, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//                 taskPushBack_csMove2(sub_task, {x_scale, y_scale, 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//                 taskPushBack_csMove2(sub_task, {-x_scale, -y_scale, 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//                 taskPushBack_csMove2(sub_task, {x_scale, y_scale, 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//                 taskPushBack_csMove2(sub_task, {-x_scale, -y_scale, 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//             }

//         };

//         for(int k = 0; k < 30; k++) {
//             subTaskScrewingChemicalHose(drum_total_task_, k);
//         }
//         ////////////////////////////////////////////////////////////////////////////////////////////

//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 1000);

//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 1000);

//         //// 기어가 빠지도록 툴 기준 3mm 후퇴
//         taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, -0.003, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 1000);

//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 1000);

//         // taskPushBack_csMove2(drum_total_task_, {0, 0, 0.005, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_total_task_, 50);

//         // drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//         // taskPushBack_delay_1ms(drum_total_task_, 1500);

//         // drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         // taskPushBack_delay_1ms(drum_total_task_, 1500);

//         taskPushBack_csMove2(drum_total_task_, {0, 0, 0.200, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         //// 작업 끝날 때까지 대기 (딜레이)
//         //// TODO: 로봇을 뺐다가 다시 넣는 작업 추가하도록(대표 지시사항)
//         taskPushBack_delay_1ms(drum_total_task_, 5000);


//     }

//     //// [작업 5] 케미컬 호스 해제 작업
//     if(1) {
//         //////////////////////////////////////////////////////
//         //// 정답자세 : 해체를 위한 재파지 자세에서 20cm 위의 JS 자세 (티칭 필요)
//         JsDouble unscrewing_pose_drum_hole_real = {-113.791, -67.204, -133.733, 20.115, 109.758, -0.953};
//         taskPushBack_jsMove2(drum_total_task_, unscrewing_pose_drum_hole_real, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 600);
//         //////////////////////////////////////////////////////

//         taskPushBack_csMove2(drum_total_task_, {0, 0, -0.2, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         //// 기어가 맞물리도록 툴 기준 3mm 전진
//         taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, 0.003, 0, 0, 0}, 0.03, 0.0125, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         ////////////////////////////////////////////////////////////////////////////////////////////
//         //// 케미컬 호스 해제 작업
//         auto subTaskUnscrewingChemicalHose = [&] (std::vector<UnitTask>& sub_task, int trial_cnt) {

//             sub_task.insert(sub_task.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
//             taskPushBack_delay_1ms(sub_task, 1000);

//             // double z_scale_up = static_cast<double>(trial_cnt);

//             double z_scale_up = 0.000266667; // 0.2666mm --> 30x0.2666mm = 8mm
//             taskPushBack_csMove2(sub_task, {0, 0, z_scale_up, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//             taskPushBack_delay_1ms(sub_task, 50);


//             // double z_scale_down = 0.00025;
//             // double x_scale = 0.00025;
//             // double y_scale = 0.00025;

//             // if(trial_cnt < 10) {
//             //     if(mod(trial_cnt,2) == 1) {
//             //         taskPushBack_csMove2(sub_task, {0, 0, -z_scale_down, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//             //         taskPushBack_delay_1ms(sub_task, 50);
//             //         taskPushBack_csMove2(sub_task, {x_scale, y_scale, 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//             //         taskPushBack_delay_1ms(sub_task, 50);
//             //     } else {
//             //         taskPushBack_csMove2(sub_task, {0, 0, z_scale_down, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//             //         taskPushBack_delay_1ms(sub_task, 50);
//             //         taskPushBack_csMove2(sub_task, {-x_scale, -y_scale, 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//             //         taskPushBack_delay_1ms(sub_task, 50);
//             //     }
//             // }

//         };

//         for(int k = 0; k < 30; k++) {
//             subTaskUnscrewingChemicalHose(drum_total_task_, k);
//         }


//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 1500);

//         //// 기어가 빠지도록 툴 기준 1.5mm 후퇴
//         taskPushBack_csMoveToolFrame(drum_total_task_, {0, 0, -0.0015, 0, 0, 0}, 0.03, 0.0125, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 1500);

//         taskPushBack_csMove2(drum_total_task_, {0, 0, 0.005, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 1500);

//         taskPushBack_csMove2(drum_total_task_, {0, 0, 0.2, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         taskPushBack_jsMove2(drum_total_task_, ready_pose_screwing_drum_hole, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 600);

//     }

//     //// [작업 6] 케미컬 호스를 다시 재자리에 가져다 놓는 작업
//     if(1) {
//         taskPushBack_jsMove2(drum_total_task_, ready_pose_grasping_hose, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 600);

//         // # 케미컬 호스 놓기 직전 자세
//         taskPushBack_csMove2(drum_total_task_, {0.08214, 0.60876, 0.31624, 179.990, 45.017, -90.006}, 0.2, 0.1, false); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         taskPushBack_csMove2(drum_total_task_, {0.07955, 0.65736, 0.25182, 179.993, 45.008, -90.004}, 0.1, 0.05, false); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 1000);

//         taskPushBack_csMoveToolFrame(drum_total_task_, {-0.1, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         taskPushBack_csMove2(drum_total_task_, {0, 0, 0.15, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);


//         taskPushBack_jsMove2(drum_total_task_, ready_pose_grasping_hose, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 600);
//     }


//     //// [작업 7] First cap을 다시 파지
//     if(1) {
//         //// TCP Setting
//         CsDouble tcp_drum_task_grasping_first_cap  = {0, 0.040, 0.28500, 0,  0, 90.0};
//         taskPushBack_setTCP (drum_total_task_, tcp_drum_task_grasping_first_cap);
//         taskPushBack_delay_1ms(drum_total_task_, 60);

//         taskPushBack_jsMove2(drum_total_task_, ready_pose_ungrasping_first_cap, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 600);

//         /////////////// 케미컬 그리퍼 열기
//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_lid_cap_close_.begin(), drum_grp_lid_cap_close_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 100);


//         ////////////////////////////////////////////////////////////////////////////////
//         //// 티칭 자세
//         std::vector<double> l_teaching_pose = {0.18253, 0.38846, 0.03330, 179.993, 0.006, -179.999};
//         ////////////////////////////////////////////////////////////////////////////////

//         std::vector<double> l_teaching_approach_pose = l_teaching_pose;
//         l_teaching_approach_pose[2] += 0.14;
//         taskPushBack_csMove2(drum_total_task_, l_teaching_approach_pose, 0.2, 0.1, false); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 60);
//         taskPushBack_csMove2(drum_total_task_, l_teaching_pose, 0.2, 0.1, false); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 60);

//         /////////////// 케미컬 그리퍼 닫기
//         drum_total_task_.insert(drum_total_task_.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
//         taskPushBack_delay_1ms(drum_total_task_, 1000);

//         // // // // ////////////// 못 잡는 것 방지 --> 추후 삭제
//         // // // // taskPushBack_csMove2(drum_total_task_, {0.0005, 0.0005, 0, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         // // // // taskPushBack_delay_1ms(drum_total_task_, 50);
//         // // // // taskPushBack_csMove2(drum_total_task_, {-0.0005, -0.0005, 0, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         // // // // taskPushBack_delay_1ms(drum_total_task_, 50);
//         // // // // taskPushBack_csMove2(drum_total_task_, {0.00025, 0.00025, 0, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         // // // // taskPushBack_delay_1ms(drum_total_task_, 50);
//         // // // // drum_total_task_.insert(drum_total_task_.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
//         // // // // taskPushBack_delay_1ms(drum_total_task_, 1000);
//         // // // // drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//         // // // // taskPushBack_delay_1ms(drum_total_task_, 1000);
//         // // // // drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         // // // // taskPushBack_delay_1ms(drum_total_task_, 1000);
//         // // // // taskPushBack_csMove2(drum_total_task_, {0, 0, 0.001, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         // // // // taskPushBack_delay_1ms(drum_total_task_, 50);
//         // // // // drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         // // // // taskPushBack_delay_1ms(drum_total_task_, 1000);
//         // // // // taskPushBack_csMove2(drum_total_task_, {0, 0, 0.001, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         // // // // taskPushBack_delay_1ms(drum_total_task_, 50);
//         // // // // drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         // // // // taskPushBack_delay_1ms(drum_total_task_, 1000);
//         // // // // taskPushBack_csMove2(drum_total_task_, {0, 0, 0.001, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         // // // // taskPushBack_delay_1ms(drum_total_task_, 50);
//         // // // // drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         // // // // taskPushBack_delay_1ms(drum_total_task_, 1000);
//         // // // // ////////////// 못 잡는 것 방지 --> 추후 삭제

//         taskPushBack_csMove2(drum_total_task_, l_teaching_approach_pose, 0.2, 0.1, false); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 60);

//         taskPushBack_jsMove2(drum_total_task_, ready_pose_ungrasping_first_cap, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 600);

//     }


//     //// First cap을 다시 체결
//     if(1) {
//         //// TCP Setting
//         CsDouble tcp_drum_task_grasping_first_cap  = {0, 0.040, 0.28500, 0,  0, 90.0};
//         taskPushBack_setTCP (drum_total_task_, tcp_drum_task_grasping_first_cap);
//         taskPushBack_delay_1ms(drum_total_task_, 60);

//         taskPushBack_jsMove2(drum_total_task_, ready_pose_screwing_drum_hole, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 600);



//         auto subTaskScrewingFirstCap = [&] (std::vector<UnitTask>& sub_task, int trial_cnt) {


//             //// 티칭 자세
//             std::vector<double> l_teaching_pose = {-0.28250, -0.58185, 0.39843, -179.995, 1.123, 89.998};
//             std::vector<double> l_teaching_approach_pose = l_teaching_pose;
//             l_teaching_approach_pose[2] += 0.04;

//             double z_scale_down = static_cast<double>(trial_cnt);
//             // z_scale_up *= 0.0004;
//             z_scale_down *= 0.0004;
//             if(trial_cnt == 0) {
//                 // 접근 직전 자세
//                 std::vector<double> tmp_approach_pose = l_teaching_approach_pose;
//                 taskPushBack_csMove2(sub_task, tmp_approach_pose, 0.2, 0.1, false); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);

//                 // 정답 자세
//                 std::vector<double> tmp_goal_pose = l_teaching_pose;
//                 tmp_goal_pose[2] -= z_scale_down;
//                 taskPushBack_csMove2(sub_task, tmp_goal_pose, 0.1, 0.05, false); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//             } else {
//                 // -0.28195, -0.58280, 0.43529, 179.998, 1.111, -45.001
//                 double rz_rotation = 45.0;
//                 // 접근 직전 자세
//                 std::vector<double> tmp_approach_pose = l_teaching_approach_pose;
//                 tmp_approach_pose[5] += rz_rotation;
//                 taskPushBack_csMove2(sub_task, tmp_approach_pose, 0.2, 0.1, false); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);

//                 // 정답 자세
//                 std::vector<double> tmp_goal_pose = l_teaching_pose;
//                 tmp_goal_pose[2] -= z_scale_down;
//                 tmp_goal_pose[5] += rz_rotation;
//                 taskPushBack_csMove2(sub_task, tmp_goal_pose, 0.1, 0.05, false); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//             }

//             if(trial_cnt == 3) { // 마지막 시도에는 손잡이에 걸리지 않도록 5도 더 회전
//                 taskPushBack_csMove2(sub_task, {0, 0, 0, 0, 0, 5.0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//             }

//             ////////////////////////////////////////////////////////////
//             /////////////// 케미컬 그리퍼 닫기
//             sub_task.insert(sub_task.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
//             ////////////////////////////////////////////////////////////
//             taskPushBack_delay_1ms(sub_task, 600);

//             //// -135 - 180 - 180 - 75 = 총 570도 회전
//             if(trial_cnt == 0) {
//                 taskPushBack_csMove2(sub_task, {0, 0, 0, 0, 0, -135.0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//             } else if(trial_cnt == 3) {
//                 taskPushBack_csMove2(sub_task, {0, 0, 0, 0, 0, -75.0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//             } else {
//                 taskPushBack_csMove2(sub_task, {0, 0, 0, 0, 0, -180.0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);
//             }

//             ////////////////////////////////////////////////////////////
//             /////////////// 케미컬 그리퍼 열기

//             if(trial_cnt != 3) {
//                 sub_task.insert(sub_task.end(), drum_grp_lid_cap_close_.begin(), drum_grp_lid_cap_close_.end());
//                 taskPushBack_delay_1ms(sub_task, 1000);
//                 taskPushBack_csMove2(sub_task, {0, 0, 0.04, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);

//                 taskPushBack_csMove2(sub_task, {0, 0, 0, 0, 0, 135.0}, 0.2, 0.1, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);

//             } else {
//                 sub_task.insert(sub_task.end(), drum_grp_lid_cap_close_.begin(), drum_grp_lid_cap_close_.end());
//                 taskPushBack_delay_1ms(sub_task, 1000);

//                 taskPushBack_csMove2(sub_task, {0, 0, 0, 0, 0, 5.0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 50);

//                 sub_task.insert(sub_task.end(), drum_grp_lid_cap_close_.begin(), drum_grp_lid_cap_close_.end());
//                 taskPushBack_delay_1ms(sub_task, 1000);
//             }
//         };

//         //// Trial 4, 이 값을 바꾸는 경우, 나민우에게 연락
//         //// Trial 4, 이 값을 바꾸는 경우, 나민우에게 연락
//         //// Trial 4, 이 값을 바꾸는 경우, 나민우에게 연락
//         for(int k = 0; k < 4; k++) {
//             subTaskScrewingFirstCap(drum_total_task_, k);
//         }

//         taskPushBack_csMove2(drum_total_task_, {0, 0, 0.04, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_total_task_, 50);

//         taskPushBack_jsMove2(drum_total_task_, ready_pose_screwing_drum_hole, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_total_task_, 600);

//     }









//     /////////////////////////////////////////////////////
//     //// 파지, Scan, 매칭, 체결까지만
//     //// drum_scan_sub_task_
//     //// drum_scan_sub_task_
//     //// drum_scan_sub_task_
//     //// drum_scan_sub_task_
//     //// drum_scan_sub_task_
//     //// drum_scan_sub_task_
//     //// drum_scan_sub_task_
//     //// drum_scan_sub_task_
//     //// drum_scan_sub_task_

//     drum_scan_sub_task_.clear();

//     /// 기존 detection_drum_hole -105.429, -65.126, -93.997, -100.813, 93.297, -14.272
//     /// 드럼통 15cm 올릴 때 -108.902, -66.161, -77.416, -113.674, 94.748, -17.468

//     CsDouble tcp_drum_task_grasping_hose_gripper2  = {0, 0, 0.252+0.019, 0,  0, 90.0}; //0.028 (28mm)
//     CsDouble tcp_drum_task_detection_gripper2  = {0, 0.1000, 0.25+0.019, 0,  0, 90.0}; //0.028 (28mm)
//     CsDouble tcp_drum_task_detection  = {0, 0.10000, 0.25200, 0,  0, 90.0};


//     // if(1) {
//     //     taskPushBack_jsMove2(drum_scan_sub_task_, ready_pose_screwing_drum_hole, 10.0, 20.0, false);
//     //     taskPushBack_delay_1ms(drum_scan_sub_task_, 200);

//     //     taskPushBack_csMove2(drum_scan_sub_task_, {0, 0, 0.15, 0, 0, 0}, 0.2, 0.35, true); // false: absolute, true: relative
//     //     taskPushBack_delay_1ms(drum_scan_sub_task_, 50);


//     // }
//     //// [작업 4-1] 주입구 스캔 및 인식
//     if(1) {
//         taskPushBack_setTCP (drum_scan_sub_task_, tcp_drum_task_detection);
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 60);

//         taskPushBack_jsMove2(drum_scan_sub_task_, ready_pose_screwing_drum_hole, 70.0, 45.0, false);
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 200);

//         JsDouble scan_pose_detection_drum_hole = {-105.429, -65.126, -93.997, -100.813, 93.297, -14.272};
//         taskPushBack_jsMove2(drum_scan_sub_task_, scan_pose_detection_drum_hole, 70.0, 45.0, false);
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 600);

//         //// Scanning - drum hole
//         // taskPushBack_task3DScanning(drum_scan_sub_task_); // ZIVID Scanning with task recognition
//         // taskPushBack_delay_1ms(drum_scan_sub_task_, 2000);

//         //// Scanning
//         taskPushBack_taskRecog_3DScanning_sharedTask(drum_scan_sub_task_); // ZIVID Scanning with task recognition
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 200); // 10초
//         //// Matching
//         taskPushBack_taskRecog_matching_sharedTask(drum_scan_sub_task_, false, 8); // ZIVID Scanning with task recognition
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 200);

//         taskPushBack_jsMove2(drum_scan_sub_task_, ready_pose_screwing_drum_hole, 70.0, 45.0, false);
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 200);

//     }



//     //// [작업 3] 케미컬 호스 파지 작업
//     if(0) {
//         taskPushBack_setTCP (drum_scan_sub_task_, tcp_drum_task_grasping_hose);
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 60);
//         ////////////////////////////////////////////////////////////
//         /////////////// 케미컬 그리퍼 초기화
//         drum_scan_sub_task_.insert(drum_scan_sub_task_.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
//         ////////////////////////////////////////////////////////////

//         taskPushBack_jsMove2(drum_scan_sub_task_, ready_pose_grasping_hose_new_middle, 90.0, 60.0, false);
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);
//         taskPushBack_jsMove2(drum_scan_sub_task_, ready_pose_grasping_hose_new_middle2, 90.0, 60.0, false);
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         taskPushBack_jsMove2(drum_scan_sub_task_, ready_pose_grasping_hose_new, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 200);


//         // //// Scanning - cylinder with hose
//         // taskPushBack_task3DScanning(drum_scan_sub_task_); // ZIVID Scanning with task recognition
//         // taskPushBack_delay_1ms(drum_scan_sub_task_, 3000);

//         /////////////////////////// TODO: 파지 자세
//         /////////////////////////// TODO: 파지 자세
//         /////////////////////////// TODO: 파지 자세

//         ///////일반 그리퍼를 사용할 땐 아래 두줄 주석처리하기
//         taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0, 0, -0.022, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         // # 케미컬 호스 파지 초기 자세 (살짝 뒤로 빠진 자세)
//         taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0.21, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         // # 완전 정답 자세 (직전 동작 대비 살짝 앞으로 전진)
//         taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0, 0, 0.005, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         // drum_scan_sub_task_.insert(drum_scan_sub_task_.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
//         // taskPushBack_delay_1ms(drum_scan_sub_task_, 500);

//         taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0, 0, 0.0013, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);
//         // 호스를 파지한 자세

//         drum_scan_sub_task_.insert(drum_scan_sub_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 1000);

//         //////////////////////////////////////////////////////////
//         //////////////////////////////////////////////////////////

//         taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0, 0, -0.15, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         taskPushBack_csMove2(drum_scan_sub_task_, {0, 0, 0.15, 0, 0, 0}, 0.7, 0.35, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         taskPushBack_jsMove2(drum_scan_sub_task_, ready_pose_grasping_hose_new_middle2, 90.0, 60.0, false);
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);
//         taskPushBack_jsMove2(drum_scan_sub_task_, ready_pose_grasping_hose_new_middle, 90.0, 60.0, false);
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 200);

//         // taskPushBack_jsMove2(drum_scan_sub_task_, ready_pose_grasping_hose, 45.0, 30.0, false);
//         // taskPushBack_delay_1ms(drum_scan_sub_task_, 200);
//     }

//     //// [작업 4-3] 자세 이동
//     if(0) {
//         CsDouble tcp_drum_task_detection  = {0, 0.1000, 0.25, 0,  0, 90.0}; //0.028
//         CsDouble tcp_drum_task_detection_gripper2  = {0, 0.1000, 0.25+0.026-0.002, 0,  0, 90.0}; //0.028

//         taskPushBack_setTCP (drum_scan_sub_task_, tcp_drum_task_detection);
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 60);

//         taskPushBack_jsMove2(drum_scan_sub_task_, ready_pose_screwing_drum_hole, 70.0, 45.0, false);
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         JsDouble scan_pose_detection_drum_hole = {-105.429, -65.126, -93.997, -100.813, 93.297, -14.272};
//         taskPushBack_jsMove2(drum_scan_sub_task_, scan_pose_detection_drum_hole, 70.0, 45.0, false);
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 200);

//         //// Grasping
//         taskPushBack_doGrasping_sharedTask(drum_scan_sub_task_, 0.1, 0.05, "drum_task"); // Move to grasping pose, (acc, vel)

//         //// TODO: 추후 dq반영하도록 처리
//         // "theta": [-1.9120197260298286,0.3181669112305074,-0.36863361159210223,0.0033912181153434107,0.7058301826036172,-0.08962205198438197]
//         taskPushBack_jsMove2(drum_scan_sub_task_, {1.9120197260298286, -0.3181669112305074, 0.36863361159210223, -0.0033912181153434107, -0.7058301826036172, 0.78962205198438197}, 15.0, 10.0, true);
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 200);



//         //// TODO: 오차를 줄이기 위한 캘리브레이션 방안 필요
//         // taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0, -0.005, 0.003, 0, 0, 0}, 0.02, 0.01, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         //// Tool frame 기준 approach 자세


//         // taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0.08, -0.0075, -0.004, 0, 0, 0}, 0.08, 0.04, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         // // 일반 그리퍼 사용시 아래 두 줄 주석 처리
//         // taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0, 0, -0.022, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative

//         taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0, -0.006, -0.021, 0, 0, 0}, 0.08, 0.04, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         taskPushBack_csMove2(drum_scan_sub_task_, {0, 0, -0.104, 0, 0, 0}, 0.02, 0.01, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);


//         taskPushBack_csMove2(drum_scan_sub_task_, {0, 0, -0.0010, 0, 0, 0}, 0.02, 0.01, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         // taskPushBack_csMove2(drum_scan_sub_task_, {0, 0, 0.0010, 0, 0, 0}, 0.02, 0.01, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_scan_sub_task_, 50);
//         taskPushBack_csMove2(drum_scan_sub_task_, {0, 0, 0.0005, 0, 0, 0}, 0.02, 0.01, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);
//         //////////////////////////////////////////////////////////////////////////////

//         //// 최종 삽입 Z축 방향으로 총 -0.0215 만큼 이동
//         // taskPushBack_csMove2(drum_scan_sub_task_, {0, 0, -0.0215, 0, 0, 0}, 0.02, 0.01, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_scan_sub_task_, 50);


//         // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//         taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0, 0, 0.0005, 0, 0, 0}, 0.006, 0.0025, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);
//         // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//         drum_scan_sub_task_.insert(drum_scan_sub_task_.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 1000);

//         taskPushBack_csMove2(drum_scan_sub_task_, {0, 0, -0.01, 0, 0, 0}, 0.01, 0.005, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         drum_scan_sub_task_.insert(drum_scan_sub_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 1000);

//         ////////////////////////////////////////////////////////////////////////////////////////////
//         //// 케미컬 호스 체결 작업
//         auto subTaskScrewingChemicalHose = [&] (std::vector<UnitTask>& sub_task, int trial_cnt) {

//             sub_task.insert(sub_task.end(), drum_grp_screwing_pose_task_.begin(), drum_grp_screwing_pose_task_.end());
//             taskPushBack_delay_1ms(sub_task, 1000);

//             /////////////////////////////////////////////////////////////////일반 그리퍼 사용시
//             // double z_scale_down = 0.0005;
//             // double x_scale = 0.0;
//             // double y_scale = 0.0;

//             // if(trial_cnt < 10) {
//             //     // if(trial_cnt % 2 == 1) {


//             //     // } else {

//             //     // }
//             //     taskPushBack_csMove2(sub_task, {0, 0, -z_scale_down, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//             //     taskPushBack_delay_1ms(sub_task, 250);
//             //}

//             /////////////////////////////////////////////////////////////////그리퍼2 사용시
//             double z_scale_down = 0.0005/5;

//             if(trial_cnt < 50) {

//                 taskPushBack_csMove2(sub_task, {0, 0, -z_scale_down, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//                 taskPushBack_delay_1ms(sub_task, 250);

//             }

//         };

//         for(int k = 0; k < 150; k++) {
//             subTaskScrewingChemicalHose(drum_scan_sub_task_, k);
//         }
//         ////////////////////////////////////////////////////////////////////////////////////////////


//         //// 체결 후 후속 작업
//         if(0) {
//             drum_scan_sub_task_.insert(drum_scan_sub_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//             taskPushBack_delay_1ms(drum_scan_sub_task_, 1000);

//             drum_scan_sub_task_.insert(drum_scan_sub_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//             taskPushBack_delay_1ms(drum_scan_sub_task_, 1000);


//             //// 기어가 빠지도록 툴 기준 3mm 후퇴
//             taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0, 0, -0.003, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//             taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//             drum_scan_sub_task_.insert(drum_scan_sub_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//             taskPushBack_delay_1ms(drum_scan_sub_task_, 1000);

//             drum_scan_sub_task_.insert(drum_scan_sub_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//             taskPushBack_delay_1ms(drum_scan_sub_task_, 1000);

//             // taskPushBack_csMove2(drum_scan_sub_task_, {0, 0, 0.005, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//             // taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//             // drum_scan_sub_task_.insert(drum_scan_sub_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//             // taskPushBack_delay_1ms(drum_scan_sub_task_, 1500);

//             // drum_scan_sub_task_.insert(drum_scan_sub_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//             // taskPushBack_delay_1ms(drum_scan_sub_task_, 1500);

//             taskPushBack_csMove2(drum_scan_sub_task_, {0, 0, 0.400, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//             taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//             //// 작업 끝날 때까지 대기 (딜레이)
//             //// TODO: 로봇을 뺐다가 다시 넣는 작업 추가하도록(대표 지시사항)
//             taskPushBack_delay_1ms(drum_scan_sub_task_, 50);
//         }
//     }

//     //// [작업 5] 케미컬 호스 해제 작업
//     if(0) {

//         drum_scan_sub_task_.insert(drum_scan_sub_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 2000);

//         drum_scan_sub_task_.insert(drum_scan_sub_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 1000);

//         taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0, 0, -0.004, 0, 0, 0}, 0.006, 0.0025, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         taskPushBack_csMove2(drum_scan_sub_task_, {0, 0, 0.3, 0, 0, 0}, 0.2, 0.4, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 4000);

//         taskPushBack_csMove2(drum_scan_sub_task_, {0, 0, -0.3, 0, 0, 0}, 0.2, 0.4, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 100);

//         taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0, 0, 0.004, 0, 0, 0}, 0.05, 0.01, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0, 0, 0.001, 0, 0, 0}, 0.03, 0.01, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         // taskPushBack_csMove2(drum_scan_sub_task_, {0, 0, 0.2, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_scan_sub_task_, 1000);


//         /////////////////////////////////////////////////////////////////////////////////
//         //여기서부터 원본
//         // taskPushBack_delay_1ms(drum_scan_sub_task_, 3000);

//         // taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0, 0, 0.002, 0, 0, 0}, 0.006, 0.0025, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         // taskPushBack_csMove2(drum_scan_sub_task_, {0, 0, -0.005, 0, 0, 0}, 0.01, 0.005, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_scan_sub_task_, 100);

//         drum_scan_sub_task_.insert(drum_scan_sub_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 2000);

//         //비틀어 돌리기
//         // taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0, 0, 0, -25, 0, 0}, 0.02, 0.008, true); // false: absolute, true: relative
//         // taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         ////////////////////////////////////////////////////////////////////////////////////////////
//         //// 케미컬 호스 해제 작업
//         auto subTaskUnscrewingChemicalHose = [&] (std::vector<UnitTask>& sub_task, int trial_cnt) {

//             sub_task.insert(sub_task.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
//             taskPushBack_delay_1ms(sub_task, 1000);

//             // double z_scale_up = static_cast<double>(trial_cnt);

//             double z_scale_up = 0.000266667; // 0.2666mm --> 30x0.2666mm = 8mm
//             taskPushBack_csMove2(sub_task, {0, 0, z_scale_up, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//             taskPushBack_delay_1ms(sub_task, 50);


//             // double z_scale_down = 0.00025;
//             // double x_scale = 0.00025;
//             // double y_scale = 0.00025;

//             // if(trial_cnt < 10) {
//             //     if(mod(trial_cnt,2) == 1) {
//             //         taskPushBack_csMove2(sub_task, {0, 0, -z_scale_down, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//             //         taskPushBack_delay_1ms(sub_task, 50);
//             //         taskPushBack_csMove2(sub_task, {x_scale, y_scale, 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//             //         taskPushBack_delay_1ms(sub_task, 50);
//             //     } else {
//             //         taskPushBack_csMove2(sub_task, {0, 0, z_scale_down, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//             //         taskPushBack_delay_1ms(sub_task, 50);
//             //         taskPushBack_csMove2(sub_task, {-x_scale, -y_scale, 0, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//             //         taskPushBack_delay_1ms(sub_task, 50);
//             //     }
//             // }

//         };

//         for(int k = 0; k < 30; k++) {
//             subTaskUnscrewingChemicalHose(drum_scan_sub_task_, k);
//         }


//         //// 기어가 빠지도록 툴 기준 1.5mm 후퇴
//         taskPushBack_csMoveToolFrame(drum_scan_sub_task_, {0, 0, -0.0015, 0, 0, 0}, 0.03, 0.0125, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         drum_scan_sub_task_.insert(drum_scan_sub_task_.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 1500);

//         taskPushBack_csMove2(drum_scan_sub_task_, {0, 0, -0.008, 0, 0, 0}, 0.1, 0.05, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         drum_scan_sub_task_.insert(drum_scan_sub_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 1500);

//         taskPushBack_csMove2(drum_scan_sub_task_, {0, 0, 0.2, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 50);

//         taskPushBack_jsMove2(drum_scan_sub_task_, ready_pose_screwing_drum_hole, 45.0, 30.0, false);
//         taskPushBack_delay_1ms(drum_scan_sub_task_, 600);

//     }


//     //// TCP Setting
//     // taskPushBack_setTCP (drum_scan_sub_task_, tcp_drum_task_grasping_hose);
//     // taskPushBack_delay_1ms(drum_scan_sub_task_, 60);

//     // taskPushBack_jsMove2(drum_scan_sub_task_, ready_pose_screwing_drum_hole, 45.0, 30.0, false);
//     // taskPushBack_delay_1ms(drum_scan_sub_task_, 600);


//     return;


//                         //     drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//                         //     taskPushBack_delay_1ms(drum_total_task_, 1000);

//                         //     drum_total_task_.insert(drum_total_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
//                         //     taskPushBack_delay_1ms(drum_total_task_, 1000);
//                         //     ////////////////////////////////////////////////////////////////////////////////////////////



//                         // //// 로봇 동작
//                         // taskPushBack_csMove2(drum_total_task_, {0, 0, 0.03, 0, 0, 0}, 0.07, 0.05, true); // false: absolute, true: relative
//                         // taskPushBack_delay_1ms(drum_total_task_, 50);

//                         // taskPushBack_csMove2(drum_total_task_, {0, 0, 0.17, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
//                         // taskPushBack_delay_1ms(drum_total_task_, 50);

//                         // taskPushBack_jsMove2(drum_total_task_, approaching_pose_screwing_drum_hole_real, 45.0, 30.0, false);
//                         // taskPushBack_delay_1ms(drum_total_task_, 600);

//                         // taskPushBack_jsMove2(drum_total_task_, ready_pose_screwing_drum_hole, 45.0, 30.0, false);
//                         // taskPushBack_delay_1ms(drum_total_task_, 600);

//                         // taskPushBack_jsMove2(drum_total_task_, ready_pose_grasping_hose, 45.0, 30.0, false);
//                         // taskPushBack_delay_1ms(drum_total_task_, 600);




//                         // //// TODO: 다시 놓기 작업 추가


//                         //             //// 놓기
//                         //             //// 접근 직전 자세
//                         //             taskPushBack_csMove2(drum_total_task_, {-0.84967, 0.37593, 0.34273, 175.633, 45.529, -5.957}, 0.3, 0.15, false); // false: absolute, true: relative
//                         //             taskPushBack_delay_1ms(drum_total_task_, 50);

//                         //             //// 파지 자세 (정답 직전, 기어 맞물리도록)
//                         //             taskPushBack_csMove2(drum_total_task_, {-0.80497, 0.37628, 0.29694, 175.633, 45.529, -5.957}, 0.1, 0.05, false); // false: absolute, true: relative
//                         //             taskPushBack_delay_1ms(drum_total_task_, 50);


//                         // ////////


//                         // //// 그리퍼 열기
//                         // if(1) {
//                         //     drum_total_task_.insert(drum_total_task_.end(), drum_grp_exit_pose_task_.begin(), drum_grp_exit_pose_task_.end());
//                         //     taskPushBack_delay_1ms(drum_total_task_, 1500);
//                         // }

//                         // //// TODO: 나오는 모션 추가




//                         // //// 초기 자세
//                         // taskPushBack_jsMove2(drum_total_task_, drum_home_pose, 45.0, 30.0, false);
//                         // taskPushBack_delay_1ms(drum_total_task_, 600);




//                         // // taskPushBack_csMove2(drum_total_task_, {0, 0, 0, 0, 0, +5}, 0.1, 0.03, true); // false: absolute, true: relative

//                         // // taskPushBack_csMove2(drum_total_task_, {0, 0, -0.0008, 0, 0, -27}, 0.1, 0.03, true); // false: absolute, true: relative
//                         // // taskPushBack_delay_1ms(drum_total_task_, 50);

//                         // // taskPushBack_csMove2(drum_total_task_, {0, 0, +0.0002, 0, 0, 0}, 0.1, 0.03, true); // false: absolute, true: relative
//                         // // taskPushBack_delay_1ms(drum_total_task_, 50);

//                         // //// 체결 접근 위치
//                         // // taskPushBack_csMove2(drum_total_task_, {0.45041, 0.35036, 0.54529, 179.405, 89.685, -128.631}, 0.2, 0.1, false); // false: absolute, true: relative
//                         // // taskPushBack_delay_1ms(drum_total_task_, 50);

//                         // // //// 체결 위치
//                         // // taskPushBack_csMove2(drum_total_task_, {0.45043, 0.35034, 0.48752, 179.766, 89.677, -128.273}, 0.06, 0.03, false); // false: absolute, true: relative
//                         // // taskPushBack_delay_1ms(drum_total_task_, 50);

//                         // // taskPushBack_csMove2(drum_total_task_, {0.45043, 0.35034, 0.4843, 99.31, 88.40, 152.52}, 0.06, 0.03, false); // false: absolute, true: relative
//                         // // taskPushBack_delay_1ms(drum_total_task_, 50);

//                         // // taskPushBack_csMove2(drum_total_task_, {0, 0, 0.0015, 0, 0, -2.0}, 0.01, 0.005, true); // false: absolute, true: relative
//                         // // taskPushBack_delay_1ms(drum_total_task_, 50);
//                         // // taskPushBack_csMove2(drum_total_task_, {0, 0, 0.0201, 0, 0, 1.0}, 0.01, 0.1, true); // false: absolute, true: relative
//                         // // taskPushBack_delay_1ms(drum_total_task_, 50);


//                         // //// 체결 작업
//                         // double insert_scale = 0.0015;
//                         // double insert_acc = 0.01;
//                         // double insert_vel = 0.0025;

//                         // ////////////////////////////////////////////////////////////
//                         // ////////////////////////////////////////////////////////////
//                         // //// 그리퍼 체결 구분동작

//                         // // taskPushBack_csMove2(drum_total_task_, {0, 0, 0, 0, 0,0}, 0.01, 0.005, true); // false: absolute, true: relative
//                         // // taskPushBack_delay_1ms(drum_total_task_, 50);

//                         // // taskPushBack_csMove2(drum_total_task_, {0, 0, -0.0002, 0, 0,0}, 0.01, 0.005, true); // false: absolute, true: relative
//                         // // taskPushBack_delay_1ms(drum_total_task_, 50);

//                         // // taskPushBackKORASGripperCmd(drum_grp_screwing_pose_task_, (uint16_t)KR_GRP::POS_RESET);
//                         // // taskPushBack_delay_1ms(drum_grp_screwing_pose_task_, 60);
//                         // // taskPushBackKORASGripperCmd(drum_grp_screwing_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 3000, 1000);
//                         // // taskPushBack_delay_1ms(drum_grp_screwing_pose_task_, 60);

//                         // // drum_total_task_.insert(drum_total_task_.end(), drum_grp_screwing_pose_task_.begin(), drum_grp_screwing_pose_task_.end());
//                         // // taskPushBack_delay_1ms(drum_total_task_, 1000);
//                         // // drum_total_task_.insert(drum_total_task_.end(), drum_grp_screwing_pose_task_.begin(), drum_grp_screwing_pose_task_.end());
//                         // // taskPushBack_delay_1ms(drum_total_task_, 1000);

//                         // taskPushBackKORASGripperCmd(drum_total_task_, (uint16_t)KR_GRP::POS_RESET);
//                         // taskPushBack_delay_1ms(drum_total_task_, 60);




// }