#include "task_planner_hanyang_eng.hpp"
#include <iostream>

TaskPlannerHanyangEng::TaskPlannerHanyangEng() {
    // JsDouble 멤버 변수들을 기본값으로 초기화
    amr_ready_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    right_home_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    right_hole_scan_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    middle_pose1_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    middle_pose2_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    middle_pose3_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    middle_pose4_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    hose_grapping_pose_top_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    hole_pose_top_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    hole_pose_top_top_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    star_top_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    star_off_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    star_bottom_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    star_off_off_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    left_home_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    left_hole_scan_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    drum_rotating_angle_detection_scan_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    makeTaskDefaultSetup();
    std::cout << "TaskPlannerHanyangEng initialized." << std::endl;
}

TaskPlannerHanyangEng::~TaskPlannerHanyangEng() {
    std::cout << "TaskPlannerHanyangEng destroyed." << std::endl;
}

bool TaskPlannerHanyangEng::makeTaskDefaultSetup() {
    std::cout << "Creating Hanyang task list..." << std::endl;
    // Hanyang 전용 작업 리스트 로직 구현
    // setGeneralSettingsForCookingTask();
    return true;
}

void TaskPlannerHanyangEng::HanyangEngCustomTask() {
    std::cout << "Executing Hanyang-specific task..." << std::endl;
    // 특정 작업 로직 구현
}

void TaskPlannerHanyangEng::generateHanyangEngTask() {
    genTaskSetting();
    genGripperTask();
    ROS_LOG_WARN("[%s] Generating Hanyang Eng. Task...", __func__);
    doosan_module_task_.clear();


    //// drum hole surface setting
    // std::string target_object = "right_drum_hole_surface";
    // std::string target_tcp = "tcp03";
    // std::string target_task = "drum_task";
    // JsDouble scan_pose_target = {32.432, -13.983, 108.903, 36.696, 66.329, -157.922};

    //// drum hole star setting
    // std::string target_object = "right_drum_lid_cap_unscrewing";
    // std::string target_tcp = "tcp05";
    // std::string target_task = "default";
    // JsDouble scan_pose_target = {32.432, -13.983, 108.903, 36.696, 66.329, -157.922};

    //// drum lid holder
    // std::string target_object = "right_holder_with_lid_cap";
    // std::string target_tcp = "tcp05";
    // std::string target_task = "default";
    // JsDouble scan_pose_target = {73.733, -4.032, 105.540, -56.375, 42.360, -15.019};

    //// drum hose holder
    std::string target_object = "right_holder_with_coupler";
    std::string target_tcp = "tcp06";
    std::string target_task = "drum_task";
    JsDouble scan_pose_target = {5.755, -8.142, 78.586, 32.294, 83.469, -206.805};

    doosan_module_task_.insert(doosan_module_task_.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(doosan_module_task_, 4000);

    taskPushBack_jsMove2(doosan_module_task_, scan_pose_target, 20.0, 10.0, false);
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    ////////////////////////////////////////////////////////////////////////////////////
    //// 파지 시에, 팁이 걸릴 경우 아래 값으로 조절: 현재: -3000
    taskPushBackKORASGripperCmd(drum_grp_minus_motor_ctrl_task_, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(drum_grp_minus_motor_ctrl_task_, 60);
    taskPushBackKORASGripperCmd(drum_grp_minus_motor_ctrl_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -3000);
    taskPushBack_delay_1ms(drum_grp_minus_motor_ctrl_task_, 500);
    ////////////////////////////////////////////////////////////////////////////////////

    //// Scanning
    taskPushBack_drflSetTCP(doosan_module_task_, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    taskPushBack_task3DScanningTargetObject(doosan_module_task_, target_object); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(doosan_module_task_, 600);
    //// Matching
    taskPushBack_taskMatchingTargetObject(doosan_module_task_, target_object, false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(doosan_module_task_, 4000);

    taskPushBack_drflSetTCP(doosan_module_task_, target_tcp); // new gripper
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    taskPushBack_doGrasping_sharedTask(doosan_module_task_, 0.1, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    //// 상대 모션, 파지 중에 걸릴 경우, 아래를 수정
    taskPushBack_csMoveToolFrame(doosan_module_task_, {0.10, 0, 0, 0, 0, 0}, 0.2, 0.1, true);
    taskPushBack_delay_1ms(doosan_module_task_, 60);
    taskPushBack_csMoveToolFrame(doosan_module_task_, {0.06, 0, 0, 0, 0, 0}, 0.04, 0.02, true);
    taskPushBack_delay_1ms(doosan_module_task_, 60);
    taskPushBack_csMoveToolFrame(doosan_module_task_, {0.02, 0, 0, 0, 0, 0}, 0.04, 0.02, true);
    taskPushBack_delay_1ms(doosan_module_task_, 60);
    taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0, -0.0045, 0, 0, 0}, 0.04, 0.02, true);
    taskPushBack_delay_1ms(doosan_module_task_, 60);
    taskPushBack_csMoveToolFrame(doosan_module_task_, {0.02, 0, 0, 0, 0, 0}, 0.01, 0.005, true);
    taskPushBack_delay_1ms(doosan_module_task_, 60);
    taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0, 0.0045, 0, 0, 0}, 0.04, 0.02, true);
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    doosan_module_task_.insert(doosan_module_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
    taskPushBack_delay_1ms(doosan_module_task_, 500);
    doosan_module_task_.insert(doosan_module_task_.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
    taskPushBack_delay_1ms(doosan_module_task_, 500);

    taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0, -0.210, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);


    taskPushBack_csMove2(doosan_module_task_, {0.250, 0, 0, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);


    //// TODO: 티칭 값 대체하도록
    //// taskPushBack_doGrasping_sharedTask을 변경


    return;

    taskPushBack_jsMove2(doosan_module_task_, right_home_pose_, 20.0, 10.0, false);
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    taskPushBack_drflSetTCP(doosan_module_task_, "tcp03");
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    //// drum hole, star scanning js position
    taskPushBack_jsMove2(doosan_module_task_, {32.432, -13.983, 108.903, 36.696, 66.329, -157.922}, 20.0, 10.0, false);
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    ////////////////////////////////////////////////////////////////////////////
    //// Doosan Robot Compliance CTRL ON/OFF
    taskPushBack_drflSetImpedance(doosan_module_task_, true); // Compliance CTRL ON
    taskPushBack_delay_1ms(doosan_module_task_, 1500);

    taskPushBack_delay_1ms(doosan_module_task_, 20000);

    taskPushBack_drflSetImpedance(doosan_module_task_, false); // Compliance CTRL OFF
    taskPushBack_delay_1ms(doosan_module_task_, 60);
    ////////////////////////////////////////////////////////////////////////////
    taskPushBack_jsMove2(doosan_module_task_, right_home_pose_, 20.0, 10.0, false);
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    return;

    taskPushBack_jsMove2(doosan_module_task_, right_home_pose_, 20.0, 10.0, false);
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    taskPushBack_jsMove2(doosan_module_task_, scan_pose_target, 20.0, 10.0, false);
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    taskPushBack_drflSetTCP(doosan_module_task_, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    // //// Scanning
    // taskPushBack_taskRecog_3DScanning_sharedTask(doosan_module_task_); // ZIVID Scanning with task recognition
    // taskPushBack_delay_1ms(doosan_module_task_, 2000);
    // //// Matching
    // taskPushBack_taskRecog_matching_sharedTask(doosan_module_task_, false, 8); // ZIVID Scanning with task recognition
    // taskPushBack_delay_1ms(doosan_module_task_, 4000);

    // taskPushBack_drflSetTCP(doosan_module_task_, "tcp01");
    // taskPushBack_delay_1ms(doosan_module_task_, 60);


    // taskPushBack_doGrasping_sharedTask(doosan_module_task_, 0.1, 0.05, "drum_task", target_object); // Move to grasping pose, (acc, vel)
    // taskPushBack_delay_1ms(doosan_module_task_, 60);

    // taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0, -0.210, 0, 0, 0}, 0.05, 0.1, true); // false: absolute, true: relative
    // taskPushBack_delay_1ms(doosan_module_task_, 50);

    //// Scanning
    taskPushBack_task3DScanningTargetObject(doosan_module_task_, "right_drum_hole_surface"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(doosan_module_task_, 2000);
    //// Matching
    taskPushBack_taskMatchingTargetObject(doosan_module_task_, "right_drum_hole_surface", false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(doosan_module_task_, 4000);

    taskPushBack_drflSetTCP(doosan_module_task_, "tcp03"); // new gripper
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    taskPushBack_doGrasping_sharedTask(doosan_module_task_, 0.1, 0.05, "drum_task", target_object); // Move to grasping pose, (acc, vel)
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    // taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0, -0.210, 0, 0, 0}, 0.05, 0.1, true); // false: absolute, true: relative
    // taskPushBack_delay_1ms(doosan_module_task_, 50);

    return;

    // doosan_module_task_.insert(doosan_module_task_.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    // taskPushBack_delay_1ms(doosan_module_task_, 200);

    taskPushBack_jsMove2(doosan_module_task_, right_home_pose_, 15, 30, false);
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    taskPushBack_csMoveToolFrame(doosan_module_task_, {0.05, 0, 0, 0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMoveToolFrame(doosan_module_task_, {-0.05, 0, 0, 0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);

    taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0.05, 0, 0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMoveToolFrame(doosan_module_task_, {0, -0.05, 0, 0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);

    taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0, 0.05, 0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0, -0.05, 0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);


    taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0, 0, 3.0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0, 0, -6.0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0, 0, 3.0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);


    taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0, 0, 0, 3.0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0, 0, 0, -6.0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0, 0, 0, 3.0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);

    taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0, 0, 0, 0, 3.0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0, 0, 0, 0, -6.0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMoveToolFrame(doosan_module_task_, {0, 0, 0, 0, 0, 3.0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);


    taskPushBack_jsMove2(doosan_module_task_, right_hole_scan_pose_, 15, 30, false);
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    taskPushBack_jsMove2(doosan_module_task_, right_home_pose_, 15, 30, false);
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    taskPushBack_csMove2(doosan_module_task_, {0.05, 0, 0, 0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMove2(doosan_module_task_, {-0.05, 0, 0, 0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);

    taskPushBack_csMove2(doosan_module_task_, {0, 0.05, 0, 0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMove2(doosan_module_task_, {0, -0.05, 0, 0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);

    taskPushBack_csMove2(doosan_module_task_, {0, 0, 0.05, 0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMove2(doosan_module_task_, {0, 0, -0.05, 0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);


    taskPushBack_csMove2(doosan_module_task_, {0, 0, 0, 3.0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMove2(doosan_module_task_, {0, 0, 0, -6.0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMove2(doosan_module_task_, {0, 0, 0, 3.0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);


    taskPushBack_csMove2(doosan_module_task_, {0, 0, 0, 0, 3.0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMove2(doosan_module_task_, {0, 0, 0, 0, -6.0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMove2(doosan_module_task_, {0, 0, 0, 0, 3.0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);

    taskPushBack_csMove2(doosan_module_task_, {0, 0, 0, 0, 0, 3.0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMove2(doosan_module_task_, {0, 0, 0, 0, 0, -6.0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);
    taskPushBack_csMove2(doosan_module_task_, {0, 0, 0, 0, 0, 3.0}, 0.2, 0.1, true); // true: relative
    taskPushBack_delay_1ms(doosan_module_task_, 50);

    taskPushBack_jsMove2(doosan_module_task_, right_home_pose_, 15, 30, false);
    taskPushBack_delay_1ms(doosan_module_task_, 60);

    ROS_LOG_WARN("[%s] Generating Hanyang Eng. Task Finished!", __func__);
}

void TaskPlannerHanyangEng::genTaskSetting() {
    try {
        JsDouble amr_ready_pose = {-0.000, -18.713, 105.394, -0.139, 93.361, -60.833};

        JsDouble right_home_pose = {-0.000, -18.713, 105.393, -0.139, 93.361, -90.833};
        JsDouble right_hole_scan_pose = {32.432, -13.983, 108.903, 36.696, 66.329, -157.922};

        JsDouble left_home_pose = {-0.143, 18.822, -105.523, 179.931, 93.345, -89.115};
        JsDouble left_hole_scan_pose = {-24.430, 11.354, -105.873, 141.652, 71.717, -15.381};


        amr_ready_pose_ = amr_ready_pose;

        right_home_pose_ = right_home_pose;
        right_hole_scan_pose_ = right_hole_scan_pose;

        left_home_pose_ = left_home_pose;
        left_hole_scan_pose_ = left_hole_scan_pose;


        // Scan JS Position
        JsDouble drum_rotating_angle_detection_scan_pose = {30.000, -18.713, 113.394, -0.139, 93.361, -90.833};
        drum_rotating_angle_detection_scan_pose_ = drum_rotating_angle_detection_scan_pose;
        
        std::cout << "genTaskSetting completed successfully" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Exception in genTaskSetting: " << e.what() << std::endl;
        throw;
    } catch (...) {
        std::cerr << "Unknown exception in genTaskSetting" << std::endl;
        throw;
    }
}

//// Only Scanning Task
void TaskPlannerHanyangEng::genScanTask() {
    ROS_LOG_WARN("[%s] Generating Hanyang Eng. Task...", __func__);
    genTaskSetting();
    genGripperTask();
    drum_scan_task_1_.clear();
    drum_scan_task_2_.clear();
    drum_scan_task_3_.clear();
    drum_scan_task_4_.clear();
    drum_scan_task_5_.clear();
    drum_scan_task_6_.clear();
    drum_scan_task_7_.clear();
    drum_scan_task_8_.clear();
    drum_scan_task_9_.clear();

    //// Scan Task 1: right_drum_lid_cap_unscrewing
    if(1) {
        // right_drum_lid_cap_unscrewing(drum hole star) setting
        std::string target_object = "right_drum_lid_cap_unscrewing";
        std::string target_tcp = "tcp05";
        std::string target_task = "default";
        JsDouble scan_pose_target = {30.001, -15.281, 104.708, 36.092, 72.196, -156.523};

        taskPushBack_jsMove2(drum_scan_task_1_, right_home_pose_, 20.0, 10.0, false);
        taskPushBack_delay_1ms(drum_scan_task_1_, 60);

        taskPushBack_jsMove2(drum_scan_task_1_, scan_pose_target, 20.0, 10.0, false);
        taskPushBack_delay_1ms(drum_scan_task_1_, 60);

        //// Scanning

        taskPushBack_drflSetTCP(drum_scan_task_1_, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(drum_scan_task_1_, 60);
        taskPushBack_task3DScanningTargetObject(drum_scan_task_1_, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(drum_scan_task_1_, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(drum_scan_task_1_, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(drum_scan_task_1_, 4000);

        taskPushBack_drflSetTCP(drum_scan_task_1_, target_tcp); // new gripper
        taskPushBack_delay_1ms(drum_scan_task_1_, 60);

        taskPushBack_doGrasping_sharedTask(drum_scan_task_1_, 0.1, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(drum_scan_task_1_, 60);
    }

    //// Scan Task 2: right_holder_without_lid_cap
    if(1) {
        // right_holder_without_lid_cap setting
        std::string target_object = "right_holder_without_lid_cap";
        std::string target_tcp = "tcp05";
        std::string target_task = "default";
        JsDouble scan_pose_target = {79.138, -16.448, 121.102, -66.679, 36.416, -4.983};
        taskPushBack_jsMove2(drum_scan_task_2_, scan_pose_target, 75.0, 75.0, false);
        taskPushBack_delay_1ms(drum_scan_task_2_, 60);

        //// Scanning
        taskPushBack_drflSetTCP(drum_scan_task_2_, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(drum_scan_task_2_, 60);
        taskPushBack_task3DScanningTargetObject(drum_scan_task_2_, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(drum_scan_task_2_, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(drum_scan_task_2_, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(drum_scan_task_2_, 4000);

        taskPushBack_drflSetTCP(drum_scan_task_2_, target_tcp); // new gripper
        taskPushBack_delay_1ms(drum_scan_task_2_, 60);

        if(1) {
            JsDouble waypoint_1 = {71.134, -7.479, 104.503, -28.592, 45.140, -136.637};
            taskPushBack_jsMove2(drum_scan_task_2_, waypoint_1, 75.0, 75.0, false);
            taskPushBack_delay_1ms(drum_scan_task_2_, 60);
            taskPushBack_csMove2(drum_scan_task_2_, {0.2, 0.2, 0.05, 0, 0, 0}, 0.05, 0.05, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(drum_scan_task_2_, 500);
        }

        taskPushBack_doGrasping_sharedTask(drum_scan_task_2_, 0.1, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(drum_scan_task_2_, 60);
    }

    //// Scan Task 3: right_holder_with_coupler
    if(1) {
        std::string target_object = "right_holder_with_coupler";
        std::string target_tcp = "tcp06";
        std::string target_task = "drum_task";
        JsDouble scan_pose_target = {5.755, -8.142, 78.586, 32.294, 83.469, -206.805};

        // drum_scan_task_3_.insert(drum_scan_task_3_.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
        taskPushBack_delay_1ms(drum_scan_task_3_, 4000);

        taskPushBack_jsMove2(drum_scan_task_3_, scan_pose_target, 20.0, 10.0, false);
        taskPushBack_delay_1ms(drum_scan_task_3_, 60);

        ////////////////////////////////////////////////////////////////////////////////////
        //// 파지 시에, 팁이 걸릴 경우 아래 값으로 조절: 현재: -3000
        // taskPushBackKORASGripperCmd(drum_scan_task_3_, (uint16_t)KR_GRP::POS_RESET);
        // taskPushBack_delay_1ms(drum_scan_task_3_, 60);
        // taskPushBackKORASGripperCmd(drum_scan_task_3_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -3000);
        // taskPushBack_delay_1ms(drum_scan_task_3_, 500);
        ////////////////////////////////////////////////////////////////////////////////////

        //// Scanning
        taskPushBack_drflSetTCP(drum_scan_task_3_, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(drum_scan_task_3_, 60);
        taskPushBack_task3DScanningTargetObject(drum_scan_task_3_, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(drum_scan_task_3_, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(drum_scan_task_3_, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(drum_scan_task_3_, 4000);

        taskPushBack_drflSetTCP(drum_scan_task_3_, target_tcp); // new gripper
        taskPushBack_delay_1ms(drum_scan_task_3_, 60);


        JsDouble waypoint_1 = {32.432, -13.983, 108.903, 36.696, 66.329, -157.922}; // drum hole surface
        taskPushBack_jsMove2(drum_scan_task_3_, waypoint_1, 20.0, 10.0, false);

        JsDouble waypoint_2 = {55.365, -14.830, 122.095, 219.794, 23.699, -308.583};
        taskPushBack_jsMove2(drum_scan_task_3_, waypoint_2, 20.0, 10.0, false);
        taskPushBack_delay_1ms(drum_scan_task_3_, 60);

        taskPushBack_doGrasping_sharedTask(drum_scan_task_3_, 0.1, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(drum_scan_task_3_, 60);
    }

    //// Scan Task 4: right_drum_hole_surface
    if(1) {
        std::string target_object = "right_drum_hole_surface";
        std::string target_tcp = "tcp03";
        std::string target_task = "drum_task";
        JsDouble scan_pose_target = {31.644, -11.474, 99.474, 33.409, 73.069, -153.435};

        taskPushBack_jsMove2(drum_scan_task_4_, scan_pose_target, 20.0, 10.0, false);
        taskPushBack_delay_1ms(drum_scan_task_4_, 60);

        //// Scanning
        taskPushBack_drflSetTCP(drum_scan_task_4_, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(drum_scan_task_4_, 60);
        taskPushBack_task3DScanningTargetObject(drum_scan_task_4_, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(drum_scan_task_4_, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(drum_scan_task_4_, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(drum_scan_task_4_, 4000);

        taskPushBack_drflSetTCP(drum_scan_task_4_, target_tcp); // new gripper
        taskPushBack_delay_1ms(drum_scan_task_4_, 60);

        taskPushBack_doGrasping_sharedTask(drum_scan_task_4_, 0.1, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(drum_scan_task_4_, 60);
    }


    //// Scan Task 5: right_drum_coupler_unscrewing
    if(1) {
        std::string target_object = "right_drum_coupler_unscrewing";
        std::string target_tcp = "tcp03";
        std::string target_task = "drum_task";
        JsDouble scan_pose_target = {21.418, -16.253, 91.451, 36.476, 90.033, -154.878};

        taskPushBack_jsMove2(drum_scan_task_5_, scan_pose_target, 20.0, 10.0, false);
        taskPushBack_delay_1ms(drum_scan_task_5_, 60);

        //// Scanning
        taskPushBack_drflSetTCP(drum_scan_task_5_, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(drum_scan_task_5_, 60);
        taskPushBack_task3DScanningTargetObject(drum_scan_task_5_, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(drum_scan_task_5_, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(drum_scan_task_5_, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(drum_scan_task_5_, 4000);

        taskPushBack_drflSetTCP(drum_scan_task_5_, target_tcp); // new gripper
        taskPushBack_delay_1ms(drum_scan_task_5_, 60);

        taskPushBack_doGrasping_sharedTask(drum_scan_task_5_, 0.1, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(drum_scan_task_5_, 60);
    }


    //// Scan Task 6: right_holder_without_coupler
    if(1) {
        std::string target_object = "right_holder_without_coupler";
        std::string target_tcp = "tcp06";
        std::string target_task = "default";
        JsDouble scan_pose_target = {16.021, -10.769, 78.136, 27.253, 81.357, -196.894};

        taskPushBack_jsMove2(drum_scan_task_6_, scan_pose_target, 20.0, 10.0, false);
        taskPushBack_delay_1ms(drum_scan_task_6_, 60);

        //// Scanning
        taskPushBack_drflSetTCP(drum_scan_task_6_, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(drum_scan_task_6_, 60);
        taskPushBack_task3DScanningTargetObject(drum_scan_task_6_, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(drum_scan_task_6_, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(drum_scan_task_6_, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(drum_scan_task_6_, 4000);

        taskPushBack_drflSetTCP(drum_scan_task_6_, target_tcp); // new gripper
        taskPushBack_delay_1ms(drum_scan_task_6_, 60);

        /// 싱귤러 방지
        JsDouble waypoint_1 = {57.079, -10.740, 136.493, 206.310, 40.185, -291.941}; // drum hole surface
        taskPushBack_jsMove2(drum_scan_task_6_, waypoint_1, 20.0, 10.0, false);

        taskPushBack_doGrasping_sharedTask(drum_scan_task_6_, 0.1, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(drum_scan_task_6_, 60);
    }


    //// Scan Task 7: right_holder_with_lid_cap
    if(1) {
        std::string target_object = "right_holder_with_lid_cap";
        std::string target_tcp = "tcp05";
        std::string target_task = "default";
        JsDouble scan_pose_target = {79.138, -16.448, 121.102, -66.679, 36.416, -4.983};

        taskPushBack_jsMove2(drum_scan_task_7_, scan_pose_target, 20.0, 10.0, false);
        taskPushBack_delay_1ms(drum_scan_task_7_, 60);

        // drum_scan_task_7_.insert(drum_scan_task_7_.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
        // taskPushBack_delay_1ms(drum_scan_task_7_, 3000);
        // drum_scan_task_7_.insert(drum_scan_task_7_.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(drum_scan_task_7_, 3000);

        //// Scanning
        taskPushBack_drflSetTCP(drum_scan_task_7_, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(drum_scan_task_7_, 60);
        taskPushBack_task3DScanningTargetObject(drum_scan_task_7_, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(drum_scan_task_7_, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(drum_scan_task_7_, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(drum_scan_task_7_, 4000);

        taskPushBack_drflSetTCP(drum_scan_task_7_, target_tcp); // new gripper
        taskPushBack_delay_1ms(drum_scan_task_7_, 60);

        if(1) {
            JsDouble waypoint_1 = {71.134, -7.479, 104.503, -28.592, 45.140, -136.637};
            taskPushBack_jsMove2(drum_scan_task_7_, waypoint_1, 75.0, 75.0, false);
            taskPushBack_delay_1ms(drum_scan_task_7_, 60);
            taskPushBack_csMove2(drum_scan_task_7_, {0.2, 0.2, 0.05, 0, 0, 0}, 0.05, 0.05, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(drum_scan_task_7_, 500);
        }

        taskPushBack_doGrasping_sharedTask(drum_scan_task_7_, 0.1, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(drum_scan_task_7_, 60);
    }


    //// Scan Task 8: right_drum_lid_cap_screwing
    if(1) {
        std::string target_object = "right_drum_lid_cap_screwing";
        std::string target_tcp = "tcp05";
        std::string target_task = "default";
        JsDouble scan_pose_target = {31.644, -11.474, 99.474, 33.409, 73.069, -153.435};

        taskPushBack_jsMove2(drum_scan_task_8_, scan_pose_target, 20.0, 10.0, false);
        taskPushBack_delay_1ms(drum_scan_task_8_, 60);

        //// Scanning
        taskPushBack_drflSetTCP(drum_scan_task_8_, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(drum_scan_task_8_, 60);
        taskPushBack_task3DScanningTargetObject(drum_scan_task_8_, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(drum_scan_task_8_, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(drum_scan_task_8_, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(drum_scan_task_8_, 4000);

        taskPushBack_drflSetTCP(drum_scan_task_8_, target_tcp); // new gripper
        taskPushBack_delay_1ms(drum_scan_task_8_, 60);

        taskPushBack_doGrasping_sharedTask(drum_scan_task_8_, 0.1, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(drum_scan_task_8_, 60);
    }

    //// Scan Task 9: drum_lid_total
    // if(0) {
    //     std::string target_object = "drum_lid_total";
    //     std::string target_tcp = "tcp05";
    //     std::string target_task = "default";
    //     JsDouble scan_pose_target = {50.071, -10.607, 69.057, 16.046, 101.500, -123.910};

    //     taskPushBack_jsMove2(drum_scan_task_9_, scan_pose_target, 20.0, 10.0, false);
    //     taskPushBack_delay_1ms(drum_scan_task_9_, 60);

    //     //// Scanning
    //     taskPushBack_drflSetTCP(drum_scan_task_9_, "tcp00"); // Scan & Matching은 tcp00
    //     taskPushBack_delay_1ms(drum_scan_task_9_, 60);
    //     taskPushBack_task3DScanningTargetObject(drum_scan_task_9_, target_object); // ZIVID Scanning with task recognition
    //     taskPushBack_delay_1ms(drum_scan_task_9_, 600);
    //     //// Matching
    //     taskPushBack_taskMatchingTargetObject(drum_scan_task_9_, target_object, false, 8); // ZIVID Scanning with task recognition
    //     taskPushBack_delay_1ms(drum_scan_task_9_, 4000);
    // }
}


void TaskPlannerHanyangEng::genLeftDrumPLCCommTask(bool is_plc_test_mode) {
    // //////////////////////////////////////////////////////////////
    // // [Task 1] 캡 풀기 작업 (left_drum_cap_detach_plc_comm_task_)
    // // BIT 8: 캡 풀기 작업 시작, BIT 9: 캡 풀기 작업 완료
    // left_drum_cap_detach_plc_comm_task_.clear();
    // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_cap_detach_plc_comm_task_, 0x41000, true, 8, true); // WORD Address, is_lsb, BIT, true/false
    // taskPushBack_delay_1ms(left_drum_cap_detach_plc_comm_task_, 600);
    // if(is_plc_test_mode) {
    //     taskPushBack_sendTaskLog(left_drum_cap_detach_plc_comm_task_, "캡 분리 작업 (PLC Comm.)");
    // } else {
    //     taskPushBack_jsMove2(left_drum_cap_detach_plc_comm_task_, left_home_pose_, 75.0, 75.0, false);
    //     taskPushBack_delay_1ms(left_drum_cap_detach_plc_comm_task_, 60);
    //     taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_cap_detach_plc_comm_task_, 0x41000, true, 4, true); // WORD Address, is_lsb, BIT, true/false
    //     taskPushBack_delay_1ms(left_drum_cap_detach_plc_comm_task_, 1000);
    //     std::vector<UnitTask> left_drum_sub_task_1 = leftGenSubTask1Task();
    //     left_drum_cap_detach_plc_comm_task_.insert(left_drum_cap_detach_plc_comm_task_.end(), left_drum_sub_task_1.begin(), left_drum_sub_task_1.end());

    //     taskPushBack_jsMove2(left_drum_cap_detach_plc_comm_task_, left_home_pose_, 75.0, 75.0, false);
    //     taskPushBack_delay_1ms(left_drum_cap_detach_plc_comm_task_, 60);
    // }
    // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_cap_detach_plc_comm_task_, 0x41000, true, 9, true); // WORD Address, is_lsb, BIT, true/false
    // taskPushBack_delay_1ms(left_drum_cap_detach_plc_comm_task_, 600);
    // //////////////////////////////////////////////////////////////


    // //////////////////////////////////////////////////////////////
    // // [Task 2] 커플러 체결 작업 (left_drum_coupler_screwing_plc_comm_task_)
    // // 구멍 인식 & 호스 파지 & 구멍 삽입 자세 직전 & 호스 체결
    // // BIT 10: 커플러 체결 작업 시작, BIT 11: 커플러 체결 작업 완료
    // left_drum_coupler_screwing_plc_comm_task_.clear();
    // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_coupler_screwing_plc_comm_task_, 0x41000, true, 10, true); // WORD Address, is_lsb, BIT, true/false
    // taskPushBack_delay_1ms(left_drum_coupler_screwing_plc_comm_task_, 600);
    // if(is_plc_test_mode) {
    //     taskPushBack_sendTaskLog(left_drum_coupler_screwing_plc_comm_task_, "커플러 체결 작업 (PLC Comm.)");
    // } else {
    //     taskPushBack_jsMove2(left_drum_coupler_screwing_plc_comm_task_, left_home_pose_, 75.0, 75.0, false);
    //     taskPushBack_delay_1ms(left_drum_coupler_screwing_plc_comm_task_, 60);

    //     std::vector<UnitTask> left_drum_sub_task_2 = leftGenSubTask2Task();
    //     std::vector<UnitTask> left_drum_sub_task_3 = leftGenSubTask3Task();
    //     left_drum_coupler_screwing_plc_comm_task_.insert(left_drum_coupler_screwing_plc_comm_task_.end(), left_drum_sub_task_2.begin(), left_drum_sub_task_2.end());
    //     left_drum_coupler_screwing_plc_comm_task_.insert(left_drum_coupler_screwing_plc_comm_task_.end(), left_drum_sub_task_3.begin(), left_drum_sub_task_3.end());

    //     taskPushBack_jsMove2(left_drum_coupler_screwing_plc_comm_task_, left_home_pose_, 75.0, 75.0, false);
    //     taskPushBack_delay_1ms(left_drum_coupler_screwing_plc_comm_task_, 60);

    // }
    // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_coupler_screwing_plc_comm_task_, 0x41000, true, 11, true); // WORD Address, is_lsb, BIT, true/false
    // taskPushBack_delay_1ms(left_drum_coupler_screwing_plc_comm_task_, 600);
    // //////////////////////////////////////////////////////////////

    // //////////////////////////////////////////////////////////////
    // // [Task 3] 커플러 풀기 (left_drum_coupler_detach_plc_comm_task_)
    // // 커플러 체결 해제 & 호스 원위치
    // // BIT 4: 커플러 풀기 작업 시작, BIT 5: 커플러 풀기 작업 완료
    // left_drum_coupler_detach_plc_comm_task_.clear();
    // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_coupler_detach_plc_comm_task_, 0x41000, true, 4, true); // WORD Address, is_lsb, BIT, true/false
    // taskPushBack_delay_1ms(left_drum_coupler_detach_plc_comm_task_, 600);
    // if(is_plc_test_mode) {
    //     taskPushBack_sendTaskLog(left_drum_coupler_detach_plc_comm_task_, "커플러 체결 해제 (PLC Comm.)");
    // } else {
    //     taskPushBack_jsMove2(left_drum_coupler_detach_plc_comm_task_, left_home_pose_, 75.0, 75.0, false);
    //     taskPushBack_delay_1ms(left_drum_coupler_detach_plc_comm_task_, 60);

    //     std::vector<UnitTask> left_drum_sub_task_4 = leftGenSubTask4Task();
    //     std::vector<UnitTask> left_drum_sub_task_5 = leftGenSubTask5Task();
    //     left_drum_coupler_detach_plc_comm_task_.insert(left_drum_coupler_detach_plc_comm_task_.end(), left_drum_sub_task_4.begin(), left_drum_sub_task_4.end());
    //     left_drum_coupler_detach_plc_comm_task_.insert(left_drum_coupler_detach_plc_comm_task_.end(), left_drum_sub_task_5.begin(), left_drum_sub_task_5.end());
    //     taskPushBack_jsMove2(left_drum_coupler_detach_plc_comm_task_, left_home_pose_, 75.0, 75.0, false);
    //     taskPushBack_delay_1ms(left_drum_coupler_detach_plc_comm_task_, 60);
    // }
    // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_coupler_detach_plc_comm_task_, 0x41000, true, 5, true); // WORD Address, is_lsb, BIT, true/false
    // taskPushBack_delay_1ms(left_drum_coupler_detach_plc_comm_task_, 600);
    // //////////////////////////////////////////////////////////////
    
    // //////////////////////////////////////////////////////////////
    // // [Task 4] 캡 체결 (left_drum_coupler_detach_plc_comm_task_)
    // // 커플러 체결 해제 & 호스 원위치
    // // BIT 6: 캡 체결 작업 시작, BIT 7: 캡 체결 작업 완료
    // left_drum_cap_screwing_plc_comm_task_.clear();
    // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_cap_screwing_plc_comm_task_, 0x41000, true, 6, true); // WORD Address, is_lsb, BIT, true/false
    // taskPushBack_delay_1ms(left_drum_cap_screwing_plc_comm_task_, 600);
    // if(is_plc_test_mode) {
    //     taskPushBack_sendTaskLog(left_drum_cap_screwing_plc_comm_task_, "캡 체결 (PLC Comm.)");
    // } else {
    //     taskPushBack_jsMove2(left_drum_cap_screwing_plc_comm_task_, left_home_pose_, 75.0, 75.0, false);
    //     taskPushBack_delay_1ms(left_drum_cap_screwing_plc_comm_task_, 60);


    //     std::vector<UnitTask> left_drum_sub_task_6 = leftGenSubTask6Task();
    //     left_drum_cap_screwing_plc_comm_task_.insert(left_drum_cap_screwing_plc_comm_task_.end(), left_drum_sub_task_6.begin(), left_drum_sub_task_6.end());
    //     taskPushBack_jsMove2(left_drum_cap_screwing_plc_comm_task_, left_home_pose_, 75.0, 75.0, false);
    //     taskPushBack_delay_1ms(left_drum_cap_screwing_plc_comm_task_, 60);
    // }
    // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_cap_screwing_plc_comm_task_, 0x41000, true, 7, true); // WORD Address, is_lsb, BIT, true/false
    // taskPushBack_delay_1ms(left_drum_cap_screwing_plc_comm_task_, 600);
    // //////////////////////////////////////////////////////////////
}


void TaskPlannerHanyangEng::genRightDrumPLCCommTask(bool is_plc_test_mode) {
    // //////////////////////////////////////////////////////////////
    // // [Task 1] 캡 풀기 작업 (right_drum_cap_detach_plc_comm_task_)
    // // BIT 8: 캡 풀기 작업 시작, BIT 9: 캡 풀기 작업 완료
    // right_drum_cap_detach_plc_comm_task_.clear();
    // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_detach_plc_comm_task_, 0x41000, true, 8, true); // WORD Address, is_lsb, BIT, true/false
    // taskPushBack_delay_1ms(right_drum_cap_detach_plc_comm_task_, 600);
    // if(is_plc_test_mode) {
    //     taskPushBack_sendTaskLog(right_drum_cap_detach_plc_comm_task_, "캡 분리 작업 (PLC Comm.)");
    // } else {
    //     taskPushBack_jsMove2(right_drum_cap_detach_plc_comm_task_, right_home_pose_, 75.0, 75.0, false);
    //     taskPushBack_delay_1ms(right_drum_cap_detach_plc_comm_task_, 60);
    //     taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_detach_plc_comm_task_, 0x41000, true, 4, true); // WORD Address, is_lsb, BIT, true/false
    //     taskPushBack_delay_1ms(right_drum_cap_detach_plc_comm_task_, 1000);
    //     std::vector<UnitTask> right_drum_sub_task_1 = rightGenSubTask1Task();
    //     right_drum_cap_detach_plc_comm_task_.insert(right_drum_cap_detach_plc_comm_task_.end(), right_drum_sub_task_1.begin(), right_drum_sub_task_1.end());

    //     taskPushBack_jsMove2(right_drum_cap_detach_plc_comm_task_, right_home_pose_, 75.0, 75.0, false);
    //     taskPushBack_delay_1ms(right_drum_cap_detach_plc_comm_task_, 60);
    // }
    // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_detach_plc_comm_task_, 0x41000, true, 9, true); // WORD Address, is_lsb, BIT, true/false
    // taskPushBack_delay_1ms(right_drum_cap_detach_plc_comm_task_, 600);
    // //////////////////////////////////////////////////////////////


    // //////////////////////////////////////////////////////////////
    // // [Task 2] 커플러 체결 작업 (right_drum_coupler_screwing_plc_comm_task_)
    // // 구멍 인식 & 호스 파지 & 구멍 삽입 자세 직전 & 호스 체결
    // // BIT 10: 커플러 체결 작업 시작, BIT 11: 커플러 체결 작업 완료
    // right_drum_coupler_screwing_plc_comm_task_.clear();
    // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_coupler_screwing_plc_comm_task_, 0x41000, true, 10, true); // WORD Address, is_lsb, BIT, true/false
    // taskPushBack_delay_1ms(right_drum_coupler_screwing_plc_comm_task_, 600);
    // if(is_plc_test_mode) {
    //     taskPushBack_sendTaskLog(right_drum_coupler_screwing_plc_comm_task_, "커플러 체결 작업 (PLC Comm.)");
    // } else {
    //     taskPushBack_jsMove2(right_drum_coupler_screwing_plc_comm_task_, right_home_pose_, 75.0, 75.0, false);
    //     taskPushBack_delay_1ms(right_drum_coupler_screwing_plc_comm_task_, 60);

    //     std::vector<UnitTask> right_drum_sub_task_2 = rightGenSubTask2Task();
    //     std::vector<UnitTask> right_drum_sub_task_3 = rightGenSubTask3Task();
    //     right_drum_coupler_screwing_plc_comm_task_.insert(right_drum_coupler_screwing_plc_comm_task_.end(), right_drum_sub_task_2.begin(), right_drum_sub_task_2.end());
    //     right_drum_coupler_screwing_plc_comm_task_.insert(right_drum_coupler_screwing_plc_comm_task_.end(), right_drum_sub_task_3.begin(), right_drum_sub_task_3.end());

    //     taskPushBack_jsMove2(right_drum_coupler_screwing_plc_comm_task_, right_home_pose_, 75.0, 75.0, false);
    //     taskPushBack_delay_1ms(right_drum_coupler_screwing_plc_comm_task_, 60);

    // }
    // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_coupler_screwing_plc_comm_task_, 0x41000, true, 11, true); // WORD Address, is_lsb, BIT, true/false
    // taskPushBack_delay_1ms(right_drum_coupler_screwing_plc_comm_task_, 600);
    // //////////////////////////////////////////////////////////////

    // //////////////////////////////////////////////////////////////
    // // [Task 3] 커플러 풀기 (right_drum_coupler_detach_plc_comm_task_)
    // // 커플러 체결 해제 & 호스 원위치
    // // BIT 4: 커플러 풀기 작업 시작, BIT 5: 커플러 풀기 작업 완료
    // right_drum_coupler_detach_plc_comm_task_.clear();
    // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_coupler_detach_plc_comm_task_, 0x41000, true, 4, true); // WORD Address, is_lsb, BIT, true/false
    // taskPushBack_delay_1ms(right_drum_coupler_detach_plc_comm_task_, 600);
    // if(is_plc_test_mode) {
    //     taskPushBack_sendTaskLog(right_drum_coupler_detach_plc_comm_task_, "커플러 체결 해제 (PLC Comm.)");
    // } else {
    //     taskPushBack_jsMove2(right_drum_coupler_detach_plc_comm_task_, right_home_pose_, 75.0, 75.0, false);
    //     taskPushBack_delay_1ms(right_drum_coupler_detach_plc_comm_task_, 60);

    //     std::vector<UnitTask> right_drum_sub_task_4 = rightGenSubTask4Task();
    //     std::vector<UnitTask> right_drum_sub_task_5 = rightGrenSubTask5Task();
    //     right_drum_coupler_detach_plc_comm_task_.insert(right_drum_coupler_detach_plc_comm_task_.end(), right_drum_sub_task_4.begin(), right_drum_sub_task_4.end());
    //     right_drum_coupler_detach_plc_comm_task_.insert(right_drum_coupler_detach_plc_comm_task_.end(), right_drum_sub_task_5.begin(), right_drum_sub_task_5.end());
    //     taskPushBack_jsMove2(right_drum_coupler_detach_plc_comm_task_, right_home_pose_, 75.0, 75.0, false);
    //     taskPushBack_delay_1ms(right_drum_coupler_detach_plc_comm_task_, 60);
    // }
    // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_coupler_detach_plc_comm_task_, 0x41000, true, 5, true); // WORD Address, is_lsb, BIT, true/false
    // taskPushBack_delay_1ms(right_drum_coupler_detach_plc_comm_task_, 600);
    // //////////////////////////////////////////////////////////////
    
    // //////////////////////////////////////////////////////////////
    // // [Task 4] 캡 체결 (right_drum_coupler_detach_plc_comm_task_)
    // // 커플러 체결 해제 & 호스 원위치
    // // BIT 6: 캡 체결 작업 시작, BIT 7: 캡 체결 작업 완료
    // right_drum_cap_screwing_plc_comm_task_.clear();
    // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_screwing_plc_comm_task_, 0x41000, true, 6, true); // WORD Address, is_lsb, BIT, true/false
    // taskPushBack_delay_1ms(right_drum_cap_screwing_plc_comm_task_, 600);
    // if(is_plc_test_mode) {
    //     taskPushBack_sendTaskLog(right_drum_cap_screwing_plc_comm_task_, "캡 체결 (PLC Comm.)");
    // } else {
    //     taskPushBack_jsMove2(right_drum_cap_screwing_plc_comm_task_, right_home_pose_, 75.0, 75.0, false);
    //     taskPushBack_delay_1ms(right_drum_cap_screwing_plc_comm_task_, 60);


    //     std::vector<UnitTask> right_drum_sub_task_6 = rightGenSubTask6Task();
    //     right_drum_cap_screwing_plc_comm_task_.insert(right_drum_cap_screwing_plc_comm_task_.end(), right_drum_sub_task_6.begin(), right_drum_sub_task_6.end());
    //     taskPushBack_jsMove2(right_drum_cap_screwing_plc_comm_task_, right_home_pose_, 75.0, 75.0, false);
    //     taskPushBack_delay_1ms(right_drum_cap_screwing_plc_comm_task_, 60);
    // }
    // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_screwing_plc_comm_task_, 0x41000, true, 7, true); // WORD Address, is_lsb, BIT, true/false
    // taskPushBack_delay_1ms(right_drum_cap_screwing_plc_comm_task_, 600);
    // //////////////////////////////////////////////////////////////
}


void TaskPlannerHanyangEng::genPLCCommTask() {

    genTaskSetting();
    genGripperTask();
    
    bool is_plc_test_mode = false; // true: 테스트, 딜레이 3초씩, false: 로봇 작업 수행
    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////
    #if IS_PLC_LS_XG5000
    uint16_t word_address_41000 = static_cast<uint16_t>(0x41000);
    uint16_t word_address_41002 = static_cast<uint16_t>(0x41002);
    uint16_t word_address_41003 = static_cast<uint16_t>(0x41003);
#else
    uint16_t word_address_41000 = static_cast<uint16_t>(1000);
    uint16_t word_address_41002 = static_cast<uint16_t>(1002);
    uint16_t word_address_41003 = static_cast<uint16_t>(1003);
#endif

    if(0) { // 단일 작업 목록
        //// Left PLC Comm. Task
        genLeftDrumPLCCommTask(is_plc_test_mode);

        //// Right PLC Comm. Task
        genRightDrumPLCCommTask(is_plc_test_mode);
    }
  
    //////////////////////////////////////////////////////////////
    //// ******************************************************************************** ////
    //////////////////////////////////////////////////////////////////////////////////////////////
    //// 1) JS Position PLC Comm. Task
    // 1-a) AMR Ready Pose (여기에 도달하여, PLC에 신호를 보내야만 AMR이 다음 동작을 수행함.)
    if(0) {
        std::vector<UnitTask> amr_ready_pose_plc_comm_task_tmp;
        std::string log_tmp = "***** [작업 시작] (AMR) 준비 포즈 *****";
        taskPushBack_sendTaskLog(amr_ready_pose_plc_comm_task_tmp, log_tmp);
        // taskPushBack_jsMove2(amr_ready_pose_plc_comm_task_tmp, amr_ready_pose_, 75.0, 75.0, false);
        // taskPushBack_delay_1ms(amr_ready_pose_plc_comm_task_tmp, 60);
        //// TODO:
        taskPushBack_sendTaskLog(amr_ready_pose_plc_comm_task_tmp, "AMR Ready Pose");
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(amr_ready_pose_plc_comm_task_tmp, word_address_41000, true, 0, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(amr_ready_pose_plc_comm_task_tmp, 1000);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(amr_ready_pose_plc_comm_task_tmp, word_address_41000, true, 1, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(amr_ready_pose_plc_comm_task_tmp, 1000);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(amr_ready_pose_plc_comm_task_tmp, word_address_41000, true, 2, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(amr_ready_pose_plc_comm_task_tmp, 1000);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(amr_ready_pose_plc_comm_task_tmp, word_address_41000, true, 2, false); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(amr_ready_pose_plc_comm_task_tmp, 1000);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(amr_ready_pose_plc_comm_task_tmp, word_address_41000, true, 1, false); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(amr_ready_pose_plc_comm_task_tmp, 1000);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(amr_ready_pose_plc_comm_task_tmp, word_address_41000, true, 0, false); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(amr_ready_pose_plc_comm_task_tmp, 1000);
        ////
        //////////////////////////////////////////////////////////////////////////////
        //// Update
        amr_ready_pose_plc_comm_task_ = amr_ready_pose_plc_comm_task_tmp;
        //////////////////////////////////////////////////////////////////////////////
    }

    // 1-b) Left Drum Home Pose
    if(0) {
        std::vector<UnitTask> left_drum_home_pose_plc_comm_task_tmp;
        std::string log_tmp = "***** [작업 시작] (Left) 홈포즈 *****";
        taskPushBack_sendTaskLog(left_drum_home_pose_plc_comm_task_tmp, log_tmp);
        // taskPushBack_jsMove2(left_drum_home_pose_plc_comm_task_tmp, left_home_pose_, 75.0, 75.0, false);
        // taskPushBack_delay_1ms(left_drum_home_pose_plc_comm_task_tmp, 60);

        //// TODO:
        taskPushBack_sendTaskLog(left_drum_home_pose_plc_comm_task_tmp, "Left Drum Home Pose");
        ////
        //////////////////////////////////////////////////////////////////////////////
        //// Update
        left_drum_home_pose_plc_comm_task_ = left_drum_home_pose_plc_comm_task_tmp;
        //////////////////////////////////////////////////////////////////////////////
    }

    // 1-c) Right Drum Home Pose
    if(0) {
        std::vector<UnitTask> right_drum_home_pose_plc_comm_task_tmp;
        std::string log_tmp = "***** [작업 시작] (Right) 홈포즈 *****";
        taskPushBack_sendTaskLog(right_drum_home_pose_plc_comm_task_tmp, log_tmp);
        // taskPushBack_jsMove2(right_drum_home_pose_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
        // taskPushBack_delay_1ms(right_drum_home_pose_plc_comm_task_tmp, 60);

        //// TODO:
        taskPushBack_sendTaskLog(right_drum_home_pose_plc_comm_task_tmp, "Right Drum Home Pose");
        ////

        //////////////////////////////////////////////////////////////////////////////
        //// Update
        right_drum_home_pose_plc_comm_task_ = right_drum_home_pose_plc_comm_task_tmp;
        //////////////////////////////////////////////////////////////////////////////
    }



    //////////////////////////////////////////////////////////////////////////////////////////////







    //////////////////////////////////////////////////////////////////////////////////////////////
    //// 2) 캡 분리, 커플러 체결 작업의 PLC Comm. Task
    // 2-a) Left Drum
    if(1) {
        std::vector<UnitTask> left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp;
        std::string log_tmp = "***** [작업 시작] (Port 1, Left) 캡 풀기 및 커플러 체결 *****";
        taskPushBack_sendTaskLog(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, log_tmp);


        //// ACK 신호 송신
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 1, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 2, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);


        ////////////////////////////////////////////////////////////////////////////////
        taskPushBack_sendTaskLog(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, "LEFT DRUM - 캡 풀기 시작");
        
        //// [PLC Write] 캡 "풀기 작업" 시작 (설비에 도착하여 Loading 작업 이전에) - BIT 8
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 8, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);

        if(!is_plc_test_mode) { 
            ////////////////////////////////////
            // [Task 1] 별모양 뚜껑 따기
            std::string log_task_step = "left_task_detach_lid_cap";
            taskPushBack_sendTaskLog(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, log_task_step);
            std::vector<UnitTask> left_drum_sub_task_1 = leftGenSubTask1Task();
            left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp.insert(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp.end(), left_drum_sub_task_1.begin(), left_drum_sub_task_1.end());

            taskPushBack_jsMove2(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, left_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 60);
            ////////////////////////////////////
        } else { // 테스트
            taskPushBack_jsMove2(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, left_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 60);
            taskPushBack_delay_1ms(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);
        }

        //// [PLC Write] 캡 "풀기 작업" 완료 (설비에 도착하여 Loading 작업 이전에) - BIT 9
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 9, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_sendTaskLog(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, "LEFT DRUM - 캡 풀기 완료");
        taskPushBack_delay_1ms(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);

        ////////////////////////////////////////////////////////////////////////////////



        ////////////////////////////////////////////////////////////////////////////////
        taskPushBack_sendTaskLog(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, "LEFT DRUM - 커플러 체결 시작");
        //// [PLC Write] 커플러 "체결 작업" 시작     (설비에 도착하여 Loading 작업 이전에) - BIT 10
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 10, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);

        if(!is_plc_test_mode) { 
            ////////////////////////////////////
            std::string log_task_step = "left_task_attach_coupler";
            taskPushBack_sendTaskLog(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, log_task_step);
            std::vector<UnitTask> left_drum_sub_task_2 = leftGenSubTask2Task();
            std::vector<UnitTask> left_drum_sub_task_3 = leftGenSubTask3Task();
            left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp.insert(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp.end(), left_drum_sub_task_2.begin(), left_drum_sub_task_2.end());
            left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp.insert(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp.end(), left_drum_sub_task_3.begin(), left_drum_sub_task_3.end());

            taskPushBack_jsMove2(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 60);
            ////////////////////////////////////
        } else { // 테스트
            taskPushBack_jsMove2(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, left_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 60);
            taskPushBack_delay_1ms(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);
        }


        //// [PLC Write] 커플러 "체결 작업" 시작     (설비에 도착하여 Loading 작업 이전에) - BIT 11
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 11, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_sendTaskLog(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, "LEFT DRUM - 커플러 체결 완료");
        taskPushBack_delay_1ms(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);

        ////////////////////////////////////////////////////////////////////////////////

        log_tmp = "***** [작업 종료] (Port 1, Left) 캡 풀기 및 커플러 체결 *****";
        taskPushBack_sendTaskLog(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, log_tmp);


        //// 비트 14, 작업 종료 신호 
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 14, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);
        //////////////////////////////////////////////////////////////////////////////
        //// Update
        left_drum_cap_detach_coupler_screwing_plc_comm_task_ = left_drum_cap_detach_coupler_screwing_plc_comm_task_tmp;
        //////////////////////////////////////////////////////////////////////////////
    }

    // 2-b) Right Drum
    if(1) {
        std::vector<UnitTask> right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp;
        std::string log_tmp = "***** [작업 시작] (Port 2, Right) 캡 풀기 및 커플러 체결 *****";
        taskPushBack_sendTaskLog(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, log_tmp);

        //// ACK 신호 송신
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 1, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 1, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 1, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 3, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 3, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 3, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);


        ////////////////////////////////////////////////////////////////////////////////
        taskPushBack_sendTaskLog(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, "RIGHT DRUM - 캡 풀기 시작");
        
        //// [PLC Write] 캡 "풀기 작업" 시작 (설비에 도착하여 Loading 작업 이전에) - BIT 8
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 8, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);

        if(!is_plc_test_mode) { 
            ////////////////////////////////////
            std::string log_task_step = "right_task_detach_lid_cap";
            taskPushBack_sendTaskLog(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, log_task_step);
            std::vector<UnitTask> right_drum_sub_task_1 = rightGenSubTask1Task();
            right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp.insert(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp.end(), right_drum_sub_task_1.begin(), right_drum_sub_task_1.end());

            taskPushBack_jsMove2(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 60);
            ////////////////////////////////////
        } else { // 테스트
            taskPushBack_jsMove2(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 60);
            taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);
        }

        //// [PLC Write] 캡 "풀기 작업" 완료 (설비에 도착하여 Loading 작업 이전에) - BIT 9
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 9, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_sendTaskLog(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, "RIGHT DRUM - 캡 풀기 완료");
        taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);
        ////////////////////////////////////////////////////////////////////////////////



        ////////////////////////////////////////////////////////////////////////////////
        taskPushBack_sendTaskLog(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, "RIGHT DRUM - 커플러 체결 시작");
        //// [PLC Write] 커플러 "체결 작업" 시작     (설비에 도착하여 Loading 작업 이전에) - BIT 10
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 10, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);


        if(!is_plc_test_mode) { 
            ////////////////////////////////////
            std::string log_task_step = "right_task_attach_coupler";
            taskPushBack_sendTaskLog(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, log_task_step);
            std::vector<UnitTask> right_drum_sub_task_2 = rightGenSubTask2Task();
            std::vector<UnitTask> right_drum_sub_task_3 = rightGenSubTask3Task();
            right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp.insert(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp.end(), right_drum_sub_task_2.begin(), right_drum_sub_task_2.end());
            right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp.insert(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp.end(), right_drum_sub_task_3.begin(), right_drum_sub_task_3.end());

            taskPushBack_jsMove2(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 60);
            ////////////////////////////////////
        } else { // 테스트
            taskPushBack_jsMove2(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 60);
            taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);
        }


        //// [PLC Write] 커플러 "체결 작업" 시작     (설비에 도착하여 Loading 작업 이전에) - BIT 11
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 11, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_sendTaskLog(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, "RIGHT DRUM - 커플러 체결 완료");
        taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);

        ////////////////////////////////////////////////////////////////////////////////

        log_tmp = "***** [작업 종료] (Port 2, Right) 캡 풀기 및 커플러 체결 *****";
        taskPushBack_sendTaskLog(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, log_tmp);

        //// 비트 14, 작업 종료 신호 
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 14, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, word_address_41000, true, 14, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 500);

        //// PLC MONITROING FLAG ON
        taskPushBack_setPLCDoMonitoringFlagOnOff(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, false);
        taskPushBack_delay_1ms(right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp, 50);
        //// PLC MONITROING FLAG ON
        //////////////////////////////////////////////////////////////////////////////
        //// Update
        right_drum_cap_detach_coupler_screwing_plc_comm_task_ = right_drum_cap_detach_coupler_screwing_plc_comm_task_tmp;
        //////////////////////////////////////////////////////////////////////////////
    }
    //////////////////////////////////////////////////////////////////////////////////////////////


    //////////////////////////////////////////////////////////////////////////////////////////////
    //// 3) 커플러 해제, 캡 체결 작업의 PLC Comm. Task
    // 3-a) Left Drum
    if(1) {
        std::vector<UnitTask> left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp;
        std::string log_tmp = "***** [작업 시작] (Port 1, Left) 커플러 풀기 및 캡 체결 *****";
        taskPushBack_sendTaskLog(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, log_tmp);

        //// ACK 신호 송신
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 0, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 2, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);



        ////////////////////////////////////////////////////////////////////////////////
        taskPushBack_sendTaskLog(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, "LEFT DRUM - 커플러 풀기 시작");
        
        //// [PLC Write] 커플러 "풀기 작업" 시작 (설비에 도착하여 Loading 작업 이전에) - BIT 4
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 4, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);

        if(!is_plc_test_mode) { 
            ////////////////////////////////////
            std::string log_task_step = "left_task_detach_coupler";
            taskPushBack_sendTaskLog(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, log_task_step);
            std::vector<UnitTask> left_drum_sub_task_4 = leftGenSubTask4Task();
            std::vector<UnitTask> left_drum_sub_task_5 = leftGenSubTask5Task();
            left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp.insert(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp.end(), left_drum_sub_task_4.begin(), left_drum_sub_task_4.end());
            left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp.insert(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp.end(), left_drum_sub_task_5.begin(), left_drum_sub_task_5.end());
            taskPushBack_jsMove2(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, left_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 60);
            ////////////////////////////////////
        } else { // 테스트
            taskPushBack_jsMove2(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, left_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 60);
            taskPushBack_delay_1ms(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);
        }

        //// [PLC Write] 커플러 "풀기 작업" 완료 (설비에 도착하여 Loading 작업 이전에) - BIT 5
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 5, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_sendTaskLog(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, "LEFT DRUM - 커플러 풀기 완료");
        taskPushBack_delay_1ms(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);

        ////////////////////////////////////////////////////////////////////////////////



        ////////////////////////////////////////////////////////////////////////////////
        taskPushBack_sendTaskLog(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, "LEFT DRUM - 캡 체결 시작");
        //// [PLC Write] 캡 "체결 작업" 시작     (설비에 도착하여 Loading 작업 이전에) - BIT 6
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 6, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);

        if(!is_plc_test_mode) { 
            ////////////////////////////////////
            std::string log_task_step = "left_task_attach_lid_cap";
            taskPushBack_sendTaskLog(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, log_task_step);
            std::vector<UnitTask> left_drum_sub_task_6 = leftGenSubTask6Task();
            left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp.insert(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp.end(), left_drum_sub_task_6.begin(), left_drum_sub_task_6.end());
            taskPushBack_jsMove2(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 60);
            ////////////////////////////////////
        } else { // 테스트
            taskPushBack_jsMove2(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, left_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 60);
            taskPushBack_delay_1ms(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);
        }

        //// [PLC Write] 캡 "체결 작업" 시작     (설비에 도착하여 Loading 작업 이전에) - BIT 7
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 7, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_sendTaskLog(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, "LEFT DRUM - 캡 체결 완료");
        taskPushBack_delay_1ms(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);

        ////////////////////////////////////////////////////////////////////////////////


        log_tmp = "***** [작업 종료] (Port 1, Left) 커플러 풀기 및 캡 체결 *****";
        taskPushBack_sendTaskLog(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, log_tmp);

        //// 비트 14, 작업 종료 신호 
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 14, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);

        //////////////////////////////////////////////////////////////////////////////
        //// Update
        left_drum_coupler_detach_cap_screwing_plc_comm_task_ = left_drum_coupler_detach_cap_screwing_plc_comm_task_tmp;
        //////////////////////////////////////////////////////////////////////////////
    }

    // 3-b) Right Drum
    if(1) {
        std::vector<UnitTask> right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp;
        std::string log_tmp = "***** [작업 시작] (Port 2, Right) 커풀러 풀기 및 캡 체결 *****";
        taskPushBack_sendTaskLog(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, log_tmp);

        //// ACK 신호 송신
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 0, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 3, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 0, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 3, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 0, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 3, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);


        ////////////////////////////////////////////////////////////////////////////////
        taskPushBack_sendTaskLog(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, "RIGHT DRUM - 커플러 풀기 시작");

        //// [PLC Write] 커플러 "풀기 작업" 시작 (설비에 도착하여 Loading 작업 이전에) - BIT 4
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 4, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);

        // //// PLC MONITROING FLAG ON
        // taskPushBack_setPLCDoMonitoringFlagOnOff(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, true);
        // taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 50);
        //// PLC MONITROING FLAG ON
        if(!is_plc_test_mode) { 
            ////////////////////////////////////
            std::string log_task_step = "right_task_detach_coupler";
            taskPushBack_sendTaskLog(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, log_task_step);
            std::vector<UnitTask> right_drum_sub_task_4 = rightGenSubTask4Task();
            std::vector<UnitTask> right_drum_sub_task_5 = rightGenSubTask5Task();
            right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp.insert(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp.end(), right_drum_sub_task_4.begin(), right_drum_sub_task_4.end());
            right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp.insert(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp.end(), right_drum_sub_task_5.begin(), right_drum_sub_task_5.end());
            taskPushBack_jsMove2(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 60);
            ////////////////////////////////////
        } else { // 테스트
            taskPushBack_jsMove2(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 60);
            taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);
        }
        // //// PLC MONITROING FLAG OFF
        // taskPushBack_setPLCDoMonitoringFlagOnOff(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, false);
        // taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 1200);
        //// PLC MONITROING FLAG OFF
        //// [PLC Write] 커플러 "풀기 작업" 완료 (설비에 도착하여 Loading 작업 이전에) - BIT 5
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 5, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_sendTaskLog(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, "RIGHT DRUM - 커플러 풀기 완료");
        taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);
        ////////////////////////////////////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////////////////////
        taskPushBack_sendTaskLog(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, "RIGHT DRUM - 캡 체결 시작");
        //// [PLC Write] 캡 "체결 작업" 시작     (설비에 도착하여 Loading 작업 이전에) - BIT 6
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 6, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);

        // //// PLC MONITROING FLAG ON
        // taskPushBack_setPLCDoMonitoringFlagOnOff(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, true);
        // taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 50);
        //// PLC MONITROING FLAG ON
        if(!is_plc_test_mode) { 
            ////////////////////////////////////
            std::string log_task_step = "right_task_attach_lid_cap";
            taskPushBack_sendTaskLog(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, log_task_step);
            std::vector<UnitTask> right_drum_sub_task_6 = rightGenSubTask6Task();
            right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp.insert(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp.end(), right_drum_sub_task_6.begin(), right_drum_sub_task_6.end());
            taskPushBack_jsMove2(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 60);
            ////////////////////////////////////
        } else { // 테스트
            taskPushBack_jsMove2(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 60);
            taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);
        }

        // //// PLC MONITROING FLAG OFF
        // taskPushBack_setPLCDoMonitoringFlagOnOff(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, false);
        // taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 1200);
        //// PLC MONITROING FLAG OFF
        //// [PLC Write] 캡 "체결 작업" 시작     (설비에 도착하여 Loading 작업 이전에) - BIT 7
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 7, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_sendTaskLog(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, "RIGHT DRUM - 캡 체결 완료");
        taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);
        ////////////////////////////////////////////////////////////////////////////////

        log_tmp = "***** [작업 종료] (Port 2, Right) 커풀러 풀기 및 캡 체결 *****";
        taskPushBack_sendTaskLog(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, log_tmp);

        //// 비트 14, 작업 종료 신호 
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 14, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, word_address_41000, true, 14, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 500);


        //// PLC MONITROING FLAG ON
        taskPushBack_setPLCDoMonitoringFlagOnOff(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, false);
        taskPushBack_delay_1ms(right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp, 50);
        //// PLC MONITROING FLAG ON

        //////////////////////////////////////////////////////////////////////////////
        //// Update
        right_drum_coupler_detach_cap_screwing_plc_comm_task_ = right_drum_coupler_detach_cap_screwing_plc_comm_task_tmp;
        //////////////////////////////////////////////////////////////////////////////
    }
    //////////////////////////////////////////////////////////////////////////////////////////////



    //////////////////////////////////////////////////////////////////////////////////////////////
    //// 4) 드럼 회전각도 인식
    if(1) { //// PLC Task - Turn this flag ON
        std::vector<UnitTask> drum_rotating_angle_detection_plc_comm_task_tmp;
        std::string log_tmp = "***** [작업 시작] 턴테이블 회전각도 인식 *****";
        taskPushBack_sendTaskLog(drum_rotating_angle_detection_plc_comm_task_tmp, log_tmp);

        //// ACK 신호 송신
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(drum_rotating_angle_detection_plc_comm_task_tmp, word_address_41002, true, 4, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 500);     
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(drum_rotating_angle_detection_plc_comm_task_tmp, word_address_41002, true, 4, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 500);    
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(drum_rotating_angle_detection_plc_comm_task_tmp, word_address_41002, true, 4, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 500);    

        ////////////////////////////////////////////////////////////////////////////////
        taskPushBack_sendTaskLog(drum_rotating_angle_detection_plc_comm_task_tmp, "턴테이블 회전각도 인식 시작");
        //// [PLC Write] 턴테이블 각도 인식 시작
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(drum_rotating_angle_detection_plc_comm_task_tmp, word_address_41002, true, 0, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 500);

        if(!is_plc_test_mode) { 
            ////////////////////////////////////
            taskPushBack_jsMove2(drum_rotating_angle_detection_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 60);


            ////////////////////////////////////////////////////////////////////////////////////////////////////
            //// 250507
            std::string target_object = "drum_lid_total";
            std::string target_tcp = "tcp05";
            std::string target_task = "default"; // 툴 기준 Z 방향으로 approach
            JsDouble drum_rotation_angle_detection_scan_pose;
            if (std::string(ROBOT_NAME) == "DS_M1013") {
                JsDouble js_pose_tmp = {1.427, -31.801, 75.443, -3.921, 115.384, -90.814};
                drum_rotation_angle_detection_scan_pose = js_pose_tmp;
            } else if (std::string(ROBOT_NAME) == "UR10e") {
                JsDouble js_pose_tmp = {-1.590, -72.253, -68.887, -105.765, 90.678, -1.316};
                drum_rotation_angle_detection_scan_pose = js_pose_tmp;
            }        
        
            taskPushBack_jsMove2(drum_rotating_angle_detection_plc_comm_task_tmp, drum_rotation_angle_detection_scan_pose, 30.0, 30.0, false);
            taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 60);
        
            if (std::string(ROBOT_NAME) == "DS_M1013") {
                taskPushBack_drflSetTCP(drum_rotating_angle_detection_plc_comm_task_tmp, "tcp00"); // Scan & Matching은 tcp00
                taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 60);
            }
        
            taskPushBack_task3DScanningTargetObject(drum_rotating_angle_detection_plc_comm_task_tmp, target_object); // ZIVID Scanning with task recognition
            taskPushBack_taskMatchingTargetObject(drum_rotating_angle_detection_plc_comm_task_tmp, target_object, false, 8); // "default", true: symmetric CAD used
            
            //// [PLC Write] 턴테이블 "회전각도 인식" OK 응답
            taskPushBack_PLCModbusWriteStatusFromRobotToAMR(drum_rotating_angle_detection_plc_comm_task_tmp, word_address_41002, true, 2, true); // WORD Address, is_lsb, BIT, true/false
            taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 500);

            //// [PLC Write] 턴테이블 "회전각도 업데이트" (각도: -180.0~180.0, resolution: 0.1deg, 전송 데이터: -1800~1800) 
            taskPushBack_setPLCModbusRotationAngle(drum_rotating_angle_detection_plc_comm_task_tmp, 1001, 0 );
            taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 8000);
            ////////////////////////////////////////////////////////////////////////////////////////////////////



            ////////////////////////////////////
        } else { // 테스트
            taskPushBack_jsMove2(drum_rotating_angle_detection_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 60);
            taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 500);

            //// [PLC Write] 턴테이블 "회전각도 인식" OK 응답
            taskPushBack_PLCModbusWriteStatusFromRobotToAMR(drum_rotating_angle_detection_plc_comm_task_tmp, word_address_41002, true, 2, true); // WORD Address, is_lsb, BIT, true/false
            taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 500);

            //// [PLC Write] 턴테이블 "회전각도 업데이트" (각도: -180.0~180.0, resolution: 0.1deg, 전송 데이터: -1800~1800) 
            taskPushBack_setPLCModbusRotationAngle(drum_rotating_angle_detection_plc_comm_task_tmp, 1001, 0, 0.5);
            taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 500);

        }
        taskPushBack_jsMove2(drum_rotating_angle_detection_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
        taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 60);
        
        //// [PLC Write] 턴테이블 각도 인식 완료
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(drum_rotating_angle_detection_plc_comm_task_tmp, word_address_41002, true, 1, true); // WORD Address, is_lsb, BIT, true/false
        log_tmp = "***** [작업 종료] 턴테이블 회전각도 *****";
        taskPushBack_sendTaskLog(drum_rotating_angle_detection_plc_comm_task_tmp, log_tmp);
        taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 500);

        taskPushBack_jsMove2(drum_rotating_angle_detection_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
        taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 60);


        //// 비트 14, 작업 종료 신호 
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(drum_rotating_angle_detection_plc_comm_task_tmp, word_address_41000, true, 14, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 500);
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(drum_rotating_angle_detection_plc_comm_task_tmp, word_address_41000, true, 14, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 500);
        //////////////////////////////////////////////////////////////////////////////

        //// PLC MONITROING FLAG ON
        taskPushBack_setPLCDoMonitoringFlagOnOff(drum_rotating_angle_detection_plc_comm_task_tmp, false);
        taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 50);
        //// PLC MONITROING FLAG ON


        //// Update
        drum_rotating_angle_detection_plc_comm_task_ = drum_rotating_angle_detection_plc_comm_task_tmp;
        //////////////////////////////////////////////////////////////////////////////


    } else { //// Button Test
        
        std::vector<UnitTask> drum_rotating_angle_detection_plc_comm_task_tmp;
        std::string log_tmp = "***** [Test 작업 시작] 턴테이블 회전각도 인식 *****";
        taskPushBack_sendTaskLog(drum_rotating_angle_detection_plc_comm_task_tmp, log_tmp);

        ////////////////////////////////////////////////////////////////////////////////////////////////////
        //// 250507
        std::string target_object = "drum_lid_total";
        std::string target_tcp = "tcp05";
        std::string target_task = "default"; // 툴 기준 Z 방향으로 approach
        JsDouble drum_rotation_angle_detection_scan_pose;
        if (std::string(ROBOT_NAME) == "DS_M1013") {
            JsDouble js_pose_tmp = {1.427, -31.801, 75.443, -3.921, 115.384, -90.814};
            drum_rotation_angle_detection_scan_pose = js_pose_tmp;
        } else if (std::string(ROBOT_NAME) == "UR10e") {
            JsDouble js_pose_tmp = {-1.590, -72.253, -68.887, -105.765, 90.678, -1.316};
            drum_rotation_angle_detection_scan_pose = js_pose_tmp;
        }        
    
        taskPushBack_jsMove2(drum_rotating_angle_detection_plc_comm_task_tmp, drum_rotation_angle_detection_scan_pose, 30.0, 30.0, false);
        taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 60);
    
        if (std::string(ROBOT_NAME) == "DS_M1013") {
            taskPushBack_drflSetTCP(drum_rotating_angle_detection_plc_comm_task_tmp, "tcp00"); // Scan & Matching은 tcp00
            taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 60);
        }
    
        taskPushBack_task3DScanningTargetObject(drum_rotating_angle_detection_plc_comm_task_tmp, target_object); // ZIVID Scanning with task recognition
        taskPushBack_taskMatchingTargetObject(drum_rotating_angle_detection_plc_comm_task_tmp, target_object, false, 8); // "default", true: symmetric CAD used
    
        taskPushBack_setPLCModbusRotationAngle(drum_rotating_angle_detection_plc_comm_task_tmp, 1001, 0);
        taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 500);
        ////////////////////////////////////////////////////////////////////////////////////////////////////


        log_tmp = "***** [Test 작업 종료] 턴테이블 회전각도 *****";
        taskPushBack_sendTaskLog(drum_rotating_angle_detection_plc_comm_task_tmp, log_tmp);
        taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 500);

        taskPushBack_jsMove2(drum_rotating_angle_detection_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
        taskPushBack_delay_1ms(drum_rotating_angle_detection_plc_comm_task_tmp, 60);
        //////////////////////////////////////////////////////////////////////////////
        //// Update
        drum_rotating_angle_detection_plc_comm_task_ = drum_rotating_angle_detection_plc_comm_task_tmp;
        //////////////////////////////////////////////////////////////////////////////
    }
    //////////////////////////////////////////////////////////////////////////////////////////////



    
    //////////////////////////////////////////////////////////////////////////////////////////////
    //// 5) 바코드 인식
    if(0) {
        std::vector<UnitTask> drum_barcode_read_plc_comm_task_tmp;
        std::string log_tmp = "***** [작업 시작] 바코드 인식 *****";
        taskPushBack_sendTaskLog(drum_barcode_read_plc_comm_task_tmp, log_tmp);
        //// ACK 신호 송신
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(drum_barcode_read_plc_comm_task_tmp, word_address_41002, true, 5, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(drum_barcode_read_plc_comm_task_tmp, 500);   

        ////////////////////////////////////////////////////////////////////////////////
        taskPushBack_sendTaskLog(drum_barcode_read_plc_comm_task_tmp, "바코드 인식 시작");
        //// [PLC Write] 바코드 인식 시작
        // taskPushBack_PLCModbusWriteStatusFromRobotToAMR(drum_barcode_read_plc_comm_task_tmp, word_address_41002, true, 0, true); // WORD Address, is_lsb, BIT, true/false
        // taskPushBack_delay_1ms(drum_barcode_read_plc_comm_task_tmp, 1000);

        if(!is_plc_test_mode) { 
            ////////////////////////////////////
            taskPushBack_jsMove2(drum_barcode_read_plc_comm_task_tmp, amr_ready_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(drum_barcode_read_plc_comm_task_tmp, 60);
            taskPushBack_jsMove2(drum_barcode_read_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(drum_barcode_read_plc_comm_task_tmp, 60);
            // taskPushBack_jsMove2(drum_barcode_read_plc_comm_task_tmp, drum_rotating_angle_detection_scan_pose_, 75.0, 75.0, false);
            // taskPushBack_delay_1ms(drum_barcode_read_plc_comm_task_tmp, 60);
            ////////////////////////////////////
        } else { // 테스트
            taskPushBack_jsMove2(drum_barcode_read_plc_comm_task_tmp, right_home_pose_, 75.0, 75.0, false);
            taskPushBack_delay_1ms(drum_barcode_read_plc_comm_task_tmp, 60);
            taskPushBack_delay_1ms(drum_barcode_read_plc_comm_task_tmp, 500);
        }

        //// [PLC Write] Barcode Reading OK
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(drum_barcode_read_plc_comm_task_tmp, 1000, true, 12, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(drum_barcode_read_plc_comm_task_tmp, 500);

        //// [PLC Write] "Barcode 업데이트"
        taskPushBack_setPLCModbusBarcodeWrite(drum_barcode_read_plc_comm_task_tmp, 1001, 0, "0123456789ABCDEFGHIJ");
        taskPushBack_delay_1ms(drum_barcode_read_plc_comm_task_tmp, 500);

        //// [PLC Write] 바코드 인식 완료
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(drum_barcode_read_plc_comm_task_tmp, word_address_41002, true, 6, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_sendTaskLog(drum_barcode_read_plc_comm_task_tmp, "바코드 인식 완료");
        taskPushBack_delay_1ms(drum_barcode_read_plc_comm_task_tmp, 500);
        ////////////////////////////////////////////////////////////////////////////////

        log_tmp = "***** [작업 종료] 바코드 인식 *****";
        taskPushBack_sendTaskLog(drum_barcode_read_plc_comm_task_tmp, log_tmp);

        //// 비트 14, 작업 종료 신호 
        taskPushBack_PLCModbusWriteStatusFromRobotToAMR(drum_barcode_read_plc_comm_task_tmp, 1000, true, 14, true); // WORD Address, is_lsb, BIT, true/false
        taskPushBack_delay_1ms(drum_barcode_read_plc_comm_task_tmp, 500);

        //////////////////////////////////////////////////////////////////////////////
        //// Update
        drum_barcode_read_plc_comm_task_ = drum_barcode_read_plc_comm_task_tmp;
        //////////////////////////////////////////////////////////////////////////////
    }
    //////////////////////////////////////////////////////////////////////////////////////////////

    return;
}

/// 두산 로봇 데모
void TaskPlannerHanyangEng::rightGenDrumTask() {
    try {
        ROS_LOG_WARN("[%s] Generating Hanyang Eng. Task...", __func__);
        genTaskSetting();
        genGripperTask();
        right_drum_total_task_.clear();



    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////
    // [Task 1] 별모양 뚜껑 따기
    right_drum_sub_task_1_ = rightGenSubTask1Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_1_.begin(), right_drum_sub_task_1_.end());
    //////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////
    // [Task 2] 구멍 인식 & 호스 파지 & 구멍 삽입 자세 직전
    right_drum_sub_task_2_ = rightGenSubTask2Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_2_.begin(), right_drum_sub_task_2_.end());
    //////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////
    // [Task 3] 호스 체결
    right_drum_sub_task_3_ = rightGenSubTask3Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_3_.begin(), right_drum_sub_task_3_.end());
    //////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////
    // [Task 4] 호스 체결 해제
    right_drum_sub_task_4_ = rightGenSubTask4Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_4_.begin(), right_drum_sub_task_4_.end());
    //////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////
    // [Task 5] 호스 원위치
    right_drum_sub_task_5_ = rightGenSubTask5Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_5_.begin(), right_drum_sub_task_5_.end());
    //////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////
    // [Task 6] 별 모양 뚜껑 원위치
    right_drum_sub_task_6_ = rightGenSubTask6Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_6_.begin(), right_drum_sub_task_6_.end());


    // [Task 7] 250819 데모
    right_drum_sub_task_7_ = rightGenSubTask7Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_7_.begin(), right_drum_sub_task_7_.end());
   
    right_drum_sub_task_8_ = rightGenSubTask8Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_8_.begin(), right_drum_sub_task_8_.end());

    right_drum_sub_task_9_ = rightGenSubTask9Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_9_.begin(), right_drum_sub_task_9_.end());

    right_drum_sub_task_10_ = rightGenSubTask10Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_10_.begin(), right_drum_sub_task_10_.end());

    right_drum_sub_task_11_ = rightGenSubTask11Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_11_.begin(), right_drum_sub_task_11_.end());

    right_drum_sub_task_12_ = rightGenSubTask12Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_12_.begin(), right_drum_sub_task_12_.end());

    right_drum_sub_task_13_ = rightGenSubTask13Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_13_.begin(), right_drum_sub_task_13_.end());

    right_drum_sub_task_14_ = rightGenSubTask14Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_14_.begin(), right_drum_sub_task_14_.end());

    right_drum_sub_task_15_ = rightGenSubTask15Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_15_.begin(), right_drum_sub_task_15_.end());

    right_drum_sub_task_16_ = rightGenSubTask16Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_16_.begin(), right_drum_sub_task_16_.end());

    right_drum_sub_task_17_ = rightGenSubTask17Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_17_.begin(), right_drum_sub_task_17_.end());

    right_drum_sub_task_18_ = rightGenSubTask18Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_18_.begin(), right_drum_sub_task_18_.end());

    right_drum_sub_task_19_ = rightGenSubTask19Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_19_.begin(), right_drum_sub_task_19_.end());

    right_drum_sub_task_20_ = rightGenSubTask20Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_20_.begin(), right_drum_sub_task_20_.end());

    right_drum_sub_task_21_ = rightGenSubTask21Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_21_.begin(), right_drum_sub_task_21_.end());

    right_drum_sub_task_22_ = rightGenSubTask22Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_22_.begin(), right_drum_sub_task_22_.end());

    right_drum_sub_task_23_ = rightGenSubTask23Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_23_.begin(), right_drum_sub_task_23_.end());

    right_drum_sub_task_24_ = rightGenSubTask24Task();
    right_drum_total_task_.insert(right_drum_total_task_.end(), right_drum_sub_task_24_.begin(), right_drum_sub_task_24_.end());
    //////////////////////////////////////////////////////////////

    // CsDouble test_cs_pose1 = {-0.07764, -0.49377, 0.57596, 85.853, -66.065, 85.812};
    // taskPushBack_csMove(right_drum_total_task_, test_cs_pose1, false);
    // taskPushBack_delay_1ms(right_drum_total_task_, 60);
    ROS_LOG_WARN("[%s] Generating Hanyang Eng. Task Finished!", __func__);
    
    std::cout << "rightGenDrumTask completed successfully" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Exception in rightGenDrumTask: " << e.what() << std::endl;
        throw;
    } catch (...) {
        std::cerr << "Unknown exception in rightGenDrumTask" << std::endl;
        throw;
    }
}

void TaskPlannerHanyangEng::leftGenDrumTask() {
    ROS_LOG_WARN("[%s] Generating Hanyang Eng. Task...", __func__);
    genTaskSetting();
    genGripperTask();
    left_drum_total_task_.clear();

    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////
    // [Task 1] 별모양 뚜껑 따기
    left_drum_sub_task_1_ = leftGenSubTask1Task();
    left_drum_total_task_.insert(left_drum_total_task_.end(), left_drum_sub_task_1_.begin(), left_drum_sub_task_1_.end());
    //////////////////////////////////////////////////////////////

    left_drum_sub_task_2_ = leftGenSubTask2Task();
    left_drum_total_task_.insert(left_drum_total_task_.end(), left_drum_sub_task_2_.begin(), left_drum_sub_task_2_.end());
    //////////////////////////////////////////////////////////////

    left_drum_sub_task_3_ = leftGenSubTask3Task();
    left_drum_total_task_.insert(left_drum_total_task_.end(), left_drum_sub_task_3_.begin(), left_drum_sub_task_3_.end());
    //////////////////////////////////////////////////////////////

    left_drum_sub_task_4_ = leftGenSubTask4Task();
    left_drum_total_task_.insert(left_drum_total_task_.end(), left_drum_sub_task_4_.begin(), left_drum_sub_task_4_.end());
    //////////////////////////////////////////////////////////////

    left_drum_sub_task_5_ = leftGenSubTask5Task();
    left_drum_total_task_.insert(left_drum_total_task_.end(), left_drum_sub_task_5_.begin(), left_drum_sub_task_5_.end());

    left_drum_sub_task_6_ = leftGenSubTask6Task();
    left_drum_total_task_.insert(left_drum_total_task_.end(), left_drum_sub_task_6_.begin(), left_drum_sub_task_6_.end());
    //////////////////////////////////////////////////////////////

    left_drum_sub_task_7_ = leftGenSubTask7Task();
    left_drum_total_task_.insert(left_drum_total_task_.end(), left_drum_sub_task_7_.begin(), left_drum_sub_task_7_.end());
    //////////////////////////////////////////////////////////////

    left_drum_sub_task_8_ = leftGenSubTask8Task();
    left_drum_total_task_.insert(left_drum_total_task_.end(), left_drum_sub_task_8_.begin(), left_drum_sub_task_8_.end());
    //////////////////////////////////////////////////////////////
    ROS_LOG_WARN("[%s] Generating Hanyang Eng. Task Finished!", __func__);
}


std::vector<UnitTask> TaskPlannerHanyangEng::rightHomePoseTask() {
    std::vector<UnitTask> right_task_homepose;

    taskPushBack_jsMove2(right_task_homepose, right_home_pose_, 75.0, 75.0, false);
    taskPushBack_delay_1ms(right_task_homepose, 60);

    return right_task_homepose;

}



/// Sub Task 1: 별모양 뚜껑 따서 거치
std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask1Task() {
    std::vector<UnitTask> right_task_sub_1;

    if(1) // 별모양 뚜껑 제거
    {
        // setup
        taskPushBack_jsMove2(right_task_sub_1, right_home_pose_, 40.0, 40.0, false);
        // taskPushBack_delay_1ms(right_task_sub_1, 60);

        right_task_sub_1.insert(right_task_sub_1.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
        taskPushBack_delay_1ms(right_task_sub_1, 2000);

        JsDouble scan_pose_hole_star = {31.644, -11.474, 99.474, 33.409, 73.069, -153.435};
        taskPushBack_jsMove2(right_task_sub_1, scan_pose_hole_star, 40.0, 40.0, false);
        // taskPushBack_delay_1ms(right_task_sub_1, 60);

        right_task_sub_1.insert(right_task_sub_1.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(right_task_sub_1, 500);

        /////////////////////////////////////////////////////////////
        std::string target_object = "right_drum_lid_cap_unscrewing";  
        std::string target_task = "default";

        //// Scanning
        taskPushBack_drflSetTCP(right_task_sub_1, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_1, 60);

        taskPushBack_task3DScanningTargetObject(right_task_sub_1, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_1, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(right_task_sub_1, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_1, 1000);

        taskPushBack_drflSetTCP(right_task_sub_1, "tcp05"); // new gripper
        taskPushBack_delay_1ms(right_task_sub_1, 60);

        //// 천장 부딪히지 않도록 경유지 설정
        if(1) {
            JsDouble waypoint_1 = {39.638, -8.522, 106.590, -0.193, 82.717, -133.691};
            taskPushBack_jsMove2(right_task_sub_1, waypoint_1, 40.0, 40.0, false);
            // taskPushBack_delay_1ms(right_task_sub_1, 50);
            taskPushBack_csMove2(right_task_sub_1, {0, 0.4, 0, 0, 0, 0}, 0.7, 0.7, true); // false: absolute, true: relative
            // taskPushBack_delay_1ms(right_task_sub_1, 50);
        }

        taskPushBack_doGrasping_sharedTask(right_task_sub_1, 0.1, 0.1, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(right_task_sub_1, 60);


        taskPushBack_csMoveToolFrame(right_task_sub_1, {0, 0, 0.055, 0, 0, 0}, 0.05, 0.1, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(right_task_sub_1, 50);


        for (int cycle = 0; cycle < 6; cycle++) {
            right_task_sub_1.insert(right_task_sub_1.end(), drum_grp_lid_cap_close_.begin(), drum_grp_lid_cap_close_.end());
            taskPushBack_delay_1ms(right_task_sub_1, 3000);
            taskPushBack_jsMove2(right_task_sub_1, {0, 0, 0, 0, 0, -195}, 70.0, 70.0, true);
            right_task_sub_1.insert(right_task_sub_1.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
            taskPushBack_delay_1ms(right_task_sub_1, 1000);
            taskPushBack_jsMove2(right_task_sub_1, {0, 0, 0, 0, 0, 15}, 40.0, 40.0, true);
            // taskPushBack_delay_1ms(right_task_sub_1, 60);
            taskPushBack_csMoveToolFrame(right_task_sub_1, {0, 0, -0.050, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
            // taskPushBack_delay_1ms(right_task_sub_1, 50);
            taskPushBack_jsMove2(right_task_sub_1, {0, 0, 0, 0, 0, 180}, 70.0, 70.0, true);
            taskPushBack_csMoveToolFrame(right_task_sub_1, {0, 0, 0.050, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
            // taskPushBack_delay_1ms(right_task_sub_1, 50);
        }

        taskPushBack_delay_1ms(right_task_sub_1, 1000);
        right_task_sub_1.insert(right_task_sub_1.end(), drum_grp_lid_cap_close_tight.begin(), drum_grp_lid_cap_close_tight.end());
        taskPushBack_delay_1ms(right_task_sub_1, 4000);

        taskPushBackKORASGripperCmd(right_task_sub_1, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(right_task_sub_1, 60);
        taskPushBackKORASGripperCmd(right_task_sub_1, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -3000);
        taskPushBack_delay_1ms(right_task_sub_1, 500);

        taskPushBack_jsMove2(right_task_sub_1, {0, 0, 0, 0, 0, -30}, 40.0, 40.0, true);
        // taskPushBack_delay_1ms(right_task_sub_1, 500);
        taskPushBack_csMove2(right_task_sub_1, {0, 0, 0.02, 0, 0, 0}, 0.025, 0.05, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(right_task_sub_1, 500);
        taskPushBack_csMove2(right_task_sub_1, {0, 0, 0.02, 0, 0, 0}, 0.2, 0.2, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(right_task_sub_1, 500);
        taskPushBack_csMove2(right_task_sub_1, {0, -0.3, 0, 0, 0, 0}, 0.5, 0.5, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(right_task_sub_1, 50);
    }
 
    if(1) // 별모양 뚜껑 거치대에 놓기
    {
        std::string target_object = "right_holder_without_lid_cap";
        std::string target_tcp = "tcp05";
        std::string target_task = "default";

        /// 스캔 전 경유자세 (sub right home pose)
        taskPushBack_jsMove2(right_task_sub_1, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 40.0, 40.0, false);
        taskPushBack_delay_1ms(right_task_sub_1, 60);

        taskPushBack_csMove2(right_task_sub_1, {0, 0.300, 0, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_1, 60);

        taskPushBack_drflSetTCP(right_task_sub_1, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_1, 60);

        //// Scanning
        taskPushBack_task3DScanningTargetObject(right_task_sub_1, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_1, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(right_task_sub_1, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_1, 1000);

        taskPushBack_drflSetTCP(right_task_sub_1, target_tcp); // new gripper
        taskPushBack_delay_1ms(right_task_sub_1, 60);

        taskPushBack_doGrasping_sharedTask(right_task_sub_1, 0.05, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(right_task_sub_1, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_1, {0, -0.002, 0.025, 0, 0, 0}, 0.05, 0.05, true);    
        taskPushBack_delay_1ms(right_task_sub_1, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_1, {0, 0, 0.020, 0, 0, 0}, 0.025, 0.025, true);    
        taskPushBack_delay_1ms(right_task_sub_1, 60);

        // taskPushBack_csMoveToolFrame(right_task_sub_1, {0, 0, 0.015, 0, 0, 0}, 0.025, 0.025, true);    
        // taskPushBack_delay_1ms(right_task_sub_1, 60);

        right_task_sub_1.insert(right_task_sub_1.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(right_task_sub_1, 2500);


        taskPushBack_csMoveToolFrame(right_task_sub_1, {0, 0, -0.022, 0, 0, 0}, 0.5, 0.5, true);
        taskPushBack_delay_1ms(right_task_sub_1, 60);

        taskPushBack_csMove2(right_task_sub_1, {0, -0.300, 0, 0, 0, 0}, 0.5, 0.5, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_1, 60);

        /// (sub right home pose)
        taskPushBack_jsMove2(right_task_sub_1, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 40.0, 40.0, false);
        taskPushBack_delay_1ms(right_task_sub_1, 60);
    }


    return right_task_sub_1;
}

/// Sub Task 2: 키코드 파지 및 체결
std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask2Task() {
    std::vector<UnitTask> right_task_sub_2;

    right_task_sub_2.insert(right_task_sub_2.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    taskPushBack_delay_1ms(right_task_sub_2, 3000);

    //////////////// lid cap screwing FOR Key Code 위치 촬영 (key code 결합에 lid cap screwing 자세 사용) ////////////////
    taskPushBack_jsMove2(right_task_sub_2, {31.644, -11.474, 99.474, 33.409, 73.069, -153.435}, 30.0, 30.0, false);
    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_2, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_2, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_2, "right_drum_lid_cap_screwing"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_2, 600);
    taskPushBack_taskMatchingTargetObject(right_task_sub_2, "right_drum_lid_cap_screwing", false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_2, 1000);

    /// 스캔 전 경유자세 (sub right home pose)
    taskPushBack_jsMove2(right_task_sub_2, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_2, 60);

    taskPushBack_csMove2(right_task_sub_2, {0, 0.300, 0, 0, 0, 0}, 0.5, 0.5, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(right_task_sub_2, 60);


    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_2, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_2, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_2, "right_drum_holder_with_key_code"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_2, 600);
    taskPushBack_taskMatchingTargetObject(right_task_sub_2, "right_drum_holder_with_key_code", false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_2, 1000);

    ///// scanning (250925대영님 scan js pose 추가 필요. 우선 임의 값 넣어둠)
    ///// scanning (250925대영님 scan js pose 추가 필요. 우선 임의 값 넣어둠)
    ///// scanning (250925대영님 scan js pose 추가 필요. 우선 임의 값 넣어둠)
    //////////////// Key Ring 위치 촬영 ////////////////
    taskPushBack_jsMove2(right_task_sub_2, {65.233, 23.014, 71.612, 25.341, 46.000, -124.438}, 30.0, 30.0, false);

    taskPushBack_task3DScanningTargetObject(right_task_sub_2, "right_drum_key_code"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_2, 600);

    taskPushBack_drflSetTCP(right_task_sub_2, "tcp08"); // TCP for drum_lid_cap_screwing
    taskPushBack_delay_1ms(right_task_sub_2, 1000);

    taskPushBack_doGrasping_sharedTask(right_task_sub_2, 0.05, 0.05, "default", "right_drum_holder_with_key_code"); // Move to grasping pose, (acc, vel)
    taskPushBack_delay_1ms(right_task_sub_2, 60);

    ///// /hanyang/coupler/keycode_holder_angle 토픽의 z 값 받아서 Tool Rz 회전 (전용 태스크)
    taskPushBack_csMoveToolFrameKeycode_for_holder(right_task_sub_2, 0.05, 0.05);
    taskPushBack_delay_1ms(right_task_sub_2, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_2, {0, 0, 0.084, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_2, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_2, {-0.0015 , 0, 0, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_2, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_2, {0, 0.0005, 0, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_2, 60);

    taskPushBackKORASGripperCmd(right_task_sub_2, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_2, 60);
    taskPushBackKORASGripperCmd(right_task_sub_2, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -9500);
    taskPushBack_delay_1ms(right_task_sub_2, 3000);
    taskPushBackKORASGripperCmd(right_task_sub_2, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_2, 60);
    taskPushBackKORASGripperCmd(right_task_sub_2, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -1500);
    taskPushBack_delay_1ms(right_task_sub_2, 3000);

    taskPushBack_csMoveToolFrame(right_task_sub_2, {0, 0, -0.048, 0, 0, 0}, 0.03, 0.015, true);

    taskPushBack_csMove2(right_task_sub_2, {-0.1, -0.450, -0.075, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative

    ///// Key Ring 위치 촬영 자세 이동
    taskPushBack_jsMove2(right_task_sub_2, {56.655, 16.627, 72.931, 0.241, 90.276, -123.864}, 30.0, 30.0, false);
    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_2, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_2, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_2, "right_drum_key_code"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_2, 600);
    taskPushBack_drflSetTCP(right_task_sub_2, "tcp08"); // TCP for drum_lid_cap_screwing
    taskPushBack_delay_1ms(right_task_sub_2, 1000);


    taskPushBack_doGrasping_sharedTask(right_task_sub_2, 0.05, 0.05, "default", "right_drum_lid_cap_screwing"); // Move to grasping pose, (acc, vel)
    taskPushBack_delay_1ms(right_task_sub_2, 60);

    ///// /hanyang/coupler/keycode_angle 토픽의 z 값 받아서 Tool Rz 회전 (전용 태스크)
    taskPushBack_csMoveToolFrameKeycode(right_task_sub_2, 0.02, 0.02);
    taskPushBack_delay_1ms(right_task_sub_2, 60);


    taskPushBack_csMoveToolFrame(right_task_sub_2, {0.001, 0, 0, 0, 0, 0}, 0.05, 0.05, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_2, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_2, {0, 0, 0.045, 0, 0, 0}, 0.05, 0.05, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_2, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_2, {0, 0, 0.002, 0, 0, 0}, 0.025, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_2, 3000);
    taskPushBack_csMoveToolFrame(right_task_sub_2, {0, 0, 0.002, 0, 0, 0}, 0.025, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_2, 3000);

    taskPushBack_csMoveToolFrame(right_task_sub_2, {0, 0, 0, 0, 0, 20}, 0.005, 0.005, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_2, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_2, {0, 0, 0, 0, 0, -25}, 0.005, 0.005, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_2, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_2, {0, 0, 0, 0, 0, 5}, 0.005, 0.005, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_2, 60);

    taskPushBackKORASGripperCmd(right_task_sub_2, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_2, 60);
    taskPushBackKORASGripperCmd(right_task_sub_2, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +4000);
    taskPushBack_delay_1ms(right_task_sub_2, 3000);

    taskPushBack_csMove2(right_task_sub_2, {0, 0, 0.100, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(right_task_sub_2, 1000);

    taskPushBack_csMove2(right_task_sub_2, {0, -0.300, 0, 0, 0, 0}, 0.5, 0.5, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(right_task_sub_2, 2000);


    // 필요 시 홈 복귀
    taskPushBack_jsMove2(right_task_sub_2, right_home_pose_, 90.0, 90.0, false);

    return right_task_sub_2;
}

/// Sub Task 3: 호스 파지 및 체결
std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask3Task() {
    std::vector<UnitTask> right_task_sub_3;

    taskPushBack_jsMove2(right_task_sub_3, right_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_3, 60);

    right_task_sub_3.insert(right_task_sub_3.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_3, 4000);

    ////////////////////////////////////////////////////////////////////////////////////
    //// 파지 시에, 팁이 걸릴 경우 아래 값으로 조절: 현재: -3000
    taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_3, 60);
    taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000);
    taskPushBack_delay_1ms(right_task_sub_3, 500);
    ////////////////////////////////////////////////////////////////////////////////////

    if(1) {
        ///// hole scan pose
        JsDouble scan_pose_target = {31.644, -11.474, 99.474, 33.409, 73.069, -153.435};
        taskPushBack_jsMove2(right_task_sub_3, scan_pose_target, 30.0, 30.0, false);

        //// Scanning
        taskPushBack_drflSetTCP(right_task_sub_3, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_3, 60);
        taskPushBack_task3DScanningTargetObject(right_task_sub_3, "right_drum_hole_surface"); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_3, 600);
        taskPushBack_taskMatchingTargetObject(right_task_sub_3, "right_drum_hole_surface", false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_3, 1000);

        taskPushBack_drflSetTCP(right_task_sub_3, "tcp03"); // TCP for drum_lid_cap_screwing
        taskPushBack_delay_1ms(right_task_sub_3, 1000);

    }

    if(1) {
        //// right_holder_with_coupler
        JsDouble scan_pose_target = {4.942, -7.319, 86.477, 39.470, 80.375, -213.474};

        taskPushBack_jsMove2(right_task_sub_3, scan_pose_target, 30.0, 30.0, false);
        taskPushBack_delay_1ms(right_task_sub_3, 500);

        //// Scanning
        taskPushBack_drflSetTCP(right_task_sub_3, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_3, 60);
        taskPushBack_task3DScanningTargetObject(right_task_sub_3, "right_holder_with_coupler"); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_3, 600);
        taskPushBack_taskMatchingTargetObject(right_task_sub_3, "right_holder_with_coupler", false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_3, 1000);

        taskPushBack_drflSetTCP(right_task_sub_3, "tcp03"); // TCP for drum_lid_cap_screwing
        taskPushBack_delay_1ms(right_task_sub_3, 1000);

        /////////////////////// 인식 자세 이동
        taskPushBack_doGrasping_sharedTask(right_task_sub_3, 0.05, 0.05, "drum_task", "right_holder_with_coupler"); // Move to grasping pose, (acc, vel)
        // taskPushBack_delay_1ms(right_task_sub_3, 60);

        //// 상대 모션, 파지 중에 걸릴 경우, 아래를 수정
        taskPushBack_csMoveToolFrame(right_task_sub_3, {0.050, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_3, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_3, {0, -0.002, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_3, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0, 0.137, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_3, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0, 0, 0, 3, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_3, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_3, {0.040, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_3, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_3, {0.060, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_3, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0, 0, 0, 1.8, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_3, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_3, {0.017, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_3, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0, 0, 0, -1.8, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_3, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0, 0.006, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_3, 60);

        right_task_sub_3.insert(right_task_sub_3.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
        taskPushBack_delay_1ms(right_task_sub_3, 4000);

        taskPushBack_csMoveToolFrame(right_task_sub_3, {-0.030, 0, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
        taskPushBack_csMoveToolFrame(right_task_sub_3, {-0.070, 0, 0, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative

    }

    taskPushBack_csMove2(right_task_sub_3, {-0.200, -0.200, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative

    /// 스캔 전 경유자세 (sub right home pose)
    taskPushBack_jsMove2(right_task_sub_3, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_3, 60);


    /////////////////////// 인식 자세 이동
    taskPushBack_doGrasping_sharedTask(right_task_sub_3, 0.05, 0.05, "drum_task", "right_drum_hole_surface"); // Move to grasping pose, (acc, vel)

    taskPushBack_csMoveToolFrame(right_task_sub_3, {0.050, -0.007, 0.007, 0, 0, 0}, 0.1, 0.1, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_3, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0, 0, 0, -3, -1}, 0.1, 0.1, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_3, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_3, {0.030, 0.004, -0.006, 0,0, 0}, 0.1, 0.1, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_3, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_3, {0.047 , 0, 0, 0, 0, 0}, 0.01, 0.01, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_3, 100);

    right_task_sub_3.insert(right_task_sub_3.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_3, 3000);

    taskPushBack_csMoveToolFrame(right_task_sub_3, {0.005, 0, -0.002, 0, 0, 0}, 0.005, 0.0025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_3, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0, 0, 0, 3.9, 0}, 0.005, 0.0025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_3, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_3, {0.003, 0, 0, 0, 0, 0}, 0.005, 0.0025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_3, 60);



    right_task_sub_3.insert(right_task_sub_3.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
    taskPushBack_delay_1ms(right_task_sub_3, 60);


    // 케미컬 호스 체결 작업 (총 0.009 가량 down)
    auto subTaskScrewingChemicalHose = [&] (std::vector<UnitTask>& task_in, int trial_cnt) {

        task_in.insert(task_in.end(), drum_grp_screwing_pose_task_.begin(), drum_grp_screwing_pose_task_.end());
        // ROS_LOG_WARN("Chemical Hose Screwing - Trial: %d / 60", trial_cnt + 1);

    };

    // for (int k = 0; k < 14; k++) { // 너무 과도하게 조임
    // k 12 에 k== 9 가 현재 최적 (0331 1110am)
    // for (int k = 0; k < 10; k++) { // 기존
    // k 9 , k< 7 : 수요일 요청 사항
    for (int k = 0; k < 9; k++) { // 수요일 요청사항

        if(k < 12) { // 기존
            // if (k % 2 == 0) {
                // double z_scale_down = 0.000266667;         ///// 총 횟수%2
                double z_scale_down = 0.000266667;         ///// 총 횟수%2
                taskPushBack_csMoveToolFrame(right_task_sub_3, {z_scale_down, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // z축 이동
            // }
        }

        // if(k == 9) {
        //     taskPushBack_drflSetImpedance(right_task_sub_3, true); // Compliance CTRL ON
        //     taskPushBack_delay_1ms(right_task_sub_3, 600);
        // }
        subTaskScrewingChemicalHose(right_task_sub_3, k);
    }

    ///////////////////////////////////////////////////////////////////////////////
    //// 기존 코드 (251015 테스트 전)
    // right_task_sub_3.insert(right_task_sub_3.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    // taskPushBack_delay_1ms(right_task_sub_3, 3500);

    // taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::POS_RESET);
    // taskPushBack_delay_1ms(right_task_sub_3, 60);
    // taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -9500);
    // taskPushBack_delay_1ms(right_task_sub_3, 3500);

    // taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0, -0.003, 0, 0, 0}, 0.01, 0.005, true);
    // // taskPushBack_delay_1ms(right_task_sub_3, 60);
    // taskPushBack_csMoveToolFrame(right_task_sub_3, {-0.02, 0, 0, 0, 0, 0}, 0.01, 0.005, true);
    // // taskPushBack_delay_1ms(right_task_sub_3, 60);
    // // taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0, -0.001, 0, 0, 0}, 0.01, 0.005, true);
    // // taskPushBack_delay_1ms(right_task_sub_3, 60);
    // taskPushBack_csMoveToolFrame(right_task_sub_3, {-0.11, 0, 0, 0, 0, 0}, 0.1, 0.1, true);
    // // taskPushBack_delay_1ms(right_task_sub_3, 20);
    // taskPushBack_csMoveToolFrame(right_task_sub_3, {-0.1, 0, 0, 0, 0, 0}, 0.1, 0.1, true);
    // // taskPushBack_delay_1ms(right_task_sub_3, 20);

    // taskPushBack_csMove2(right_task_sub_3, {0, -0.2, 0, 0, 0, 0}, 0.05, 0.05, true); // false: absolute, true: relative

    //////// -0.11 테스트 후 잘 안되면 -0.096 으로 수정 ///////
    ///////////////////////////////////////////////////////////////////////////////
    //// 방안1 // 추가 티칭 후 성공
    // right_task_sub_3.insert(right_task_sub_3.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    // taskPushBack_delay_1ms(right_task_sub_3, 3500);

    // taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::POS_RESET);
    // taskPushBack_delay_1ms(right_task_sub_3, 60);
    // taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -9500);
    // taskPushBack_delay_1ms(right_task_sub_3, 3500);

    taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_3, 60);
    taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +2000);
    taskPushBack_delay_1ms(right_task_sub_3, 3500);

    taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0, -0.003, 0, 0, 0}, 0.01, 0.005, true);
    // taskPushBack_delay_1ms(right_task_sub_3, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_3, {-0.02, 0, 0, 0, 0, 0}, 0.01, 0.005, true);
    taskPushBack_delay_1ms(right_task_sub_3, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0, +0.001, 0, 0, 0}, 0.01, 0.005, true);
    taskPushBack_delay_1ms(right_task_sub_3, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_3, {-0.110, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_3, 20);
    taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0.004, 0, 0, 0, 0}, 0.1, 0.1, true);
    taskPushBack_delay_1ms(right_task_sub_3, 20);

    // taskPushBack_delay_1ms(right_task_sub_3, 20);
    taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_3, 60);
    taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -5000);
    taskPushBack_delay_1ms(right_task_sub_3, 3500);

    taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0, -0.2, 0, 0, 0}, 0.05, 0.05, true); // false: absolute, true: relative

    taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_3, 60);
    taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +5000);
    taskPushBack_delay_1ms(right_task_sub_3, 3500);

    ///////////////////////////////////////////////////////////////////////////////
    //// 방안2
    // taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::POS_RESET);
    // taskPushBack_delay_1ms(right_task_sub_3, 60);
    // taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +4000);
    // taskPushBack_delay_1ms(right_task_sub_3, 3500);

    // taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0, -0.003, 0, 0, 0}, 0.01, 0.005, true);
    // taskPushBack_delay_1ms(right_task_sub_3, 60);
    
    // taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::POS_RESET);
    // taskPushBack_delay_1ms(right_task_sub_3, 60);
    // taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +6000);
    // taskPushBack_delay_1ms(right_task_sub_3, 3500);


    // taskPushBack_csMoveToolFrame(right_task_sub_3, {-0.02, 0, 0, 0, 0, 0}, 0.01, 0.005, true);
    // // taskPushBack_delay_1ms(right_task_sub_3, 60);
    // // taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0, -0.001, 0, 0, 0}, 0.01, 0.005, true);
    // // taskPushBack_delay_1ms(right_task_sub_3, 60);
    // taskPushBack_csMoveToolFrame(right_task_sub_3, {-0.11, 0, 0, 0, 0, 0}, 0.1, 0.1, true);
    // // taskPushBack_delay_1ms(right_task_sub_3, 20);
    // taskPushBack_csMoveToolFrame(right_task_sub_3, {-0.1, 0, 0, 0, 0, 0}, 0.1, 0.1, true);
    // taskPushBack_csMove2(right_task_sub_3, {0, -0.2, 0, 0, 0, 0}, 0.05, 0.05, true); // false: absolute, true: relative
    // taskPushBack_delay_1ms(right_task_sub_3, 200);

    ///////////////////////////////////////////////////////////////////////////////
    //// 방안 1 + 방안2
    // taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::POS_RESET);
    // taskPushBack_delay_1ms(right_task_sub_3, 60);
    // taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +4000);
    // taskPushBack_delay_1ms(right_task_sub_3, 3500);

    // taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0, -0.003, 0, 0, 0}, 0.01, 0.005, true);
    // taskPushBack_delay_1ms(right_task_sub_3, 60);
    
    // taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::POS_RESET);
    // taskPushBack_delay_1ms(right_task_sub_3, 60);
    // taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +6000);
    // taskPushBack_delay_1ms(right_task_sub_3, 3500);

    // taskPushBack_csMoveToolFrame(right_task_sub_3, {-0.02, 0, 0, 0, 0, 0}, 0.01, 0.005, true);
    // taskPushBack_csMoveToolFrame(right_task_sub_3, {-0.11, 0, 0, 0, 0, 0}, 0.1, 0.1, true);
    // taskPushBack_delay_1ms(right_task_sub_3, 20);
    // taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::POS_RESET);
    // taskPushBack_delay_1ms(right_task_sub_3, 60);
    // taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -5000);
    // taskPushBack_delay_1ms(right_task_sub_3, 3500);

    // taskPushBack_csMoveToolFrame(right_task_sub_3, {0, 0, -0.2, 0, 0, 0}, 0.05, 0.05, true); // false: absolute, true: relative


    // taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::POS_RESET);
    // taskPushBack_delay_1ms(right_task_sub_3, 60);
    // taskPushBackKORASGripperCmd(right_task_sub_3, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +5000);
    // taskPushBack_delay_1ms(right_task_sub_3, 3500);




    /// 스캔 전 경유자세 (sub right home pose)
    taskPushBack_jsMove2(right_task_sub_3, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_3, 60);

    taskPushBack_jsMove2(right_task_sub_3, right_home_pose_, 70.0, 70.0, false);

    return right_task_sub_3;
}

/// Sub Task 4: 호스 체결 해제 /// 
std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask4Task() {
    std::vector<UnitTask> right_task_sub_4;

    taskPushBack_jsMove2(right_task_sub_4, right_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_4, 60);

    right_task_sub_4.insert(right_task_sub_4.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_4, 4000);

    ////////////////////////////////////////////////////////////////////////////////////
    //// 파지 시에, 팁이 걸릴 경우 아래 값으로 조절: 현재: -3000
    taskPushBackKORASGripperCmd(right_task_sub_4, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_4, 60);
    taskPushBackKORASGripperCmd(right_task_sub_4, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000);
    taskPushBack_delay_1ms(right_task_sub_4, 500);
    ////////////////////////////////////////////////////////////////////////////////////

    /// holder without coupler 촬영

    JsDouble scan_pose_target = {4.942, -7.319, 86.477, 39.470, 80.375, -213.474};

    taskPushBack_jsMove2(right_task_sub_4, scan_pose_target, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_4, 500);

    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_4, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_4, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_4, "right_holder_without_coupler"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_4, 600);
    taskPushBack_taskMatchingTargetObject(right_task_sub_4, "right_holder_without_coupler", false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_4, 1000);

    /// unscrewing coupler 촬영
    if(1) {
        //// right_holder_with_coupler
        JsDouble scan_pose_target = {31.644, -11.474, 99.474, 33.409, 73.069, -153.435};

        taskPushBack_jsMove2(right_task_sub_4, scan_pose_target, 30.0, 30.0, false);
        taskPushBack_delay_1ms(right_task_sub_4, 500);

        //// Scanning
        taskPushBack_drflSetTCP(right_task_sub_4, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_4, 60);
        taskPushBack_task3DScanningTargetObject(right_task_sub_4, "right_drum_coupler_unscrewing"); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_4, 600);
        taskPushBack_taskMatchingTargetObject(right_task_sub_4, "right_drum_coupler_unscrewing", false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_4, 1000);

        taskPushBack_drflSetTCP(right_task_sub_4, "tcp03"); // TCP for drum_lid_cap_screwing
        taskPushBack_delay_1ms(right_task_sub_4, 1000);

        ///// 싱귤러 대비 경유 자세
        taskPushBack_jsMove2(right_task_sub_4, {72.382, -13.048, 121.957, 64.492, -20.119, -154.690}, 30.0, 30.0, false);
        taskPushBack_delay_1ms(right_task_sub_4, 60);

        /////////////////////// 인식 자세 이동
        taskPushBack_doGrasping_sharedTask(right_task_sub_4, 0.05, 0.05, "drum_task", "right_drum_coupler_unscrewing"); // Move to grasping pose, (acc, vel)
        //taskPushBack_delay_1ms(right_task_sub_4, 60);
    
        //// 상대 모션, 파지 중에 걸릴 경우, 아래를 수정
        taskPushBack_csMoveToolFrame(right_task_sub_4, {0.034, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_4, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_4, {0, -0.003, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_4, 60);


        taskPushBackKORASGripperCmd(right_task_sub_4, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(right_task_sub_4, 60);
        taskPushBackKORASGripperCmd(right_task_sub_4, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -5000);
        taskPushBack_delay_1ms(right_task_sub_4, 2000);

        taskPushBack_csMoveToolFrame(right_task_sub_4, {0, 0, 0.142, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_4, 60);

        taskPushBackKORASGripperCmd(right_task_sub_4, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(right_task_sub_4, 60);
        taskPushBackKORASGripperCmd(right_task_sub_4, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 5000);
        taskPushBack_delay_1ms(right_task_sub_4, 2000);

        taskPushBack_csMoveToolFrame(right_task_sub_4, {0, 0, 0, 0, 2, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_4, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_4, {0.060, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_4, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_4, {0, 0, 0, 0, 1, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_4, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_4, {0, 0, -0.002, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_4, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_4, {0.071, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_4, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_4, {0, 0, 0, 0, -2, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_4, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_4, {0, 0, 0.0025, 0, 0, 0}, 0.025, 0.025, true);
        taskPushBack_delay_1ms(right_task_sub_4, 60);


        auto subTaskUnScrewingChemicalHose = [&] (std::vector<UnitTask>& task_in, int trial_cnt) {
    
            task_in.insert(task_in.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    
        };
    
        taskPushBack_drflSetImpedance(right_task_sub_4, true); // Compliance CTRL OFF
        taskPushBack_delay_1ms(right_task_sub_4, 600);
        /////////////////////////////////////////////////////////////////////////////////
        ///// 현재 k = 13 , k > 11 최적
        for (int k = 0; k < 11; k++) {
    
            // if(k > 11) {
            //     // if (k % 2 == 0) {
            //         // double z_scale_down = 0.000266667;         ///// 총 횟수%2
            //         double z_scale_down = 0.000266667;         ///// 총 횟수%2
            //         taskPushBack_csMoveToolFrame(right_task_sub_4, {-z_scale_down, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // z축 이동
            //     // }
            // }
    
            subTaskUnScrewingChemicalHose(right_task_sub_4, k);
        }
    
        /////////////////////////////////////////////////////////////////////////////////
        taskPushBack_drflSetImpedance(right_task_sub_4, false); // Compliance CTRL OFF
        taskPushBack_delay_1ms(right_task_sub_4, 600);
        /////////////////////////////////////////////////////////////////////////////////
    
        right_task_sub_4.insert(right_task_sub_4.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
        taskPushBack_delay_1ms(right_task_sub_4, 100);
    
        taskPushBack_csMoveToolFrame(right_task_sub_4, {-0.002, 0, 0, 0, 0, 0}, 0.05, 0.025, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_4, 50);
    
        taskPushBack_csMoveToolFrame(right_task_sub_4, {0, 0, -0.0005, 0, -1, 0}, 0.01, 0.005, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_4, 60);
    
        right_task_sub_4.insert(right_task_sub_4.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
        taskPushBack_delay_1ms(right_task_sub_4, 100);
        
        taskPushBack_csMove2(right_task_sub_4, {0, 0, 0.05, 0, 0, 0}, 0.02, 0.01, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_4, 50);
        taskPushBack_csMove2(right_task_sub_4, {0, 0, 0.100, 0, 0, 0}, 0.7, 0.7, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_4, 50);

        ///// holder without coupler 진입 전 경유 자세 
        // 호스가 낭창거려 임의로 주석처리, 호스 고정시 복원
        // taskPushBack_jsMove2(right_task_sub_4, {33.288, -12.467, 122.337, -75.089, -53.057, -26.890}, 30.0, 30.0, false);
        // taskPushBack_delay_1ms(right_task_sub_4, 500);
        taskPushBack_jsMove2(right_task_sub_4, {76.454, -19.965, 132.023, 58.685, -35.616, -144.755}, 30.0, 30.0, false);
        taskPushBack_delay_1ms(right_task_sub_4, 500);

    }

    /////////////////////// 인식 자세 이동
    taskPushBack_doGrasping_sharedTask(right_task_sub_4, 0.05, 0.05, "drum_task", "right_holder_without_coupler"); // Move to grasping pose, (acc, vel)
    // taskPushBack_delay_1ms(right_task_sub_4, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_4, {0.030, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_4, 100);


    taskPushBack_csMoveToolFrame(right_task_sub_4, {0.020, -0.007, 0.012, 0, 0, 0}, 0.1, 0.05, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_4, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_4, {0, 0, 0, 0, -1, -1}, 0.05, 0.05, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_4, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_4, {0.030, 0.004, -0.006, 0,0, 0}, 0.5, 0.5, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_4, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_4, {0.013, 0.005, 0, 0, 0, 0}, 0.05, 0.05, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_4, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_4, {0.024, 0, 0, 0, 0, 0}, 0.05, 0.05, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_4, 100);

    // taskPushBack_csMoveToolFrame(right_task_sub_4, {0.030, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // true: relative
    // taskPushBack_delay_1ms(right_task_sub_4, 100);
    // taskPushBack_csMoveToolFrame(right_task_sub_4, {0.009, 0, 0, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    // taskPushBack_delay_1ms(right_task_sub_4, 100);
    
    // taskPushBack_csMoveToolFrame(right_task_sub_4, {0, 0, -0.0005, 0, -0.5, 0}, 0.05, 0.05, true); // true: relative
    // taskPushBack_delay_1ms(right_task_sub_4, 100);

    right_task_sub_4.insert(right_task_sub_4.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_4, 3000);

    taskPushBackKORASGripperCmd(right_task_sub_4, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_4, 60);
    taskPushBackKORASGripperCmd(right_task_sub_4, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -4000);
    taskPushBack_delay_1ms(right_task_sub_4, 500);

    taskPushBack_csMoveToolFrame(right_task_sub_4, {0, 0, 0, 0, 0.5, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_4, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_4, {0, 0.006, 0, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_4, 100);
    taskPushBack_csMoveToolFrame(right_task_sub_4, {0, 0, -0.001, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_4, 100);
    taskPushBack_csMoveToolFrame(right_task_sub_4, {0, 0, 0, 0, 3, 0}, 0.025, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_4, 100);
    taskPushBack_csMoveToolFrame(right_task_sub_4, {-0.030, 0, 0, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_4, 100);
    taskPushBack_csMoveToolFrame(right_task_sub_4, {0, 0, -0.001, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_4, 100);
    taskPushBack_csMoveToolFrame(right_task_sub_4, {-0.070, 0, 0, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_4, 100);
    taskPushBack_csMoveToolFrame(right_task_sub_4, {0, 0, -0.100, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_4, 100);
            
    taskPushBack_csMove2(right_task_sub_4, {-0.250, 0, 0, 0, 0, 0}, 0.2, 0.2, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_4, 50);

    /// 스캔 전 경유자세 (right home pose)
    taskPushBack_jsMove2(right_task_sub_4, right_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_4, 60);

    return right_task_sub_4;
}

/// Sub Task 5: Key Code 해제 및 거치
std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask5Task() {
    std::vector<UnitTask> right_task_sub_5;

    taskPushBack_jsMove2(right_task_sub_5, right_home_pose_, 30.0, 30.0, false);

    right_task_sub_5.insert(right_task_sub_5.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    taskPushBack_delay_1ms(right_task_sub_5, 3000);

    //////////////// lid cap screwing FOR Key Code 위치 촬영 (key code 결합에 lid cap screwing 자세 사용) ////////////////
    taskPushBack_jsMove2(right_task_sub_5, {31.644, -11.474, 99.474, 33.409, 73.069, -153.435}, 60.0, 60.0, false);
    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_5, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_5, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_5, "right_drum_key_code_unscrewing"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_5, 600);
    taskPushBack_taskMatchingTargetObject(right_task_sub_5, "right_drum_key_code_unscrewing", false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_5, 1000);

    ///////Key Ring 촬영 전 경유 자세
    taskPushBack_jsMove2(right_task_sub_5, {37.715, -4.753, 101.891, 0.115, 82.740, -142.838}, 30.0, 30.0, false);
    taskPushBack_csMove2(right_task_sub_5, {0, 0.3, 0, 0, 0, 0}, 0.5, 0.5, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(right_task_sub_5, 50);

    //////////////// Key Ring 위치 촬영 ////////////////
    JsDouble scan_pose = {56.655, 16.627, 72.931, 0.241, 90.276, -123.864};
    // taskPushBack_jsMove2(right_task_sub_5, scan_pose, 30.0, 30.0, false);
    // taskPushBack_delay_1ms(right_task_sub_5, 1000);

    //// Scanning
    // taskPushBack_drflSetTCP(right_task_sub_5, "tcp00"); // Scan & Matching은 tcp00
    // taskPushBack_delay_1ms(right_task_sub_5, 60);
    // taskPushBack_task3DScanningTargetObject(right_task_sub_5, "right_drum_key_code"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_5, 60);
    taskPushBack_drflSetTCP(right_task_sub_5, "tcp08"); // TCP for drum_lid_cap_screwing
    taskPushBack_delay_1ms(right_task_sub_5, 1000);

    taskPushBack_doGrasping_sharedTask(right_task_sub_5, 0.05, 0.05, "default", "right_drum_key_code_unscrewing"); // Move to grasping pose, (acc, vel)
    taskPushBack_delay_1ms(right_task_sub_5, 60);

    // taskPushBack_csMoveToolFrameKeycode(right_task_sub_5, 0.02, 0.02);
    // taskPushBack_delay_1ms(right_task_sub_5, 60);

    // 여기 티칭값 추가
    // 여기 티칭값 추가
    // taskPushBack_csMoveToolFrame(right_task_sub_5, {0, 0, 0, 0, 0, 173}, 0.05, 0.05, true);
    // taskPushBack_delay_1ms(right_task_sub_5, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_5, {0, 0, 0, 0, 0, -127}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_5, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_5, {0, 0, 0, 0, 0, -43}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_5, 60);
    

    taskPushBack_csMoveToolFrame(right_task_sub_5, {0, 0, 0.055, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_5, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_5, {0, -0.0025, 0, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_5, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_5, {0, 0, 0.007, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_5, 60);

    taskPushBackKORASGripperCmd(right_task_sub_5, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_5, 60);
    taskPushBackKORASGripperCmd(right_task_sub_5, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -9500);
    taskPushBack_delay_1ms(right_task_sub_5, 3000);

    taskPushBack_csMoveToolFrame(right_task_sub_5, {0, 0, -0.100, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_5, 60);

    //// key code holder 진입 전 경유자세
    taskPushBack_jsMove2(right_task_sub_5, scan_pose, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_5, 1000);

    //// key code holder 진입 전 경유자세
    taskPushBack_jsMove2(right_task_sub_5, {55.312, 8.986, 100.241, 75.696, 33.058, -178.847}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_5, 1000);

    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_5, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_5, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_5, "right_drum_holder_without_key_code"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_5, 600);
    taskPushBack_taskMatchingTargetObject(right_task_sub_5, "right_drum_holder_without_key_code", false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_5, 1000);


    taskPushBack_drflSetTCP(right_task_sub_5, "tcp08"); // TCP for drum_lid_cap_screwing
    taskPushBack_delay_1ms(right_task_sub_5, 1000);
    //// 파지
    taskPushBack_doGrasping_sharedTask(right_task_sub_5, 0.05, 0.05, "default", "right_drum_holder_without_key_code"); // Move to grasping pose, (acc, vel)
    taskPushBack_delay_1ms(right_task_sub_5, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_5, {0, 0, 0.060, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_5, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_5, {-0.002, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_5, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_5, {0, 0.002, 0, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_5, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_5, {0, 0, 0.018, 0, 0, 0}, 0.025, 0.025, true);
    taskPushBack_delay_1ms(right_task_sub_5, 60);




    taskPushBackKORASGripperCmd(right_task_sub_5, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_5, 60);
    taskPushBackKORASGripperCmd(right_task_sub_5, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +4000);
    taskPushBack_delay_1ms(right_task_sub_5, 3000);

    //// 빠져나오기
    taskPushBack_csMoveToolFrame(right_task_sub_5, {0, 0, -0.1, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_5, 60);

    //// key code holder 진입 전 경유자세 사용
    taskPushBack_jsMove2(right_task_sub_5, {55.312, 8.986, 100.241, 75.696, 33.058, -178.847}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_5, 1000);

    taskPushBack_csMove2(right_task_sub_5, {0, -0.3, 0, 0, 0, 0}, 0.05, 0.05, true); // false: absolute, true: relative
    taskPushBack_csMove2(right_task_sub_5, {0, -0.2, 0, 0, 0, 0}, 0.05, 0.05, true); // false: absolute, true: relative


    taskPushBack_jsMove2(right_task_sub_5, right_home_pose_, 30.0, 30.0, false);

    return right_task_sub_5;
}

/// Sub Task 6: 별 모양 뚜껑 원위치
std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask6Task() {
    std::vector<UnitTask> right_task_sub_6;

    taskPushBack_jsMove2(right_task_sub_6, right_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_6, 60);

    right_task_sub_6.insert(right_task_sub_6.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_6, 2000);
    right_task_sub_6.insert(right_task_sub_6.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
    taskPushBack_delay_1ms(right_task_sub_6, 1000);

    /// 스캔 전 경유자세 (sub right home pose)
    taskPushBack_jsMove2(right_task_sub_6, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_6, 60);

    taskPushBack_csMove2(right_task_sub_6, {0, 0.300, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(right_task_sub_6, 60);

    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_6, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_6, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_6, "right_holder_with_lid_cap"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_6, 600);
    taskPushBack_taskMatchingTargetObject(right_task_sub_6, "right_holder_with_lid_cap", false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_6, 1000);

    taskPushBack_drflSetTCP(right_task_sub_6, "tcp05"); // TCP for drum_lid_cap_screwing
    taskPushBack_delay_1ms(right_task_sub_6, 1000);

    taskPushBack_doGrasping_sharedTask(right_task_sub_6, 0.05, 0.05, "default", "right_holder_with_lid_cap"); // Move to grasping pose, (acc, vel)
    taskPushBack_delay_1ms(right_task_sub_6, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_6, {0, 0 , 0.070, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_6, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_6, {0, -0.003 , 0, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_6, 60);
    //// 파지
    right_task_sub_6.insert(right_task_sub_6.end(), drum_grp_lid_cap_close_tight.begin(), drum_grp_lid_cap_close_tight.end());
    taskPushBack_delay_1ms(right_task_sub_6, 4000);

    taskPushBackKORASGripperCmd(right_task_sub_6, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_6, 60);
    taskPushBackKORASGripperCmd(right_task_sub_6, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -3500);
    taskPushBack_delay_1ms(right_task_sub_6, 3000);

    taskPushBack_csMoveToolFrame(right_task_sub_6, {0, 0, -0.048, 0, 0, 0}, 0.03, 0.015, true);

    // taskPushBack_setRobotAutoMode(right_task_sub_6);

    taskPushBack_csMove2(right_task_sub_6, {0, -0.450, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative

    //// 경유 자세
    taskPushBack_jsMove2(right_task_sub_6, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);

    if(1){
        /////////////////////////////////////////////////////////////////////////
        std::string target_tcp = "tcp05"; // screwing new gripper tcp
        std::string target_task = "default";
        JsDouble scan_pose_target = {31.644, -11.474, 99.474, 33.409, 73.069, -153.435};

        //// lid cap screwing scan pose
        taskPushBack_jsMove2(right_task_sub_6, scan_pose_target, 30.0, 30.0, false);
        taskPushBack_delay_1ms(right_task_sub_6, 60);

        taskPushBack_drflSetTCP(right_task_sub_6, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_6, 60);

        taskPushBack_task3DScanningTargetObject(right_task_sub_6, "right_drum_lid_cap_screwing"); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_6, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(right_task_sub_6, "right_drum_lid_cap_screwing", false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_6, 1000);

        taskPushBack_drflSetTCP(right_task_sub_6, target_tcp); // new gripper
        taskPushBack_delay_1ms(right_task_sub_6, 60);

        //// 천장 부딪히지 않도록 경유지 설정
        if(1) {
            JsDouble waypoint_1 = {39.638, -8.522, 106.590, -0.193, 82.717, -313.690};
            taskPushBack_jsMove2(right_task_sub_6, waypoint_1, 30.0, 30.0, false);
            // taskPushBack_delay_1ms(right_task_sub_6, 50);
            taskPushBack_csMove2(right_task_sub_6, {0, 0.400, 0, 0, 0, 0}, 0.2, 0.2, true); // false: absolute, true: relative
            // taskPushBack_delay_1ms(right_task_sub_6, 50);
        }

        /////////////////////// 인식 자세 이동
        taskPushBack_doGrasping_sharedTask(right_task_sub_6, 0.1, 0.05, target_task, "right_drum_lid_cap_screwing"); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(right_task_sub_6, 60);

        // // taskPushBack_csMoveToolFrame(right_task_sub_6, {-0.004, 0, 0, 0, 0, 0}, 0.02, 0.04, true); // true: relative
        // taskPushBack_csMoveToolFrame(right_task_sub_6, {0, -0.004, 0, 0, 0, 0}, 0.02, 0.04, true); // true: relative

        // taskPushBack_csMoveToolFrame(right_task_sub_6, {0, 0, 0.026, 0, 0, 0}, 0.1, 0.1, true); // true: relative
        // // taskPushBack_delay_1ms(right_task_sub_6, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_6, {0, 0, 0.025, 0, 0, 0}, 0.025, 0.025, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_6, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_6, {-0.004, 0, 0, 0, 0, 0}, 0.025, 0.025, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_6, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_6, {0, 0.002, 0, 0, 0, 0}, 0.025, 0.025, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_6, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_6, {0, 0, 0.005, 0, 0, 0}, 0.025, 0.025, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_6, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_6, {0, 0, 0.005, 0, 0, 10}, 0.025, 0.025, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_6, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_6, {0, 0, 0.004, 0, 0, 0}, 0.025, 0.025, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_6, 60);


        right_task_sub_6.insert(right_task_sub_6.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(right_task_sub_6, 1000);
        right_task_sub_6.insert(right_task_sub_6.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(right_task_sub_6, 1000);

        taskPushBack_csMoveToolFrame(right_task_sub_6, {0, 0, -0.008, 0, 0, 0}, 0.025, 0.025, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_6, 60);

        taskPushBack_jsMove2(right_task_sub_6, {0, 0, 0, 0, 0, -10}, 15.0, 15.0, true);

        taskPushBack_csMoveToolFrame(right_task_sub_6, {0, 0, 0.012, 0, 0, 0}, 0.02, 0.04, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_6, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_6, {0, 0.004, 0, 0, 0, 0}, 0.02, 0.04, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_6, 60);

        taskPushBack_torquePollingOn(right_task_sub_6);
        taskPushBack_delay_1ms(right_task_sub_6, 2000);

        // 토크 가드 ON: J6 기준 15.0 이상이면 정지 (필요 시 조정)
        taskPushBack_torqueGuardOn(right_task_sub_6, 5, 1.5);

        int cnt_lid_cap_screwing = 6;
        for (int cycle = 0; cycle < cnt_lid_cap_screwing; cycle++) {
            right_task_sub_6.insert(right_task_sub_6.end(), drum_grp_lid_cap_close_for_screwing_.begin(), drum_grp_lid_cap_close_for_screwing_.end());


            if(cycle != cnt_lid_cap_screwing - 1) { // 마지막 trial에서는 스킵
                taskPushBack_delay_1ms(right_task_sub_6, 2000);
                taskPushBack_jsMove2(right_task_sub_6, {0, 0, 0, 0, 0, 195}, 70.0, 70.0, true);
                right_task_sub_6.insert(right_task_sub_6.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
                taskPushBack_delay_1ms(right_task_sub_6, 2000);
            } else {
                //// 마지막 횟수에서 더 돌리기 (90도만)
                taskPushBack_delay_1ms(right_task_sub_6, 2000);
                taskPushBack_jsMove2(right_task_sub_6, {0, 0, 0, 0, 0, 135}, 40.0, 40.0, true);
                // taskPushBack_delay_1ms(right_task_sub_6, 20);
                //// 아래에서 더 돌리면 팁 파손 가능성
                taskPushBack_jsMove2(right_task_sub_6, {0, 0, 0, 0, 0, 15}, 15.0, 15.0, true);
                taskPushBack_jsMove2(right_task_sub_6, {0, 0, 0, 0, 0, 5}, 15.0, 15.0, true);
                // taskPushBack_delay_1ms(right_task_sub_6, 20);
                taskPushBack_jsMove2(right_task_sub_6, {0, 0, 0, 0, 0, -10}, 40.0, 40.0, true);
                right_task_sub_6.insert(right_task_sub_6.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
                taskPushBack_delay_1ms(right_task_sub_6, 2000);
                taskPushBack_jsMove2(right_task_sub_6, {0, 0, 0, 0, 0, -5}, 40.0, 40.0, true);
                // taskPushBack_delay_1ms(right_task_sub_6, 20);
            }

            if(cycle != cnt_lid_cap_screwing - 1) { // 마지막 trial에서는 스킵
                taskPushBack_jsMove2(right_task_sub_6, {0, 0, 0, 0, 0, -15}, 40.0, 40.0, true);
                // taskPushBack_delay_1ms(right_task_sub_6, 20);
                taskPushBack_csMoveToolFrame(right_task_sub_6, {0, 0, -0.050, 0, 0, 0}, 0.2, 0.2, true); // false: absolute, true: relative
                // taskPushBack_delay_1ms(right_task_sub_6, 20);
                taskPushBack_jsMove2(right_task_sub_6, {0, 0, 0, 0, 0, -180}, 40.0, 40.0, true);
                taskPushBack_csMoveToolFrame(right_task_sub_6, {0, 0, 0.050, 0, 0, 0}, 0.2, 0.2, true); // false: absolute, true: relative
                taskPushBack_delay_1ms(right_task_sub_6, 20);
            }
        }

        // === 분기 타겟 라벨 ===
        taskPushBack_label(right_task_sub_6, "SCREW_OPEN_STEP");
        taskPushBack_jsMove2(right_task_sub_6, {0, 0, 0, 0, 0, -20}, 40.0, 40.0, true);
        right_task_sub_6.insert(right_task_sub_6.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
        taskPushBack_delay_1ms(right_task_sub_6, 2000);
        taskPushBack_jsMove2(right_task_sub_6, {0, 0, 0, 0, 0, -5}, 40.0, 40.0, true);

        taskPushBack_csMove2(right_task_sub_6, {0, 0, 0.045, 0, 0, 0}, 0.4, 0.4, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_6, 60);

        // 마지막 자세
        taskPushBack_csMove2(right_task_sub_6, {0, -0.4, 0, 0, 0, 0}, 0.4, 0.4, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_6, 60);

        taskPushBack_jsMove2(right_task_sub_6, right_home_pose_, 70.0, 70.0, false);
        taskPushBack_delay_1ms(right_task_sub_6, 60);


        taskPushBack_torquePollingOff(right_task_sub_6);
        taskPushBack_delay_1ms(right_task_sub_6, 1000);

        // 토크 가드 OFF
        taskPushBack_torqueGuardOff(right_task_sub_6);

    }

    return right_task_sub_6;
}
///////////// Last Task.

/// Sub Task 7
std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask7Task() {
    std::vector<UnitTask> right_task_sub_7;

    taskPushBack_jsMove2(right_task_sub_7, right_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_7, 60);
    
    // right_task_sub_7.insert(right_task_sub_7.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    // taskPushBack_delay_1ms(right_task_sub_7, 4000);

    // ////////////////////////////////////////////////////////////////////////////////////
    // //// 파지 시에, 팁이 걸릴 경우 아래 값으로 조절: 현재: -3000
    // taskPushBackKORASGripperCmd(right_task_sub_7, (uint16_t)KR_GRP::POS_RESET);
    // taskPushBack_delay_1ms(right_task_sub_7, 60);
    // taskPushBackKORASGripperCmd(right_task_sub_7, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000);
    // taskPushBack_delay_1ms(right_task_sub_7, 500);
    // ////////////////////////////////////////////////////////////////////////////////////

    // if(1) { // Detection
    //     //// right_holder_with_coupler
    //     std::string target_object = "right_holder_with_coupler";
    //     std::string target_tcp = "tcp06";
    //     std::string target_task = "drum_task";
    //     // JsDouble scan_pose_target = {5.755, -8.142, 78.586, 32.294, 83.469, -206.805};
    //     JsDouble scan_pose_target = {15.949, -16.089, 96.922, 47.002, 65.014, -217.187};

    //     taskPushBack_jsMove2(right_task_sub_7, scan_pose_target, 120.0, 120.0, false);
    //     taskPushBack_delay_1ms(right_task_sub_7, 500);

    //     //// Scanning
    //     taskPushBack_drflSetTCP(right_task_sub_7, "tcp00"); // Scan & Matching은 tcp00
    //     taskPushBack_delay_1ms(right_task_sub_7, 60);

    //     //// Scanning
    //     taskPushBack_task3DScanningTargetObject(right_task_sub_7, target_object); // ZIVID Scanning with task recognition
    //     taskPushBack_delay_1ms(right_task_sub_7, 600);
    //     //// Matching
    //     taskPushBack_taskMatchingTargetObject(right_task_sub_7, target_object, false, 8); // ZIVID Scanning with task recognition
    //     taskPushBack_delay_1ms(right_task_sub_7, 1000);

    //     taskPushBack_drflSetTCP(right_task_sub_7, target_tcp); // new gripper
    //     taskPushBack_delay_1ms(right_task_sub_7, 60);

    //     ////////////////////// 경유점 JS 1
    //     JsDouble waypoint_1 = {70.522, -14.141, 111.397, -104.781, 31.213, 15.475};
    //     taskPushBack_jsMove2(right_task_sub_7, waypoint_1, 30.0, 30.0, false);
    //     // taskPushBack_delay_1ms(right_task_sub_7, 60);


    //     /////////////////////// 인식 자세 이동
    //     taskPushBack_doGrasping_sharedTask(right_task_sub_7, 0.1, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
    //     // taskPushBack_delay_1ms(right_task_sub_7, 60);
    //     taskPushBack_csMoveToolFrame(right_task_sub_7, {0, 0.003, 0, 0, 0, 0}, 0.05, 0.05, true);
    //     taskPushBack_delay_1ms(right_task_sub_7, 60);
    //     taskPushBack_csMoveToolFrame(right_task_sub_7, {0, 0, 0.120, 0, 0, 0}, 0.05, 0.05, true);
    //     taskPushBack_delay_1ms(right_task_sub_7, 60);

    //     taskPushBack_csMoveToolFrame(right_task_sub_7, {0.020, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
    //     taskPushBack_delay_1ms(right_task_sub_7, 60);


    //     //// 상대 모션, 파지 중에 걸릴 경우, 아래를 수정
    //     taskPushBack_csMoveToolFrame(right_task_sub_7, {0.027, 0, 0, 0, 0, 0}, 0.2, 0.1, true);
    //     taskPushBack_delay_1ms(right_task_sub_7, 60);

    //     taskPushBack_csMoveToolFrame(right_task_sub_7, {0, 0, 0, 0, 3, 0}, 0.02, 0.01, true);
    //     taskPushBack_delay_1ms(right_task_sub_7, 60);

    //     taskPushBack_csMoveToolFrame(right_task_sub_7, {0.020, 0, 0, 0, 0, 0}, 0.2, 0.1, true);
    //     taskPushBack_delay_1ms(right_task_sub_7, 60);

    //     taskPushBack_csMoveToolFrame(right_task_sub_7, {0.06, 0, 0, 0, 0, 0}, 0.04, 0.02, true);
    //     taskPushBack_delay_1ms(right_task_sub_7, 60);
    //     taskPushBack_csMoveToolFrame(right_task_sub_7, {0.021, 0, 0, 0, 0, 0}, 0.04, 0.02, true);
    //     taskPushBack_delay_1ms(right_task_sub_7, 60);

    //     taskPushBack_csMoveToolFrame(right_task_sub_7, {0, 0, 0, 0, -4, 0}, 0.02, 0.01, true);
    //     taskPushBack_delay_1ms(right_task_sub_7, 60);

    //     taskPushBack_csMoveToolFrame(right_task_sub_7, {0, 0, 0.006, 0, 0, 0}, 0.01, 0.005, true);
    //     taskPushBack_delay_1ms(right_task_sub_7, 60);

    //     right_task_sub_7.insert(right_task_sub_7.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
    //     taskPushBack_delay_1ms(right_task_sub_7, 4000);

    //     taskPushBack_csMoveToolFrame(right_task_sub_7, {-0.03, 0, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
    //     taskPushBack_csMoveToolFrame(right_task_sub_7, {-0.15, 0, 0, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative

    // }

    // taskPushBack_csMove2(right_task_sub_7, {-0.1, 0, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative


    // if(1) {
    //     //// drum hole surface
    //     std::string target_object = "right_drum_hole_surface";
    //     std::string target_tcp = "tcp03"; // screwing new gripper tcp
    //     std::string target_task = "drum_task";
    //     JsDouble scan_pose_target = {60.974, -18.933, 104.790, 20.208, 61.469, -124.065};
    //     taskPushBack_jsMove2(right_task_sub_7, scan_pose_target, 50.0, 50.0, false);
    //     taskPushBack_delay_1ms(right_task_sub_7, 500);

    //     taskPushBack_drflSetTCP(right_task_sub_7, "tcp00"); // Scan & Matching은 tcp00
    //     taskPushBack_delay_1ms(right_task_sub_7, 60);

    //     //// Scanning
    //     taskPushBack_task3DScanningTargetObject(right_task_sub_7, target_object); // ZIVID Scanning with task recognition
    //     taskPushBack_delay_1ms(right_task_sub_7, 500);
    //     //// Matching
    //     taskPushBack_taskMatchingTargetObject(right_task_sub_7, target_object, false, 8); // ZIVID Scanning with task recognition
    //     taskPushBack_delay_1ms(right_task_sub_7, 1000);


    //     taskPushBack_drflSetTCP(right_task_sub_7, target_tcp); // new gripper
    //     taskPushBack_delay_1ms(right_task_sub_7, 60);
        
    //     /////////////////////// 인식 자세 이동
    //     taskPushBack_doGrasping_sharedTask(right_task_sub_7, 0.2, 0.2, target_task, target_object); // Move to grasping pose, (acc, vel)
    //     // taskPushBack_delay_1ms(right_task_sub_7, 60);
    // }

    // taskPushBack_csMove2(right_task_sub_7, {0.001, 0, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative

    // taskPushBack_csMoveToolFrame(right_task_sub_7, {0.0, 0, 0, 0, 0, -3}, 0.1, 0.05, true); // true: relative

    // taskPushBack_csMoveToolFrame(right_task_sub_7, {0.0, 0.01, 0, 0, 0, 0}, 0.1, 0.05, true); // true: relative

    // taskPushBack_csMoveToolFrame(right_task_sub_7, {0.080, 0, 0, 0, 0, 0}, 0.2, 0.1, true); // true: relative
    // taskPushBack_csMoveToolFrame(right_task_sub_7, {0.030, 0, 0, 0, 0, 0}, 0.05, 0.05, true); // true: relative


    // // 기울이는 모션
    // taskPushBack_csMoveToolFrame(right_task_sub_7, {0, 0, -0.0014, 0, -2.0, 0}, 0.01, 0.005, true); // false: absolute, true: relative
    // taskPushBack_delay_1ms(right_task_sub_7, 60);

    // taskPushBack_csMoveToolFrame(right_task_sub_7, {0.007, 0, 0, 0, 0, 0}, 0.005, 0.0025, true); // true: relative
    // taskPushBack_delay_1ms(right_task_sub_7, 100);
    // taskPushBack_csMoveToolFrame(right_task_sub_7, {0.005, 0, 0, 0, 0, 0}, 0.005, 0.0025, true); // true: relative
    // taskPushBack_delay_1ms(right_task_sub_7, 100);

    // taskPushBack_csMoveToolFrame(right_task_sub_7, {0.002, 0, 0, 0, 0, 0}, 0.005, 0.0025, true); // true: relative
    // taskPushBack_delay_1ms(right_task_sub_7, 100);
    // taskPushBack_csMoveToolFrame(right_task_sub_7, {0.004, 0, 0, 0, 0, 0}, 0.005, 0.0025, true); // true: relative
    // taskPushBack_delay_1ms(right_task_sub_7, 100);

    // right_task_sub_7.insert(right_task_sub_7.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    // taskPushBack_delay_1ms(right_task_sub_7, 3000);

    // taskPushBack_csMoveToolFrame(right_task_sub_7, {-0.004, 0.0, 0, 0, 0, 0}, 0.02, 0.01, true); // false: absolute, true: relative
    // taskPushBack_delay_1ms(right_task_sub_7, 60);

    // right_task_sub_7.insert(right_task_sub_7.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
    // // taskPushBack_delay_1ms(right_task_sub_7, 2000);


    // // 케미컬 호스 체결 작업 (총 0.009 가량 down)
    // auto subTaskScrewingChemicalHose = [&] (std::vector<UnitTask>& task_in, int trial_cnt) {

    //     task_in.insert(task_in.end(), drum_grp_screwing_pose_task_.begin(), drum_grp_screwing_pose_task_.end());
    //     // ROS_LOG_WARN("Chemical Hose Screwing - Trial: %d / 60", trial_cnt + 1);

    // };

    // // for (int k = 0; k < 14; k++) { // 너무 과도하게 조임
    // // k 12 에 k== 9 가 현재 최적 (0331 1110am)
    // // for (int k = 0; k < 10; k++) { // 기존
    // // k 9 , k< 7 : 수요일 요청 사항
    // for (int k = 0; k < 10; k++) { // 수요일 요청사항

    //     if(k < 17) { // 기존
    //         // if (k % 2 == 0) {
    //             // double z_scale_down = 0.000266667;         ///// 총 횟수%2
    //             double z_scale_down = 0.000266667;         ///// 총 횟수%2
    //             taskPushBack_csMoveToolFrame(right_task_sub_7, {z_scale_down, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // z축 이동
    //         // }
    //     }

    //     // if(k == 9) {
    //     //     taskPushBack_drflSetImpedance(right_task_sub_7, true); // Compliance CTRL ON
    //     //     taskPushBack_delay_1ms(right_task_sub_7, 600);
    //     // }
    //     subTaskScrewingChemicalHose(right_task_sub_7, k);
    // }


    // right_task_sub_7.insert(right_task_sub_7.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    // taskPushBack_delay_1ms(right_task_sub_7, 3500);

    // taskPushBackKORASGripperCmd(right_task_sub_7, (uint16_t)KR_GRP::POS_RESET);
    // taskPushBack_delay_1ms(right_task_sub_7, 60);
    // taskPushBackKORASGripperCmd(right_task_sub_7, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -9500);
    // taskPushBack_delay_1ms(right_task_sub_7, 3500);

    
    // taskPushBack_csMoveToolFrame(right_task_sub_7, {0, 0, -0.003, 0, 0, 0}, 0.01, 0.005, true);
    // // taskPushBack_delay_1ms(right_task_sub_7, 60);
    // taskPushBack_csMoveToolFrame(right_task_sub_7, {-0.02, 0, 0, 0, 0, 0}, 0.01, 0.005, true);
    // // taskPushBack_delay_1ms(right_task_sub_7, 60);
    // // taskPushBack_csMoveToolFrame(right_task_sub_7, {0, 0, -0.001, 0, 0, 0}, 0.01, 0.005, true);
    // // taskPushBack_delay_1ms(right_task_sub_7, 60);
    // taskPushBack_csMoveToolFrame(right_task_sub_7, {-0.11, 0, 0, 0, 0, 0}, 0.1, 0.1, true);
    // // taskPushBack_delay_1ms(right_task_sub_7, 20);
    // taskPushBack_csMoveToolFrame(right_task_sub_7, {-0.1, 0, 0, 0, 0, 0}, 0.1, 0.1, true);
    // // taskPushBack_delay_1ms(right_task_sub_7, 20);

    // taskPushBack_csMove2(right_task_sub_7, {0, -0.2, 0, 0, 0, 0}, 0.05, 0.05, true); // false: absolute, true: relative
    // // taskPushBack_delay_1ms(right_task_sub_7, 200);


    // ///// 호스용 middle home pose (right home pose 로 가면 호스가 당겨짐)
    // taskPushBack_jsMove2(right_task_sub_7, {31.644, -11.474, 99.474, 33.409, 73.069, -153.435}, 30.0, 30.0, false);

    // taskPushBack_torquePollingOn(right_task_sub_7);
    // taskPushBack_delay_1ms(right_task_sub_7, 3000);

    // taskPushBack_torquePollingOff(right_task_sub_7);
    // taskPushBack_delay_1ms(right_task_sub_7, 3000);
    


    return right_task_sub_7;
}

/// Sub Task 8
std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask8Task() {
    std::vector<UnitTask> right_task_sub_8;

    taskPushBack_jsMove2(right_task_sub_8, right_home_pose_, 40.0, 40.0, false);

    // taskPushBack_setRobotAutoMode(right_task_sub_8);

    std::string target_object = "right_drum_coupler_unscrewing";
    std::string target_tcp = "tcp03";
    // std::string target_task = "default"; // 툴 기준 Z 방향으로 approach
    std::string target_task = "drum_task"; // 툴 기준 X 방향으로 approach
    JsDouble right_scan_pose_target = {57.713, -21.621, 102.406, 25.093, 62.506, -127.110};

    right_task_sub_8.insert(right_task_sub_8.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_8, 3500);

    taskPushBackKORASGripperCmd(right_task_sub_8, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_8, 60);
    taskPushBackKORASGripperCmd(right_task_sub_8, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000);
    taskPushBack_delay_1ms(right_task_sub_8, 1500);

    taskPushBack_jsMove2(right_task_sub_8, right_scan_pose_target, 40.0, 40.0, false);
    taskPushBack_delay_1ms(right_task_sub_8, 500);

    taskPushBack_drflSetTCP(right_task_sub_8, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_8, 60);

    taskPushBack_task3DScanningTargetObject(right_task_sub_8, target_object); // ZIVID Scanning with task recognition
    // taskPushBack_delay_1ms(right_task_sub_8, 600);
    //// Matching
    taskPushBack_taskMatchingTargetObject(right_task_sub_8, target_object, false, 8); // "default", true: symmetric CAD used
    // taskPushBack_delay_1ms(right_task_sub_8, 1000);

    taskPushBack_drflSetTCP(right_task_sub_8, target_tcp); // new gripper
    taskPushBack_delay_1ms(right_task_sub_8, 60);


    // taskPushBack_setRobotAutoMode(right_task_sub_8);

    JsDouble waypoint_4 = {54.614, -24.756, 118.745, 95.693, 36.084, -186.462};
    taskPushBack_jsMove2(right_task_sub_8, waypoint_4, 120.0, 120.0, false);
    // taskPushBack_delay_1ms(right_task_sub_8, 60);

    taskPushBack_doGrasping_sharedTask(right_task_sub_8, 0.05, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
    taskPushBack_delay_1ms(right_task_sub_8, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_8, {0, 0, 0.123, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_8, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_8, {0.050, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_8, 60);


    //// 상대 모션, 파지 중에 걸릴 경우, 아래를 수정
    taskPushBack_csMoveToolFrame(right_task_sub_8, {0.027, 0, 0, 0, 0, 0}, 0.2, 0.1, true);
    taskPushBack_delay_1ms(right_task_sub_8, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_8, {0, 0, 0, 0, 3, 0}, 0.04, 0.02, true);
    taskPushBack_delay_1ms(right_task_sub_8, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_8, {0.020, 0, 0, 0, 0, 0}, 0.2, 0.1, true);
    taskPushBack_delay_1ms(right_task_sub_8, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_8, {0.03, 0, 0, 0, 0, 0}, 0.04, 0.02, true);
    taskPushBack_delay_1ms(right_task_sub_8, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_8, {0.023, 0, 0, 0, 0, 0}, 0.04, 0.02, true);
    taskPushBack_delay_1ms(right_task_sub_8, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_8, {0, 0, 0, 0, -4, 0}, 0.02, 0.01, true);
    taskPushBack_delay_1ms(right_task_sub_8, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_8, {0, 0, 0.005, 0, 0, 0}, 0.01, 0.005, true);
    taskPushBack_delay_1ms(right_task_sub_8, 60);


    auto subTaskUnScrewingChemicalHose = [&] (std::vector<UnitTask>& task_in, int trial_cnt) {

        task_in.insert(task_in.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());

    };

    taskPushBack_drflSetImpedance(right_task_sub_8, true); // Compliance CTRL OFF
    taskPushBack_delay_1ms(right_task_sub_8, 600);
    /////////////////////////////////////////////////////////////////////////////////
    ///// 현재 k = 13 , k > 11 최적
    for (int k = 0; k < 11; k++) {

        // if(k > 11) {
        //     // if (k % 2 == 0) {
        //         // double z_scale_down = 0.000266667;         ///// 총 횟수%2
        //         double z_scale_down = 0.000266667;         ///// 총 횟수%2
        //         taskPushBack_csMoveToolFrame(right_task_sub_8, {-z_scale_down, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // z축 이동
        //     // }
        // }

        subTaskUnScrewingChemicalHose(right_task_sub_8, k);
    }

    /////////////////////////////////////////////////////////////////////////////////
    taskPushBack_drflSetImpedance(right_task_sub_8, false); // Compliance CTRL OFF
    taskPushBack_delay_1ms(right_task_sub_8, 600);
    /////////////////////////////////////////////////////////////////////////////////

    right_task_sub_8.insert(right_task_sub_8.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    taskPushBack_delay_1ms(right_task_sub_8, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_8, {-0.002, 0, 0, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_8, 50);

    taskPushBack_csMoveToolFrame(right_task_sub_8, {0, 0, -0.0005, 0, -1, 0}, 0.01, 0.005, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(right_task_sub_8, 60);

    right_task_sub_8.insert(right_task_sub_8.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    taskPushBack_delay_1ms(right_task_sub_8, 100);

    // taskPushBack_setRobotAutoMode(right_task_sub_8);

    taskPushBack_csMove2(right_task_sub_8, {0, 0, 0.05, 0, 0, 0}, 0.02, 0.01, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_8, 50);
    taskPushBack_csMove2(right_task_sub_8, {0, 0, 0.100, 0, 0, 0}, 0.7, 0.7, true); // true: relative
    // taskPushBack_delay_1ms(right_task_sub_8, 50);

    taskPushBack_csMove2(right_task_sub_8, {0.1, -0.2, 0, 0, 0, 0}, 0.7, 0.7, true); // false: absolute, true: relative
    // taskPushBack_delay_1ms(right_task_sub_8, 500);

    taskPushBack_jsMove2(right_task_sub_8, {31.644, -11.474, 99.474, 33.409, 73.069, -153.435}, 30.0, 30.0, false);



    if(1) {
        //// drum hole surface
        std::string target_object = "right_holder_without_coupler";
        std::string target_tcp = "tcp03"; // screwing new gripper tcp
        std::string target_task = "drum_task";

        // taskPushBack_setRobotAutoMode(right_task_sub_8);

        // JsDouble scan_pose_target = {9.969, -5.642, 88.461, 32.060, 70.802, -209.990};
        JsDouble scan_pose_target = {15.949, -16.089, 96.922, 47.002, 65.014, -217.187};
        taskPushBack_jsMove2(right_task_sub_8, scan_pose_target, 30.0, 30.0, false);
        taskPushBack_delay_1ms(right_task_sub_8, 500);

        taskPushBack_drflSetTCP(right_task_sub_8, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_8, 60);
        //// Scanning
        taskPushBack_task3DScanningTargetObject(right_task_sub_8, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_8, 200);
        //// Matching
        taskPushBack_taskMatchingTargetObject(right_task_sub_8, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_8, 1000);

        taskPushBack_drflSetTCP(right_task_sub_8, target_tcp); // new gripper
        taskPushBack_delay_1ms(right_task_sub_8, 60);

        ////////////////////// 경유점 JS 1
        if(1) { // 역순
            JsDouble waypoint_1 = {70.522, -14.141, 111.397, -104.781, 31.213, 15.475}; // drum hole surface
            taskPushBack_jsMove2(right_task_sub_8, waypoint_1, 30.0, 30.0, false);
            // taskPushBack_delay_1ms(right_task_sub_8, 60);
        }
        //////////////////////

        /////////////////////// 인식 자세 이동
        taskPushBack_doGrasping_sharedTask(right_task_sub_8, 0.1, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(right_task_sub_8, 1000);


        // 기울이는 모션
        taskPushBack_csMoveToolFrame(right_task_sub_8, {0, 0, 0.008, 0, 0, 0}, 0.01, 0.005, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_8, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_8, {0, 0.001, 0, 0, 0, 0}, 0.005, 0.0025, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_8, 100);

        taskPushBack_csMoveToolFrame(right_task_sub_8, {0.050, 0, 0, 0, 0, 0}, 0.02, 0.02, true); // true: relative

        taskPushBack_csMoveToolFrame(right_task_sub_8, {0.040, 0, 0, 0, 0, 0}, 0.02, 0.02, true); // true: relative
        taskPushBack_csMoveToolFrame(right_task_sub_8, {0.015, 0, 0, 0, 0, 0}, 0.005, 0.0025, true); // true: relative


        taskPushBack_csMoveToolFrame(right_task_sub_8, {0.02, 0, 0, 0, 0, 0}, 0.005, 0.0025, true); // true: relative

        right_task_sub_8.insert(right_task_sub_8.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
        taskPushBack_delay_1ms(right_task_sub_8, 3000);

        taskPushBackKORASGripperCmd(right_task_sub_8, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(right_task_sub_8, 60);
        taskPushBackKORASGripperCmd(right_task_sub_8, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000);
        taskPushBack_delay_1ms(right_task_sub_8, 500);

        taskPushBack_csMoveToolFrame(right_task_sub_8, {0, 0, -0.003, 0, 0, 0}, 0.01, 0.005, true);
        // taskPushBack_delay_1ms(right_task_sub_8, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_8, {-0.020, 0, 0, 0, 0, 0}, 0.01, 0.005, true);
        // taskPushBack_delay_1ms(right_task_sub_8, 60);
        // taskPushBack_csMoveToolFrame(right_task_sub_8, {0, 0, -0.001, 0, 0, 0}, 0.01, 0.005, true);
        // taskPushBack_delay_1ms(right_task_sub_8, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_8, {-0.13, 0, 0, 0, 0, 0}, 0.1, 0.1, true);
        // taskPushBack_delay_1ms(right_task_sub_8, 20);
        taskPushBack_csMoveToolFrame(right_task_sub_8, {0, 0, -0.050, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_csMoveToolFrame(right_task_sub_8, {0, 0, -0.150, 0, 0, 0}, 0.1, 0.1, true);


        taskPushBack_jsMove2(right_task_sub_8, right_home_pose_, 30.0, 30.0, false);
        ////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////


    }

    return right_task_sub_8;
}

std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask9Task() {
    std::vector<UnitTask> right_task_sub_9;

    taskPushBack_jsMove2(right_task_sub_9, right_home_pose_, 70.0, 70.0, false);

    right_task_sub_9.insert(right_task_sub_9.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_9, 2000);
    right_task_sub_9.insert(right_task_sub_9.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
    taskPushBack_delay_1ms(right_task_sub_9, 1000);

    // taskPushBack_setRobotAutoMode(right_task_sub_9);

    if(1) {
        std::string target_object = "right_holder_with_lid_cap";
        std::string target_tcp = "tcp05"; // screwing new gripper tcp
        std::string target_task = "default";

        JsDouble scan_pose_target = {86.371, -0.738, 102.824, -65.589, 43.892, -4.230};
        taskPushBack_jsMove2(right_task_sub_9, scan_pose_target, 70.0, 70.0, false);
        taskPushBack_delay_1ms(right_task_sub_9, 500);
        //// Scanning
        taskPushBack_drflSetTCP(right_task_sub_9, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_9, 60);

        taskPushBack_task3DScanningTargetObject(right_task_sub_9, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_9, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(right_task_sub_9, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_9, 1000);
        taskPushBack_drflSetTCP(right_task_sub_9, target_tcp); // new gripper
        taskPushBack_delay_1ms(right_task_sub_9, 60);

        // taskPushBack_setRobotAutoMode(right_task_sub_9);

        if(1) {
            JsDouble waypoint_1 = {82.733, -2.915, 98.377, -35.213, 52.585, -124.726};
            taskPushBack_jsMove2(right_task_sub_9, waypoint_1, 120.0, 120.0, false);
            taskPushBack_csMove2(right_task_sub_9, {0.2, 0.2, 0.05, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
        }

        /////////////////////// 인식 자세 이동
        taskPushBack_doGrasping_sharedTask(right_task_sub_9, 0.05, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(right_task_sub_9, 60);
        /////////////////////// 인식 자세 이동
    }

    taskPushBack_csMoveToolFrame(right_task_sub_9, {0, -0.002, 0.057, 0, 0, 0}, 0.1, 0.1, true);
    taskPushBack_delay_1ms(right_task_sub_9, 60);

    //// 파지
    right_task_sub_9.insert(right_task_sub_9.end(), drum_grp_lid_cap_close_tight.begin(), drum_grp_lid_cap_close_tight.end());
    taskPushBack_delay_1ms(right_task_sub_9, 4000);

    taskPushBackKORASGripperCmd(right_task_sub_9, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_9, 60);
    taskPushBackKORASGripperCmd(right_task_sub_9, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000);
    taskPushBack_delay_1ms(right_task_sub_9, 3000);

    taskPushBack_csMoveToolFrame(right_task_sub_9, {0, 0, -0.048, 0, 0, 0}, 0.03, 0.015, true);

    // taskPushBack_setRobotAutoMode(right_task_sub_9);

    taskPushBack_csMove2(right_task_sub_9, {-0.2, -0.2, -0.05, 0, 0, 0}, 0.7, 0.7, true); // false: absolute, true: relative

    if(1){
        /////////////////////////////////////////////////////////////////////////
        std::string target_object = "right_drum_lid_cap_screwing";
        std::string target_tcp = "tcp05"; // screwing new gripper tcp
        std::string target_task = "default";
        JsDouble scan_pose_target = {60.974, -18.933, 104.790, 20.208, 61.469, -124.065};

        taskPushBack_jsMove2(right_task_sub_9, scan_pose_target, 120.0, 120.0, false);
        taskPushBack_delay_1ms(right_task_sub_9, 500);

        taskPushBack_drflSetTCP(right_task_sub_9, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_9, 60);

        taskPushBack_task3DScanningTargetObject(right_task_sub_9, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_9, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(right_task_sub_9, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_9, 1000);

        taskPushBack_drflSetTCP(right_task_sub_9, target_tcp); // new gripper
        taskPushBack_delay_1ms(right_task_sub_9, 60);

        //// 천장 부딪히지 않도록 경유지 설정
        if(1) {
            JsDouble waypoint_1 = {70.488, 8.216, 82.186, 0.163, 90.371, -282.865};
            taskPushBack_jsMove2(right_task_sub_9, waypoint_1, 70.0, 70.0, false);
            // taskPushBack_delay_1ms(right_task_sub_9, 50);
            taskPushBack_csMove2(right_task_sub_9, {0, 0.100, 0.03, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
            // taskPushBack_delay_1ms(right_task_sub_9, 50);
        }

        ///////////////////// 인식 자세 이동
        taskPushBack_doGrasping_sharedTask(right_task_sub_9, 0.1, 0.1, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(right_task_sub_9, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_9, {0, -0.004, 0, 0, 0, 0}, 0.7, 0.7, true); // true: relative

        taskPushBack_csMoveToolFrame(right_task_sub_9, {0, 0, 0.017, 0, 0, 0}, 0.7, 0.7, true); // true: relative

        ////////0403 test
        taskPushBack_csMoveToolFrame(right_task_sub_9, {0, 0, 0.002, 0, 0, 10}, 0.025, 0.025, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_9, 60);

        right_task_sub_9.insert(right_task_sub_9.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(right_task_sub_9, 1000);
        right_task_sub_9.insert(right_task_sub_9.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(right_task_sub_9, 1000);
        taskPushBack_jsMove2(right_task_sub_9, {0, 0, 0, 0, 0, -10}, 30.0, 30.0, true);

        taskPushBack_csMoveToolFrame(right_task_sub_9, {0, 0, 0.012, 0, 0, 0}, 0.1, 0.1, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_9, 60);

        // 토크 가드 ON: J6 기준 15.0 이상이면 정지 (필요 시 조정)
        // taskPushBack_torqueGuardOn(right_task_sub_9, 5, 4.0);
        taskPushBack_torqueGuardOn(right_task_sub_9, 5, 3.5);

        // taskPushBack_setRobotAutoMode(right_task_sub_9);

        int cnt_lid_cap_screwing = 10;
        for (int cycle = 0; cycle < cnt_lid_cap_screwing; cycle++) {
            right_task_sub_9.insert(right_task_sub_9.end(), drum_grp_lid_cap_close_for_screwing_.begin(), drum_grp_lid_cap_close_for_screwing_.end());


            if(cycle != cnt_lid_cap_screwing - 1) { // 마지막 trial에서는 스킵
                taskPushBack_delay_1ms(right_task_sub_9, 2000);
                taskPushBack_jsMove2(right_task_sub_9, {0, 0, 0, 0, 0, 195}, 100.0, 100.0, true);
                right_task_sub_9.insert(right_task_sub_9.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
                taskPushBack_delay_1ms(right_task_sub_9, 2000);
            } else {
                //// 마지막 횟수에서 더 돌리기 (90도만)
                taskPushBack_delay_1ms(right_task_sub_9, 2000);
                taskPushBack_jsMove2(right_task_sub_9, {0, 0, 0, 0, 0, 135}, 100.0, 100.0, true);
                //// 아래에서 더 돌리면 팁 파손 가능성
                taskPushBack_jsMove2(right_task_sub_9, {0, 0, 0, 0, 0, 15}, 15.0, 15.0, true);
                taskPushBack_jsMove2(right_task_sub_9, {0, 0, 0, 0, 0, 5}, 15.0, 15.0, true);
                taskPushBack_jsMove2(right_task_sub_9, {0, 0, 0, 0, 0, -10}, 100.0, 100.0, true);
                right_task_sub_9.insert(right_task_sub_9.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
                taskPushBack_delay_1ms(right_task_sub_9, 2000);
                taskPushBack_jsMove2(right_task_sub_9, {0, 0, 0, 0, 0, -5}, 100.0, 100.0, true);
            }

            if(cycle != cnt_lid_cap_screwing - 1) { // 마지막 trial에서는 스킵
                taskPushBack_jsMove2(right_task_sub_9, {0, 0, 0, 0, 0, -15}, 100.0, 100.0, true);
                taskPushBack_csMoveToolFrame(right_task_sub_9, {0, 0, -0.050, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
                taskPushBack_jsMove2(right_task_sub_9, {0, 0, 0, 0, 0, -180}, 100.0, 100.0, true);
                taskPushBack_csMoveToolFrame(right_task_sub_9, {0, 0, 0.050, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
                taskPushBack_delay_1ms(right_task_sub_9, 20);
            }
        }

        // === 분기 타겟 라벨 ===
        taskPushBack_label(right_task_sub_9, "SCREW_OPEN_STEP");
        // 토크 가드 OFF
        taskPushBack_torqueGuardOff(right_task_sub_9);

        // taskPushBack_setRobotAutoMode(right_task_sub_9);

        taskPushBack_jsMove2(right_task_sub_9, {0, 0, 0, 0, 0, -10}, 70.0, 70.0, true);
        right_task_sub_9.insert(right_task_sub_9.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
        taskPushBack_delay_1ms(right_task_sub_9, 2000);
        taskPushBack_jsMove2(right_task_sub_9, {0, 0, 0, 0, 0, -5}, 70.0, 70.0, true);

        taskPushBack_csMove2(right_task_sub_9, {0, 0, 0.060, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_9, 60);


        taskPushBack_csMove2(right_task_sub_9, {0, -0.4, 0, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_9, 60);

        taskPushBack_jsMove2(right_task_sub_9, right_home_pose_, 70.0, 70.0, false);
        taskPushBack_delay_1ms(right_task_sub_9, 60);

    }

    return right_task_sub_9;
}

std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask10Task() {
    std::vector<UnitTask> right_task_sub_10;

    // taskPushBack_setRobotAutoMode(right_task_sub_10);
    
    if(1) // 별모양 뚜껑 제거
    {
        // setup
        taskPushBack_jsMove2(right_task_sub_10, right_home_pose_, 70.0, 70.0, false);
        // taskPushBack_delay_1ms(right_task_sub_10, 60);

        right_task_sub_10.insert(right_task_sub_10.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
        taskPushBack_delay_1ms(right_task_sub_10, 3000);
        right_task_sub_10.insert(right_task_sub_10.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(right_task_sub_10, 500);

        JsDouble scan_pose_hole_star = {60.974, -18.933, 104.790, 20.208, 61.469, -124.065};
        taskPushBack_jsMove2(right_task_sub_10, scan_pose_hole_star, 70.0, 70.0, false);
        taskPushBack_delay_1ms(right_task_sub_10, 1000);

        /////////////////////////////////////////////////////////////
        std::string target_object = "right_drum_lid_cap_unscrewing";  
        std::string target_task = "default";

        //// Scanning
        taskPushBack_drflSetTCP(right_task_sub_10, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_10, 60);

        taskPushBack_task3DScanningTargetObject(right_task_sub_10, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_10, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(right_task_sub_10, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_10, 1000);

        taskPushBack_drflSetTCP(right_task_sub_10, "tcp05"); // new gripper
        taskPushBack_delay_1ms(right_task_sub_10, 60);

        // taskPushBack_setRobotAutoMode(right_task_sub_10);

        //// 천장 부딪히지 않도록 경유지 설정
        if(1) {
            JsDouble waypoint_1 = {70.488, 8.216, 82.186, 0.163, 90.371, -102.865};
            taskPushBack_jsMove2(right_task_sub_10, waypoint_1, 120.0, 120.0, false);
            // taskPushBack_delay_1ms(right_task_sub_10, 50);
            taskPushBack_csMove2(right_task_sub_10, {0, 0.100, 0.03, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
            // taskPushBack_delay_1ms(right_task_sub_10, 50);
        }

        taskPushBack_doGrasping_sharedTask(right_task_sub_10, 0.1, 0.1, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(right_task_sub_10, 60);


        taskPushBack_csMoveToolFrame(right_task_sub_10, {0, 0, 0.040, 0, 0, 0}, 0.05, 0.1, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(right_task_sub_10, 50);

        // taskPushBack_setRobotAutoMode(right_task_sub_10);


        for (int cycle = 0; cycle < 6; cycle++) {
            right_task_sub_10.insert(right_task_sub_10.end(), drum_grp_lid_cap_close_.begin(), drum_grp_lid_cap_close_.end());
            taskPushBack_delay_1ms(right_task_sub_10, 3000);
            taskPushBack_jsMove2(right_task_sub_10, {0, 0, 0, 0, 0, -195}, 100.0, 100.0, true);
            right_task_sub_10.insert(right_task_sub_10.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
            taskPushBack_delay_1ms(right_task_sub_10, 1000);
            taskPushBack_jsMove2(right_task_sub_10, {0, 0, 0, 0, 0, 15}, 100.0, 100.0, true);
            // taskPushBack_delay_1ms(right_task_sub_10, 60);
            taskPushBack_csMoveToolFrame(right_task_sub_10, {0, 0, -0.050, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
            // taskPushBack_delay_1ms(right_task_sub_10, 50);
            taskPushBack_jsMove2(right_task_sub_10, {0, 0, 0, 0, 0, 180}, 100.0, 100.0, true);
            taskPushBack_csMoveToolFrame(right_task_sub_10, {0, 0, 0.050, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
            // taskPushBack_delay_1ms(right_task_sub_10, 50);
        }

        taskPushBack_delay_1ms(right_task_sub_10, 1000);
        right_task_sub_10.insert(right_task_sub_10.end(), drum_grp_lid_cap_close_tight.begin(), drum_grp_lid_cap_close_tight.end());
        taskPushBack_delay_1ms(right_task_sub_10, 4000);

        taskPushBackKORASGripperCmd(right_task_sub_10, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(right_task_sub_10, 60);
        taskPushBackKORASGripperCmd(right_task_sub_10, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000);
        taskPushBack_delay_1ms(right_task_sub_10, 3000);

        taskPushBack_jsMove2(right_task_sub_10, {0, 0, 0, 0, 0, -30}, 40.0, 40.0, true);
        // taskPushBack_delay_1ms(right_task_sub_10, 500);

        taskPushBackKORASGripperCmd(right_task_sub_10, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(right_task_sub_10, 60);
        taskPushBackKORASGripperCmd(right_task_sub_10, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -1500);
        taskPushBack_delay_1ms(right_task_sub_10, 3000);

        taskPushBack_csMove2(right_task_sub_10, {0, 0, 0.02, 0, 0, 0}, 0.025, 0.05, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(right_task_sub_10, 500);
        taskPushBack_csMove2(right_task_sub_10, {0, 0, 0.02, 0, 0, 0}, 0.2, 0.2, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(right_task_sub_10, 500);
        taskPushBack_csMove2(right_task_sub_10, {0, -0.3, 0, 0, 0, 0}, 0.5, 0.5, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(right_task_sub_10, 50);

    }


    if(1) // 별모양 뚜껑 거치대에 놓기
    {
        std::string target_object = "right_holder_without_lid_cap";
        std::string target_tcp = "tcp05";
        std::string target_task = "default";
        // JsDouble scan_pose_target = {70.094, -11.916, 114.557, -59.840, 29.435, -15.609 };
        JsDouble scan_pose_target = {86.371, -0.738, 102.824, -65.589, 43.892, -4.230};

        // taskPushBack_setRobotAutoMode(right_task_sub_10);

        taskPushBack_jsMove2(right_task_sub_10, scan_pose_target, 120.0, 120.0, false);
        taskPushBack_delay_1ms(right_task_sub_10, 1000);

        taskPushBack_drflSetTCP(right_task_sub_10, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_10, 60);

        //// Scanning
        taskPushBack_task3DScanningTargetObject(right_task_sub_10, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_10, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(right_task_sub_10, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_10, 1000);

        taskPushBack_drflSetTCP(right_task_sub_10, target_tcp); // new gripper
        taskPushBack_delay_1ms(right_task_sub_10, 60);

        // taskPushBack_setRobotAutoMode(right_task_sub_10);

        if(1) {
            JsDouble waypoint_1 = {82.733, -2.915, 98.377, -35.213, 52.585, -124.726};
            taskPushBack_jsMove2(right_task_sub_10, waypoint_1, 120.0, 120.0, false);
            taskPushBack_csMove2(right_task_sub_10, {0.2, 0.2, 0.05, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
        }

        taskPushBack_doGrasping_sharedTask(right_task_sub_10, 0.05, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(right_task_sub_10, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_10, {-0.003, 0, 0, 0, 0, 0}, 0.025, 0.025, true);    
        taskPushBack_delay_1ms(right_task_sub_10, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_10, {0, -0.002, 0.025, 0, 0, 0}, 0.05, 0.05, true);    
        taskPushBack_delay_1ms(right_task_sub_10, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_10, {0, 0, 0.015, 0, 0, 0}, 0.025, 0.025, true);    
        taskPushBack_delay_1ms(right_task_sub_10, 60);

        
        // taskPushBack_setRobotAutoMode(right_task_sub_10);

        right_task_sub_10.insert(right_task_sub_10.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(right_task_sub_10, 2500);


        taskPushBack_csMoveToolFrame(right_task_sub_10, {0, 0, -0.022, 0, 0, 0}, 0.1, 0.1, true);
        // taskPushBack_delay_1ms(right_task_sub_10, 60);

        taskPushBack_csMove2(right_task_sub_10, {-0.2, -0.2, -0.07, 0, 0, 0}, 0.4, 0.4, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_10, 500);

        

        taskPushBack_jsMove2(right_task_sub_10, right_home_pose_, 70.0, 70.0, false);

    }

    return right_task_sub_10;
}


////// SubTask11 Key Code
////// lid cap screwing 자세 사용 (key code 위에 올려놓고 돌림)
////// 1. lid cap screwing 구멍 촬영
////// 2. key code holder   촬영
////// 3. key code 빨간점 각도 촬영
////// 4. lid cap screwing 구멍 이동
////// 5. key code 빨간점 각도 돌리기
std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask11Task() {
    std::vector<UnitTask> right_task_sub_11;
    
    taskPushBack_jsMove2(right_task_sub_11, right_home_pose_, 30.0, 30.0, false);
    // taskPushBack_delay_1ms(right_task_sub_10, 60);


    right_task_sub_11.insert(right_task_sub_11.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    taskPushBack_delay_1ms(right_task_sub_11, 3000);

    //////////////// lid cap screwing FOR Key Code 위치 촬영 (key code 결합에 lid cap screwing 자세 사용) ////////////////
    taskPushBack_jsMove2(right_task_sub_11, {31.644, -11.474, 99.474, 33.409, 73.069, -153.435}, 30.0, 30.0, false);
    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_11, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_11, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_11, "right_drum_lid_cap_screwing"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_11, 600);
    taskPushBack_taskMatchingTargetObject(right_task_sub_11, "right_drum_lid_cap_screwing", false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_11, 1000);

    /// 스캔 전 경유자세 (sub right home pose)
    taskPushBack_jsMove2(right_task_sub_11, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_11, 60);

    taskPushBack_csMove2(right_task_sub_11, {0, 0.300, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(right_task_sub_11, 60);


     //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_11, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_11, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_11, "right_drum_holder_with_key_code"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_11, 600);
    taskPushBack_taskMatchingTargetObject(right_task_sub_11, "right_drum_holder_with_key_code", false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_11, 1000);

    taskPushBack_drflSetTCP(right_task_sub_11, "tcp08"); // TCP for drum_lid_cap_screwing
    taskPushBack_delay_1ms(right_task_sub_11, 1000);

    taskPushBack_doGrasping_sharedTask(right_task_sub_11, 0.05, 0.05, "default", "right_drum_holder_with_key_code"); // Move to grasping pose, (acc, vel)
    taskPushBack_delay_1ms(right_task_sub_11, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_11, {0, 0, 0.084, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_11, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_11, {-0.0015 , 0, 0, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_11, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_11, {0, 0.0005, 0, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_11, 60);

    taskPushBackKORASGripperCmd(right_task_sub_11, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_11, 60);
    taskPushBackKORASGripperCmd(right_task_sub_11, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -9500);
    taskPushBack_delay_1ms(right_task_sub_11, 3000);
    taskPushBackKORASGripperCmd(right_task_sub_11, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_11, 60);
    taskPushBackKORASGripperCmd(right_task_sub_11, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -1500);
    taskPushBack_delay_1ms(right_task_sub_11, 3000);

    taskPushBack_csMoveToolFrame(right_task_sub_11, {0, 0, -0.048, 0, 0, 0}, 0.03, 0.015, true);

    taskPushBack_csMove2(right_task_sub_11, {-0.1, -0.450, -0.075, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative

    ///// Key Ring 위치 촬영 자세 이동
    taskPushBack_jsMove2(right_task_sub_11, {56.655, 16.627, 72.931, 0.241, 90.276, -123.864}, 30.0, 30.0, false);
    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_11, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_11, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_11, "right_drum_key_code"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_11, 600);
    taskPushBack_drflSetTCP(right_task_sub_11, "tcp08"); // TCP for drum_lid_cap_screwing
    taskPushBack_delay_1ms(right_task_sub_11, 1000);


    taskPushBack_doGrasping_sharedTask(right_task_sub_11, 0.05, 0.05, "default", "right_drum_lid_cap_screwing"); // Move to grasping pose, (acc, vel)
    taskPushBack_delay_1ms(right_task_sub_11, 60);

    ///// /hanyang/coupler/keycode_angle 토픽의 z 값 받아서 Tool Rz 회전 (전용 태스크)
    taskPushBack_csMoveToolFrameKeycode(right_task_sub_11, 0.02, 0.02);
    taskPushBack_delay_1ms(right_task_sub_11, 60);


    taskPushBack_csMoveToolFrame(right_task_sub_11, {0.001, 0, 0, 0, 0, 0}, 0.05, 0.05, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_11, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_11, {0, 0, 0.045, 0, 0, 0}, 0.05, 0.05, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_11, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_11, {0, 0, 0.002, 0, 0, 0}, 0.025, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_11, 3000);
    taskPushBack_csMoveToolFrame(right_task_sub_11, {0, 0, 0.002, 0, 0, 0}, 0.025, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_11, 3000);

    // taskPushBackKORASGripperCmd(right_task_sub_11, (uint16_t)KR_GRP::POS_RESET);
    // taskPushBack_delay_1ms(right_task_sub_11, 60);
    // taskPushBackKORASGripperCmd(right_task_sub_11, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +1000);
    // taskPushBack_delay_1ms(right_task_sub_11, 3000);


    taskPushBack_csMoveToolFrame(right_task_sub_11, {0, 0, 0, 0, 0, 20}, 0.005, 0.005, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_11, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_11, {0, 0, 0, 0, 0, -25}, 0.005, 0.005, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_11, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_11, {0, 0, 0, 0, 0, 5}, 0.005, 0.005, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_11, 60);

    taskPushBackKORASGripperCmd(right_task_sub_11, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_11, 60);
    taskPushBackKORASGripperCmd(right_task_sub_11, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +4000);
    taskPushBack_delay_1ms(right_task_sub_11, 3000);

    taskPushBack_csMove2(right_task_sub_11, {0, 0, 0.100, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(right_task_sub_11, 1000);

    taskPushBack_csMove2(right_task_sub_11, {0, -0.300, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(right_task_sub_11, 2000);


    // 필요 시 홈 복귀
    taskPushBack_jsMove2(right_task_sub_11, right_home_pose_, 70.0, 70.0, false);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    return right_task_sub_11;
}

std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask12Task() {
    std::vector<UnitTask> right_task_sub_12;
    
    taskPushBack_jsMove2(right_task_sub_12, right_home_pose_, 30.0, 30.0, false);

    right_task_sub_12.insert(right_task_sub_12.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    taskPushBack_delay_1ms(right_task_sub_12, 3000);

    //////////////// lid cap screwing FOR Key Code 위치 촬영 (key code 결합에 lid cap screwing 자세 사용) ////////////////
    taskPushBack_jsMove2(right_task_sub_12, {31.644, -11.474, 99.474, 33.409, 73.069, -153.435}, 30.0, 30.0, false);
    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_12, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_12, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_12, "right_drum_key_code_unscrewing"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_12, 600);
    taskPushBack_taskMatchingTargetObject(right_task_sub_12, "right_drum_key_code_unscrewing", false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_12, 1000);

    ///////Key Ring 촬영 전 경유 자세
    taskPushBack_jsMove2(right_task_sub_12, {37.715, -4.753, 101.891, 0.115, 82.740, -142.838}, 30.0, 30.0, false);
    taskPushBack_csMove2(right_task_sub_12, {0, 0.3, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative

    //////////////// Key Ring 위치 촬영 ////////////////
    JsDouble scan_pose = {56.655, 16.627, 72.931, 0.241, 90.276, -123.864};
    taskPushBack_jsMove2(right_task_sub_12, scan_pose, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_12, 1000);

    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_12, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_12, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_12, "right_drum_key_code"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_12, 600);
    taskPushBack_drflSetTCP(right_task_sub_12, "tcp08"); // TCP for drum_lid_cap_screwing
    taskPushBack_delay_1ms(right_task_sub_12, 1000);

    taskPushBack_doGrasping_sharedTask(right_task_sub_12, 0.05, 0.05, "default", "right_drum_key_code_unscrewing"); // Move to grasping pose, (acc, vel)
    taskPushBack_delay_1ms(right_task_sub_12, 3000);

    taskPushBack_csMoveToolFrameKeycode(right_task_sub_12, 0.02, 0.02);
    taskPushBack_delay_1ms(right_task_sub_12, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_12, {0, 0, 0.055, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_12, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_12, {0, -0.0025, 0, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_12, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_12, {0, 0, 0.007, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_12, 60);

    taskPushBackKORASGripperCmd(right_task_sub_12, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_12, 60);
    taskPushBackKORASGripperCmd(right_task_sub_12, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -9500);
    taskPushBack_delay_1ms(right_task_sub_12, 3000);

    taskPushBack_csMoveToolFrame(right_task_sub_12, {0, 0, -0.100, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_12, 60);

    //// key code holder 진입 전 경유자세
    taskPushBack_jsMove2(right_task_sub_12, scan_pose, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_12, 1000);

    //// key code holder 진입 전 경유자세
    taskPushBack_jsMove2(right_task_sub_12, {55.312, 8.986, 100.241, 75.696, 33.058, -178.847}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_12, 1000);

    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_12, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_12, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_12, "right_drum_holder_without_key_code"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_12, 600);
    taskPushBack_taskMatchingTargetObject(right_task_sub_12, "right_drum_holder_without_key_code", false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_12, 1000);


    taskPushBack_drflSetTCP(right_task_sub_12, "tcp08"); // TCP for drum_lid_cap_screwing
    taskPushBack_delay_1ms(right_task_sub_12, 1000);
    //// 파지
    taskPushBack_doGrasping_sharedTask(right_task_sub_12, 0.05, 0.05, "default", "right_drum_holder_without_key_code"); // Move to grasping pose, (acc, vel)
    taskPushBack_delay_1ms(right_task_sub_12, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_12, {-0.0045, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_12, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_12, {0, 0.001, 0, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_12, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_12, {0, 0, 0.070, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_12, 5000);
    taskPushBack_csMoveToolFrame(right_task_sub_12, {0, 0, 0.010, 0, 0, 0}, 0.025, 0.025, true);
    taskPushBack_delay_1ms(right_task_sub_12, 60);

    taskPushBackKORASGripperCmd(right_task_sub_12, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_12, 60);
    taskPushBackKORASGripperCmd(right_task_sub_12, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +4000);
    taskPushBack_delay_1ms(right_task_sub_12, 3000);

    //// 빠져나오기
    taskPushBack_csMoveToolFrame(right_task_sub_12, {0, 0, -0.1, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_12, 60);

    //// key code holder 진입 전 경유자세 사용
    taskPushBack_jsMove2(right_task_sub_12, {55.312, 8.986, 100.241, 75.696, 33.058, -178.847}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_12, 1000);

    taskPushBack_csMove2(right_task_sub_12, {0, -0.3, 0, 0, 0, 0}, 0.05, 0.05, true); // false: absolute, true: relative
    taskPushBack_csMove2(right_task_sub_12, {0, -0.2, 0, 0, 0, 0}, 0.05, 0.05, true); // false: absolute, true: relative


    taskPushBack_jsMove2(right_task_sub_12, right_home_pose_, 30.0, 30.0, false);



    return right_task_sub_12;
}

std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask13Task() {
    std::vector<UnitTask> right_task_sub_13;
    
    taskPushBack_jsMove2(right_task_sub_13, right_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_13, 60);

    right_task_sub_13.insert(right_task_sub_13.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_13, 2000);
    right_task_sub_13.insert(right_task_sub_13.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
    taskPushBack_delay_1ms(right_task_sub_13, 1000);

    /// 스캔 전 경유자세 (sub right home pose)
    taskPushBack_jsMove2(right_task_sub_13, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_13, 60);

    taskPushBack_csMove2(right_task_sub_13, {0, 0.300, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(right_task_sub_13, 60);

    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_13, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_13, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_13, "right_holder_with_lid_cap"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_13, 600);
    taskPushBack_taskMatchingTargetObject(right_task_sub_13, "right_holder_with_lid_cap", false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_13, 1000);

    taskPushBack_drflSetTCP(right_task_sub_13, "tcp05"); // TCP for drum_lid_cap_screwing
    taskPushBack_delay_1ms(right_task_sub_13, 1000);

    taskPushBack_doGrasping_sharedTask(right_task_sub_13, 0.05, 0.05, "default", "right_holder_with_lid_cap"); // Move to grasping pose, (acc, vel)
    taskPushBack_delay_1ms(right_task_sub_13, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_13, {0, 0 , 0.070, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_13, 60);

    //// 파지
    right_task_sub_13.insert(right_task_sub_13.end(), drum_grp_lid_cap_close_tight.begin(), drum_grp_lid_cap_close_tight.end());
    taskPushBack_delay_1ms(right_task_sub_13, 4000);

    taskPushBackKORASGripperCmd(right_task_sub_13, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_13, 60);
    taskPushBackKORASGripperCmd(right_task_sub_13, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -3500);
    taskPushBack_delay_1ms(right_task_sub_13, 3000);

    taskPushBack_csMoveToolFrame(right_task_sub_13, {0, 0, -0.048, 0, 0, 0}, 0.03, 0.015, true);

    // taskPushBack_setRobotAutoMode(right_task_sub_13);

    taskPushBack_csMove2(right_task_sub_13, {-0.2, -0.450, -0.075, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative

    taskPushBack_jsMove2(right_task_sub_13, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);

    return right_task_sub_13;
}

std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask14Task() {
    std::vector<UnitTask> right_task_sub_14;
    
    taskPushBack_jsMove2(right_task_sub_14, right_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_14, 60);

    right_task_sub_14.insert(right_task_sub_14.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    taskPushBack_delay_1ms(right_task_sub_14, 3000);

    /// 스캔 전 경유자세 (sub right home pose)
    taskPushBack_jsMove2(right_task_sub_14, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_14, 60);

    taskPushBack_csMove2(right_task_sub_14, {0, 0.300, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(right_task_sub_14, 60);

    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_14, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_14, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_14, "right_drum_holder_with_key_code"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_14, 600);
    taskPushBack_taskMatchingTargetObject(right_task_sub_14, "right_drum_holder_with_key_code", false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_14, 1000);

    taskPushBack_drflSetTCP(right_task_sub_14, "tcp08"); // TCP for drum_lid_cap_screwing
    taskPushBack_delay_1ms(right_task_sub_14, 1000);

    taskPushBack_doGrasping_sharedTask(right_task_sub_14, 0.05, 0.05, "default", "right_drum_holder_with_key_code"); // Move to grasping pose, (acc, vel)
    taskPushBack_delay_1ms(right_task_sub_14, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_14, {0, 0, 0.084, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_14, 60);
    taskPushBackKORASGripperCmd(right_task_sub_14, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_14, 60);
    taskPushBackKORASGripperCmd(right_task_sub_14, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -9500);
    taskPushBack_delay_1ms(right_task_sub_14, 3000);

    taskPushBack_csMoveToolFrame(right_task_sub_14, {0, 0, -0.048, 0, 0, 0}, 0.03, 0.015, true);

    taskPushBack_csMove2(right_task_sub_14, {-0.1, -0.450, -0.075, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative


    taskPushBack_jsMove2(right_task_sub_14, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);

    return right_task_sub_14;
}


std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask15Task() {
    std::vector<UnitTask> right_task_sub_15;
    
    taskPushBack_jsMove2(right_task_sub_15, right_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_15, 60);

    right_task_sub_15.insert(right_task_sub_15.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_15, 4000);

    ////////////////////////////////////////////////////////////////////////////////////
    //// 파지 시에, 팁이 걸릴 경우 아래 값으로 조절: 현재: -3000
    taskPushBackKORASGripperCmd(right_task_sub_15, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_15, 60);
    taskPushBackKORASGripperCmd(right_task_sub_15, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000);
    taskPushBack_delay_1ms(right_task_sub_15, 500);
    ////////////////////////////////////////////////////////////////////////////////////

    if(1) {
        ///// hole scan pose
        JsDouble scan_pose_target = {31.644, -11.474, 99.474, 33.409, 73.069, -153.435};
        taskPushBack_jsMove2(right_task_sub_15, scan_pose_target, 30.0, 30.0, false);

        //// Scanning
        taskPushBack_drflSetTCP(right_task_sub_15, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_15, 60);
        taskPushBack_task3DScanningTargetObject(right_task_sub_15, "right_drum_hole_surface"); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_15, 600);
        taskPushBack_taskMatchingTargetObject(right_task_sub_15, "right_drum_hole_surface", false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_15, 1000);

        taskPushBack_drflSetTCP(right_task_sub_15, "tcp03"); // TCP for drum_lid_cap_screwing
        taskPushBack_delay_1ms(right_task_sub_15, 1000);

    }

    if(1) {
        //// right_holder_with_coupler
        JsDouble scan_pose_target = {4.942, -7.319, 86.477, 39.470, 80.375, -213.474};

        taskPushBack_jsMove2(right_task_sub_15, scan_pose_target, 30.0, 30.0, false);
        taskPushBack_delay_1ms(right_task_sub_15, 500);

        //// Scanning
        taskPushBack_drflSetTCP(right_task_sub_15, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_15, 60);
        taskPushBack_task3DScanningTargetObject(right_task_sub_15, "right_holder_with_coupler"); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_15, 600);
        taskPushBack_taskMatchingTargetObject(right_task_sub_15, "right_holder_with_coupler", false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_15, 1000);

        taskPushBack_drflSetTCP(right_task_sub_15, "tcp03"); // TCP for drum_lid_cap_screwing
        taskPushBack_delay_1ms(right_task_sub_15, 1000);

        /////////////////////// 인식 자세 이동
        taskPushBack_doGrasping_sharedTask(right_task_sub_15, 0.05, 0.05, "drum_task", "right_holder_with_coupler"); // Move to grasping pose, (acc, vel)
        // taskPushBack_delay_1ms(right_task_sub_15, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_15, {0, 0.002, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_15, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_15, {0, 0, 0.135, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_15, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_15, {0, 0, 0, 0, 3, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_15, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_15, {0.020, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_15, 60);


        //// 상대 모션, 파지 중에 걸릴 경우, 아래를 수정
        taskPushBack_csMoveToolFrame(right_task_sub_15, {0.027, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_15, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_15, {0, 0, 0, 0, 3, 0}, 0.02, 0.01, true);
        taskPushBack_delay_1ms(right_task_sub_15, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_15, {0, 0, -0.002, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_15, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_15, {0.020, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_15, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_15, {0.060, 0, 0, 0, 0, 0}, 0.04, 0.02, true);
        taskPushBack_delay_1ms(right_task_sub_15, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_15, {0.040, 0, 0, 0, 0, 0}, 0.04, 0.02, true);
        taskPushBack_delay_1ms(right_task_sub_15, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_15, {0, 0, 0, 0, -4, 0}, 0.02, 0.01, true);
        taskPushBack_delay_1ms(right_task_sub_15, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_15, {0, 0, 0.008, 0, 0, 0}, 0.01, 0.005, true);
        taskPushBack_delay_1ms(right_task_sub_15, 60);

        right_task_sub_15.insert(right_task_sub_15.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
        taskPushBack_delay_1ms(right_task_sub_15, 4000);

        taskPushBack_csMoveToolFrame(right_task_sub_15, {-0.03, 0, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
        taskPushBack_csMoveToolFrame(right_task_sub_15, {-0.15, 0, 0, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative

    }

    taskPushBack_csMove2(right_task_sub_15, {-0.200, -0.200, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative

    /// 스캔 전 경유자세 (sub right home pose)
    taskPushBack_jsMove2(right_task_sub_15, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_15, 60);


    /////////////////////// 인식 자세 이동
    taskPushBack_doGrasping_sharedTask(right_task_sub_15, 0.05, 0.05, "drum_task", "right_drum_hole_surface"); // Move to grasping pose, (acc, vel)
    // taskPushBack_delay_1ms(right_task_sub_15, 60);

    taskPushBack_csMove2(right_task_sub_15, {0.001, 0, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative

    taskPushBack_csMoveToolFrame(right_task_sub_15, {0.0, 0, 0, 0, 0, -3}, 0.1, 0.05, true); // true: relative

    taskPushBack_csMoveToolFrame(right_task_sub_15, {0.0, 0.01, 0, 0, 0, 0}, 0.1, 0.05, true); // true: relative
    taskPushBack_csMoveToolFrame(right_task_sub_15, {0.0, 0, 0.003, 0, 0, 0}, 0.1, 0.05, true); // true: relative

    taskPushBack_csMoveToolFrame(right_task_sub_15, {0.080, 0, 0, 0, 0, 0}, 0.2, 0.1, true); // true: relative
    taskPushBack_csMoveToolFrame(right_task_sub_15, {0.030, 0, 0, 0, 0, 0}, 0.05, 0.05, true); // true: relative


    // 기울이는 모션
    taskPushBack_csMoveToolFrame(right_task_sub_15, {0, 0, -0.0014, 0, -2.0, 0}, 0.01, 0.005, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(right_task_sub_15, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_15, {0.007, 0, 0, 0, 0, 0}, 0.005, 0.0025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_15, 100);
    taskPushBack_csMoveToolFrame(right_task_sub_15, {0.005, 0, 0, 0, 0, 0}, 0.005, 0.0025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_15, 100);

    right_task_sub_15.insert(right_task_sub_15.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_15, 3000);

    taskPushBack_csMoveToolFrame(right_task_sub_15, {0.001, 0, 0, 0, 0, 0}, 0.005, 0.0025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_15, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_15, {0, 0, -0.006, 0, 0, 0}, 0.005, 0.0025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_15, 100);


    right_task_sub_15.insert(right_task_sub_15.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
    // taskPushBack_delay_1ms(right_task_sub_15, 2000);


    // 케미컬 호스 체결 작업 (총 0.009 가량 down)
    auto subTaskScrewingChemicalHose = [&] (std::vector<UnitTask>& task_in, int trial_cnt) {

        task_in.insert(task_in.end(), drum_grp_screwing_pose_task_.begin(), drum_grp_screwing_pose_task_.end());
        // ROS_LOG_WARN("Chemical Hose Screwing - Trial: %d / 60", trial_cnt + 1);

    };

    // for (int k = 0; k < 14; k++) { // 너무 과도하게 조임
    // k 12 에 k== 9 가 현재 최적 (0331 1110am)
    // for (int k = 0; k < 10; k++) { // 기존
    // k 9 , k< 7 : 수요일 요청 사항
    for (int k = 0; k < 10; k++) { // 수요일 요청사항

        if(k < 17) { // 기존
            // if (k % 2 == 0) {
                // double z_scale_down = 0.000266667;         ///// 총 횟수%2
                double z_scale_down = 0.000266667;         ///// 총 횟수%2
                taskPushBack_csMoveToolFrame(right_task_sub_15, {z_scale_down, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // z축 이동
            // }
        }

        // if(k == 9) {
        //     taskPushBack_drflSetImpedance(right_task_sub_15, true); // Compliance CTRL ON
        //     taskPushBack_delay_1ms(right_task_sub_15, 600);
        // }
        subTaskScrewingChemicalHose(right_task_sub_15, k);
    }


    right_task_sub_15.insert(right_task_sub_15.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    taskPushBack_delay_1ms(right_task_sub_15, 3500);

    taskPushBackKORASGripperCmd(right_task_sub_15, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_15, 60);
    taskPushBackKORASGripperCmd(right_task_sub_15, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -9500);
    taskPushBack_delay_1ms(right_task_sub_15, 3500);

    
    taskPushBack_csMoveToolFrame(right_task_sub_15, {0, 0, -0.003, 0, 0, 0}, 0.01, 0.005, true);
    // taskPushBack_delay_1ms(right_task_sub_15, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_15, {-0.02, 0, 0, 0, 0, 0}, 0.01, 0.005, true);
    // taskPushBack_delay_1ms(right_task_sub_15, 60);
    // taskPushBack_csMoveToolFrame(right_task_sub_15, {0, 0, -0.001, 0, 0, 0}, 0.01, 0.005, true);
    // taskPushBack_delay_1ms(right_task_sub_15, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_15, {-0.11, 0, 0, 0, 0, 0}, 0.1, 0.1, true);
    // taskPushBack_delay_1ms(right_task_sub_15, 20);
    taskPushBack_csMoveToolFrame(right_task_sub_15, {-0.1, 0, 0, 0, 0, 0}, 0.1, 0.1, true);
    // taskPushBack_delay_1ms(right_task_sub_15, 20);

    taskPushBack_csMove2(right_task_sub_15, {0, -0.2, 0, 0, 0, 0}, 0.05, 0.05, true); // false: absolute, true: relative
    // taskPushBack_delay_1ms(right_task_sub_15, 200);

    /// 스캔 전 경유자세 (sub right home pose)
    taskPushBack_jsMove2(right_task_sub_15, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_15, 60);


    return right_task_sub_15;
}

std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask16Task() {
    std::vector<UnitTask> right_task_sub_16;

    taskPushBack_jsMove2(right_task_sub_16, right_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_16, 60);

    right_task_sub_16.insert(right_task_sub_16.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_16, 4000);

    ////////////////////////////////////////////////////////////////////////////////////
    //// 파지 시에, 팁이 걸릴 경우 아래 값으로 조절: 현재: -3000
    taskPushBackKORASGripperCmd(right_task_sub_16, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_16, 60);
    taskPushBackKORASGripperCmd(right_task_sub_16, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000);
    taskPushBack_delay_1ms(right_task_sub_16, 500);
    ////////////////////////////////////////////////////////////////////////////////////

    /// holder without coupler 촬영

    JsDouble scan_pose_target = {4.942, -7.319, 86.477, 39.470, 80.375, -213.474};

    taskPushBack_jsMove2(right_task_sub_16, scan_pose_target, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_16, 500);

    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_16, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_16, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_16, "right_holder_without_coupler"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_16, 600);
    taskPushBack_taskMatchingTargetObject(right_task_sub_16, "right_holder_without_coupler", false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_16, 1000);

    /// unscrewing coupler 촬영
    if(1) {
        //// right_holder_with_coupler
        JsDouble scan_pose_target = {31.644, -11.474, 99.474, 33.409, 73.069, -153.435};

        taskPushBack_jsMove2(right_task_sub_16, scan_pose_target, 30.0, 30.0, false);
        taskPushBack_delay_1ms(right_task_sub_16, 500);

        //// Scanning
        taskPushBack_drflSetTCP(right_task_sub_16, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_16, 60);
        taskPushBack_task3DScanningTargetObject(right_task_sub_16, "right_drum_coupler_unscrewing"); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_16, 600);
        taskPushBack_taskMatchingTargetObject(right_task_sub_16, "right_drum_coupler_unscrewing", false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_16, 1000);

        taskPushBack_drflSetTCP(right_task_sub_16, "tcp03"); // TCP for drum_lid_cap_screwing
        taskPushBack_delay_1ms(right_task_sub_16, 1000);

        ///// 싱귤러 대비 경유 자세
        taskPushBack_jsMove2(right_task_sub_16, {72.382, -13.048, 121.957, 64.492, -20.119, -154.690}, 30.0, 30.0, false);
        taskPushBack_delay_1ms(right_task_sub_16, 500);

        /////////////////////// 인식 자세 이동
        taskPushBack_doGrasping_sharedTask(right_task_sub_16, 0.05, 0.05, "drum_task", "right_drum_coupler_unscrewing"); // Move to grasping pose, (acc, vel)
        // taskPushBack_delay_1ms(right_task_sub_16, 60);
    
        taskPushBack_csMoveToolFrame(right_task_sub_16, {0, 0, 0.135, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_16, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_16, {0, 0, 0, 0, 3, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_16, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_16, {0.020, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_16, 60);


        //// 상대 모션, 파지 중에 걸릴 경우, 아래를 수정
        taskPushBack_csMoveToolFrame(right_task_sub_16, {0.027, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_16, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_16, {0, 0, 0, 0, 3, 0}, 0.02, 0.01, true);
        taskPushBack_delay_1ms(right_task_sub_16, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_16, {0, 0, 0.001, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_16, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_16, {0.020, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_16, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_16, {0.060, 0, 0, 0, 0, 0}, 0.04, 0.02, true);
        taskPushBack_delay_1ms(right_task_sub_16, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_16, {0.038, 0, 0, 0, 0, 0}, 0.04, 0.02, true);
        taskPushBack_delay_1ms(right_task_sub_16, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_16, {0, 0, 0, 0, -4, 0}, 0.02, 0.01, true);
        taskPushBack_delay_1ms(right_task_sub_16, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_16, {0, 0, 0.011, 0, 0, 0}, 0.01, 0.005, true);
        taskPushBack_delay_1ms(right_task_sub_16, 60);

        auto subTaskUnScrewingChemicalHose = [&] (std::vector<UnitTask>& task_in, int trial_cnt) {
    
            task_in.insert(task_in.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    
        };
    
        taskPushBack_drflSetImpedance(right_task_sub_16, true); // Compliance CTRL OFF
        taskPushBack_delay_1ms(right_task_sub_16, 600);
        /////////////////////////////////////////////////////////////////////////////////
        ///// 현재 k = 13 , k > 11 최적
        for (int k = 0; k < 11; k++) {
    
            // if(k > 11) {
            //     // if (k % 2 == 0) {
            //         // double z_scale_down = 0.000266667;         ///// 총 횟수%2
            //         double z_scale_down = 0.000266667;         ///// 총 횟수%2
            //         taskPushBack_csMoveToolFrame(right_task_sub_16, {-z_scale_down, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // z축 이동
            //     // }
            // }
    
            subTaskUnScrewingChemicalHose(right_task_sub_16, k);
        }
    
        /////////////////////////////////////////////////////////////////////////////////
        taskPushBack_drflSetImpedance(right_task_sub_16, false); // Compliance CTRL OFF
        taskPushBack_delay_1ms(right_task_sub_16, 600);
        /////////////////////////////////////////////////////////////////////////////////
    
        right_task_sub_16.insert(right_task_sub_16.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
        taskPushBack_delay_1ms(right_task_sub_16, 100);
    
        taskPushBack_csMoveToolFrame(right_task_sub_16, {-0.002, 0, 0, 0, 0, 0}, 0.05, 0.025, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_16, 50);
    
        taskPushBack_csMoveToolFrame(right_task_sub_16, {0, 0, -0.0005, 0, -1, 0}, 0.01, 0.005, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_16, 60);
    
        right_task_sub_16.insert(right_task_sub_16.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
        taskPushBack_delay_1ms(right_task_sub_16, 100);
    
        // taskPushBack_setRobotAutoMode(right_task_sub_16);
    
        taskPushBack_csMove2(right_task_sub_16, {0, 0, 0.05, 0, 0, 0}, 0.02, 0.01, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_16, 50);
        taskPushBack_csMove2(right_task_sub_16, {0, 0, 0.100, 0, 0, 0}, 0.7, 0.7, true); // true: relative
        // taskPushBack_delay_1ms(right_task_sub_16, 50);
    
        taskPushBack_csMove2(right_task_sub_16, {0, -0.3, 0, 0, 0, 0}, 0.7, 0.7, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(right_task_sub_16, 500);


        ///// holder without coupler 진입 전 경유 자세
        taskPushBack_jsMove2(right_task_sub_16, {4.942, -7.319, 86.477, 39.470, 80.375, -213.474}, 30.0, 30.0, false);
        taskPushBack_delay_1ms(right_task_sub_16, 500);

    }

    /////////////////////// 인식 자세 이동
    taskPushBack_doGrasping_sharedTask(right_task_sub_16, 0.05, 0.05, "drum_task", "right_holder_without_coupler"); // Move to grasping pose, (acc, vel)
    // taskPushBack_delay_1ms(right_task_sub_16, 60);


    // 기울이는 모션
    taskPushBack_csMoveToolFrame(right_task_sub_16, {0, 0, 0.008, 0, 0, 0}, 0.01, 0.005, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(right_task_sub_16, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_16, {0, 0.001, 0, 0, 0, 0}, 0.005, 0.0025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_16, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_16, {0.050, 0, 0, 0, 0, 0}, 0.02, 0.02, true); // true: relative

    taskPushBack_csMoveToolFrame(right_task_sub_16, {0.040, 0, 0, 0, 0, 0}, 0.02, 0.02, true); // true: relative
    taskPushBack_csMoveToolFrame(right_task_sub_16, {0.015, 0, 0, 0, 0, 0}, 0.005, 0.0025, true); // true: relative


    taskPushBack_csMoveToolFrame(right_task_sub_16, {0.02, 0, 0, 0, 0, 0}, 0.005, 0.0025, true); // true: relative

    right_task_sub_16.insert(right_task_sub_16.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_16, 3000);

    taskPushBackKORASGripperCmd(right_task_sub_16, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_16, 60);
    taskPushBackKORASGripperCmd(right_task_sub_16, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000);
    taskPushBack_delay_1ms(right_task_sub_16, 500);

    taskPushBack_csMoveToolFrame(right_task_sub_16, {0, 0, -0.003, 0, 0, 0}, 0.01, 0.005, true);
    // taskPushBack_delay_1ms(right_task_sub_16, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_16, {-0.020, 0, 0, 0, 0, 0}, 0.01, 0.005, true);
    // taskPushBack_delay_1ms(right_task_sub_16, 60);
    // taskPushBack_csMoveToolFrame(right_task_sub_16, {0, 0, -0.001, 0, 0, 0}, 0.01, 0.005, true);
    // taskPushBack_delay_1ms(right_task_sub_16, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_16, {-0.13, 0, 0, 0, 0, 0}, 0.1, 0.1, true);
    // taskPushBack_delay_1ms(right_task_sub_16, 20);
    taskPushBack_csMoveToolFrame(right_task_sub_16, {0, 0, -0.050, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_csMoveToolFrame(right_task_sub_16, {0, 0, -0.250, 0, 0, 0}, 0.1, 0.1, true);


    /// 스캔 전 경유자세 (sub right home pose)
    taskPushBack_jsMove2(right_task_sub_16, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_16, 60);

    return right_task_sub_16;
}

std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask17Task() {
    std::vector<UnitTask> right_task_sub_17;

    if(1) // 별모양 뚜껑 제거
    {
        // setup
        taskPushBack_jsMove2(right_task_sub_17, right_home_pose_, 120.0, 120.0, false);
        // taskPushBack_delay_1ms(right_task_sub_17, 60);

        right_task_sub_17.insert(right_task_sub_17.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
        taskPushBack_delay_1ms(right_task_sub_17, 2000);

        JsDouble scan_pose_hole_star = {31.644, -11.474, 99.474, 33.409, 73.069, -153.435};
        taskPushBack_jsMove2(right_task_sub_17, scan_pose_hole_star, 120.0, 120.0, false);
        // taskPushBack_delay_1ms(right_task_sub_17, 60);

        right_task_sub_17.insert(right_task_sub_17.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(right_task_sub_17, 500);

        /////////////////////////////////////////////////////////////
        std::string target_object = "right_drum_lid_cap_unscrewing";  
        std::string target_task = "default";

        //// Scanning
        taskPushBack_drflSetTCP(right_task_sub_17, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_17, 60);

        taskPushBack_task3DScanningTargetObject(right_task_sub_17, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_17, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(right_task_sub_17, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_17, 1000);

        taskPushBack_drflSetTCP(right_task_sub_17, "tcp05"); // new gripper
        taskPushBack_delay_1ms(right_task_sub_17, 60);

        //// 천장 부딪히지 않도록 경유지 설정
        if(1) {
            JsDouble waypoint_1 = {39.638, -8.522, 106.590, -0.193, 82.717, -133.691};
            taskPushBack_jsMove2(right_task_sub_17, waypoint_1, 10.0, 10.0, false);
            // taskPushBack_delay_1ms(right_task_sub_17, 50);
            taskPushBack_csMove2(right_task_sub_17, {0, 0.4, 0, 0, 0, 0}, 0.7, 0.7, true); // false: absolute, true: relative
            // taskPushBack_delay_1ms(right_task_sub_17, 50);
        }

        taskPushBack_doGrasping_sharedTask(right_task_sub_17, 0.1, 0.1, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(right_task_sub_17, 60);


        taskPushBack_csMoveToolFrame(right_task_sub_17, {0, 0, 0.055, 0, 0, 0}, 0.05, 0.1, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(right_task_sub_17, 50);


        for (int cycle = 0; cycle < 3; cycle++) {
            right_task_sub_17.insert(right_task_sub_17.end(), drum_grp_lid_cap_close_.begin(), drum_grp_lid_cap_close_.end());
            taskPushBack_delay_1ms(right_task_sub_17, 3000);
            taskPushBack_jsMove2(right_task_sub_17, {0, 0, 0, 0, 0, -195}, 10.0, 10.0, true);
            right_task_sub_17.insert(right_task_sub_17.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
            taskPushBack_delay_1ms(right_task_sub_17, 1000);
            taskPushBack_jsMove2(right_task_sub_17, {0, 0, 0, 0, 0, 15}, 10.0, 10.0, true);
            // taskPushBack_delay_1ms(right_task_sub_17, 60);
            taskPushBack_csMoveToolFrame(right_task_sub_17, {0, 0, -0.050, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
            // taskPushBack_delay_1ms(right_task_sub_17, 50);
            taskPushBack_jsMove2(right_task_sub_17, {0, 0, 0, 0, 0, 180}, 10.0, 10.0, true);
            taskPushBack_csMoveToolFrame(right_task_sub_17, {0, 0, 0.050, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
            // taskPushBack_delay_1ms(right_task_sub_17, 50);
        }

        taskPushBack_delay_1ms(right_task_sub_17, 1000);
        right_task_sub_17.insert(right_task_sub_17.end(), drum_grp_lid_cap_close_tight.begin(), drum_grp_lid_cap_close_tight.end());
        taskPushBack_delay_1ms(right_task_sub_17, 4000);

        taskPushBackKORASGripperCmd(right_task_sub_17, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(right_task_sub_17, 60);
        taskPushBackKORASGripperCmd(right_task_sub_17, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -3000);
        taskPushBack_delay_1ms(right_task_sub_17, 500);

        taskPushBack_jsMove2(right_task_sub_17, {0, 0, 0, 0, 0, -30}, 10.0, 10.0, true);
        // taskPushBack_delay_1ms(right_task_sub_17, 500);
        taskPushBack_csMove2(right_task_sub_17, {0, 0, 0.02, 0, 0, 0}, 0.025, 0.05, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(right_task_sub_17, 500);
        taskPushBack_csMove2(right_task_sub_17, {0, 0, 0.02, 0, 0, 0}, 0.2, 0.2, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(right_task_sub_17, 500);
        taskPushBack_csMove2(right_task_sub_17, {0, -0.3, 0, 0, 0, 0}, 0.5, 0.5, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(right_task_sub_17, 50);
    }

    taskPushBack_jsMove2(right_task_sub_17, right_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_17, 60);

    return right_task_sub_17;
}

std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask18Task() {
    std::vector<UnitTask> right_task_sub_18;

    if(1) // 별모양 뚜껑 거치대에 놓기
    {
        std::string target_object = "right_holder_without_lid_cap";
        std::string target_tcp = "tcp05";
        std::string target_task = "default";

        /// 스캔 전 경유자세 (sub right home pose)
        taskPushBack_jsMove2(right_task_sub_18, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 10.0, 10.0, false);
        taskPushBack_delay_1ms(right_task_sub_18, 60);

        taskPushBack_csMove2(right_task_sub_18, {0, 0.300, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_18, 60);

        taskPushBack_drflSetTCP(right_task_sub_18, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_18, 60);

        //// Scanning
        taskPushBack_task3DScanningTargetObject(right_task_sub_18, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_18, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(right_task_sub_18, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_18, 1000);

        taskPushBack_drflSetTCP(right_task_sub_18, target_tcp); // new gripper
        taskPushBack_delay_1ms(right_task_sub_18, 60);

        taskPushBack_doGrasping_sharedTask(right_task_sub_18, 0.05, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(right_task_sub_18, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_18, {0, -0.002, 0.025, 0, 0, 0}, 0.05, 0.05, true);    
        taskPushBack_delay_1ms(right_task_sub_18, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_18, {0, 0, 0.015, 0, 0, 0}, 0.025, 0.025, true);    
        taskPushBack_delay_1ms(right_task_sub_18, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_18, {0, 0, 0.015, 0, 0, 0}, 0.025, 0.025, true);    
        taskPushBack_delay_1ms(right_task_sub_18, 60);

        right_task_sub_18.insert(right_task_sub_18.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(right_task_sub_18, 2500);


        taskPushBack_csMoveToolFrame(right_task_sub_18, {0, 0, -0.022, 0, 0, 0}, 0.5, 0.5, true);
        taskPushBack_delay_1ms(right_task_sub_18, 60);

        taskPushBack_csMove2(right_task_sub_18, {0, -0.300, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_18, 60);

        /// (sub right home pose)
        taskPushBack_jsMove2(right_task_sub_18, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 10.0, 10.0, false);
        taskPushBack_delay_1ms(right_task_sub_18, 60);
    }

    return right_task_sub_18;
}

std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask19Task() {
    std::vector<UnitTask> right_task_sub_19;
    
    taskPushBack_jsMove2(right_task_sub_19, right_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_19, 60);

    right_task_sub_19.insert(right_task_sub_19.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_19, 4000);

    ////////////////////////////////////////////////////////////////////////////////////
    //// 파지 시에, 팁이 걸릴 경우 아래 값으로 조절: 현재: -3000
    taskPushBackKORASGripperCmd(right_task_sub_19, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_19, 60);
    taskPushBackKORASGripperCmd(right_task_sub_19, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000);
    taskPushBack_delay_1ms(right_task_sub_19, 500);
    ////////////////////////////////////////////////////////////////////////////////////

    if(1) {
        ///// hole scan pose
        JsDouble scan_pose_target = {31.644, -11.474, 99.474, 33.409, 73.069, -153.435};
        taskPushBack_jsMove2(right_task_sub_19, scan_pose_target, 30.0, 30.0, false);

        //// Scanning
        taskPushBack_drflSetTCP(right_task_sub_19, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_19, 60);
        taskPushBack_task3DScanningTargetObject(right_task_sub_19, "right_drum_hole_surface"); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_19, 600);
        taskPushBack_taskMatchingTargetObject(right_task_sub_19, "right_drum_hole_surface", false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_19, 1000);

        taskPushBack_drflSetTCP(right_task_sub_19, "tcp03"); // TCP for drum_lid_cap_screwing
        taskPushBack_delay_1ms(right_task_sub_19, 1000);

    }

    if(1) {
        //// right_holder_with_coupler
        JsDouble scan_pose_target = {4.942, -7.319, 86.477, 39.470, 80.375, -213.474};

        taskPushBack_jsMove2(right_task_sub_19, scan_pose_target, 30.0, 30.0, false);
        taskPushBack_delay_1ms(right_task_sub_19, 500);

        //// Scanning
        taskPushBack_drflSetTCP(right_task_sub_19, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_19, 60);
        taskPushBack_task3DScanningTargetObject(right_task_sub_19, "right_holder_with_coupler"); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_19, 600);
        taskPushBack_taskMatchingTargetObject(right_task_sub_19, "right_holder_with_coupler", false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_19, 1000);

        taskPushBack_drflSetTCP(right_task_sub_19, "tcp03"); // TCP for drum_lid_cap_screwing
        taskPushBack_delay_1ms(right_task_sub_19, 1000);

        /////////////////////// 인식 자세 이동
        taskPushBack_doGrasping_sharedTask(right_task_sub_19, 0.05, 0.05, "drum_task", "right_holder_with_coupler"); // Move to grasping pose, (acc, vel)
        // taskPushBack_delay_1ms(right_task_sub_19, 60);

        //// 상대 모션, 파지 중에 걸릴 경우, 아래를 수정
        taskPushBack_csMoveToolFrame(right_task_sub_19, {0.050, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_19, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_19, {0, -0.002, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_19, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_19, {0, 0, 0.140, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_19, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_19, {0, 0, 0, 0, 3, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_19, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_19, {0.040, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_19, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_19, {0.060, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_19, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_19, {0, 0, 0, 0, 2, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_19, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_19, {0.017, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_19, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_19, {0, 0, 0, 0, -2, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_19, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_19, {0, 0, 0.003, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_19, 60);

        right_task_sub_19.insert(right_task_sub_19.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
        taskPushBack_delay_1ms(right_task_sub_19, 4000);

        taskPushBack_csMoveToolFrame(right_task_sub_19, {-0.030, 0, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
        taskPushBack_csMoveToolFrame(right_task_sub_19, {-0.070, 0, 0, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative

    }

    taskPushBack_csMove2(right_task_sub_19, {-0.200, -0.200, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative

    /// 스캔 전 경유자세 (sub right home pose)
    taskPushBack_jsMove2(right_task_sub_19, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_19, 60);


    /////////////////////// 인식 자세 이동
    taskPushBack_doGrasping_sharedTask(right_task_sub_19, 0.05, 0.05, "drum_task", "right_drum_hole_surface"); // Move to grasping pose, (acc, vel)

    taskPushBack_csMoveToolFrame(right_task_sub_19, {0.050, -0.007, 0.007, 0, 0, 0}, 0.1, 0.1, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_19, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_19, {0, 0, 0, 0, -3, -1}, 0.1, 0.1, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_19, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_19, {0.030, 0.004, -0.006, 0,0, 0}, 0.1, 0.1, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_19, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_19, {0.047 , 0, 0, 0, 0, 0}, 0.01, 0.01, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_19, 100);

    right_task_sub_19.insert(right_task_sub_19.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_19, 3000);

    taskPushBack_csMoveToolFrame(right_task_sub_19, {0.005, 0, -0.002, 0, 0, 0}, 0.005, 0.0025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_19, 100);


    right_task_sub_19.insert(right_task_sub_19.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
    // taskPushBack_delay_1ms(right_task_sub_19, 2000);


    // 케미컬 호스 체결 작업 (총 0.009 가량 down)
    auto subTaskScrewingChemicalHose = [&] (std::vector<UnitTask>& task_in, int trial_cnt) {

        task_in.insert(task_in.end(), drum_grp_screwing_pose_task_.begin(), drum_grp_screwing_pose_task_.end());
        // ROS_LOG_WARN("Chemical Hose Screwing - Trial: %d / 60", trial_cnt + 1);

    };

    // for (int k = 0; k < 14; k++) { // 너무 과도하게 조임
    // k 12 에 k== 9 가 현재 최적 (0331 1110am)
    // for (int k = 0; k < 10; k++) { // 기존
    // k 9 , k< 7 : 수요일 요청 사항
    for (int k = 0; k < 8; k++) { // 수요일 요청사항

        if(k < 12) { // 기존
            // if (k % 2 == 0) {
                // double z_scale_down = 0.000266667;         ///// 총 횟수%2
                double z_scale_down = 0.000266667;         ///// 총 횟수%2
                taskPushBack_csMoveToolFrame(right_task_sub_19, {z_scale_down, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // z축 이동
            // }
        }

        // if(k == 9) {
        //     taskPushBack_drflSetImpedance(right_task_sub_19, true); // Compliance CTRL ON
        //     taskPushBack_delay_1ms(right_task_sub_19, 600);
        // }
        subTaskScrewingChemicalHose(right_task_sub_19, k);
    }


    right_task_sub_19.insert(right_task_sub_19.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    taskPushBack_delay_1ms(right_task_sub_19, 3500);

    taskPushBackKORASGripperCmd(right_task_sub_19, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_19, 60);
    taskPushBackKORASGripperCmd(right_task_sub_19, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -9500);
    taskPushBack_delay_1ms(right_task_sub_19, 3500);

    
    taskPushBack_csMoveToolFrame(right_task_sub_19, {0, 0, -0.003, 0, 0, 0}, 0.01, 0.005, true);
    // taskPushBack_delay_1ms(right_task_sub_19, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_19, {-0.02, 0, 0, 0, 0, 0}, 0.01, 0.005, true);
    // taskPushBack_delay_1ms(right_task_sub_19, 60);
    // taskPushBack_csMoveToolFrame(right_task_sub_19, {0, 0, -0.001, 0, 0, 0}, 0.01, 0.005, true);
    // taskPushBack_delay_1ms(right_task_sub_19, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_19, {-0.11, 0, 0, 0, 0, 0}, 0.1, 0.1, true);
    // taskPushBack_delay_1ms(right_task_sub_19, 20);
    taskPushBack_csMoveToolFrame(right_task_sub_19, {-0.1, 0, 0, 0, 0, 0}, 0.1, 0.1, true);
    // taskPushBack_delay_1ms(right_task_sub_19, 20);

    taskPushBack_csMove2(right_task_sub_19, {0, -0.2, 0, 0, 0, 0}, 0.05, 0.05, true); // false: absolute, true: relative
    // taskPushBack_delay_1ms(right_task_sub_19, 200);

    /// 스캔 전 경유자세 (sub right home pose)
    taskPushBack_jsMove2(right_task_sub_19, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_19, 60);


    return right_task_sub_19;
}

std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask20Task() {
    std::vector<UnitTask> right_task_sub_20;

    taskPushBack_jsMove2(right_task_sub_20, right_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_20, 60);

    return right_task_sub_20;
}

std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask21Task() {
    std::vector<UnitTask> right_task_sub_21;

    taskPushBack_jsMove2(right_task_sub_21, right_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_21, 60);

    right_task_sub_21.insert(right_task_sub_21.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_21, 4000);

    ////////////////////////////////////////////////////////////////////////////////////
    //// 파지 시에, 팁이 걸릴 경우 아래 값으로 조절: 현재: -3000
    taskPushBackKORASGripperCmd(right_task_sub_21, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_21, 60);
    taskPushBackKORASGripperCmd(right_task_sub_21, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000);
    taskPushBack_delay_1ms(right_task_sub_21, 500);
    ////////////////////////////////////////////////////////////////////////////////////

    /// holder without coupler 촬영

    JsDouble scan_pose_target = {4.942, -7.319, 86.477, 39.470, 80.375, -213.474};

    taskPushBack_jsMove2(right_task_sub_21, scan_pose_target, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_21, 500);

    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_21, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_21, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_21, "right_holder_without_coupler"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_21, 600);
    taskPushBack_taskMatchingTargetObject(right_task_sub_21, "right_holder_without_coupler", false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_21, 1000);

    /// unscrewing coupler 촬영
    if(1) {
        //// right_holder_with_coupler
        JsDouble scan_pose_target = {31.644, -11.474, 99.474, 33.409, 73.069, -153.435};

        taskPushBack_jsMove2(right_task_sub_21, scan_pose_target, 30.0, 30.0, false);
        taskPushBack_delay_1ms(right_task_sub_21, 500);

        //// Scanning
        taskPushBack_drflSetTCP(right_task_sub_21, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_21, 60);
        taskPushBack_task3DScanningTargetObject(right_task_sub_21, "right_drum_coupler_unscrewing"); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_21, 600);
        taskPushBack_taskMatchingTargetObject(right_task_sub_21, "right_drum_coupler_unscrewing", false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_21, 1000);

        taskPushBack_drflSetTCP(right_task_sub_21, "tcp03"); // TCP for drum_lid_cap_screwing
        taskPushBack_delay_1ms(right_task_sub_21, 1000);

        ///// 싱귤러 대비 경유 자세
        taskPushBack_jsMove2(right_task_sub_21, {72.382, -13.048, 121.957, 64.492, -20.119, -154.690}, 30.0, 30.0, false);
        taskPushBack_delay_1ms(right_task_sub_21, 500);

        /////////////////////// 인식 자세 이동
        taskPushBack_doGrasping_sharedTask(right_task_sub_21, 0.05, 0.05, "drum_task", "right_drum_coupler_unscrewing"); // Move to grasping pose, (acc, vel)
        // taskPushBack_delay_1ms(right_task_sub_21, 60);
    
        //// 상대 모션, 파지 중에 걸릴 경우, 아래를 수정
        taskPushBack_csMoveToolFrame(right_task_sub_21, {0.050, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_21, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_21, {0, -0.003, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_21, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_21, {0, 0, 0.142, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_21, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_21, {0, 0, 0, 0, 2, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_21, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_21, {0.060, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_21, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_21, {0, 0, 0, 0, 1, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_21, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_21, {0, 0, -0.002, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_21, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_21, {0.055, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_21, 60);
        taskPushBack_csMoveToolFrame(right_task_sub_21, {0, 0, 0, 0, -2, 0}, 0.05, 0.05, true);
        taskPushBack_delay_1ms(right_task_sub_21, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_21, {0, 0, 0.0025, 0, 0, 0}, 0.025, 0.025, true);
        taskPushBack_delay_1ms(right_task_sub_21, 60);


        auto subTaskUnScrewingChemicalHose = [&] (std::vector<UnitTask>& task_in, int trial_cnt) {
    
            task_in.insert(task_in.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    
        };
    
        taskPushBack_drflSetImpedance(right_task_sub_21, true); // Compliance CTRL OFF
        taskPushBack_delay_1ms(right_task_sub_21, 600);
        /////////////////////////////////////////////////////////////////////////////////
        ///// 현재 k = 13 , k > 11 최적
        for (int k = 0; k < 11; k++) {
    
            // if(k > 11) {
            //     // if (k % 2 == 0) {
            //         // double z_scale_down = 0.000266667;         ///// 총 횟수%2
            //         double z_scale_down = 0.000266667;         ///// 총 횟수%2
            //         taskPushBack_csMoveToolFrame(right_task_sub_21, {-z_scale_down, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // z축 이동
            //     // }
            // }
    
            subTaskUnScrewingChemicalHose(right_task_sub_21, k);
        }
    
        /////////////////////////////////////////////////////////////////////////////////
        taskPushBack_drflSetImpedance(right_task_sub_21, false); // Compliance CTRL OFF
        taskPushBack_delay_1ms(right_task_sub_21, 600);
        /////////////////////////////////////////////////////////////////////////////////
    
        right_task_sub_21.insert(right_task_sub_21.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
        taskPushBack_delay_1ms(right_task_sub_21, 100);
    
        taskPushBack_csMoveToolFrame(right_task_sub_21, {-0.002, 0, 0, 0, 0, 0}, 0.05, 0.025, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_21, 50);
    
        taskPushBack_csMoveToolFrame(right_task_sub_21, {0, 0, -0.0005, 0, -1, 0}, 0.01, 0.005, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_21, 60);
    
        right_task_sub_21.insert(right_task_sub_21.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
        taskPushBack_delay_1ms(right_task_sub_21, 100);
        
        taskPushBack_csMove2(right_task_sub_21, {0, 0, 0.05, 0, 0, 0}, 0.02, 0.01, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_21, 50);
        taskPushBack_csMove2(right_task_sub_21, {0, 0, 0.100, 0, 0, 0}, 0.7, 0.7, true); // true: relative
        // taskPushBack_delay_1ms(right_task_sub_21, 50);

        ///// holder without coupler 진입 전 경유 자세
        taskPushBack_jsMove2(right_task_sub_21, {33.288, -12.467, 122.337, -75.089, -53.057, -26.890}, 30.0, 30.0, false);
        taskPushBack_delay_1ms(right_task_sub_21, 500);

    }

    /////////////////////// 인식 자세 이동
    taskPushBack_doGrasping_sharedTask(right_task_sub_21, 0.05, 0.05, "drum_task", "right_holder_without_coupler"); // Move to grasping pose, (acc, vel)
    // taskPushBack_delay_1ms(right_task_sub_21, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_21, {0.030, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_21, 100);


    taskPushBack_csMoveToolFrame(right_task_sub_21, {0.020, -0.007, 0.012, 0, 0, 0}, 0.1, 0.05, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_21, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_21, {0, 0, 0, 0, -1, -1}, 0.05, 0.05, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_21, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_21, {0.030, 0.004, -0.006, 0,0, 0}, 0.5, 0.5, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_21, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_21, {0.030, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_21, 100);
    taskPushBack_csMoveToolFrame(right_task_sub_21, {0.009, 0, 0, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_21, 100);
    
    taskPushBack_csMoveToolFrame(right_task_sub_21, {0, 0, -0.0005, 0, -0.5, 0}, 0.05, 0.05, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_21, 100);

    right_task_sub_21.insert(right_task_sub_21.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_21, 3000);

    taskPushBackKORASGripperCmd(right_task_sub_21, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_21, 60);
    taskPushBackKORASGripperCmd(right_task_sub_21, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000);
    taskPushBack_delay_1ms(right_task_sub_21, 500);

    taskPushBack_csMoveToolFrame(right_task_sub_21, {0, 0, 0, 0, 0.5, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_21, 100);

    taskPushBack_csMoveToolFrame(right_task_sub_21, {0, 0.006, 0, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_21, 100);
    taskPushBack_csMoveToolFrame(right_task_sub_21, {0, 0, -0.001, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_21, 100);
    taskPushBack_csMoveToolFrame(right_task_sub_21, {0, 0, 0, 0, 3, 0}, 0.025, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_21, 100);
    taskPushBack_csMoveToolFrame(right_task_sub_21, {-0.030, 0, 0, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_21, 100);
    taskPushBack_csMoveToolFrame(right_task_sub_21, {0, 0, -0.001, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_21, 100);
    taskPushBack_csMoveToolFrame(right_task_sub_21, {-0.070, 0, 0, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_21, 100);
    taskPushBack_csMoveToolFrame(right_task_sub_21, {0, 0, -0.100, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_21, 100);
            
    taskPushBack_csMove2(right_task_sub_21, {-0.250, 0, 0, 0, 0, 0}, 0.2, 0.2, true); // true: relative
    taskPushBack_delay_1ms(right_task_sub_21, 50);

    /// 스캔 전 경유자세 (right home pose)
    taskPushBack_jsMove2(right_task_sub_21, right_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_21, 60);

    return right_task_sub_21;
}

std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask22Task() {
    std::vector<UnitTask> right_task_sub_22;

    taskPushBack_jsMove2(right_task_sub_22, right_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_22, 60);

    return right_task_sub_22;
}

std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask23Task() {
    std::vector<UnitTask> right_task_sub_23;
    
    taskPushBack_jsMove2(right_task_sub_23, right_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_23, 60);

    right_task_sub_23.insert(right_task_sub_23.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(right_task_sub_23, 2000);
    right_task_sub_23.insert(right_task_sub_23.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
    taskPushBack_delay_1ms(right_task_sub_23, 1000);

    /// 스캔 전 경유자세 (sub right home pose)
    taskPushBack_jsMove2(right_task_sub_23, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);
    taskPushBack_delay_1ms(right_task_sub_23, 60);

    taskPushBack_csMove2(right_task_sub_23, {0, 0.300, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(right_task_sub_23, 60);

    //// Scanning
    taskPushBack_drflSetTCP(right_task_sub_23, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(right_task_sub_23, 60);
    taskPushBack_task3DScanningTargetObject(right_task_sub_23, "right_holder_with_lid_cap"); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_23, 600);
    taskPushBack_taskMatchingTargetObject(right_task_sub_23, "right_holder_with_lid_cap", false, 8); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(right_task_sub_23, 1000);

    taskPushBack_drflSetTCP(right_task_sub_23, "tcp05"); // TCP for drum_lid_cap_screwing
    taskPushBack_delay_1ms(right_task_sub_23, 1000);

    taskPushBack_doGrasping_sharedTask(right_task_sub_23, 0.05, 0.05, "default", "right_holder_with_lid_cap"); // Move to grasping pose, (acc, vel)
    taskPushBack_delay_1ms(right_task_sub_23, 60);

    taskPushBack_csMoveToolFrame(right_task_sub_23, {0, 0 , 0.070, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_23, 60);
    taskPushBack_csMoveToolFrame(right_task_sub_23, {0, -0.003 , 0, 0, 0, 0}, 0.05, 0.05, true);
    taskPushBack_delay_1ms(right_task_sub_23, 60);
    //// 파지
    right_task_sub_23.insert(right_task_sub_23.end(), drum_grp_lid_cap_close_tight.begin(), drum_grp_lid_cap_close_tight.end());
    taskPushBack_delay_1ms(right_task_sub_23, 4000);

    taskPushBackKORASGripperCmd(right_task_sub_23, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_23, 60);
    taskPushBackKORASGripperCmd(right_task_sub_23, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -3500);
    taskPushBack_delay_1ms(right_task_sub_23, 3000);

    taskPushBack_csMoveToolFrame(right_task_sub_23, {0, 0, -0.048, 0, 0, 0}, 0.03, 0.015, true);

    // taskPushBack_setRobotAutoMode(right_task_sub_23);

    taskPushBack_csMove2(right_task_sub_23, {0, -0.450, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative

    //// 경유 자세
    taskPushBack_jsMove2(right_task_sub_23, {30.512, -11.813, 120.449, 73.255, 55.786, -186.929}, 30.0, 30.0, false);

    if(1){
        /////////////////////////////////////////////////////////////////////////
        std::string target_tcp = "tcp05"; // screwing new gripper tcp
        std::string target_task = "default";
        JsDouble scan_pose_target = {31.644, -11.474, 99.474, 33.409, 73.069, -153.435};

        //// lid cap screwing scan pose
        taskPushBack_jsMove2(right_task_sub_23, scan_pose_target, 30.0, 30.0, false);
        taskPushBack_delay_1ms(right_task_sub_23, 60);

        taskPushBack_drflSetTCP(right_task_sub_23, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_23, 60);

        taskPushBack_task3DScanningTargetObject(right_task_sub_23, "right_drum_lid_cap_screwing"); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_23, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(right_task_sub_23, "right_drum_lid_cap_screwing", false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(right_task_sub_23, 1000);

        taskPushBack_drflSetTCP(right_task_sub_23, target_tcp); // new gripper
        taskPushBack_delay_1ms(right_task_sub_23, 60);

        //// 천장 부딪히지 않도록 경유지 설정
        if(1) {
            JsDouble waypoint_1 = {39.638, -8.522, 106.590, -0.193, 82.717, -313.690};
            taskPushBack_jsMove2(right_task_sub_23, waypoint_1, 30.0, 30.0, false);
            // taskPushBack_delay_1ms(right_task_sub_23, 50);
            taskPushBack_csMove2(right_task_sub_23, {0, 0.400, 0, 0, 0, 0}, 0.2, 0.2, true); // false: absolute, true: relative
            // taskPushBack_delay_1ms(right_task_sub_23, 50);
        }

        /////////////////////// 인식 자세 이동
        taskPushBack_doGrasping_sharedTask(right_task_sub_23, 0.1, 0.05, target_task, "right_drum_lid_cap_screwing"); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(right_task_sub_23, 60);

        // taskPushBack_csMoveToolFrame(right_task_sub_23, {-0.004, 0, 0, 0, 0, 0}, 0.02, 0.04, true); // true: relative
        taskPushBack_csMoveToolFrame(right_task_sub_23, {0, -0.004, 0, 0, 0, 0}, 0.02, 0.04, true); // true: relative

        taskPushBack_csMoveToolFrame(right_task_sub_23, {0, 0, 0.026, 0, 0, 0}, 0.1, 0.1, true); // true: relative
        // taskPushBack_delay_1ms(right_task_sub_23, 60);

        ////////0403 test
        taskPushBack_csMoveToolFrame(right_task_sub_23, {0, 0, 0.005, 0, 0, 10}, 0.025, 0.025, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_23, 60);

        right_task_sub_23.insert(right_task_sub_23.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(right_task_sub_23, 1000);
        right_task_sub_23.insert(right_task_sub_23.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(right_task_sub_23, 1000);
        taskPushBack_jsMove2(right_task_sub_23, {0, 0, 0, 0, 0, -10}, 15.0, 15.0, true);

        taskPushBack_csMoveToolFrame(right_task_sub_23, {0, 0, 0.012, 0, 0, 0}, 0.02, 0.04, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_23, 60);

        taskPushBack_csMoveToolFrame(right_task_sub_23, {0, 0.004, 0, 0, 0, 0}, 0.02, 0.04, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_23, 60);

        // 토크 가드 ON: J6 기준 15.0 이상이면 정지 (필요 시 조정)
        taskPushBack_torqueGuardOn(right_task_sub_23, 5, 3.0);


        int cnt_lid_cap_screwing = 10;
        for (int cycle = 0; cycle < cnt_lid_cap_screwing; cycle++) {
            right_task_sub_23.insert(right_task_sub_23.end(), drum_grp_lid_cap_close_for_screwing_.begin(), drum_grp_lid_cap_close_for_screwing_.end());


            if(cycle != cnt_lid_cap_screwing - 1) { // 마지막 trial에서는 스킵
                taskPushBack_delay_1ms(right_task_sub_23, 2000);
                taskPushBack_jsMove2(right_task_sub_23, {0, 0, 0, 0, 0, 195}, 40.0, 40.0, true);
                right_task_sub_23.insert(right_task_sub_23.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
                taskPushBack_delay_1ms(right_task_sub_23, 2000);
            } else {
                //// 마지막 횟수에서 더 돌리기 (90도만)
                taskPushBack_delay_1ms(right_task_sub_23, 2000);
                taskPushBack_jsMove2(right_task_sub_23, {0, 0, 0, 0, 0, 135}, 40.0, 40.0, true);
                // taskPushBack_delay_1ms(right_task_sub_23, 20);
                //// 아래에서 더 돌리면 팁 파손 가능성
                taskPushBack_jsMove2(right_task_sub_23, {0, 0, 0, 0, 0, 15}, 15.0, 15.0, true);
                taskPushBack_jsMove2(right_task_sub_23, {0, 0, 0, 0, 0, 5}, 15.0, 15.0, true);
                // taskPushBack_delay_1ms(right_task_sub_23, 20);
                taskPushBack_jsMove2(right_task_sub_23, {0, 0, 0, 0, 0, -10}, 40.0, 40.0, true);
                right_task_sub_23.insert(right_task_sub_23.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
                taskPushBack_delay_1ms(right_task_sub_23, 2000);
                taskPushBack_jsMove2(right_task_sub_23, {0, 0, 0, 0, 0, -5}, 40.0, 40.0, true);
                // taskPushBack_delay_1ms(right_task_sub_23, 20);
            }

            if(cycle != cnt_lid_cap_screwing - 1) { // 마지막 trial에서는 스킵
                taskPushBack_jsMove2(right_task_sub_23, {0, 0, 0, 0, 0, -15}, 40.0, 40.0, true);
                // taskPushBack_delay_1ms(right_task_sub_23, 20);
                taskPushBack_csMoveToolFrame(right_task_sub_23, {0, 0, -0.050, 0, 0, 0}, 0.2, 0.2, true); // false: absolute, true: relative
                // taskPushBack_delay_1ms(right_task_sub_23, 20);
                taskPushBack_jsMove2(right_task_sub_23, {0, 0, 0, 0, 0, -180}, 40.0, 40.0, true);
                taskPushBack_csMoveToolFrame(right_task_sub_23, {0, 0, 0.050, 0, 0, 0}, 0.2, 0.2, true); // false: absolute, true: relative
                taskPushBack_delay_1ms(right_task_sub_23, 20);
            }
        }

        // === 분기 타겟 라벨 ===
        taskPushBack_label(right_task_sub_23, "SCREW_OPEN_STEP");
        taskPushBack_jsMove2(right_task_sub_23, {0, 0, 0, 0, 0, -10}, 40.0, 40.0, true);
        right_task_sub_23.insert(right_task_sub_23.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
        taskPushBack_delay_1ms(right_task_sub_23, 2000);
        taskPushBack_jsMove2(right_task_sub_23, {0, 0, 0, 0, 0, -5}, 40.0, 40.0, true);

        taskPushBack_csMove2(right_task_sub_23, {0, 0, 0.045, 0, 0, 0}, 0.4, 0.4, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_23, 60);

        // 마지막 자세
        taskPushBack_csMove2(right_task_sub_23, {0, -0.4, 0, 0, 0, 0}, 0.4, 0.4, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_23, 60);

        taskPushBack_jsMove2(right_task_sub_23, right_home_pose_, 90.0, 90.0, false);
        taskPushBack_delay_1ms(right_task_sub_23, 60);

        // 토크 가드 OFF
        taskPushBack_torqueGuardOff(right_task_sub_23);

    }


    return right_task_sub_23;
}

std::vector<UnitTask> TaskPlannerHanyangEng::rightGenSubTask24Task() {
    std::vector<UnitTask> right_task_sub_24;

    // if(1) {
    //     std::string target_object = "right_holder_with_lid_cap";
    //     std::string target_tcp = "tcp05"; // screwing new gripper tcp
    //     std::string target_task = "default";

    //     JsDouble scan_pose_target = {70.875, -12.106, 116.590, -63.188, 29.224, -11.559};
    //     taskPushBack_jsMove2(right_task_sub_24, scan_pose_target, 90.0, 90.0, false);
    //     taskPushBack_delay_1ms(right_task_sub_24, 60);
    //     //// Scanning
    //     taskPushBack_drflSetTCP(right_task_sub_24, "tcp00"); // Scan & Matching은 tcp00
    //     taskPushBack_delay_1ms(right_task_sub_24, 60);

    //     taskPushBack_task3DScanningTargetObject(right_task_sub_24, target_object); // ZIVID Scanning with task recognition
    //     taskPushBack_delay_1ms(right_task_sub_24, 600);
    //     //// Matching
    //     taskPushBack_taskMatchingTargetObject(right_task_sub_24, target_object, false, 8); // ZIVID Scanning with task recognition
    //     taskPushBack_delay_1ms(right_task_sub_24, 1000);
    //     taskPushBack_drflSetTCP(right_task_sub_24, target_tcp); // new gripper
    //     taskPushBack_delay_1ms(right_task_sub_24, 60);


    //     if(1) {
    //         JsDouble waypoint_1 = {71.134, -7.479, 104.503, -28.592, 45.140, -136.637};
    //         taskPushBack_jsMove2(right_task_sub_24, waypoint_1, 90.0, 90.0, false);
    //         taskPushBack_csMove2(right_task_sub_24, {0.2, 0.2, 0.05, 0, 0, 0}, 0.7, 0.7, true); // false: absolute, true: relative
    //     }

    //     /////////////////////// 인식 자세 이동
    //     taskPushBack_doGrasping_sharedTask(right_task_sub_24, 0.05, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
    //     taskPushBack_delay_1ms(right_task_sub_24, 60);
    //     /////////////////////// 인식 자세 이동

    // }

    //// 파지
    // taskPushBack_csMoveToolFrame(right_task_sub_24, {0, -0.002, 0.057, 0, 0, 0}, 0.1, 0.1, true);
    // taskPushBack_delay_1ms(right_task_sub_24, 60);

    right_task_sub_24.insert(right_task_sub_24.end(), drum_grp_lid_cap_close_tight.begin(), drum_grp_lid_cap_close_tight.end());
    taskPushBack_delay_1ms(right_task_sub_24, 4000);

    taskPushBackKORASGripperCmd(right_task_sub_24, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(right_task_sub_24, 60);
    taskPushBackKORASGripperCmd(right_task_sub_24, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -1500);
    taskPushBack_delay_1ms(right_task_sub_24, 500);

    // taskPushBack_csMoveToolFrame(right_task_sub_24, {0, 0, -0.048, 0, 0, 0}, 0.03, 0.015, true);
    
    // taskPushBack_csMove2(right_task_sub_24, {-0.2, -0.2, -0.05, 0, 0, 0}, 0.7, 0.7, true); // false: absolute, true: relative

    // taskPushBack_setRobotAutoMode(right_task_sub_24);

    if(1){
        /////////////////////////////////////////////////////////////////////////
        std::string target_object = "right_drum_lid_cap_screwing";
        std::string target_tcp = "tcp05"; // screwing new gripper tcp
        std::string target_task = "default";
        JsDouble scan_pose_target = {31.644, -11.474, 99.474, 33.409, 73.069, -153.435};

        taskPushBack_jsMove2(right_task_sub_24, scan_pose_target, 120.0, 120.0, false);
        taskPushBack_delay_1ms(right_task_sub_24, 60);

        taskPushBack_drflSetTCP(right_task_sub_24, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(right_task_sub_24, 60);

        // taskPushBack_task3DScanningTargetObject(right_task_sub_24, target_object); // ZIVID Scanning with task recognition
        // taskPushBack_delay_1ms(right_task_sub_24, 600);
        // //// Matching
        // taskPushBack_taskMatchingTargetObject(right_task_sub_24, target_object, false, 8); // ZIVID Scanning with task recognition
        // taskPushBack_delay_1ms(right_task_sub_24, 1000);

        taskPushBack_drflSetTCP(right_task_sub_24, target_tcp); // new gripper
        taskPushBack_delay_1ms(right_task_sub_24, 60);

        //// 천장 부딪히지 않도록 경유지 설정
        if(1) {
            JsDouble waypoint_1 = {39.638, -8.522, 106.590, -0.193, 82.717, -313.690};
            taskPushBack_jsMove2(right_task_sub_24, waypoint_1, 120.0, 120.0, false);
            // taskPushBack_delay_1ms(right_task_sub_24, 50);
            taskPushBack_csMove2(right_task_sub_24, {0, 0.400, 0.03, 0, 0, 0}, 0.7, 0.7, true); // false: absolute, true: relative
            // taskPushBack_delay_1ms(right_task_sub_24, 50);
        }

        /////////////////////// 인식 자세 이동
        taskPushBack_doGrasping_sharedTask(right_task_sub_24, 0.1, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(right_task_sub_24, 60);

        // taskPushBack_csMoveToolFrame(right_task_sub_24, {-0.004, 0, 0, 0, 0, 0}, 0.02, 0.04, true); // true: relative
        taskPushBack_csMoveToolFrame(right_task_sub_24, {0, -0.004, 0, 0, 0, 0}, 0.02, 0.04, true); // true: relative

        taskPushBack_csMoveToolFrame(right_task_sub_24, {0, 0, 0.017, 0, 0, 0}, 0.1, 0.1, true); // true: relative
        // taskPushBack_delay_1ms(right_task_sub_24, 60);

        ////////0403 test
        taskPushBack_csMoveToolFrame(right_task_sub_24, {0, 0, 0.005, 0, 0, 10}, 0.025, 0.025, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_24, 60);

        right_task_sub_24.insert(right_task_sub_24.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(right_task_sub_24, 1000);
        right_task_sub_24.insert(right_task_sub_24.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(right_task_sub_24, 1000);
        taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, -10}, 15.0, 15.0, true);

        taskPushBack_csMoveToolFrame(right_task_sub_24, {0, 0, 0.012, 0, 0, 0}, 0.02, 0.04, true); // true: relative
        taskPushBack_delay_1ms(right_task_sub_24, 60);

        // 토크 가드 ON: J6 기준 15.0 이상이면 정지 (필요 시 조정)
        taskPushBack_torqueGuardOn(right_task_sub_24, 5, 3.0);


        int cnt_lid_cap_screwing = 10;
        for (int cycle = 0; cycle < cnt_lid_cap_screwing; cycle++) {
            right_task_sub_24.insert(right_task_sub_24.end(), drum_grp_lid_cap_close_for_screwing_.begin(), drum_grp_lid_cap_close_for_screwing_.end());


            if(cycle != cnt_lid_cap_screwing - 1) { // 마지막 trial에서는 스킵
                taskPushBack_delay_1ms(right_task_sub_24, 2000);
                taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, 195}, 40.0, 40.0, true);
                right_task_sub_24.insert(right_task_sub_24.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
                taskPushBack_delay_1ms(right_task_sub_24, 2000);
            } else {
                //// 마지막 횟수에서 더 돌리기 (90도만)
                taskPushBack_delay_1ms(right_task_sub_24, 2000);
                taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, 135}, 40.0, 40.0, true);
                // taskPushBack_delay_1ms(right_task_sub_24, 20);
                //// 아래에서 더 돌리면 팁 파손 가능성
                taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, 15}, 15.0, 15.0, true);
                taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, 5}, 15.0, 15.0, true);
                // taskPushBack_delay_1ms(right_task_sub_24, 20);
                taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, -10}, 40.0, 40.0, true);
                right_task_sub_24.insert(right_task_sub_24.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
                taskPushBack_delay_1ms(right_task_sub_24, 2000);
                taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, -5}, 40.0, 40.0, true);
                // taskPushBack_delay_1ms(right_task_sub_24, 20);
            }

            if(cycle != cnt_lid_cap_screwing - 1) { // 마지막 trial에서는 스킵
                taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, -15}, 40.0, 40.0, true);
                // taskPushBack_delay_1ms(right_task_sub_24, 20);
                taskPushBack_csMoveToolFrame(right_task_sub_24, {0, 0, -0.050, 0, 0, 0}, 0.2, 0.2, true); // false: absolute, true: relative
                // taskPushBack_delay_1ms(right_task_sub_24, 20);
                taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, -180}, 40.0, 40.0, true);
                taskPushBack_csMoveToolFrame(right_task_sub_24, {0, 0, 0.050, 0, 0, 0}, 0.2, 0.2, true); // false: absolute, true: relative
                taskPushBack_delay_1ms(right_task_sub_24, 20);
            }
        }

        // === 분기 타겟 라벨 ===
        taskPushBack_label(right_task_sub_24, "SCREW_OPEN_STEP");
        taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, -10}, 40.0, 40.0, true);
        right_task_sub_24.insert(right_task_sub_24.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
        taskPushBack_delay_1ms(right_task_sub_24, 2000);
        taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, -5}, 40.0, 40.0, true);

        taskPushBack_csMove2(right_task_sub_24, {0, 0, 0.045, 0, 0, 0}, 0.4, 0.4, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_24, 60);

        // 마지막 자세
        taskPushBack_csMove2(right_task_sub_24, {0, -0.4, 0, 0, 0, 0}, 0.4, 0.4, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(right_task_sub_24, 60);

        taskPushBack_jsMove2(right_task_sub_24, right_home_pose_, 90.0, 90.0, false);
        taskPushBack_delay_1ms(right_task_sub_24, 60);

        // 토크 가드 OFF
        taskPushBack_torqueGuardOff(right_task_sub_24);

    // taskPushBack_jsMove2(right_task_sub_24, right_home_pose_, 120.0, 120.0, false);

    // right_task_sub_24.insert(right_task_sub_24.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    // taskPushBack_delay_1ms(right_task_sub_24, 3000);
    // right_task_sub_24.insert(right_task_sub_24.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
    // taskPushBack_delay_1ms(right_task_sub_24, 5000);

    // right_task_sub_24.insert(right_task_sub_24.end(), drum_grp_lid_cap_close_tight.begin(), drum_grp_lid_cap_close_tight.end());
    // taskPushBack_delay_1ms(right_task_sub_24, 5000); // 리드캡 잡기

    // if(1) {
    //     std::string target_object = "right_holder_with_lid_cap";
    //     std::string target_tcp = "tcp05"; // screwing new gripper tcp
    //     std::string target_task = "default";
        

    //     taskPushBack_jsMove2(right_task_sub_24, {71.153, -24.013, 93.094, -3.236, 76.114, -84.919}, 40.0, 40.0, false);
    //     taskPushBack_delay_1ms(right_task_sub_24, 3000); // 임의 스캔 자세

    //     taskPushBack_jsMove2(right_task_sub_24, {68.202, 10.434, 60.916, -0.413, 110.702, -87.745}, 40.0, 40.0, false);
    //     taskPushBack_delay_1ms(right_task_sub_24, 1000); // 4번위치(체결 위치에서 tool Z축 기준 10cm위)로 가기전 경유지

    //     taskPushBack_jsMove2(right_task_sub_24, {75.543, 11.184, 78.427, -0.449, 88.957, -105.438}, 40.0, 40.0, false);
    //     taskPushBack_delay_1ms(right_task_sub_24, 1000); // 체결 위치에서 tool Z축 기준 5cm위
    //     taskPushBack_jsMove2(right_task_sub_24, {75.490, 10.934, 81.815, -0.448, 85.818, -95.466}, 40.0, 40.0, false);
    //     taskPushBack_delay_1ms(right_task_sub_24, 1000); // 체결 위치에서 tool Z축 기준 2cm위
        
    //     taskPushBack_csMoveToolFrame(right_task_sub_24, {0, 0, 0.02, 0, 0, 10}, 0.025, 0.025, true); // true: relative
    //     taskPushBack_delay_1ms(right_task_sub_24, 60); // 5cm 하강
    // }


    // if(1){
    //     /////////////////////////////////////////////////////////////////////////
    //     std::string target_object = "right_drum_lid_cap_screwing";
    //     std::string target_tcp = "tcp05"; // screwing new gripper tcp
    //     std::string target_task = "default";

    //     // 토크 가드 ON: J6 기준 15.0 이상이면 정지 (필요 시 조정)
    //     taskPushBack_torqueGuardOn(right_task_sub_24, 5, 5.0);

    //     int cnt_lid_cap_screwing = 10;
    //     for (int cycle = 0; cycle < cnt_lid_cap_screwing; cycle++) {
    //         right_task_sub_24.insert(right_task_sub_24.end(), drum_grp_lid_cap_close_for_screwing_.begin(), drum_grp_lid_cap_close_for_screwing_.end());


    //         if(cycle != cnt_lid_cap_screwing - 1) { // 마지막 trial에서는 스킵
    //             taskPushBack_delay_1ms(right_task_sub_24, 2000);
    //             taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, 195}, 120.0, 120.0, true);
    //             right_task_sub_24.insert(right_task_sub_24.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
    //             taskPushBack_delay_1ms(right_task_sub_24, 2000);
    //         } else {
    //             //// 마지막 횟수에서 더 돌리기 (90도만)
    //             taskPushBack_delay_1ms(right_task_sub_24, 2000);
    //             taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, 135}, 120.0, 120.0, true);
    //             //// 아래에서 더 돌리면 팁 파손 가능성
    //             taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, 15}, 15.0, 15.0, true);
    //             taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, 5}, 15.0, 15.0, true);
    //             taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, -10}, 120.0, 120.0, true);
    //             right_task_sub_24.insert(right_task_sub_24.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
    //             taskPushBack_delay_1ms(right_task_sub_24, 2000);
    //             taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, -5}, 120.0, 120.0, true);
    //         }

    //         if(cycle != cnt_lid_cap_screwing - 1) { // 마지막 trial에서는 스킵
    //             taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, -15}, 120.0, 120.0, true);
    //             taskPushBack_csMoveToolFrame(right_task_sub_24, {0, 0, -0.050, 0, 0, 0}, 0.7, 0.7, true); // false: absolute, true: relative
    //             taskPushBack_jsMove2(right_task_sub_24, {0, 0, 0, 0, 0, -180}, 120.0, 120.0, true);
    //             taskPushBack_csMoveToolFrame(right_task_sub_24, {0, 0, 0.050, 0, 0, 0}, 0.7, 0.7, true); // false: absolute, true: relative
    //             taskPushBack_delay_1ms(right_task_sub_24, 20);
    //         }
    //     }

    //     // === 분기 타겟 라벨 ===
    //     taskPushBack_label(right_task_sub_24, "SCREW_OPEN_STEP");
    //     right_task_sub_24.insert(right_task_sub_24.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
    //     taskPushBack_delay_1ms(right_task_sub_24, 2000);

    //     taskPushBack_jsMove2(right_task_sub_24, {71.174, 11.109, 73.339, -0.344, 94.094, -106.661}, 60.0, 60.0, false);
    //     taskPushBack_delay_1ms(right_task_sub_24, 3000);

    //     // 마지막 자세
    //     taskPushBack_jsMove2(right_task_sub_24, {64.286, -6.836, 92.500, -0.086, 92.857, -113.521}, 60.0, 60.0, false);
    //     taskPushBack_delay_1ms(right_task_sub_24, 3000);
        

    //     taskPushBack_jsMove2(right_task_sub_24, right_home_pose_, 120.0, 120.0, false);
    //     taskPushBack_delay_1ms(right_task_sub_24, 60);

    //     // 토크 가드 OFF
    //     taskPushBack_torqueGuardOff(right_task_sub_24);


    }

    return right_task_sub_24;
}
///////////////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////


std::vector<UnitTask> TaskPlannerHanyangEng::leftGenSubTask1Task() {
    std::vector<UnitTask> left_task_sub_1;


    if(1) // 별모양 뚜껑 제거
    {
        taskPushBack_jsMove2(left_task_sub_1, left_home_pose_, 30.0, 30.0, false);
        taskPushBack_delay_1ms(left_task_sub_1, 60);

        left_task_sub_1.insert(left_task_sub_1.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
        taskPushBack_delay_1ms(left_task_sub_1, 2000);

        JsDouble left_scan_pose_hole_star = {-24.430, 11.354, -105.873, 141.652, 71.717, -15.381};
        taskPushBack_jsMove2(left_task_sub_1, left_scan_pose_hole_star, 75.0, 75.0, false);
        taskPushBack_delay_1ms(left_task_sub_1, 60);


        left_task_sub_1.insert(left_task_sub_1.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(left_task_sub_1, 500);

        // left_task_sub_1.insert(left_task_sub_1.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
        // taskPushBack_delay_1ms(left_task_sub_1, 500);

        /////////////////////////////////////////////////////////////
        std::string target_object = "left_drum_lid_cap_unscrewing";
        std::string target_task = "default";


        //// Scanning
        taskPushBack_drflSetTCP(left_task_sub_1, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(left_task_sub_1, 60);

        taskPushBack_task3DScanningTargetObject(left_task_sub_1, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(left_task_sub_1, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(left_task_sub_1, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(left_task_sub_1, 1000);

        taskPushBack_drflSetTCP(left_task_sub_1, "tcp07"); // left star gripper
        taskPushBack_delay_1ms(left_task_sub_1, 60);

        //// 천장 부딪히지 않도록 경유지 설정
        if(1) {
            // taskPushBack_csMove2(left_task_sub_1, {0.53823, 0.75675, 0.30108, -179.641, 0.762, 82.817}, 0.05, 0.1, false); // false: absolute, true: relative
            // taskPushBack_delay_1ms(left_task_sub_1, 500);
            JsDouble left_waypoint_1 = {-66.444, -5.972, -92.399, 180.705, 82.178, 107.035};
            taskPushBack_jsMove2(left_task_sub_1, left_waypoint_1, 75.0, 75.0, false);
            taskPushBack_delay_1ms(left_task_sub_1, 50);
            taskPushBack_csMove2(left_task_sub_1, {-0.15, 0.15, 0, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(left_task_sub_1, 50);
        }

        taskPushBack_doGrasping_sharedTask(left_task_sub_1, 0.05, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(left_task_sub_1, 60);

        taskPushBack_drflSetTCP(left_task_sub_1, "tcp05"); // new gripper
        taskPushBack_delay_1ms(left_task_sub_1, 60);

        taskPushBack_csMoveToolFrame(left_task_sub_1, {0, 0, 0.040, 0, 0, 0}, 0.1, 0.2, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(left_task_sub_1, 50);


        for (int cycle = 0; cycle < 3; cycle++) {
            left_task_sub_1.insert(left_task_sub_1.end(), drum_grp_lid_cap_close_.begin(), drum_grp_lid_cap_close_.end());
            taskPushBack_delay_1ms(left_task_sub_1, 3000);
            taskPushBack_jsMove2(left_task_sub_1, {0, 0, 0, 0, 0, -195}, 40.0, 40.0, true);
            left_task_sub_1.insert(left_task_sub_1.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
            taskPushBack_delay_1ms(left_task_sub_1, 1000);
            taskPushBack_jsMove2(left_task_sub_1, {0, 0, 0, 0, 0, 15}, 40.0, 40.0, true);
            // taskPushBack_delay_1ms(left_task_sub_1, 60);
            taskPushBack_csMoveToolFrame(left_task_sub_1, {0, 0, -0.050, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
            // taskPushBack_delay_1ms(left_task_sub_1, 50);
            taskPushBack_jsMove2(left_task_sub_1, {0, 0, 0, 0, 0, 180}, 40.0, 40.0, true);
            taskPushBack_csMoveToolFrame(left_task_sub_1, {0, 0, 0.050, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
            // taskPushBack_delay_1ms(left_task_sub_1, 50);
        }

        taskPushBack_delay_1ms(left_task_sub_1, 1000);
        left_task_sub_1.insert(left_task_sub_1.end(), drum_grp_lid_cap_close_tight.begin(), drum_grp_lid_cap_close_tight.end());
        taskPushBack_delay_1ms(left_task_sub_1, 4000);

        taskPushBackKORASGripperCmd(left_task_sub_1, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(left_task_sub_1, 60);
        taskPushBackKORASGripperCmd(left_task_sub_1, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -1500);
        taskPushBack_delay_1ms(left_task_sub_1, 500);

        taskPushBack_csMoveToolFrame(left_task_sub_1, {0, 0, 0, 0, 0, -30.0}, 0.1, 0.2, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(left_task_sub_1, 500);
        taskPushBack_csMove2(left_task_sub_1, {0, 0, 0.02, 0, 0, 0}, 0.025, 0.05, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(left_task_sub_1, 500);
        taskPushBack_csMove2(left_task_sub_1, {0, 0, 0.04, 0, 0, 0}, 0.05, 0.1, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(left_task_sub_1, 500);
        taskPushBack_csMove2(left_task_sub_1, {0, -0.3, 0, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
        // taskPushBack_delay_1ms(left_task_sub_1, 50);

    }


    if(1) // 별모양 뚜껑 거치대에 놓기
    {
        std::string target_object = "left_holder_without_lid_cap";
        std::string target_tcp = "tcp05";
        std::string target_task = "default";
        JsDouble left_scan_pose_target = {-63.620, 7.844, -106.944, 38.263, -27.484, 36.323};

        taskPushBack_jsMove2(left_task_sub_1, left_scan_pose_target, 75.0, 75.0, false);
        taskPushBack_delay_1ms(left_task_sub_1, 60);

        taskPushBack_drflSetTCP(left_task_sub_1, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(left_task_sub_1, 60);

        taskPushBack_task3DScanningTargetObject(left_task_sub_1, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(left_task_sub_1, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(left_task_sub_1, target_object, false, 8); // "default", true: symmetric CAD used
        taskPushBack_delay_1ms(left_task_sub_1, 1000);

        taskPushBack_drflSetTCP(left_task_sub_1, target_tcp); // new gripper
        taskPushBack_delay_1ms(left_task_sub_1, 60);

        if(1) {
            JsDouble waypoint_1 = {-64.736, 27.570, -115.122, 9.968, -39.035, 152.603};
            taskPushBack_jsMove2(left_task_sub_1, waypoint_1, 75.0, 75.0, false);
            taskPushBack_delay_1ms(left_task_sub_1, 60);
            taskPushBack_csMove2(left_task_sub_1, {-0.3, 0.3, 0, 0, 0, 0}, 0.3, 0.6, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(left_task_sub_1, 500);
        }

        taskPushBack_doGrasping_sharedTask(left_task_sub_1, 0.05, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(left_task_sub_1, 60);

        taskPushBack_csMoveToolFrame(left_task_sub_1, {0, 0, 0.025, 0, 0, 0}, 0.04, 0.02, true);
        taskPushBack_delay_1ms(left_task_sub_1, 60);

        taskPushBack_csMoveToolFrame(left_task_sub_1, {0, 0.002, 0.005, 0, 0, 0}, 0.04, 0.02, true);
        taskPushBack_delay_1ms(left_task_sub_1, 60);

        left_task_sub_1.insert(left_task_sub_1.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(left_task_sub_1, 4000);

        taskPushBack_csMoveToolFrame(left_task_sub_1, {0, 0, -0.022, 0, 0, 0}, 0.08, 0.04, true);
        taskPushBack_delay_1ms(left_task_sub_1, 60);

        taskPushBack_csMove2(left_task_sub_1, {0.3, -0.3, 0, 0, 0, 0}, 0.3, 0.6, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(left_task_sub_1, 500);

        /// 경유 자세
        JsDouble left_waypoint_2 = {-66.444, -5.972, -92.399, 180.705, 82.178, 107.035};
        taskPushBack_jsMove2(left_task_sub_1, left_waypoint_2, 75.0, 75.0, false);

        taskPushBack_jsMove2(left_task_sub_1, left_home_pose_, 30.0, 30.0, false);
        taskPushBack_delay_1ms(left_task_sub_1, 60);
    }

    return left_task_sub_1;
}



std::vector<UnitTask> TaskPlannerHanyangEng::leftGenSubTask2Task() {
    std::vector<UnitTask> left_task_sub_2;

    left_task_sub_2.insert(left_task_sub_2.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(left_task_sub_2, 4000);

    taskPushBack_jsMove2(left_task_sub_2, left_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(left_task_sub_2, 60);

    ////////////////////////////////////////////////////////////////////////////////////
    //// 파지 시에, 팁이 걸릴 경우 아래 값으로 조절: 현재: -3000

    taskPushBackKORASGripperCmd(left_task_sub_2, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(left_task_sub_2, 60);
    taskPushBackKORASGripperCmd(left_task_sub_2, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -3000);
    taskPushBack_delay_1ms(left_task_sub_2, 500);
    ////////////////////////////////////////////////////////////////////////////////////

    if(1) { // Detection
        //// left_holder_with_coupler
        std::string target_object = "left_holder_with_coupler";
        std::string target_tcp = "tcp06";
        std::string target_task = "drum_task";
        JsDouble scan_pose_target = {-5.589, 4.486, -86.330, 137.716, 90.786, -60.594};

        taskPushBack_jsMove2(left_task_sub_2, scan_pose_target, 75.0, 75.0, false);
        taskPushBack_delay_1ms(left_task_sub_2, 60);
        //// Scanning
        taskPushBack_drflSetTCP(left_task_sub_2, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(left_task_sub_2, 60);

        taskPushBack_task3DScanningTargetObject(left_task_sub_2, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(left_task_sub_2, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(left_task_sub_2, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(left_task_sub_2, 4000);

        taskPushBack_drflSetTCP(left_task_sub_2, target_tcp); // new gripper
        taskPushBack_delay_1ms(left_task_sub_2, 60);

        ////////////////////// 경유점 JS 1
        JsDouble waypoint_1 = {-14.982, 12.179, -102.482, 140.886, 80.900, -10.503}; // left drum hole surface
        taskPushBack_jsMove2(left_task_sub_2, waypoint_1, 75.0, 75.0, false);
        taskPushBack_delay_1ms(left_task_sub_2, 60);

        ////////////////////// 경유점 JS 2
        JsDouble waypoint_2 = {-47.583, 15.553, -121.652, -42.531, 17.764, 125.252};
        taskPushBack_jsMove2(left_task_sub_2, waypoint_2, 75.0, 75.0, false);
        taskPushBack_delay_1ms(left_task_sub_2, 60);

        ////////////////////// 경유점 JS 3
        taskPushBack_jsMove2(left_task_sub_2, {-43.706, -3.644, -101.190, 6.187, 11.038, 83.869}, 75.0, 75.0, false);
        taskPushBack_delay_1ms(left_task_sub_2, 60);
        /////////////////////// 인식 자세 이동

        taskPushBack_doGrasping_sharedTask(left_task_sub_2, 0.1, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(left_task_sub_2, 60);

        taskPushBack_csMoveToolFrame(left_task_sub_2, {0, 0, -0.002, 0, 0, 0}, 0.2, 0.1, true);


        //// 상대 모션, 파지 중에 걸릴 경우, 아래를 수정
        taskPushBack_csMoveToolFrame(left_task_sub_2, {0.03, 0, 0, 0, 0, 0}, 0.2, 0.1, true);
        taskPushBack_delay_1ms(left_task_sub_2, 60);
        taskPushBack_csMoveToolFrame(left_task_sub_2, {0.06, 0, 0, 0, 0, 0}, 0.04, 0.02, true);
        taskPushBack_delay_1ms(left_task_sub_2, 60);
        taskPushBack_csMoveToolFrame(left_task_sub_2, {0.02, 0, 0, 0, 0, 0}, 0.04, 0.02, true);
        taskPushBack_delay_1ms(left_task_sub_2, 60);
        taskPushBack_csMoveToolFrame(left_task_sub_2, {0, 0, -0.0045, 0, 0, 0}, 0.04, 0.02, true);
        taskPushBack_delay_1ms(left_task_sub_2, 60);
        taskPushBack_csMoveToolFrame(left_task_sub_2, {0.02, 0, 0, 0, 0, 0}, 0.01, 0.005, true);
        taskPushBack_delay_1ms(left_task_sub_2, 60);
        taskPushBack_csMoveToolFrame(left_task_sub_2, {0, 0, 0.0045, 0, 0, 0}, 0.01, 0.005, true);
        taskPushBack_delay_1ms(left_task_sub_2, 60);

    } 

    left_task_sub_2.insert(left_task_sub_2.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());
    taskPushBack_delay_1ms(left_task_sub_2, 800);

    taskPushBack_csMoveToolFrame(left_task_sub_2, {0, 0, -0.210, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(left_task_sub_2, 50);

    // taskPushBack_csMove2(left_task_sub_2, {0.250, 0, 0, 0, 0, 0}, 0.2, 0.1, true); // false: absolute, true: relative
    // taskPushBack_delay_1ms(left_task_sub_2, 50);


    if(1) {
        //// drum hole surface
        std::string target_object = "left_drum_hole_surface";
        std::string target_tcp = "tcp03"; // screwing new gripper tcp
        std::string target_task = "drum_task";
        JsDouble scan_pose_target = {-26.793, 14.913, -108.540, 142.594, 70.024, -18.025};
        taskPushBack_jsMove2(left_task_sub_2, scan_pose_target, 75.0, 75.0, false);
        taskPushBack_delay_1ms(left_task_sub_2, 60);
        //// Scanning
        taskPushBack_drflSetTCP(left_task_sub_2, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(left_task_sub_2, 60);

        taskPushBack_task3DScanningTargetObject(left_task_sub_2, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(left_task_sub_2, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(left_task_sub_2, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(left_task_sub_2, 4000);


        taskPushBack_drflSetTCP(left_task_sub_2, target_tcp); // new gripper
        taskPushBack_delay_1ms(left_task_sub_2, 60);
        /////////////////////// 인식 자세 이동


        JsDouble waypoint_4 = {-38.899, 1.375, -113.809, 72.610, 54.124, 28.087};
        taskPushBack_jsMove2(left_task_sub_2, waypoint_4, 75.0, 75.0, false);
        taskPushBack_delay_1ms(left_task_sub_2, 60);

        /////////////////////// 인식 자세 이동

        taskPushBack_doGrasping_sharedTask(left_task_sub_2, 0.1, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(left_task_sub_2, 60);

    }

    return left_task_sub_2;
}

std::vector<UnitTask> TaskPlannerHanyangEng::leftGenSubTask3Task() {
    std::vector<UnitTask> left_task_sub_3;

    taskPushBack_drflSetTCP(left_task_sub_3, "tcp03"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(left_task_sub_3, 60);

    // 인식 자세로부터 총 200 mm down
    taskPushBack_csMoveToolFrame(left_task_sub_3, {0.0, 0, 0, 0, 0, 2}, 0.1, 0.05, true); // true: relative

    taskPushBack_csMoveToolFrame(left_task_sub_3, {0.0, -0.007, 0.001, 0, 0, 0}, 0.1, 0.05, true); // true: relative

    taskPushBack_csMoveToolFrame(left_task_sub_3, {0.110, 0, 0, 0, 0, 0}, 0.2, 0.1, true); // true: relative

    taskPushBack_csMoveToolFrame(left_task_sub_3, {0.005, 0, 0, 0, 0, 0}, 0.5, 0.025, true); // true: relative

    // 기울이는 모션
    taskPushBack_csMoveToolFrame(left_task_sub_3, {0, 0, -0.0014, 0, -2.0, 0}, 0.01, 0.005, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(left_task_sub_3, 60);

    taskPushBack_csMoveToolFrame(left_task_sub_3, {0.007, 0, 0, 0, 0, 0}, 0.005, 0.0025, true); // true: relative
    taskPushBack_delay_1ms(left_task_sub_3, 100);

    left_task_sub_3.insert(left_task_sub_3.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(left_task_sub_3, 2000);

    // 땡겨주는 모션 (모바일 로봇이 기울어있음)
    taskPushBack_csMoveToolFrame(left_task_sub_3, {0.003, 0.0, 0, 0, 0, 0}, 0.02, 0.01, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(left_task_sub_3, 60);

    if(1)  {
        for (int cycle = 0; cycle < 3; cycle++) {
            left_task_sub_3.insert(left_task_sub_3.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
            taskPushBack_delay_1ms(left_task_sub_3, 100);
        }
    }

    //// 추가로 더 내리기
    taskPushBack_csMoveToolFrame(left_task_sub_3, {0.005, 0, 0, 0, 0, 0}, 0.02, 0.01, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(left_task_sub_3, 60);
    //// 추가로 더 내리기

    left_task_sub_3.insert(left_task_sub_3.end(), drum_grp_grasping_pose_task_.begin(), drum_grp_grasping_pose_task_.end());


        // 케미컬 호스 체결 작업 (총 0.009 가량 down)
    auto subTaskScrewingChemicalHose = [&] (std::vector<UnitTask>& task_in, int trial_cnt) {

        task_in.insert(task_in.end(), drum_grp_screwing_pose_task_.begin(), drum_grp_screwing_pose_task_.end());

        // if (trial_cnt % 2 == 0) {
        //     double z_scale_down = 0.003 / 4;         ///// 총 횟수%2

        //     taskPushBack_csMoveToolFrame(task_in, {z_scale_down, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // z축 이동
        //         //taskPushBack_delay_1ms(sub_task, 1000);
        // }
        // ROS_LOG_WARN("Chemical Hose Screwing - Trial: %d / 60", trial_cnt + 1);

    };

    // for (int k = 0; k < 14; k++) { // 너무 과도하게 조임
    // k 12 에 k== 9 가 현재 최적 (0331 1110am)
    // for (int k = 0; k < 10; k++) { // 기존
    // k 9 , k< 7 : 수요일 요청 사항
    for (int k = 0; k < 14; k++) { // 수요일 요청사항

        if(k < 8) { // 기존
            // if (k % 2 == 0) {
                // double z_scale_down = 0.000266667;         ///// 총 횟수%2
                double z_scale_down = 0.000266667;         ///// 총 횟수%2
                taskPushBack_csMoveToolFrame(left_task_sub_3, {z_scale_down, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // z축 이동
            // }
        }

        // if(k == 9) {
        //     taskPushBack_drflSetImpedance(right_task_sub_3, true); // Compliance CTRL ON
        //     taskPushBack_delay_1ms(right_task_sub_3, 600);
        // }
        subTaskScrewingChemicalHose(left_task_sub_3, k);
    }


    ////////////////////////////// unscrewing scan 과정 필요할 때 사용
    ////////////////////////////// unscrewing scan 과정 필요할 때 사용
    ////////////////////////////// unscrewing scan 과정 필요할 때 사용
    ////////////////////////////// unscrewing scan 과정 필요할 때 사용

    // left_task_sub_3.insert(left_task_sub_3.end(), drum_grp_screwing_pose_task_.begin(), drum_grp_screwing_pose_task_.end());
    // taskPushBack_delay_1ms(left_task_sub_3, 2000);
    left_task_sub_3.insert(left_task_sub_3.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    taskPushBack_delay_1ms(left_task_sub_3, 3500);

    taskPushBackKORASGripperCmd(left_task_sub_3, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(left_task_sub_3, 60);
    taskPushBackKORASGripperCmd(left_task_sub_3, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -10500);
    taskPushBack_delay_1ms(left_task_sub_3, 3500);

    taskPushBack_csMoveToolFrame(left_task_sub_3, {0, 0, -0.003, 0, 0, 0}, 0.01, 0.005, true);
    taskPushBack_delay_1ms(left_task_sub_3, 60);
    taskPushBack_csMoveToolFrame(left_task_sub_3, {-0.02, 0, 0, 0, 0, 0}, 0.01, 0.005, true);
    taskPushBack_delay_1ms(left_task_sub_3, 60);
    taskPushBack_csMoveToolFrame(left_task_sub_3, {-0.11, 0, 0, 0, 0, 0}, 0.1, 0.1, true);
    taskPushBack_delay_1ms(left_task_sub_3, 20);
    taskPushBack_csMoveToolFrame(left_task_sub_3, {-0.1, 0, 0, 0, 0, 0}, 0.1, 0.1, true);
    taskPushBack_delay_1ms(left_task_sub_3, 20);

    taskPushBack_csMove2(left_task_sub_3, {0, -0.2, 0, 0, 0, 0}, 0.7, 0.7, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(left_task_sub_3, 200);

    taskPushBack_jsMove2(left_task_sub_3, left_home_pose_, 30.0, 30.0, false);
    taskPushBack_jsMove2(left_task_sub_3, right_home_pose_, 30.0, 30.0, false);

    return left_task_sub_3;
}

std::vector<UnitTask> TaskPlannerHanyangEng::leftGenSubTask4Task() {
    std::vector<UnitTask> left_task_sub_4;

    ////////////////////////////// unscrewing scan 과정 필요할 때 사용
    ////////////////////////////// unscrewing scan 과정 필요할 때 사용
    ////////////////////////////// unscrewing scan 과정 필요할 때 사용
    ////////////////////////////// unscrewing scan 과정 필요할 때 사용

    std::string target_object = "left_drum_coupler_unscrewing";
    std::string target_tcp = "tcp03";
    // std::string target_task = "default"; // 툴 기준 Z 방향으로 approach
    std::string target_task = "drum_task"; // 툴 기준 X 방향으로 approach
    JsDouble left_scan_pose_target = {-20.726, 12.800, -104.929, 141.444, 75.885, -13.906};

    taskPushBack_jsMove2(left_task_sub_4, left_home_pose_, 30.0, 30.0, false);


    left_task_sub_4.insert(left_task_sub_4.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(left_task_sub_4, 3500);

    taskPushBackKORASGripperCmd(left_task_sub_4, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(left_task_sub_4, 60);
    taskPushBackKORASGripperCmd(left_task_sub_4, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -3000);
    taskPushBack_delay_1ms(left_task_sub_4, 500);

    taskPushBack_jsMove2(left_task_sub_4, left_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(left_task_sub_4, 60);

    taskPushBack_jsMove2(left_task_sub_4, left_scan_pose_target, 75.0, 75.0, false);
    taskPushBack_delay_1ms(left_task_sub_4, 60);

    taskPushBack_drflSetTCP(left_task_sub_4, "tcp00"); // Scan & Matching은 tcp00
    taskPushBack_delay_1ms(left_task_sub_4, 60);

    taskPushBack_task3DScanningTargetObject(left_task_sub_4, target_object); // ZIVID Scanning with task recognition
    taskPushBack_delay_1ms(left_task_sub_4, 600);
    //// Matching
    taskPushBack_taskMatchingTargetObject(left_task_sub_4, target_object, false, 8); // "default", true: symmetric CAD used
    taskPushBack_delay_1ms(left_task_sub_4, 4000);

    taskPushBack_drflSetTCP(left_task_sub_4, target_tcp); // new gripper
    taskPushBack_delay_1ms(left_task_sub_4, 60);

    JsDouble waypoint_4 = {-38.899, 1.375, -113.809, 72.610, 54.124, 28.087};
    taskPushBack_jsMove2(left_task_sub_4, waypoint_4, 75.0, 75.0, false);
    taskPushBack_delay_1ms(left_task_sub_4, 60);

    taskPushBack_doGrasping_sharedTask(left_task_sub_4, 0.05, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
    taskPushBack_delay_1ms(left_task_sub_4, 60);

    //// 상대 모션, 파지 중에 걸릴 경우, 아래를 수정
    taskPushBack_csMoveToolFrame(left_task_sub_4, {0, 0, -0.004, 0, 0, 0}, 0.05, 0.05, true);
    // taskPushBack_delay_1ms(left_task_sub_4, 60);
    taskPushBack_csMoveToolFrame(left_task_sub_4, {0.06, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
    // taskPushBack_delay_1ms(left_task_sub_4, 60);
    taskPushBack_csMoveToolFrame(left_task_sub_4, {0.010, 0, -0.001, 0, 0, 0}, 0.05, 0.05, true);
    // taskPushBack_delay_1ms(left_task_sub_4, 60);
    taskPushBack_csMoveToolFrame(left_task_sub_4, {0.025, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
    // taskPushBack_delay_1ms(left_task_sub_4, 60);
    taskPushBack_csMoveToolFrame(left_task_sub_4, {0, 0, -0.006, 0, 0, 0}, 0.05, 0.05, true);
    // taskPushBack_delay_1ms(left_task_sub_4, 60);
    taskPushBack_csMoveToolFrame(left_task_sub_4, {0.014, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
    // taskPushBack_delay_1ms(left_task_sub_4, 60);
    taskPushBack_csMoveToolFrame(left_task_sub_4, {0, 0, 0.0045, 0, 0, 0}, 0.05, 0.05, true);
    // taskPushBack_delay_1ms(left_task_sub_4, 60);
    taskPushBack_csMoveToolFrame(left_task_sub_4, {0, 0, 0.0002, 0, 0, 0}, 0.05, 0.05, true);
    
    // taskPushBack_csMoveToolFrame(left_task_sub_4, {0.06, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
    // taskPushBack_delay_1ms(left_task_sub_4, 60);
    // taskPushBack_csMoveToolFrame(left_task_sub_4, {0.035, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
    // taskPushBack_delay_1ms(left_task_sub_4, 60);
    // taskPushBack_csMoveToolFrame(left_task_sub_4, {0, 0, -0.006, 0, 0, 0}, 0.05, 0.05, true);
    // taskPushBack_delay_1ms(left_task_sub_4, 60);
    // taskPushBack_csMoveToolFrame(left_task_sub_4, {0.014, 0, 0, 0, 0, 0}, 0.05, 0.05, true);
    // taskPushBack_delay_1ms(left_task_sub_4, 60);
    // taskPushBack_csMoveToolFrame(left_task_sub_4, {0, 0, 0.0045, 0, 0, 0}, 0.05, 0.05, true);
    // taskPushBack_delay_1ms(left_task_sub_4, 60);
    // taskPushBack_csMoveToolFrame(left_task_sub_4, {0, 0, 0.0002, 0, 0, 0}, 0.05, 0.05, true);

    ////////////////////////////// unscrewing scan 과정 필요할 때 사용
    ////////////////////////////// unscrewing scan 과정 필요할 때 사용
    ////////////////////////////// unscrewing scan 과정 필요할 때 사용
    ////////////////////////////// unscrewing scan 과정 필요할 때 사용
    ////////////////////////////////////////////////////////////////

    auto subTaskUnScrewingChemicalHose = [&] (std::vector<UnitTask>& task_in, int trial_cnt) {

        task_in.insert(task_in.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());

        // if (trial_cnt % 4 == 0) {
        //     double z_scale_down = 0.003 / 3;         ///// 총 횟수%2

        //     taskPushBack_csMoveToolFrame(task_in, {-z_scale_down, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // z축 이동
        //         //taskPushBack_delay_1ms(sub_task, 1000);
        // }

        ROS_LOG_WARN("Chemical Hose UnScrewing - Trial: %d / 60", trial_cnt + 1);

    };

    // taskPushBack_drflSetTCP(left_task_sub_4, "tcp03"); // Scan & Matching은 tcp00
    // taskPushBack_delay_1ms(left_task_sub_4, 60);

    /////////////////////////////////////////////////////////////////////////////////
    //// NOTICE: 바로 체결에 이어서 해제를 하므로, 켤 필요가 없음
    // taskPushBack_drflSetImpedance(left_task_sub_4, true); // Compliance CTRL ON
    // taskPushBack_delay_1ms(left_task_sub_4, 3000);
    taskPushBack_drflSetImpedance(left_task_sub_4, true); // Compliance CTRL OFF
    taskPushBack_delay_1ms(left_task_sub_4, 600);
    /////////////////////////////////////////////////////////////////////////////////
    ///// 현재 k = 13 , k > 11 최적
    for (int k = 0; k < 11; k++) {

        // if(k > 11) {
        //     // if (k % 2 == 0) {
        //         // double z_scale_down = 0.000266667;         ///// 총 횟수%2
        //         double z_scale_down = 0.000266667;         ///// 총 횟수%2
        //         taskPushBack_csMoveToolFrame(left_task_sub_4, {-z_scale_down, 0, 0, 0, 0, 0}, 0.1, 0.05, true); // z축 이동
        //     // }
        // }

        subTaskUnScrewingChemicalHose(left_task_sub_4, k);
    }

    /////////////////////////////////////////////////////////////////////////////////
    taskPushBack_drflSetImpedance(left_task_sub_4, false); // Compliance CTRL OFF
    taskPushBack_delay_1ms(left_task_sub_4, 600);
    /////////////////////////////////////////////////////////////////////////////////

    // left_task_sub_4.insert(left_task_sub_4.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    // taskPushBack_delay_1ms(left_task_sub_4, 100);

    // taskPushBack_csMoveToolFrame(left_task_sub_4, {-0.002, 0, 0, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    // taskPushBack_delay_1ms(left_task_sub_4, 50);

    left_task_sub_4.insert(left_task_sub_4.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    taskPushBack_delay_1ms(left_task_sub_4, 100);

    taskPushBack_csMoveToolFrame(left_task_sub_4, {-0.002, 0, 0, 0, 0, 0}, 0.05, 0.025, true); // true: relative
    taskPushBack_delay_1ms(left_task_sub_4, 50);

    taskPushBack_csMoveToolFrame(left_task_sub_4, {0, 0, -0.0005, 0, -1, 0}, 0.01, 0.005, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(left_task_sub_4, 60);

    left_task_sub_4.insert(left_task_sub_4.end(), drum_grp_unscrewing_pose_task_.begin(), drum_grp_unscrewing_pose_task_.end());
    taskPushBack_delay_1ms(left_task_sub_4, 100);

    taskPushBack_csMove2(left_task_sub_4, {0, 0, 0.05, 0, 0, 0}, 0.02, 0.01, true); // true: relative
    taskPushBack_delay_1ms(left_task_sub_4, 50);
    taskPushBack_csMove2(left_task_sub_4, {0, 0, 0.100, 0, 0, 0}, 0.7, 0.7, true); // true: relative
    // taskPushBack_delay_1ms(left_task_sub_4, 50);

    taskPushBack_csMove2(left_task_sub_4, {0, -0.2, 0, 0, 0, 0}, 0.7, 0.7, true); // false: absolute, true: relative
    // taskPushBack_delay_1ms(left_task_sub_4, 500);

    taskPushBack_jsMove2(left_task_sub_4, left_home_pose_, 30.0, 30.0, false);
    // taskPushBack_delay_1ms(left_task_sub_4, 60);

    return left_task_sub_4;
}

std::vector<UnitTask> TaskPlannerHanyangEng::leftGenSubTask5Task() {
    std::vector<UnitTask> left_task_sub_5;

    if(1) {
        //// drum hole surface
        std::string target_object = "left_holder_without_coupler";
        std::string target_tcp = "tcp06"; // screwing new gripper tcp
        std::string target_task = "default";

        JsDouble scan_pose_target = {-5.589, 4.486, -86.330, 137.716, 90.786, -60.594};
        taskPushBack_jsMove2(left_task_sub_5, scan_pose_target, 75.0, 75.0, false);
        taskPushBack_delay_1ms(left_task_sub_5, 60);
        //// Scanning
        taskPushBack_drflSetTCP(left_task_sub_5, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(left_task_sub_5, 60);

        taskPushBack_task3DScanningTargetObject(left_task_sub_5, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(left_task_sub_5, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(left_task_sub_5, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(left_task_sub_5, 4000);


        taskPushBack_drflSetTCP(left_task_sub_5, target_tcp); // new gripper
        taskPushBack_delay_1ms(left_task_sub_5, 60);

        ////////////////////// 경유점 JS 1
        if(1) { // 역순
            JsDouble waypoint_1 = {-14.982, 12.179, -102.482, 140.886, 80.900, -10.503}; // drum hole surface
            taskPushBack_jsMove2(left_task_sub_5, waypoint_1, 75.0, 75.0, false);
            taskPushBack_delay_1ms(left_task_sub_5, 60);
        }

        ////////////////////// 경유점 JS 2
        JsDouble waypoint_2 = {-47.583, 15.553, -121.652, -42.531, 17.764, 125.252};
        taskPushBack_jsMove2(left_task_sub_5, waypoint_2, 75.0, 75.0, false);
        taskPushBack_delay_1ms(left_task_sub_5, 60);


        /////////////////////// 인식 자세 이동

        taskPushBack_doGrasping_sharedTask(left_task_sub_5, 0.1, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(left_task_sub_5, 60);
        ///////////////////// 인식 자세 이동


        //// 입장 거리 (210mm)
        taskPushBack_csMoveToolFrame(left_task_sub_5, {-0.007, 0, 0.100, 0, 0, 0}, 0.1, 0.1, true);
        taskPushBack_delay_1ms(left_task_sub_5, 60);
        taskPushBack_csMoveToolFrame(left_task_sub_5, {0.005, 0, 0.062, 0, 0, 0}, 0.04, 0.02, true);
        taskPushBack_delay_1ms(left_task_sub_5, 60);
        taskPushBack_csMoveToolFrame(left_task_sub_5, {0, 0, 0.047, 0, 0, 0}, 0.04, 0.02, true);
        taskPushBack_delay_1ms(left_task_sub_5, 60);
        // taskPushBack_csMoveToolFrame(left_task_sub_5, {0, 0, 0.010, 0, 0, 0}, 0.04, 0.02, true);
        // taskPushBack_delay_1ms(left_task_sub_5, 60);

        left_task_sub_5.insert(left_task_sub_5.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
        taskPushBack_delay_1ms(left_task_sub_5, 2500);
    
        taskPushBackKORASGripperCmd(left_task_sub_5, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(left_task_sub_5, 60);
        taskPushBackKORASGripperCmd(left_task_sub_5, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -3000);
        taskPushBack_delay_1ms(left_task_sub_5, 500);

        taskPushBack_csMoveToolFrame(left_task_sub_5, {-0.02, 0, 0, 0, 0, 0}, 0.01, 0.005, true);
        taskPushBack_delay_1ms(left_task_sub_5, 60);
        taskPushBack_csMoveToolFrame(left_task_sub_5, {-0.120, 0, 0, 0, 0, 0}, 0.2, 0.1, true);
        // taskPushBack_delay_1ms(left_task_sub_5, 60);

        if(1) { // 역순
            taskPushBack_csMoveToolFrame(left_task_sub_5, {0, 0, -0.2, 0, 0, 0}, 0.2, 0.1, true);
            // taskPushBack_delay_1ms(left_task_sub_5, 60);

            taskPushBack_jsMove2(left_task_sub_5, left_home_pose_, 30.0, 30.0, false);
            // taskPushBack_delay_1ms(left_task_sub_5, 60);
        }

    }

    return left_task_sub_5;
}

std::vector<UnitTask> TaskPlannerHanyangEng::leftGenSubTask6Task() {
    std::vector<UnitTask> left_task_sub_6;

    // taskPushBack_jsMove2(left_task_sub_6, left_home_pose_, 60.0, 60.0, false);
    // taskPushBack_delay_1ms(left_task_sub_6, 60);

    // JsDouble scan_pose_target = {-69.525, 5.651, -104.120, 49.058, -37.210, 24.425};
    // taskPushBack_jsMove2(left_task_sub_6, scan_pose_target, 120.0, 80.0, false);
    // taskPushBack_delay_1ms(left_task_sub_6, 60);

    // taskPushBack_jsMove2(left_task_sub_6, left_home_pose_, 60.0, 60.0, false);
    // taskPushBack_delay_1ms(left_task_sub_6, 60);

    // taskPushBack_jsMove2(left_task_sub_6, scan_pose_target, 120.0, 80.0, false);
    // taskPushBack_delay_1ms(left_task_sub_6, 60);

    // taskPushBack_jsMove2(left_task_sub_6, left_home_pose_, 60.0, 60.0, false);
    // taskPushBack_delay_1ms(left_task_sub_6, 60);

    taskPushBack_jsMove2(left_task_sub_6, left_home_pose_, 30.0, 30.0, false);
    taskPushBack_delay_1ms(left_task_sub_6, 60);

    left_task_sub_6.insert(left_task_sub_6.end(), drum_grp_initialize_task_.begin(), drum_grp_initialize_task_.end());
    taskPushBack_delay_1ms(left_task_sub_6, 3000);
    left_task_sub_6.insert(left_task_sub_6.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
    taskPushBack_delay_1ms(left_task_sub_6, 3000);


    if(1) {
        std::string target_object = "left_holder_with_lid_cap";
        std::string target_tcp = "tcp05"; // screwing new gripper tcp
        std::string target_task = "default";

        JsDouble scan_pose_target = {-69.525, 5.651, -104.120, 49.058, -37.210, 24.425};
        taskPushBack_jsMove2(left_task_sub_6, scan_pose_target, 75.0, 75.0, false);
        taskPushBack_delay_1ms(left_task_sub_6, 60);
        //// Scanning
        taskPushBack_drflSetTCP(left_task_sub_6, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(left_task_sub_6, 60);

        taskPushBack_task3DScanningTargetObject(left_task_sub_6, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(left_task_sub_6, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(left_task_sub_6, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(left_task_sub_6, 4000);


        taskPushBack_drflSetTCP(left_task_sub_6, target_tcp); // new gripper
        taskPushBack_delay_1ms(left_task_sub_6, 60);


        if(1) {
            JsDouble waypoint_1 = {-64.736, 27.570, -115.122, 9.968, -39.035, 152.603};
            taskPushBack_jsMove2(left_task_sub_6, waypoint_1, 75.0, 75.0, false);
            taskPushBack_delay_1ms(left_task_sub_6, 60);
            taskPushBack_csMove2(left_task_sub_6, {-0.3, 0.3, 0, 0, 0, 0}, 0.05, 0.05, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(left_task_sub_6, 500);
        }

        /////////////////////// 인식 자세 이동

        taskPushBack_doGrasping_sharedTask(left_task_sub_6, 0.05, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(left_task_sub_6, 60);
        /////////////////////// 인식 자세 이동

    }

    //// 파지
    taskPushBack_csMoveToolFrame(left_task_sub_6, {0, 0.003, 0.057, 0, 0, 0}, 0.03, 0.015, true);
    taskPushBack_delay_1ms(left_task_sub_6, 60);

    left_task_sub_6.insert(left_task_sub_6.end(), drum_grp_lid_cap_close_tight.begin(), drum_grp_lid_cap_close_tight.end());
    taskPushBack_delay_1ms(left_task_sub_6, 4000);


    taskPushBackKORASGripperCmd(left_task_sub_6, (uint16_t)KR_GRP::POS_RESET);
    taskPushBack_delay_1ms(left_task_sub_6, 60);
    taskPushBackKORASGripperCmd(left_task_sub_6, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -1500);
    taskPushBack_delay_1ms(left_task_sub_6, 500);


    taskPushBack_csMoveToolFrame(left_task_sub_6, {0, 0, -0.048, 0, 0, 0}, 0.03, 0.015, true);
    taskPushBack_delay_1ms(left_task_sub_6, 60);

    taskPushBack_csMove2(left_task_sub_6, {0.2, -0.2, -0.05, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
    taskPushBack_delay_1ms(left_task_sub_6, 500);

    
    if(1){
        /////////////////////////////////////////////////////////////////////////
        std::string target_object = "left_drum_lid_cap_screwing";
        std::string target_task = "default";
        JsDouble scan_pose_target = {-28.525, 15.174, -105.454, 141.370, 71.875, -20.901};

        taskPushBack_jsMove2(left_task_sub_6, scan_pose_target, 75.0, 75.0, false);
        taskPushBack_delay_1ms(left_task_sub_6, 60);

        taskPushBack_drflSetTCP(left_task_sub_6, "tcp00"); // Scan & Matching은 tcp00
        taskPushBack_delay_1ms(left_task_sub_6, 60);

        taskPushBack_task3DScanningTargetObject(left_task_sub_6, target_object); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(left_task_sub_6, 600);
        //// Matching
        taskPushBack_taskMatchingTargetObject(left_task_sub_6, target_object, false, 8); // ZIVID Scanning with task recognition
        taskPushBack_delay_1ms(left_task_sub_6, 1000);

        taskPushBack_drflSetTCP(left_task_sub_6, "tcp07"); 
        taskPushBack_delay_1ms(left_task_sub_6, 60);

        //// 천장 부딪히지 않도록 경유지 설정
        if(1) {
            JsDouble waypoint_1 = {-57.689, -2.486, -96.170, 179.491, 80.932, -64.043};
            taskPushBack_jsMove2(left_task_sub_6, waypoint_1, 75.0, 75.0, false);
            taskPushBack_delay_1ms(left_task_sub_6, 50);
            taskPushBack_csMove2(left_task_sub_6, {-0.1, 0.3, 0.0, 0, 0, 0}, 0.2, 0.2, true); // false: absolute, true: relative
            taskPushBack_delay_1ms(left_task_sub_6, 50);
        }

        /////////////////////// 인식 자세 이동

        taskPushBack_doGrasping_sharedTask(left_task_sub_6, 0.1, 0.05, target_task, target_object); // Move to grasping pose, (acc, vel)
        taskPushBack_delay_1ms(left_task_sub_6, 60);

        taskPushBack_drflSetTCP(left_task_sub_6, "tcp05"); 
        taskPushBack_delay_1ms(left_task_sub_6, 60);

        taskPushBack_csMoveToolFrame(left_task_sub_6, {0, 0.002, 0.010, 0, 0, 0}, 0.1, 0.05, true); // true: relative
        taskPushBack_csMoveToolFrame(left_task_sub_6, {0, 0, 0.010, 0, 0, 0}, 0.1, 0.05, true); // true: relative
        taskPushBack_csMoveToolFrame(left_task_sub_6, {0, 0, 0.005, 0, 0, 10}, 0.025, 0.025, true); // true: relative


        left_task_sub_6.insert(left_task_sub_6.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(left_task_sub_6, 1000);
        left_task_sub_6.insert(left_task_sub_6.end(), drum_grp_lid_cap_open_.begin(), drum_grp_lid_cap_open_.end());
        taskPushBack_delay_1ms(left_task_sub_6, 1000);
        taskPushBack_csMoveToolFrame(left_task_sub_6, {0, 0, 0, 0, 0, -10}, 0.1, 0.05, true); // true: relative

        taskPushBack_csMoveToolFrame(left_task_sub_6, {0, 0, 0.012, 0, 0, 0}, 0.02, 0.04, true); // true: relative
        taskPushBack_delay_1ms(left_task_sub_6, 60);


        int cnt_lid_cap_screwing = 4;
        for (int cycle = 0; cycle < cnt_lid_cap_screwing; cycle++) {
            left_task_sub_6.insert(left_task_sub_6.end(), drum_grp_lid_cap_close_for_screwing_.begin(), drum_grp_lid_cap_close_for_screwing_.end());


            if(cycle != cnt_lid_cap_screwing - 1) { // 마지막 trial에서는 스킵
                taskPushBack_delay_1ms(left_task_sub_6, 2000);
                taskPushBack_jsMove2(left_task_sub_6, {0, 0, 0, 0, 0, 195}, 40.0, 40.0, true);
                left_task_sub_6.insert(left_task_sub_6.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
                taskPushBack_delay_1ms(left_task_sub_6, 2000);
            } else {
                //// 마지막 횟수에서 더 돌리기 (90도만)
                taskPushBack_delay_1ms(left_task_sub_6, 2000);
                taskPushBack_jsMove2(left_task_sub_6, {0, 0, 0, 0, 0, 135}, 40.0, 40.0, true);
                // taskPushBack_delay_1ms(left_task_sub_6, 20);
                //// 아래에서 더 돌리면 팁 파손 가능성
                taskPushBack_jsMove2(left_task_sub_6, {0, 0, 0, 0, 0, 15}, 15.0, 15.0, true);
                taskPushBack_jsMove2(left_task_sub_6, {0, 0, 0, 0, 0, 5}, 15.0, 15.0, true);
                // taskPushBack_delay_1ms(left_task_sub_6, 20);
                taskPushBack_jsMove2(left_task_sub_6, {0, 0, 0, 0, 0, -10}, 40.0, 40.0, true);
                left_task_sub_6.insert(left_task_sub_6.end(), drum_grp_lid_cap_screwing_open_.begin(), drum_grp_lid_cap_screwing_open_.end());
                taskPushBack_delay_1ms(left_task_sub_6, 2000);
                taskPushBack_jsMove2(left_task_sub_6, {0, 0, 0, 0, 0, -5}, 40.0, 40.0, true);
                // taskPushBack_delay_1ms(left_task_sub_6, 20);
            }


            if(cycle != cnt_lid_cap_screwing - 1) { // 마지막 trial에서는 스킵
                taskPushBack_jsMove2(left_task_sub_6, {0, 0, 0, 0, 0, -15}, 40.0, 40.0, true);
                // taskPushBack_delay_1ms(left_task_sub_6, 20);
                taskPushBack_csMoveToolFrame(left_task_sub_6, {0, 0, -0.050, 0, 0, 0}, 0.2, 0.2, true); // false: absolute, true: relative
                // taskPushBack_delay_1ms(left_task_sub_6, 20);
                taskPushBack_jsMove2(left_task_sub_6, {0, 0, 0, 0, 0, -180}, 40.0, 40.0, true);
                taskPushBack_csMoveToolFrame(left_task_sub_6, {0, 0, 0.050, 0, 0, 0}, 0.2, 0.2, true); // false: absolute, true: relative
                taskPushBack_delay_1ms(left_task_sub_6, 20);
            }
        }


        taskPushBack_csMove2(left_task_sub_6, {0, 0, 0.045, 0, 0, 0}, 0.1, 0.1, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(left_task_sub_6, 60);

        // 마지막 자세
        taskPushBack_csMove2(left_task_sub_6, {0, -0.4, 0, 0, 0, 0}, 0.3, 0.3, true); // false: absolute, true: relative
        taskPushBack_delay_1ms(left_task_sub_6, 60);

        taskPushBack_jsMove2(left_task_sub_6, left_home_pose_, 30.0, 30.0, false);
        taskPushBack_delay_1ms(left_task_sub_6, 60);
        taskPushBack_jsMove2(left_task_sub_6, right_home_pose_, 30.0, 30.0, false);
        taskPushBack_delay_1ms(left_task_sub_6, 60);
    }


    return left_task_sub_6;
}

std::vector<UnitTask> TaskPlannerHanyangEng::leftGenSubTask7Task() {
    std::vector<UnitTask> left_task_sub_7;


    return left_task_sub_7;
}

std::vector<UnitTask> TaskPlannerHanyangEng::leftGenSubTask8Task() {
    std::vector<UnitTask> left_task_sub_8;


    return left_task_sub_8;
}
///////////////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////
/// @brief ////////////////////////////////////////////////////////////////////////

void TaskPlannerHanyangEng::genGripperTask() {
    try {
        drum_grp_initialize_task_.clear();
        drum_grp_grasping_pose_task_.clear();
        drum_grp_screwing_pose_task_.clear();
        drum_grp_unscrewing_pose_task_.clear();
        drum_grp_exit_pose_task_.clear();
        drum_grp_lid_cap_close_.clear();
        drum_grp_lid_cap_open_.clear();

        drum_grp_lid_cap_close_tight.clear();
        drum_grp_lid_cap_open_tight.clear();
        drum_grp_plus_motor_ctrl_task_.clear();
        drum_grp_minus_motor_ctrl_task_.clear();

        drum_grp_lid_cap_screwing_open_.clear();

        drum_grp_lid_cap_close_for_screwing_.clear();


    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    /////////////// 케미컬 그리퍼 초기화
    const int DEFAULT_GRP = 1;
    const int R5_GRP = 2; // 250225, 기존 그리퍼
    const int R3_GRP = 3;
    const int NEW_GRP = 4;  //New 그리퍼 ///끝에서 끝이 22000 (+-11000)
    const int NEW_GRP_FOR_SMALL_COUPLER = 5;
    int whatGripper; // 기본 감속비
    // whatGripper = R5_GRP;// 1: 기본 감속비, 2: 감속비 (1:5), 3: 감속비 (1:3)
    whatGripper = NEW_GRP_FOR_SMALL_COUPLER;// 1: 기본 감속비, 2: 감속비 (1:5), 3: 감속비 (1:3)


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

        ////////////////////////////////////////////////////////////
        /////////////// 케미컬 그리퍼 열기 (초기 뚜껑 파지를 위한 모션)

        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_, (uint16_t)KR_GRP::MOTOR_ENABLE);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_, 200);
        ////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////
        /////////////// 케미컬 그리퍼 닫기 (초기 뚜껑 파지를 위한 모션)
        // taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -1600, 1000);
        // taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -1200, 1000);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -900, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_open_, 600);

        ////////////////////////////////////////////////////////////

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

        ////////////////////////////////////////////////////////////
        /////////////// 케미컬 그리퍼 열기 (초기 뚜껑 파지를 위한 모션)

        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_, (uint16_t)KR_GRP::MOTOR_ENABLE);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_, 200);
        ////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////
        /////////////// 케미컬 그리퍼 닫기 (초기 뚜껑 파지를 위한 모션)
        // taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -1600, 1000);
        // taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -1200, 1000);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -900, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_open_, 600);

        ////////////////////////////////////////////////////////////


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

        ////////////////////////////////////////////////////////////
        /////////////// 케미컬 그리퍼 열기 (초기 뚜껑 파지를 위한 모션)

        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_, (uint16_t)KR_GRP::MOTOR_ENABLE);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_, 200);
        ////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////
        /////////////// 케미컬 그리퍼 닫기 (초기 뚜껑 파지를 위한 모션)
        // taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -1600, 1000);
        // taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -1200, 1000);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -900, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_open_, 600);

        ////////////////////////////////////////////////////////////

        break;

    case NEW_GRP:                /// 값 확정 (수정 X)
        ///// 그리퍼 Initialize
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_ENABLE);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -20000, 1000);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 5000);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -5700, 1000);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);

        ///// 그리퍼 파지
        taskPushBackKORASGripperCmd(drum_grp_grasping_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_grasping_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_grasping_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -14000, 1000);
        taskPushBack_delay_1ms(drum_grp_grasping_pose_task_, 2000);

        ///// 체결
        taskPushBackKORASGripperCmd(drum_grp_screwing_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_screwing_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_screwing_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -25000, 1000);
        taskPushBack_delay_1ms(drum_grp_screwing_pose_task_, 3250);

        //// 체결 해제
        taskPushBackKORASGripperCmd(drum_grp_unscrewing_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_unscrewing_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_unscrewing_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +25000, 1000);
        taskPushBack_delay_1ms(drum_grp_unscrewing_pose_task_, 3250);

        //// 나올 때
        taskPushBackKORASGripperCmd(drum_grp_exit_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_exit_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_exit_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 14300, 1000);
        taskPushBack_delay_1ms(drum_grp_exit_pose_task_, 2000);

        //// star tip open
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -10700, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_, 500);

        //// star tip close
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_open_, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 10700, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_open_, 500);

        //// star tip open - for task6
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_tight, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_tight, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_tight, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -16800, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_tight, 500);

        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_for_screwing_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_for_screwing_, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_for_screwing_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -10400, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_for_screwing_, 500);

        taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_tight, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_open_tight, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_tight, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 16800, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_open_tight, 500);

        //// open jog
        taskPushBackKORASGripperCmd(drum_grp_plus_motor_ctrl_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_plus_motor_ctrl_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_plus_motor_ctrl_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 1000, 1000);
        taskPushBack_delay_1ms(drum_grp_plus_motor_ctrl_task_, 500);

        //// close jog
        taskPushBackKORASGripperCmd(drum_grp_minus_motor_ctrl_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_minus_motor_ctrl_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_minus_motor_ctrl_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -1000, 1000);
        taskPushBack_delay_1ms(drum_grp_minus_motor_ctrl_task_, 500);

        //// star tip open
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_screwing_open_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_screwing_open_, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_screwing_open_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 10400, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_screwing_open_, 500);

    case NEW_GRP_FOR_SMALL_COUPLER:                /// 값 확정 (수정 X)
        ///// 그리퍼 Initialize
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_ENABLE);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -20000, 1000);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 5000);
        taskPushBackKORASGripperCmd(drum_grp_initialize_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -5700, 1000);
        taskPushBack_delay_1ms(drum_grp_initialize_task_, 60);

        ///// 그리퍼 파지
        taskPushBackKORASGripperCmd(drum_grp_grasping_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_grasping_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_grasping_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -14000, 1000);
        taskPushBack_delay_1ms(drum_grp_grasping_pose_task_, 2000);

        ///// 체결
        taskPushBackKORASGripperCmd(drum_grp_screwing_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_screwing_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_screwing_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -25000, 1000);
        taskPushBack_delay_1ms(drum_grp_screwing_pose_task_, 3250);

        //// 체결 해제
        taskPushBackKORASGripperCmd(drum_grp_unscrewing_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_unscrewing_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_unscrewing_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, +25000, 1000);
        taskPushBack_delay_1ms(drum_grp_unscrewing_pose_task_, 3250);

        //// 나올 때
        taskPushBackKORASGripperCmd(drum_grp_exit_pose_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_exit_pose_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_exit_pose_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 14300, 1000);
        taskPushBack_delay_1ms(drum_grp_exit_pose_task_, 2000);

        //// star tip open
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -10700, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_, 500);

        //// star tip close
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_open_, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 10700, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_open_, 500);

        //// star tip open - for task6
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_tight, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_tight, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_tight, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -16800, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_tight, 500);

        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_for_screwing_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_for_screwing_, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_close_for_screwing_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -10400, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_close_for_screwing_, 500);

        taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_tight, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_open_tight, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_open_tight, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 16800, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_open_tight, 500);

        //// open jog
        taskPushBackKORASGripperCmd(drum_grp_plus_motor_ctrl_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_plus_motor_ctrl_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_plus_motor_ctrl_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 1000, 1000);
        taskPushBack_delay_1ms(drum_grp_plus_motor_ctrl_task_, 500);

        //// close jog
        taskPushBackKORASGripperCmd(drum_grp_minus_motor_ctrl_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_minus_motor_ctrl_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_minus_motor_ctrl_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -1000, 1000);
        taskPushBack_delay_1ms(drum_grp_minus_motor_ctrl_task_, 500);

        //// star tip open
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_screwing_open_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_lid_cap_screwing_open_, 60);
        taskPushBackKORASGripperCmd(drum_grp_lid_cap_screwing_open_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 10400, 1000);
        taskPushBack_delay_1ms(drum_grp_lid_cap_screwing_open_, 500);

        //// 그리퍼 +5000 
        taskPushBackKORASGripperCmd(drum_grp_plus_5000_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_plus_5000_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_plus_5000_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 5000, 1000);
        taskPushBack_delay_1ms(drum_grp_plus_5000_task_, 500);

        //// 그리퍼 +4000 
        taskPushBackKORASGripperCmd(drum_grp_plus_4000_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_plus_4000_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_plus_4000_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 4000, 1000);
        taskPushBack_delay_1ms(drum_grp_plus_4000_task_, 500);

        //// 그리퍼 +3000 
        taskPushBackKORASGripperCmd(drum_grp_plus_3000_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_plus_3000_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_plus_3000_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 3000, 1000);
        taskPushBack_delay_1ms(drum_grp_plus_3000_task_, 500);

        //// 그리퍼 +2000 
        taskPushBackKORASGripperCmd(drum_grp_plus_2000_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_plus_2000_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_plus_2000_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, 2000, 1000);
        taskPushBack_delay_1ms(drum_grp_plus_2000_task_, 500);

        //// 그리퍼 -5000 
        taskPushBackKORASGripperCmd(drum_grp_minus_5000_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_minus_5000_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_minus_5000_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -5000, 1000);
        taskPushBack_delay_1ms(drum_grp_minus_5000_task_, 500);

        //// 그리퍼 -4000 
        taskPushBackKORASGripperCmd(drum_grp_minus_4000_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_minus_4000_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_minus_4000_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -4000, 1000);
        taskPushBack_delay_1ms(drum_grp_minus_4000_task_, 500);

        //// 그리퍼 -3000 
        taskPushBackKORASGripperCmd(drum_grp_minus_3000_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_minus_3000_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_minus_3000_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -3000, 1000);
        taskPushBack_delay_1ms(drum_grp_minus_3000_task_, 500);

        //// 그리퍼 -2000 
        taskPushBackKORASGripperCmd(drum_grp_minus_2000_task_, (uint16_t)KR_GRP::POS_RESET);
        taskPushBack_delay_1ms(drum_grp_minus_2000_task_, 60);
        taskPushBackKORASGripperCmd(drum_grp_minus_2000_task_, (uint16_t)KR_GRP::MOTOR_POS_CTRL, -2000, 1000);
        taskPushBack_delay_1ms(drum_grp_minus_2000_task_, 500);

        break;

    default:
        break;
    }
    
    std::cout << "genGripperTask completed successfully" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Exception in genGripperTask: " << e.what() << std::endl;
        throw;
    } catch (...) {
        std::cerr << "Unknown exception in genGripperTask" << std::endl;
        throw;
    }
}

//// NOTICE: COPY THIS CODE
// std::vector<UnitTask> TaskPlannerHanyangEng::genMotionGraphy_Task_C3_Mode1(bool is_marker_applied) {
//     std::vector<UnitTask> task_c3_mode_1;


//     return task_c3_mode_1;
// }