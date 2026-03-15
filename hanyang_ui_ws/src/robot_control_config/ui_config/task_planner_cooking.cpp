#include "task_planner_cooking.hpp"
#include <iostream>

TaskPlannerCooking::TaskPlannerCooking() {
    makeTaskDefaultSetup();
    std::cout << "TaskPlannerCooking initialized." << std::endl;
}

TaskPlannerCooking::~TaskPlannerCooking() {
    std::cout << "TaskPlannerCooking destroyed." << std::endl;
}

bool TaskPlannerCooking::makeTaskDefaultSetup() {
    std::cout << "Creating Hanyang task list..." << std::endl;
    // Hanyang 전용 작업 리스트 로직 구현

    setGeneralSettingsForCookingTask();
    return true;
}

void TaskPlannerCooking::CookingCustomTask() {
    std::cout << "Executing Hanyang-specific task..." << std::endl;
    // 특정 작업 로직 구현
}

void TaskPlannerCooking::generateCookingTask() {
    ROS_LOG_WARN("[%s] Generating Cooking Task...", __func__);
    doosan_module_task_.clear();

    // taskPushBackKORASGripperCmd(doosan_module_task_, (uint16_t)KR_GRP::CLOSE);
    taskPushBack_delay_1ms(doosan_module_task_, 1000);
    // taskPushBackKORASGripperCmd(doosan_module_task_, (uint16_t)KR_GRP::OPEN);
    taskPushBack_delay_1ms(doosan_module_task_, 1000);
    ROS_LOG_WARN("[%s] Generating Cooking Task Finished!", __func__);
}


void TaskPlannerCooking::setGeneralSettingsForCookingTask() {
    /////////////////////////////////////////////////////////////////
    //// General settings
    /////////////////////////////////////////////////////////////////

    // waypoints
    JsDouble js_home_pose  = {-39.41, 132.64, -96.56, 2.81, -33.83, -143.11};
    JsDouble js_mid_pose = {-88.26, 111.15, -106.03, -21.29, -3.11, -71.29};
    JsDouble js_mid_2f_pose = {-88.26, 111.15, -106.03, -21.29, -3.11, -71.29};
    JsDouble js_mid_tag_pose = {-154.564, 120.774, -116.569, -23.045, -0.511, -70.208};
    js_home_pose_ = js_home_pose;
    js_mid_pose_ = js_mid_pose;
    js_mid_2f_pose_ = js_mid_2f_pose;
    js_mid_tag_pose_ = js_mid_tag_pose;

    // gripper
    CsDouble cs_position_gripper_change_init_1 = {};
    CsDouble cs_position_gripper_change_init_2 = {};
    CsDouble cs_position_gripper_change_init_3 = {};
    CsDouble cs_position_gripper_change_init_4 = {};
    CsDouble cs_position_gripper_change_init_5 = {};
    cs_position_gripper_change_init_1_ = cs_position_gripper_change_init_1;
    cs_position_gripper_change_init_2_ = cs_position_gripper_change_init_2;
    cs_position_gripper_change_init_3_ = cs_position_gripper_change_init_3;
    cs_position_gripper_change_init_4_ = cs_position_gripper_change_init_4;
    cs_position_gripper_change_init_5_ = cs_position_gripper_change_init_5;

    // tcp
    CsDouble tcp_default  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    tcp_default_ = tcp_default;
}

//////////////////////////////////////////////Cooking robot///////////////////////////////////////////////////
std::vector<UnitTask> TaskPlannerCooking::CookToolPointTask(const std::string &tool_name, double x, double y, double z, double roll, double pitch, double yaw) {
    std::vector<UnitTask> task_toolpoints;

    // 속도 및 가속도 설정
    const double vel_cs = 0.01;
    const double acc_cs = 0.02;
    const double vel_js = 15;
    const double acc_js = 30;

    std::cout << "[DEBUG] Tool Name: " << tool_name << std::endl;


    std::cout << "[DEBUG] Target Coordinates (in meters): x=" << x << ", y=" << y << ", z=" << z
              << ", roll=" << roll << ", pitch=" << pitch << ", yaw=" << yaw << std::endl;

    roll = 118.0;
    pitch = -2.0;
    yaw = 60.0;

    // 툴 좌표로 이동
    std::vector<double> tool_point = {x, y, z, roll, pitch, yaw};

    // 홈 포즈로 복귀
    const JsDouble middle_home_pose = {-88.26, 111.15, -106.03, -21.29, -3.11, -71.29};
    const JsDouble middle_tag_pose = {-154.564, 120.774, -116.569, -23.045, -0.511, -70.208};
    const JsDouble home_pose = {-39.41, 132.64, -96.56, 2.81, -33.83, -143.11};
    taskPushBack_jsMove2(task_toolpoints, home_pose, vel_js, acc_js, false);  // 관절 공간 이동
    taskPushBack_delay(task_toolpoints, 50);
    taskPushBack_jsMove2(task_toolpoints, middle_home_pose, vel_js, acc_js, false);  // 관절 공간 이동
    taskPushBack_delay(task_toolpoints, 50);
    taskPushBack_jsMove2(task_toolpoints, middle_tag_pose, vel_js, acc_js, false);  // 관절 공간 이동
    taskPushBack_delay(task_toolpoints, 50);
    taskPushBack_csMove2(task_toolpoints, tool_point, vel_cs, acc_cs, false); // 절대 좌표 이동
    taskPushBack_delay(task_toolpoints, 50);  // 50ms 대기

    std::cout << "[DEBUG] Tool Point Coordinates: ";
    for (const auto& value : tool_point) {
        std::cout << value << " ";
    }
    std::cout << std::endl;

    std::cout << "[DEBUG] Total tasks generated: " << task_toolpoints.size() << std::endl;

    return task_toolpoints;
}


std::vector<UnitTask> TaskPlannerCooking::LangSAMMoveTask(double x, double y, double z, double roll, double pitch, double yaw) {
    std::vector<UnitTask> task_toolpoints;

    // 속도 및 가속도 설정
    const double vel_cs = 0.015;
    const double acc_cs = 0.03;
    const double vel_js = 15;
    const double acc_js = 30;

    // LangSAM 좌표로 이동
    std::vector<double> langsam_point = {x, y, z, roll, pitch, yaw};
    //taskPushBack_setTCP(task_toolpoints, {0, 0, 0, 0, 0, 90});  // TCP 설정
    taskPushBack_csMove2(task_toolpoints, langsam_point, vel_cs, acc_cs, false); // 절대 좌표 이동
    taskPushBack_delay(task_toolpoints, 50);  // 100ms 대기

    // 홈 포즈로 복귀
    const JsDouble home_pose = {-39.41, 132.64, -96.56, 2.81, -33.83, -143.11};
    taskPushBack_jsMove2(task_toolpoints, home_pose, vel_js, acc_js, false);  // 관절 공간 이동
    taskPushBack_delay(task_toolpoints, 50);

    return task_toolpoints;
}


//// NOTICE: COPY THIS CODE
// std::vector<UnitTask> TaskPlannerCooking::genMotionGraphy_Task_C3_Mode1(bool is_marker_applied) {
//     std::vector<UnitTask> task_c3_mode_1;


//     return task_c3_mode_1;
// }

