#include "task_planner_default.hpp"
#include <functional>
#include <tuple>
#include <utility>

#define TASK_LIST \
    X2("SAVE_POINT", savePoint_pushBack) \
    X2("TASK_REWIND", taskPushBack_rewind) \
    X2("TASK_TIC", taskPushBack_tic) \
    X2("TASK_TOC", taskPushBack_toc) \
    X2("TASK_DELAY", taskPushBack_delay) \
    X2("TASK_PAUSE", taskPushBack_pause) \
    X2("TASK_CSMOVE_REDUNDANT", taskPushBack_csMove_redundant) \
    X2("TASK_CSMOVE", taskPushBack_csMove) \
    X2("TASK_JSMOVE", taskPushBack_jsMove) \
    X2("TASK_JSMOVE_ONLY_J6", taskPushBack_jsMoveOnlyJ6) \
    X2("TASK_TCPMOVE", taskPushBack_tcpMove) \
    X2("TASK_DTC", taskPushBack_setDtc) \
    X2("TASK_SET_TCP", taskPushBack_setTCP) \
    X2("TASK_RESET_TCP", taskPushBack_resetTCP) \
    X2("TASK_SAVE_FORCE_DATA", taskPushBack_saveForceData) \
    X2("TASK_SET_FORCE_BIAS", taskPushBack_setForceBias) \
    X2("TASK_LOAD_IDENTIFICATION", taskPushBack_loadIdentification) \
    X2("TASK_IMPEDANCE_ON", taskPushBack_impedance_on) \
    X2("TASK_IMPEDANCE_OFF", taskPushBack_impedance_off) \
    X2("TASK_IRLGRP_MOVE", taskPushBack_irl_grp_cmd) \
    X2("TASK_IRLGRP_INIT", taskPushBack_irl_grp_init) \
    X2("TASK_IRLGRP_STOP", taskPushBack_irl_grp_stop) \
    X2("TASK_IRLGRP_ENABLE", taskPushBack_irl_grp_enable) \
    X2("TASK_IRLGRP_DISABLE", taskPushBack_irl_grp_disable) \
    X2("TASK_IRLGRP", taskPushBack_irl_grp) \
    X2("TASK_CSMOVE2", taskPushBack_csMove2) \
    X2("TASK_CSMOVE2_FOR_TEACHING", taskPushBack_csMove2ForTeaching) \
    X2("TASK_JSMOVE2", taskPushBack_jsMove2) \
    X2("TASK_JSMOVE2_FOR_TEACHING", taskPushBack_jsMove2ForTeaching) \
    X2("TASK_CSMOVE_TOOL_FRAME", taskPushBack_csMoveToolFrame) \
    X2("TASK_CSMOVE_TOOL_FRAME_KEYCODE", taskPushBack_csMoveToolFrameKeycode) \
    X2("TASK_CSMOVE_TOOL_FRAME_FOR_TEACHING", taskPushBack_csMoveToolFrameForTeaching) \
    X2("TASK_CSMOVE_BASE_FRAME_TEACHING_TARGET_POSE", taskPushBack_csMoveBaseFrameTeachingTargetPose) \
    X2("TASK_CSMOVE_TOOL_FRAME_TEACHING_TARGET_POSE", taskPushBack_csMoveToolFrameTeachingTargetPose) \
    X2("TASK_SET_TEACHING_TARGET_POSE", taskPushBack_setTeachingTargetPose) \
    X2("TASK_SET_TEACHING_TARGET_POSE_WITH_MARKER_DETECTION", taskPushBack_setTeachingTargetPoseWithMarkerDetection) \
    X2("TASK_SEND_TASK_LOG", taskPushBack_sendTaskLog) \
    X2("TASK_RECOG_POSE_TAG_CSMOVE", taskPushBack_taskRecog_csMoveMotionTag) \
    X2("TASK_POSE_ESTIMATION_CAD_MATCHING", taskPushBack_templateMatching) \
    X2("TASK_POSE_ESTIMATION_AI", taskPushBack_poseEstimation) \
    X2("TASK_JSON_POSE_TAG_CSMOVE", taskPushBack_taskJSON_csMoveMotionTag) \
    X2("TASK_POSE_ESTIMATION_AI", taskPushBack_poseEstimation) \
    X2("TASK_GRASPING_OPTIMAL_POSE_BIN_PICKING", taskPushBack_doGrasping_binPicking) \
    X2("TASK_GRASPING_SUB_OPTIMAL_POSE_BIN_PICKING", taskPushBack_doSubGrasping_binPicking) \
    X2("TASK_GRASPING_AI_ESTIMATED_POSE_BIN_PICKING", taskPushBack_doGrasping_binPicking_poseEstResults) \
    X2("TASK_3D_SCANNING", taskPushBack_taskRecog_3DScanning) \
    X2("TASK_3D_SCANNING_AND_MATCHING", taskPushBack_taskRecog_3DScanningAndMatching) \
    X2("TASK_CHECK_MATCHING_FINISHED", taskPushBack_taskRecog_checkMatchingFinished) \
    X2("TASK_CHECK_MATCHING_FINISHED_AND_TIP_CHANGING", taskPushBack_checkMatchingFinishedAndTipChanging) \
    X2("TASK_SET_TIP_CHANGING_FLAG", taskPushBack_setTipChangingFlag) \
    X2("TASK_DS_CUSTOM_CODE", taskPushBack_customCode) \
    X2("TASK_KORAS_GRIPPER_INITIALIZE", taskPushBack_initializeGripper) \
    X2("TASK_KORAS_GRIPPER_INITIALIZE_2", taskPushBack_initializeVer2Gripper) \
    X2("TASK_KORAS_SET_GRIPPER_DRIVER", taskPushBack_setGripperDriver) \
    X2("TASK_KORAS_SET_GRIPPER_MIN_VALUE", taskPushBack_setGripperMinValue) \
    X2("TASK_KORAS_SET_GRIPPER_MAX_VALUE", taskPushBack_setGripperMaxValue) \
    X2("TASK_START_STACKING_MODE", taskPushBack_startStackingMode) \
    X2("TASK_STOP_STACKING_MODE", taskPushBack_stopStackingMode) \
    X2("TASK_GET_MEASURED_FORCE", taskPushBack_getMeasuredForce) \
    X2("TASK_SET_MEASURED_FORCE", taskPushBack_setMeasuredForce) \
    X2("TASK_KORAS_GRIPPER_COMMAND", taskPushBackKORASGripperCmd) \
    X2("TASK_KORAS_GRIPPER_COMMAND_FOR_TEACHING", taskPushBackKORASGripperCmdForTeaching) \
    X2("TASK_KORAS_GRIPPER_COMMAND_TOOL_DETACHING", taskPushBackKORASGripperCmdToolDetaching) \
    X2("TASK_RECOG_KORAS_GRIPPER_COMMAND", taskPushBack_taskRecog_KORASGripperCmd) \
    X2("TASK_RECOG_KORAS_GRIPPER_CHECK_GRASPING", taskPushBack_taskRecog_KORASGripperCheckGrasping) \
    X2("TASK_PLC_MODBUS_COMMAND", taskPushBackPLCModbusCmd) \
    X2("TASK_GRIPPER_RELAY_CONTROL_COMMAND", taskPushBackGripperRelayCmd) \
    X2("TASK_SELECT_ROBOT_TCP", taskPushBack_selectTCP) \
    X2("TASK_RECOG_SELECT_ROBOT_TCP", taskPushBack_taskRecog_selectTCP) \
    X2("TASK_RECOG_TOOL_CHANGING_ATTACH_JSMOVE", taskPushBack_taskRecog_ToolChangingAttach_jsMove) \
    X2("TASK_RECOG_TOOL_CHANGING_DETACH_JSMOVE", taskPushBack_taskRecog_ToolChangingDetach_jsMove) \
    X2("TASK_RECOG_CURRENT_TOOL_DETACHING_JSMOVE", taskPushBack_taskRecog_CurrentToolDetaching_jsMove) \
    X2("TASK_RECOG_TOOL_CHANGING_DEFAULT_ATTACH_JSMOVE", taskPushBack_taskRecog_ToolChangingDefaultAttach_jsMove) \
    X2("TASK_RECOG_TOOL_CHANGING_DEFAULT_DETACH_JSMOVE", taskPushBack_taskRecog_ToolChangingDefaultDetach_jsMove) \
    X2("TASK_RECOG_TOOL_CHANGING_GOAL_CSMOVE", taskPushBack_taskRecog_ToolChangingGoal_csMove) \
    X2("TASK_RECOG_CURRENT_TIP_CHANGING_DETACH_JSMOVE", taskPushBack_taskRecog_CurrentTipChangingDetach_jsMove) \
    X2("TASK_RECOG_TIP_CHANGING_ATTACH_JSMOVE", taskPushBack_taskRecog_TipChangingAttach_jsMove) \
    X2("TASK_RECOG_TIP_CHANGING_DETACH_JSMOVE", taskPushBack_taskRecog_TipChangingDetach_jsMove) \
    X2("TASK_RECOG_TIP_CHANGING_DEFAULT_ATTACH_JSMOVE", taskPushBack_taskRecog_TipChangingDefaultAttach_jsMove) \
    X2("TASK_RECOG_TIP_CHANGING_DEFAULT_DETACH_JSMOVE", taskPushBack_taskRecog_TipChangingDefaultDetach_jsMove) \
    X2("TASK_RECOG_TIP_CHANGING_SET_ROBOT_TCP", taskPushBack_taskRecog_TipChangingSetTCP) \
    X2("TASK_RECOG_TIP_CHANGING_SET_CURRENT_TIP_INDEX", taskPushBack_taskRecog_TipChangingSetCurrentTipIndex) \
    X2("TASK_DETACH_COUNTING", taskPushBack_countDetachingPose) \
    X2("TASK_MOVE_TO_TAG", taskPushBack_moveToTag) \
    X2("TASK_APRIL_TAG_DETECTION", taskPushBack_aprilTagDetection) \
    X2("TASK_ROBOT_ENABLE", taskPushBack_robotEnable) \
    X2("TASK_ROBOT_DISABLE", taskPushBack_robotDisable) \
    X2("TASK_SET_FLAG_ON_ROBOT_ENABLE_DISABLE_MODE", taskPushBack_setFlagRobotEnableDisableMode) \
    X2("TASK_START_TAG_RECOG", taskPushBack_startTagRecog) \
    X2("TASK_END_SAVE_TAG_RECOG", taskPushBack_endSaveTagRecog) \
    X2("TASK_SET_CURRENT_TAG", taskPushBack_setCurrentTag) \
    X2("TASK_SET_CURRENT_TAG_FROM_JSON", taskPushBack_setCurrentTagFromJSON) \
    X2("TASK_CSMOVE_TAG_BASED", taskPushBack_csMove_tagBased) \
    X2("TASK_CSMOVE_TO_TAG", taskPushBack_csMove_toTag) \
    X2("TASK_CSMOVE_TO_TAG_FOR_APPROACHING", taskPushBack_csMove_toTag_forApproaching) \
    X2("TASK_CSMOVE_TO_TAG_FOR_CONVERGING", taskPushBack_csMove_toTag_forConverging) \
    X2("TASK_CSMOVE_TAG_SEARCH", taskPushBack_csMove_tagSearch) \
    X2("TASK_SET_CAM_AUTO_FOCUS", taskPushBack_setCameraAutoFocus) \
    X2("TASK_PYBIND_CSMOVE", taskPushBack_taskPybindCsMove) \
    //X2("TASK_RECOG_BLENDING", taskPushBack_taskRecog_moveBlending) \
    //X2("TASK_BLENDING", taskPushBack_moveBlending) \

// Specialization of stringToEnum for KR_GRP
template <>
KR_GRP stringToEnum<KR_GRP>(const std::string& enum_str) {
    if (enum_str == "VEL_CTRL") return KR_GRP::VEL_CTRL;
    else if (enum_str == "INIT") return KR_GRP::INIT;
    else if (enum_str == "OPEN") return KR_GRP::OPEN;
    else if (enum_str == "CLOSE") return KR_GRP::CLOSE;
    else if (enum_str == "POS_CTRL") return KR_GRP::POS_CTRL;
    else if (enum_str == "INIT_2") return KR_GRP::INIT_2;
    else if (enum_str == "VACUUM_ON") return KR_GRP::VACUUM_ON;
    else if (enum_str == "VACUUM_OFF") return KR_GRP::VACUUM_OFF;
    else if (enum_str == "CHANGE_ADDRESS") return KR_GRP::CHANGE_ADDRESS;
    else throw std::runtime_error("Unknown KR_GRP string: " + enum_str);
}


template <typename Ret, typename ClassType, typename... Args>
void TaskPlannerDefault::addTaskToMap(
    const std::string& taskName,
    Ret (ClassType::*method)(std::vector<UnitTask>&, Args...),
    std::map<std::string, std::function<void(std::vector<UnitTask>&, const nlohmann::json&)>>& taskMap
) {
    taskMap[taskName] = [this, method, taskName](std::vector<UnitTask>& tasks, const nlohmann::json& params) {
        try {
            auto func = [this, method](std::vector<UnitTask>& tasks, Args... args) {
                (this->*method)(tasks, args...);
            };
            // Note the inclusion of decltype(func) in the template parameters
            callTaskPushBack<decltype(func), Args...>(func, tasks, params);
        } catch (const std::exception& e) {
            std::cerr << "Error while processing " << taskName << ": " << e.what() << std::endl;
        }
    };
}

template <typename Func, typename... Args>
void TaskPlannerDefault::callTaskPushBack(Func func, std::vector<UnitTask>& tasks, const nlohmann::json& json_params) {
    callTaskPushBackImpl<Func, Args...>(func, tasks, json_params, std::index_sequence_for<Args...>{});
}

template <typename Func, typename... Args, std::size_t... I>
void TaskPlannerDefault::callTaskPushBackImpl(Func func, std::vector<UnitTask>& tasks, const nlohmann::json& json_params, std::index_sequence<I...>) {
    func(tasks, getJsonArg<Args>(json_params, I)...);
}

void TaskPlannerDefault::initializeTaskPushBackMap() {
    #define X2(task_name, func_name) addTaskToMap(task_name, &TaskPlannerDefault::func_name, task_push_back_map);
    TASK_LIST
    #undef X2
}
TaskPlannerDefault::TaskPlannerDefault() {

    initializeTaskPushBackMap();
    // string file_name("task_planner_default.cpp");
    // config_path_ = __FILE__;
    // config_path_.resize(config_path_.length() - file_name.length());
    // config_path_ += "../../robot_control_config/controller_config/";

    // string pose_file = config_path_ + "load_id_pose.json";
    // std::ifstream ifs;
    // Json::Value pose;
    // ifs.open(pose_file);
    // ifs >> pose;
    // int pose_num = pose.size();
    // load_id_task_.clear();
    // for (int i = 1; i <= pose_num; i++) {
    //     string key_pose = "pose" + std::to_string(i);
    //     JsDouble pose_temp;
    //     for (int j = 0; j < JS_DOF; j++) {
    //         pose_temp[j]  = pose[key_pose][j].asDouble();
    //     }
    //     ref_unit_task_.vel_js = 30;
    //     ref_unit_task_.acc_js = 60;
    //     taskPushBack_jsMove(load_id_task_, pose_temp);
    //     taskPushBack_delay(load_id_task_, 200);
    //     taskPushBack_saveForceData(load_id_task_, std::to_string(i));
    // }
    // taskPushBack_loadIdentification(load_id_task_);
}

TaskPlannerDefault::~TaskPlannerDefault() {

}

void TaskPlannerDefault::setPosePositionDefault() {

}

void TaskPlannerDefault::makeTaskListDefault() {
}

void TaskPlannerDefault::makeGraphyTaskListDefault() {
}

void TaskPlannerDefault::savePoint_pushBack(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = SAVE_POINT;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_rewind(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_REWIND;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_moduleTask(vector<UnitTask>& target_module_task,
                                                 vector<UnitTask>& push_module_task) {
    for (uint i = 0; i < push_module_task.size(); i++) {
        target_module_task.push_back(push_module_task.at(i));
    }
}

void TaskPlannerDefault::taskPushBack_tic(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_TIC;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_toc(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_TOC;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_delay(vector<UnitTask>& target_module_task
    , double delay_10ms) {
    ref_unit_task_.task_mode  = TASK_DELAY;
    if(SW_MODE_GRAPHY) {
        ref_unit_task_.delay_10ms = static_cast<int>(1.25*delay_10ms); // QNODE: 8ms
    } else {
        ref_unit_task_.delay_10ms = static_cast<int>(delay_10ms); // QNODE: 10ms
    }
    target_module_task.push_back(ref_unit_task_);
}
void TaskPlannerDefault::taskPushBack_pause(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode  = TASK_PAUSE;
    target_module_task.push_back(ref_unit_task_);
}

/// Robot move task
void TaskPlannerDefault::taskPushBack_csMove_redundant(vector<UnitTask>& target_module_task,
                                                       CsDouble pose, double q_redundant,
                                                       bool relative) {
    ref_unit_task_.task_mode = TASK_CSMOVE_REDUNDANT;
    ref_unit_task_.relative  = relative;
    ref_unit_task_.x_target  = pose;

    ref_unit_task_.q_redundant = q_redundant;

    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_csMove(vector<UnitTask>& target_module_task,
                                             CsDouble pose, bool relative) {
    ref_unit_task_.task_mode = TASK_CSMOVE;
    ref_unit_task_.relative  = relative;
    ref_unit_task_.x_target  = pose;

    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_jsMove(vector<UnitTask>& target_module_task,
                                             JsDouble joint_position, bool relative) {
    ref_unit_task_.task_mode = TASK_JSMOVE;
    ref_unit_task_.relative  = relative;
    ref_unit_task_.q_target  = joint_position;

    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_jsMoveOnlyJ6(vector<UnitTask>& target_module_task,
                                             JsDouble joint_position, bool relative) {
    ref_unit_task_.task_mode = TASK_JSMOVE_ONLY_J6;
    ref_unit_task_.relative  = relative;
    ref_unit_task_.q_target  = joint_position;

    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_jsMove2(vector<UnitTask>& target_module_task,
                                             JsDouble joint_position, double acc, double vel, bool relative) {
    ref_unit_task_.task_mode = TASK_JSMOVE2;
    ref_unit_task_.relative  = relative;
    ref_unit_task_.q_target  = joint_position;
    ref_unit_task_.acc_js_custom = acc;
    ref_unit_task_.vel_js_custom = vel;

    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_tcpMove(vector<UnitTask>& target_module_task,
                                              CsDouble tcp_pose) {
    ref_unit_task_.task_mode = TASK_TCPMOVE;
    ref_unit_task_.x_target  = tcp_pose;

    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_moveBlending(vector<UnitTask>& target_module_task,
                                                   BlendingTraj traj_blending) {
    ref_unit_task_.task_mode = TASK_BLENDING;
    ref_unit_task_.traj_blending = traj_blending;

    target_module_task.push_back(ref_unit_task_);

    // ref_unit_task_.traj_blending = BlendingTraj();
}

void TaskPlannerDefault::taskPushBack_setDtc(vector<UnitTask>& target_module_task,
                                             JsBool is_dtc_mode, JsDouble target_torque) {
    ref_unit_task_.task_mode = TASK_DTC;
    ref_unit_task_.is_dtc_mode = is_dtc_mode;
    ref_unit_task_.target_torque = target_torque;

    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_torqueGuardOn(std::vector<UnitTask>& target_module_task,
                                                   int joint_index, double threshold) {
    ref_unit_task_.task_mode = TASK_TORQUE_GUARD_ON;
    ref_unit_task_.torque_guard_joint_index = joint_index;
    ref_unit_task_.torque_guard_threshold = threshold;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_torqueGuardOff(std::vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_TORQUE_GUARD_OFF;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_torquePollingOn(std::vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_TORQUE_POLLING_ON;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_torquePollingOff(std::vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_TORQUE_POLLING_OFF;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_label(std::vector<UnitTask>& target_module_task, const std::string& label_name) {
    ref_unit_task_.task_mode = TASK_LABEL;
    ref_unit_task_.label = label_name;
    target_module_task.push_back(ref_unit_task_);
}

/// TCP setting
void TaskPlannerDefault::taskPushBack_drflSetTCP(vector<UnitTask>& target_module_task, string name) {
    ref_unit_task_.task_mode = TASK_SET_DS_TCP;
    ref_unit_task_.demo_name = name;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_setRobotAutoMode(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_SET_ROBOT_MODE_AUTO;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_setRobotManualMode(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_SET_ROBOT_MODE_MANUAL;
    target_module_task.push_back(ref_unit_task_);
}
/// Impedance Mode Setting
void TaskPlannerDefault::taskPushBack_drflSetImpedance(vector<UnitTask>& target_module_task, bool mode_on) {
    ref_unit_task_.task_mode = TASK_SET_DS_SET_COMPLIANCE_MODE;
    ref_unit_task_.mode_on = mode_on;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_setTCP(vector<UnitTask>& target_module_task,
                                             CsDouble tcp) {
    ref_unit_task_.task_mode = TASK_SET_TCP;
    ref_unit_task_.tcp = tcp;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_resetTCP(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_RESET_TCP;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_saveForceData(vector<UnitTask>& target_module_task, string index) {
    ref_unit_task_.task_mode = TASK_SAVE_FORCE_DATA;
    ref_unit_task_.load_id.index = index;
    target_module_task.push_back(ref_unit_task_);
}

/// impedance_
void TaskPlannerDefault::taskPushBack_setForceBias(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_SET_FORCE_BIAS;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_loadIdentification(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_LOAD_IDENTIFICATION;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_impedance_on(vector<UnitTask>& target_module_task,
                                                   CsDouble impedance_stiffness) {
    ref_unit_task_.task_mode = TASK_SET_FORCE_BIAS;
    target_module_task.push_back(ref_unit_task_);

    ref_unit_task_.task_mode = TASK_IMPEDANCE_ON;
    ref_unit_task_.impedance.k = impedance_stiffness;

    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_impedance_off(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_IMPEDANCE_OFF;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_irl_grp_cmd(vector<UnitTask>& target_module_task,
                                                  int grp_pos_percent) {
    ref_unit_task_.task_mode = TASK_IRLGRP_MOVE;
    ref_unit_task_.grp_pos_percent = grp_pos_percent;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_irl_grp_init(vector<UnitTask>& target_module_task,
                                                  int grp_pos_percent) {
    ref_unit_task_.task_mode = TASK_IRLGRP_INIT;
    ref_unit_task_.grp_pos_percent = grp_pos_percent;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_irl_grp_stop(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_IRLGRP_STOP;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_irl_grp_enable (vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_IRLGRP_ENABLE;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_irl_grp_disable(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_IRLGRP_DISABLE;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_irl_grp(vector<UnitTask>& target_module_task,
                                              KR_GRP command, uint16_t value, uint16_t address) {
    ref_unit_task_.task_mode = TASK_IRLGRP;
    ref_unit_task_.grp_command = (uint16_t) command;
    ref_unit_task_.grp_value   = value;
    ref_unit_task_.grp_address = address;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_led_system(vector<UnitTask>& target_module_task, std::string cmd) {
    ref_unit_task_.task_mode = TASK_LED_SYSTEM;
    ref_unit_task_.send_command = cmd;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_relay_system(vector<UnitTask>& target_module_task, std::string cmd) {
    ref_unit_task_.task_mode = TASK_RELAY_SYSTEM;
    ref_unit_task_.send_command = cmd;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_rl(vector<UnitTask>& target_module_task, std::string cmd) {
    ref_unit_task_.task_mode = TASK_RL;
    ref_unit_task_.send_command = cmd;

    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_rl(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_RECOG_RL;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_rlDone(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_RL_DONE;

    target_module_task.push_back(ref_unit_task_);
}

//// using task recognition
void TaskPlannerDefault::taskPushBack_taskRecog_csMove_index(vector<UnitTask>& target_module_task,
                                      unsigned int idx, double acc, double vel, bool isRelative) {
    ref_unit_task_.task_mode = TASK_RECOG_CSMOVE;
    ref_unit_task_.relative  = isRelative;
    ref_unit_task_.bp_param.cs_pose_idx = idx;
    ref_unit_task_.acc_cs_custom = acc;
    ref_unit_task_.vel_cs_custom = vel;

    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_jsMove_index(vector<UnitTask>& target_module_task,
                                      unsigned int idx, double acc, double vel, bool isRelative) {
    ref_unit_task_.task_mode = TASK_RECOG_JSMOVE;
    ref_unit_task_.relative  = isRelative;
    ref_unit_task_.bp_param.js_position_idx = idx;
    ref_unit_task_.acc_js_custom = acc;
    ref_unit_task_.vel_js_custom = vel;

    target_module_task.push_back(ref_unit_task_);
}


void TaskPlannerDefault::taskPushBack_PLCModbusInitializeStatusFromRobotToAMR(vector<UnitTask>& target_module_task,
                                        uint16_t register_address, bool is_lsb, bool status) {
    ref_unit_task_.task_mode = TASK_PLC_INITIALIZE_STATUS_FROM_ROBOT_TO_AMR;
    ref_unit_task_.register_address = register_address;
    ref_unit_task_.is_lsb = is_lsb;
    ref_unit_task_.status = status;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_PLCModbusWriteStatusFromRobotToAMR(vector<UnitTask>& target_module_task,
                                        uint16_t register_address, bool is_lsb, uint16_t bit_address, bool status) {
    ref_unit_task_.task_mode = TASK_PLC_WRITE_STATUS_FROM_ROBOT_TO_AMR;
    ref_unit_task_.register_address = register_address;
    ref_unit_task_.is_lsb = is_lsb;
    ref_unit_task_.bit_address = bit_address;
    ref_unit_task_.status = status;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_setPLCModbusRotationAngle(vector<UnitTask>& target_module_task,
                                        uint16_t register_address, uint16_t bit_address, double angle) {
    ref_unit_task_.task_mode = TASK_PLC_WRITE_ROTATION_ANGLE_FROM_ROBOT_TO_AMR;
    ref_unit_task_.register_address = register_address;
    ref_unit_task_.bit_address = bit_address;
    ref_unit_task_.angle = angle;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_setPLCModbusBarcodeWrite(vector<UnitTask>& target_module_task,
                                        uint16_t register_address, uint16_t bit_address, const std::string& barcode) {
    ref_unit_task_.task_mode = TASK_PLC_WRITE_BARCODE_FROM_ROBOT_TO_AMR;
    ref_unit_task_.register_address = register_address;
    ref_unit_task_.bit_address = bit_address;
    ref_unit_task_.barcode = barcode;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_setPLCDoMonitoringFlagOnOff(vector<UnitTask>& target_module_task,
                                        bool is_flag_on) {
    ref_unit_task_.task_mode = TASK_PLC_DO_MONITORING_FLAG_ON_OFF;
    ref_unit_task_.mode_on = is_flag_on;
    target_module_task.push_back(ref_unit_task_);
}


bool TaskPlannerDefault::blendingPathInit(BlendingTraj &target_traj) {

    flag_blending_traj_error_ = false;

    target_traj.waypoint_num = 0;
    target_traj.is_radius_percent = true;
    target_traj.xs  .clear();
    target_traj.xds .clear();
    target_traj.xdds.clear();
    target_traj.waypoint_xds.clear();
    target_traj.radiuses.clear();
    target_traj.qs_redundant.clear();

    return true;
}

bool TaskPlannerDefault::blendingPathPushBack(BlendingTraj &target_traj, CsDouble x, double xd,
                                              double xdd, double waypoint_xd, double radius,
                                              double q_redundant, bool is_radius_percent) {
    if (flag_blending_traj_error_) {
        ROS_LOG_ERROR("[Blending path] Blending trajectory error state, variable is reset !");
        blendingPathInit(target_traj);
        return false;
    } else {
        if (target_traj.waypoint_num == 0 || target_traj.waypoint_num == 1) {
            target_traj.waypoint_xds.push_back(0);
        } else if (target_traj.waypoint_num >= WAYPOINT_LIMIT - 1) {
            ROS_LOG_ERROR("[Blending path] Waypoint limit error, variable is reset !");
            blendingPathInit(target_traj);
            return false;
        } else {
            target_traj.waypoint_xds.push_back(waypoint_xd);
        }

        if (target_traj.waypoint_num != 0 && target_traj.is_radius_percent != is_radius_percent) {
            ROS_LOG_ERROR("[Blending path] is_radius_percent changed, variable is reset !");
            blendingPathInit(target_traj);
            return false;
        }

        target_traj.xs.push_back(x);
        target_traj.xds.push_back(xd);
        target_traj.xdds.push_back(xdd);
        target_traj.qs_redundant.push_back(q_redundant);

        target_traj.radiuses.push_back(radius);
        target_traj.is_radius_percent = is_radius_percent;
        target_traj.waypoint_num++;
        return true;
    }
}


CsDouble TaskPlannerDefault::arraySumCS(const CsDouble input1, const CsDouble input2) {
    CsDouble return_value;

    for (int i = 0; i < CS_DOF; i++) {
        return_value[i] = input1[i] + input2[i];
    }

    return return_value;
}

JsDouble TaskPlannerDefault::arraySumJS(const JsDouble input1, const JsDouble input2) {
    JsDouble return_value;

    for (int i = 0; i < JS_DOF; i++) {
        return_value[i] = input1[i] + input2[i];
    }

    return return_value;
}

vector<double> TaskPlannerDefault::vectorSum(const vector<double> input1,
                                                  const vector<double> input2) {
    uint size = input1.size();

    if (input1.size() != input2.size()) {
        cout << "\nVector size is different [return input1]" << endl;

        return input1;
    } else {
        vector<double> return_value(size);

        for (uint i = 0; i < size; i++) {
            return_value[i] = input1[i] + input2[i];
        }

        return return_value;
    }
}

void TaskPlannerDefault::vectorMonitor(string string, vector<double> target,
                                       unsigned int rowNum, unsigned int columnNum) {
    cout << string << " is : " << endl;

    for (size_t i = 0; i < rowNum; i++) {
        printf(" ");
        for (size_t  j = 0; j < columnNum; j++) {
            if (target.size() >= i * columnNum + j)	{
                printf("%f\t", target[i * columnNum + j]);
            }
        }
        printf("\n");
    }
}


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
//// Robot Calibration
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
void TaskPlannerDefault::taskPushBack_save_ply(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_ZIVID_SCAN_FOR_CAL;
    target_module_task.push_back(ref_unit_task_);
}


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
//// Bin picking
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

/** @brief Korean: 작업 대기를 위한 단위 작업을 모듈 작업 리스트에 추가한다.
 * @param[in] targetModuleTask : 모듈 작업 리스트
 * @param[in] ref_unit_task_ : 단위 작업
 * @param[in] delay : 대기 시간
 */
void TaskPlannerDefault::taskPushBack_delay_1ms(std::vector<UnitTask>& targetModuleTask,
                                        double delay) {
    ref_unit_task_.task_mode = TASK_DELAY;
    // ref_unit_task_.delay_10ms  = static_cast<int>(delay/10);
    if(SW_MODE_GRAPHY) {
        ref_unit_task_.delay_10ms  = static_cast<int>(1.25*delay/10.0); // QNODE: 8ms
    } else {
        ref_unit_task_.delay_10ms  = static_cast<int>(delay/10.0); // QNODE: 10ms
    }
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_delay_1msForTeaching(std::vector<UnitTask>& targetModuleTask,
                                     double delay) {
    ref_unit_task_.task_mode = TASK_DELAY_FOR_TEACHING;
    if(SW_MODE_GRAPHY) {
        ref_unit_task_.delay_10ms  = static_cast<int>(1.25*delay/10.0); // QNODE: 8ms
    } else {
        ref_unit_task_.delay_10ms  = static_cast<int>(delay/10.0); // QNODE: 10ms
    }
    targetModuleTask.push_back(ref_unit_task_);
}

/** @brief Korean: 작업 대기를 위한 단위 작업을 모듈 작업 리스트에 추가한다.
 * @param[in] targetModuleTask : 모듈 작업 리스트
 * @param[in] ref_unit_task_ : 단위 작업
 * @param[in] delay : 대기 시간
 */
void TaskPlannerDefault::taskPushBack_delayScanning(std::vector<UnitTask>& targetModuleTask,
                                     double delay) {
    ref_unit_task_.task_mode = TASK_DELAY_SCANNING;
    ref_unit_task_.delay_10ms  = delay;

    targetModuleTask.push_back(ref_unit_task_);
}

/** @brief Korean: 직교공간 상의 동작을 위한 단위 작업을 모듈 작업 리스트에 추가한다.
 * @param[in] targetModuleTask : 모듈 작업 리스트
 * @param[in] ref_unit_task_ : 단위 작업
 * @param[in] pose : 목표 자세, [m], [deg]
 * @param[in] acc : 목표 가속도, [m/s^2], [deg/s^2]
 * @param[in] vel : 목표 속도, [m/s], [deg/s]
 * @param[in] isRelative : true(relative mode), false(absolute mode)
 */
void TaskPlannerDefault::taskPushBack_csMove2(std::vector<UnitTask>& targetModuleTask,
                                      std::vector<double> pose, double acc, double vel, bool isRelative) {
    ref_unit_task_.task_mode = TASK_CSMOVE2;
    ref_unit_task_.relative = isRelative;

    CsDouble x_target;
    for (int i = 0; i < CS_DOF; i++) {
        x_target[i] = pose[i];
    }
    ref_unit_task_.x_target = x_target;
    ref_unit_task_.acc_cs_custom = acc;
    ref_unit_task_.vel_cs_custom = vel;
    // printf("Target CS pose: %f %f %f %f %f %f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
    // printf("Target CS acc: %f\n", acc);
    // printf("Target CS vel: %f\n", vel);
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_csMove2ForTeaching(std::vector<UnitTask>& targetModuleTask,
                                      std::vector<double> pose, double acc, double vel, bool isRelative) {
    ref_unit_task_.task_mode = TASK_CSMOVE2_FOR_TEACHING;
    ref_unit_task_.relative = isRelative;

    CsDouble x_target;
    for (int i = 0; i < CS_DOF; i++) {
        x_target[i] = pose[i];
    }
    ref_unit_task_.x_target = x_target;
    ref_unit_task_.acc_cs_custom = acc;
    ref_unit_task_.vel_cs_custom = vel;
    targetModuleTask.push_back(ref_unit_task_);
}


void TaskPlannerDefault::taskPushBack_jsMove2ForTeaching(vector<UnitTask>& target_module_task,
                                             JsDouble joint_position, double acc, double vel, bool relative) {
    ref_unit_task_.task_mode = TASK_JSMOVE2_FOR_TEACHING;
    ref_unit_task_.relative  = relative;
    ref_unit_task_.q_target  = joint_position;
    ref_unit_task_.acc_js_custom = acc;
    ref_unit_task_.vel_js_custom = vel;

    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_checkJ6LimitsAndJsMove2OnlyJ6(vector<UnitTask>& target_module_task,
                                             JsDouble joint_position, double acc, double vel, bool relative) {
    ref_unit_task_.task_mode = TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6;
    ref_unit_task_.relative  = relative;
    ref_unit_task_.q_target  = joint_position;
    ref_unit_task_.acc_js_custom = acc;
    ref_unit_task_.vel_js_custom = vel;

    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_checkJ6LimitsAndJsMove2OnlyJ6_turnMargin(vector<UnitTask>& target_module_task,
                                             JsDouble joint_position, double acc, double vel, bool relative) {
    ref_unit_task_.task_mode = TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6_turnMargin;
    ref_unit_task_.relative  = relative;
    ref_unit_task_.q_target  = joint_position;
    ref_unit_task_.acc_js_custom = acc;
    ref_unit_task_.vel_js_custom = vel;

    target_module_task.push_back(ref_unit_task_);
}

// sb 3 24.11.25
void TaskPlannerDefault::taskPushBack_checkJ6LimitsAndJsMove2OnlyJ6_reverse(vector<UnitTask>& target_module_task,
                                             double acc, double vel) {
    ref_unit_task_.task_mode = TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6_reverse;
    ref_unit_task_.acc_js_custom = acc;
    ref_unit_task_.vel_js_custom = vel;

    target_module_task.push_back(ref_unit_task_);
}


void TaskPlannerDefault::taskPushBack_SetJ6PossibleDirection(vector<UnitTask>& target_module_task,
                                             JsDouble joint_position, double acc, double vel, bool relative) {
    ref_unit_task_.task_mode = TASK_SET_J6_POSSIBLE_DIRECTION;
    ref_unit_task_.relative  = relative;
    ref_unit_task_.q_target  = joint_position;
    ref_unit_task_.acc_js_custom = acc;
    ref_unit_task_.vel_js_custom = vel;

    target_module_task.push_back(ref_unit_task_);
}

// sb 25.04.11
void TaskPlannerDefault::taskPushBack_csmoverotateknobtoinit(vector<UnitTask>& target_module_task,
                                                             double acc, double vel) {
    ref_unit_task_.task_mode = TASK_CSMOVE_ROTATE_KNOB_TO_INIT;
    ref_unit_task_.acc_js_custom = acc;
    ref_unit_task_.vel_js_custom = vel;

    target_module_task.push_back(ref_unit_task_);
}


void TaskPlannerDefault::taskPushBack_JsMove2OnlyJ6_withAssignedDirection(vector<UnitTask>& target_module_task,
                                             JsDouble joint_position, double acc, double vel, bool relative) {
    ref_unit_task_.task_mode = TASK_JSMOVE_ONLY_J6_ASSIGNED_DIRECTION;
    ref_unit_task_.relative  = relative;
    ref_unit_task_.q_target  = joint_position;
    ref_unit_task_.acc_js_custom = acc;
    ref_unit_task_.vel_js_custom = vel;

    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_JsMove2OnlyJ6_withReverseAssignedDirection(vector<UnitTask>& target_module_task,
                                             JsDouble joint_position, double acc, double vel, bool relative) {
    ref_unit_task_.task_mode = TASK_JSMOVE_ONLY_J6_ASSIGNED_DIRECTION_REVERSE;
    ref_unit_task_.relative  = relative;
    ref_unit_task_.q_target  = joint_position;
    ref_unit_task_.acc_js_custom = acc;
    ref_unit_task_.vel_js_custom = vel;

    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_J6RotateByKeycodeAngle(std::vector<UnitTask>& target_module_task, double deg_z) {
    ref_unit_task_.task_mode = TASK_J6_ROTATE_BY_KEYCODE_ANGLE;
    ref_unit_task_.relative  = true;
    // 각도는 QNode(drum_rotation_angle_)에서 읽음. 여기서는 표식으로 저장하지 않음
    JsDouble q_target = {0, 0, 0, 0, 0, 0};
    ref_unit_task_.q_target = q_target;
    ref_unit_task_.acc_js_custom = 30.0;
    ref_unit_task_.vel_js_custom = 30.0;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_jsMove2KeycodeJ6Only(std::vector<UnitTask>& target_module_task, double acc, double vel) {
    ref_unit_task_.task_mode = TASK_JSMOVE2_KEYCODE_J6_ONLY;
    ref_unit_task_.relative  = true;
    // 나머지 관절은 0, J6은 QNode에서 채움
    JsDouble q_target = {0, 0, 0, 0, 0, 0};
    ref_unit_task_.q_target = q_target;
    ref_unit_task_.acc_js_custom = acc;
    ref_unit_task_.vel_js_custom = vel;
    target_module_task.push_back(ref_unit_task_);
}

/** @brief Korean: 직교공간 상의 동작(상대자세 입력인 경우, Tool frame 기준의 동작을 생성, 절대자세 입력의 경우, CSMove와 동일함)을 위한 단위 작업을 모듈 작업 리스트에 추가한다.
 * @param[in] targetModuleTask : 모듈 작업 리스트
 * @param[in] ref_unit_task_ : 단위 작업
 * @param[in] pose : 목표 자세, [m], [deg]
 * @param[in] acc : 목표 가속도, [m/s^2], [deg/s^2]
 * @param[in] vel : 목표 속도, [m/s], [deg/s]
 * @param[in] isRelative : true(relative mode), false(absolute mode)
 */
void TaskPlannerDefault::taskPushBack_csMoveToolFrame(std::vector<UnitTask>& targetModuleTask,
                                      std::vector<double> pose, double acc, double vel, bool isRelative) {
    ref_unit_task_.task_mode = TASK_CSMOVE_TOOL_FRAME;
    ref_unit_task_.relative = isRelative;

    CsDouble x_target;
    for (int i = 0; i < CS_DOF; i++) {
        x_target[i] = pose[i];
    }
    ref_unit_task_.x_target = x_target;
    ref_unit_task_.acc_cs_custom = acc;
    ref_unit_task_.vel_cs_custom = vel;
    // if(isRelative) {
    //     printf("(Tool frame) Target CS pose: %f %f %f %f %f %f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
    //     printf("(Tool frame) Target CS acc: %f\n", acc);
    //     printf("(Tool frame) Target CS vel: %f\n", vel);
    // }
    // else {
    //     printf("Target CS pose: %f %f %f %f %f %f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
    //     printf("Target CS acc: %f\n", acc);
    //     printf("Target CS vel: %f\n", vel);
    // }
    targetModuleTask.push_back(ref_unit_task_);
}


void TaskPlannerDefault::taskPushBack_csMoveToolFrameKeycode(std::vector<UnitTask>& targetModuleTask,
                                      double acc, double vel) {
    ref_unit_task_.task_mode = TASK_CSMOVE_TOOL_FRAME_KEYCODE;
    ref_unit_task_.relative = true;

    // Placeholder: runtime injection in QNode (Tool Rz from drum_rotation_angle_)
    CsDouble x_target;
    for (int i = 0; i < CS_DOF; i++) {
        x_target[i] = 0.0;
    }
    ref_unit_task_.x_target = x_target;
    ref_unit_task_.acc_cs_custom = acc;
    ref_unit_task_.vel_cs_custom = vel;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_csMoveToolFrameKeycode_for_holder(std::vector<UnitTask>& targetModuleTask,
                                      double acc, double vel) {
    ref_unit_task_.task_mode = TASK_CSMOVE_TOOL_FRAME_KEYCODE_FOR_HOLDER;
    ref_unit_task_.relative = true;

    // Placeholder: runtime injection in QNode (Tool Rz from drum_rotation_angle_holder_)
    CsDouble x_target;
    for (int i = 0; i < CS_DOF; i++) {
        x_target[i] = 0.0;
    }
    ref_unit_task_.x_target = x_target;
    ref_unit_task_.acc_cs_custom = acc;
    ref_unit_task_.vel_cs_custom = vel;
    targetModuleTask.push_back(ref_unit_task_);
}



void TaskPlannerDefault::taskPushBack_csMoveToolFrameForTeaching(std::vector<UnitTask>& targetModuleTask,
                                      std::vector<double> pose, double acc, double vel, bool isRelative) {
    ref_unit_task_.task_mode = TASK_CSMOVE_TOOL_FRAME_FOR_TEACHING;
    ref_unit_task_.relative = isRelative;

    CsDouble x_target;
    for (int i = 0; i < CS_DOF; i++) {
        x_target[i] = pose[i];
    }
    ref_unit_task_.x_target = x_target;
    ref_unit_task_.acc_cs_custom = acc;
    ref_unit_task_.vel_cs_custom = vel;
    // if(isRelative) {
    //     printf("(Tool frame) Target CS pose: %f %f %f %f %f %f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
    //     printf("(Tool frame) Target CS acc: %f\n", acc);
    //     printf("(Tool frame) Target CS vel: %f\n", vel);
    // }
    // else {
    //     printf("Target CS pose: %f %f %f %f %f %f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
    //     printf("Target CS acc: %f\n", acc);
    //     printf("Target CS vel: %f\n", vel);
    // }
    targetModuleTask.push_back(ref_unit_task_);
}



void TaskPlannerDefault::taskPushBack_csMoveBaseFrameTeachingTargetPose(std::vector<UnitTask>& targetModuleTask,
                                      std::vector<double> relative_pose_from_teaching_pose, double acc, double vel) {
    ref_unit_task_.task_mode = TASK_CSMOVE_BASE_FRAME_TEACHING_TARGET_POSE;
    ref_unit_task_.relative = false;

    CsDouble x_target;
    for (int i = 0; i < CS_DOF; i++) {
        x_target[i] = relative_pose_from_teaching_pose[i];
    }
    ref_unit_task_.x_target = x_target;
    ref_unit_task_.acc_cs_custom = acc;
    ref_unit_task_.vel_cs_custom = vel;
    // printf("[Base Frame] Target relative CS pose to be add to teaching pose: %0.3f, %0.3f, %0.3f, %0.1f, %0.1f, %0.1f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
    // printf("Target CS acc: %f\n", acc);
    // printf("Target CS vel: %f\n", vel);
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_csMoveToolFrameTeachingTargetPose(std::vector<UnitTask>& targetModuleTask,
                                      std::vector<double> relative_pose_from_teaching_pose, double acc, double vel) {
    ref_unit_task_.task_mode = TASK_CSMOVE_TOOL_FRAME_TEACHING_TARGET_POSE;
    ref_unit_task_.relative = false;

    CsDouble x_target;
    for (int i = 0; i < CS_DOF; i++) {
        x_target[i] = relative_pose_from_teaching_pose[i];
    }
    ref_unit_task_.x_target = x_target;
    ref_unit_task_.acc_cs_custom = acc;
    ref_unit_task_.vel_cs_custom = vel;
    // printf("[Tool Frame] Target relative CS pose to be add to teaching pose: %0.3f, %0.3f, %0.3f, %0.1f, %0.1f, %0.1f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
    // printf("Target CS acc: %f\n", acc);
    // printf("Target CS vel: %f\n", vel);
    targetModuleTask.push_back(ref_unit_task_);
}




void TaskPlannerDefault::taskPushBack_csMoveBaseFrameTeachingTargetPose2(std::vector<UnitTask>& targetModuleTask,
                                      std::vector<double> target_teaching_pose,
                                      std::vector<double> relative_pose_from_teaching_pose, double acc, double vel) {
    ref_unit_task_.task_mode = TASK_CSMOVE_BASE_FRAME_TEACHING_TARGET_POSE2;
    ref_unit_task_.relative = false;

    CsDouble x_target;
    CsDouble x_teaching_target;
    for (int i = 0; i < CS_DOF; i++) {
        x_target[i] = relative_pose_from_teaching_pose[i];
        x_teaching_target[i] = target_teaching_pose[i];
    }
    ref_unit_task_.x_target = x_target;
    ref_unit_task_.x_teaching_target = x_teaching_target;
    ref_unit_task_.acc_cs_custom = acc;
    ref_unit_task_.vel_cs_custom = vel;
    // printf("[Base Frame] Target relative CS pose to be add to teaching pose: %0.3f, %0.3f, %0.3f, %0.1f, %0.1f, %0.1f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
    // printf("Target CS acc: %f\n", acc);
    // printf("Target CS vel: %f\n", vel);
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_csMoveToolFrameTeachingTargetPose2(std::vector<UnitTask>& targetModuleTask,
                                      std::vector<double> target_teaching_pose,
                                      std::vector<double> relative_pose_from_teaching_pose, double acc, double vel) {
    ref_unit_task_.task_mode = TASK_CSMOVE_TOOL_FRAME_TEACHING_TARGET_POSE2;
    ref_unit_task_.relative = false;

    CsDouble x_target;
    CsDouble x_teaching_target;
    for (int i = 0; i < CS_DOF; i++) {
        x_target[i] = relative_pose_from_teaching_pose[i];
        x_teaching_target[i] = target_teaching_pose[i];
    }
    ref_unit_task_.x_target = x_target;
    ref_unit_task_.x_teaching_target = x_teaching_target;
    ref_unit_task_.acc_cs_custom = acc;
    ref_unit_task_.vel_cs_custom = vel;
    // printf("[Tool Frame] Target relative CS pose to be add to teaching pose: %0.3f, %0.3f, %0.3f, %0.1f, %0.1f, %0.1f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
    // printf("Target CS acc: %f\n", acc);
    // printf("Target CS vel: %f\n", vel);
    targetModuleTask.push_back(ref_unit_task_);
}

/** @brief Korean: 직교공간 상의 동작을 위한 단위 작업을 모듈 작업 리스트에 추가한다.
 * @param[in] targetModuleTask : 모듈 작업 리스트
 * @param[in] ref_unit_task_ : 단위 작업
 * @param[in] demo_tag : demo tag
 * @param[in] motion_tag : motion tag
 * @param[in] isRelative : true(relative mode), false(absolute mode)
 * @param[in] acc : 목표 가속도, [m/s^2], [deg/s^2]
 * @param[in] vel : 목표 속도, [m/s], [deg/s]
 */
void TaskPlannerDefault::taskPushBack_taskJSON_csMoveMotionTag(std::vector<UnitTask>& targetModuleTask, CsDouble pose,
                                                uint16_t demo_tag, uint16_t motion_tag) {
    ref_unit_task_.task_mode = TASK_JSON_POSE_TAG_CSMOVE;
    ref_unit_task_.x_target  = pose;
    ref_unit_task_.bp_param.demo_tag = demo_tag;
    ref_unit_task_.bp_param.motion_tag = motion_tag;
    ref_unit_task_.relative = false;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_moveBlending(std::vector<UnitTask>& targetModuleTask,
                                                   BlendingTraj traj_blending, uint16_t demo_tag, uint16_t motion_tag) {
    ref_unit_task_.task_mode = TASK_RECOG_BLENDING;
    ref_unit_task_.traj_blending = traj_blending;
    ref_unit_task_.bp_param.demo_tag = demo_tag;
    ref_unit_task_.bp_param.motion_tag = motion_tag;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_setTeachingTargetPose(std::vector<UnitTask>& targetModuleTask,
                                      std::string tag_teaching_pose, std::string pose_idx_teaching_pose) {
    ref_unit_task_.task_mode = TASK_SET_TEACHING_TARGET_POSE;
    // ROS_LOG_WARN("[Teaching Tag]: %s - %s\n", tag_teaching_pose.c_str(), pose_idx_teaching_pose.c_str());

    CsDouble x_target;

    //// TODO: JSON Pose load
    //// TODO: 아래에서 체크한 뒤에, task planner 내부로 넣기
    //// FILE PATH
    std::string pose_config_path = "/home/" + USER_NAME + "/[Pose server]/teaching_pose/graphy_demo_cs_waypoint.json";
    //// JSON INPUT
    std::stringstream ss;
    std::ifstream ifs(pose_config_path.c_str());
    nlohmann::json j_in = nlohmann::json::parse(ifs);

    //// 1) JSON Load Teaching Pose
    std::vector<double> teaching_pose_in;
    try {
        // ROS_LOG_WARN("[Tag..]: %s - %s\n", tag_teaching_pose.c_str(), pose_idx_teaching_pose.c_str());
        teaching_pose_in = j_in[tag_teaching_pose.c_str()][pose_idx_teaching_pose.c_str()].get<std::vector<double>>();
        // ROS_LOG_WARN("[JSON LOAD IN TASK PLANNER] TeachingTarget CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", teaching_pose_in[0], teaching_pose_in[1], teaching_pose_in[2], teaching_pose_in[3], teaching_pose_in[4], teaching_pose_in[5]);
    } catch (const std::exception & e) {

        std::cout << e.what() << std::endl;
        // ROS_LOG_ERROR("Check JSON file! - loading error! (teaching pose loading failed)");
        ROS_LOG_ERROR("[Human Teaching] JSON Unmanaged Pose tag has been generated...");

        nlohmann::json j_out = j_in;
        std::vector<double> pose_update(6, 0.0);
        for(int j = 0; j < 3; j++) {
            pose_update[j] = round(pose_update[j]*1e7) / 1e7; // [m]
            pose_update[j + 3] = round(pose_update[j + 3]*1e4) / 1e4; // [deg]
        }
        j_out[tag_teaching_pose.c_str()][pose_idx_teaching_pose.c_str()] = pose_update;

        std::ofstream ofs(pose_config_path.c_str());
        ofs << j_out.dump(4) << std::endl;

        ROS_LOG_ERROR("[Human Teaching] Please restart the program!");
    }
    for (int i = 0; i < CS_DOF; i++) {
        x_target[i] = teaching_pose_in[i];
    }

    ref_unit_task_.x_target = x_target;
    ref_unit_task_.tag_teaching_pose = tag_teaching_pose;
    ref_unit_task_.pose_idx_teaching_pose = pose_idx_teaching_pose;
    // ROS_LOG_WARN("[Task Planner] Set Current TeachingTarget CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_setTeachingTargetPoseWithMarkerDetection(std::vector<UnitTask>& targetModuleTask,
                                      std::string tag_teaching_pose, std::string pose_idx_teaching_pose) {
    ref_unit_task_.task_mode = TASK_SET_TEACHING_TARGET_POSE_WITH_MARKER_DETECTION;
    // ROS_LOG_WARN("[MARKER DETECTION] [Teaching Tag]: %s - %s\n", tag_teaching_pose.c_str(), pose_idx_teaching_pose.c_str());

    // CsDouble x_target;
    // //// FILE PATH
    // // std::string pose_config_path = "/home/" + USER_NAME + "/[Pose server]/marker_pose/graphy_demo_marker_cs_waypoint.json";
    // std::string pose_config_path = "/home/" + USER_NAME + "/[Pose server]/marker_pose/graphy_demo_marker_cs_waypoint.json";
    // //// JSON INPUT
    // std::stringstream ss;
    // std::ifstream ifs(pose_config_path.c_str());
    // nlohmann::json j_in = nlohmann::json::parse(ifs);

    // //// 1) JSON Load Teaching Pose
    // std::vector<double> teaching_pose_in;
    // try {
    //     teaching_pose_in = j_in[tag_teaching_pose.c_str()][pose_idx_teaching_pose.c_str()].get<std::vector<double>>();
    //     // ROS_LOG_WARN("[JSON LOAD IN TASK PLANNER] TeachingTarget CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", teaching_pose_in[0], teaching_pose_in[1], teaching_pose_in[2], teaching_pose_in[3], teaching_pose_in[4], teaching_pose_in[5]);
    // } catch (const std::exception & e) {
    //     std::cout << e.what() << std::endl;
    //     // ROS_LOG_ERROR("Check JSON file! - loading error! (teaching pose loading failed)");
    //     ROS_LOG_ERROR("[Marker Teaching] JSON Unmanaged Pose tag has been generated...");

    //     nlohmann::json j_out = j_in;
    //     std::vector<double> pose_update(6, 0.0);
    //     for(int j = 0; j < 3; j++) {
    //         pose_update[j] = round(pose_update[j]*1e7) / 1e7; // [m]
    //         pose_update[j + 3] = round(pose_update[j + 3]*1e4) / 1e4; // [deg]
    //     }
    //     j_out[tag_teaching_pose.c_str()][pose_idx_teaching_pose.c_str()] = pose_update;

    //     std::ofstream ofs(pose_config_path.c_str());
    //     ofs << j_out.dump(4) << std::endl;

    //     ROS_LOG_ERROR("[Marker Teaching] Please restart the program!");
    // }
    // for (int i = 0; i < CS_DOF; i++) {
    //     x_target[i] = teaching_pose_in[i];
    // }

    // ref_unit_task_.x_target = x_target;
    
    ref_unit_task_.tag_teaching_pose = tag_teaching_pose;
    ref_unit_task_.pose_idx_teaching_pose = pose_idx_teaching_pose;
    // ROS_LOG_WARN("[Task Planner] Set Current TeachingTarget CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_setTeachingTargetPoseBaseFrameInMarkerTask(std::vector<UnitTask>& targetModuleTask,
    std::string tag_teaching_pose, std::string pose_idx_teaching_pose) {
ref_unit_task_.task_mode = TASK_SET_TEACHING_TARGET_POSE_BASE_FRAME_IN_MARKER_TASK;

ref_unit_task_.tag_teaching_pose = tag_teaching_pose;
ref_unit_task_.pose_idx_teaching_pose = pose_idx_teaching_pose;
// ROS_LOG_WARN("[Task Planner] Set Current TeachingTarget CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_sendTaskLog(std::vector<UnitTask>& targetModuleTask, std::string log) {
    ref_unit_task_.task_mode = TASK_SEND_TASK_LOG;
    ref_unit_task_.task_log = log;
    targetModuleTask.push_back(ref_unit_task_);
}
// sb 25.02.10
void TaskPlannerDefault::task_index_count(std::vector<UnitTask>& targetModuleTask, int task_index_n){
    ref_unit_task_.task_mode = TASK_INDEX_COUNT;
    ref_unit_task_.task_index_n = task_index_n;
    targetModuleTask.push_back(ref_unit_task_);

}


void TaskPlannerDefault::taskPushBack_taskRecog_csMoveMotionTag(std::vector<UnitTask>& targetModuleTask,
                                      CsDouble pose, double acc, double vel, uint16_t demo_tag, uint16_t motion_tag) {
    ref_unit_task_.task_mode = TASK_RECOG_POSE_TAG_CSMOVE;
    ref_unit_task_.relative = false;
    ref_unit_task_.x_target  = pose;
    ref_unit_task_.acc_cs_custom = acc;
    ref_unit_task_.vel_cs_custom = vel;
    ref_unit_task_.bp_param.demo_tag = demo_tag;
    ref_unit_task_.bp_param.motion_tag = motion_tag;
    // printf("Exit bin relative pose: %f %f %f %f %f %f\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    // printf("Target CS acc: %f\n", acc);
    // printf("Target CS vel: %f\n", vel);
    targetModuleTask.push_back(ref_unit_task_);
}


//// Grasping - template matching
void TaskPlannerDefault::taskPushBack_templateMatching(std::vector<UnitTask>& targetModuleTask, bool do_sampling, size_t sampling_num) {
    ref_unit_task_.task_mode = TASK_POSE_ESTIMATION_CAD_MATCHING; // Template matching
    ref_unit_task_.bp_param.detection.do_scan_sampling = do_sampling;
    ref_unit_task_.bp_param.detection.sampling_num = sampling_num;
    targetModuleTask.push_back(ref_unit_task_);
}


//// Grasping - pose estimation (ud)
void TaskPlannerDefault::taskPushBack_poseEstimation(std::vector<UnitTask>& targetModuleTask, bool do_sampling, size_t sampling_num) {
    ref_unit_task_.task_mode = TASK_POSE_ESTIMATION_AI; // Pose estimation (ud)
    ref_unit_task_.bp_param.detection.do_scan_sampling = do_sampling;
    ref_unit_task_.bp_param.detection.sampling_num = sampling_num;
    targetModuleTask.push_back(ref_unit_task_);
}

//// Grasping - do grasping - bin picking
void TaskPlannerDefault::taskPushBack_doGrasping_binPicking(std::vector<UnitTask>& targetModuleTask,
                                                    double acc, double vel) {
    ref_unit_task_.task_mode = TASK_GRASPING_OPTIMAL_POSE_BIN_PICKING; // grasping
    ref_unit_task_.relative = false; // false: absolute, true: relative
    ref_unit_task_.acc_cs_custom = acc;
    ref_unit_task_.vel_cs_custom = vel;
    targetModuleTask.push_back(ref_unit_task_);
}

//// Grasping - do grasping - shared task
void TaskPlannerDefault::taskPushBack_doGrasping_sharedTask(std::vector<UnitTask>& targetModuleTask,
                                                    double acc, double vel, std::string demo_name, std::string object_name) {
    ref_unit_task_.task_mode = TASK_GRASPING_OPTIMAL_POSE_BIN_PICKING_SHARED_TASK; // grasping
    ref_unit_task_.relative = false; // false: absolute, true: relative
    ref_unit_task_.acc_cs_custom = acc;
    ref_unit_task_.vel_cs_custom = vel;
    ref_unit_task_.demo_name = demo_name;
    ref_unit_task_.object_name = object_name;
    targetModuleTask.push_back(ref_unit_task_);
}

//// Grasping - do grasping - bin picking
void TaskPlannerDefault::taskPushBack_doSubGrasping_binPicking(std::vector<UnitTask>& targetModuleTask,
                                                    double acc, double vel) {
    ref_unit_task_.task_mode = TASK_GRASPING_SUB_OPTIMAL_POSE_BIN_PICKING; // grasping
    ref_unit_task_.relative = false; // false: absolute, true: relative
    ref_unit_task_.acc_cs_custom = acc;
    ref_unit_task_.vel_cs_custom = vel;
    // printf("Target CS acc: %f\n", acc);
    // printf("Target CS vel: %f\n", vel);
    targetModuleTask.push_back(ref_unit_task_);
}

//// Grasping - do grasping - bin picking (pose estimation)
void TaskPlannerDefault::taskPushBack_doGrasping_binPicking_poseEstResults(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_GRASPING_AI_ESTIMATED_POSE_BIN_PICKING; // grasping
    ref_unit_task_.relative = false; // false: absolute, true: relative
    targetModuleTask.push_back(ref_unit_task_);
}


void TaskPlannerDefault::taskPushBack_task3DScanningTargetObjectNoWaiting(std::vector<UnitTask>& targetModuleTask, std::string target_object) {
    ref_unit_task_.task_mode = TASK_3D_SCANNING_TARGET_OBJECT_NO_WAITING; // matching topic
    ref_unit_task_.object_name = target_object;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_task3DScanningTargetObject(std::vector<UnitTask>& targetModuleTask, std::string target_object) {
    ref_unit_task_.task_mode = TASK_3D_SCANNING_TARGET_OBJECT; // matching topic
    ref_unit_task_.object_name = target_object;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskMatchingTargetObject(std::vector<UnitTask>& targetModuleTask, std::string target_object, bool do_sampling, size_t sampling_num, std::string demo_name, bool is_symmetric) {
    ref_unit_task_.task_mode = TASK_MATCHING_TARGET_OBJECT; // matching topic
    ref_unit_task_.object_name = target_object;
    ref_unit_task_.bp_param.detection.do_scan_sampling = do_sampling;
    ref_unit_task_.bp_param.detection.sampling_num = sampling_num;
    ref_unit_task_.demo_name = demo_name;
    ref_unit_task_.is_symmetric = is_symmetric;
    targetModuleTask.push_back(ref_unit_task_);
}


void TaskPlannerDefault::taskPushBack_taskRecog_3DScanning(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_3D_SCANNING; // ZIVID scanning2: with task recognition
    targetModuleTask.push_back(ref_unit_task_);
}



void TaskPlannerDefault::taskPushBack_task3DScanning(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_3D_SCANNING_SHARED_TASK_NO_WAITING; // matching topic
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_3DScanning_sharedTask(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_3D_SCANNING_SHARED_TASK; // matching topic
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_matching_sharedTask(std::vector<UnitTask>& targetModuleTask, bool do_sampling, size_t sampling_num, std::string demo_name) {
    ref_unit_task_.task_mode = TASK_MATCHING_SHARED_TASK; // matching topic
    ref_unit_task_.bp_param.detection.do_scan_sampling = do_sampling;
    ref_unit_task_.bp_param.detection.sampling_num = sampling_num;
    ref_unit_task_.demo_name = demo_name;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_3DScanningAndMatching(std::vector<UnitTask>& targetModuleTask, bool do_sampling, size_t sampling_num) {
    ref_unit_task_.task_mode = TASK_3D_SCANNING_AND_MATCHING; // matching topic
    ref_unit_task_.bp_param.detection.do_scan_sampling = do_sampling;
    ref_unit_task_.bp_param.detection.sampling_num = sampling_num;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_SharedTaskZividScanningAndMatching(std::vector<UnitTask>& targetModuleTask, bool do_sampling, size_t sampling_num) {
    ref_unit_task_.task_mode = TASK_RECOG_ZIVID_SCANNING_AND_MATCHING; // matching topic
    ref_unit_task_.bp_param.detection.do_scan_sampling = do_sampling;
    ref_unit_task_.bp_param.detection.sampling_num = sampling_num;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_checkMatchingFinished(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_CHECK_MATCHING_FINISHED;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_checkMatchingFinishedAndTipChanging(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_CHECK_MATCHING_FINISHED_AND_TIP_CHANGING;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_setTipChangingFlag(std::vector<UnitTask>& targetModuleTask, bool task_flag) {
    ref_unit_task_.task_mode = TASK_SET_TIP_CHANGING_FLAG;
    ref_unit_task_.bp_param.tip_changing_task_flag = task_flag;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_customCode(std::vector<UnitTask>& targetModuleTask, std::string script) {
    ref_unit_task_.task_mode = TASK_DS_CUSTOM_CODE;
    ref_unit_task_.task_log = script;
    ref_unit_task_.delay_10ms = 100;
    targetModuleTask.push_back(ref_unit_task_);
}

/** @brief Korean: 그리퍼를 초기화한다.
 * @param[in] targetModuleTask : 모듈 작업 리스트
 * @param[in] ref_unit_task_ : 단위 작업
 * @param[in] grp_driver_idx : Driver #
 */
void TaskPlannerDefault::taskPushBack_initializeGripper(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_KORAS_GRIPPER_INITIALIZE;
    targetModuleTask.push_back(ref_unit_task_);
}
void TaskPlannerDefault::taskPushBack_initializeVer2Gripper(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_KORAS_GRIPPER_INITIALIZE_2;
    targetModuleTask.push_back(ref_unit_task_);
}

/** @brief Korean: 그리퍼 드라이버를 변경한다.
 * @param[in] targetModuleTask : 모듈 작업 리스트
 * @param[in] ref_unit_task_ : 단위 작업
 * @param[in] grp_driver_idx : Driver #
 */
void TaskPlannerDefault::taskPushBack_setGripperDriver(std::vector<UnitTask>& targetModuleTask, unsigned int grp_driver_idx) {
    ref_unit_task_.task_mode = TASK_KORAS_SET_GRIPPER_DRIVER;
    ref_unit_task_.bp_param.grp_driver_idx = grp_driver_idx;
    targetModuleTask.push_back(ref_unit_task_);
}

/** @brief Korean: 그리퍼의 최소 거리를 설정한다.
 * @param[in] targetModuleTask : 모듈 작업 리스트
 * @param[in] ref_unit_task_ : 단위 작업
 */
void TaskPlannerDefault::taskPushBack_setGripperMinValue(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_KORAS_SET_GRIPPER_MIN_VALUE;
    targetModuleTask.push_back(ref_unit_task_);
}

/** @brief Korean: 그리퍼의 최소 거리를 설정한다.
 * @param[in] targetModuleTask : 모듈 작업 리스트
 * @param[in] ref_unit_task_ : 단위 작업
 */
void TaskPlannerDefault::taskPushBack_setGripperMaxValue(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_KORAS_SET_GRIPPER_MAX_VALUE;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_startStackingMode(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_START_STACKING_MODE;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_stopStackingMode(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_STOP_STACKING_MODE;
    targetModuleTask.push_back(ref_unit_task_);
}

/** @brief Korean: 부하동정 수행 시 힘/토크 정보 수집을 위한 단위 작업을 모듈 작업 리스트에 추가한다.
 * @param[in] targetModuleTask : 모듈 작업 리스트
 * @param[in] ref_unit_task_ : 단위 작업
 * @param[in] idx : 데이터 로깅(data logging) 자세 인덱스
 */
void TaskPlannerDefault::taskPushBack_getMeasuredForce(std::vector<UnitTask>& targetModuleTask,
                                      unsigned int idx) {
    ref_unit_task_.task_mode = TASK_GET_MEASURED_FORCE;
    ref_unit_task_.bp_param.js_position_idx = idx;
    targetModuleTask.push_back(ref_unit_task_);
}

/** @brief Korean: 부하동정 수행 시 수집된 힘/토크 정보를 저장하기 위한 단위 작업을 모듈 작업 리스트에 추가한다.
 * @param[in] targetModuleTask : 모듈 작업 리스트
 * @param[in] ref_unit_task_ : 단위 작업
 */
void TaskPlannerDefault::taskPushBack_setMeasuredForce(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_SET_MEASURED_FORCE;
    targetModuleTask.push_back(ref_unit_task_);
}




//// KORAS gripper ON/OFF
void TaskPlannerDefault::taskPushBackKORASGripperCmd(std::vector<UnitTask>& targetModuleTask,
                                     uint16_t grp_cmd, int16_t position, uint16_t speed) {
    ref_unit_task_.task_mode = TASK_KORAS_GRIPPER_COMMAND;
    if(grp_cmd == KORAS_GRP_CMD_CLOSE || grp_cmd == KORAS_GRP_CMD_VAC_ON) {
        ref_unit_task_.bp_param.grip_state  = true;
    } else {
        ref_unit_task_.bp_param.grip_state  = false;
    }
    ref_unit_task_.bp_param.gripper_command  = grp_cmd; // 101: initialize, 102: open, 103: close, 104: pos ctrl, 106: vac on, 107: vac off
    ref_unit_task_.bp_param.gripper_param[0] = position;

    if(GRIPPER_MODE_FAIRINO) {
        if(grp_cmd == (uint16_t)KR_GRP::OPEN) {
            ref_unit_task_.bp_param.gripper_param[0] = 0;
        } else if(grp_cmd == (uint16_t)KR_GRP::CLOSE) {
            ref_unit_task_.bp_param.gripper_param[0] = 10000;
        }
    }
    ref_unit_task_.bp_param.gripper_param[1] = speed;
    // ref_unit_task_.bp_param.gripper_param[2] = force;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBackKORASGripperCmdForTeaching(std::vector<UnitTask>& targetModuleTask,
                                     uint16_t grp_cmd, uint16_t position, uint16_t speed) {
    ref_unit_task_.task_mode = TASK_KORAS_GRIPPER_COMMAND_FOR_TEACHING;
    if(grp_cmd == KORAS_GRP_CMD_CLOSE || grp_cmd == KORAS_GRP_CMD_VAC_ON) {
        ref_unit_task_.bp_param.grip_state  = true;
    } else {
        ref_unit_task_.bp_param.grip_state  = false;
    }
    ref_unit_task_.bp_param.gripper_command  = grp_cmd; // 101: initialize, 102: open, 103: close, 104: pos ctrl, 106: vac on, 107: vac off
    ref_unit_task_.bp_param.gripper_param[0] = position;
    ref_unit_task_.bp_param.gripper_param[1] = speed;
    // ref_unit_task_.bp_param.gripper_param[2] = force;
    targetModuleTask.push_back(ref_unit_task_);
}


//// KORAS gripper ON/OFF
void TaskPlannerDefault::taskPushBackKORASGripperCmdToolDetaching(std::vector<UnitTask>& targetModuleTask,
                                     uint16_t grp_cmd, uint16_t position, uint16_t speed) {
    ref_unit_task_.task_mode = TASK_KORAS_GRIPPER_COMMAND_TOOL_DETACHING;
    if(grp_cmd == KORAS_GRP_CMD_CLOSE || grp_cmd == KORAS_GRP_CMD_VAC_ON) {
        ref_unit_task_.bp_param.grip_state  = true;
    } else {
        ref_unit_task_.bp_param.grip_state  = false;
    }
    ref_unit_task_.bp_param.gripper_command  = grp_cmd; // 101: initialize, 102: open, 103: close, 104: pos ctrl, 106: vac on, 107: vac off
    ref_unit_task_.bp_param.gripper_param[0] = position;
    ref_unit_task_.bp_param.gripper_param[1] = speed;
    // ref_unit_task_.bp_param.gripper_param[2] = force;
    targetModuleTask.push_back(ref_unit_task_);
}



//// KORAS gripper ON/OFF
void TaskPlannerDefault::taskPushBack_taskRecog_KORASGripperCmd(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_RECOG_KORAS_GRIPPER_COMMAND;
    ref_unit_task_.bp_param.grip_state  = false; // true: close, false: open
    ref_unit_task_.bp_param.gripper_command  = (uint16_t)KR_GRP::POS_CTRL;
    targetModuleTask.push_back(ref_unit_task_);
}

//// KORAS gripper ON/OFF
void TaskPlannerDefault::taskPushBack_taskRecog_KORASGripperCheckGrasping(std::vector<UnitTask>& targetModuleTask,
                                                                            uint16_t target_position) {

    if(0) {
        ref_unit_task_.task_mode = GO_TO_NEXT_TASK;
        targetModuleTask.push_back(ref_unit_task_);
    } else {
        if(!GRIPPER_MODE_FAIRINO) {
            ref_unit_task_.task_mode = TASK_RECOG_KORAS_GRIPPER_CHECK_GRASPING;
            ref_unit_task_.bp_param.gripper_param[0] = target_position;
            targetModuleTask.push_back(ref_unit_task_);
        } else {
            ref_unit_task_.task_mode = GO_TO_NEXT_TASK;
            targetModuleTask.push_back(ref_unit_task_);
        }
    }

}

//// PCL Command (Modbus TCP)
void TaskPlannerDefault::taskPushBackPLCModbusCmd(std::vector<UnitTask>& targetModuleTask,
                                     int plc_command) {
    ref_unit_task_.task_mode = TASK_PLC_MODBUS_COMMAND;
    ref_unit_task_.bp_param.plc_command = plc_command;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBackPLCModbusCmdWriteRegister(std::vector<UnitTask>& targetModuleTask, int plc_command, int target_value) {
    ref_unit_task_.task_mode = TASK_PLC_MODBUS_COMMAND_WRITE_REGISTER;
    ref_unit_task_.bp_param.plc_command = plc_command;
    ref_unit_task_.bp_param.plc_write_data = target_value;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBackGripperRelayCmd(std::vector<UnitTask>& targetModuleTask,
                                     int relay_signal) {
    ref_unit_task_.task_mode = TASK_GRIPPER_RELAY_CONTROL_COMMAND;
    ref_unit_task_.bp_param.relay_signal = relay_signal;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_led_system(vector<UnitTask>& target_module_task,
                                     int led_signal) {
    ref_unit_task_.task_mode = TASK_RECOG_LED_SYSTEM;
    ref_unit_task_.bp_param.relay_signal = led_signal;
    target_module_task.push_back(ref_unit_task_);
}


////////////////////////////////////////////////////////////////////////////////////////
//// Tool Changing


/** @brief Korean: 로봇 TCP를 변경한다.
 * @param[in] targetModuleTask : 모듈 작업 리스트
 * @param[in] ref_unit_task_ : 단위 작업
 * @param[in] tcp_idx : TCP #
 */
void TaskPlannerDefault::taskPushBack_selectTCP(std::vector<UnitTask>& targetModuleTask, unsigned int tcp_idx) {
    ref_unit_task_.task_mode = TASK_SELECT_ROBOT_TCP;
    ref_unit_task_.bp_param.tcp_idx = tcp_idx;
    targetModuleTask.push_back(ref_unit_task_);
}

/** @brief Korean: 로봇 TCP를 변경한다.
 * @param[in] targetModuleTask : 모듈 작업 리스트
 * @param[in] ref_unit_task_ : 단위 작업
 */
void TaskPlannerDefault::taskPushBack_taskRecog_selectTCP(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_RECOG_SELECT_ROBOT_TCP;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_ToolChangingAttach_jsMove(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_RECOG_TOOL_CHANGING_ATTACH_JSMOVE;
    ref_unit_task_.relative = false;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_ToolChangingDetach_jsMove(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_RECOG_TOOL_CHANGING_DETACH_JSMOVE;
    ref_unit_task_.relative = false;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_CurrentToolDetaching_jsMove(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_RECOG_CURRENT_TOOL_DETACHING_JSMOVE;
    ref_unit_task_.relative = false;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_ToolChangingDefaultAttach_jsMove(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_RECOG_TOOL_CHANGING_DEFAULT_ATTACH_JSMOVE;
    ref_unit_task_.relative = false;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_ToolChangingDefaultDetach_jsMove(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_RECOG_TOOL_CHANGING_DEFAULT_DETACH_JSMOVE;
    ref_unit_task_.relative = false;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_ToolChangingGoal_csMove(std::vector<UnitTask>& targetModuleTask, bool is_tool_attaching, std::vector<double> pose) {
    ref_unit_task_.task_mode = TASK_RECOG_TOOL_CHANGING_GOAL_CSMOVE;
    CsDouble x_target;
    for (int i = 0; i < CS_DOF; i++) {
        x_target[i] = pose[i];
    }
    ref_unit_task_.x_target = x_target;
    ref_unit_task_.relative = false;
    ref_unit_task_.is_tool_attaching = is_tool_attaching;

    targetModuleTask.push_back(ref_unit_task_);
}

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
//// Tip Changing
void TaskPlannerDefault::taskPushBack_taskRecog_TipChangingAttach_jsMove(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_RECOG_TIP_CHANGING_ATTACH_JSMOVE;
    ref_unit_task_.relative = false;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_TipChangingDetach_jsMove(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_RECOG_TIP_CHANGING_DETACH_JSMOVE;
    ref_unit_task_.relative = false;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_CurrentTipChangingDetach_jsMove(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_RECOG_CURRENT_TIP_CHANGING_DETACH_JSMOVE;
    ref_unit_task_.relative = false;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_TipChangingDefaultAttach_jsMove(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_RECOG_TIP_CHANGING_DEFAULT_ATTACH_JSMOVE;
    ref_unit_task_.relative = false;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_TipChangingDefaultDetach_jsMove(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_RECOG_TIP_CHANGING_DEFAULT_DETACH_JSMOVE;
    ref_unit_task_.relative = false;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_TipChangingSetTCP(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_RECOG_TIP_CHANGING_SET_ROBOT_TCP;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskRecog_TipChangingSetCurrentTipIndex(std::vector<UnitTask>& targetModuleTask, size_t current_tip_idx) {
    ref_unit_task_.task_mode = TASK_RECOG_TIP_CHANGING_SET_CURRENT_TIP_INDEX;
    ref_unit_task_.bp_param.current_tip_idx = current_tip_idx;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_countDetachingPose(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_DETACH_COUNTING;
    targetModuleTask.push_back(ref_unit_task_);
}

//
void TaskPlannerDefault::taskPushBack_llm(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_LLM;
    targetModuleTask.push_back(ref_unit_task_);
}
void TaskPlannerDefault::taskPushBack_llm_start(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_LLM_START;
    std::cout << "taskpushback_llm_start" << endl;
    std::cout << "taskpushback_llm_start" << endl;
    targetModuleTask.push_back(ref_unit_task_);
}
void TaskPlannerDefault::taskPushBack_llm_fin(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_LLM_FIN;
    std::cout << "taskpushback_llm_finish" << endl;
    std::cout << "taskpushback_llm_finish" << endl;
    targetModuleTask.push_back(ref_unit_task_);
}
////////////////////////////////////////////////////////////////////////////////////////

void TaskPlannerDefault::taskPushBack_moveToTag(std::vector<UnitTask>& targetModuleTask, unsigned int tag_num) {
    ref_unit_task_.task_mode = TASK_MOVE_TO_TAG;
    ref_unit_task_.tag_num = tag_num;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_aprilTagDetection(std::vector<UnitTask>& target_module_task, unsigned int tag_num) {
    ref_unit_task_.task_mode = TASK_APRIL_TAG_DETECTION;
    ref_unit_task_.tag_num = tag_num;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_robotEnable(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_ROBOT_ENABLE;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_robotDisable(std::vector<UnitTask>& targetModuleTask) {
    ref_unit_task_.task_mode = TASK_ROBOT_DISABLE;
    targetModuleTask.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_setFlagRobotEnableDisableMode(std::vector<UnitTask>& targetModuleTask, bool is_flag_on) {

    if(is_flag_on) {
        ref_unit_task_.task_mode = TASK_SET_FLAG_ON_ROBOT_ENABLE_DISABLE_MODE;
    } else {
        ref_unit_task_.task_mode = TASK_SET_FLAG_OFF_ROBOT_ENABLE_DISABLE_MODE;
    }
    targetModuleTask.push_back(ref_unit_task_);
}

////////////////////////////////////////////////////////////////////////////////////////
// Apriltag related
void TaskPlannerDefault::taskPushBack_startTagRecog(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_START_TAG_RECOG;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_endSaveTagRecog(vector<UnitTask>& target_module_task) {
    ref_unit_task_.task_mode = TASK_END_SAVE_TAG_RECOG;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_setCurrentTag(vector<UnitTask>& target_module_task,
                                                    string tag_name) {
    ref_unit_task_.task_mode = TASK_SET_CURRENT_TAG;
    ref_unit_task_.tag_name  = tag_name;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_setCurrentTagFromJSON(vector<UnitTask>& target_module_task,
                                                    string tag_name) {
    ref_unit_task_.task_mode = TASK_SET_CURRENT_TAG_FROM_JSON;
    ref_unit_task_.tag_name  = tag_name;
    target_module_task.push_back(ref_unit_task_);
}


void TaskPlannerDefault::taskPushBack_csMove_tagBased(vector<UnitTask>& target_module_task,
                                                      CsDouble pose_tag_based) {
    ref_unit_task_.task_mode = TASK_CSMOVE_TAG_BASED;
    ref_unit_task_.x_target_tag_based = pose_tag_based;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_csMove_toTag(vector<UnitTask>& target_module_task,
                                                   uint distance_mm) {
    ref_unit_task_.task_mode = TASK_CSMOVE_TO_TAG;
    ref_unit_task_.des_tag_distance_mm = distance_mm;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_csMove_toTag_forConverging(vector<UnitTask>& target_module_task,
                                                   string tag_name, uint distance_mm) {
    ref_unit_task_.task_mode = TASK_MARKER_DETECTION_FOR_CONVERGING;
    ref_unit_task_.tag_name  = tag_name;
    ref_unit_task_.des_tag_distance_mm = distance_mm;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_csMove_toTag_forApproaching(vector<UnitTask>& target_module_task,
                                                   string tag_name, uint distance_mm, bool is_second_tag_approach) {
    ref_unit_task_.task_mode = TASK_MARKER_DETECTION_FOR_APPROACHING;
    ref_unit_task_.tag_name  = tag_name;
    ref_unit_task_.des_tag_distance_mm = distance_mm;
    ref_unit_task_.is_second_tag_approach = is_second_tag_approach;
    target_module_task.push_back(ref_unit_task_);
}


void TaskPlannerDefault::taskPushBack_csMove_tagSearch(vector<UnitTask>& target_module_task,
                                                       string tag_name) {
    taskPushBack_startTagRecog(target_module_task);
    taskPushBack_delay(target_module_task, 200);
    taskPushBack_endSaveTagRecog(target_module_task);
    taskPushBack_setCurrentTag(target_module_task, tag_name);

    ref_unit_task_.task_mode = TASK_CSMOVE_TO_TAG;
    ref_unit_task_.tag_name  = tag_name;
    target_module_task.push_back(ref_unit_task_);

    // Secondary search
    taskPushBack_startTagRecog(target_module_task);
    taskPushBack_delay(target_module_task, 200);
    taskPushBack_endSaveTagRecog(target_module_task);
    taskPushBack_setCurrentTag(target_module_task, tag_name);

    ref_unit_task_.task_mode = TASK_CSMOVE_TO_TAG;
    ref_unit_task_.tag_name  = tag_name;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_setCameraAutoFocus(vector<UnitTask>& target_module_task,
                                                   bool is_auto_focus_on) {
    ref_unit_task_.task_mode = TASK_SET_CAM_AUTO_FOCUS;
    ref_unit_task_.is_cam_auto_focus_on = is_auto_focus_on;
    target_module_task.push_back(ref_unit_task_);
}

void TaskPlannerDefault::taskPushBack_taskPybindCsMove(vector<UnitTask>& target_module_task,
                                                   std::string func_name) {
    ref_unit_task_.task_mode = TASK_PYBIND_CSMOVE;
    ref_unit_task_.pybind_func_name = func_name;
    target_module_task.push_back(ref_unit_task_);
}


// #if DRFL_CONTROL
// template<typename Func, typename... Args>
// void TaskPlannerDefault::taskPushBack_drfl_api_task(std::vector<UnitTask>& target_module_task, Func func, Args... args) {
//     UnitTask ref_unit_task_;
//     ref_unit_task_.task_mode = Task::TASK_DRFL_API_TASK;
//     ref_unit_task_.function_wrapper = std::make_shared<FunctionWrapper<Func, Args...>>(func, args...);
//     target_module_task.push_back(ref_unit_task_);
// }
// #endif