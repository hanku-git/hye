#include "taskPlannerWidget.hpp"
#include <QFormLayout>
#include <QFile>
#include <QJsonDocument>
#include <QTextStream>

#include <memory>

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <map>
#include <type_traits>
#include <functional>
#include <QDebug>
#include <unordered_map>

namespace taskplan {

// Template function to register member functions without explicit casting
template <typename ClassType>
class LuaBinder {
public:
    LuaBinder(sol::state& lua, ClassType* instance)
        : lua_(lua), instance_(instance) {}

    template <typename Ret, typename... Args>
    void registerFunction(const std::string& funcName, Ret (ClassType::*func)(Args...)) {
        lua_.set_function(funcName, [this, func](Args... args) -> Ret {
            return (instance_->*func)(std::forward<Args>(args)...);
        });
    }

    template <typename Ret, typename... Args>
    void registerFunction(const std::string& funcName, Ret (ClassType::*func)(Args...) const) {
        lua_.set_function(funcName, [this, func](Args... args) -> Ret {
            return (instance_->*func)(std::forward<Args>(args)...);
        });
    }

private:
    sol::state& lua_;
    ClassType* instance_;
};

// UnitTask_ 생성자 구현
CompositeTask::CompositeTask(QString name)
    : name(name) {}

UnitTask_::UnitTask_(QString taskType, QJsonArray params)
    : taskType(taskType), params(params) {}


// Task 생성자 구현
Task::Task(QString name)
    : name(name) {}

#define QPAIR_STR(FIRST, SECOND) QPair<QString, QString>(FIRST, SECOND)

#define DEFINE_TASK(TASK_NAME, LUA_FUNC, LUA_SCRIPT, ...)                     \
    definitions.emplace_back(                                                 \
        TaskDefinitionBuilder()                                               \
            .setTaskType(#TASK_NAME, QList<QPair<QString, QString>>{__VA_ARGS__}) \
            .setLuaFunctionName(#LUA_FUNC)                                    \
            .setLuaScriptTemplate(LUA_SCRIPT)                                  \
            .setLuaRegistration(&TaskPlannerDefault::LUA_FUNC, &planner, targetModuleTask) \
            .build()                                                          \
    )

std::vector<TaskDefinition> TaskPlannerWidget::createTaskDefinitions(TaskPlannerDefault& planner, std::vector<UnitTask>& targetModuleTask) {
    std::vector<TaskDefinition> definitions;

    // original form
    // definitions.emplace_back(TaskDefinitionBuilder()
    //     .setTaskType("TASK_CSMOVE", {{"Pose", "CsDouble"}, {"Relative", "bool"}})
    //     .setLuaFunctionName("taskPushBack_csMove")
    //     .setLuaScriptTemplate("taskPushBack_csMove(${Pose}, ${Relative});")
    //     .setLuaRegistration(&TaskPlannerDefault::taskPushBack_csMove, &planner, targetModuleTask)
    //     .build());

    //     .setTaskType("TASK_RECOG_TIP_CHANGING_ATTACH_JSMOVE", {})
    //     .setLuaFunctionName("taskPushBack_taskRecog_TipChangingAttach_jsMove")
    //     .setLuaScriptTemplate("taskPushBack_taskRecog_TipChangingAttach_jsMove();")
    //     .setLuaRegistration(&TaskPlannerDefault::taskPushBack_taskRecog_TipChangingAttach_jsMove, &planner, targetModuleTask)
    //     .build());


// Task planner function & UI component definition

// DEFINE_TASK(TASK_REWIND, taskPushBack_rewind, "taskPushBack_rewind();");
// DEFINE_TASK(TASK_TIC, taskPushBack_tic, "taskPushBack_tic();");
// DEFINE_TASK(TASK_TOC, taskPushBack_toc, "taskPushBack_toc();");
// DEFINE_TASK(TASK_DELAY, taskPushBack_delay, "taskPushBack_delay(${delay_10ms});",
//     QPAIR_STR("delay_10ms", "int")
// );
// DEFINE_TASK(TASK_PAUSE, taskPushBack_pause, "taskPushBack_pause();");
// DEFINE_TASK(TASK_CSMOVE_REDUNDANT, taskPushBack_csMove_redundant, "taskPushBack_csMove_redundant(${pose}, ${q_redundant}, ${relative});",
//     QPAIR_STR("pose", "CsDouble"),
//     QPAIR_STR("q_redundant", "double"),
//     QPAIR_STR("relative", "bool")
// );
// DEFINE_TASK(TASK_CSMOVE, taskPushBack_csMove, "taskPushBack_csMove(${pose}, ${relative});",
//     QPAIR_STR("pose", "CsDouble"),
//     QPAIR_STR("relative", "bool")
// );
// DEFINE_TASK(TASK_JSMOVE, taskPushBack_jsMove, "taskPushBack_jsMove(${joint_position}, ${relative});",
//     QPAIR_STR("joint_position", "JsDouble"),
//     QPAIR_STR("relative", "bool")
// );
// DEFINE_TASK(TASK_JS_MOVE_ONLYJ6, taskPushBack_jsMoveOnlyJ6, "taskPushBack_jsMoveOnlyJ6(${joint_position}, ${relative});",
//     QPAIR_STR("joint_position", "JsDouble"),
//     QPAIR_STR("relative", "bool")
// );
// DEFINE_TASK(TASK_TCP_MOVE, taskPushBack_tcpMove, "taskPushBack_tcpMove(${tcp_pose});",
//     QPAIR_STR("tcp_pose", "CsDouble")
// );
// DEFINE_TASK(TASK_MOVE_BLENDING, taskPushBack_moveBlending, "taskPushBack_moveBlending(${traj_blending});",
//     QPAIR_STR("traj_blending", "BlendingTraj")
// );
// DEFINE_TASK(TASK_SET_DTC, taskPushBack_setDtc, "taskPushBack_setDtc(${is_dtc_mode}, ${target_torque});",
//     QPAIR_STR("is_dtc_mode", "JsBool"),
//     QPAIR_STR("target_torque", "JsDouble")
// );
DEFINE_TASK(ROBOT_SET_TCP, taskPushBack_setTCP, "taskPushBack_setTCP(${tcp});",
    QPAIR_STR("tcp", "CsDouble")
);
// DEFINE_TASK(TASK_RESET_TCP, taskPushBack_resetTCP, "taskPushBack_resetTCP();");
// DEFINE_TASK(TASK_SAVEFORCEDATA, taskPushBack_saveForceData, "taskPushBack_saveForceData(${index});",
//     QPAIR_STR("index", "string")
// );
// DEFINE_TASK(TASK_SET_FORCE_BIAS, taskPushBack_setForceBias, "taskPushBack_setForceBias();");
// DEFINE_TASK(TASK_LOAD_IDENTIFICATION, taskPushBack_loadIdentification, "taskPushBack_loadIdentification();");
// DEFINE_TASK(TASK_IMPEDANCE_ON, taskPushBack_impedance_on, "taskPushBack_impedance_on(${impedance_stiffness});",
//     QPAIR_STR("impedance_stiffness", "CsDouble")
// );
// DEFINE_TASK(TASK_IMPEDANCE_OFF, taskPushBack_impedance_off, "taskPushBack_impedance_off();");
// DEFINE_TASK(TASK_IRL_GRP_CMD, taskPushBack_irl_grp_cmd, "taskPushBack_irl_grp_cmd(${grp_pos_percent});",
//     QPAIR_STR("grp_pos_percent", "int")
// );
// DEFINE_TASK(TASK_IRL_GRP_INIT, taskPushBack_irl_grp_init, "taskPushBack_irl_grp_init(${grp_pos_percent});",
//     QPAIR_STR("grp_pos_percent", "int")
// );
// DEFINE_TASK(TASK_IRL_GRP_STOP, taskPushBack_irl_grp_stop, "taskPushBack_irl_grp_stop();");
// DEFINE_TASK(TASK_IRL_GRP_ENABLE, taskPushBack_irl_grp_enable, "taskPushBack_irl_grp_enable();");
// DEFINE_TASK(TASK_IRL_GRP_DISABLE, taskPushBack_irl_grp_disable, "taskPushBack_irl_grp_disable();");
// DEFINE_TASK(TASK_IRL_GRP, taskPushBack_irl_grp, "taskPushBack_irl_grp(${command}, ${value}, ${address});",
//     QPAIR_STR("command", "KR_GRP"),
//     QPAIR_STR("value", "uint16_t"),
//     QPAIR_STR("address", "uint16_t")
// );
// DEFINE_TASK(TASK_SAVE_PLY, taskPushBack_save_ply, "taskPushBack_save_ply();");

// DEFINE_TASK(TASK_DELAY_1MS_FOR_TEACHING, taskPushBack_delay_1msForTeaching, "taskPushBack_delay_1msForTeaching(${delay});",
//     QPAIR_STR("delay", "double")
// );
// DEFINE_TASK(TASK_DELAY_SCANNING, taskPushBack_delayScanning, "taskPushBack_delayScanning(${delay});",
//     QPAIR_STR("delay", "double")
// );
DEFINE_TASK(ROBOT_MOVE_CS, taskPushBack_csMove2, "taskPushBack_csMove2(${pose}, ${vel}, ${acc}, ${isRelative});",
    QPAIR_STR("pose", "CsDouble"),
    QPAIR_STR("vel", "double"),
    QPAIR_STR("acc", "double"),
    QPAIR_STR("isRelative", "bool")
);
// DEFINE_TASK(TASK_CSMOVE2_FOR_TEACHING, taskPushBack_csMove2ForTeaching, "taskPushBack_csMove2ForTeaching(${pose}, ${acc}, ${vel}, ${isRelative});",
//     QPAIR_STR("pose", "CsDouble"),
//     QPAIR_STR("acc", "double"),
//     QPAIR_STR("vel", "double"),
//     QPAIR_STR("isRelative", "bool")
// );

// DEFINE_TASK(TASK_JSMOVE2_FOR_TEACHING, taskPushBack_jsMove2ForTeaching, "taskPushBack_jsMove2ForTeaching(${joint_position}, ${acc}, ${vel}, ${relative});",
//     QPAIR_STR("joint_position", "JsDouble"),
//     QPAIR_STR("acc", "double"),
//     QPAIR_STR("vel", "double"),
//     QPAIR_STR("relative", "bool")
// );
DEFINE_TASK(ROBOT_MOVE_CS_TOOL_FRAME, taskPushBack_csMoveToolFrame, "taskPushBack_csMoveToolFrame(${pose}, ${acc}, ${vel}, ${isRelative});",
    QPAIR_STR("pose", "CsDouble"),
    QPAIR_STR("acc", "double"),
    QPAIR_STR("vel", "double"),
    QPAIR_STR("isRelative", "bool")
);
DEFINE_TASK(ROBOT_MOVE_JS, taskPushBack_jsMove2, "taskPushBack_jsMove2(${joint_position}, ${vel}, ${acc}, ${relative});",
    QPAIR_STR("joint_position", "JsDouble"),
    QPAIR_STR("vel", "double"),
    QPAIR_STR("acc", "double"),
    QPAIR_STR("relative", "bool")
);
// DEFINE_TASK(TASK_CSMOVE_TOOL_FRAME_FOR_TEACHING, taskPushBack_csMoveToolFrameForTeaching, "taskPushBack_csMoveToolFrameForTeaching(${pose}, ${acc}, ${vel}, ${isRelative});",
//     QPAIR_STR("pose", "CsDouble"),
//     QPAIR_STR("acc", "double"),
//     QPAIR_STR("vel", "double"),
//     QPAIR_STR("isRelative", "bool")
// );
// DEFINE_TASK(TASK_CSMOVE_BASE_FRAME_TEACHING_TARGET_POSE, taskPushBack_csMoveBaseFrameTeachingTargetPose, "taskPushBack_csMoveBaseFrameTeachingTargetPose(${relative_pose_from_teaching_pose}, ${acc}, ${vel});",
//     QPAIR_STR("relative_pose_from_teaching_pose", "CsDouble"),
//     QPAIR_STR("acc", "double"),
//     QPAIR_STR("vel", "double")
// );
// DEFINE_TASK(TASK_CSMOVE_TOOL_FRAME_TEACHING_TARGET_POSE, taskPushBack_csMoveToolFrameTeachingTargetPose, "taskPushBack_csMoveToolFrameTeachingTargetPose(${relative_pose_from_teaching_pose}, ${acc}, ${vel});",
//     QPAIR_STR("relative_pose_from_teaching_pose", "CsDouble"),
//     QPAIR_STR("acc", "double"),
//     QPAIR_STR("vel", "double")
// );
// DEFINE_TASK(TASK_TASKJSON_CSMOVE_MOTION_TAG, taskPushBack_taskJSON_csMoveMotionTag, "taskPushBack_taskJSON_csMoveMotionTag(${pose}, ${demo_tag}, ${motion_tag});",
//     QPAIR_STR("pose", "CsDouble"),
//     QPAIR_STR("demo_tag", "uint16_t"),
//     QPAIR_STR("motion_tag", "uint16_t")
// );
// DEFINE_TASK(TASK_TASKRECOG_MOVE_BLENDING, taskPushBack_taskRecog_moveBlending, "taskPushBack_taskRecog_moveBlending(${traj_blending}, ${demo_tag}, ${motion_tag});",
//     QPAIR_STR("traj_blending", "BlendingTraj"),
//     QPAIR_STR("demo_tag", "uint16_t"),
//     QPAIR_STR("motion_tag", "uint16_t")
// );
// DEFINE_TASK(TASK_TASK_RECOG_CSMOVE_MOTION_TAG, taskPushBack_taskRecog_csMoveMotionTag, "taskPushBack_taskRecog_csMoveMotionTag(${pose}, ${acc}, ${vel}, ${demo_tag}, ${motion_tag});",
//     QPAIR_STR("pose", "CsDouble"),
//     QPAIR_STR("acc", "double"),
//     QPAIR_STR("vel", "double"),
//     QPAIR_STR("demo_tag", "uint16_t"),
//     QPAIR_STR("motion_tag", "uint16_t")
// );
// DEFINE_TASK(TASK_SET_TEACHING_TARGET_POSE, taskPushBack_setTeachingTargetPose, "taskPushBack_setTeachingTargetPose(${tag_teaching_pose}, ${pose_idx_teaching_pose});",
//     QPAIR_STR("tag_teaching_pose", "string"),
//     QPAIR_STR("pose_idx_teaching_pose", "string")
// );
// DEFINE_TASK(TASK_SET_TEACHING_TARGET_POSE_WITH_MARKER_DETECTION, taskPushBack_setTeachingTargetPoseWithMarkerDetection, "taskPushBack_setTeachingTargetPoseWithMarkerDetection(${tag_teaching_pose}, ${pose_idx_teaching_pose});",
//     QPAIR_STR("tag_teaching_pose", "string"),
//     QPAIR_STR("pose_idx_teaching_pose", "string")
// );

DEFINE_TASK(GRIPPER_MOVE_POS_CTRL, taskPushBackKORASGripperCmd, "taskPushBackKORASGripperCmd(${grp_cmd}, ${position}, ${speed});",
    QPAIR_STR("grp_cmd", "uint"),
    QPAIR_STR("position", "uint"),
    QPAIR_STR("speed", "uint")
);
DEFINE_TASK(ADD_DELAY_1MS, taskPushBack_delay_1ms, "taskPushBack_delay_1ms(${delay});",
    QPAIR_STR("delay", "double")
);
DEFINE_TASK(DISPLAY_TASK_LOG, taskPushBack_sendTaskLog, "taskPushBack_sendTaskLog(${log});",
    QPAIR_STR("log", "string")
);
// DEFINE_TASK(TASK_TEMPLATE_MATCHING, taskPushBack_templateMatching, "taskPushBack_templateMatching(${do_sampling}, ${sampling_num});",
//     QPAIR_STR("do_sampling", "bool"),
//     QPAIR_STR("sampling_num", "size_t")
// );
// DEFINE_TASK(TASK_POSE_ESTIMATION, taskPushBack_poseEstimation, "taskPushBack_poseEstimation(${do_sampling}, ${sampling_num});",
//     QPAIR_STR("do_sampling", "bool"),
//     QPAIR_STR("sampling_num", "size_t")
// );
// DEFINE_TASK(TASK_DO_GRASPING_BINPICKING, taskPushBack_doGrasping_binPicking, "taskPushBack_doGrasping_binPicking(${acc}, ${vel});",
//     QPAIR_STR("acc", "double"),
//     QPAIR_STR("vel", "double")
// );
// DEFINE_TASK(TASK_DO_SUBGRASPING_BINPICKING, taskPushBack_doSubGrasping_binPicking, "taskPushBack_doSubGrasping_binPicking(${acc}, ${vel});",
//     QPAIR_STR("acc", "double"),
//     QPAIR_STR("vel", "double")
// );
// DEFINE_TASK(TASK_DO_GRASPING_BINPICKING_POSE_EST_RESULTS, taskPushBack_doGrasping_binPicking_poseEstResults, "taskPushBack_doGrasping_binPicking_poseEstResults();");
// DEFINE_TASK(TASK_TASK_RECOG_3D_SCANNING, taskPushBack_taskRecog_3DScanning, "taskPushBack_taskRecog_3DScanning();");
// DEFINE_TASK(TASK_TASK_RECOG_3D_SCANNING_AND_MATCHING, taskPushBack_taskRecog_3DScanningAndMatching, "taskPushBack_taskRecog_3DScanningAndMatching(${do_sampling}, ${sampling_num});",
//     QPAIR_STR("do_sampling", "bool"),
//     QPAIR_STR("sampling_num", "size_t")
// );
// DEFINE_TASK(TASK_TASKRECOG_CHECK_MATCHING_FINISHED, taskPushBack_taskRecog_checkMatchingFinished, "taskPushBack_taskRecog_checkMatchingFinished();");
// DEFINE_TASK(TASK_CHECK_MATCHING_FINISHED_AND_TIP_CHANGING, taskPushBack_checkMatchingFinishedAndTipChanging, "taskPushBack_checkMatchingFinishedAndTipChanging();");
// DEFINE_TASK(TASK_SET_TIPCHANGING_FLAG, taskPushBack_setTipChangingFlag, "taskPushBack_setTipChangingFlag(${task_flag});",
//     QPAIR_STR("task_flag", "bool")
// );

// DEFINE_TASK(TASK_INITIALIZE_GRIPPER, taskPushBack_initializeGripper, "taskPushBack_initializeGripper();");

// DEFINE_TASK(TASK_INITIALIZE_VER2_GRIPPER, taskPushBack_initializeVer2Gripper, "taskPushBack_initializeVer2Gripper();");
// DEFINE_TASK(TASK_SET_GRIPPER_DRIVER, taskPushBack_setGripperDriver, "taskPushBack_setGripperDriver(${int});",
//     QPAIR_STR("int", "unsigned")
// );
// DEFINE_TASK(TASK_SET_GRIPPER_MIN_VALUE, taskPushBack_setGripperMinValue, "taskPushBack_setGripperMinValue();");
// DEFINE_TASK(TASK_SET_GRIPPER_MAX_VALUE, taskPushBack_setGripperMaxValue, "taskPushBack_setGripperMaxValue();");
// DEFINE_TASK(TASK_START_STACKING_MODE, taskPushBack_startStackingMode, "taskPushBack_startStackingMode();");
// DEFINE_TASK(TASK_STOP_STACKING_MODE, taskPushBack_stopStackingMode, "taskPushBack_stopStackingMode();");
// DEFINE_TASK(TASK_GET_MEASURED_FORCE, taskPushBack_getMeasuredForce, "taskPushBack_getMeasuredForce(${int});",
//     QPAIR_STR("int", "unsigned")
// );
// DEFINE_TASK(TASK_SETMEASUREDFORCE, taskPushBack_setMeasuredForce, "taskPushBack_setMeasuredForce();");
// DEFINE_TASK(TASK_TASKRECOG_KORASGRIPPERCMD, taskPushBack_taskRecog_KORASGripperCmd, "taskPushBack_taskRecog_KORASGripperCmd();");
// DEFINE_TASK(TASK_TASKRECOG_KORASGRIPPERCHECKGRASPING, taskPushBack_taskRecog_KORASGripperCheckGrasping, "taskPushBack_taskRecog_KORASGripperCheckGrasping(${target_position});",
//     QPAIR_STR("target_position", "uint16_t")
// );
// DEFINE_TASK(TASK_SELECT_TCP, taskPushBack_selectTCP, "taskPushBack_selectTCP(${int});",
//     QPAIR_STR("int", "unsigned")
// );
// DEFINE_TASK(TASK_TASKRECOG_SELECT_TCP, taskPushBack_taskRecog_selectTCP, "taskPushBack_taskRecog_selectTCP();");
// DEFINE_TASK(TASK_TASKRECOG_TOOLCHANGINGATTACH_JSMOVE, taskPushBack_taskRecog_ToolChangingAttach_jsMove, "taskPushBack_taskRecog_ToolChangingAttach_jsMove();");
// DEFINE_TASK(TASK_TASKRECOG_TOOLCHANGINGDETACH_JSMOVE, taskPushBack_taskRecog_ToolChangingDetach_jsMove, "taskPushBack_taskRecog_ToolChangingDetach_jsMove();");
// DEFINE_TASK(TASK_TASKRECOG_CURRENTTOOLDETACHING_JSMOVE, taskPushBack_taskRecog_CurrentToolDetaching_jsMove, "taskPushBack_taskRecog_CurrentToolDetaching_jsMove();");
// DEFINE_TASK(TASK_TASKRECOG_TOOLCHANGINGDEFAULTATTACH_JSMOVE, taskPushBack_taskRecog_ToolChangingDefaultAttach_jsMove, "taskPushBack_taskRecog_ToolChangingDefaultAttach_jsMove();");
// DEFINE_TASK(TASK_TASKRECOG_TOOLCHANGINGDEFAULTDETACH_JSMOVE, taskPushBack_taskRecog_ToolChangingDefaultDetach_jsMove, "taskPushBack_taskRecog_ToolChangingDefaultDetach_jsMove();");
// DEFINE_TASK(TASK_TASKRECOG_TOOLCHANGINGGOAL_CSMOVE, taskPushBack_taskRecog_ToolChangingGoal_csMove, "taskPushBack_taskRecog_ToolChangingGoal_csMove(${is_tool_attaching}, ${pose});",
//     QPAIR_STR("is_tool_attaching", "bool"),
//     QPAIR_STR("pose", "CsDouble")
// );
// DEFINE_TASK(TASK_TASKRECOG_CURRENTTIPCHANGINGDETACH_JSMOVE, taskPushBack_taskRecog_CurrentTipChangingDetach_jsMove, "taskPushBack_taskRecog_CurrentTipChangingDetach_jsMove();");
// DEFINE_TASK(TASK_TASKRECOG_TIPCHANGINGATTACH_JSMOVE, taskPushBack_taskRecog_TipChangingAttach_jsMove, "taskPushBack_taskRecog_TipChangingAttach_jsMove();");
// DEFINE_TASK(TASK_TASKRECOG_TIPCHANGINGDETACH_JSMOVE, taskPushBack_taskRecog_TipChangingDetach_jsMove, "taskPushBack_taskRecog_TipChangingDetach_jsMove();");
// DEFINE_TASK(TASK_TASKRECOG_TIPCHANGINGDEFAULTATTACH_JSMOVE, taskPushBack_taskRecog_TipChangingDefaultAttach_jsMove, "taskPushBack_taskRecog_TipChangingDefaultAttach_jsMove();");
// DEFINE_TASK(TASK_TASKRECOG_TIPCHANGINGDEFAULTDETACH_JSMOVE, taskPushBack_taskRecog_TipChangingDefaultDetach_jsMove, "taskPushBack_taskRecog_TipChangingDefaultDetach_jsMove();");
// DEFINE_TASK(TASK_TASKRECOG_TIPCHANGINGSETTCP, taskPushBack_taskRecog_TipChangingSetTCP, "taskPushBack_taskRecog_TipChangingSetTCP();");
// DEFINE_TASK(TASK_TASKRECOG_TIPCHANGINGSETCURRENTTIPINDEX, taskPushBack_taskRecog_TipChangingSetCurrentTipIndex, "taskPushBack_taskRecog_TipChangingSetCurrentTipIndex(${current_tip_idx});",
//     QPAIR_STR("current_tip_idx", "size_t")
// );
// DEFINE_TASK(TASK_COUNT_DETACHING_POSE, taskPushBack_countDetachingPose, "taskPushBack_countDetachingPose();");
// DEFINE_TASK(TASK_MOVE_TO_TAG, taskPushBack_moveToTag, "taskPushBack_moveToTag(${int});",
//     QPAIR_STR("int", "unsigned")
// );
// DEFINE_TASK(TASK_APRIL_TAG_DETECTION, taskPushBack_aprilTagDetection, "taskPushBack_aprilTagDetection(${tag_num});",
//     QPAIR_STR("tag_num", "uint")
// );
// DEFINE_TASK(TASK_ROBOT_ENABLE, taskPushBack_robotEnable, "taskPushBack_robotEnable();");
// DEFINE_TASK(TASK_ROBOT_DISABLE, taskPushBack_robotDisable, "taskPushBack_robotDisable();");
// DEFINE_TASK(TASK_SET_FLAG_ROBOT_ENABLED_IS_ABLE_MODE, taskPushBack_setFlagRobotEnableDisableMode, "taskPushBack_setFlagRobotEnableDisableMode(${is_flag_on});",
//     QPAIR_STR("is_flag_on", "bool")
// );
// DEFINE_TASK(TASK_START_TAG_RECOG, taskPushBack_startTagRecog, "taskPushBack_startTagRecog();");
// DEFINE_TASK(TASK_END_SAVE_TAG_RECOG, taskPushBack_endSaveTagRecog, "taskPushBack_endSaveTagRecog();");
// DEFINE_TASK(TASK_SET_CURRENT_TAG, taskPushBack_setCurrentTag, "taskPushBack_setCurrentTag(${tag_name});",
//     QPAIR_STR("tag_name", "string")
// );
// DEFINE_TASK(TASK_SET_CURRENT_TAG_FROM_JSON, taskPushBack_setCurrentTagFromJSON, "taskPushBack_setCurrentTagFromJSON(${tag_name});",
//     QPAIR_STR("tag_name", "string")
// );
// DEFINE_TASK(TASK_CSMOVE_TAG_BASED, taskPushBack_csMove_tagBased, "taskPushBack_csMove_tagBased(${pose_tag_based});",
//     QPAIR_STR("pose_tag_based", "CsDouble")
// );
// DEFINE_TASK(TASK_CSMOVE_TO_TAG, taskPushBack_csMove_toTag, "taskPushBack_csMove_toTag(${distance_mm});",
//     QPAIR_STR("distance_mm", "uint")
// );
// DEFINE_TASK(TASK_CSMOVE_TO_TAG_FOR_APPROACHING, taskPushBack_csMove_toTag_forApproaching, "taskPushBack_csMove_toTag_forApproaching(${tag_name}, ${distance_mm}, ${is_second_tag_approach});",
//     QPAIR_STR("tag_name", "string"),
//     QPAIR_STR("distance_mm", "uint"),
//     QPAIR_STR("is_second_tag_approach", "bool")
// );
// DEFINE_TASK(TASK_CSMOVE_TO_TAG_FOR_CONVERGING, taskPushBack_csMove_toTag_forConverging, "taskPushBack_csMove_toTag_forConverging(${tag_name}, ${distance_mm});",
//     QPAIR_STR("tag_name", "string"),
//     QPAIR_STR("distance_mm", "uint")
// );
// DEFINE_TASK(TASK_CSMOVE_TAG_SEARCH, taskPushBack_csMove_tagSearch, "taskPushBack_csMove_tagSearch(${tag_name});",
//     QPAIR_STR("tag_name", "string")
// );
// DEFINE_TASK(TASK_SET_CAMERA_AUTO_FOCUS, taskPushBack_setCameraAutoFocus, "taskPushBack_setCameraAutoFocus(${is_auto_focus_on});",
//     QPAIR_STR("is_auto_focus_on", "bool")
// );
// DEFINE_TASK(TASK_TASK_PYBIND_CSMOVE, taskPushBack_taskPybindCsMove, "taskPushBack_taskPybindCsMove(${func_name});",
//     QPAIR_STR("func_name", "string")
// );



    // 추가 태스크 정의...

    return definitions;
}

void TaskPlannerWidget::registerTaskFunctions() {
    for (const auto& definition : taskDefinitions) {
        if (definition.luaRegistration) {
            definition.luaRegistration(lua_, definition.luaFunctionName);
        }
        ui.taskTypeSelector->addItem(definition.taskType);
    }
}

TaskPlannerWidget::TaskPlannerWidget(QNode* qnode, QWidget* parent) : qnode_(qnode), QWidget(parent), compositeTask("Composite Task") {

    ui.setupUi(this);  // .ui와 위젯 바인딩

    lua_.open_libraries(sol::lib::base, sol::lib::math);
    taskDefinitions = createTaskDefinitions(*qnode_->task_planner_, targetModuleTask_);

    // initUI();
    registerTaskFunctions();

    // UI 세팅
    ui.taskTable->setColumnCount(1);
    // ui.taskTable->setHorizontalHeaderLabels(QStringList() << "Module Tasks");
    ui.taskTable->setColumnWidth(0, 180);
    ui.scriptEditor_lua->setEnabled(true); // 활성화


    connectSignals();  // 시그널-슬롯 연결
    updateParamInputs();  // 초기화
}

TaskPlannerWidget::~TaskPlannerWidget() {

}

void TaskPlannerWidget::connectSignals() {
    connect(ui.addTaskBtn, &QPushButton::clicked, this, &TaskPlannerWidget::addTask);
    connect(ui.removeTaskBtn, &QPushButton::clicked, this, &TaskPlannerWidget::removeTask);
    connect(ui.taskTable, &QTableWidget::currentCellChanged, this, &TaskPlannerWidget::onTaskSelectionChanged);
    connect(ui.scriptEditor_lua, &QTextEdit::textChanged, this, [this]() {
        int row = ui.taskTable->currentRow();
        if (row >= 0 && row < tasks.size()) {
            tasks[row]->scriptText = ui.scriptEditor_lua->toPlainText();
        }
    });

    connect(ui.addModuleTaskBtn, &QPushButton::clicked, this, &TaskPlannerWidget::addModuleTaskToCompositeTask);
    connect(ui.removeModuleTaskBtn, &QPushButton::clicked, this, &TaskPlannerWidget::removeModuleTaskFromCompositeTask);
    connect(ui.addUnitTaskBtn, &QPushButton::clicked, this, &TaskPlannerWidget::addUnitTask);
    connect(ui.saveAllTasksBtn, &QPushButton::clicked, this, &TaskPlannerWidget::saveAllTasks);
    connect(ui.loadAllTasksBtn, &QPushButton::clicked, this, &TaskPlannerWidget::loadAllTasks);
    connect(ui.runLuaScriptBtn, &QPushButton::clicked, this, &TaskPlannerWidget::executeScript);
    connect(ui.saveCompositeTaskBtn, &QPushButton::clicked, this, &TaskPlannerWidget::saveCompositeTask);
    connect(ui.addConditionBtn, &QPushButton::clicked, this, &TaskPlannerWidget::addConditionalBlockUI);
    connect(ui.addLoopBtn, &QPushButton::clicked, this, &TaskPlannerWidget::addLoopBlockUI);
    connect(ui.closeBlockBtn, &QPushButton::clicked, this, &TaskPlannerWidget::closeBlock);

    ui.runButton->setStyleSheet("background-color: red; color: white;");
    connect(ui.runButton, &QPushButton::clicked, this, &TaskPlannerWidget::onRunButtonClicked);
    ui.stopButton->setStyleSheet("background-color: gray; color: white;");
    connect(ui.stopButton, &QPushButton::clicked, this, &TaskPlannerWidget::onStopButtonClicked);


    ui.repeatSpinBox->setToolTip("Set the number of repetitions (0 for infinite)");


    connect(ui.taskTypeSelector, &QComboBox::currentTextChanged, this, &TaskPlannerWidget::updateParamInputs);

}

void TaskPlannerWidget::initUI() {
    // mainLayout = new QVBoxLayout(this);

    // // Top Layout
    // topLayout = new QHBoxLayout();

    // QVBoxLayout* compositeTaskListLayout = new QVBoxLayout();
    // // Arrow Buttons Layout
    // QVBoxLayout* arrowButtonsLayout = new QVBoxLayout();

    // addModuleTaskBtn = new QPushButton("<-");
    // connect(addModuleTaskBtn, &QPushButton::clicked, this, &TaskPlannerWidget::addModuleTaskToCompositeTask);
    // arrowButtonsLayout->addWidget(addModuleTaskBtn);

    // removeModuleTaskBtn = new QPushButton("->");
    // connect(removeModuleTaskBtn, &QPushButton::clicked, this, &TaskPlannerWidget::removeModuleTaskFromCompositeTask);
    // arrowButtonsLayout->addWidget(removeModuleTaskBtn);

    // // Composite Task List
    // ui.compositeTaskList-> = new QListWidget();
    // compositeTaskListLayout->addWidget(new QLabel("Composite Task (json file)"));
    // compositeTaskListLayout->addWidget(ui.compositeTaskList->);
    // topLayout->addLayout(compositeTaskListLayout);
    // topLayout->addLayout(arrowButtonsLayout);

    // // Task Table
    // taskTable = new QTableWidget(0, 1);
    // ui.taskTable->setHorizontalHeaderLabels({"Module Tasks"});
    // ui.taskTable->setColumnWidth(0, 180);

    // connect(taskTable, &QTableWidget::currentCellChanged, this, &TaskPlannerWidget::onTaskSelectionChanged);
    // topLayout->addWidget(taskTable);

    // // Lua Script Editor with Title
    // QVBoxLayout* luaEditorLayout = new QVBoxLayout();
    // QLabel* luaEditorTitle = new QLabel("Lua Script Editor");
    // luaEditorTitle->setAlignment(Qt::AlignLeft);
    // luaEditorLayout->addWidget(luaEditorTitle);

    // scriptEditor_lua = new QTextEdit();
    // luaEditorLayout->addWidget(scriptEditor_lua);
    // ui.scriptEditor_lua->setEnabled(false); // 초기화 시 비활성화
    // connect(scriptEditor_lua, &QTextEdit::textChanged, this, [this]() {
    //     int currentRow = ui.taskTable->currentRow();
    //     if (currentRow >= 0 && currentRow < tasks.size()) {
    //         tasks[currentRow]->scriptText = ui.scriptEditor_lua->toPlainText();
    //     }
    // });
    // topLayout->addLayout(luaEditorLayout);

    // topLayout->setStretch(0, 4);  // compositeTaskListLayout이 4배 비율로 차지
    // topLayout->setStretch(1, 1);  // arrowButtonsLayout이 1배 비율로 차지
    // topLayout->setStretch(2, 3);  // taskTable이 3배 비율로 차지
    // topLayout->setStretch(3, 5);  // scriptEditor가 5배 비율로 차지
    // mainLayout->addLayout(topLayout);

    // // Task Type Selector and Parameter Inputs
    // paramInputLayout = new QVBoxLayout();
    // taskTypeSelector = new QComboBox();
    // // ui.taskTypeSelector->addItems(taskType.keys());
    // connect(taskTypeSelector, &QComboBox::currentTextChanged, this, &TaskPlannerWidget::updateParamInputs);
    // paramInputLayout->addWidget(taskTypeSelector);

    // paramInputWidget = new QWidget();
    // paramInputLayout->addWidget(paramInputWidget);

    // mainLayout->addLayout(paramInputLayout);

    // // Buttons
    // QHBoxLayout* taskButtonLayout = new QHBoxLayout();
    // addTaskBtn = new QPushButton("Add Module Task");
    // connect(addTaskBtn, &QPushButton::clicked, this, &TaskPlannerWidget::addTask);
    // taskButtonLayout->addWidget(addTaskBtn);

    // removeTaskBtn = new QPushButton("Remove Task");
    // connect(removeTaskBtn, &QPushButton::clicked, this, &TaskPlannerWidget::removeTask);
    // taskButtonLayout->addWidget(removeTaskBtn);

    // addUnitTaskBtn = new QPushButton("Add Unit Task");
    // connect(addUnitTaskBtn, &QPushButton::clicked, this, &TaskPlannerWidget::addUnitTask);
    // taskButtonLayout->addWidget(addUnitTaskBtn);

    // mainLayout->addLayout(taskButtonLayout);

    // // Save All Tasks Button
    // saveAllTasksBtn = new QPushButton("Save Module Tasks(*.task)");
    // connect(saveAllTasksBtn, &QPushButton::clicked, this, &TaskPlannerWidget::saveAllTasks);
    // mainLayout->addWidget(saveAllTasksBtn);

    // // Load All Tasks Button
    // loadAllTasksBtn = new QPushButton("Load Module Tasks(*.task)");
    // connect(loadAllTasksBtn, &QPushButton::clicked, this, &TaskPlannerWidget::loadAllTasks);
    // mainLayout->addWidget(loadAllTasksBtn);

    // // Lua script run button
    // runLuaScriptBtn = new QPushButton("Apply Current Lua Script");
    // connect(runLuaScriptBtn, &QPushButton::clicked, this, &TaskPlannerWidget::executeScript);
    // mainLayout->addWidget(runLuaScriptBtn);

    // // Save Composite Task Button
    // saveCompositeTaskBtn = new QPushButton("Apply Composite Task");
    // connect(saveCompositeTaskBtn, &QPushButton::clicked, this, &TaskPlannerWidget::saveCompositeTask);
    // mainLayout->addWidget(saveCompositeTaskBtn);


    // connect(taskTable, &QTableWidget::itemChanged, this, &TaskPlannerWidget::onModuleTaskNameChanged);


    // initConditionalAndLoopUI();

    // // Control Buttons Layout (Run, Stop, Progress Bar)
    // QHBoxLayout* controlLayout = new QHBoxLayout();

    // runButton = new QPushButton("Run");
    // runButton->setStyleSheet("background-color: red; color: white;");
    // connect(runButton, &QPushButton::clicked, this, &TaskPlannerWidget::onRunButtonClicked);
    // controlLayout->addWidget(runButton);

    // // 반복 횟수 설정 SpinBox 추가
    // repeatSpinBox = new QSpinBox();
    // ui.repeatSpinBox->setRange(0, 1000);  // 0 = 무한 반복, 1000 = 최대 반복 횟수
    // ui.repeatSpinBox->setValue(1);        // 기본값: 1회 반복
    // ui.repeatSpinBox->setPrefix("Repeat(0 for infinite): ");
    // ui.repeatSpinBox->setToolTip("Set the number of repetitions (0 for infinite)");
    // controlLayout->addWidget(repeatSpinBox);

    // stopButton = new QPushButton("Stop");
    // stopButton->setStyleSheet("background-color: gray; color: white;");
    // connect(stopButton, &QPushButton::clicked, this, &TaskPlannerWidget::onStopButtonClicked);
    // controlLayout->addWidget(stopButton);

    // progressBar = new QProgressBar();
    // ui.progressBar->setRange(0, 100);
    // ui.progressBar->setValue(0);
    // controlLayout->addWidget(progressBar);


    // connect(qnode_, &QNode::taskFinLogger, this, &TaskPlannerWidget::updateProgressBar);
    // connect(qnode_, &QNode::taskEndLogger, this, &TaskPlannerWidget::onTaskExecutionFinished);

    // mainLayout->addLayout(controlLayout);

}

void TaskPlannerWidget::addModuleTaskToCompositeTask() {
    int currentRow = ui.taskTable->currentRow();
    if (currentRow == -1) {
        currentRow = ui.taskTable->rowCount(); // 마지막 위치로 설정
    }

    if (currentRow >= 0 && currentRow < tasks.size()) {
        std::shared_ptr<Task> moduleTask = tasks[currentRow];  // shared_ptr로 접근
        compositeTask.moduleTasks.append(moduleTask);          // shared_ptr 추가

        // 현재 선택된 위치를 확인하고, 유효하지 않으면 목록의 끝에 추가
        int compositeRow = ui.compositeTaskList->currentRow();
        if (compositeRow == -1) {
            compositeRow = ui.compositeTaskList->count(); // 마지막 위치로 설정
        }

        // UI 업데이트
        ui.compositeTaskList->insertItem(compositeRow, moduleTask->name);
    }
}

void TaskPlannerWidget::removeModuleTaskFromCompositeTask() {
    int currentRow = ui.compositeTaskList->currentRow();
    if (currentRow >= 0 && currentRow < compositeTask.moduleTasks.size()) {
        compositeTask.moduleTasks.removeAt(currentRow);  // 포인터 제거
        // UI 업데이트
        delete ui.compositeTaskList->takeItem(currentRow);
    }
}

void TaskPlannerWidget::onModuleTaskNameChanged(QTableWidgetItem* item) {
    if (!item) return;

    int row = item->row();
    if (row >= 0 && row < tasks.size()) {
        QString newName = item->text();

        // 중복 검사
        bool isDuplicate = std::any_of(tasks.begin(), tasks.end(), [&](const std::shared_ptr<Task>& task) {
            return task->name == newName && task != tasks[row];
        });

        if (isDuplicate) {
            QMessageBox::warning(this, "Duplicate Name", "Task name must be unique.");
            item->setText(tasks[row]->name);  // 이전 이름으로 복원
            return;
        }

        // 이름 업데이트
        std::shared_ptr<Task> moduleTask = tasks[row];
        moduleTask->name = newName;

        // Composite Task에 포함된 동일한 Task의 이름 업데이트
        for (int i = 0; i < compositeTask.moduleTasks.size(); ++i) {
            if (compositeTask.moduleTasks[i] == moduleTask) {  // shared_ptr 비교로 동일 Task 확인
                ui.compositeTaskList->item(i)->setText(moduleTask->name);  // 이름 업데이트
            }
        }
    }
}


void TaskPlannerWidget::saveCompositeTask() {

    if (compositeTask.moduleTasks.isEmpty()) {
        QMessageBox::warning(this, "Error", "Composite Task is empty!");
        return;
    }
        for (const auto& moduleTask : compositeTask.moduleTasks) {
            if (!moduleTask->scriptText.isEmpty()) {
                // Lua 스크립트 실행
                try {
                    // runParsedScript 실행, 예외 발생 시 catch로 처리
                    runParsedScript(moduleTask->scriptText);

                } catch (const sol::error& e) {
                    QMessageBox::critical(this, moduleTask->name + "Lua Execution Error", QString("%1").arg(e.what()));
                    return;
                } catch (const std::exception& e) {
                    QMessageBox::critical(this, moduleTask->name + "Execution Error", QString("%1").arg(e.what()));
                    return;
                } catch (...) {
                    QMessageBox::critical(this, moduleTask->name + "Unknown Error", "An unknown error occurred while executing the script.");
                    return;
                }
            } else {
                QMessageBox::warning(this, "Error", "moudule task " + moduleTask->name + " is empty!");
                return;
            }
        }
        QMessageBox::information(this, "Success", "Composite Task executed successfully!");
}


// QString TaskPlannerWidget::generateUniqueTaskName(const QString& baseName) {
//     QString taskName = baseName;
//     int counter = 1;

//     // 중복이 없는 이름이 나올 때까지 반복
//     while (std::any_of(tasks.begin(), tasks.end(), [&](const std::shared_ptr<Task>& task) {
//                return task->name == taskName;
//            }) || ui.taskTable->findItems(taskName, Qt::MatchExactly).size() > 0) {
//         taskName = QString("%1_%2").arg(baseName).arg(counter++);
//     }

//     return taskName;
// }

QString TaskPlannerWidget::generateUniqueTaskName(const QString& baseName) {
    int counter = 1;
    QString taskName;

    do {
        taskName = QString("%1_%2").arg(baseName).arg(counter++);
    } while (std::any_of(tasks.begin(), tasks.end(), [&](const std::shared_ptr<Task>& task) {
        return task->name == taskName;
    }) || ui.taskTable->findItems(taskName, Qt::MatchExactly).size() > 0);

    return taskName;
}


// moudule Task 추가
void TaskPlannerWidget::addTask() {
    ROS_LOG_WARN("[%s]", __func__);
    int row = ui.taskTable->rowCount();
    ui.taskTable->insertRow(row);
    QString taskName = generateUniqueTaskName("module_task");
    ui.taskTable->setItem(row, 0, new QTableWidgetItem(taskName));
    tasks.push_back(std::make_shared<Task>(taskName));

    ui.taskTable->setCurrentCell(row, 0);
    ui.scriptEditor_lua->clear();
}

// moudule Task 삭제
void TaskPlannerWidget::removeTask() {
    int taskRow = ui.taskTable->currentRow();
    if (taskRow >= 0) {
        QString taskName = ui.taskTable->item(taskRow, 0)->text();

        // Composite Task에서 이름이 같은 Task를 삭제할지 묻는 확인 창 띄우기
        QMessageBox::StandardButton resBtn = QMessageBox::question(
            this, "Remove Confirmation",
            QString("Are you sure you want to remove the selected Module Task '%1'? "
                    "This will also remove it from any Composite Tasks where it is used.")
                .arg(taskName),
            QMessageBox::No | QMessageBox::Yes, QMessageBox::Yes);

        if (resBtn == QMessageBox::Yes) {
            // Composite Task에서 해당 Task 제거
            for (int i = 0; i < compositeTask.moduleTasks.size(); ++i) {
                if (compositeTask.moduleTasks[i]->name == taskName) {
                    compositeTask.moduleTasks.removeAt(i);
                    delete ui.compositeTaskList->takeItem(i); // UI에서 항목 제거
                    --i; // 삭제 후 인덱스 조정
                }
            }

            // Module Task 삭제
            ui.taskTable->removeRow(taskRow);
            tasks.erase(tasks.begin() + taskRow); // tasks 벡터에서 삭제

            // Lua Script Editor 초기화
            ui.scriptEditor_lua->clear();
            QMessageBox::information(this, "Task Removed",
                                     QString("Module Task '%1' and its references in Composite Task have been removed.")
                                         .arg(taskName));
        }
    } else {
        QMessageBox::warning(this, "No Task Selected", "Please select a Module Task to remove.");
    }
}

void TaskPlannerWidget::saveAllTasks() {
    QString fileName = QFileDialog::getSaveFileName(this, "Save All Tasks to File", "", "Task Files (*.task)");
    if (!fileName.isEmpty()) {
        int currentRow = ui.taskTable->currentRow();
        if (currentRow >= 0 && currentRow < tasks.size()) {
            tasks[currentRow]->scriptText = ui.scriptEditor_lua->toPlainText();
        }

        QFile file(fileName);
        if (file.open(QIODevice::WriteOnly)) {
            QTextStream out(&file);

            // Module Task 데이터 저장
            out << "All Module Tasks:\n";
            for (const auto& task : tasks) {
                out << "Module Task Name: " << task->name << "\n";
                out << task->scriptText << "\n";
                out << "End Script\n";
            }

            // Composite Task 데이터 저장 (Module Task 순서 정보만 포함)
            out << "Composite Task Order:\n";
            for (const auto& moduleTask : compositeTask.moduleTasks) {
                out << moduleTask->name << "\n";
            }
            out << "End Composite Task\n";

            file.close();
            QMessageBox::information(this, "Success", "All tasks and Composite Task order saved successfully!");
        }
    }
}

void TaskPlannerWidget::loadAllTasks() {
    QString fileName = QFileDialog::getOpenFileName(this, "Load All Tasks from File", "", "Task Files (*.task)");
    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) return;

    QTextStream in(&file);
    QHash<QString, std::shared_ptr<Task>> moduleTaskCache;
    bool loadingCompositeOrder = false;
    QList<std::shared_ptr<Task>> newCompositeTasks;
    QList<QString> newCompositeTaskNames;
    QHash<QString, QString> nameChangeMap; // originalTaskName과 최종 taskName을 저장


    QMessageBox::StandardButton resBtn = QMessageBox::question(
        this, "Load Confirmation",
        "Some tasks with the same names may exist. Do you want to overwrite them?",
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No
    );
    bool overwriteExisting = (resBtn == QMessageBox::Yes);
    while (!in.atEnd()) {
        QString line = in.readLine();
        if (line == "All Module Tasks:") {
            while (!(line = in.readLine()).isEmpty() && !line.startsWith("Composite Task Order:")) {
                if (line.startsWith("Module Task Name: ")) {
                    QString originalTaskName = line.mid(QString("Module Task Name: ").length());
                    QString taskName = originalTaskName;
                    QString scriptText;

                    while (!(line = in.readLine()).startsWith("End Script")) {
                        scriptText += line + "\n";
                    }

                    auto existingTaskIt = std::find_if(tasks.begin(), tasks.end(),
                                                       [&](const std::shared_ptr<Task>& task) { return task->name == taskName; });
                    std::shared_ptr<Task> loadedTask;

                    if (existingTaskIt != tasks.end()) {
                        if (overwriteExisting) {
                            // 기존 태스크가 존재할 때 스크립트 텍스트만 갱신
                            loadedTask = *existingTaskIt;
                            loadedTask->scriptText = scriptText;
                        } else {
                            // 이름 충돌 시 새로운 이름으로 추가
                            int counter = 1;
                            do {
                                taskName = QString("%1_%2_new").arg(originalTaskName).arg(counter++);
                            } while (std::any_of(tasks.begin(), tasks.end(),
                                                 [&](const std::shared_ptr<Task>& task) { return task->name == taskName; }));
                            loadedTask = std::make_shared<Task>(taskName);
                            loadedTask->scriptText = scriptText;
                            tasks.push_back(loadedTask);

                            // 새로운 이름으로 매핑 추가
                            nameChangeMap.insert(originalTaskName, taskName);
                            // 새로운 모듈 태스크를 테이블에 추가
                            int row = ui.taskTable->rowCount();
                            ui.taskTable->insertRow(row);
                            ui.taskTable->setItem(row, 0, new QTableWidgetItem(taskName));
                        }

                    } else {
                        // 새 태스크인 경우 추가
                        loadedTask = std::make_shared<Task>(taskName);
                        loadedTask->scriptText = scriptText;

                        tasks.push_back(loadedTask);

                        // 새로운 모듈 태스크를 테이블에 추가
                        int row = ui.taskTable->rowCount();
                        ui.taskTable->insertRow(row);
                        ui.taskTable->setItem(row, 0, new QTableWidgetItem(taskName));
                    }

                    moduleTaskCache[taskName] = loadedTask;
                }
            }
        }

        if (line == "Composite Task Order:") {
            loadingCompositeOrder = true;
            continue;
        }
        if (line == "End Composite Task") {
            loadingCompositeOrder = false;
            continue;
        }
        if (loadingCompositeOrder && !line.isEmpty()) {
            QString moduleTaskName = line.trimmed();
            newCompositeTaskNames.append(moduleTaskName);
        }
    }
    file.close();

    // 컴포지트 테스크 업데이트
    for (const auto& taskName : newCompositeTaskNames) {
        QString finalTaskName = taskName;

        // 만약 이름이 변경되었다면 nameChangeMap에서 새로운 이름을 가져옵니다
        if (nameChangeMap.contains(taskName)) {
            finalTaskName = nameChangeMap[taskName];
        }

        // 최종 이름을 사용하여 모듈 테스크를 추가
        if (moduleTaskCache.contains(finalTaskName)) {
            std::shared_ptr<Task> moduleTask = moduleTaskCache[finalTaskName];
            compositeTask.moduleTasks.append(moduleTask);
            ui.compositeTaskList->addItem(moduleTask->name);
        }
    }

    QMessageBox::information(this, "Success", "All tasks and Composite Task order loaded successfully!");
}

void TaskPlannerWidget::onTaskSelectionChanged(int currentRow, int currentColumn, int previousRow, int previousColumn) {
    // 이전 Task의 스크립트를 저장
    if (previousRow >= 0 && previousRow < tasks.size()) {
        tasks[previousRow]->scriptText = ui.scriptEditor_lua->toPlainText();
    }

    // 현재 Task의 스크립트를 로드
    if (currentRow >= 0 && currentRow < tasks.size()) {
        std::shared_ptr<Task> currentTask = tasks[currentRow];
        ui.scriptEditor_lua->setPlainText(currentTask->scriptText);
        ui.scriptEditor_lua->setEnabled(true); // 활성화
    } else {
        ui.scriptEditor_lua->clear();
        ui.scriptEditor_lua->setEnabled(false); // 비활성화
    }
}

void TaskPlannerWidget::addUnitTask() {
    int taskRow = ui.taskTable->currentRow();
    ROS_LOG_WARN("taskRow: %i", taskRow);
    if (taskRow == -1) {
        QMessageBox::warning(this, tr("Graphy Robot Task Mode"),
                             tr("No module task!"));
        return;
    } else {
        QString taskType = ui.taskTypeSelector->currentText();
        QString luaScriptTemplate;
        QList<QPair<QString, QString>> paramNames;

        // taskDefinitions에서 선택된 taskType의 정보를 가져옵니다.
        for (const auto& definition : taskDefinitions) {
            if (definition.taskType == taskType) {
                luaScriptTemplate = definition.luaScriptTemplate;
                paramNames = definition.parameters;
                break;
            }
        }

        if (luaScriptTemplate.isEmpty()) {
            QMessageBox::warning(this, "Task Error", "Invalid task type selected.");
            return;
        }

        QStringList paramValues;

        // UI 입력 필드에서 파라미터 값들을 가져옵니다.
        for (const auto& param : paramNames) {
            QString paramName = param.first;
            QString paramType = param.second;
            QWidget* widget = paramInputFields[paramName];

            if (paramType == "int") {
                QSpinBox* spinBox = qobject_cast<QSpinBox*>(widget);
                paramValues.append(QString::number(spinBox->value()));
            } else if (paramType == "double") {
                QDoubleSpinBox* doubleSpinBox = qobject_cast<QDoubleSpinBox*>(widget);
                paramValues.append(QString::number(doubleSpinBox->value()));
            } else if (paramType == "bool") {
                QCheckBox* checkBox = qobject_cast<QCheckBox*>(widget);
                paramValues.append(checkBox->isChecked() ? "true" : "false");
            } else if (paramType == "CsDouble" || paramType == "JsDouble") {
                QList<QDoubleSpinBox*> spinBoxes = widget->property("spinBoxes").value<QList<QDoubleSpinBox*>>();
                QStringList arrayValues;
                for (QDoubleSpinBox* spinBox : spinBoxes) {
                    arrayValues.append(QString::number(spinBox->value()));
                }
                paramValues.append("{" + arrayValues.join(", ") + "}");
            } else {
                QLineEdit* lineEdit = qobject_cast<QLineEdit*>(widget);
                paramValues.append('"' + lineEdit->text() + '"');
            }
        }

        // luaScriptTemplate에서 변수들을 실제 값으로 치환
        for (int i = 0; i < paramNames.size(); ++i) {
            QString placeholder = QString("${%1}").arg(paramNames[i].first);
            luaScriptTemplate.replace(placeholder, paramValues[i]);
        }

        // Lua 스크립트 에디터에 반영
        ui.scriptEditor_lua->insertPlainText(luaScriptTemplate + "\n");
    }
}


void TaskPlannerWidget::updateParamInputs() {
    QString taskType = ui.taskTypeSelector->currentText();
    QList<QPair<QString, QString>> paramNames;

    std::string task_type_str = taskType.toStdString();
    ROS_LOG_WARN("taskType: %s", task_type_str.c_str());

    // taskDefinitions 벡터에서 taskType에 해당하는 TaskDefinition을 찾습니다.
    for (const auto& definition : taskDefinitions) {
        if (definition.taskType == taskType) {
            paramNames = definition.parameters; // TaskDefinition의 paramNames를 가져옵니다.
            break;
        }
    }

    // 기존 파라미터 입력 위젯 제거
    QLayout* oldLayout = ui.paramInputWidget->layout();
    if (oldLayout != nullptr) {
        QLayoutItem* item;
        while ((item = oldLayout->takeAt(0)) != nullptr) {
            if (item->widget()) {
                delete item->widget();
            }
            delete item;
        }
        delete oldLayout;
    }
    paramInputFields.clear();

    QFormLayout* formLayout = new QFormLayout(ui.paramInputWidget);
    ui.paramInputWidget->setLayout(formLayout);

    for (const auto& param : paramNames) {
        QLabel* label = new QLabel(param.first);
        if (param.second == "int") {
            QSpinBox* spinBox = new QSpinBox();
            formLayout->addRow(label, spinBox);
            paramInputFields[param.first] = spinBox;
        } else if (param.second == "double") {
            QDoubleSpinBox* doubleSpinBox = new QDoubleSpinBox();
            if (taskType == "ROBOT_MOVE_JS") {
                doubleSpinBox->setRange(0.0, 90.0); // acc, vel
                doubleSpinBox->setSingleStep(5.0);
                doubleSpinBox->setDecimals(2); // 소수점 2자리까지 허용
                doubleSpinBox->setValue(10.0);
            } else if(taskType == "ROBOT_MOVE_CS" || taskType == "ROBOT_MOVE_CS_TOOL_FRAME") {
                doubleSpinBox->setRange(0.0, 0.75); // acc, vel
                doubleSpinBox->setSingleStep(0.01);
                doubleSpinBox->setDecimals(3); // 소수점 5자리까지 허용
                doubleSpinBox->setValue(0.1);
            } else if(taskType == "ADD_DELAY_1MS") {
                doubleSpinBox->setRange(0.0, 100000000.0);
                doubleSpinBox->setSingleStep(200.0);
                doubleSpinBox->setDecimals(0);
                doubleSpinBox->setValue(1000);
            } else {
                doubleSpinBox->setRange(-1.0, 1.0);
                doubleSpinBox->setSingleStep(0.01);
                doubleSpinBox->setDecimals(5); // 소수점 5자리까지 허용
                doubleSpinBox->setValue(0.1);
            }
            formLayout->addRow(label, doubleSpinBox);
            paramInputFields[param.first] = doubleSpinBox;
        } else if (param.second == "bool") {
            QCheckBox* checkBox = new QCheckBox();
            formLayout->addRow(label, checkBox);
            paramInputFields[param.first] = checkBox;

            // ROBOT_MOVE_CS && 파라미터 이름이 isRelative일 때만 동작 설정
            if(param.first == "isRelative") {
                if (taskType == "ROBOT_MOVE_CS") {
                    // pose 파라미터 위젯 가져오기
                    QWidget* poseWidget = paramInputFields.contains("pose") ? paramInputFields["pose"] : nullptr;
                    if (poseWidget) {
                        connect(checkBox, &QCheckBox::toggled, this, [this, poseWidget](bool checked) {
                            if (checked) {
                                auto spinBoxes = poseWidget->property("spinBoxes").value<QList<QDoubleSpinBox*>>();
                                if (spinBoxes.size() != 6) {
                                    QMessageBox::warning(this, "오류", "스핀박스 개수가 6이 아닙니다.");
                                    return;
                                }
                                for (int i = 0; i < 6; ++i) {
                                    spinBoxes[i]->setValue(0.0);
                                }
                            } else {
                                auto spinBoxes = poseWidget->property("spinBoxes").value<QList<QDoubleSpinBox*>>();
                                if (spinBoxes.size() != 6) {
                                    QMessageBox::warning(this, "오류", "스핀박스 개수가 6이 아닙니다.");
                                    return;
                                }

                                std::vector<double> currentRobotAPose(6);
                                arr2Vec(qnode_->params_.meas.x, currentRobotAPose);
                                ROS_LOG_WARN("%0.4f, %0.4f, %0.4f, %0.1f, %0.1f, %0.1f", currentRobotAPose[0], currentRobotAPose[1], currentRobotAPose[2], currentRobotAPose[3], currentRobotAPose[4], currentRobotAPose[5]);

                                for (int i = 0; i < 6; ++i) {
                                    spinBoxes[i]->setValue(currentRobotAPose[i]);
                                }
                            }
                        });
                    }
                } else if (taskType == "ROBOT_MOVE_CS_TOOL_FRAME") {
                    // 무조건 체크 상태로 유지
                    checkBox->setChecked(true);
                    QWidget* poseWidget = paramInputFields.contains("pose") ? paramInputFields["pose"] : nullptr;
                    if (poseWidget) {
                        connect(checkBox, &QCheckBox::clicked, this, [this, checkBox, poseWidget](bool) {
                            checkBox->setChecked(true);
                            auto spinBoxes = poseWidget->property("spinBoxes").value<QList<QDoubleSpinBox*>>();
                            if (spinBoxes.size() != 6) {
                                QMessageBox::warning(this, "오류", "스핀박스 개수가 6이 아닙니다.");
                                return;
                            }
                            for (int i = 0; i < 6; ++i) {
                                spinBoxes[i]->setValue(0.0);
                            }
                        });
                    }
                }
            }


            if(param.first == "relative") {
                if (taskType == "ROBOT_MOVE_JS") {
                    // pose 파라미터 위젯 가져오기
                    QWidget* poseWidget = paramInputFields.contains("joint_position") ? paramInputFields["joint_position"] : nullptr;
                    if (poseWidget) {
                        connect(checkBox, &QCheckBox::toggled, this, [this, poseWidget](bool checked) {
                            if (checked) {
                                auto spinBoxes = poseWidget->property("spinBoxes").value<QList<QDoubleSpinBox*>>();
                                if (spinBoxes.size() != 6) {
                                    QMessageBox::warning(this, "오류", "스핀박스 개수가 6이 아닙니다.");
                                    return;
                                }
                                for (int i = 0; i < 6; ++i) {
                                    spinBoxes[i]->setValue(0.0);
                                }
                            } else {
                                auto spinBoxes = poseWidget->property("spinBoxes").value<QList<QDoubleSpinBox*>>();
                                if (spinBoxes.size() != 6) {
                                    QMessageBox::warning(this, "오류", "스핀박스 개수가 6이 아닙니다.");
                                    return;
                                }

                                std::vector<double> curQRobotA(6);
                                arr2Vec(qnode_->params_.meas.q, curQRobotA);
                                ROS_LOG_WARN("%0.1f, %0.1f, %0.1f, %0.1f, %0.1f, %0.1f", curQRobotA[0], curQRobotA[1], curQRobotA[2], curQRobotA[3], curQRobotA[4], curQRobotA[5]);

                                for (int i = 0; i < 6; ++i) {
                                    spinBoxes[i]->setValue(curQRobotA[i]);
                                }
                            }
                        });
                    }
                }

            }

        } else if (param.second == "CsDouble" || param.second == "JsDouble") {
            QWidget* widget = new QWidget();
            QHBoxLayout* hLayout = new QHBoxLayout(widget);
            QList<QDoubleSpinBox*> spinBoxes;
            for (int i = 0; i < 6; ++i) {
                QDoubleSpinBox* doubleSpinBox = new QDoubleSpinBox();

                if (taskType == "ROBOT_MOVE_JS") {
                    doubleSpinBox->setRange(-360.0, 360.0);
                    doubleSpinBox->setSingleStep(1.0);  // 한 스텝에 0.1만큼 증감
                    doubleSpinBox->setDecimals(2); // 소수점 2자리까지 허용
                } else if(taskType == "ROBOT_MOVE_CS" || taskType == "ROBOT_MOVE_CS_TOOL_FRAME" || taskType == "ROBOT_SET_TCP") {
                    if(i < 3) {
                        doubleSpinBox->setRange(-1.0, 1.0);
                        doubleSpinBox->setSingleStep(0.01);  // 한 스텝에 0.1만큼 증감
                        doubleSpinBox->setDecimals(5); // 소수점 5자리까지 허용
                    } else {
                        doubleSpinBox->setRange(-360.0, 360.0);
                        doubleSpinBox->setSingleStep(1.0);  // 한 스텝에 0.1만큼 증감
                        doubleSpinBox->setDecimals(2); // 소수점 2자리까지 허용
                    }
                } else {
                    doubleSpinBox->setRange(-1.0, 1.0);
                    doubleSpinBox->setDecimals(5); // 소수점 5자리까지 허용
                }

                hLayout->addWidget(doubleSpinBox);
                spinBoxes.append(doubleSpinBox);
            }
            formLayout->addRow(label, widget);
            // spinBoxes 리스트를 QVariant로 저장
            paramInputFields[param.first] = widget;
            widget->setProperty("spinBoxes", QVariant::fromValue(spinBoxes));



            // 버튼 생성 1
            if (param.first == "pose" && taskType == "ROBOT_MOVE_CS") {
                QPushButton* button = new QPushButton("현재 로봇 위치 적용 [*주의: isRelative 버튼이 해제된 상태에서만 사용]");
                formLayout->addRow(button);

                connect(button, &QPushButton::clicked, this, [this, widget]() {
                    auto spinBoxes = widget->property("spinBoxes").value<QList<QDoubleSpinBox*>>();
                    if (spinBoxes.size() != 6) {
                        QMessageBox::warning(this, "오류", "스핀박스 개수가 6이 아닙니다.");
                        return;
                    }

                    std::vector<double> currentRobotAPose(6);
                    arr2Vec(qnode_->params_.meas.x, currentRobotAPose);
                    ROS_LOG_WARN("%0.4f, %0.4f, %0.4f, %0.1f, %0.1f, %0.1f", currentRobotAPose[0], currentRobotAPose[1], currentRobotAPose[2], currentRobotAPose[3], currentRobotAPose[4], currentRobotAPose[5]);
                    for (int i = 0; i < 6; ++i) {
                        spinBoxes[i]->setValue(currentRobotAPose[i]);
                    }
                });
            }

            // 버튼 생성 2
            if (param.first == "joint_position" && taskType == "ROBOT_MOVE_JS") {
                QPushButton* button = new QPushButton("현재 로봇 위치 적용 [*주의: relative 버튼이 해제된 상태에서만 사용]");
                formLayout->addRow(button);

                connect(button, &QPushButton::clicked, this, [this, widget]() {
                    auto spinBoxes = widget->property("spinBoxes").value<QList<QDoubleSpinBox*>>();
                    if (spinBoxes.size() != 6) {
                        QMessageBox::warning(this, "오류", "스핀박스 개수가 6이 아닙니다.");
                        return;
                    }
                    std::vector<double> curQRobotA(6);
                    arr2Vec(qnode_->params_.meas.q, curQRobotA);
                    ROS_LOG_WARN("%0.1f, %0.1f, %0.1f, %0.1f, %0.1f, %0.1f", curQRobotA[0], curQRobotA[1], curQRobotA[2], curQRobotA[3], curQRobotA[4], curQRobotA[5]);

                    for (int i = 0; i < 6; ++i) {
                        spinBoxes[i]->setValue(curQRobotA[i]);
                    }
                });
            }

        } else {
            QLineEdit* lineEdit = new QLineEdit();


            if (taskType == "GRIPPER_MOVE_POS_CTRL") {

                if(param.first == "grp_cmd") {
                    lineEdit->setText("104");
                } else if(param.first == "position") {
                    lineEdit->setText("8500");
                } else if(param.first == "speed") {
                    lineEdit->setText("1000");
                }

            }


            formLayout->addRow(label, lineEdit);
            paramInputFields[param.first] = lineEdit;
        }

    }
}

void TaskPlannerWidget::initConditionalAndLoopUI() {
    QHBoxLayout* blockControlLayout = new QHBoxLayout();

    QPushButton* addConditionBtn = new QPushButton("조건부 실행 추가");
    connect(addConditionBtn, &QPushButton::clicked, this, &TaskPlannerWidget::addConditionalBlockUI);
    blockControlLayout->addWidget(addConditionBtn);

    QPushButton* addLoopBtn = new QPushButton("루프 추가");
    connect(addLoopBtn, &QPushButton::clicked, this, &TaskPlannerWidget::addLoopBlockUI);
    blockControlLayout->addWidget(addLoopBtn);

    QPushButton* closeBlockBtn = new QPushButton("블록 닫기");
    connect(closeBlockBtn, &QPushButton::clicked, this, &TaskPlannerWidget::closeBlock);
    blockControlLayout->addWidget(closeBlockBtn);

    mainLayout->addLayout(blockControlLayout);
}
void TaskPlannerWidget::addConditionalBlockUI() {
    bool ok;
    QString condition = QInputDialog::getText(this, "조건 입력", "조건식을 입력하세요:", QLineEdit::Normal, "", &ok);

    if (ok && !condition.isEmpty()) {
        QString selectedText = ui.scriptEditor_lua->textCursor().selectedText();

        // 선택된 텍스트를 조건부 블록으로 감싸기
        QString block = QString("if %1 then\n%2\nend\n").arg(condition, selectedText);
        ui.scriptEditor_lua->insertPlainText(block);
    } else if (ok) {
        QMessageBox::warning(this, "오류", "조건식을 입력해야 합니다.");
    }
}
void TaskPlannerWidget::addLoopBlockUI() {
    bool ok;
    QString loopCondition = QInputDialog::getText(this, "루프 조건 입력", "루프 조건식을 입력하세요 (예: i=1,10):", QLineEdit::Normal, "", &ok);

    if (ok && !loopCondition.isEmpty()) {
        QString selectedText = ui.scriptEditor_lua->textCursor().selectedText();

        // 선택된 텍스트를 루프 블록으로 감싸기
        QString block = QString("for %1 do\n%2\nend\n").arg(loopCondition, selectedText);
        ui.scriptEditor_lua->insertPlainText(block);
    } else if (ok) {
        QMessageBox::warning(this, "오류", "루프 조건을 입력해야 합니다.");
    }
}

void TaskPlannerWidget::closeBlock() {
    ui.scriptEditor_lua->insertPlainText("end\n");
}


void TaskPlannerWidget::runParsedScript(const QString& script) {
    // 예외는 호출자에게 다시 전달
    lua_.script(script.toStdString());
}

void TaskPlannerWidget::executeScript() {
    targetModuleTask_.clear();
    QString script = ui.scriptEditor_lua->toPlainText();
    if (script.isEmpty()) {
        QMessageBox::warning(this, "Error", "Script Editor is empty!");
        return;
    }
    try {
        // runParsedScript 실행, 예외 발생 시 catch로 처리
        runParsedScript(script);

        // // 성공적으로 실행되었을 경우에만 메시지 표시
        // QMessageBox::information(this, "Success", "Script executed successfully!");
        qDebug() << "Inserted size: " << targetModuleTask_.size() << "\n";

    } catch (const sol::error& e) {
        QMessageBox::critical(this, "Lua Execution Error", QString("%1").arg(e.what()));
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Execution Error", QString("%1").arg(e.what()));
    } catch (...) {
        QMessageBox::critical(this, "Unknown Error", "An unknown error occurred while executing the script.");
    }
}

void TaskPlannerWidget::onRunButtonClicked() {

    //// 1) 현재 Script를 작업화
    executeScript();

    if (targetModuleTask_.empty()) {
        QMessageBox::warning(this, tr("Graphy Robot Task Mode"),
                             tr("No task exists!"));
        return;
    }

    //// 2) 작업 수행
    int repeatCount = ui.repeatSpinBox->value();
    if (repeatCount == 0) {
        isRepeating = true;  // 무한 반복 설정
    } else {
        isRepeating = false;
        currentIteration = 0;
    }
    executeTaskCycle();
}
// 개별 태스크 실행 사이클 처리
void TaskPlannerWidget::executeTaskCycle() {
    if (currentIteration >= ui.repeatSpinBox->value() && !isRepeating) {
        // 반복 종료
        QMessageBox::information(this, "Execution Complete", "All tasks executed successfully!");
        ui.progressBar->setValue(100);
        return;
    } else {
        ui.progressBar->setValue(0);
    }
    qnode_->current_task_list_ = targetModuleTask_;
    qnode_->beforeTaskStart();
    qnode_->task_cycle_ = 0;

    // 현재 반복 횟수 업데이트
    currentIteration++;
}

void TaskPlannerWidget::onStopButtonClicked() {
    isRepeating = false;
    qnode_->is_task_mode_ = false;
    qnode_->stopRobot();
}

void TaskPlannerWidget::updateProgressBar() {
    static int completedTasks = 0; // 완료된 태스크 수를 저장
    int totalTasks = qnode_->current_task_list_.size();

    if (totalTasks > 0) {
        completedTasks++;
        int progress = (completedTasks * 100) / totalTasks;
        ui.progressBar->setValue(progress);
    }
}

void TaskPlannerWidget::onTaskExecutionFinished() {
    if (isRepeating || currentIteration < ui.repeatSpinBox->value()) {
        // 다음 반복 실행
        executeTaskCycle();
    } else {
        QMessageBox::information(this, "Execution Complete", "All tasks executed successfully!");
        ui.progressBar->setValue(100);
    }
}
} // namespace taskplan