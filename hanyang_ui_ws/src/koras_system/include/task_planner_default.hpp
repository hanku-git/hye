#ifndef TASK_PLANNER_DEFAULT_HPP
#define TASK_PLANNER_DEFAULT_HPP

#include <vector>
#include <string>
#include <array>
#include "ui_define.hpp"

#include <type_traits>
#include <tuple>
#include <utility>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <functional>
//#include <jsoncpp/json/json.h>
#include <rclcpp/rclcpp.hpp>
#include "model_matrix.hpp"
#include "task_parameter.hpp"
#include <bin_picking/nlohmann/json.hpp>

using namespace std;

// api 함수 정보를 저장할 베이스 클래스 인터페이스
class IFunctionWrapper {
public:
    virtual ~IFunctionWrapper() = default;
    virtual void execute() = 0;
};

// 함수 정보를 저장할 클래스 템플릿
template <typename Func, typename... Args>
class FunctionWrapper : public IFunctionWrapper {
public:
    FunctionWrapper(Func func, Args... args)
        : func_(func), args_(std::make_tuple(args...)) {}

    void execute() override {
        std::apply(func_, args_);
    }

private:
    Func func_;
    std::tuple<Args...> args_;
};

typedef enum {
    MB_NONE    = 0,
    MB_ENABLE  = 1,
    MB_STOP_P  = 2,
    MB_STOP_V  = 3,
    MB_DISABLE = 4,

    MB_POS_CTRL = 5,
    MB_VEL_CTRL = 6,
    MB_TOR_CTRL = 7,

    MB_GRP_SET_DIR  = 100,
    MB_GRP_INIT     = 101,
    MB_GRP_OPEN     = 102,
    MB_GRP_CLOSE    = 103,
    MB_GRP_POS_CTRL = 104,
    MB_GRP_INIT2    = 105,
    MB_VAC_ON       = 106,
    MB_VAC_OFF      = 107,

    MB_SET_MAX_VAL = 201,

    MB_SET_PID_GAIN = 211,
} MB_CMD_1_t;

enum class KR_GRP {
    VEL_CTRL = 6,

    INIT       = 101,
    OPEN       = 102,
    CLOSE      = 103,
    POS_CTRL   = 104, // 0 ~ 1000
    INIT_2     = 105,
    VACUUM_ON  = 106,
    VACUUM_OFF = 107,
    SET_TORQUE = 212, // 0 ~ 100
    SET_SPEED  = 213, // 0 ~ 100
    CHANGE_ADDRESS = 111,

    MOTOR_ENABLE       = 1,
    MOTOR_POS_CTRL       = 5,
    POS_RESET       = 8,

};

template <typename EnumType>
EnumType stringToEnum(const std::string& enum_str);


struct TaskParameter {
    vector<double> pose_retry;

	bool is_unit_task_fin       = false;
	bool is_robot_move_fin      = false;
    bool is_llm_node_fin        = false;
    bool is_assembly_fin        = false;
    bool is_assembly_successed  = false;
    bool is_pause_status        = false;

    uint time_10ms = 0;
    uint task_step = 0;
    uint task_end_num;

	Task task_mode = TASK_DEFAULT;

#if BIN_PICKING_FLAG
    ////////////////////////////////////////
    //// Bin picking
	bool is_detection_fin = false;
	bool is_rough_align_fin = false;
	bool is_gripper_fin = false;
	bool is_task_seq_fin = false;
	bool is_task_fail = false;
	bool is_ready_matching = false;
	bool is_ready_task_recog = false;

	bool is_recovery_task = false;
	bool is_scanning_recovery_task = false;
    bool is_json_trajectory = false;

	bool is_assembly_failure_flag = false;
	bool is_scanning_failure_flag = false;
    ////////////////////////////////////////
#endif
};

// typedef struct _imped {
//     CsDouble stiffness;
//     CsDouble nf;
//     CsDouble zeta;
//     CsDouble force_limit;

//     _imped() {
//         stiffness   = {300, 300, 300, 0.3, 0.3, 0.3};
//         nf          = {5, 5, 5, 5, 5, 5};
//         zeta        = {10, 10, 10, 10, 10, 10};
//         force_limit = {50, 50, 50 ,10, 10, 10};
//     }
// } Imped;

typedef struct _assem {
    double insertion_depth;

    // Imped impedance;

    vector<double> move_vel;
    vector<double> contact_vel;
    vector<double> insert_vel;
    vector<double> target_force;
    vector<double> contact_force;

    _assem() {
        move_vel     .resize(6);
        insert_vel   .resize(6);
        contact_vel  .resize(6);
        target_force .resize(6);
        contact_force.resize(6);
    }
} Assem;

typedef struct _loadId {
    string index;
} LoadId;
#if BIN_PICKING_FLAG



typedef struct _detect {
    bool do_scan_sampling;
    size_t sampling_num;

    _detect() {
        do_scan_sampling = false;
        sampling_num = 1;
    }
} Detect;

typedef struct _binPickingParam {
    unsigned int task_mode;

    // double delay;
    // double velocityCS;
    // double velocityJS;
    // double accelerationCS;
    // double accelerationJS;

    bool relative;
    bool grip_state;
    int selectStage;


    int target_id;
    std::string target_name;

    unsigned int tcp_idx;
    unsigned int grp_driver_idx;

    unsigned int js_position_idx;
    unsigned int cs_pose_idx;

    std::vector<double> impedStiffness;
    std::vector<int16_t> gripper_param;   // position, speed, force seq
    uint16_t gripper_command;
    Assem  assemble;
    Detect detection;

    BlendingTraj traj_blending;

    int relay_signal;
    int plc_command;
    int plc_write_data;
    uint16_t demo_tag;
    uint16_t motion_tag;

    bool tip_changing_task_flag;

    size_t current_tip_idx;

    _binPickingParam() {
        gripper_param.resize(3);

        target_id = 0;
        target_name = "none";

        impedStiffness = {500, 500, 500, 1, 1, 1};

        tip_changing_task_flag = false;
    }
} BinPickingParam;
#endif
typedef struct _unitTask {
    Task task_mode;

    int delay_10ms;
    uint time_10ms_between_tasks = 0;
    size_t cnt_grp_pos_cmd = 0;

    double vel_cs;
    double vel_js;
    double acc_cs;
    double acc_js;
    double q_redundant;

    bool relative;
    bool is_assem_fin;
    JsBool is_dtc_mode;

    int grp_pos_percent;
    uint16_t grp_command;
    uint16_t grp_value;
    uint16_t grp_address;

    JsDouble q_target;
    CsDouble x_target;
    CsDouble x_teaching_target;
    JsDouble target_torque;
    CsDouble tcp;
    // Torque guard params
    double torque_guard_threshold = 0.0;
    int    torque_guard_joint_index = -1; // -1 means use vector norm

    Impedance impedance;
    Assem assembly_param;
    LoadId load_id;

    BlendingTraj traj_blending;
    CircularTraj traj_circular;

    uint16_t tag_num;
    string send_command;

    std::string demo_name;
    std::string object_name;

    bool mode_on = false;

#if BIN_PICKING_FLAG
    double vel_cs_custom;
    double vel_js_custom;
    double acc_cs_custom;
    double acc_js_custom;

    bool is_tool_attaching = true;

    std::string tag_teaching_pose;
    std::string pose_idx_teaching_pose;

    std::string task_log;
    // sb 25.02.10
    int task_index_n;

    //// Bin picking
    BinPickingParam bp_param;
#endif

    // Apriltag related
    CsDouble x_target_tag_based;
    std::string tag_name;
    bool is_second_tag_approach = false;
    uint des_tag_distance_mm;
    bool is_cam_auto_focus_on = false;

    std::string pybind_func_name;
    std::string label; // for TASK_LABEL

    std::shared_ptr<IFunctionWrapper> function_wrapper;


    uint16_t register_address;
    bool is_lsb;
    uint16_t bit_address;
    bool status;
    double angle;
    std::string barcode;

    bool is_symmetric;

    _unitTask() {
        task_mode = Task::TASK_DEFAULT;
        delay_10ms = 0;
        grp_command = 0;
        grp_value = 0;
        grp_address = 1;
    }
} UnitTask;

struct taskList {
    vector<UnitTask> task_list;
    string name;
};

class TaskPlannerDefault {


using TaskPushBackFunc = std::function<void(std::vector<UnitTask>&, const nlohmann::json&)>;

public:
    TaskPlannerDefault();
    ~TaskPlannerDefault();

public:
    void setPosePositionDefault();
    void makeTaskListDefault();
    void makeGraphyTaskListDefault();

    void taskPushBack_moduleTask(vector<UnitTask>& target_module_task,
                                 vector<UnitTask>& push_module_task);

public:
    UnitTask ref_unit_task_;

    Impedance impedance_default_;
    Assem assemble_param_;

    double plate_z_base_to_teeth_center_ = 0.0;

    // Task pose & position
    JsDouble task_q_default_;
    CsDouble task_x_default_;

    vector<UnitTask> void_task_list_;
    vector<UnitTask> module_task_test1_;
    vector<UnitTask> module_task_test2_;
    vector<UnitTask> module_task_test3_;
    vector<UnitTask> module_task_test4_;
    vector<UnitTask> module_task_test5_;

    vector<UnitTask> module_task_tool1_attach_;
    vector<UnitTask> module_task_tool1_detach_;
    vector<UnitTask> module_task_tool2_attach_;
    vector<UnitTask> module_task_tool2_detach_;
    vector<UnitTask> module_task_tool3_attach_;
    vector<UnitTask> module_task_tool3_detach_;

    vector<UnitTask> bp_gui_node_task_;

    vector<UnitTask> load_id_task_;

    // Apriltag related
    vector<UnitTask> tag_tracking_1_;
    vector<UnitTask> tag_tracking_2_;
    vector<UnitTask> tag_test_1_;
    vector<UnitTask> tag_test_2_;
    vector<UnitTask> tag_test_3_;

    std::map<std::string, TaskPushBackFunc> task_push_back_map;
public:
    void initializeTaskPushBackMap();
    static vector<double> vectorSum(const vector<double> input1, const vector<double> input2);
    static CsDouble arraySumCS(const CsDouble input1, const CsDouble input2);
    static JsDouble arraySumJS(const JsDouble input1, const JsDouble input2);
    static void vectorMonitor(string string, vector<double> target, unsigned int rowNum,
                              unsigned int columnNum);

    bool blendingPathInit    (BlendingTraj &target_traj);
    bool blendingPathPushBack(BlendingTraj &target_traj, CsDouble x, double xd, double xdd,
                              double waypoint_xd, double radius, double q_redundant = 0,
                              bool is_radius_percent = true);

    void savePoint_pushBack(vector<UnitTask>& target_module_task);

    void taskPushBack_rewind(vector<UnitTask>& target_module_task);

    void taskPushBack_tic  (vector<UnitTask>& target_module_task);
    void taskPushBack_toc  (vector<UnitTask>& target_module_task);
    void taskPushBack_delay(vector<UnitTask>& target_module_task, double delay_10ms);
    void taskPushBack_pause(vector<UnitTask>& target_module_task);

    void taskPushBack_csMove_redundant (vector<UnitTask>& target_module_task, CsDouble pose,
                                        double q_redundant, bool relative = false);
    void taskPushBack_csMove (vector<UnitTask>& target_module_task, CsDouble pose,
                              bool relative = false);
    void taskPushBack_jsMove (vector<UnitTask>& target_module_task, JsDouble joint_position,
                              bool relative = false);
    void taskPushBack_jsMoveOnlyJ6 (vector<UnitTask>& target_module_task, JsDouble joint_position,
                              bool relative = false);



    void taskPushBack_tcpMove(vector<UnitTask>& target_module_task, CsDouble tcp_pose);

    void taskPushBack_moveBlending(vector<UnitTask>& target_module_task, BlendingTraj traj_blending);

    void taskPushBack_setDtc(vector<UnitTask>& target_module_task, JsBool is_dtc_mode, JsDouble target_torque);

    // Torque guard control
    void taskPushBack_torqueGuardOn(std::vector<UnitTask>& target_module_task, int joint_index /*0-based, -1: norm*/, double threshold);
    void taskPushBack_torqueGuardOff(std::vector<UnitTask>& target_module_task);

    // Torque polling control (for periodic torque service calls)
    void taskPushBack_torquePollingOn(std::vector<UnitTask>& target_module_task);
    void taskPushBack_torquePollingOff(std::vector<UnitTask>& target_module_task);
    
    // Flow control label
    void taskPushBack_label(std::vector<UnitTask>& target_module_task, const std::string& label_name);

    // TCP setting
    void taskPushBack_drflSetTCP(vector<UnitTask>& target_module_task, string name);

    void taskPushBack_drflSetImpedance(vector<UnitTask>& target_module_task, bool mode_on = false);
    void taskPushBack_setRobotAutoMode(std::vector<UnitTask>& target_module_task);  
    void taskPushBack_setRobotManualMode(std::vector<UnitTask>& target_module_task);   

    void taskPushBack_setTCP  (vector<UnitTask>& target_module_task, CsDouble tcp);
    void taskPushBack_resetTCP(vector<UnitTask>& target_module_task);
    void taskPushBack_saveForceData(vector<UnitTask>& target_module_task, string index);

    // impedance_
    void taskPushBack_setForceBias (vector<UnitTask>& target_module_task);
    void taskPushBack_loadIdentification(vector<UnitTask>& targeT_module_task);
    void taskPushBack_impedance_on (vector<UnitTask>& target_module_task,
                                    CsDouble impedance_stiffness);
    void taskPushBack_impedance_off(vector<UnitTask>& target_module_task);

    // Gripper related
    void taskPushBack_irl_grp_cmd(vector<UnitTask>& target_module_task,
                                  int grp_pos_percent);
    void taskPushBack_irl_grp_init(vector<UnitTask>& target_module_task,
                                  int grp_pos_percent);
    void taskPushBack_irl_grp_stop(vector<UnitTask>& target_module_task);
    void taskPushBack_irl_grp_enable (vector<UnitTask>& target_module_task);
    void taskPushBack_irl_grp_disable(vector<UnitTask>& target_module_task);
    void taskPushBack_irl_grp(vector<UnitTask>& target_module_task,
                              KR_GRP command, uint16_t value = 0, uint16_t address = 1);
    ////////////////////////////////////////////////////////////
    //// Robot calibration
    ////////////////////////////////////////////////////////////
    void taskPushBack_save_ply(vector<UnitTask>& target_module_task);

    //
    void taskPushBack_llm(vector<UnitTask>& target_module_task);
    void taskPushBack_llm_start(vector<UnitTask>& target_module_task);
    void taskPushBack_llm_fin(vector<UnitTask>& target_module_task);

#if BIN_PICKING_FLAG
    ////////////////////////////////////////////////////////////
    //// Bin picking
    ////////////////////////////////////////////////////////////
    void taskPushBack_delay_1ms  (std::vector<UnitTask>& targetModuleTask, double delay);
    void taskPushBack_delay_1msForTeaching  (std::vector<UnitTask>& targetModuleTask, double delay);
    void taskPushBack_delayScanning  (std::vector<UnitTask>& targetModuleTask, double delay);
    void taskPushBack_csMove2    (std::vector<UnitTask>& targetModuleTask, std::vector<double> pose, double acc, double vel, bool isRelative);
    void taskPushBack_csMove2ForTeaching    (std::vector<UnitTask>& targetModuleTask, std::vector<double> pose, double acc, double vel, bool isRelative);
    void taskPushBack_jsMove2 (std::vector<UnitTask>& target_module_task, JsDouble joint_position, double acc, double vel, bool relative = false);
    void taskPushBack_jsMove2ForTeaching (std::vector<UnitTask>& target_module_task, JsDouble joint_position, double acc, double vel, bool relative = false);
    void taskPushBack_checkJ6LimitsAndJsMove2OnlyJ6 (std::vector<UnitTask>& target_module_task, JsDouble joint_position, double acc, double vel, bool relative = false);
    void taskPushBack_checkJ6LimitsAndJsMove2OnlyJ6_turnMargin (std::vector<UnitTask>& target_module_task, JsDouble joint_position, double acc, double vel, bool relative = false);
    void taskPushBack_checkJ6LimitsAndJsMove2OnlyJ6_reverse (std::vector<UnitTask>& target_module_task, double acc, double vel);


    void taskPushBack_SetJ6PossibleDirection (std::vector<UnitTask>& target_module_task, JsDouble joint_position, double acc, double vel, bool relative = false);
    // sb 25.04.11
    void taskPushBack_csmoverotateknobtoinit (std::vector<UnitTask>& target_module_task, double acc, double vel);
    void taskPushBack_JsMove2OnlyJ6_withAssignedDirection (std::vector<UnitTask>& target_module_task, JsDouble joint_position, double acc, double vel, bool relative = false);
    void taskPushBack_JsMove2OnlyJ6_withReverseAssignedDirection (std::vector<UnitTask>& target_module_task, JsDouble joint_position, double acc, double vel, bool relative = false);
    void taskPushBack_J6RotateByKeycodeAngle(std::vector<UnitTask>& target_module_task, double deg_z);
    void taskPushBack_jsMove2KeycodeJ6Only(std::vector<UnitTask>& target_module_task, double acc, double vel);
    void taskPushBack_csMoveToolFrameKeycode(std::vector<UnitTask>& targetModuleTask, double acc, double vel);
    void taskPushBack_csMoveToolFrameKeycode_for_holder(std::vector<UnitTask>& targetModuleTask, double acc, double vel);

    void taskPushBack_csMoveToolFrame    (std::vector<UnitTask>& targetModuleTask, std::vector<double> pose, double acc, double vel, bool isRelative);
    void taskPushBack_csMoveToolFrameForTeaching    (std::vector<UnitTask>& targetModuleTask, std::vector<double> pose, double acc, double vel, bool isRelative);
    void taskPushBack_csMoveBaseFrameTeachingTargetPose    (std::vector<UnitTask>& targetModuleTask, std::vector<double> relative_pose_from_teaching_pose, double acc, double vel);
    void taskPushBack_csMoveToolFrameTeachingTargetPose    (std::vector<UnitTask>& targetModuleTask, std::vector<double> relative_pose_from_teaching_pose, double acc, double vel);

    void taskPushBack_csMoveBaseFrameTeachingTargetPose2    (std::vector<UnitTask>& targetModuleTask, std::vector<double> target_teaching_pose, std::vector<double> relative_pose_from_teaching_pose, double acc, double vel);
    void taskPushBack_csMoveToolFrameTeachingTargetPose2    (std::vector<UnitTask>& targetModuleTask, std::vector<double> target_teaching_pose, std::vector<double> relative_pose_from_teaching_pose, double acc, double vel);

    void taskPushBack_taskJSON_csMoveMotionTag(std::vector<UnitTask>& targetModuleTask, CsDouble pose, uint16_t demo_tag, uint16_t motion_tag);
    void taskPushBack_taskRecog_moveBlending(std::vector<UnitTask>& targetModuleTask, BlendingTraj traj_blending, uint16_t demo_tag, uint16_t motion_tag);
    void taskPushBack_taskRecog_csMoveMotionTag(std::vector<UnitTask>& targetModuleTask, CsDouble pose, double acc, double vel, uint16_t demo_tag, uint16_t motion_tag);

    void taskPushBack_setTeachingTargetPose(std::vector<UnitTask>& targetModuleTask, std::string tag_teaching_pose, std::string pose_idx_teaching_pose);
    void taskPushBack_setTeachingTargetPoseWithMarkerDetection(std::vector<UnitTask>& targetModuleTask, std::string tag_teaching_pose, std::string pose_idx_teaching_pose);
    void taskPushBack_setTeachingTargetPoseBaseFrameInMarkerTask(std::vector<UnitTask>& targetModuleTask, std::string tag_teaching_pose, std::string pose_idx_teaching_pose);


    void taskPushBack_sendTaskLog(std::vector<UnitTask>& targetModuleTask, std::string log);

    // sb 25.02.10
    void task_index_count(std::vector<UnitTask>& targetModuleTask,int task_index_n);

    //
    void taskPushBack_templateMatching(std::vector<UnitTask>& targetModuleTask, bool do_sampling, size_t sampling_num);


    // AI
    void taskPushBack_poseEstimation(std::vector<UnitTask>& targetModuleTask, bool do_sampling, size_t sampling_num);


    void taskPushBack_doGrasping_binPicking(std::vector<UnitTask>& targetModuleTask, double acc, double vel);
    void taskPushBack_doGrasping_sharedTask(std::vector<UnitTask>& targetModuleTask, double acc, double vel, std::string demo_name = "default", std::string object_name = "default");
    void taskPushBack_doSubGrasping_binPicking(std::vector<UnitTask>& targetModuleTask, double acc, double vel);
    void taskPushBack_doGrasping_binPicking_poseEstResults(std::vector<UnitTask>& targetModuleTask);


    void taskPushBack_task3DScanningTargetObjectNoWaiting(std::vector<UnitTask>& targetModuleTask, std::string target_object = "none");
    void taskPushBack_task3DScanningTargetObject(std::vector<UnitTask>& targetModuleTask, std::string target_object = "none");
    void taskPushBack_taskMatchingTargetObject(std::vector<UnitTask>& targetModuleTask, std::string target_object = "none", bool do_sampling = true, size_t sampling_num = 8, std::string demo_name = "default", bool is_symmetric = false);

    void taskPushBack_taskRecog_3DScanning(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_task3DScanning(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_taskRecog_3DScanning_sharedTask(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_taskRecog_matching_sharedTask(std::vector<UnitTask>& targetModuleTask, bool do_sampling, size_t sampling_num, std::string demo_name = "default");
    void taskPushBack_taskRecog_3DScanningAndMatching(std::vector<UnitTask>& targetModuleTask, bool do_sampling, size_t sampling_num);
    void taskPushBack_taskRecog_SharedTaskZividScanningAndMatching(std::vector<UnitTask>& targetModuleTask, bool do_sampling, size_t sampling_num);
    void taskPushBack_taskRecog_checkMatchingFinished(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_checkMatchingFinishedAndTipChanging(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_setTipChangingFlag(std::vector<UnitTask>& targetModuleTask, bool task_flag);

    void taskPushBack_initializeGripper(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_initializeVer2Gripper(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_setGripperDriver(std::vector<UnitTask>& targetModuleTask, unsigned int grp_driver_idx);
    void taskPushBack_setGripperMinValue(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_setGripperMaxValue(std::vector<UnitTask>& targetModuleTask);

    void taskPushBack_startStackingMode(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_stopStackingMode(std::vector<UnitTask>& targetModuleTask);

    void taskPushBack_getMeasuredForce(std::vector<UnitTask>& targetModuleTask, unsigned int idx);
    void taskPushBack_setMeasuredForce(std::vector<UnitTask>& targetModuleTask);

    // KORAS gripper
    void taskPushBackKORASGripperCmd(std::vector<UnitTask>& targetModuleTask, uint16_t grp_cmd, int16_t position = 10000, uint16_t speed = 1000);
    void taskPushBackKORASGripperCmdForTeaching(std::vector<UnitTask>& targetModuleTask, uint16_t grp_cmd, uint16_t position = 10000, uint16_t speed = 1000);
    void taskPushBackKORASGripperCmdToolDetaching(std::vector<UnitTask>& targetModuleTask, uint16_t grp_cmd, uint16_t position = 10000, uint16_t speed = 1000);

    void taskPushBack_taskRecog_KORASGripperCmd(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_taskRecog_KORASGripperCheckGrasping(std::vector<UnitTask>& targetModuleTask, uint16_t target_position);

    // PLC Modbus
    void taskPushBackPLCModbusCmd(std::vector<UnitTask>& targetModuleTask, int plc_command);
    void taskPushBackPLCModbusCmdWriteRegister(std::vector<UnitTask>& targetModuleTask, int plc_command, int target_pulse);

    // Relay
    void taskPushBackGripperRelayCmd(std::vector<UnitTask>& targetModuleTask, int relay_signal);

    //// Tool changing
    void taskPushBack_selectTCP(std::vector<UnitTask>& targetModuleTask, unsigned int tcp_idx);
    void taskPushBack_taskRecog_selectTCP(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_taskRecog_ToolChangingAttach_jsMove(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_taskRecog_ToolChangingDetach_jsMove(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_taskRecog_CurrentToolDetaching_jsMove(std::vector<UnitTask>& targetModuleTask);

    void taskPushBack_taskRecog_ToolChangingDefaultAttach_jsMove(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_taskRecog_ToolChangingDefaultDetach_jsMove(std::vector<UnitTask>& targetModuleTask);

    //
    void taskPushBack_taskRecog_ToolChangingGoal_csMove(std::vector<UnitTask>& targetModuleTask, bool is_tool_attaching, std::vector<double> pose);


    //// Tip changing
    void taskPushBack_taskRecog_CurrentTipChangingDetach_jsMove(std::vector<UnitTask>& targetModuleTask);

    ////
    void taskPushBack_taskRecog_TipChangingAttach_jsMove(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_taskRecog_TipChangingDetach_jsMove(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_taskRecog_TipChangingDefaultAttach_jsMove(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_taskRecog_TipChangingDefaultDetach_jsMove(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_taskRecog_TipChangingSetTCP(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_taskRecog_TipChangingSetCurrentTipIndex(std::vector<UnitTask>& targetModuleTask, size_t current_tip_idx);
    ////


    //// New
    void taskPushBack_countDetachingPose(std::vector<UnitTask>& targetModuleTask);

    // conveyor system
    void taskPushBack_led_system(vector<UnitTask>& target_module_task, std::string cmd);
    void taskPushBack_taskRecog_led_system(vector<UnitTask>& target_module_task, int led_signal);

    void taskPushBack_relay_system(vector<UnitTask>& target_module_task, std::string cmd);

    // rl
    void taskPushBack_rl (vector<UnitTask>& target_module_task, std::string cmd);
    void taskPushBack_taskRecog_rl(vector<UnitTask>& target_module_task);
    void taskPushBack_rlDone (vector<UnitTask>& target_module_task);


    void taskPushBack_taskRecog_jsMove_index(vector<UnitTask>& target_module_task, unsigned int idx, double acc, double vel, bool isRelative);
    void taskPushBack_taskRecog_csMove_index(vector<UnitTask>& target_module_task, unsigned int idx, double acc, double vel, bool isRelative);


    //// PLC
    void taskPushBack_PLCModbusInitializeStatusFromRobotToAMR(vector<UnitTask>& target_module_task, uint16_t register_address, bool is_lsb, bool status);
    void taskPushBack_PLCModbusWriteStatusFromRobotToAMR(vector<UnitTask>& target_module_task, uint16_t register_address, bool is_lsb, uint16_t bit_address, bool status);
    void taskPushBack_setPLCModbusRotationAngle(vector<UnitTask>& target_module_task, uint16_t register_address, uint16_t bit_address, double angle = -9999);
    void taskPushBack_setPLCModbusBarcodeWrite(vector<UnitTask>& target_module_task, uint16_t register_address, uint16_t bit_address, const std::string& barcode);
    void taskPushBack_setPLCDoMonitoringFlagOnOff(vector<UnitTask>& target_module_task, bool is_flag_on = true);

    ////////////////////////////////////////////////////////////
#endif


    //// Tag
    void taskPushBack_moveToTag(std::vector<UnitTask>& targetModuleTask, unsigned int tag_num);
    void taskPushBack_aprilTagDetection(std::vector<UnitTask>& target_module_task, uint tag_num);

    void taskPushBack_robotEnable(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_robotDisable(std::vector<UnitTask>& targetModuleTask);
    void taskPushBack_setFlagRobotEnableDisableMode(std::vector<UnitTask>& targetModuleTask, bool is_flag_on);
    ////////////////////////////////////////////////////////////

    // Apriltag related
    void taskPushBack_startTagRecog   (vector<UnitTask>& target_module_task);
    void taskPushBack_endSaveTagRecog (vector<UnitTask>& target_module_task);
    void taskPushBack_setCurrentTag   (vector<UnitTask>& target_module_task, string tag_name);
    void taskPushBack_setCurrentTagFromJSON   (vector<UnitTask>& target_module_task, string tag_name);
    void taskPushBack_csMove_tagBased (vector<UnitTask>& target_module_task, CsDouble pose_tag_based);
    void taskPushBack_csMove_toTag    (vector<UnitTask>& target_module_task, uint distance_mm = 0);
    void taskPushBack_csMove_toTag_forApproaching    (vector<UnitTask>& target_module_task, string tag_name, uint distance_mm = 0, bool is_second_tag_approach = false);
    void taskPushBack_csMove_toTag_forConverging    (vector<UnitTask>& target_module_task, string tag_name, uint distance_mm = 0);
    void taskPushBack_csMove_tagSearch(vector<UnitTask>& target_module_task, string tag_name);
    void taskPushBack_setCameraAutoFocus(vector<UnitTask>& target_module_task, bool is_auto_focus_on = false);

    void taskPushBack_taskPybindCsMove(vector<UnitTask>& target_module_task, std::string func_name);
    void taskPushBack_customCode(std::vector<UnitTask>& target_module_task, std::string script);


#if DRFL_CONTROL
template<typename Func, typename... Args>
void taskPushBack_drfl_api_task(std::vector<UnitTask>& target_module_task, Func func, Args... args) {
    UnitTask ref_unit_task_;
    ref_unit_task_.task_mode = Task::TASK_DRFL_API_TASK;
    ref_unit_task_.function_wrapper = std::make_shared<FunctionWrapper<Func, Args...>>(func, args...);
    target_module_task.push_back(ref_unit_task_);
}
#endif

    template <typename Func, typename... Args>
    void callTaskPushBack(Func func, std::vector<UnitTask>& tasks, const nlohmann::json& json_params);

    template <typename Func, typename... Args, std::size_t... I>
    void callTaskPushBackImpl(Func func, std::vector<UnitTask>& tasks, const nlohmann::json& json_params, std::index_sequence<I...>);

//////////////////// method 1///////////////////////////////////
    // 기본 타입에 대한 getJsonArg (예: double, int 등)

// Helper metafunction to check if a type is a specialization of a template (e.g., std::vector)
template <typename, template <typename...> class>
struct is_specialization_of : std::false_type {};

template <template <typename...> class Template, typename... Args>
struct is_specialization_of<Template<Args...>, Template> : std::true_type {};

// Helper metafunction to check if a type is std::array
template <typename T>
struct is_std_array : std::false_type {};

template <typename T, std::size_t N>
struct is_std_array<std::array<T, N>> : std::true_type {};

// Parsing basic types (e.g., int, double, bool)
template <typename T, typename std::enable_if_t<
    !std::is_enum<T>::value &&
    !is_specialization_of<T, std::vector>::value &&
    !is_std_array<T>::value>* = nullptr>
T getJsonArg(const nlohmann::json& json_params, std::size_t index) {
    try {
        T value = json_params.at(index).get<T>();
        return value;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing basic type in getJsonArg at index " << index << ": " << e.what() << std::endl;
        throw;
    }
}

// Parsing std::vector types
template <typename T, typename std::enable_if_t<
    is_specialization_of<T, std::vector>::value>* = nullptr>
T getJsonArg(const nlohmann::json& json_params, std::size_t index) {
    try {
        const auto& json_array = json_params.at(index);
        if (!json_array.is_array()) {
            throw std::runtime_error("Expected a JSON array for std::vector at index " + std::to_string(index));
        }

        T result;
        for (const auto& item : json_array) {
            result.push_back(item.get<typename T::value_type>());
        }
        return result;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing std::vector in getJsonArg at index " << index << ": " << e.what() << std::endl;
        throw;
    }
}

// Parsing std::array types
template <typename T, typename std::enable_if_t<
    is_std_array<T>::value>* = nullptr>
T getJsonArg(const nlohmann::json& json_params, std::size_t index) {
    try {
        const auto& json_array = json_params.at(index);
        if (!json_array.is_array()) {
            throw std::runtime_error("Expected a JSON array for std::array at index " + std::to_string(index));
        }

        T result;
        if (json_array.size() != result.size()) {
            throw std::runtime_error("JSON array size does not match std::array size at index " + std::to_string(index));
        }

        for (std::size_t i = 0; i < result.size(); ++i) {
            result[i] = json_array.at(i).get<typename T::value_type>();
        }
        return result;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing std::array in getJsonArg at index " << index << ": " << e.what() << std::endl;
        throw;
    }
}

// Parsing enum types
template <typename T, typename std::enable_if_t<
    std::is_enum<T>::value>* = nullptr>
T getJsonArg(const nlohmann::json& json_params, std::size_t index) {
    using UnderlyingType = std::underlying_type_t<T>;
    try {
        if (json_params.at(index).is_string()) {
            std::string enum_str = json_params.at(index).get<std::string>();
            return stringToEnum<T>(enum_str);
        } else {
            UnderlyingType value = json_params.at(index).get<UnderlyingType>();
            return static_cast<T>(value);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error parsing enum in getJsonArg at index " << index << ": " << e.what() << std::endl;
        throw;
    }
}
    template <typename Ret, typename ClassType, typename... Args>
    void addTaskToMap(
        const std::string& taskName,
        Ret (ClassType::*method)(std::vector<UnitTask>&, Args...),
        std::map<std::string, std::function<void(std::vector<UnitTask>&, const nlohmann::json&)>>& taskMap
    );

protected:
    bool flag_blending_traj_error_ = true;
    string config_path_;
};

// 함수 타입 정의
using TaskPushBackFunc = std::function<void(std::vector<UnitTask>&, const nlohmann::json&)>;

// // 전역 변수 선언
// extern std::map<std::string, TaskPushBackFunc> task_push_back_map;



#endif // TASK_PLANNER_DEFAULT_HPP
