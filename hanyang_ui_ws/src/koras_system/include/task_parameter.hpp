#pragma once

#ifndef TASK_PARAMETER_HPP
#define TASK_PARAMETER_HPP

#include "ui_define.hpp"
#include <iostream>
#include <string>

using namespace std;

#if JS_DOF != CS_DOF
template<typename T>
inline void arr2Vec(const array<T, JS_DOF> arr, vector<T> &vec) {
    vec = vector<T>(arr.begin(), arr.end());
}

template<typename T>
inline void vec2Arr(const vector<T> vec, array<T, JS_DOF> &arr) {
    if (vec.size() != arr.size()) {
        cout << "vec2Arr size error, size of [arr, vec]: ["
                  << arr.size() << ", " << vec.size() << "]" << endl;
    } else {
        copy_n(vec.begin(), vec.size(), arr.begin());
    }
}
#endif

template<typename T>
inline void arr2Vec(const array<T, CS_DOF> arr, vector<T> &vec) {
    vec = vector<T>(arr.begin(), arr.end());
}

template<typename T>
inline void vec2Arr(const vector<T> vec, array<T, CS_DOF> &arr) {
    if (vec.size() != arr.size()) {
        cout << "vec2Arr size error, size of [arr, vec]: ["
                  << arr.size() << ", " << vec.size() << "]" << endl;
    } else {
        copy_n(vec.begin(), vec.size(), arr.begin());
    }
}

// 매크로로 정의된 Task Enum
#define TASK_ENUM_LIST \
    X(TASK_DEFAULT) \
    X(TASK_DELAY) \
    X(TASK_PAUSE) \
    X(TASK_WAIT_RESUME) \
    X(TASK_TIC) \
    X(TASK_TOC) \
    X(WAIT_TIL_FIN_ROBOTMOVE) \
    X(WAIT_TIL_FIN_ROBOTMOVE_500MS_DELAY) \
    X(WAIT_TIL_FIN_ROBOTMOVE_WITH_SCAN_AND_MATCHING) \
    X(WAIT_TIL_FIN_ROBOTMOVE_FOR_MARKER_DETECTION_CONVERGING) \
    X(WAIT_TIL_FIN_ROBOTMOVE_FOR_MARKER_DETECTION_APPROACHING) \
    X(WAIT_TIL_FIN_WITH_500MS_DELAY) \
    X(WAIT_TIL_FIN_ASSEMBLY) \
    X(WAIT_TIL_FIN_COOPMOVE) \
    X(TASK_CSMOVE_REDUNDANT) \
    X(TASK_CSMOVE) \
    X(TASK_JSMOVE) \
    X(TASK_JSMOVE_ONLY_J6) \
    X(TASK_TCPMOVE) \
    X(TASK_BLENDING) \
    X(TASK_DTC) \
    X(TASK_TORQUE_GUARD_ON) \
    X(TASK_TORQUE_GUARD_OFF) \
    X(TASK_TORQUE_POLLING_ON) \
    X(TASK_TORQUE_POLLING_OFF) \
    X(TASK_LABEL) \
    X(TASK_TOOL_DETACH) \
    X(TASK_TOOL_ATTACH_ROBOTIQGRP) \
    X(TASK_TOOL_ATTACH_BOLTINGGRP) \
    X(TASK_TOOL_ATTACH_VACUUMGRP) \
    X(TASK_SET_DS_TCP) \
    X(TASK_SET_ROBOT_MODE_AUTO) \
    X(TASK_SET_ROBOT_MODE_MANUAL) \ 
    X(TASK_SET_DS_SET_COMPLIANCE_MODE) \
    X(TASK_SET_TCP) \
    X(TASK_RESET_TCP) \
    X(TASK_IMPEDANCE_ON) \
    X(TASK_IMPEDANCE_OFF) \
    X(TASK_SET_FORCE_BIAS) \
    X(TASK_SAVE_FORCE_DATA) \
    X(TASK_LOAD_IDENTIFICATION) \
    X(TASK_ROBOTIQ_ENABLE) \
    X(TASK_ROBOTIQ_GRASP) \
    X(TASK_ROBOTIQ_MOVE) \
    X(TASK_ROBOTIQ_RELEASE) \
    X(TASK_IRLGRP_ENABLE) \
    X(TASK_IRLGRP_DISABLE) \
    X(TASK_IRLGRP_MOVE) \
    X(TASK_IRLGRP_INIT) \
    X(TASK_IRLGRP_STOP) \
    X(TASK_IRLGRP) \
    X(PARALLELTASK_START_ROBOTA) \
    X(PARALLELTASK_START_ROBOTB) \
    X(PARALLELTASK_STOP_ROBOTA) \
    X(PARALLELTASK_STOP_ROBOTB) \
    X(TASK_RETURN_TO_RETRY_FLAG) \
    X(TASK_RETRY_FLAG) \
    X(TASK_RETRY_FLAG_HIDDEN) \
    X(SAVE_POINT) \
    X(LOAD_POINT) \
    X(TASK_REWIND) \
    X(TASK_TCP_MOVE_TO_TAG) \
    X(TASK_PCL_MOVE_TO_SCAN) \
    X(TASK_PCL_SCAN) \
    X(TASK_PCL_SAVE_SCENE) \
    X(TASK_PCL_LOAD_SCENE) \
    X(TASK_PCL_LOAD_ENV_MODEL) \
    X(TASK_PCL_GEN_STL) \
    X(TASK_PCL_SET_COORDINATE) \
    X(TASK_PCL_SET_COORDINATE_PICK_PLACE) \
    X(TASK_PCL_GEN_WAYPOINT) \
    X(TASK_PCL_MOVE_WAYPOINT_1) \
    X(TASK_PCL_MOVE_WAYPOINT_2) \
    X(TASK_PCL_MOVE_TARGET_POSE) \
    X(TASK_PCL_CHECK_LINK_COLLISION) \
    X(TASK_FLAG_PICK_AND_PLACE_END) \
    X(TASK_ZAXIS_DOWN) \
    X(TASK_PEG_IN_HOLE_LG_1) \
    X(TASK_PEG_IN_HOLE_LG_2) \
    X(TASK_MAKE_AUTO_TEACHING_TASK) \
    X(TASK_MAKE_AUTO_TEACHING_PLACE_BACK_TASK) \
    X(TASK_MAKE_PICK_AND_PLACE_TASK) \
    X(TASK_SAVE_REAL_POSE_MAG) \
    X(TASK_SAVE_REAL_POSE_TARGET) \
    X(TASK_JSMOVE_TAG_ALIGN) \
    X(TASK_ZIVID_SCAN_FOR_CAL) \
    X(TASK_ZIVID_WAIT_SAVE_CAL) \
    X(TASK_ZIVID_CAL_ERROR) \
    X(GO_TO_NEXT_TASK) \
    X(WAIT_TIL_FIN_GRASPING) \
    X(WAIT_TIL_FIN_GRASPING_SCANNING_SHARED_TASK) \
    X(WAIT_TIL_FIN_GRASPING_MATCHING_SHARED_TASK) \
    X(WAIT_TIL_FIN_ALIGNMENT) \
    X(WAIT_TIL_FIN_GRIPPER) \
    X(WAIT_TIL_FIN_TEACHING) \
    X(WAIT_TIL_FIN_TEACHING_WITH_MARKER_DETECTION) \
    X(WAIT_TIL_FIN_GRIPPER_delay_100) \
    X(WAIT_TIL_FIN_TEACHING_POSE_BASE_FRAME_IN_MARKER_TASK) \
    X(WAIT_TIL_FIN_TASK_SEQ) \
    X(TASK_DELAY_FOR_TEACHING) \
    X(TASK_DELAY_SCANNING) \
    X(TASK_CSMOVE2) \
    X(TASK_JSMOVE2) \
    X(TASK_RECOG_JSMOVE) \
    X(TASK_RECOG_CSMOVE) \
    X(TASK_JSMOVE2_FOR_TEACHING) \
    X(TASK_CSMOVE2_FOR_TEACHING) \
    X(TASK_CSMOVE_TOOL_FRAME) \
    X(TASK_CSMOVE_TOOL_FRAME_KEYCODE) \
    X(TASK_CSMOVE_TOOL_FRAME_KEYCODE_FOR_HOLDER) \
    X(TASK_CSMOVE_TOOL_FRAME_FOR_TEACHING) \
    X(TASK_CSMOVE_BASE_FRAME_TEACHING_TARGET_POSE) \
    X(TASK_CSMOVE_TOOL_FRAME_TEACHING_TARGET_POSE) \
    X(TASK_CSMOVE_BASE_FRAME_TEACHING_TARGET_POSE2) \
    X(TASK_CSMOVE_TOOL_FRAME_TEACHING_TARGET_POSE2) \
    X(TASK_SET_TEACHING_TARGET_POSE) \
    X(TASK_SET_TEACHING_TARGET_POSE_WITH_MARKER_DETECTION) \
    X(TASK_SEND_TASK_LOG) \
    X(TASK_SELECT_ROBOT_TCP) \
    X(TASK_KORAS_SET_GRIPPER_DRIVER) \
    X(TASK_KORAS_SET_GRIPPER_MIN_VALUE) \
    X(TASK_KORAS_SET_GRIPPER_MAX_VALUE) \
    X(TASK_START_STACKING_MODE) \
    X(TASK_STOP_STACKING_MODE) \
    X(TASK_DS_CUSTOM_CODE) \
    X(TASK_KORAS_GRIPPER_INITIALIZE) \
    X(TASK_KORAS_GRIPPER_INITIALIZE_2) \
    X(TASK_RECOG_SELECT_ROBOT_TCP) \
    X(TASK_RECOG_TOOL_CHANGING_ATTACH_JSMOVE) \
    X(TASK_RECOG_TOOL_CHANGING_DETACH_JSMOVE) \
    X(TASK_RECOG_TOOL_CHANGING_DEFAULT_ATTACH_JSMOVE) \
    X(TASK_RECOG_TOOL_CHANGING_DEFAULT_DETACH_JSMOVE) \
    X(TASK_RECOG_CURRENT_TOOL_DETACHING_JSMOVE) \
    X(TASK_RECOG_TOOL_CHANGING_GOAL_CSMOVE) \
    X(TASK_RECOG_TIP_CHANGING_ATTACH_JSMOVE) \
    X(TASK_RECOG_TIP_CHANGING_DETACH_JSMOVE) \
    X(TASK_RECOG_CURRENT_TIP_CHANGING_DETACH_JSMOVE) \
    X(TASK_RECOG_TIP_CHANGING_DEFAULT_ATTACH_JSMOVE) \
    X(TASK_RECOG_TIP_CHANGING_DEFAULT_DETACH_JSMOVE) \
    X(TASK_RECOG_TIP_CHANGING_SET_ROBOT_TCP) \
    X(TASK_RECOG_TIP_CHANGING_SET_CURRENT_TIP_INDEX) \
    X(TASK_3D_SCANNING_TARGET_OBJECT_NO_WAITING) \
    X(TASK_3D_SCANNING_TARGET_OBJECT) \
    X(TASK_MATCHING_TARGET_OBJECT) \
    X(TASK_3D_SCANNING) \
    X(TASK_3D_SCANNING_AND_MATCHING) \
    X(TASK_CHECK_MATCHING_FINISHED) \
    X(TASK_CHECK_MATCHING_FINISHED_AND_TIP_CHANGING) \
    X(TASK_SET_TIP_CHANGING_FLAG) \
    X(TASK_POSE_ESTIMATION_CAD_MATCHING) \
    X(TASK_POSE_ESTIMATION_AI) \
    X(TASK_GRASPING_OPTIMAL_POSE_BIN_PICKING) \
    X(TASK_GRASPING_OPTIMAL_POSE_BIN_PICKING_SHARED_TASK) \
    X(TASK_GRASPING_SUB_OPTIMAL_POSE_BIN_PICKING) \
    X(TASK_GRASPING_AI_ESTIMATED_POSE_BIN_PICKING) \
    X(TASK_RECOG_BLENDING) \
    X(TASK_RECOG_POSE_TAG_CSMOVE) \
    X(TASK_JSON_POSE_TAG_CSMOVE) \
    X(TASK_DETACH_COUNTING) \
    X(KORAS_GRP_CMD_INITIALIZE) \
    X(KORAS_GRP_CMD_OPEN) \
    X(KORAS_GRP_CMD_CLOSE) \
    X(KORAS_GRP_CMD_POS_CTRL) \
    X(KORAS_GRP_CMD_VAC_ON) \
    X(KORAS_GRP_CMD_VAC_OFF) \
    X(TASK_KORAS_GRIPPER_COMMAND) \
    X(TASK_KORAS_GRIPPER_COMMAND_FOR_TEACHING) \
    X(TASK_KORAS_GRIPPER_COMMAND_TOOL_DETACHING) \
    X(TASK_RECOG_KORAS_GRIPPER_COMMAND) \
    X(TASK_RECOG_KORAS_GRIPPER_CHECK_GRASPING) \
    X(TASK_PLC_MODBUS_COMMAND) \
    X(TASK_PLC_MODBUS_COMMAND_WRITE_REGISTER) \
    X(TASK_PLC_INITIALIZE_STATUS_FROM_ROBOT_TO_AMR) \
    X(TASK_PLC_WRITE_STATUS_FROM_ROBOT_TO_AMR) \
    X(TASK_PLC_WRITE_ROTATION_ANGLE_FROM_ROBOT_TO_AMR) \
    X(TASK_PLC_WRITE_BARCODE_FROM_ROBOT_TO_AMR) \
    X(TASK_PLC_DO_MONITORING_FLAG_ON_OFF) \
    X(TASK_GRIPPER_RELAY_CONTROL_COMMAND) \
    X(TASK_LOAD_ID_JSMOVE) \
    X(TASK_GET_MEASURED_FORCE) \
    X(TASK_SET_MEASURED_FORCE) \
    X(TASK_LED_SYSTEM) \
    X(TASK_RECOG_LED_SYSTEM) \
    X(TASK_RELAY_SYSTEM) \
    X(TASK_RL) \
    X(TASK_RECOG_RL) \
    X(TASK_RL_DONE) \
    X(TASK_3D_SCANNING_SHARED_TASK_NO_WAITING) \
    X(TASK_3D_SCANNING_SHARED_TASK) \
    X(TASK_MATCHING_SHARED_TASK) \
    X(TASK_3D_SCANNING_AND_MATCHING_SHARED_TASK) \
    X(TASK_RECOG_ZIVID_SCANNING_AND_MATCHING) \
    X(TASK_APRIL_TAG_DETECTION) \
    X(TASK_MOVE_TO_TAG) \
    X(TASK_ROBOT_ENABLE) \
    X(TASK_ROBOT_DISABLE) \
    X(TASK_SET_FLAG_ON_ROBOT_ENABLE_DISABLE_MODE) \
    X(TASK_SET_FLAG_OFF_ROBOT_ENABLE_DISABLE_MODE) \
    X(TASK_START_TAG_RECOG) \
    X(TASK_END_SAVE_TAG_RECOG) \
    X(TASK_SET_CURRENT_TAG) \
    X(TASK_SET_CURRENT_TAG_FROM_JSON) \
    X(TASK_CSMOVE_TAG_BASED) \
    X(TASK_CSMOVE_TO_TAG) \
    X(TASK_MARKER_DETECTION_FOR_CONVERGING) \
    X(TASK_MARKER_DETECTION_FOR_APPROACHING) \
    X(TASK_SET_CAM_AUTO_FOCUS) \
    X(TASK_PYBIND_CSMOVE) \
    X(TASK_DRFL_API_TASK) \
    X(TASK_LLM) \
    X(TASK_LLM_START) \
    X(TASK_LLM_FIN) \
    X(TASK_CSMOVE_LLM) \
    X(WAIT_TIL_FIN_ROBOTMOVE_LLM) \
    X(TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6) \
    X(TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6_turnMargin) \
    X(TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6_reverse) \
    X(TASK_CSMOVE_ROTATE_KNOB_TO_INIT) \
    X(TASK_SET_J6_POSSIBLE_DIRECTION) \
    X(TASK_JSMOVE_ONLY_J6_ASSIGNED_DIRECTION) \
    X(TASK_JSMOVE_ONLY_J6_ASSIGNED_DIRECTION_REVERSE) \
    X(TASK_SET_TEACHING_TARGET_POSE_BASE_FRAME_IN_MARKER_TASK) \
    X(TASK_INDEX_COUNT) \
    X(TASK_GRIPPER_CHECK_DELAY_BETWEEN_TASKS) \
    X(TASK_J6_ROTATE_BY_KEYCODE_ANGLE) \
    X(TASK_JSMOVE2_KEYCODE_J6_ONLY)



enum Task {
    #define X(name) name,
    TASK_ENUM_LIST
    #undef X
};

inline std::string task_to_string(Task task) {
    switch (task) {
        #define X(name) case name: return #name;
        TASK_ENUM_LIST
        #undef X
        default: return "UNKNOWN_TASK";
    }
}

inline Task string_to_task(const std::string& task_str) {
    static const std::unordered_map<std::string, Task> task_map = {
        #define X(name) {#name, name},
        TASK_ENUM_LIST
        #undef X
    };

    auto it = task_map.find(task_str);
    if (it != task_map.end()) {
        return it->second;
    }

    throw std::invalid_argument("Invalid task string: " + task_str);
}



enum class MoveErrorType {
    NONE         = 0x0000,
    POS_SIZE     = 0x0001,
    VEL_VALUE    = 0x0002,
    ACC_VALUE    = 0x0004,
    ENABLE_FIRST = 0x0008,
    MALFUNCTION  = 0x0010,
    COL_DETECTED = 0x0020,
    TIME_OUT     = 0x0040
};

enum class DriverCtrlMode {
    CSP,
    CSV,
    CST
};

enum class InvKineMethod {
    CLOSED_FORM,
    OPTIMIZATION,
    JACOBIAN
};

enum ControlSpace {
    JS,
    CS
};

enum JogSpace {
    JS_JOG,
    CS_JOG,
    XPAD_JOG
};

//// Bin picking
enum TargetObject {
    OBJECT_STRIKER,
    OBJECT_SCU,
    OBJECT_GRILL,
    OBJECT_HINGE,
    OBJECT_BOLT,

    OBJECT_CYLINDER,
    OBJECT_SQUARE_PEG,
    OBJECT_BOLT_BUSH,
    OBJECT_WELDING_T_JOINT_GRP_2F,
    OBJECT_WELDING_ELBOW_JOINT,

    OBJECT_CHEMICAL_COUPLER_HOLDER,
    OBJECT_DRUM_HOLE_SURFACE,
    OBJECT_DRUM_HOLE_UNSCREWING,
    OBJECT_DRUM_LID_CAP_UNSCREWING,
    OBJECT_DRUM_LID_CAP_HOLDER

};

//// Bin picking
enum TargetObjectHanyangEng {
    OBJECT_HANYANG_ENG_RIGHT_CHEMICAL_COUPLER_HOLDER,
    OBJECT_HANYANG_ENG_RIGHT_DRUM_HOLE_SURFACE,
    OBJECT_HANYANG_ENG_RIGHT_DRUM_COUPLER_UNSCREWING,
    OBJECT_HANYANG_ENG_RIGHT_DRUM_LID_CAP_UNSCREWING,
    OBJECT_HANYANG_ENG_RIGHT_DRUM_LID_CAP_HOLDER,
    OBJECT_HANYANG_ENG_RIGHT_DRUM_LID_CAP_HOLDER_EMPTY,
    OBJECT_HANYANG_ENG_RIGHT_CHEMICAL_COUPLER_HOLDER_EMPTY,
    OBJECT_HANYANG_ENG_RIGHT_DRUM_LID_CAP_SCREWING,
    OBJECT_HANYANG_ENG_DRUM_LID_TOTAL,
    OBJECT_HANYANG_ENG_LEFT_CHEMICAL_COUPLER_HOLDER,
    OBJECT_HANYANG_ENG_LEFT_DRUM_HOLE_SURFACE,
    OBJECT_HANYANG_ENG_LEFT_DRUM_COUPLER_UNSCREWING,
    OBJECT_HANYANG_ENG_LEFT_DRUM_LID_CAP_UNSCREWING,
    OBJECT_HANYANG_ENG_LEFT_DRUM_LID_CAP_HOLDER,
    OBJECT_HANYANG_ENG_LEFT_DRUM_LID_CAP_HOLDER_EMPTY,
    OBJECT_HANYANG_ENG_LEFT_CHEMICAL_COUPLER_HOLDER_EMPTY,
    OBJECT_HANYANG_ENG_LEFT_DRUM_LID_CAP_SCREWING,
    OBJECT_HANYANG_ENG_RIGHT_DRUM_KEY_CODE,
    OBJECT_HANYANG_ENG_RIGHT_DRUM_HOLDER_WITH_KEY_CODE,
    OBJECT_HANYANG_ENG_RIGHT_DRUM_HOLDER_WITHOUT_KEY_CODE,
    OBJECT_HANYANG_ENG_RIGHT_DRUM_KEY_CODE_UNSCREWING
};

enum BPDemoType {
    DEMO_BP_CNC_LATHE_CYLINDER,
    DEMO_BP_CNC_MILLING_SQUARE_PEG,
    DEMO_BP_CNC_MILLING_BOLT_BUSH,
    DEMO_BP_PICK_WELDING_T_JOINT_2F_GRP,
    DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP,
    DEMO_BP_PICK_BOLT_BUSH,
    DEMO_BP_PICK_WELDING_ELBOW_JOINT,
    DEMO_BP_PICK_CYLINDER,
    DEMO_BP_PICK_SQUARE_PEG,
    DEMO_BP_PICK_GEAR
};

enum MotionTag {
    MOTION_TAG_BLENDING_HOME_TO_CNC_INDOOR,
    MOTION_TAG_BLENDING_HOME_TO_REGRASP_ZIG,
    MOTION_TAG_BLENDING_REGRASP_ZIG_TO_CNC_INDOOR,
    MOTION_TAG_BLENDING_CNC_INDOOR_TO_DETACH,
    MOTION_TAG_BLENDING_DETACH_TO_HOME_1,
    MOTION_TAG_BLENDING_HOME_TO_GRASP_APPROACH_POSE,
    MOTION_TAG_BLENDING_PICK_AND_PLACE_HOME_TO_GRASP_APPROACH_POSE,
    MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_P2P_POSE,
    MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_JSON_POSE,
    MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_DETECTED_ZIG_POSE,
    MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_REGRASPING_ZIG,
    MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_CNC_INNER,
    MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_DETACH,
    MOTION_TAG_JSON_CSMOVE_REGRASP_PICK,
    MOTION_TAG_JSON_CSMOVE_REGRASP_PLACE,
    MOTION_TAG_JSON_CSMOVE_STACKING,
    MOTION_TAG_PLANAR_VACUUM_Z_FIXED_DETECTED_GRASPING_POSE,
    MOTION_TAG_DETECTED_GRASPING_POSE,
    MOTION_TAG_DETECTED_ZIG_POSE,

    MOTION_TAG_BLENDING_PICK_AND_PLACE_HOME_TO_BEFORE_GRASP_APPROACH_POSE,
    MOTION_TAG_PLANAR_VACUUM_Z_FIXED_BEFORE_DETECTED_GRASPING_POSE,
    MOTION_TAG_BEFORE_DETECTED_GRASPING_POSE,
    MOTION_TAG_REGRASPING_POSE,
    MOTION_TAG_REGRASPING_POSE_ORI_DETECTED
};

enum PLCCommand {
    LATHE_CHUCK_CLOSE,
    LATHE_CHUCK_OPEN,
    MILLING_CHUCK_CLOSE,
    MILLING_CHUCK_OPEN,
    CNC_DOOR_CLOSE,
    CNC_DOOR_OPEN,
    CNC_LED_RED_ON,
    CNC_LED_RED_OFF,
    CNC_LED_YELLOW_ON,
    CNC_LED_YELLOW_OFF,
    CNC_LED_GREEN_ON,
    CNC_LED_GREEN_OFF,
    STEP_MOTOR_POSITION_CONTROL,
    STEP_MOTOR_SET_VELOCITY
};

enum RelayCommand {
    PNE1_OFF,
    PNE1_ON,
    PNE2_OFF,
    PNE2_ON,
    VAC1_OFF,
    VAC1_ON,
    VAC2_OFF,
    VAC2_ON,
    ETC1_OFF,
    ETC1_ON,
    ETC2_OFF,
    ETC2_ON
};

enum ConveyorCommand {
    CONVEYOR_1A_ON,
    CONVEYOR_1A_OFF,
    CONVEYOR_1B_ON,
    CONVEYOR_1B_OFF,
    CONVEYOR_2A_ON,
    CONVEYOR_2A_OFF,
    CONVEYOR_2B_ON,
    CONVEYOR_2B_OFF,
    CONVEYOR_3A_ON,
    CONVEYOR_3A_OFF,
    CONVEYOR_3B_ON,
    CONVEYOR_3B_OFF,
    CONVEYOR_4A_ON,
    CONVEYOR_4A_OFF,
    CONVEYOR_4B_ON,
    CONVEYOR_4B_OFF
};

enum LEDCommand {
    ON_LED_RED,
    ON_LED_YELLOW,
    ON_LED_GREEN,
    ON_LED_BUZZER,
    OFF_LED,
};




enum PCDemoTargetStage {
    ON_STAGE_DEFAULT,
    ON_STAGE_INIT,
    ON_STAGE_A1,
    ON_STAGE_A2,
    ON_STAGE_B1,
    ON_STAGE_B2,
    ON_STAGE_FINISHED
};

enum PCDemoTargetPart {
    ON_PART_CPU,
    ON_PART_COOLER,
    ON_PART_SSD,
    ON_PART_RAM,
    ON_PART_HDMI,
    ON_PART_USB,
    ON_PART_POWER,
    ON_PART_DEFAULT
};

typedef struct _MeasuredValue {
    JsDouble q;
    JsDouble qd;

    JsDouble torque_jts;
    // JsDouble voltage;
    // JsDouble current;

    CsDouble x;
    CsDouble xd;

    CsDouble force;
    CsDouble force_sensor;
    CsDouble force_tool;
    CsDouble force_base;

    _MeasuredValue() {
        q = {0, };
        x = {0, };
        qd = {0, };
        xd = {0, };
        force = {0, };
        force_sensor = {0, };
        force_tool = {0, };
        force_base = {0, };
        torque_jts = {0, };
    }
} MeasuredValue;

typedef struct _Waypoint {
    int num;
    array<JsDouble, WAYPOINT_LIMIT> poses;
    array<JsDouble, WAYPOINT_LIMIT> velocities;
    array<JsDouble, WAYPOINT_LIMIT> finish_velocities;
    array<JsDouble, WAYPOINT_LIMIT> accelerations;
} Waypoint;

typedef struct _TargetValue {
    JsDouble q;
    JsDouble qd;
    JsDouble qdd;

    JsDouble q_redundant;
    JsDouble qd_redundant;
    JsDouble qdd_redundant;

    JsDouble torque;

    CsDouble x;
    CsDouble xd;
    CsDouble xdd;

    Waypoint waypoint;
} TargetValue;

typedef struct _Dynamics {
    double payload;
    CsDouble tcp;
} Dynamics;

typedef struct _Collision {
    bool check_flag;
    JsBool flag;

    _Collision() {
        check_flag = false;
        flag.fill(false);
    }
} Collision;

typedef struct _CircularTraj {
    array<CsDouble, 3> poses;
    CsDouble vel;
    CsDouble acc;
} CircularTraj;

typedef struct _BlendingTraj {
    int waypoint_num;
    bool is_radius_percent;

    /////////////// modified
    double xd;
    double xdd;
    double waypoint_xd;
    double radius;
    /////////////// modified

    vector<CsDouble> xs;
    vector<double> xds;
    vector<double> xdds;
    vector<double> radiuses;
    vector<double> waypoint_xds;
    vector<double> qs_redundant;

    _BlendingTraj() {
        xs  .clear();
        xds .clear();
        xdds.clear();
        radiuses    .clear();
        waypoint_xds.clear();
        qs_redundant.clear();

        waypoint_num = 0;
        is_radius_percent = true;

        /////////////// modified
        xd = 0.0;
        xdd = 0.0;
        waypoint_xd = 0.0;
        radius = 0.0;
        /////////////// modified
    }
} BlendingTraj;

typedef struct _Impedance {
    CsDouble m;
    CsDouble b;
    CsDouble k;
    CsDouble force_limit;
    CsDouble force_des;

    bool is_tool_frame;
    CsBool imped_selection;
    CsBool force_selection;
    CsBool pos_selection;
} Impedance;

typedef struct _Malfunction {
    bool status;
    JsBool flag;

    _Malfunction() {
        status = false;
        flag.fill(false);
    }
} Malfunction;

typedef struct _TorqueLimit {
    bool status; /**< Torque limit state of robot */
    JsBool flag; /**< Torque limit state of each joint */

    _TorqueLimit() { /**< Struct constructor for setting initial values */
        status = false;
        flag.fill(false);
    }
} TorqueLimit;

typedef struct _CtrlMode {
    bool is_friction_observer_mode;
    bool is_collision_detection_mode;
    bool is_collision_demo_mode;
    bool is_teaching_mode;
    bool is_impedance_ctrl_mode;
    bool is_force_ctrl_mode;

    DriverCtrlMode driver_ctrl_mode;
    InvKineMethod ik_method;
} CtrlMode;

typedef struct _CtrlStatus {
    ControlSpace ctrl_space;

    bool is_enable;
    bool is_emergency_stop;
    bool is_path_operating;
    bool is_path_finished;
    bool is_gripper_finished;
    bool is_detection_finished;
    bool isTaskSeqFin;

    _CtrlStatus() {
        ctrl_space = JS;
        is_enable  = false;
        is_emergency_stop = false;
        is_path_operating = false;
        is_path_finished = false;
        is_gripper_finished = false;
        is_detection_finished = false;
        isTaskSeqFin = false;


    }
} CtrlStatus;

typedef struct _LLM_PARAM {

    bool llm_text_received = false;

    JsDouble q;
    JsDouble qd;
    JsDouble qdd;

    CsDouble x;
    CsDouble xd;
    CsDouble xdd;

    uint8_t cnt;

    std::string task;
    std::string target;
    std::string goal;
    std::vector<std::string> passes;
    std::vector<int32_t> velocities;

} LLM_PARAM;


// typedef struct _DirectTorque {
//     bool is_direct_input_mode;
//     bool direct_input_selection[JS_DOF];

//     double torque_input[JS_DOF];

//     _DirectTorque() {
//         is_direct_input_mode = false;
//     }
// } DirectTorque;

typedef struct _ControlParameters {
    MeasuredValue meas;
    TargetValue   target;
    Dynamics      dynamics;
    CircularTraj  traj_circular;
    BlendingTraj  traj_blending;
    Impedance     imped;
    CtrlMode      mode;
    CtrlStatus    status;
    Collision     col;
    Malfunction   malfunction;
    TorqueLimit   torque_limit;
    LLM_PARAM     llm_param;
    // DirectTorque  direct_torque;
    // ControlParameters 구조체의 JSON 직렬화 정의

} ControlParameters;

enum class LLM_TASK {
    CSP,
    CSV,
    CST
};

#endif // TASK_PARAMETER_HPP
