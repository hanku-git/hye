#ifndef DEFINE_H
#define DEFINE_H

// #ifndef BOOL
//     #define BOOL int
// #endif

#ifndef TRUE
    #define TRUE 1
    #define FALSE 0
#endif

#include <cmath>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
// const double CONTROL_PERIOD = 0.008; // 8ms
const double CONTROL_PERIOD = 0.001; // 2ms

const double MAX_UI_VELOCITY = 0.3;
const double MAX_UI_ACCELERATION = 0.3;
const double MAX_UI_ANGULAR_VELOCITY = 1.0;
const double MAX_UI_ANGULAR_ACCELERATION = 0.5;

// JW align
const double MAX_RL_ALIGN_VELOCITY = 0.2; // [m/s]
const double MAX_RL_ALIGN_ACCELERATION = 0.2; // [m/s^2]
const double MAX_RL_ALIGN_ANGULAR_VELOCITY = 1.0; // [deg/s]
const double MAX_RL_ALIGN_ANGULAR_ACCELERATION = 0.5; // [deg/s^2] 

const double MAX_VELOCITY = 0.7;
const double MAX_ACCELERATION = 0.7;
const double MAX_ANGULAR_VELOCITY = 120.0;
const double MAX_ANGULAR_ACCELERATION = 360.0;

const double ERROR_EPSILON = 0.0001;
const double ERROR_REPORT = 0.003;

const double DEGTORAD = M_PI / 180.0;
const double RADTODEG = 180.0 / M_PI;
const double GRAVITY = 9.81;
const double EPSILON = 0.0001;

const int ROBOT_DOF = 6;
const int UR10_DOF = 6;

// Status
const int STATUS_DEFAULTS = 100;
const int ENABLE = 101;
const int DISABLE = 102;
const int JSCTRL = 200;
const int CSCTRL = 300;
const int DEFAULT = 242;
const int CSCTRL_8DOF = 300;
const int CSCTRL_7DOF = 333;
const int CSCTRL_UR_ONLY = 444;
const int CSCTRL_CIRCULAR = 504;
const int CSCTRL_CIRCULAR_UR_ONLY = 555;
const int INITIAL_ANGLE = 503;
const int DEMO = 400;

const int ASSEMBLE_CONTROL = 900;
const int IMPEDANCE_CONTROL = 901;
const int YG_RL_Insert = 902;
const int YG_RL_connector_Insert = 903;
const int JW_RL_Align = 904;
const int JW_RL_Align_POLYNOMIAL = 905;

// Control Command
const int COMMAND_DEFAULTS = 100;
const int ROBOT_ENABLE = 101;
const int ROBOT_DISABLE = 102;
const int ROBOT_STOP = 103;

const int JTS_BIAS = 104;
const int CD_ON = 105;
const int CD_OFF = 106;
const int FRICTION_OBSERVER = 107;
const int FRICTION_MODEL = 108;

const int JS_HG_ON = 110;
const int JS_HG_OFF = 111;
const int CS_HG_ON = 112;
const int CS_HG_OFF = 113;
const int CS_HG_X_ON = 114;
const int CS_HG_X_OFF = 115;
const int CS_HG_Y_ON = 116;
const int CS_HG_Y_OFF = 117;
const int CS_HG_Z_ON = 118;
const int CS_HG_Z_OFF = 119;

const int JS_FRICTION_OBSERVER = 107;
const int CS_IMPEDANCE_INITIATION = 316;

const int SET_DYN_PARA_GRIPPER = 121;
const int SET_DYN_PARA_POINTER = 122;
const int SET_DYN_PARA_PAYLOAD = 123;

const int JSCTRL_START = 200;
const int JS_TARGET_PATH = 201;
const int JS_TARGET_FOR_CASE_PATH = 205;

const int CSCTRL_START = 300;
const int CS_TARGET_PATH_8DOF = 301;
const int CS_TARGET_PATH_7DOF = 189;
const int CS_TARGET_PATH = 789;

const int CASE0 = 299;
const int CASE00 = 239;
const int CASE1 = 799;
const int CASE2 = 389;
const int CASE3 = 289;
const int CASE4 = 199;
const int CASE5 = 999;
const int CASE6 = 949;
const int CASE7 = 499;
const int CASE8 = 439;
const int CASE9 = 793;
const int CASE10 = 324;
const int CASE11 = 334;
const int CASE12 = 323;
const int CASE13 = 634;
const int CASE14 = 134;
const int CASE15 = 635;
const int CASE16 = 432;
const int CASE17 = 633;

const int MARKER_DETECTION = 790;
const int MOBILE_TRANSLATION = 791;
const int MOBILE_TS = 797;
const int MOBILE_ROTATION = 792;

const int CS_CIRCULAR_PATH = 317;
const int CS_CIRCULAR_PATH_UR_ONLY = 645;

const int CS_IMPEDANCE_CTRL_ON = 310;
const int CS_IMPEDANCE_CTRL_OFF = 311;

const int CS_FORCE_CTRL_ON = 320;
const int CS_FORCE_CTRL_OFF = 321;
const int CS_FORCE_BIAS = 322;

const int DEMO1_START = 400;
const int DEMO1_END = 401;
const int DEMO2_START = 402;
const int DEMO2_END = 403;

const int ROBOTIQ_GRP_ATTACH_INIT = 410;
const int ROBOTIQ_GRP_DETACH_INIT = 411;
const int BOLTING_GRP_ATTACH_INIT = 412;
const int BOLTING_GRP_DETACH_INIT = 413;

const int ROBOTIQ_GRP_ATTACH = 420;
const int ROBOTIQ_GRP_DETACH = 421;
const int BOLTING_GRP_ATTACH = 422;
const int BOLTING_GRP_DETACH = 423;

const int ROBOT_PATH_STOP = 444;

const int GRIPPER_OPEN = 500;
const int GRIPPER_CLOSED = 501;

const int SET_TCP = 600;
const int SET_PAYLOAD = 601;
const int SET_SAFETY = 602;

const int COMMAND_ARCBLENDING = 800;
const int COMMAND_ARCBLENDING_ORIENTATION = 801;
const int COMMAND_ARCBLENDING_CHECK = 802;
const int COMMAND_ARCBLENDING_DIFFVEL = 803;

const int ASSEMBLE_TARGET_START = 900;
const int RL_ASSEMBLE_TARGET_START = 901;
const int RL_connecotr_ASSEMBLE_TARGET_START = 902;
const int RL_ALIGN_TARGET_START = 903;
const int RL_ALIGN_POLYNOMIAL_PATH_TARGET_START = 904;

// const int CS_JOG_START = 910;
// const int JS_JOG_START = 911;
// const int CS_JOG_CTRL = 920;
// const int JS_JOG_CTRL = 921;


const int CS_JOG_START = 910;
const int JS_JOG_START = 911;
const int CS_JOG_END   = 912;
const int JS_JOG_END   = 913;




//////////////////////////////////////// 

#define ROBOT_DOF 6
#define ROBOT_DOF_INSP 6
#define GLOBAL_NORMAL_AXIS 2 // 0: x, 1: y, 2: z



// ROS log macro
const std::string log_prefix = std::string("MODULE");
#define _DEBUG
#define EMPTY_INFO(loger, ...) loger;
#ifdef _DEBUG
#define ROS_LOG_INFO(...)  RCLCPP_INFO (rclcpp::get_logger(log_prefix), __VA_ARGS__);
#else
#define ROS_LOG_INFO(...)  EMPTY_INFO (rclcpp::get_logger(log_prefix), __VA_ARGS__);
#endif
#define ROS_LOG_WARN(...)  RCLCPP_WARN (rclcpp::get_logger(log_prefix), __VA_ARGS__);
#define ROS_LOG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger(log_prefix), __VA_ARGS__);



// 3D Scanner
#define SCANNER_TYPE_ZIVID2 true
#define SCANNER_TYPE_ZIVID1 false
#define SCANNER_TYPE_RVBUST_I2370 false
#define SCANNER_TYPE_RVBUST_P31330 false


// 250223
#define DO_FK_CALCULATION false


#endif
