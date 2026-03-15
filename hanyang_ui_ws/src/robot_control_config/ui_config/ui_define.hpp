#ifndef DEFINE_HPP
#define DEFINE_HPP

#define _DEBUG

// DOF
#define DRFL_CONTROL true

#if DRFL_CONTROL
    // #define ROBOT_NAME "DS_E0509"
    #define ROBOT_NAME "DS_M1013"
#else
    #define ROBOT_NAME "UR10e"
#endif

#define JS_DOF 6
#define CS_DOF 6

#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <vector>
#include <array>
#include <algorithm>
#include <stdint.h>
#include <iostream>

#include <eigen3/Eigen/Eigen>

const double kDeg2Rad = M_PI / 180.0;
const double kRad2Deg = 180.0 / M_PI;
const double kEpsilon = 0.0001;

#define WAYPOINT_LIMIT 30

using namespace std;

typedef std::array<double, JS_DOF> JsDouble;
typedef std::array<bool  , JS_DOF> JsBool;
typedef std::array<double, CS_DOF> CsDouble;
typedef std::array<bool  , CS_DOF> CsBool;

const JsDouble kQHome = {0, 90, -90, 0, 0, 0};
const JsDouble kQTask = {0, 100, -120, 0, -60, 0};
const CsDouble kXTask = {0.39, 0.01, 0.25, 180, 0, 180};

#define JS_VEL 45
#define JS_ACC 90
#define CS_VEL 0.210
#define CS_ACC 0.410
#define JS_VEL_TASK 45
#define JS_ACC_TASK 90
#define CS_VEL_TASK 0.210
#define CS_ACC_TASK 0.410

#define EMPTY_INFO(loger, ...) loger;

// ROS log macro
const std::string log_prefix = std::string(ROBOT_NAME) + std::string("_UI");
#ifdef _DEBUG
#define ROS_LOG_INFO(...)  RCLCPP_INFO (rclcpp::get_logger(log_prefix), __VA_ARGS__);
#else
#define ROS_LOG_INFO(...)  EMPTY_INFO (rclcpp::get_logger(log_prefix), __VA_ARGS__);
#endif
#define ROS_LOG_WARN(...)  RCLCPP_WARN (rclcpp::get_logger(log_prefix), __VA_ARGS__);
#define ROS_LOG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger(log_prefix), __VA_ARGS__);


// Bin picking flag
#define BIN_PICKING_FLAG true         /**< Bin Picking Package flag */

#define FOOD_AI_FLAG true
// #define TASK_TEST

// Gripper comm.
#define GRIPPER_COMM_RS485 true         /**< Bin Picking Package flag */

#define CAM_TYPE_REALSENSE_D405 false
#define CAM_TYPE_LOGITEC_BRIO true


#define ROBOT_CALIBRATION_MODE false


#define DRFL_FLANGE_GRP_CONTROL false
#define NEW_VER_KORAS_GRIPPER_PACKAGE true


#define IS_PLC_LS_XG5000 false
#define DO_NOT_TRY_HANYANG_ENG_TASK false

////////////////////////////////////////
//// PLC Monitoring ON/OFF
#define IS_PLC_COMMUNICATION false  // SIM: false, REAL: true
////////////////////////////////////////
//// TEST (true면 진행, false면 스킵)
#define IS_SCANNING_AND_GRASPING_MODE true
////////////////////////////////////////


#define IS_RELEASE_VERSION false


// #define SW_MODE_COOKING true
// 전역 변수 선언
extern bool SW_MODE_DEFAULT;
extern bool SW_MODE_HD_LLM;
extern bool SW_MODE_COOKING;
extern bool SW_MODE_HANYANG_ENG;
extern bool SW_MODE_GRAPHY;

extern bool GRIPPER_MODE_KORAS;
extern bool GRIPPER_MODE_FAIRINO;

extern std::string USER_NAME;



#endif // DEFINE_HPP

