#ifndef ROBOT_DEFINE_HPP
#define ROBOT_DEFINE_HPP

#include <array>
#include <math.h>
#include <rclcpp/rclcpp.hpp>

#define _DEBUG

// ///@{
// /** EC-Master License related macro */
// #define LAN_CARD_NUM 3
// #if LAN_CARD_NUM == 0
// #define LICENSE_KEY ""
// #elif LAN_CARD_NUM == 1
// #define LICENSE_KEY "53612D2D-462B1B48-DA3BE5E8"
// #elif LAN_CARD_NUM == 2
// #define LICENSE_KEY "0B6D9EE9-A0C0065F-3A565373"
// #elif LAN_CARD_NUM == 3
// #define LICENSE_KEY "53612D2D-3B181BD7-5C691549"
// #endif
// ///@}

// DOF & control parameter
#define ROBOT_NAME "UR10e" /**< The name of robot */
#define JS_DOF 6              /**< DOF (degree of freedom) of joint space (usually 6 or 7) */
#define CS_DOF 6              /**< DOF (degree of freedom) of cartesian space (usually 6) */
#define CTRL_JOINT_NUM 6      /**< Control joint number from 1-axis (ex. If you want to control 1-4 axis of 6 axis robot, CTRL_JOINT_NUM = 4 & JS_DOF = 6) */
#define LED_BOARD flase       /**< LED board flag */
#define DEAFULT_PAYLOAD false /**< Default payload setting flag */
#define CBM_FLAG false         /**< Counterbalance Mechanism flag */
#define ENABLE_MOTION false   /**< If defined, enable motion is performed when enable sequence */

#define CTRL_PERIOD	0.002     /**< Control period of robot (usually ) */
#define WAYPOINT_LIMIT 30     /**< Maximum number of waypoints of trajectory (default: 30) */

#define FTS false
#define UR_FTS true

const std::array<bool, JS_DOF> kJtsSelection = {true, true, true, false, true, true}; /**< JTS (joint torque sensor) selection vector */

#if CBM_FLAG
const std::array<bool, JS_DOF> kCbmSelection = {false, true, false, false, false, false}; /**< CBM selection vector */
#endif

#if DEAFULT_PAYLOAD
const double kPayloadDefault = 0; /**< Default payload */
const std::array<double, 3> kCOMDefault = {0, 0, 0}; /**< Default center of mass (COM) of end-effector */
#endif

// Const variables
const double kDeg2Rad = M_PI / 180.0; /**< A constant that converts degree to radian */
const double kRad2Deg = 180.0 / M_PI; /**< A constant that converts radian to degree */
const double kErrThresholdCS = 0.003; /**< Error threshold, which is the criterion for determining trajectory completion (CS) */
const double kErrThresholdJS = 0.05;  /**< Error threshold, which is the criterion for determining trajectory completion (JS) */

const double kXdMax  = 0.75;     /**< Maximum speed of robot end-effector */
const double kXddMax = 1.25;     /**< Maximum acceleration of robot end-effector */
const double kQdMax  = 85.0;    /**< Maximum angular velocity */
const double kQddMax = 120.0;   /**< Maximum angular acceleration */
const double kXdDefault = 0.2;  /**< Default speed of robot end-effector */
const double kQdDefault = 60.0; /**< Default angular velocity*/
const double kPayloadMax = 10;  /**< Maximum payload of robot */
const double kJogMaxQ = 150;    /**< Maximum jog angle in joint space (JS) */
const double kJogMaxX = 0.7;    /**< Maximum jog angle in cartesian space (CS) */

// Inverse kinematics parameters (optimixation method)
const double kInverseKineTol    = 0.00000005; /**< Tolerance of LM method */
const double kInverseKineLambda = 0.001;      /**< Lambda of LM method */
const int kInverseKineIterLimit = 10;         /**< Iteration limit of LM method */

///@{
/** Typedef of joint space and cartesian space variables */
typedef std::array<double, JS_DOF> JsDouble;
typedef std::array<double, CS_DOF> CsDouble;
typedef std::array<bool  , JS_DOF> JsBool;
typedef std::array<bool  , CS_DOF> CsBool;
///@}

#define EMPTY_INFO(loger, ...) loger;

///@{
/** ROS log macro */
#ifdef _DEBUG
#define ROS_LOG_INFO(...)  RCLCPP_INFO (rclcpp::get_logger(ROBOT_NAME), __VA_ARGS__);
#else
#define ROS_LOG_INFO(...)  EMPTY_INFO (rclcpp::get_logger(ROBOT_NAME), __VA_ARGS__);
#endif
#define ROS_LOG_WARN(...)  RCLCPP_WARN (rclcpp::get_logger(ROBOT_NAME), __VA_ARGS__);
#define ROS_LOG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger(ROBOT_NAME), __VA_ARGS__);
///@}

#endif // ROBOT_DEFINE_HPP
