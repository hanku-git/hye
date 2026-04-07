#pragma once
#ifndef ROBOT_DEFINE_HPP
#define ROBOT_DEFINE_HPP

#include <array>
#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <math.h>
#include <cstdio>

using namespace std;

#define ROBOT_NAME "DS_M1013"
#define JS_DOF 6
#define CS_DOF 6
#define CTRL_JOINT_NUM 6
#define CBM_FLAG false
#define DEAFULT_PAYLOAD false
#define CTRL_PERIOD 0.001
#define WAYPOINT_LIMIT 30

const std::array<bool, JS_DOF> kJtsSelection = {true, true, true, false, true, true};

const double kErrThresholdCS   = 0.003;
const double kErrThresholdJS   = 0.05;
const double kXdMax            = 0.75;
const double kXddMax           = 1.25;
const double kQdMax            = 85.0;
const double kQddMax           = 120.0;
const double kXdDefault        = 0.2;
const double kQdDefault        = 60.0;
const double kPayloadMax       = 10;
const double kJogMaxQ          = 150;
const double kJogMaxX          = 0.7;

const double kDeg2Rad = M_PI / 180.0;
const double kRad2Deg = 180.0 / M_PI;

const double kInverseKineTol      = 0.00000005;
const double kInverseKineLambda   = 0.001;
const int    kInverseKineIterLimit = 10;

typedef std::array<double, JS_DOF> JsDouble;
typedef std::array<double, CS_DOF> CsDouble;
typedef std::array<bool,   JS_DOF> JsBool;
typedef std::array<bool,   CS_DOF> CsBool;

// Replace ROS logging with printf
#define ROS_LOG_INFO(fmt, ...)  printf("[INFO] " fmt "\n", ##__VA_ARGS__)
#define ROS_LOG_WARN(fmt, ...)  printf("[WARN] " fmt "\n", ##__VA_ARGS__)
#define ROS_LOG_ERROR(fmt, ...) printf("[ERROR] " fmt "\n", ##__VA_ARGS__)

#endif // ROBOT_DEFINE_HPP
