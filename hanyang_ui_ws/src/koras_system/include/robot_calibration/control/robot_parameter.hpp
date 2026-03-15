/**
 * @file robot_parameter.hpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief Robot parameter setting class
 * @details A class that reads json files to apply and update robot parameters in real time
 * @version 1.0
 * @date 2022-12-29
 *
 * @copyright Copyright (c) 2022 KorasRobotics All Rights Reserved.
 *
 */

#ifndef ROBOT_PARAMETER_HPP
#define ROBOT_PARAMETER_HPP

#include "task_parameter.hpp"
#include "robot_define.hpp"
#include <math.h>
#include <vector>
#include <array>
#include <algorithm>
#include <stdint.h>
#include <iostream>

#include <filesystem>
#include <fstream>
#include <stdio.h>
#include <jsoncpp/json/json.h>

using namespace std;

// #if JS_DOF != CS_DOF
// /**
//  * @brief A function that returns vector<T> corresponding to array<T, n> (if JS_DOF is different from CS_DOF)
//  *
//  * @tparam T Type of the array & vector
//  * @param[in] arr Input variable array<T, n>
//  * @param[out] vec Output variable vector<T>
//  */
// template<typename T>
// inline void arr2Vec(const array<T, JS_DOF> arr, vector<T> &vec) {
//     vec = vector<T>(arr.begin(), arr.end());
// }

// /**
//  * @brief A function that returns array<T, n> corresponding to vector<T> (if JS_DOF is different from CS_DOF)
//  *
//  * @tparam T Type of the array & vector
//  * @param[in] vec Input variable vector<T>
//  * @param[out] arr Output variable array<T, n>
//  */
// template<typename T>
// inline void vec2Arr(const vector<T> vec, array<T, JS_DOF> &arr) {
//     if (vec.size() != arr.size()) {
//         cout << "vec2Arr size error, size of [arr, vec]: ["
//                   << arr.size() << ", " << vec.size() << "]" << endl;
//     } else {
//         copy_n(vec.begin(), vec.size(), arr.begin());
//     }
// }
// #endif

// /**
//  * @brief A function that returns vector<T> corresponding to array<T, n>
//  *
//  * @tparam T Type of the array & vector
//  * @param[in] arr Input variable array<T, n>
//  * @param[out] vec Output variable vector<T>
//  */
// template<typename T>
// inline void arr2Vec(const array<T, CS_DOF> arr, vector<T> &vec) {
//     vec = vector<T>(arr.begin(), arr.end());
// }

// /**
//  * @brief A function that returns array<T, n> corresponding to vector<T>
//  *
//  * @tparam T Type of the array & vector
//  * @param[in] vec Input variable vector<T>
//  * @param[out] arr Output variable array<T, n>
//  */
// template<typename T>
// inline void vec2Arr(const vector<T> vec, array<T, CS_DOF> &arr) {
//     if (vec.size() != arr.size()) {
//         cout << "vec2Arr size error, size of [arr, vec]: ["
//                   << arr.size() << ", " << vec.size() << "]" << endl;
//     } else {
//         copy_n(vec.begin(), vec.size(), arr.begin());
//     }
// }

// /**
//  * @brief A function that returns a generic array corresponding to vector<T>
//  *
//  * @tparam T Type of the array & vector
//  * @param[in] vec Input variable vector<T>
//  * @param[out] arr Output variable T[]
//  */
// template<typename T>
// inline void vec2Arr(const vector<T> vec, T *arr) {
//     for (uint i = 0; i < vec.size(); i++) {
//         arr[i] = vec[i];
//     }
// }

// /**
//  * @brief A function that returns a vector<T> corresponding to T[]
//  *
//  * @tparam T Type of the array & vector
//  * @param[in] arr Input variable T[]
//  * @param[out] vec Output variable vector<T>
//  */
// template<typename T>
// inline void arr2Vec(T *arr, vector<T> &vec) {
//     int size_temp = sizeof arr/sizeof arr[0];
//     vec.resize(size_temp);

//     for (int i = 0; i < size_temp; i++) {
//         vec[i] = arr[i];
//     }
// }

/**
 * @brief A function that returns a vector<T> corresponding to T[] and size
 *
 * @tparam T Type of the array & vector
 * @param[in] arr Input variable T[]
 * @param[out] vec Output variable vector<T>
 * @param[in] size Size of arr
 */
template<typename T>
inline void arr2Vec_size(T *arr, vector<T> &vec, int size) {
    vec.resize(size);

    for (int i = 0; i < size; i++) {
        vec[i] = arr[i];
    }
}


// BIT
#define sbi(PORTX,bitX) PORTX |= (0x01<<bitX)
#define cbi(PORTX,bitX) PORTX &= ~(0x01<<bitX)

inline unsigned char gbi(unsigned char portx, unsigned char bit) {
    return (portx &= (1<<bit))>>bit;
}

inline bool isEqual(double value, double match, int number = 10) {
    return (number >= 1 && number <= 9) ? fabs(value - match) < pow(10, -number) :
                                          fabs(value - match) < pow(10, -10);
}

inline double divideWithFloatingError(double v1, double v2) {
    double value = v1 / v2;
    if (isnan(value) || !isfinite(value)) {
        return 0.0;
    }
    return value;
}

/**
 * @brief Command enumerator inside the robot control cycle (robot control command)
 */
enum class ControlCommand {
    COMMAND_DEFAULTS,         /**< Default command */
    ROBOT_ENABLE,             /**< Robot enable command */
    ROBOT_DISABLE,            /**< Robot disable command */
    ROBOT_STOP,               /**< Stop command (stop trajectory) */
    CLEAR_MALFUNCTION,        /**< Malfunction state clear command */
    BRAKE_LOCK,               /**< Brake lock command */
    BRAKE_UNLOCK,             /**< Brake unlock command */
    BRAKE_UNLOCK_ENABLE,      /**< Enable execution command immediately after brake release */
    JTS_BIAS,                 /**< Command to set the bias of the joint torque sensor (JTS) */
    FTS_BIAS,                 /**< Command to set the bias of the force torque sensor (FTS) */
    JS_HG_ON,                 /**< Joint space hand guiding start command */
    JS_HG_OFF,                /**< Joint space hand guiding stop command */
    FRICTION_OBSERVER,        /**< Change to friction estimation method through friction observer */
    FRICTION_MODEL,           /**< Change to friction estimation method through friction model */
    CD_ON,                    /**< Change to collision detection mode */
    CD_OFF,                   /**< Turn off the collision detection mode */
    JS_TARGET_PATH,           /**< Joint space move command */
    CS_TARGET_PATH,           /**< Cartesian space move command */
    CS_TARGET_PATH_WAYPOINTS, /**< Cartesian space move command to pass multiple waypoints */
    CS_CIRCULAR_3P_PATH,      /**< Circular trajectory move command passing through 3 points */
    CS_CIRCULAR_TAN_PATH,     /**< Circular trajectory move command that moves while looking at the center of the arc */
    CS_BLENDING,              /**< Cartesian space blending move command */
    JS_JOG_START,             /**< Joint space jog start command */
    CS_JOG_START,             /**< Cartesian space jog start command */
    JOG,                      /**< Jog using socket communication*/
    XPAD_JOG_START,           /**< Jog start command using xpad */
    CS_IMPEDANCE_CTRL_ON,     /**< Impedance control start command */
    CS_IMPEDANCE_CTRL_OFF,    /**< Impedance control stop command */
    CS_FORCE_CTRL_ON,         /**< Force control start command */
    CS_FORCE_CTRL_OFF,        /**< Force control stop command */
    SET_TCP,                  /**< Set the tool center pose (TCP) command */
    CS_FORCE_BIAS,            /**< Command to set the bias of the end-effector force */
    SET_OUTPUT,               /**< Set digital output order */
    CSP_CTRL_MODE,            /**< Set the driver control mode CSP */
    CSV_CTRL_MODE,            /**< Set the driver control mode CSV */
    CST_CTRL_MODE,            /**< Set the driver control mode CST */
    // CSP_TOR_OFFSET_ON,        /**<  */
    // CSP_TOR_OFFSET_OFF,       /**<  */
    // SET_OBS_GAIN,             /**<  */
    // SET_P_GAIN,               /**<  */
    // SET_I_GAIN,               /**<  */
    // SET_D_GAIN,               /**<  */
    // GET_P_GAIN,               /**<  */
    // GET_I_GAIN,               /**<  */
    // GET_D_GAIN,               /**<  */
    RECORD_ON,
    RECORD_OFF,
    UPDATE_EEPROM,
    JAMMING_STATE_ENABLE,
};

/**
 * @brief Status enumerator inside the robot control cycle (robot control status)
 */
enum class ControlStatus {
    STATUS_DEFAULTS, /**< Default status */
    ENABLE,          /**< Enable status */
    DISABLE,         /**< Dsiable status */
    ERROR,           /**< Error status */
    JSCTRL,          /**< Joint space control status */
    CSCTRL,          /**< Cartesian space control status */
    JS_JOG,          /**< Joint space jog status */
    CS_JOG,          /**< Cartesian space jog status */
    XPAD_JOG,        /**< Jog with xpad status */
    JAMMING_STATE_ENABLE
};

// /**
//  * @brief Control mode enumerator of motor driver
//  */
// enum class DriverCtrlMode {
//     CSP, /**< Cyclic synchronous position control mode */
//     CSV, /**< Cyclic synchronous velocity control mode */
//     CST  /**< Cyclic synchronous torque control mode */
// };

/**
 * @brief LED color enumerator
 */
enum LedColor {
    LED_OFF    = 0x00, /**< Off */
    LED_RED    = 0x01, /**< Red */
    LED_ORANGE = 0x02, /**< Orange */
    LED_YELLOW = 0x03, /**< Yellow */
    LED_GREEN  = 0x04, /**< Green */
    LED_BLUE   = 0x05, /**< Blue */
    LED_PURPLE = 0x06, /**< Purple */
    LED_MAGENTA= 0x07, /**< Magenta */
    LED_CYAN   = 0x08, /**< Cyan */
    LED_WHITE  = 0x09, /**< White */
};

/**
 * @brief LED brightness enumerator
 */
enum LedBrightness {
    LED_BRIGHTNESS_0 = 10, /**< Least bright */
    LED_BRIGHTNESS_1 = 20,
    LED_BRIGHTNESS_2 = 30,
    LED_BRIGHTNESS_3 = 40, /**< Most bright */
};

/**
 * @brief LED light state enumerator
 */
enum LedSwitching {
    LED_ON       = 1, /**< On */
    LED_BLINK    = 2, /**< Toggle period: 0.5s */
    LED_BLINK_2X = 3, /**< Toggle period: 0.25s  */
    LED_BLINK_4X = 4, /**< Toggle period: 0.125s */
};

/**
 * @brief Enumerator of optimization method in inverse kinematics
 */
enum LMMethod {
    LEVEN,     /**< Levenberg method */
    LEVEN_MARQ /**< Levenberg-marquardt method */
};

// /**
//  * @brief Enumerator of the inverse kinematics solution method
//  */
// enum class InvKineMethod {
//     CLOSED_FORM,  /**< Closed form method */
//     OPTIMIZATION, /**< Optimization method (Levenberg-marquardt method) */
// };

// /**
//  * @brief Robot's control space enumerator
//  */
// enum ControlSpace {
//     JS, /**< Joint space */
//     CS  /**< Cartesian space */
// };

using namespace std;

/**
 * @brief A class that reads a json file to apply and update robot parameters in real time
 *
 * @class RobotParameter
 */
class RobotParameter {
public:
    /**
     * @brief Parameters measured by motor driver and sensor
     */
    typedef struct _MeasuredValue {
        double q [JS_DOF];          /**< Angle of each joint */
        double qd[JS_DOF];          /**< Angular velocity of each joint */
        double qd_filtered[JS_DOF]; /**< Angular velocity through a band-stop (BS) filter */
        double q_prev[JS_DOF];      /**< Angle of each joint of -1 control cycle of the robot */

        double torque_motor       [JS_DOF]; /**< Motor torque */
        double torque_motor_lpf   [JS_DOF]; /**< Motor torque through a low-pass-filter (LPF) */
        double torque_jts_raw     [JS_DOF]; /**< Joint torque sensor (JTS) value of each joint */
        double torque_jts_filtered[JS_DOF]; /**< Filtered JTS value of each joint */

        double voltage[JS_DOF]; /**< Voltage of each joint */
        double current[JS_DOF]; /**< Current of each joint */

        double x [CS_DOF]; /**< Pose of the robot end-effector (based on the base frame of robot) (x, y, z, R, P, Y) */
        double xd[CS_DOF]; /**< Velocity of the robot end-effector (based on the base frame of robot) (x, y, z, R, P, Y) */

        double q_error_integral[JS_DOF];     /**< Sum of angle error (des.q - meas.q) */
        double x_error_integral[CS_DOF];     /**< Sum of pose error (des.x - meas.x) */
        double force_error_integral[CS_DOF]; /**< Sum of force error (target.force - meas.force) */

        double force_sens[CS_DOF]; /**< Froce measured at sensor frame */
        double force_base[CS_DOF]; /**< Force at the end-effector of robot passing through the LPF */
        double force_tool[CS_DOF]; /**< Force at the end-effector of robot in sensor frame */
        double fts_bias[CS_DOF];   /**< Raw data bias of the FTS sensor */

        double input_encoder [JS_DOF]; /**< Input axis encoder value of each joint (deg) */
        double output_encoder[JS_DOF]; /**< Output axis encoder value of each joint (deg) */

        double input_angle_bias[JS_DOF];     /**< Bias for calculating the absolute position of each joint */
        double input_abs_angle_bias[JS_DOF]; /**< Bias for initialize the absolute position of each joint */

        _MeasuredValue() { /**< Struct constructor for setting initial values */
            for (int i = 0; i < JS_DOF; i++) {
                torque_motor_lpf[i] = 0;
                torque_jts_filtered[i] = 0;
                q_error_integral[i] = 0;
            }

            for (int i = 0; i < CS_DOF; i++) {
                force_sens[i] = 0.0;
                force_base[i] = 0.0;
                force_tool[i] = 0.0;
                x_error_integral    [i] = 0;
                force_error_integral[i] = 0;
            }
        }
    } MeasuredValue;

    /**
     * @brief Desired parameters used when control software performs control
     */
    typedef struct _DesiredValue {
        double q  [JS_DOF]; /**< Angle of each joint */
        double qd [JS_DOF]; /**< Angular velocity of each joint */
        double qdd[JS_DOF]; /**< Angular acceleration of each joint */

        double q_prev_1[JS_DOF]; /**< Angle of each joint of -1 control cycle of the robot */
        double q_prev_2[JS_DOF]; /**< Angle of each joint of -2 control cycle of the robot */
        double q_prev_3[JS_DOF]; /**< Angle of each joint of -3 control cycle of the robot */

        double qd_prev_1[JS_DOF]; /**< Angular velocity of each joint of -1 control cycle of the robot */
        double qd_prev_2[JS_DOF]; /**< Angular velocity of each joint of -2 control cycle of the robot */
        double qd_prev_3[JS_DOF]; /**< Angular velocity of each joint of -3 control cycle of the robot */

        double x  [CS_DOF];    /**< Pose of the robot end-effector (based on the base frame of robot) (x, y, z, R, P, Y) */

        double link1 [CS_DOF];
        double link2 [CS_DOF];
        double link3 [CS_DOF];
        double link4 [CS_DOF];
        double link5 [CS_DOF];
        double link6 [CS_DOF];

        double xd [CS_DOF];    /**< Velocity of the robot end-effector (based on the base frame of robot) (x, y, z, R, P, Y) */
        double xdd[CS_DOF];    /**< Acceleration of the robot end-effector (based on the base frame of robot) (x, y, z, R, P, Y) */
        double x_prev[CS_DOF]; /**< Pose of -1 control cycle of the robot (used for jacobian-based CS control) */

        double torque[JS_DOF];        /**< Torque of each joint (including torque compensated through dynamics and PID control gain) */
        double torque_offset[JS_DOF]; /**< Motor torque offset to the motor driver during CSP control */

        double q_arm;  /**< Arm angle used for CS control of 7-DOF robot (closed-form) */
        double qd_arm; /**< Arm angular velocity used for CS control of 7-DOF robot (closed-form) */

        double q_elbow; /**< Elbow axis (3 axis) angle used for control of 7-DOF robot (LM method) */
    } DesiredValue;

    /**
     * @brief Waypoint trajectory parameter
     */
    typedef struct _Waypoint {
        int32_t num; /**< Number of waypoints */
        double poses            [WAYPOINT_LIMIT][CS_DOF]; /**< Pose of each waypoint */
        double velocities       [WAYPOINT_LIMIT][CS_DOF]; /**< Velocity of each waypoint */
        double finish_velocities[WAYPOINT_LIMIT][CS_DOF]; /**< Velocity at the end of each waypoint */
        double accelerations    [WAYPOINT_LIMIT][CS_DOF]; /**< Acceleration at each waypoint */
    } Waypoint;

    /**
     * @brief Parameters used to generate circular trajectory
     */
    typedef struct _CircularTraj {
        double poses[3][CS_DOF]; /**< The beginning, middle, and end poses of the circular trajectory */
        double vel[CS_DOF];      /**< Target speed used for circular trajectory */
        double acc[CS_DOF];      /**< Target acceleration used for circular trajectory */
    } CircularTraj;

    /**
     * @brief Parameters used to generate blending trajectory
     */
    typedef struct _BlendingTraj {
        int waypoint_num;       /**< Number of waypoints */
        bool is_radius_percent; /**< Calculation method of a circular trajectory (true: percent of radius, false: specified radius) */

        double xs  [WAYPOINT_LIMIT][CS_DOF]; /**< Pose of each waypoint */
        double xds [WAYPOINT_LIMIT];         /**< Velocity of each waypoint */
        double xdds[WAYPOINT_LIMIT];         /**< Acceleration of each waypoint */
        double radiuses[WAYPOINT_LIMIT];     /**< Radius of arc trajectory */
        double waypoint_xds[WAYPOINT_LIMIT]; /**< Velocity at the end of each waypoint */ // TODO: 확인 필요
        double qs_redundant[WAYPOINT_LIMIT]; /**< Redundant angle of each waypoint */
    } BlendingTraj;

    /**
     * @brief Target parameters given by the user to the control software
     */
    typedef struct _TargetValue {
        double q  [JS_DOF]; /**< Angle of each joint */
        double qd [JS_DOF]; /**< Angular velocity of each joint */
        double qdd[JS_DOF]; /**< Angular acceleration of each joint */

        double x  [CS_DOF]; /**< Pose of the robot end-effector (based on the base frame of robot) (x, y, z, R, P, Y) */
        double xd [CS_DOF]; /**< Velocity of the robot end-effector (based on the base frame of robot) (x, y, z, R, P, Y) */
        double xdd[CS_DOF]; /**< Acceleration of the robot end-effector (based on the base frame of robot) (x, y, z, R, P, Y) */

        double force[CS_DOF]; /**< Force commands used for direct force control */

        double q_arm;   /**< Arm angle used for CS control of 7-DOF robot (closed-form) */
        double qd_arm;  /**< Arm anglular velocity used for CS control of 7-DOF robot (closed-form) */
        double qdd_arm; /**< Arm anglular acceleration used for CS control of 7-DOF robot (closed-form) */

        double q_elbow;   /**< Elbow (3-axis) angle used for CS control of 7-DOF robot (LM method) */
        double qd_elbow;  /**< Elbow (3-axis) anglular velocity used for CS control of 7-DOF robot (LM method) */
        double qdd_elbow; /**< Elbow (3-axis) anglular acceleration used for CS control of 7-DOF robot (LM method) */

        Waypoint     waypoint;      /**< Parameter used when waypoint trajectory command */
        CircularTraj traj_circular; /**< Circular trajectory command */
        BlendingTraj traj_blending; /**< Blending trajectory command */

        _TargetValue() { /**< Struct constructor for setting initial values */
            for (int i = 0; i < JS_DOF; i++) {
                qd[i]  = 30.0;
                qdd[i] = 60.0;
            }

            for (int i = 0; i < 3; i++) {
                xd [i]     = 0.05;
                xdd[i]     = 0.15;
                xd [i + 3] = 30.0;
                xdd[i + 3] = 120.0;
            }

            qd_arm  = 30.0;
            qdd_arm = 60.0;
        }
    } TargetValue;

    /**
     * @brief Friction related parameters
     */
    typedef struct _Friction {
        double friction_observer [JS_DOF]; /**< Friction torque calculated by friction observer */
        double friction_model    [JS_DOF]; /**< Friction torque calculated through friction model */
        double friction_model_lpf[JS_DOF]; /**< Friction torque through LPF */
        double friction_model_hg [JS_DOF]; /**< Friction model used in hand guiding */

        double friction_model_coeff_coulomb[JS_DOF];    /**< Coulomb coefficient of friction */
        double friction_model_coeff_viscous[JS_DOF][5]; /**< Viscous coefficient of friction */

        _Friction() { /**< Struct constructor for setting initial values */
            for (int i = 0; i < JS_DOF; i++) {
                friction_model[i] = 0.0;
                friction_model_lpf[i] = 0.0;
            }
        }
    } Friction;

    /**
     * @brief Parameters obtained through dynamics calculations
     */
    typedef struct _Dynamics {
        double torque_inertia [JS_DOF]; /**< Inertia torque (including the torque caused by the acceleration of the trajectory) */
        double torque_coriolis[JS_DOF]; /**< Coriolis and centrifugal torque */
        double torque_gravity [JS_DOF]; /**< Gravity torque to withstand the load of the robot */
        double torque_cbm[JS_DOF];      /**< Gravity torque compensated by gravity compensator */

        double inertia[JS_DOF]; /**< Inertia used in CSP control mode (including rotor inertia) */

        double residual        [JS_DOF]; /**< Estimated disturbance torque for collision detection (based on JTS) */
        double residual_current[JS_DOF]; /**< Estimated disturbance torque for collision detection (current based) */

        double tcp[CS_DOF]; /**< Tool center pose of the attached instrument based on the robot end-effector frame */
        double payload;     /**< The mass of the object currently attached to the robot end-effector */
    } Dynamics;

    /**
     * @brief Robot tuning parameters including various control gains
     */
    typedef struct _Gain {
        double cst_p[JS_DOF]; /**< P-gain used in CST control mode */
        double cst_i[JS_DOF]; /**< I-gain used in CST control mode */
        double cst_d[JS_DOF]; /**< D-gain used in CST control mode */

        double csp_p[JS_DOF]; /**< P-gain used in CSP control mode*/
        double csv_p[JS_DOF]; /**< P-gain scaler of CSV control mode */
        double csv_i[JS_DOF]; /**< I-gain scaler of CSV control mode */
        double csv_b[JS_DOF]; /**< Bandwidth of CSV control loop */

        double cs_p[CS_DOF]; /**< P-gain used in jacobian-based CS control */
        double cs_i[CS_DOF]; /**< I-gain used in jacobian-based CS control */
        double cs_d[CS_DOF]; /**< D-gain used in jacobian-based CS control */

        double force_p[CS_DOF]; /**< P-gain used in direct force control */
        double force_i[CS_DOF]; /**< I-gain used in direct force control */

        double observer[JS_DOF]; /**< LPF gain of disturbance observer */

        _Gain() { /**< Struct constructor for setting initial values */
            for (int i = 0; i < JS_DOF; i++) {
                observer[i] = 30.0;
            }
        }
    } Gain;

    /**
     * @brief Parameters related to collision detection
     */
    typedef struct _Collision {
        bool status;      /**< Current collision state */
        JsBool flag; /**< Current collision state of each joint */

        double sensitivity; /**< Collision detection sensitivity (default: 1.0) */

        double threshold_upper    [JS_DOF]; /**< Upper Threshold Used in current-cased Collision Detection */
        double threshold_lower    [JS_DOF]; /**< Lower Threshold Used in current-cased Collision Detection */
        double threshold_upper_acc[JS_DOF]; /**< Upper Threshold Used in current-cased Collision Detection (가감속구간) */
        double threshold_lower_acc[JS_DOF]; /**< Lower Threshold Used in current-cased Collision Detection (가감속구간) */

        double threshold_jts_upper    [JS_DOF]; /**< Upper threshold used in JTS-based collision detection */
        double threshold_jts_lower    [JS_DOF]; /**< Lower threshold used in JTS-based collision detection */
        double threshold_jts_upper_acc[JS_DOF]; /**< Upper threshold used in JTS-based collision detection (acceleration/deceleration range) */
        double threshold_jts_lower_acc[JS_DOF]; /**< Lower threshold used in JTS-based collision detection (acceleration/deceleration range) */

        bool signal_JTS;        /**< Collision signal detected by JTS */
        bool signal_FSR;        /**< Collision signal detected by FSR */
        bool signal_sensorless; /**< Collision signal detected by sensorless collision detection */

        _Collision() { /**< Struct constructor for setting initial values */
            status     = false;
            signal_JTS = false;
            signal_FSR = false;
            signal_sensorless = false;

            flag.fill(false);
        }
    } Collision;

    /**
     * @brief Parameters related to joint angle limitation
     */
    typedef struct _JointLimit {
        bool status; /**< The state where the joint angle limit is exceeded */
        JsBool flag; /**< The state where the joint angle limit is exceeded of each joint */
    } JointLimit;

    typedef struct _JointVelLimit {
        bool status; /**< The state where the joint velocity limit is exceeded */
        JsBool flag; /**< The state where the joint velocity limit is exceeded of each joint */
    } JointVelLimit;

    /**
     * @brief Parameters related to hand guiding
     */
    typedef struct _HandGuiding {
        double coeff_damping;              /**< Damping coefficient used for hand guiding (default: 0.15) */
        double coeff_grav        [JS_DOF]; /**< Used to fine-tune the gravity torque compensation rate in hand guiding (default: 1.0) */
        double coeff_grav_JTS    [JS_DOF]; /**< Used to fine-tune the gravity torque compensation rate in JTS-based hand guiding (default: 1.0) */
        double coeff_friction    [JS_DOF]; /**< Used to fine-tune the friction torque compensation rate in hand guiding (default: 1.0) */
        double coeff_friction_JTS[JS_DOF]; /**< Used to fine-tune the friction torque compensation rate in JTS-based hand guiding (default: 1.0) */

        double w_threshold; // MEMO: 비직관적임
        double w_init;
        double w_des;
        double w_meas;
    } HandGuiding;

    /**
     * @brief Impedance related parameters
     */
    typedef struct _Impedance {
        CsDouble m; /**< Mass vector of impedance parameter */
        CsDouble b; /**< Damping vector of impedance parameter */
        CsDouble k; /**< Stiffness vector of impedance parameter */

        CsDouble force_limit; /**< Maximum value of force allowed in impedance control mode */
        CsDouble force_des;   /**< Desired force for force control mode */
        CsDouble deadzone;
        CsDouble fts_bias;
        CsDouble force_bias_base; // MEMO: 비직관적?
        CsDouble force_bias_tool; // MEMO: 비직관적?
        CsDouble direction;

        CsDouble x_prev_1;  /**< Pose of -1 control cycle of robot used for backward difference during impedance/admittance calculation */
        CsDouble x_prev_2;  /**< Pose of -2 control cycle of robot used for backward difference during impedance/admittance calculation */
        CsDouble xd_prev_1; /**< Velocity of -1 control cycle of robot used for backward difference during impedance/admittance calculation */
        CsDouble xd_prev_2; /**< Velocity of -2 control cycle of robot used for backward difference during impedance/admittance calculation */

        CsBool selection; /**< Impedance selection vector of each direction */
        bool is_tool_frame;

        _Impedance() { /**< Struct constructor for setting initial values */
            selection.fill(true);
            is_tool_frame = false;
    #if CS_DOF == 6
            m = {10, 10, 10, 0.02, 0.02, 0.02};
            b = {150, 150, 150, 0.5, 0.5, 0.5};
            k = {200, 200, 200, 0.1, 0.1, 0.1};
            force_limit = {50, 50, 50, 10, 10, 10};
            force_des = {20, 20, 20, 2, 2, 2};
            deadzone = {0.5, 0.5, 0.5, 0.02, 0.02, 0.02};
            fts_bias = {0, 0, 0, 0, 0 ,0};
            force_bias_base = {0, 0, 0, 0, 0 ,0};
            force_bias_tool = {0, 0, 0, 0, 0 ,0};
            direction = {0, 0, 0, 0, 0, 0};
    #endif
        }
    } Impedance;

    /**
     * @brief Parameters related to robot malfunction
     */
    typedef struct _Malfunction {
        bool status; /**< Malfunction state of robot */
        JsBool flag; /**< Malfunction state of each joint */

        _Malfunction() { /**< Struct constructor for setting initial values */
            status = false;
            flag.fill(false);
        }
    } Malfunction;

    /**
     * @brief Parameters related to joint torque limit
     */
    typedef struct _TorqueLimit {
        bool status; /**< Torque limit state of robot */
        JsBool flag; /**< Torque limit state of each joint */

        _TorqueLimit() { /**< Struct constructor for setting initial values */
            status = false;
            flag.fill(false);
        }
    } TorqueLimit;

    /**
     * @brief Parameters related to control mode
     */
    typedef struct _CtrlMode {
        bool is_friction_observer_mode;   /**< Whether to use observer-based friction estimation (if false, friction model-based estimation) */
        bool is_collision_detection_mode; /**< Collision detection flag */
        bool is_collision_demo_mode;      /**< Collision detection demo mode flag */
        bool is_teaching_mode;            /**< Hand guiding flag */
        bool is_impedance_ctrl_mode;      /**< Impedance control flag */
        bool is_force_ctrl_mode;          /**< Force control flag */
        bool is_csp_torque_offset_mode;   /**< Torque offset flag in CSP control mode */
        bool is_direct_input_mode;        /**< Direct torque control flag */

        DriverCtrlMode driver_ctrl_mode;  /**< Control mode of motor driver: CSP, CSV, CST */

        InvKineMethod ik_method; /**< Methods for solving inverse kinematics: closed-form, LM method, jacobian */

        _CtrlMode() { /**< Struct constructor for setting initial values */
            is_friction_observer_mode   = true;  // true: observer, false: model, true여도 kJtsSelection에 의해 보정됨
            is_collision_detection_mode = false; // true: CD on
            is_collision_demo_mode      = false;
            is_teaching_mode            = false;
            is_impedance_ctrl_mode      = false;
            is_force_ctrl_mode          = false;
            is_csp_torque_offset_mode   = true;
            is_direct_input_mode        = false;

            driver_ctrl_mode = DriverCtrlMode::CSP;
            ik_method = InvKineMethod::OPTIMIZATION;
        }
    } CtrlMode;

    /**
     * @brief Parameters related to the current state of the robot
     */
    typedef struct _CtrlStatus {
        ControlSpace ctrl_space; /**< Control space of robot */ // QUESTION: params_.ctrl_status = ControlStatus::CSCTRL를 이 용도로 사용할 수 있지 않을까?

        bool is_enable;         /**< Enable flag */
        bool is_emergency_stop; /**< Emergency stop flag */
        bool is_path_operating; /**< Path operating flag */
        bool is_path_finished;  /**< Path finished flag */
        int  enable_step[JS_DOF];
        int eeprom_step;

        _CtrlStatus() { /**< Struct constructor for setting initial values */
            ctrl_space = JS;

            is_enable = false;
            is_emergency_stop = false;
            is_path_operating = false;
            is_path_finished  = false;
            for (int i = 0; i < JS_DOF; i++) {
                enable_step[i] = 0;
            }
            eeprom_step = 0;
        }
    } CtrlStatus;

    /**
     * @brief Parameters related to buttons at the robot end-effector
     */
    typedef struct _Button {
        bool btn_1; /**< State of button 1 at the robot end-effector */
        bool btn_2; /**< State of button 2 at the robot end-effector */
        bool btn_3; /**< State of button 3 at the robot end-effector */
        bool btn_4; /**< State of button 4 at the robot end-effector */

        _Button() { /**< Struct constructor for setting initial values */
            btn_1 = false;
            btn_2 = false;
            btn_3 = false;
            btn_4 = false;
        }
    } Button;

    /**
     * @brief Parameters related to direct torque control
     */
    typedef struct _DirectTorque {
        JsBool   selection;    /**< Joint selection vector for direct torque input */
        JsDouble torque_input; /**< Input torque of each joint */

        _DirectTorque() { /**< Struct constructor for setting initial values */
            selection.fill(false);
            torque_input.fill(0);
        }
    } DirectTorque;

    typedef struct _Gripper {
        uint16_t comm_status;
        uint16_t grp_status;
        int16_t  grp_pos;
        int16_t  grp_pos_abs;
        int16_t  grp_vel;
        int16_t  grp_current;
        int16_t  grp_voltage;
        bool     is_grp_operating;

        _Gripper() {
            is_grp_operating = false;
        }
    } Gripper;

    /**
     * @brief Parameters related to digital IO of Beckhoff hardware
     */
    typedef struct _DigitalIO {
        int16_t output_index; /**< Digital output terminal selection */
        int16_t output_value; /**< Digital output value*/

        unsigned char input_state;  /**< Digital input status*/
        unsigned char output_state; /**< Digital output status */
    } DigitalIO;

    /**
     * @brief Encoder related parameters
     */
    typedef struct _EncoderParameters {
        // input incremental encoder
        double rotor_pulse_to_deg     [JS_DOF]; /**< The constant that converts the pulse of the input axis (relative encoder) into an angle */
        double rotor_encoder_direction[JS_DOF]; /**< Direction of the input axis encoder */

        // input abs encoder
        double rotor_abs_pulse_bias  [JS_DOF]; /**< Pulse bias of input axis absolute encoder */
        double rotor_abs_pulse_to_deg[JS_DOF]; /**< The constant that converts the pulse of the absolute encoder on the input axis into an angle */
        int rotor_abs_rotation       [JS_DOF]; /**< Number of revolutions of the input shaft (mainly used in DETS) */

        // output abs encoder
        double abs_pulse_bias   [JS_DOF]; /**< Pulse bias of the absolute encoder on the output axis */
        double abs_pulse_to_deg [JS_DOF]; /**< The constant that converts the pulse of the absolute encoder on the output axis into an angle */
        double encoder_direction[JS_DOF]; /**< Direction of output axis encoder */
    } EncoderParameters;

    /**
     * @brief Module related parameters
     */
    typedef struct _ModuleParameters {
        double reduction_ratio   [JS_DOF]; /**< Reduction ratio */
        double rated_current     [JS_DOF]; /**< Rated current */
        double motor_torque_const[JS_DOF]; /**< Motor torque constant */
        double rotor_inertia     [JS_DOF]; /**< Rotor inertia */

        double init_angle_bias[JS_DOF]; /**< Initial bias of each joint angle */
        double angle_limit_min[JS_DOF]; /**< Joint angle lower limit of each joint */
        double angle_limit_max[JS_DOF]; /**< Joint angle upper limit of each joint */
        double torque_limit   [JS_DOF]; /**< Motor torque limit */

        double jts_torque_const[JS_DOF]; /**< JTS torque constant */
        double jts_bias        [JS_DOF]; /**< Bias of JTS */

        EncoderParameters encoder; /**< Encoder related parameters */

        _ModuleParameters() { /**< Struct constructor for setting initial values */
            for (int i = 0; i < JS_DOF; i++) {
                jts_bias[i] = 0;
            }
        }
    } ModuleParameters;

public:
    /**
     * @brief Construct a new Robot Parameter object
     */
    RobotParameter();
    /**
     * @brief Destroy the Robot Parameter object
     */
    ~RobotParameter();

    JsDouble gain_cst_p_;      /**< P-gain used in CST control mode */
    JsDouble gain_cst_i_;      /**< I-gain used in CST control mode */
    JsDouble gain_cst_d_;      /**< D-gain used in CST control mode */
    JsDouble gain_csp_p_;      /**< P-gain used in CSP control mode */
    JsDouble gain_csv_p_;      /**< P-gain scaler of CSV control mode  */
    JsDouble gain_csv_i_;      /**< I-gain scaler of CSV control mode */
    JsDouble gain_csv_b_;      /**< Bandwidth of CSV control loop */
    CsDouble gain_cs_p_;       /**< P-gain used in jacobian-based CS control */
    CsDouble gain_cs_i_;       /**< I-gain used in jacobian-based CS control */
    CsDouble gain_cs_d_;       /**< D-gain used in jacobian-based CS control */
    CsDouble gain_cs_force_p_; /**< P-gain used in direct force control*/
    CsDouble gain_cs_force_i_; /**< I-gain used in direct force control*/
    CsDouble fts_bias_;        /**< Raw data bias of the FTS */
    CsDouble imped_deadzone_;

    JsDouble abs_pulse_bias;        /**< Pulse bias of the absolute encoder on the output axis */
    JsDouble rotor_abs_pulse_bias_; /**< Pulse bias of the absolute encoder on the input axis*/
    JsDouble jts_bias_;             /**< Bias of JTS value */

    JsDouble col_threshold_upper_jts_; /**< Upper threshold used in JTS-based collision detection */
    JsDouble col_threshold_lower_jts_; /**< Lower threshold used in JTS-based collision detection */
    JsDouble col_threshold_upper_;     /**< Upper Threshold Used in current-cased Collision Detection */
    JsDouble col_threshold_lower_;     /**< Lower Threshold Used in current-cased Collision Detection */
    double col_sensitivity_;           /**< Collision detection sensitivity (default: 1.0) */

    // Gravity, Friction, and hand guiding sensitivity coefficient
    JsDouble hg_coeff_gravity_;      /**< Used to fine-tune the gravity torque compensation rate in hand guiding (default: 1.0) */
    JsDouble hg_coeff_friction_;     /**< Used to fine-tune the friction torque compensation rate in hand guiding (default: 1.0) */
    JsDouble hg_coeff_gravity_jts_;  /**< Used to fine-tune the gravity torque compensation rate in JTS-based hand guiding (default: 1.0) */
    JsDouble hg_coeff_friction_jts_; /**< Used to fine-tune the friction torque compensation rate in JTS-based hand guiding (default: 1.0) */
    double hg_coeff_damping_;        /**< Damping coefficient used for hand guiding (default: 0.15) */
    double w_threshold_;
    double w_init_;

    // friction model
    array<double, JS_DOF>           friction_coulomb_; /**< Coulomb coefficient of friction */
    array<array<double, 5>, JS_DOF> friction_viscous_; /**< Viscous coefficient of friction */
    array<double, JS_DOF> friction_deadzone_;  /**< Total length of Maxwell-slip model's operators */
    array<int, JS_DOF>    number_of_elements_; /**< The number of Maxwell-slip model's operators */

    // Motor param
    JsDouble rated_current_;           /**< Rated current */
    JsDouble motor_torque_const_;      /**< Motor torque constant */
    JsDouble jts_torque_const_;        /**< JTS torque constant */
    JsDouble reduction_ratio_;         /**< Reduction ratio */
    JsDouble angle_limit_min_;         /**< The lower limit of the joint angle of each axis */
    JsDouble angle_limit_max_;         /**< The upper limit of the joint angle of each axis */
    JsDouble init_angle_bias_;         /**< Initial bias of each axis angle */
    JsDouble rotor_pulse_to_deg_;      /**< The constant that converts the pulse of the input axis (relative encoder) into an angle */
    JsDouble rotor_abs_pulse_to_deg_;  /**< The constant that converts the pulse of the absolute encoder on the input axis into an angle */
    JsDouble rotor_encoder_direction_; /**< Direction of the input axis encoder */
    JsDouble abs_pulse_to_deg_;        /**< The constant that converts the pulse of the absolute encoder on the output axis into an angle.*/
    JsDouble encoder_direction_;       /**< Direction of output axis encoder */
    JsDouble torque_limit_;            /**< Motor torque limit */

    // Kinematics & Dynamics param
    array<array<double, 4>, JS_DOF> dh_param_;    /**< DH parameters of the robot */
    array<array<double, 3>, JS_DOF> center_mass_; /**< Center of mass of each link */
    array<array<double, 9>, JS_DOF> inertia_;     /**< Inertia of each link */
    JsDouble rotor_inertia_; /**< Rotor inertia */
    JsDouble link_mass_;     /**< Mass of each link */

public:
    /**
     * @brief Inside the class, read the json file and store it in each variable
     *
     * @return true: success
     * @return false: fail
     */
    bool init();

    /**
     * @brief Update only parameters that can be updated in real time, such as control gains
     *
     * @return true: json file read success
     * @return false: failed to read json file
     */
    bool updateParams();

    string getJsonFolderPath();

private:
    /**
     * @brief Read the ctrl_gain.json file and save it to each variable
     *
     * @return true: success
     * @return false: fail
     */
    bool loadByJson_gain();

    /**
     * @brief Read the motor_param.json file and store it in each variable
     *
     * @return true: success
     * @return false: fail
     */
    bool loadByJson_motorParam();

    /**
     * @brief Read the dynamics_param.json file and store it in each variable
     *
     * @return true: success
     * @return false: fail
     */
    bool loadByJson_dynamicsParam();

    string path_json_folder_; /**< Abs path of the config folder */
};

#endif // ROBOT_PARAMETER_HPP
