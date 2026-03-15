/**
 * @file robot_model.hpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief Robot dynamics and kinematics calculation
 * @version 1.0
 * @date 2022-12-29
 *
 * @copyright Copyright (c) 2022 KorasRobotics All Rights Reserved.
 *
 */

#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

#include "robot_model_dynamics.hpp"

using namespace std;

/**
 * @brief Classes for calculating robot dynamics, kinematics, etc.
 * Real-time dynamics calculation is performed by inheriting RobotModelDynamics class.
 */
class RobotModel : protected RobotModelDynamics {
public:
    /**
     * @brief Type of singularity that occurs during inverse kinematics solution through closed-form
     */
    enum class SingularType {
        NONE = 0x00,            /**< None */
        SHOULDER = 0x01,        /**< Shoulder singularity */
        NEAR_SHOULDER = 0x02,   /**< Near the shoulder singularity */
        ELBOW = 0x04,           /**< Elbow singularity */
        NEAR_ELBOW = 0x08,      /**< Near the elbow singularity */
        WRIST = 0x10,           /**< Wrist singularity */
        NEAR_WRIST = 0x20,      /**< Near the wrist singularity */
        UNREACHABLE = 0x40,     /**< Unreachable workspace */
        HIGH_JOINT_VEL = 0x80,  /**< High joint velocity */
        UPPER_ARM = 0x100,      /**< Upper arm singularity */
        NEAR_UPPER_ARM = 0x200, /**< Near the upper arm singularity */
    };

private:
    // JS parameter
    double q_      [JS_DOF]; /**< Angle of each joint (deg) */
    double qd_     [JS_DOF]; /**< Angular velocity of each joint (deg/s) */
    double q_prev_ [JS_DOF]; /**< Angle of each joint of -1 control cycle of the robot */
    double qd_prev_[JS_DOF]; /**< Angular velocity of each joint of -1 control cycle of the robot */

    // Friction compensation
    double friction_model_param_[JS_DOF][6];              /**< Coulomb-viscous friction model parameter */
    int friction_operator_number_[JS_DOF];                /**< The number of Maxwell-slip model's operator */
    std::vector<vector<double>> friction_operator_;       /**< Maxwell-slip model's operator for Coulomb friction compensation  */
    std::vector<vector<double>> friction_operator_max_;   /**< Total length of Maxwell-slip model oerator */
    std::vector<vector<double>> friction_operator_coeff_; /**< Stiffness of Maxwell-slip model's operator */
    double friction_torque_[JS_DOF];                      /**< Friction torque */
    double friction_torque_hg_[JS_DOF];                   /**< Friction torque for hand guiding */
    double coulomb_friction_direction_[JS_DOF];           /**< Signum variable for coulomb friction direction */
    double coulomb_friction_direction_hg_[JS_DOF];        /**< Signum variable for coulomb friction direction in hand guiding mode */
    double tcp_[CS_DOF];                                  /**< Tool center pose (TCP, m, deg) */

    // Homogeneous transformation matrix
    /**
     * @brief Return the transformation matrix of the (joint_num + 1)th joint calculated through the set information (DH, q)
     *
     * @param joint_num Joint of robot (1st Joint when input is 0)
     * @return ModelMatrix Transformation matrix of the joint
     */
    ModelMatrix transformationMatrix(int joint_num);

    /**
     * @brief Return the transformation matrix of the (joint_num + 1)th joint calculated through the input joint angle
     *
     * @param joint_num Joint of robot (1st Joint when input is 0)
     * @param q Angle of joint (deg)
     * @return ModelMatrix Transformation matrix of the joint
     */
    ModelMatrix transformationMatrix(int joint_num, double q);

    /**
     * @brief Function to create tool center pose (TCP) matrix via tcp_
     *
     * @return ModelMatrix Tool center pose (TCP) matrix
     */
    ModelMatrix makeTcpMatrix();

public:
    /**
     * @brief Construct a new RobotModel object
     */
    RobotModel(void);

    /**
     * @brief Destroy the RobotModel object
     */
    ~RobotModel(void);

    /**
     * @brief DH parameter setting
     *
     * @param[in] dh DH parameter (not modified DH parameter)
     */
    void setDH(double dh[JS_DOF][4]);

    /**
     * @brief Return the DH parameter
     *
     * @param[out] dh DH parameter (not modified DH parameter)
     */
    void getDH(double dh[JS_DOF][4]);

    /**
     * @brief Return the angle of each joint
     *
     * @param[out] q Angle of each joint (deg)
     */
    void getq(double q[JS_DOF]);

    /**
     * @brief Return the angular velocity of each joint
     *
     * @param[out] qd Angular velocity of each joint (deg/s)
     */
    void getqd(double qd[JS_DOF]);

    /**
     * @brief Set the angle of each joint
     *
     * @param[in] q Angle of each joint (deg)
     */
    void setq(double q[JS_DOF]);

    /**
     * @brief Set the angular velocity of each joint
     *
     * @param[in] qd Angular velocity of each joint (deg/s)
     */
    void setqd(double qd[JS_DOF]);

    /**
     * @brief Set tool center pose (TCP)
     *
     * @param[in] tcp TCP pose (x, y, z, R, P, Y) relative to the robot end-effector
     */
    void setTcp(double tcp[CS_DOF]);

    /**
     * @brief Return each joint angle of the robot for the target pose of the robot end-effector through inverse kinematics
     *
     * @param[in] x_target Target pose of the robot end-effector (x, y, z, R, P, Y)
     * @param[in] q_current Previous angle (deg) of each joint angle of the robot
     * @param[out] q_target Each joint angle of the robot calculated through inverse kinematics
     * @return true If inverse kinematics solved successfully
     * @return false If inverse kinematics failed
     */
    bool inverseKinematics(vector<double> x_target, vector<double> q_current, vector<double> &q_target);

    bool inverseKinematicsTemp(vector<double> x_target, vector<double> q_current, vector<double> &q_target, int &iter, double &err_norm_output);

    /**
     * @brief Return the transformation matrix of the robot end-effector frame based on base frame of robot through forward kinematics of the robot
     *
     * @param[in] q Angle of each joint (deg)
     * @return ModelMatrix Transformation matrix
     */
    ModelMatrix fwdKine_q2Tr(ModelMatrix q);

    /**
     * @brief Return the jacobian for the input joint angle
     *
     * @param[in] q Angle of each joint (deg)
     * @return ModelMatrix Jacobian matrix
     */
    ModelMatrix calJacobian(ModelMatrix q);

    /**
     * @brief Calculate the error (delta) vector of tr_target for the transformation matrix tr_old
     *
     * @param[in] tr_old Input transformation matrix
     * @param[in] tr_target Input transformation matrix
     * @return ModelMatrix Error vector (delta)
     */
    ModelMatrix tr2delta(ModelMatrix tr_old, ModelMatrix tr_target);

    /**
     * @brief Converts a pose (x, y, z, R, P, Y) into a 4x4 transformation matrix
     *
     * @param[in] xyzrpy Pose in 3D space (x, y, z, R, P, Y)
     * @return ModelMatrix Transformation matrix
     */
    static ModelMatrix pose2tr(ModelMatrix xyzrpy);

    /**
     * @brief Converts a 4x4 transformation matrix into a pose (x, y, z, R, P, Y)
     *
     * @param trans_mat Transformation matrix
     * @return ModelMatrix Pose in 3D space (x, y, z, R, P, Y)
     */
    static ModelMatrix tr2pose(ModelMatrix trans_mat);

    /**
     * @brief Return inverse matrix of transformation matrix
     *
     * @param tr Input transformation matrix
     * @return ModelMatrix Inverse matrix of tr
     */
    ModelMatrix invTrMat(const ModelMatrix &tr);

    /**
     * @brief Return norm for input 3D vector (x, y, z, R, P, Y)
     *
     * @param[in] xyzrpyMat Pose in 3D space (x, y, z, R, P, Y)
     * @return double Norm
     */
    double norm(ModelMatrix xyzrpyMat);

    /**
     * @brief Return the transforamation matrix calculated through the DH parameter and angle of the (joint_num + 1)th joint
     *
     * @param[in] joint_num Joint of robot (1st Joint when input is 0)
     * @param[in] q Angle of each joint (deg)
     * @return ModelMatrix Transformation matrix
     */
    ModelMatrix transformationMatrix(int joint_num, ModelMatrix q);

    /**
     * @brief Return the pose of the robot end-effector (x, y, z, R, P, Y) calculated through the set information (DH, q, etc.)
     *
     * @param[out] x The calculated pose of the robot end-effector (x, y, z, R, P, Y)
     */
    void fwdKine(double x[6]);

    /**
     * @brief Return the pose of the (joint_num + 1)th joint calculated through the set information (DH, q, etc.)
     *
     * @param[in] joint_num Joint of robot (1st Joint when input is 0)
     * @param[out] x The calculated pose of the robot end-effector (x, y, z, R, P, Y)
     */
    void fwdKine(int joint_num, double x[6]);

    /**
     * @brief Return the pose of the robot end-effector (x, y, z, R, P, Y) calculated through the set information (DH, etc.) and the entered joint angle
     *
     * @param[in] q Angle of each joint (deg)
     * @param[out] x The calculated pose of the robot end-effector (x, y, z, R, P, Y)
     */
    void fwdKine(double q[6], double x[6]);

    /**
     * @brief Return the pose of the robot end-effector (ZYZ Euler angle) calculated through the set information (DH, q, etc.)
     *
     * @param[out] x The calculated pose of the robot end-effector (ZYZ Euler angle)
     */
    void fwdKineZYZ(double x[6]);

    /**
     * @brief Return the pose of the (joint_num + 1)th joint calculated through the set information (DH, q, etc.)
     *
     * @param[in] joint_num Joint of robot (1st Joint when input is 0)
     * @param[out] x The calculated pose of the robot end-effector (ZYZ Euler angle)
     */
    void fwdKineZYZ(int joint_num, double x[6]);

    /**
     * @brief Return the velocity (x, y, z, R, P, Y) of the robot end-effector calculated based on the input joint angular velocity (deg/s)
     *
     * @param[in] qd Joint angular velocity (deg/s)
     * @param[out] xd Velocity of the robot end-effector (x, y, z, R, P, Y)
     */
    void fwdDiffKine(double qd[JS_DOF], double xd[6]);

    /**
     * @brief Return the calculated joint angular velocity (deg/s) based on the entered velocity (x, y, z, Rx, Ry, Rz) of the robot end-effector
     *
     * @param[in] xd Velocity of the robot end-effector (x, y, z, R, P, Y)
     * @param[out] qd Joint angular velocity (deg/s)
     */
    void invDiffKine(double xd[6], double qd[JS_DOF]);

    /**
     * @brief Inverse kinematics calculation through damped least square (DLS) inverse
     *
     * @param[in] xd Velocity of the robot end-effector (x, y, z, R, P, Y)
     * @param[in] sigma DLS input variable sigma
     * @param[out] qd Joint angular velocity (deg/s)
     */
    void invDiffKine_DLS(double xd[6], double sigma, double qd[JS_DOF]);

    /**
     * @brief Return the joint angular velocity (deg/s) calculated based on the entered velocity (x, y, z, R, P, Y) of the robot end-effector
     *
     * @param[in] xd Velocity of the robot end-effector (x, y, z, R, P, Y)
     * @param[out] qd Joint angular velocity (deg/s)
     */
    void invDiffKine_rpy(double xd[6], double qd[JS_DOF]);

    /**
     * @brief Calculation of inverse kinematics through damped least square (DLS) inverse
     *
     * @param[in] xd Velocity of the robot end-effector (x, y, z, R, P, Y)
     * @param[in] sigma DLS input variable sigma
     * @param[out] qd Joint angular velocity (deg/s)
     */
    void invDiffKine_rpy_DLS(double xd[6], double sigma, double qd[JS_DOF]);

    /**
     * @brief Calculate the acceleration of the robot end-effector (x, y, z, Rx, Ry, Rz) calculated based on the input joint angular acceleration (deg/s^2)
     *
     * @param[in] qdd Joint angular acceleration (deg/s^2)
     * @param[out] xdd Acceleration of the robot end-effector (x, y, z, Rx, Ry, Rz)
     */
    void fwdAccKine(double qdd[JS_DOF], double xdd[6]);

    /**
     * @brief Calculate joint angular acceleration (deg/s^2) based on the input acceleration (x, y, z, Rx, Ry, Rz) of the robot end-effector
     *
     * @param[in] xd Velocity of the robot end-effector (x, y, z, Rx, Ry, Rz)
     * @param[in] xdd Acceleration of the robot end-effector (x, y, z, Rx, Ry, Rz)
     * @param[out] qdd Joint angular acceleration (deg/s^2)
     */
    void invAccKine(double xd[6], double xdd[6], double qdd[JS_DOF]);

    /**
     * @brief Force/torque (x, y, z, Rx, Ry, Rz) of the robot end-effector calculated through the input torque (Nm)
     *
     * @param[in] torque Torque of each joint (Nm)
     * @param[out] force Force/Torque at the robot end-effector (x, y, z, Rx, Ry, Rz)
     */
    void fwdStatics(double torque[JS_DOF], double force[6]);

    /**
     * @brief Force/torque (x, y, z, Rx, Ry, Rz) of the robot end-effector calculated through DLS inverse
     *
     * @param[in] torque Torque of each joint (Nm)
     * @param[in] sigma DLS input variable sigma
     * @param[out] force Force/Torque at the robot end-effector (x, y, z, Rx, Ry, Rz)
     */
    void fwdStatics_DLS(double torque[JS_DOF], double sigma, double force[6]);

    /**
     * @brief Torque (Nm) of each joint calculated through the input force/torque (x, y, z, Rx, Ry, Rz) of the robot end-effector
     *
     * @param[in] force Force/Torque at the robot end-effector (x, y, z, Rx, Ry, Rz)
     * @param[out] torque Torque of each joint (Nm)
     */
    void invStatics(double force[6], double torque[JS_DOF]);

    /**
     * @brief Calculate and return the determinant of a jacobian (RPY) matrix
     *
     * @return double Determinant of jacobian matrix
     */
    double calDeterminant();

    /**
     * @brief Return geometric jacobian matrix
     *
     * @param[out] jacobian_geo Jacobian matrix
     */
    void calJacobianGeo(ModelMatrix* jacobian_geo);

    /**
     * @brief Return RPY jacobian matrix
     *
     * @param[out] jacobian_rpy Jacobian matrix
     */
    void calJacobianRPY(ModelMatrix* jacobian_rpy);

    /**
     * @brief Return the first-order differentiated jacobian matrix
     *
     * @param[out] jacobian_dot First-order differentiated Jacobian matrix
     */
    void calJacobianTimeDerivate(ModelMatrix* jacobian_dot);

    double calManipulability(std::vector<double> q);

    /**
     * @brief Dynamics parameter settings
     *
     * @param[in] link_mass Mass of each link (kg)
     * @param[in] com Center of mass of each link (m)
     * @param[in] inertia Inertia of each link (kgm^2)
     * @param[in] inertia_rotor Inertia of each rotor (kgm^2)
     */
    void setDynamicParameters(double link_mass[JS_DOF], double com[JS_DOF][3], double inertia[JS_DOF][9], double inertia_rotor[JS_DOF]);

    /**
     * @brief Set filter gain for residual observer
     *
     * @param ctrl_period Control period (sec)
     * @param gain Filter gain
     */
    void setResidualObserver(double ctrl_period, double gain[JS_DOF]);

    /**
     * @brief Return residual torque calculated through JTS-based residual observer
     *
     * @param jts_value Torque (Nm) measured via JTS
     * @param residual_torque Residual Torque (Nm)
     */
    void calResidualObserver(double jts_value[JS_DOF], double residual_torque[JS_DOF]);

    /**
     * @brief Return the residual torque calculated through the current-based residual observer
     *
     * @param torque Estimated torque via current (Nm, excluding estimated friction)
     * @param residual_torque Residual Torque (Nm)
     */
    void calResidualObserverCurrent(double torque[JS_DOF], double residual_torque[JS_DOF]);

    /**
     * @brief Return the friction torque estimated through the JTS-based friction observer
     *
     * @param[in] motor_torque Motor torque (Nm)
     * @param[in] jts_value JTS value (Nm)
     * @param[out] friction_torque Friction torque (Nm)
     */
    void calFrictionObserver(double motor_torque[JS_DOF], double jts_value[JS_DOF], double friction_torque[JS_DOF]);

    /**
     * @brief Return the friction torque estimated through the JTS-based friction observer and the input angular velocity
     *
     * @param[in] qd Joint angular velocity (deg/s)
     * @param[in] motor_torque Motor torque (Nm)
     * @param[in] jts_value JTS value (Nm)
     * @param[out] friction_torque Friction torque (Nm)
     */
    void calFrictionObserver(double qd[JS_DOF], double motor_torque[JS_DOF], double jts_value[JS_DOF], double friction_torque[JS_DOF]);

    /**
     * @brief Calculate and return inertia torque (Nm)
     *
     * @param[in] q Angle of each joint (deg)
     * @param[in] qdd Joint angular acceleration (deg/s^2)
     * @param[out] inertia_torque Inertia torque (Nm)
     */
    void calInertiaTorque(double q[JS_DOF], double qdd[JS_DOF], double inertia_torque[JS_DOF]);

    /**
     * @brief Calculate and return coriolis & centrifugal torque (Nm)
     *
     * @param[in] q Angle of each joint (deg)
     * @param[in] qd Angular velocity of each joint (deg/s)
     * @param[out] coriolis_torque Coriolis torque (Nm)
     */
    void calCoriolisTorque(double q[JS_DOF], double qd[JS_DOF], double coriolis_torque[JS_DOF]);

    /**
     * @brief Calculate and return gravity torque (Nm)
     *
     * @param[in] dq Angle of each joint (deg)
     * @param[out] gravity_torque Gravity torque (Nm)
     */
    void calGravityTorque(double dq[JS_DOF], double gravity_torque[JS_DOF]);

    /**
     * @brief Get the inertia vector
     *
     * @param[out] inertia inertia vector
     */
    void getInertia(double inertia[JS_DOF]);

#if CBM_FLAG
    /**
     * @brief Return the torque compensated through the gravity compensator
     *
     * @param[out] torque_cbm torque compensated (Nm)
     */
    void getCBM(double torque_cbm[JS_DOF]);
#endif

    // Orientation computation
    /**
     * @brief Convert rotation matrix to RPY vector
     *
     * @param[in] rot Rotation matrix (3x3)
     * @param[out] rpy RPY vector
     */
    static void rot2rpy(const double rot[3][3], double rpy[3]);

    /**
     * @brief Convert RPY vector to rotation matrix
     *
     * @param[in] rpy RPY vector
     * @param[out] rot Rotation matrix (3x3)
     */
    static void rpy2rot(const double rpy[3], double rot[3][3], bool is_rad = false);

    /**
     * @brief Return a rotation matrix that is the sum of two rotation matrices
     *
     * @param[in] rot_1 Rotation matrix (3x3)
     * @param[in] rot_2 Rotation matrix (3x3)
     * @param[out] rot_sum Rotation matrix (3x3)
     */
    static void rotSum(double rot_1[3][3], double rot_2[3][3], double rot_sum[3][3]);

    /**
     * @brief Calculate orientation error computed through given rotation matrix
     *
     * @param[in] rot_d Rotation matrix (3x3)
     * @param[in] rot_m Rotation matrix (3x3)
     * @param[out] err_orientation Orientation error vector
     */
    static void orientationError(double rot_d[3][3], double rot_m[3][3], double err_orientation[3]);

    /**
     * @brief Calculate angular velocity through RPY velocity
     *
     * @param[in] rpy RPY angle (deg)
     * @param[in] rpy_dot RPY velocity (deg/s)
     * @param[out] angular_vel Angular velocity (Rx, Ry, Rz) (deg/s)
     */
    static void rpyDot2AngularVel(double rpy[3], double rpy_dot[3], double angular_vel[3]);

    /**
     * @brief Calculate RPY velocity through angular velocity
     *
     * @param[in] rpy RPY angle (deg)
     * @param[in] angular_vel Angular velocity (Rx, Ry, Rz) (deg/s)
     * @param[out] rpy_dot RPY velocity (deg/s)
     */
    static void angularVel2RPYdot(double rpy[3], double angular_vel[3], double rpy_dot[3]);

    /**
     * @brief Calculate angular acceleration through RPY acceleration
     *
     * @param[in] rpy RPY angle (deg)
     * @param[in] rpy_dot RPY velocity (deg/s)
     * @param[in] rpy_acc RPY acceleration (deg/s^2)
     * @param[out] angular_acc Angular acceleration (Rx, Ry, Rz) (deg/s^2)
     */
    static void rpyAcc2AngularAcc(double rpy[3], double rpy_dot[3], double rpy_acc[3], double angular_acc[3]);

    /**
     * @brief Calculate RPY acceleration through angular acceleration
     *
     * @param[in] rpy RPY angle (deg)
     * @param[in] rpy_dot RPY velocity (deg/s)
     * @param[in] angular_acc Angular acceleration (Rx, Ry, Rz) (deg/s^2)
     * @param[out] rpy_acc RPY acceleration (deg/s^2)
     */
    static void angularAcc2RPYAcc(double rpy[3], double rpy_dot[3], double angular_acc[3], double rpy_acc[3]);

    // Frction model
    /**
     * @brief Set the friction model coefficient (coulomb-viscous friction model)
     *
     * @param[in] coeff_coulomb Coulomb friction coefficient
     * @param[in] coeff_viscous Viscous friction coefficient
     * @param[in] deadzone Total length of Maxwell-slip model's operators
     * @param[in] number_of_elements The number of Maxwell-slip model's oeprators
     */
    void setFrictionModel(double coeff_coulomb[JS_DOF], double coeff_viscous[JS_DOF][5], double deadzone[JS_DOF], int number_of_elements[JS_DOF]);

    /**
     * @brief Calculate the friction torque through the input velocity and acceleration and the set friction model
     *
     * @param[in] q_des Desired angle of each joint (deg)
     * @param[in] qd_des Desired angular velocity of each joint (deg/s)
     */
    void calFrictionTorque(double q_des[JS_DOF], double qd_des[JS_DOF]);

    /**
     * @brief Get the friction torque
     *
     * @param[out] friction_torque Friction torque (Nm)
     */
    void getFrictionTorque(double friction_torque[JS_DOF]);

    /**
     * @brief Calculate the friction torque for hand guiding through the input velocity and acceleration and the set friction model
     *
     * @param[in] q_des Desired angle of each joint (deg)
     * @param[in] q_des_prev Desired angle of each joint of -1 control cycle of the robot
     * @param[in] qd_des Desired angular velocity of each joint (deg/s)
     */
    void calFrictionTorqueHG(double q_des[JS_DOF], double q_des_prev[JS_DOF], double qd_des[JS_DOF]);

    /**
     * @brief Get the friction torque for hand guiding
     *
     * @param[out] friction_torque Friction torque (Nm)
     */
    void getFrictionTorqueHG(double friction_torque[JS_DOF]);

    // error
    /**
     * @brief Calculate the RMS error between desired and measured angle
     *
     * @param[in] q_des Desired angle (deg)
     * @param[in] q_meas Measured angle (deg)
     * @return double RMS error of the joint angle
     */
    static double jointPositionRmsError(double q_des[JS_DOF], double q_meas[JS_DOF]);

    /**
     * @brief Calculate the RMS error between desired and measured pose
     *
     * @param[in] q_des Desired pose (x, y, z, r, P, Y)
     * @param[in] q_meas Measured pose (x, y, z, r, P, Y)
     * @return double RMS error between desired and measured pose
     */
    static double poseRmsError(double q_des[6], double q_meas[6]);

    /**
     * @brief Calculate the error between desired and measured pose (not RMS error)
     *
     * @param[in] q_des Desired pose (x, y, z, r, P, Y)
     * @param[in] q_meas Measured pose (x, y, z, r, P, Y)
     * @param[out] error Error between desired and measured pose
     */
    static void poseError(double q_des[6], double q_meas[6], double error[6]);

    // CS control frame transformation
    void end2RPY(double dPoseEndFrame[6], double dPoseRPY[6]);
    void base2RPY(double dPoseBaseFrame[6], double dPoseRPY[6]);

    // effective mass
    void calEffectiveMass(double q[JS_DOF], double xd_des[6], double effective_mass[1]);

    // Inverse kine (closed-form)
    void getPreviousQ(double q[JS_DOF]);
    void setPreviousQ(double q[JS_DOF]);
    void getPreviousQd(double qd[JS_DOF]);
    void setPreviousQd(double qd[JS_DOF]);
    void calculateInverseSolutions_7dof(double current_q[7], double arm_angle, double desired_x[6], double output_q[8][7], bool is_singular[8][5]);
    int inverseClosedForm_7dof(double arm_angle, double current_q[JS_DOF], double target_x[6], double output_q[JS_DOF]);
    int inverseClosedForm_7dof(double arm_angle, double dt, double desired_x[6], double output_q[JS_DOF], double output_qd[JS_DOF], double output_qdd[JS_DOF]);
    void matrixmultiple(double matrix_a[9], double matrix_b[9], double matrix_c[9]);
    void pose2Matrix(double x[6], double temp_t06[16]);
    ModelMatrix pose2Matrix(double x[6]);
    void initInverseKine(double q[JS_DOF]);
    void inverseClosedForm(double q_current[JS_DOF], double x[6], double q_out[JS_DOF], double q_dot_out[JS_DOF], double q_ddot_out[JS_DOF]);
    void compareAngle(double Q1, double Q2, double Q3, double* Q4);

private:
    // Residual
    ModelMatrix* residual_torque_vec_;          /**< JTS based residual torque calculated (Nm) */
    ModelMatrix* residual_torque_current_vec_;  /**< Current based residual torque calculated (Nm) */
    ModelMatrix* friction_torque_observer_vec_; /**< Friction torque estimated by friction observer (Nm) */

    ModelMatrix* residual_torque_integral_;          /**< Integral value of residual torque based on JTS */
    ModelMatrix* residual_torque_current_integral_;  /**< Integral value of residual torque based on current */
    ModelMatrix* friction_torque_observer_integral_; /**< Integral value of friction torque estimated by friction observer */

    ModelMatrix* observer_gain_; /**< Observer gain of residual torque and friction torque */
    double ctrl_period_;         /**< Control period (usually 0.001) */

    ModelMatrix* inertia_rotor_mat_; /**< Rotor inertia matrix */
};

#endif // ROBOT_MODEL_HPP
