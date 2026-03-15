/**
 * @file RobotModel.hpp
 * @brief 로봇 제어를 위해 필요한 각종 연산을 위한 헤더파일
 */

#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

#include <vector>
#include <string>
#include <eigen3/Eigen/Eigen>

#include "define.h"

/**
 * @class CModelMatrix
 * @brief 로봇 제어를 위한 연산 클래스
 * @ingroup ROBOT_CONTROL
 */
class CRobotModel {


public:
    CRobotModel(void);
    ~CRobotModel(void);

public:
    void SetDH(double dDH[6][4]);
    void SetTcp(std::vector<double> tcp);
    void GetTcp(std::vector<double>& tcp) {tcp = m_dTcp;}
    void GenTcpHTM(std::vector<double> tcp, double HTM_out[4][4]);
    void GenTcpInverseHTM(std::vector<double> tcp, double HTM_out[4][4]);
    std::vector<double> forwardKinematics(const std::vector<double> q);
    std::vector<double> forwardKinematicsWithJointCenter(const std::vector<double> q, std::vector<Eigen::Vector3f> &joint_center);
    Eigen::Matrix4f forwardKinematics_q2Tr(const Eigen::VectorXf &q);
    int inverseClosedForm(const Eigen::VectorXf &q_past, Eigen::VectorXf &x_current, Eigen::VectorXf &q_current); // Closed-form inverse kinematics
    int inverseNumericalForm(double kInverseKineLambda, int kInverseKineIterLimit, double kInverseKineTol, const Eigen::VectorXf &q_past, const Eigen::VectorXf &x_target, Eigen::VectorXf &q_target); // Numerical-form inverse kinematics
    int inverseNumericalForm2(double kInverseKineLambda, int kInverseKineIterLimit, double kInverseKineTransTol, double kInverseKineOriTol, const Eigen::VectorXf &q_past, const Eigen::VectorXf &x_target, Eigen::VectorXf &q_target);
    Eigen::VectorXf tr2delta(const Eigen::MatrixXf &tr_old, const Eigen::MatrixXf &tr_target);
	void pose2HTM(const Eigen::VectorXf &x, Eigen::Matrix4f &T_out);
    void Rot2RPY(double dRot[3][3], double dRPY[3]);        
    void RPY2Rot(double dRPY[3], double dRot[3][3]);
	void RPY2Rot(const Eigen::Vector3f &dRPY, Eigen::Matrix3f &dRot);
    Eigen::Vector3f rotm2ZYX(Eigen::Matrix3f &RotMat);
    void Base2RPY(double dPoseBaseFrame[6], double dPoseRPY[6], std::vector<double> &measured_q);
    void JogBase2RPY(double dPoseBaseFrame[6], double dPoseRPY[6], std::vector<double> &target_x);
    void End2RPY(double dPoseEndFrame[6], double dPoseRPY[6], std::vector<double> &measured_q);
    void JogEnd2RPY(double dPoseEndFrame[6], double dPoseRPY[6], std::vector<double> &target_x);
    Eigen::Matrix3f genRotM(double angle, int direction);
    Eigen::Matrix4f poseVector2HTM(const std::vector<double> &pose);
	void transformToolFrame(const std::vector<double> &dPoseEndFrame, const std::vector<double> &x_measured, std::vector<double> &output);
	void calJacobianGeo(const Eigen::VectorXf &q_deg, Eigen::MatrixXf &J_geo);
    void calJacobianGeoRad(const Eigen::VectorXf &q_rad, Eigen::MatrixXf &J_geo);

    int val_test_i;

private:
    double m_DH[6][4];
    Eigen::Matrix4f m_TCP;
    Eigen::Matrix4f m_TCP_inv;
    std::vector<double> m_dTcp;
	Eigen::Matrix4f TransformationMatrix(int n, double q);
	double checkFloatingError(double v1, double v2);
	bool checkEquality(double value, double match);

};

#endif
