/**
 * @file bin_picking_math.hpp
 * @brief 로봇 제어를 위해 필요한 각종 연산을 위한 헤더파일
 */

#ifndef BIN_PICKING_MATH_HPP
#define BIN_PICKING_MATH_HPP

#include <vector>
#include <string>
#include <eigen3/Eigen/Eigen>
// #include "ui_define.hpp"

/**
 * @class BinPickingMath
 * @brief 빈피킹 시스템을 위한 연산 클래스
 * @ingroup BIN_PICKING
 */
class BinPickingMath {

public:
    BinPickingMath(void);
    ~BinPickingMath(void);

public:
	void transformToolFrame(const std::vector<double> &dPoseEndFrame, const std::vector<double> &x_measured, std::vector<double> &output);
	void RPY2Rot(const Eigen::Vector3d &dRPY, Eigen::Matrix3d &rotM);
	void RPY2Rot(const Eigen::Vector3f &dRPY, Eigen::Matrix3f &rotM);
	void pose2HTM(const Eigen::VectorXd &x, Eigen::Matrix4d &T_out);
	void pose2HTM(const Eigen::VectorXf &x, Eigen::Matrix4f &T_out);
	std::vector<double> htm2pose(const Eigen::Matrix4f &T_in);
	Eigen::Vector3f rotm2ZYX(Eigen::Matrix3f &RotMat);
	Eigen::Matrix3d genRotM(double angle, int direction);
	Eigen::Matrix3f genRotMf(double angle, int direction);
    Eigen::Vector3f rotationMatrixToZYZEulerAngles(const Eigen::Matrix3f& R);
    Eigen::Vector3f rotationMatrixToZYXEulerAngles(const Eigen::Matrix3f& R);
	
	Eigen::Matrix3f ZYZEulerAnglesToRotationMatrix(const Eigen::Vector3f& euler_zyz);
	Eigen::Matrix3f ZYXEulerAnglesToRotationMatrix(const Eigen::Vector3f& euler_zyx);

	Eigen::Vector3f ZYZEulerAnglesToZYXEulerAngles(const Eigen::Vector3f& euler_zyz);
};

#endif
