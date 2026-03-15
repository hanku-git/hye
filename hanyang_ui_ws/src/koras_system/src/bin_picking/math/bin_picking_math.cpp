/**
 * @file bin_picking_math.cpp
 * @brief 로봇 제어를 위해 필요한 각종 연산을 위한 구현파일
 */

#include "bin_picking/math/bin_picking_math.hpp"
#include <cmath>
#include <cstring>
#include <iostream>

BinPickingMath::BinPickingMath(void) {

}

BinPickingMath::~BinPickingMath(void) {
}

//// Tool frame HTM
void BinPickingMath::transformToolFrame(const std::vector<double> &dPoseEndFrame, const std::vector<double> &x_measured, std::vector<double> &output) {
    std::vector<double> dPoseRPY(6);
    // dPoseEndFrame: [rad], measured_q: [rad], dPoseRPY [rad]
	// x,y,z 회전행렬
	Eigen::Matrix4d matRotx = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d matRoty = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d matRotz = Eigen::Matrix4d::Identity();

	// pose 2 HTM
	Eigen::Matrix4d matTrans = Eigen::Matrix4d::Identity();
	Eigen::VectorXd x_current(6);
	for(int i=0; i<6; i++) x_current[i] = x_measured[i];
	pose2HTM(x_current, matTrans);

    // 말단 좌표계 업데이트 (이 함수 내에서는 의미없음)
	Eigen::Matrix4d matTransB2E = Eigen::Matrix4d::Identity();
    matTransB2E = matTrans * matRotz;

    Eigen::VectorXd vecPosEF(4);
    for(int i = 0; i < 3; i++) vecPosEF[i] = dPoseEndFrame[i]; // 말단 기준 변위
    vecPosEF[3] = 1.0;

    // 말단 좌표 기준의 목표점을 RPY 기준으로 변환
    Eigen::VectorXd vecPosBF(4);
    vecPosBF = matTransB2E * vecPosEF;

    // Translation vector
    for(int i = 0; i < 3; i++) dPoseRPY[i] = vecPosBF[i];

    // Orientation 변환
    double Rx = dPoseEndFrame[3]; // [rad]
    double Ry = dPoseEndFrame[4]; // [rad]
    double Rz = dPoseEndFrame[5]; // [rad]
	matRotx.block<3,3>(0,0) = genRotM(Rx, 0); // 0: rx, 1:ry, 2:rz
	matRoty.block<3,3>(0,0) = genRotM(Ry, 1); // 0: rx, 1:ry, 2:rz
	matRotz.block<3,3>(0,0) = genRotM(Rz, 2); // 0: rx, 1:ry, 2:rz

    // 말단 좌표계를 기준으로 회전
    matTransB2E = matTrans * matRotz * matRoty * matRotx;

    // Orientation vector
    dPoseRPY[4] = atan2(-matTransB2E(2, 0), sqrt(matTransB2E(2, 1) * matTransB2E(2, 1) + matTransB2E(2, 2) * matTransB2E(2, 2)));

    if (fabs(cos(dPoseRPY[4])) > 0.0000001) {
        dPoseRPY[3] = atan2(matTransB2E(2, 1), matTransB2E(2, 2));
        dPoseRPY[5] = atan2(matTransB2E(1, 0), matTransB2E(0, 0));
    } else if (fabs(dPoseRPY[4] - M_PI / 2) < 0.01) {
        dPoseRPY[5] = atan2(matTransB2E(2, 1), matTransB2E(2, 2));
        dPoseRPY[3] = dPoseRPY[5] - atan2(matTransB2E(1, 2), matTransB2E(0, 2));
    } else {
        dPoseRPY[5] = atan2(matTransB2E(2, 1), matTransB2E(2, 2));
        dPoseRPY[3] = -dPoseRPY[5] + atan2(-matTransB2E(1, 2), -matTransB2E(0, 2));
    }

    for (int i = 0; i < 3; i++) {
        if (dPoseRPY[i + 3] > M_PI) {
            dPoseRPY[i + 3] = dPoseRPY[i + 3] - 2 * M_PI;
        } else if (dPoseRPY[i + 3] < -M_PI) {
            dPoseRPY[i + 3] = dPoseRPY[i + 3] + 2 * M_PI;
        }
    }

    output = dPoseRPY;
}

/** @brief Korean: RPY를 회전행렬로 변환한다.
 * @param[in] dRPY : Roll-Pitch-Yaw (Rx, Ry, Rz)
 * @param[out] rotM : 출력 회전행렬
 */
void BinPickingMath::RPY2Rot(const Eigen::Vector3d &dRPY, Eigen::Matrix3d &rotM) // [rad]
{
	double dRot[3][3] = { 0.0, };
	dRot[0][0] = cos(dRPY[2]) * cos(dRPY[1]);
	dRot[0][1] = cos(dRPY[2]) * sin(dRPY[1]) * sin(dRPY[0]) - sin(dRPY[2]) * cos(dRPY[0]);
	dRot[0][2] = cos(dRPY[2]) * sin(dRPY[1]) * cos(dRPY[0]) + sin(dRPY[2]) * sin(dRPY[0]);

	dRot[1][0] = sin(dRPY[2]) * cos(dRPY[1]);
	dRot[1][1] = sin(dRPY[2]) * sin(dRPY[1]) * sin(dRPY[0]) + cos(dRPY[2]) * cos(dRPY[0]);
	dRot[1][2] = sin(dRPY[2]) * sin(dRPY[1]) * cos(dRPY[0]) - cos(dRPY[2]) * sin(dRPY[0]);

	dRot[2][0] = -sin(dRPY[1]);
	dRot[2][1] = cos(dRPY[1]) * sin(dRPY[0]);
	dRot[2][2] = cos(dRPY[1]) * cos(dRPY[0]);

	for (int i = 0; i < 3; i++) { for (int j = 0; j < 3; j++) rotM(i, j) = dRot[i][j]; }
}

/** @brief Korean: RPY를 회전행렬로 변환한다.
 * @param[in] dRPY : Roll-Pitch-Yaw (Rx, Ry, Rz)
 * @param[out] rotM : 출력 회전행렬
 */
void BinPickingMath::RPY2Rot(const Eigen::Vector3f &dRPY, Eigen::Matrix3f &rotM) // [rad]
{
	double dRot[3][3] = { 0.0, };
	dRot[0][0] = cos(dRPY[2]) * cos(dRPY[1]);
	dRot[0][1] = cos(dRPY[2]) * sin(dRPY[1]) * sin(dRPY[0]) - sin(dRPY[2]) * cos(dRPY[0]);
	dRot[0][2] = cos(dRPY[2]) * sin(dRPY[1]) * cos(dRPY[0]) + sin(dRPY[2]) * sin(dRPY[0]);

	dRot[1][0] = sin(dRPY[2]) * cos(dRPY[1]);
	dRot[1][1] = sin(dRPY[2]) * sin(dRPY[1]) * sin(dRPY[0]) + cos(dRPY[2]) * cos(dRPY[0]);
	dRot[1][2] = sin(dRPY[2]) * sin(dRPY[1]) * cos(dRPY[0]) - cos(dRPY[2]) * sin(dRPY[0]);

	dRot[2][0] = -sin(dRPY[1]);
	dRot[2][1] = cos(dRPY[1]) * sin(dRPY[0]);
	dRot[2][2] = cos(dRPY[1]) * cos(dRPY[0]);

	for (int i = 0; i < 3; i++) { for (int j = 0; j < 3; j++) rotM(i, j) = dRot[i][j]; }
}

/** @brief Korean: 자세를 변환행렬로 형식으로 변환한다.
 * @param[in] x : 작업공간의 자세 (x, y, z, Rx, Ry, Rz)
 * @param[out] T_out : 변환행렬 형식의 자세
 */
void BinPickingMath::pose2HTM(const Eigen::VectorXd &x, Eigen::Matrix4d &T_out)
{
	T_out = Eigen::Matrix4d::Identity();
	Eigen::Matrix3d rotM_tmp = Eigen::Matrix3d::Identity();
	T_out(0, 3) = x[0];
	T_out(1, 3) = x[1];
	T_out(2, 3) = x[2];
	RPY2Rot(x.segment(3, 3), rotM_tmp);
	T_out.block<3, 3>(0, 0) = rotM_tmp;
}

/** @brief Korean: 자세를 변환행렬로 형식으로 변환한다.
 * @param[in] x : 작업공간의 자세 (x, y, z, Rx, Ry, Rz)
 * @param[out] T_out : 변환행렬 형식의 자세
 */
void BinPickingMath::pose2HTM(const Eigen::VectorXf &x, Eigen::Matrix4f &T_out)
{
	T_out = Eigen::Matrix4f::Identity();
	Eigen::Matrix3f rotM_tmp = Eigen::Matrix3f::Identity();
	T_out(0, 3) = x[0];
	T_out(1, 3) = x[1];
	T_out(2, 3) = x[2];
	RPY2Rot(x.segment(3, 3), rotM_tmp);
	T_out.block<3, 3>(0, 0) = rotM_tmp;
}


/** @brief Korean: 입력된 postion(x, y, z)을 변환행렬 형식으로 반환한다.
 * @param[in] T_in : 입력 변환행렬 [m]
 * @param[out] vec_out : 출력 자세 (x,y,z,R,P,Y) [m], [rad]
 */
std::vector<double> BinPickingMath::htm2pose(const Eigen::Matrix4f &T_in)
{
    std::vector<double> vec_out(6);
	Eigen::Matrix3f rotM = T_in.block<3, 3>(0, 0);
	Eigen::Vector3f RPY = rotm2ZYX(rotM);

	vec_out[0] = T_in(0, 3); // x
	vec_out[1] = T_in(1, 3); // y
	vec_out[2] = T_in(2, 3); // z
	vec_out[3] = RPY[0]; // Rx
	vec_out[4] = RPY[1]; // Ry
	vec_out[5] = RPY[2]; // Rz

    return vec_out;
}

/** @brief Korean: 입력된 회전행렬을 ZYX euler angle (roll, pitch, yaw)로 반환한다.
 * @param[in] RotMat : 입력 회전행렬
 */
Eigen::Vector3f BinPickingMath::rotm2ZYX(Eigen::Matrix3f &RotMat)
{
	// Rotation matrix to ZYX euler angle (roll, pitch, yaw)
	Eigen::Vector3f RPY;
	double Roll, Pitch, Yaw;
	double r11 = RotMat(0, 0);
	double r12 = RotMat(0, 1);
	double r13 = RotMat(0, 2);

	double r21 = RotMat(1, 0);
	double r22 = RotMat(1, 1);
	double r23 = RotMat(1, 2);

	double r31 = RotMat(2, 0);
	double r32 = RotMat(2, 1);
	double r33 = RotMat(2, 2);

	if (r11*r11 + r21*r21 != 0)
	{
		Roll = atan2(r32, r33);
		Pitch = atan2(-r31, sqrt(r32*r32 + r33*r33));
		Yaw = atan2(r21, r11);
	}
	else
	{
		Roll = 0;
		Pitch = 0;
		Yaw = 0;
	}
	RPY[0] = Roll; // Rx
	RPY[1] = Pitch; // Ry
	RPY[2] = Yaw; // Rz
	return RPY;
}


/** @brief Korean: 입력된 회전축, 회전각도를 이용하여 회전 및 변환행렬을 계산한다.
 * @param[in] angle : 회전각도, [rad]
 * @param[in] direction : 회전 축 설정 (0: Rx, 1: Ry, 2: Rz)
 * @return rotM : 회전축을 중심으로 회전각도만큼 회전한 회전행렬
 */
Eigen::Matrix3d BinPickingMath::genRotM(double angle, int direction)
{
	Eigen::Matrix3d rotM = Eigen::Matrix3d::Identity();
	double angle_rad = angle;
	switch (direction)
	{
	case 0: // in x direction
		rotM(1, 1) = cos(angle_rad);	rotM(1, 2) = -sin(angle_rad);
		rotM(2, 1) = sin(angle_rad);	rotM(2, 2) = cos(angle_rad);
		break;

	case 1: // in y direction
		rotM(0, 0) = cos(angle_rad);	rotM(0, 2) = sin(angle_rad);
		rotM(2, 0) = -sin(angle_rad);	rotM(2, 2) = cos(angle_rad);
		break;

	case 2: // in z direction
		rotM(0, 0) = cos(angle_rad);	rotM(0, 1) = -sin(angle_rad);
		rotM(1, 0) = sin(angle_rad);	rotM(1, 1) = cos(angle_rad);
		break;
	}
	return rotM;
}

/** @brief Korean: 입력된 회전축, 회전각도를 이용하여 회전 및 변환행렬을 계산한다.
 * @param[in] angle : 회전각도, [rad]
 * @param[in] direction : 회전 축 설정 (0: Rx, 1: Ry, 2: Rz)
 * @return rotM : 회전축을 중심으로 회전각도만큼 회전한 회전행렬
 */
Eigen::Matrix3f BinPickingMath::genRotMf(double angle, int direction)
{
	Eigen::Matrix3f rotM = Eigen::Matrix3f::Identity();
	double angle_rad = angle;
	switch (direction)
	{
	case 0: // in x direction
		rotM(1, 1) = cos(angle_rad);	rotM(1, 2) = -sin(angle_rad);
		rotM(2, 1) = sin(angle_rad);	rotM(2, 2) = cos(angle_rad);
		break;

	case 1: // in y direction
		rotM(0, 0) = cos(angle_rad);	rotM(0, 2) = sin(angle_rad);
		rotM(2, 0) = -sin(angle_rad);	rotM(2, 2) = cos(angle_rad);
		break;

	case 2: // in z direction
		rotM(0, 0) = cos(angle_rad);	rotM(0, 1) = -sin(angle_rad);
		rotM(1, 0) = sin(angle_rad);	rotM(1, 1) = cos(angle_rad);
		break;
	}
	return rotM;
}

Eigen::Vector3f BinPickingMath::rotationMatrixToZYZEulerAngles(const Eigen::Matrix3f& R) {
    Eigen::Vector3f euler_angles;

    double z1, y, z2;

    // Y축 회전 각도 (theta)
    y = acos(R(2, 2)); 

    // 특이점 검사 (y ≈ 0 or y ≈ 180도)
    if (fabs(y) < 1e-6) { 
        // y = 0도 -> Z1과 Z2의 합만 정의됨
        z1 = 0;
        z2 = atan2(R(1, 0), R(0, 0)); 
    } else if (fabs(y - M_PI) < 1e-6) { 
        // y = 180도 -> Z1과 Z2의 차만 정의됨
        z1 = 0;
        z2 = atan2(-R(1, 0), -R(0, 0)); 
    } else {
        // 일반적인 경우
        z1 = atan2(R(1, 2), R(0, 2));
        z2 = atan2(R(2, 1), -R(2, 0));
    }
	euler_angles(0) = z1;
	euler_angles(1) = y;
	euler_angles(2) = z2;

    // Convert to degrees
    euler_angles *= 180.0 / M_PI;
    return euler_angles;
}


/** @brief Korean: 입력된 회전행렬을 ZYX euler angle (roll, pitch, yaw)로 반환한다.
 * @param[in] R : 입력 회전행렬
 */
Eigen::Vector3f BinPickingMath::rotationMatrixToZYXEulerAngles(const Eigen::Matrix3f &R)
{
	// Rotation matrix to ZYX euler angle (roll, pitch, yaw)
	Eigen::Vector3f euler_angles;
	double Roll, Pitch, Yaw;
	double r11 = R(0, 0);
	double r12 = R(0, 1);
	double r13 = R(0, 2);

	double r21 = R(1, 0);
	double r22 = R(1, 1);
	double r23 = R(1, 2);

	double r31 = R(2, 0);
	double r32 = R(2, 1);
	double r33 = R(2, 2);

	if (r11*r11 + r21*r21 != 0)
	{
		Roll = atan2(r32, r33);
		Pitch = atan2(-r31, sqrt(r32*r32 + r33*r33));
		Yaw = atan2(r21, r11);
	}
	else
	{
		Roll = 0;
		Pitch = 0;
		Yaw = 0;
	}
	euler_angles[0] = Roll; // Rx
	euler_angles[1] = Pitch; // Ry
	euler_angles[2] = Yaw; // Rz

    // Convert to degrees
    euler_angles *= 180.0 / M_PI;
    return euler_angles;
}

Eigen::Matrix3f BinPickingMath::ZYZEulerAnglesToRotationMatrix(const Eigen::Vector3f& euler_zyz) {

	Eigen::Matrix3f Rz_1 = genRotMf(euler_zyz[0] * (M_PI / 180.0), 2); // 0: rx, 1:ry, 2:rz
	Eigen::Matrix3f Ry   = genRotMf(euler_zyz[1] * (M_PI / 180.0), 1); // 0: rx, 1:ry, 2:rz
	Eigen::Matrix3f Rz_2 = genRotMf(euler_zyz[2] * (M_PI / 180.0), 2); // 0: rx, 1:ry, 2:rz
	Eigen::Matrix3f R = Rz_1*Ry*Rz_2;

	return R;
}


Eigen::Matrix3f BinPickingMath::ZYXEulerAnglesToRotationMatrix(const Eigen::Vector3f& euler_zyx) {
	Eigen::Matrix3f Rx = genRotMf(euler_zyx[0] * (M_PI / 180.0), 0); // 0: rx, 1:ry, 2:rz
	Eigen::Matrix3f Ry = genRotMf(euler_zyx[1] * (M_PI / 180.0), 1); // 0: rx, 1:ry, 2:rz
	Eigen::Matrix3f Rz = genRotMf(euler_zyx[2] * (M_PI / 180.0), 2); // 0: rx, 1:ry, 2:rz
	Eigen::Matrix3f R = Rz*Ry*Rx;

	return R;
}



Eigen::Vector3f BinPickingMath::ZYZEulerAnglesToZYXEulerAngles(const Eigen::Vector3f& euler_zyz) {

	Eigen::Vector3f euler_angles_zyx;
	// ROS_LOG_WARN("*******************************************");
	// ROS_LOG_WARN("[%s]DRFL_CONTROL MODE! - ZYZ to RotM", __func__);
	// // Eigen::Vector3f zyz_in(pos[3], pos[4], pos[5]); // [deg]
	// Eigen::Matrix3f R_zyz = ZYZEulerAnglesToRotationMatrix(euler_zyz); // [deg]
	// Eigen::Vector3f euler_angles_zyz = rotationMatrixToZYZEulerAngles(R_zyz); // [deg]
	// std::cout << "[R_zyz]:\n" << R_zyz << std::endl;
	// std::cout << "ZYZ Euler Angles (degrees):\n" << euler_angles_zyz.transpose() << std::endl;
	// ROS_LOG_WARN("*******************************************");

	// ROS_LOG_WARN("*******************************************");
	// ROS_LOG_WARN("[%s]DRFL_CONTROL MODE! - RotM to ZYX", __func__);

	// Eigen::Vector3f euler_angles_zyx = rotationMatrixToZYXEulerAngles(R_zyz); // [deg]
	// Eigen::Matrix3f R_zyx = ZYXEulerAnglesToRotationMatrix(euler_angles_zyx); // [deg]

	// std::cout << "[R_zyx]:\n" << R_zyx << std::endl;
	// std::cout << "ZYX Euler Angles (degrees):\n" << euler_angles_zyx.transpose() << std::endl;
	// ROS_LOG_WARN("*******************************************");
	// ROS_LOG_WARN("*******************************************\n\n");

	return euler_angles_zyx;
}