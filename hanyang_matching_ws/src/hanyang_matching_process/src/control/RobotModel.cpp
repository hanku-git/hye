/**
 * @file RobotModel.cpp
 * @brief 로봇 제어를 위해 필요한 각종 연산을 위한 구현파일
 */

#include "control/RobotModel.hpp"
#include <cmath>
#include <cstring>
#include <iostream>

CRobotModel::CRobotModel(void) {
    m_dTcp.resize(6);
    SetTcp({0, 0, 0, 0, 0, 0});
    m_TCP = Eigen::Matrix4f::Identity();
    m_TCP_inv = Eigen::Matrix4f::Identity();
}

CRobotModel::~CRobotModel(void) {
}

/** @brief Korean: 로봇의 DH 파라미터를 입력한다.
 * @param[in] dDH : 입력 DH 파라미터
 */
void CRobotModel::SetDH(double dDH[6][4]) {
    for (std::size_t i = 0; i < 6; i++) {
        for (std::size_t  j = 0; j < 4; j++) {
            m_DH[i][j] = dDH[i][j];
        }
    }
}

/** @brief Korean: 로봇의 TCP 정보를 입력한다.
 * @param[in] tcp : 로봇의 TCP 정보, [m], [rad]
 */
void CRobotModel::SetTcp(std::vector<double> tcp) {
    m_dTcp = tcp;

    double rot[3][3] = {{0.0,},};
    double rpyofTcp[3] = {m_dTcp[3], m_dTcp[4], m_dTcp[5]};

    RPY2Rot(rpyofTcp, rot);
    for (std::size_t i = 0; i < 3; i++) {
        for (std::size_t  j = 0; j < 3; j++) {
            m_TCP(i,j) = rot[i][j];
        }
        m_TCP(i,3) = m_dTcp[i];
    }
    m_TCP_inv = m_TCP.inverse();
}

/** @brief Korean: 로봇의 TCP 정보를 입력하여 좌표변환 행렬을 산출한다.
 * @param[in] tcp : 로봇의 TCP 정보, [m], [rad]
 * @param[out] HTM_out : 좌표변환 행렬
 */
void CRobotModel::GenTcpHTM(std::vector<double> tcp, double HTM_out[4][4]) {
    double htm[4][4] = {{0.0,},};
    double rot[3][3] = {{0.0,},};
    double rpyofTcp[3] = {tcp[3], tcp[4], tcp[5]};

    RPY2Rot(rpyofTcp, rot);
    for (std::size_t i = 0; i < 3; i++) {
        for (std::size_t  j = 0; j < 3; j++) {
            HTM_out[i][j] = rot[i][j];
        }
        HTM_out[i][3] = tcp[i];
    }
}

/** @brief Korean: 로봇의 TCP 정보를 입력하여 좌표변환 행렬의 역행렬을 산출한다.
 * @param[in] tcp : 로봇의 TCP 정보, [m], [rad]
 * @param[out] HTM_out : 좌표변환 행렬
 */
void CRobotModel::GenTcpInverseHTM(std::vector<double> tcp, double HTM_out[4][4]) {
    double htm[4][4] = {{0.0,},};
    double rot[3][3] = {{0.0,},};
    double rpyofTcp[3] = {tcp[3], tcp[4], tcp[5]};

    RPY2Rot(rpyofTcp, rot);
	Eigen::Matrix4f htm_tmp;
	htm_tmp = Eigen::Matrix4f::Identity();
    for (std::size_t i = 0; i < 3; i++) {
        for (std::size_t  j = 0; j < 3; j++) {
            htm_tmp(i,j) = rot[i][j];
        }
        htm_tmp(i,3) = tcp[i];
    }

    Eigen::Matrix4f htm_tmp_inv = htm_tmp.inverse();
    for (std::size_t i = 0; i < 4; i++) {
        for (std::size_t  j = 0; j < 4; j++) {
            HTM_out[i][j] = htm_tmp_inv(i,j);
        }
    }
}

/** @brief Korean: Homogeneous transformation matrix을 계산한다.
 * @param[in] n : 로봇의 자유도
 * @param[in] q : 관절각도 [rad]
 * @return T_out : 변환행렬을 반환
 */
Eigen::Matrix4f CRobotModel::TransformationMatrix(int n, double q) // [rad]
{
	Eigen::Matrix4f T_out = Eigen::Matrix4f::Identity();
	double dSinQ = sin(q);
	double dCosQ = cos(q);
	double dSinAlpha = sin(m_DH[n][0]);
	double dCosAlpha = cos(m_DH[n][0]);
	double a = m_DH[n][1];
	double d = m_DH[n][2];
	double temp[4][4] = { { 0.0, }, };

	temp[0][0] = dCosQ;
	temp[0][1] = -dSinQ * dCosAlpha;
	temp[0][2] = dSinQ * dSinAlpha;
	temp[0][3] = a * dCosQ;

	temp[1][0] = dSinQ;
	temp[1][1] = dCosQ * dCosAlpha;
	temp[1][2] = -dCosQ * dSinAlpha;
	temp[1][3] = a * dSinQ;

	temp[2][0] = 0;
	temp[2][1] = dSinAlpha;
	temp[2][2] = dCosAlpha;
	temp[2][3] = d;

	temp[3][0] = 0;
	temp[3][1] = 0;
	temp[3][2] = 0;
	temp[3][3] = 1;

	for (int i = 0; i < 4; i++) 
	{
		for (int j = 0; j < 4; j++) { T_out(i,j) = temp[i][j]; }
	}
	return T_out;
}

/** @brief Korean: 순기구학(forward kinematics)을 수행한다.
 * @param[in] q : 로봇의 관절 각도 [rad]
 * @return output_pose : 순기구학의 결과로 로봇의 자세 정보를 반환. [m], [rad]
 */
std::vector<double> CRobotModel::forwardKinematics(const std::vector<double> q)
{
	Eigen::Matrix4f T_kine = Eigen::Matrix4f::Identity();
	for (int i = 0; i < 6; i++)
	{
        // m_DH[i][0]: alpha, m_DH[i][1]: a, m_DH[i][2]: d, m_DH[i][3]: theta
		Eigen::Matrix4f T_tmp = Eigen::Matrix4f::Identity();
		double q_des = q[i];
		T_tmp(0, 0) = cos(q_des);
		T_tmp(0, 1) = -sin(q_des)*cos(m_DH[i][0]);
		T_tmp(0, 2) = sin(q_des)*sin(m_DH[i][0]);
		T_tmp(0, 3) = m_DH[i][1] * cos(q_des);
		T_tmp(1, 0) = sin(q_des);
		T_tmp(1, 1) = cos(q_des)*cos(m_DH[i][0]);
		T_tmp(1, 2) = -cos(q_des)*sin(m_DH[i][0]);
		T_tmp(1, 3) = m_DH[i][1] * sin(q_des);
		T_tmp(2, 0) = 0.0;
		T_tmp(2, 1) = sin(m_DH[i][0]);
		T_tmp(2, 2) = cos(m_DH[i][0]);
		T_tmp(2, 3) = m_DH[i][2];

		T_kine = T_kine*T_tmp;
	}

    // TCP 적용
    Eigen::Matrix4f T_out = T_kine*m_TCP;
    Eigen::Matrix3f rotM = T_out.block<3,3>(0,0);

    std::vector<double> output_pose(6);

    output_pose[0] = T_out(0,3);
    output_pose[1] = T_out(1,3);
    output_pose[2] = T_out(2,3);

    double r11 = rotM(0,0);
    double r12 = rotM(0,1);
    double r13 = rotM(0,2);

    double r21 = rotM(1,0);
    double r22 = rotM(1,1);
    double r23 = rotM(1,2);

    double r31 = rotM(2,0);
    double r32 = rotM(2,1);
    double r33 = rotM(2,2);

    // Roll-Pitch-Yaw - Euler ZYX
    output_pose[3] = atan2(r32, r33);                          // Roll
    output_pose[4] = atan2(-r31, sqrt(r32 * r32 + r33 * r33)); // Pitch
    output_pose[5] = atan2(r21, r11);                          // Yaw

    return output_pose; // [m], [rad]
}

/** @brief Korean: 순기구학(forward kinematics)을 수행한다.
 * @param[in] q : 로봇의 관절 각도 [rad]
 * @return output_pose : 순기구학의 결과로 로봇의 자세 정보를 반환. [m], [rad]
 */
std::vector<double> CRobotModel::forwardKinematicsWithJointCenter(const std::vector<double> q, std::vector<Eigen::Vector3f> &joint_center)
{
	Eigen::Matrix4f T_kine = Eigen::Matrix4f::Identity();
    joint_center.clear();
    Eigen::Vector3f j_pt;
	for (int i = 0; i < 6; i++)
	{
        // m_DH[i][0]: alpha, m_DH[i][1]: a, m_DH[i][2]: d, m_DH[i][3]: theta
		Eigen::Matrix4f T_tmp = Eigen::Matrix4f::Identity();
		double q_des = q[i];
		T_tmp(0, 0) = cos(q_des);
		T_tmp(0, 1) = -sin(q_des)*cos(m_DH[i][0]);
		T_tmp(0, 2) = sin(q_des)*sin(m_DH[i][0]);
		T_tmp(0, 3) = m_DH[i][1] * cos(q_des);
		T_tmp(1, 0) = sin(q_des);
		T_tmp(1, 1) = cos(q_des)*cos(m_DH[i][0]);
		T_tmp(1, 2) = -cos(q_des)*sin(m_DH[i][0]);
		T_tmp(1, 3) = m_DH[i][1] * sin(q_des);
		T_tmp(2, 0) = 0.0;
		T_tmp(2, 1) = sin(m_DH[i][0]);
		T_tmp(2, 2) = cos(m_DH[i][0]);
		T_tmp(2, 3) = m_DH[i][2];

		T_kine = T_kine*T_tmp;

        // Joint center
        j_pt[0] = T_kine(0,3);
        j_pt[1] = T_kine(1,3);
        j_pt[2] = T_kine(2,3);
        joint_center.push_back(j_pt);
	}

    // TCP 적용
    Eigen::Matrix4f T_out = T_kine*m_TCP;
    Eigen::Matrix3f rotM = T_out.block<3,3>(0,0);

    // Joint center
    j_pt[0] = T_out(0,3);
    j_pt[1] = T_out(1,3);
    j_pt[2] = T_out(2,3);
    joint_center.push_back(j_pt);

    std::vector<double> output_pose(6);

    output_pose[0] = T_out(0,3);
    output_pose[1] = T_out(1,3);
    output_pose[2] = T_out(2,3);

    double r11 = rotM(0,0);
    double r12 = rotM(0,1);
    double r13 = rotM(0,2);

    double r21 = rotM(1,0);
    double r22 = rotM(1,1);
    double r23 = rotM(1,2);

    double r31 = rotM(2,0);
    double r32 = rotM(2,1);
    double r33 = rotM(2,2);

    // Roll-Pitch-Yaw - Euler ZYX
    output_pose[3] = atan2(r32, r33);                          // Roll
    output_pose[4] = atan2(-r31, sqrt(r32 * r32 + r33 * r33)); // Pitch
    output_pose[5] = atan2(r21, r11);                          // Yaw

    return output_pose; // [m], [rad]
}

/** @brief Korean: 순기구학(forward kinematics)을 수행한다.
 * @param[in] q : 로봇의 관절 각도 [rad]
 * @return T_out : 순기구학의 결과로 로봇의 변환행렬을 반환
 */
Eigen::Matrix4f CRobotModel::forwardKinematics_q2Tr(const Eigen::VectorXf &q)
{
	Eigen::Matrix4f T_kine = Eigen::Matrix4f::Identity();
	for (int i = 0; i < 6; i++)
	{
        // m_DH[i][0]: alpha, m_DH[i][1]: a, m_DH[i][2]: d, m_DH[i][3]: theta
		Eigen::Matrix4f T_tmp = Eigen::Matrix4f::Identity();
		double q_des = q[i];
		T_tmp(0, 0) = cos(q_des);
		T_tmp(0, 1) = -sin(q_des)*cos(m_DH[i][0]);
		T_tmp(0, 2) = sin(q_des)*sin(m_DH[i][0]);
		T_tmp(0, 3) = m_DH[i][1] * cos(q_des);
		T_tmp(1, 0) = sin(q_des);
		T_tmp(1, 1) = cos(q_des)*cos(m_DH[i][0]);
		T_tmp(1, 2) = -cos(q_des)*sin(m_DH[i][0]);
		T_tmp(1, 3) = m_DH[i][1] * sin(q_des);
		T_tmp(2, 0) = 0.0;
		T_tmp(2, 1) = sin(m_DH[i][0]);
		T_tmp(2, 2) = cos(m_DH[i][0]);
		T_tmp(2, 3) = m_DH[i][2];

		T_kine = T_kine*T_tmp;
	}

    // TCP 적용
    Eigen::Matrix4f T_out = T_kine*m_TCP;
    return T_out;
}

/** @brief Korean: 입력된 작업공간(task space)의 자세를 UR5 로봇의 역기구학(closed-form )을 이용하여 관절공간의 자세로 변환한다.
 * @param[in] q_past : 이전 관절 각도 [deg]
 * @param[in] x_current : 목표 자세 [m], [rad]
 * @param[out] q_current : 역기구학을 통해 계산된 다음 관절 각도 [deg]
 */
int CRobotModel::inverseClosedForm(const Eigen::VectorXf &q_past, Eigen::VectorXf &x_current, Eigen::VectorXf &q_current)
{
	double q_past_radian[ROBOT_DOF] = { 0.0, };
	double q_plus[ROBOT_DOF] = { 0.0, };     
	double q_minus[ROBOT_DOF] = { 0.0, };    
	double q_find[8][ROBOT_DOF] = { { 0.0, }, };
	double q_sum[8] = { 0.0, };
	int reach_out[8] = { 0, };
	int reach_in[8] = { 0, };
	int number = 0;

	for (int i = 0; i < ROBOT_DOF; i++) { q_past_radian[i] = q_past[i] * (M_PI / 180.0); }
	Eigen::Matrix4f T06;
	pose2HTM(x_current, T06);

    //// TCP 적용
    Eigen::Matrix4f T_TCP_inv = m_TCP.inverse();
    T06 = T06*T_TCP_inv;
    ////////

	// q1 계산
	double vector_p05[3] = { 0.0, };
	for (int i = 0; i < 3; i++) { vector_p05[i] = T06(i, 3) - m_DH[5][2] * T06(i, 2); }
	double theta1_constraint = checkFloatingError(m_DH[3][2], sqrt(vector_p05[0] * vector_p05[0] + vector_p05[1] * vector_p05[1]));
	double theta1_check = 0.0;
	if (theta1_constraint > 1.0) 
	{
		theta1_check = theta1_constraint;
		theta1_constraint = 1.0;
	}
	else if (theta1_constraint < -1.0) 
	{
		theta1_check = theta1_constraint;
		theta1_constraint = -1.0;
	}
	q_plus[0] = atan2(vector_p05[1], vector_p05[0]) + acos(theta1_constraint) + M_PI / 2;
	q_minus[0] = atan2(vector_p05[1], vector_p05[0]) - acos(theta1_constraint) + M_PI / 2;

	for (int i = 0; i < 2; i++)
	{
		double q_temp[ROBOT_DOF] = { 0.0, };
		if (i == 0) { q_temp[0] = q_plus[0]; }
		else { q_temp[0] = q_minus[0]; }

		// q5 계산
		Eigen::Matrix4f T01 = TransformationMatrix(0, q_temp[0]);
		Eigen::Matrix4f T10 = T01.inverse();
		Eigen::Matrix4f T16 = T10 * T06;
		double p16z = T16(2,3);
		double theta5_constraint = (p16z - m_DH[3][2]) / (m_DH[5][2]);

		if (theta5_constraint > 1.0) { theta5_constraint = 1.0; }
		else if (theta5_constraint < -1.0) { theta5_constraint = -1.0; }

		q_plus[4] = acos(theta5_constraint);
		q_minus[4] = -acos(theta5_constraint);

		for (int j = 0; j < 2; j++) 
		{
			if (j == 0) { q_temp[4] = q_plus[4]; }
			else { q_temp[4] = q_minus[4]; }

			Eigen::Matrix4f T61 = T16.inverse();
			if (sin(q_temp[4]) > 0.0000001) { q_temp[5] = atan2(-T61(1,2), T61(0,2)); }
			else if (sin(q_temp[4]) < -0.0000001) {	q_temp[5] = atan2(T61(1,2), -T61(0,2)); }
			else { q_temp[5] = q_past_radian[5]; }

			Eigen::Matrix4f T45 = TransformationMatrix(4, q_temp[4]);
			Eigen::Matrix4f T56 = TransformationMatrix(5, q_temp[5]);
			Eigen::Matrix4f T46 = T45 * T56;
			Eigen::Matrix4f T64 = T46.inverse();
			Eigen::Matrix4f T04 = T06 * T64;
			Eigen::Matrix4f T05 = T04 * T45;
			Eigen::Matrix4f T14 = T10 * T04;

			double p13[3] = { 0.0, };
			for (int i = 0; i < 3; i++) 
			{
				p13[i] = T14(i, 3) - m_DH[3][2] * T14(i, 1);
			}

			double theta3_check = 0.0;
			double theta3_constraint = (p13[0] * p13[0] + p13[1] * p13[1] - m_DH[1][1] * m_DH[1][1] - m_DH[2][1] * m_DH[2][1]) / (2 * m_DH[1][1] * m_DH[2][1]);
			if (theta3_constraint > 1.0) 
			{
				theta3_check = theta3_constraint;
				theta3_constraint = 1.0;
			}
			else if (theta3_constraint < -1.0) 
			{
				theta3_check = theta3_constraint;
				theta3_constraint = -1.0;
			}

			q_plus[2] = acos(theta3_constraint);
			q_minus[2] = -acos(theta3_constraint);

			for (int k = 0; k < 2; k++) 
			{
				int solution_index = 2 * 2 * i + 2 * j + k;
				if (k == 0) { q_temp[2] = q_plus[2]; }
				else { q_temp[2] = q_minus[2]; }

				double theta2_constraint = checkFloatingError(m_DH[2][1] * sin(q_temp[2]), (sqrt(p13[0] * p13[0] + p13[1] * p13[1])));
				q_temp[1] = -atan2(p13[1], -p13[0]) + asin(theta2_constraint);
				
				Eigen::Matrix4f T12 = TransformationMatrix(1, q_temp[1]);
				Eigen::Matrix4f T23 = TransformationMatrix(2, q_temp[2]);
				Eigen::Matrix4f T13 = T12 * T23;
				Eigen::Matrix4f T31 = T13.inverse();
				Eigen::Matrix4f T34 = T31 * T14;

				q_temp[3] = atan2(T34(1, 0), T34(0, 0));

				for (int n = 0; n < 6; n++) 
				{
					q_find[2 * 2 * i + 2 * j + k][n] = q_temp[n];
					double q_sum_temp = 0.0;
					q_sum_temp = fabs(q_find[2 * 2 * i + 2 * j + k][n] - q_past_radian[n]);
					if (q_sum_temp > 180.0 * (M_PI/180.0)) { q_sum_temp = q_sum_temp - 180.0 * (M_PI/180.0); }
					else if (q_sum_temp < -180.0 * (M_PI/180.0)) { q_sum_temp = q_sum_temp + 180.0 * (M_PI/180.0); }
					q_sum[2 * 2 * i + 2 * j + k] += q_sum_temp;
				}
				if (fabs(theta1_check) > 1.0) {	reach_in[number] = 1; }
				if (fabs(theta3_check) > 1.0) { reach_out[number] = 3; }
				number++;
			}
		}
	}

	int q_number = 0;
	Eigen::VectorXf q_find_out(6);
	double q_sum_temp = 2 * M_PI * 6;
	for (int i = 0; i < 8; i++)
	{
		if (q_sum_temp > q_sum[i])
		{
			q_sum_temp = q_sum[i];
			q_number = i;
		}
	}

	Eigen::VectorXf q_dis_tmp(6);
	double norm_min = 0.0;
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			if (j > 2)
			{
				if (q_find[i][j] - q_past_radian[j] > M_PI) { q_find[i][j] = q_find[i][j] - 2 * M_PI; }
				else if (q_find[i][j] - q_past_radian[j] < -M_PI) { q_find[i][j] = q_find[i][j] + 2 * M_PI;	}
			}
			q_dis_tmp[j] = q_find[i][j] - q_past_radian[j];
		}

		if (i == 0) { norm_min = q_dis_tmp.norm(); }
		if (norm_min > q_dis_tmp.norm())
		{
			norm_min = q_dis_tmp.norm();
			q_number = i;
		}
	}

    if(val_test_i != q_number)
    {
     ROS_LOG_INFO("q_number: %i", q_number);
     ROS_LOG_INFO("q_past: %f %f %f %f %f %f", 5, q_past_radian[0]*RADTODEG, q_past_radian[1]*RADTODEG, q_past_radian[2]*RADTODEG, q_past_radian[3]*RADTODEG, q_past_radian[4]*RADTODEG, q_past_radian[5]*RADTODEG);
     ROS_LOG_INFO("q_find(%i): %f %f %f %f %f %f", 5, q_find[5][0]*RADTODEG, q_find[5][1]*RADTODEG, q_find[5][2]*RADTODEG, q_find[5][3]*RADTODEG, q_find[5][4]*RADTODEG, q_find[5][5]*RADTODEG);
    }
    val_test_i = q_number;


	for (int i = 0; i < 6; i++) { q_current[i] = q_find[q_number][i] * (180.0/M_PI); }
	if (reach_in[q_number] == 0 && reach_out[q_number] == 0) { return 0; } // singular error가 없을 경우
	else if ((reach_in[q_number] != 0) && (reach_out[q_number] == 0)) {	return 1; }
	else if ((reach_out[q_number] != 0) && (reach_in[q_number] == 0)) { return 2; }
	else { return 3; }
}


/** @brief Korean: 입력된 작업공간(task space)의 자세를 UR5 로봇의 역기구학(closed-form )을 이용하여 관절공간의 자세로 변환한다.
 * @param[in] q_past : 이전 관절 각도 [deg]
 * @param[in] x_target : 목표 자세 [m], [rad]
 * @param[out] q_target : 역기구학을 통해 계산된 다음 관절 각도 [deg]
 */
int CRobotModel::inverseNumericalForm(double kInverseKineLambda, int kInverseKineIterLimit, double kInverseKineTol, const Eigen::VectorXf &q_past, const Eigen::VectorXf &x_target, Eigen::VectorXf &q_target)
{
    int iter = 0;
    double lambda = kInverseKineLambda;
    Eigen::VectorXf return_q(ROBOT_DOF);
    Eigen::VectorXf q_current = q_past;
	for (int i = 0; i < ROBOT_DOF; i++) { q_current[i] = q_current[i] * DEGTORAD; }
    Eigen::VectorXf pose_target = x_target;

    Eigen::MatrixXf jacobian(6, 6);
    Eigen::MatrixXf JTJ(6, 6);

    // Target
	Eigen::Matrix4f tr_target;
	pose2HTM(pose_target, tr_target);
    //// TCP 적용
    tr_target = tr_target*m_TCP_inv;

    ////////

    // Forward kinematics - Current
    Eigen::VectorXf q(6);
	for (int j = 0; j < 6; j++) { q[j] = q_current[j]; } // [rad]
    Eigen::Matrix4f tr_current = forwardKinematics_q2Tr(q);   // input: [rad], output: position: [m], orientation: RPY[rad]
    //// TCP 적용
    tr_current = tr_current*m_TCP_inv;

    Eigen::VectorXf err = tr2delta(tr_current, tr_target);
    double err_norm_prev = err.norm();

    if (err_norm_prev <= kInverseKineTol) {
        for (int i = 0; i < ROBOT_DOF; i++) return_q[i] = q[i] * RADTODEG;
        q_target = return_q;
        return 0;
    }

    Eigen::MatrixXf lambda_I = lambda*Eigen::MatrixXf::Identity(6, 6);
    if (ROBOT_DOF != 6) {
        for (int i = 0; i < ROBOT_DOF; i++) return_q[i] = q_current[i] * RADTODEG;
        q_target = return_q;
        return 0;
    }
    else {
        while (true) {
            calJacobianGeoRad(q, jacobian); // [rad]
            JTJ = jacobian.transpose() * jacobian;

            if (1) 
            {
                // Levenberg metod
                // for (int i = 0; i < 6; i++) {
                //     JTJ.element_[6 * i + i] += lambda;
                // }
                JTJ = JTJ + lambda_I;
            } else  
            {
                // // Levenberg - Marquadt method
                // for (int i = 0; i < 6; i++) {
                //     JTJ.element_[6 * i + i] += lambda * JTJ.element_[6 * i + i];
                // }
            }

            Eigen::VectorXf dq = JTJ.inverse() * jacobian.transpose() * err;
            if (ROBOT_DOF == 6) {
                q = q + dq; // [rad]
            }
            Eigen::VectorXf q_deg = RADTODEG*q;
            // else if (ROBOT_DOF == 7) {
            //     for (int i = 0; i < ROBOT_DOF; i++) {
            //         if (i == 2) {
            //             q.element_[i] = q_current[i];
            //         }
            //         else if (i > 2) {
            //             q.element_[i] += dq.element_[i - 1];
            //         }
            //         else {
            //             q.element_[i] += dq.element_[i];
            //         }
            //     }
            // }

            tr_current = forwardKinematics_q2Tr(q);   // input: [rad], output: position: [m], orientation: RPY[rad]
            //// TCP 적용
            tr_current = tr_current*m_TCP_inv;
            err = tr2delta(tr_current, tr_target);
            double err_norm = err.norm();
            if (err_norm <= kInverseKineTol) {
                for (int i = 0; i < ROBOT_DOF; i++) return_q[i] = q[i] * RADTODEG;
                q_target = return_q;
                return 0;
            }
            else if (err_norm < err_norm_prev) {
                err_norm_prev = err_norm;
                lambda /= 2;	// Leven_Marq method ?
            }
            else {
                err_norm_prev = err_norm;
                lambda *= 2;
            }

            if (iter++ >= kInverseKineIterLimit) {
                std::cerr << "Can't converge the value" << std::endl;
                std::cerr << "err_norm: " << err.norm() << std::endl;
                for (int i = 0; i < ROBOT_DOF; i++) return_q[i] = q_current[i] * RADTODEG;
                q_target = return_q;
                return -1;
            }

            if (iter >= 7) {
                std::cerr << "[Warning] Iteration is " << iter << std::endl;
            }
        }
    }
}

/** @brief Korean: 입력된 작업공간(task space)의 자세를 UR5 로봇의 역기구학(closed-form )을 이용하여 관절공간의 자세로 변환한다.
 * @param[in] q_past : 이전 관절 각도 [deg]
 * @param[in] x_target : 목표 자세 [m], [rad]
 * @param[out] q_target : 역기구학을 통해 계산된 다음 관절 각도 [deg]
 */
int CRobotModel::inverseNumericalForm2(double kInverseKineLambda, int kInverseKineIterLimit, double kInverseKineTransTol, double kInverseKineOriTol, const Eigen::VectorXf &q_past, const Eigen::VectorXf &x_target, Eigen::VectorXf &q_target)
{
    int iter = 0;
    double lambda = kInverseKineLambda;
    Eigen::VectorXf return_q(ROBOT_DOF);
    Eigen::VectorXf q_current = q_past;
	for (int i = 0; i < ROBOT_DOF; i++) { q_current[i] = q_current[i] * DEGTORAD; }
    Eigen::VectorXf pose_target = x_target;

    Eigen::MatrixXf jacobian(6, 6);
    Eigen::MatrixXf JTJ(6, 6);

    // Target
	Eigen::Matrix4f tr_target;
	pose2HTM(pose_target, tr_target);
    //// TCP 적용
    tr_target = tr_target*m_TCP_inv;

    ////////

    // Forward kinematics - Current
    Eigen::VectorXf q(6);
	for (int j = 0; j < 6; j++) { q[j] = q_current[j]; } // [rad]
    Eigen::Matrix4f tr_current = forwardKinematics_q2Tr(q);   // input: [rad], output: position: [m], orientation: RPY[rad]
    //// TCP 적용
    tr_current = tr_current*m_TCP_inv;

    Eigen::VectorXf err = tr2delta(tr_current, tr_target);
    Eigen::Vector3f err_trans = err.block<3,1>(0,0);
    Eigen::Vector3f err_ori = err.block<3,1>(3,0);

    // double err_norm_prev = err.norm();
    double err_trans_norm_prev = err_trans.norm();
    double err_ori_norm_prev = err_ori.norm();
    double err_trans_norm = err_trans_norm_prev;
    double err_ori_norm = err_ori_norm_prev;

    // printf("err: %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f\n", err[0], err[1], err[2], err[3]*RADTODEG, err[4]*RADTODEG, err[5]*RADTODEG);
    // printf("err_trans: %0.6f, %0.6f, %0.6f, err_ori: %0.6f, %0.6f, %0.6f\n", err_trans[0], err_trans[1], err_trans[2], err_ori[0]*RADTODEG, err_ori[1]*RADTODEG, err_ori[2]*RADTODEG);
    // printf("err_trans_norm: %0.6f, err_ori_norm: %0.6f\n", err_trans_norm, err_ori_norm*RADTODEG);
    if (err_trans_norm_prev <= kInverseKineTransTol && err_ori_norm_prev <= kInverseKineOriTol)
    {
        for (int i = 0; i < ROBOT_DOF; i++) return_q[i] = q[i] * RADTODEG;
        q_target = return_q;
        return 0;
    }

    Eigen::MatrixXf lambda_I = lambda*Eigen::MatrixXf::Identity(6, 6);
    Eigen::VectorXf dq;
    Eigen::VectorXf q_deg;
    if (ROBOT_DOF != 6) {
        for (int i = 0; i < ROBOT_DOF; i++) return_q[i] = q_current[i] * RADTODEG;
        q_target = return_q;
        return 0;
    }
    else {
        while (true) {
            calJacobianGeoRad(q, jacobian); // [rad]
            JTJ = jacobian.transpose() * jacobian;

            if (1) 
            {
                // Levenberg metod
                // for (int i = 0; i < 6; i++) {
                //     JTJ.element_[6 * i + i] += lambda;
                // }
                JTJ = JTJ + lambda_I;
            } else  
            {
                // // Levenberg - Marquadt method
                // for (int i = 0; i < 6; i++) {
                //     JTJ.element_[6 * i + i] += lambda * JTJ.element_[6 * i + i];
                // }
            }

            dq = JTJ.inverse() * jacobian.transpose() * err;
            if (ROBOT_DOF == 6) {
                q = q + dq; // [rad]
            }
            q_deg = RADTODEG*q;
            // else if (ROBOT_DOF == 7) {
            //     for (int i = 0; i < ROBOT_DOF; i++) {
            //         if (i == 2) {
            //             q.element_[i] = q_current[i];
            //         }
            //         else if (i > 2) {
            //             q.element_[i] += dq.element_[i - 1];
            //         }
            //         else {
            //             q.element_[i] += dq.element_[i];
            //         }
            //     }
            // }

            tr_current = forwardKinematics_q2Tr(q);   // input: [rad], output: position: [m], orientation: RPY[rad]
            //// TCP 적용
            tr_current = tr_current*m_TCP_inv;
            err = tr2delta(tr_current, tr_target);
            err_trans = err.block<3,1>(0,0);
            err_ori = err.block<3,1>(3,0);
            err_trans_norm = err_trans.norm();
            err_ori_norm = err_ori.norm();

            // printf("err: %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f\n", err[0], err[1], err[2], err[3]*RADTODEG, err[4]*RADTODEG, err[5]*RADTODEG);
            // printf("err_trans: %0.6f, %0.6f, %0.6f, err_ori: %0.6f, %0.6f, %0.6f\n", err_trans[0], err_trans[1], err_trans[2], err_ori[0]*RADTODEG, err_ori[1]*RADTODEG, err_ori[2]*RADTODEG);
            // printf("err_trans_norm: %0.6f, err_ori_norm: %0.6f\n", err_trans_norm, err_ori_norm*RADTODEG);
            if (err_trans_norm <= kInverseKineTransTol && err_ori_norm <= kInverseKineOriTol)
            {
                for (int i = 0; i < ROBOT_DOF; i++) return_q[i] = q[i] * RADTODEG;
                q_target = return_q;
                return 0;
            }
            else if (err_trans_norm < err_trans_norm_prev || err_ori_norm < err_ori_norm_prev) {
                if(err_trans_norm < err_trans_norm_prev) err_trans_norm_prev = err_trans_norm;
                if(err_ori_norm < err_ori_norm_prev) err_ori_norm_prev = err_ori_norm;
                lambda /= 2;	// Leven_Marq method ?
            }
            else {
                err_trans_norm_prev = err_trans_norm;
                err_ori_norm_prev = err_ori_norm;
                lambda *= 2;
            }

            if (iter++ >= kInverseKineIterLimit) {
                std::cerr << "Can't converge the value" << std::endl;
                std::cerr << "err_trans_norm: " << err_trans_norm << std::endl;
                std::cerr << "err_ori_norm: " << err_ori_norm << std::endl;
                for (int i = 0; i < ROBOT_DOF; i++) return_q[i] = q_current[i] * RADTODEG;
                q_target = return_q;
                return -1;
            }

            if (iter >= 7) {
                // std::cerr << "[Warning] Iteration is " << iter << std::endl;
            }
        }
    }
}


Eigen::VectorXf CRobotModel::tr2delta(const Eigen::MatrixXf &tr_old, const Eigen::MatrixXf &tr_target) 
{
    Eigen::MatrixXf TD(4, 4);
    Eigen::VectorXf delta(6);

    TD = tr_old.inverse() * tr_target;

    // delta.element_[0] = TD.element_[3];
    // delta.element_[1] = TD.element_[7];
    // delta.element_[2] = TD.element_[11];
    // delta.element_[3] = 0.5 * (TD.element_[9] - TD.element_[6]);
    // delta.element_[4] = 0.5 * (TD.element_[2] - TD.element_[8]);
    // delta.element_[5] = 0.5 * (TD.element_[4] - TD.element_[1]);
    // 0  1  2  3
    // 4  5  6  7
    // 8  9  10 11
    // 12 13 14 15

    delta[0] = TD(0, 3);
    delta[1] = TD(1, 3);
    delta[2] = TD(2, 3);
    delta[3] = 0.5 * (TD(2, 1) - TD(1, 2));
    delta[4] = 0.5 * (TD(0, 2) - TD(2, 0));
    delta[5] = 0.5 * (TD(1, 0) - TD(0, 1));

    return delta;
}
/** @brief Korean: 자세를 변환행렬로 형식으로 변환한다.
 * @param[in] x : 작업공간의 자세 (x, y, z, Rx, Ry, Rz)
 * @param[out] T_out : 변환행렬 형식의 자세
 */
void CRobotModel::pose2HTM(const Eigen::VectorXf &x, Eigen::Matrix4f &T_out)
{
	T_out = Eigen::Matrix4f::Identity();
	Eigen::Matrix3f rotM_tmp = Eigen::Matrix3f::Identity();
	T_out(0, 3) = x[0];
	T_out(1, 3) = x[1];
	T_out(2, 3) = x[2];
	RPY2Rot(x.segment(3, 3), rotM_tmp);
	T_out.block<3, 3>(0, 0) = rotM_tmp;
}

/** @brief Korean: 두 double 형 변수를 입력하여 하나라도 0에 가까우면 0을 산출
 * @param[in] v1 : 입력 변수1
 * @param[in] v2 : 입력 변수2
 * @return : v1/v2 또는 0
 */
double CRobotModel::checkFloatingError(double v1, double v2)
{
	if (checkEquality(v1, 0.0) && checkEquality(v2, 0.0))
	{
		return 0.0;
	}
	else if (checkEquality(v1, 0.0) && !checkEquality(v2, 0.0))
	{
		return 0.0;
	}
	else if (!checkEquality(v1, 0.0) && checkEquality(v2, 0.0))
	{
		return 0.0;
	}
	else
	{
		return v1 / v2;
	}
}

/** @brief Korean: 입력된 두 값의 차이가 10^(-6)보다 작으면 true, 크면 false
 * @param[in] value : 입력 변수1
 * @param[in] match : 입력 변수2
 * @return Output_x : 순기구학의 결과를 로봇 자세로 반환
 */
bool CRobotModel::checkEquality(double value, double match)
{
	return fabs(value - match) < 0.000001;
}

/** @brief Korean: 회전행렬(rotation matrix)을 RPY의 형태의 방위로 변환
 * @param[in] dRot : 입력 회전행렬
 * @param[out] dRPY : 출력 Roll-Pitch-Yaw (Rx, Ry, Rz)
 */
void CRobotModel::Rot2RPY(double dRot[3][3], double dRPY[3]) {
    // Roll-Pitch-Yaw - Euler ZYX
    dRPY[0] = atan2(dRot[2][1], dRot[2][2]);											    //Rx
    dRPY[1] = atan2(-dRot[2][0], sqrt(dRot[2][1] * dRot[2][1] + dRot[2][2] * dRot[2][2]));	//Ry
    dRPY[2] = atan2(dRot[1][0], dRot[0][0]);											    //Rz
}

/** @brief Korean: RPY의 형태의 방위를 회전행렬(rotation matrix)로 변환
 * @param[in] dRPY : 입력 Roll-Pitch-Yaw (Rx, Ry, Rz)
 * @param[out] dRot : 출력 회전행렬
 */
void CRobotModel::RPY2Rot(double dRPY[3], double dRot[3][3]) {
    dRot[0][0] =  cos(dRPY[2]) * cos(dRPY[1]);
    dRot[0][1] =  cos(dRPY[2]) * sin(dRPY[1]) * sin(dRPY[0]) - sin(dRPY[2]) * cos(dRPY[0]);
    dRot[0][2] =  cos(dRPY[2]) * sin(dRPY[1]) * cos(dRPY[0]) + sin(dRPY[2]) * sin(dRPY[0]);

    dRot[1][0] =  sin(dRPY[2]) * cos(dRPY[1]);
    dRot[1][1] =  sin(dRPY[2]) * sin(dRPY[1]) * sin(dRPY[0]) + cos(dRPY[2]) * cos(dRPY[0]);
    dRot[1][2] =  sin(dRPY[2]) * sin(dRPY[1]) * cos(dRPY[0]) - cos(dRPY[2]) * sin(dRPY[0]);

    dRot[2][0] = -sin(dRPY[1]);
    dRot[2][1] =  cos(dRPY[1]) * sin(dRPY[0]);
    dRot[2][2] =  cos(dRPY[1]) * cos(dRPY[0]);
}

/** @brief Korean: RPY를 회전행렬로 변환한다.
 * @param[in] dRPY : Roll-Pitch-Yaw (Rx, Ry, Rz)
 * @param[out] rotM : 출력 회전행렬
 */
void CRobotModel::RPY2Rot(const Eigen::Vector3f &dRPY, Eigen::Matrix3f &rotM) // [rad]
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

/** @brief Korean: 입력된 회전축, 회전각도를 이용하여 회전 및 변환행렬을 계산한다.
 * @param[in] angle : 회전각도, [rad]
 * @param[in] direction : 회전 축 설정 (0: Rx, 1: Ry, 2: Rz)
 * @return rotM : 회전축을 중심으로 회전각도만큼 회전한 회전행렬
 */
Eigen::Matrix3f CRobotModel::genRotM(double angle, int direction)
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

/** @brief Korean: 입력된 회전행렬을 ZYX euler angle (roll, pitch, yaw)로 반환한다.
 * @param[in] RotMat : 입력 회전행렬
 */
Eigen::Vector3f CRobotModel::rotm2ZYX(Eigen::Matrix3f &RotMat)
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


    // dPoseRPY[4] = atan2(-matRPY2Base(2, 0), sqrt(matRPY2Base(2, 1) * matRPY2Base(2, 1) + matRPY2Base(2, 2) * matRPY2Base(2, 2)));

    // if (fabs(cos(dPoseRPY[4])) > 0.0000001) {
    //     dPoseRPY[3] = atan2(matRPY2Base(2, 1), matRPY2Base(2, 2));
    //     dPoseRPY[5] = atan2(matRPY2Base(1, 0), matRPY2Base(0, 0));
    // } else if (fabs(dPoseRPY[4] - M_PI / 2) < 0.01) {
    //     dPoseRPY[5] = atan2(matRPY2Base(2, 1), matRPY2Base(2, 2));
    //     dPoseRPY[3] = dPoseRPY[5] - atan2(matRPY2Base(1, 2), matRPY2Base(0, 2));
    // } else {
    //     dPoseRPY[5] = atan2(matRPY2Base(2, 1), matRPY2Base(2, 2));
    //     dPoseRPY[3] = -dPoseRPY[5] + atan2(-matRPY2Base(1, 2), -matRPY2Base(0, 2));
    // }


}

void CRobotModel::Base2RPY(double dPoseBaseFrame[6], double dPoseRPY[6], std::vector<double> &measured_q) {
    // dPoseBaseFrame: [rad], measured_q: [rad], dPoseRPY [rad]
	// x,y,z 회전행렬
	Eigen::Matrix4f matRotx = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f matRoty = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f matRotz = Eigen::Matrix4f::Identity();
    double Rx = dPoseBaseFrame[3]; // [rad]
    double Ry = dPoseBaseFrame[4]; // [rad]
    double Rz = dPoseBaseFrame[5]; // [rad]
	matRotx.block<3,3>(0,0) = genRotM(Rx, 0); // 0: rx, 1:ry, 2:rz
	matRoty.block<3,3>(0,0) = genRotM(Ry, 1); // 0: rx, 1:ry, 2:rz
	matRotz.block<3,3>(0,0) = genRotM(Rz, 2); // 0: rx, 1:ry, 2:rz

    // Transformation matrix
	Eigen::Matrix4f matTrans = TransformationMatrix(0, measured_q[0]); // [rad]
    for (int i = 1; i < ROBOT_DOF; i++) matTrans = matTrans * TransformationMatrix(i, measured_q[i]); // [rad]

	// TCP 적용
    matTrans = matTrans * m_TCP;

    // 절대 좌표계 기준 회전
	Eigen::Matrix4f matRPY2Base = Eigen::Matrix4f::Identity();
    matRPY2Base = matRotz * matRoty * matRotx * matTrans;

    dPoseRPY[0] = dPoseBaseFrame[0];
    dPoseRPY[1] = dPoseBaseFrame[1];
    dPoseRPY[2] = dPoseBaseFrame[2];

    // Orientation vector (RPY)
	// Eigen::Matrix3f rotm = matRPY2Base.block<3,3>(0,0);
	// Eigen::Vector3f rpy = rotm2ZYX(rotm);
    // dPoseRPY[3] = rpy[0];
    // dPoseRPY[4] = rpy[1];
    // dPoseRPY[5] = rpy[2];

    dPoseRPY[4] = atan2(-matRPY2Base(2, 0), sqrt(matRPY2Base(2, 1) * matRPY2Base(2, 1) + matRPY2Base(2, 2) * matRPY2Base(2, 2)));

    if (fabs(cos(dPoseRPY[4])) > 0.0000001) {
        dPoseRPY[3] = atan2(matRPY2Base(2, 1), matRPY2Base(2, 2));
        dPoseRPY[5] = atan2(matRPY2Base(1, 0), matRPY2Base(0, 0));
    } else if (fabs(dPoseRPY[4] - M_PI / 2) < 0.01) {
        dPoseRPY[5] = atan2(matRPY2Base(2, 1), matRPY2Base(2, 2));
        dPoseRPY[3] = dPoseRPY[5] - atan2(matRPY2Base(1, 2), matRPY2Base(0, 2));
    } else {
        dPoseRPY[5] = atan2(matRPY2Base(2, 1), matRPY2Base(2, 2));
        dPoseRPY[3] = -dPoseRPY[5] + atan2(-matRPY2Base(1, 2), -matRPY2Base(0, 2));
    }
    for (int i = 0; i < 3; i++) {
        if (dPoseRPY[i + 3] > M_PI) {
            dPoseRPY[i + 3] = dPoseRPY[i + 3] - 2 * M_PI;
        } else if (dPoseRPY[i + 3] < -M_PI) {
            dPoseRPY[i + 3] = dPoseRPY[i + 3] + 2 * M_PI;
        }
    }

}

void CRobotModel::JogBase2RPY(double dPoseBaseFrame[6], double dPoseRPY[6], std::vector<double> &target_x) {
    // dPoseBaseFrame: [rad], measured_q: [rad], dPoseRPY [rad]
	// x,y,z 회전행렬
	Eigen::Matrix4f matRotx = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f matRoty = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f matRotz = Eigen::Matrix4f::Identity();
    double Rx = dPoseBaseFrame[3]; // [rad]
    double Ry = dPoseBaseFrame[4]; // [rad]
    double Rz = dPoseBaseFrame[5]; // [rad]
	matRotx.block<3,3>(0,0) = genRotM(Rx, 0); // 0: rx, 1:ry, 2:rz
	matRoty.block<3,3>(0,0) = genRotM(Ry, 1); // 0: rx, 1:ry, 2:rz
	matRotz.block<3,3>(0,0) = genRotM(Rz, 2); // 0: rx, 1:ry, 2:rz

	// pose 2 HTM
	Eigen::Matrix4f matTrans = Eigen::Matrix4f::Identity();
	Eigen::VectorXf x_current(6);
	for(int i=0; i<6; i++) x_current[i] = target_x[i];
	pose2HTM(x_current, matTrans);

    // 절대 좌표계 기준 회전
	Eigen::Matrix4f matRPY2Base = Eigen::Matrix4f::Identity();
    matRPY2Base = matRotz * matRoty * matRotx * matTrans;

    dPoseRPY[0] = dPoseBaseFrame[0];
    dPoseRPY[1] = dPoseBaseFrame[1];
    dPoseRPY[2] = dPoseBaseFrame[2];

    // Orientation vector (RPY)
	// Eigen::Matrix3f rotm = matRPY2Base.block<3,3>(0,0);
	// Eigen::Vector3f rpy = rotm2ZYX(rotm);
    // dPoseRPY[3] = rpy[0];
    // dPoseRPY[4] = rpy[1];
    // dPoseRPY[5] = rpy[2];

    dPoseRPY[4] = atan2(-matRPY2Base(2, 0), sqrt(matRPY2Base(2, 1) * matRPY2Base(2, 1) + matRPY2Base(2, 2) * matRPY2Base(2, 2)));

    if (fabs(cos(dPoseRPY[4])) > 0.0000001) {
        dPoseRPY[3] = atan2(matRPY2Base(2, 1), matRPY2Base(2, 2));
        dPoseRPY[5] = atan2(matRPY2Base(1, 0), matRPY2Base(0, 0));
    } else if (fabs(dPoseRPY[4] - M_PI / 2) < 0.01) {
        dPoseRPY[5] = atan2(matRPY2Base(2, 1), matRPY2Base(2, 2));
        dPoseRPY[3] = dPoseRPY[5] - atan2(matRPY2Base(1, 2), matRPY2Base(0, 2));
    } else {
        dPoseRPY[5] = atan2(matRPY2Base(2, 1), matRPY2Base(2, 2));
        dPoseRPY[3] = -dPoseRPY[5] + atan2(-matRPY2Base(1, 2), -matRPY2Base(0, 2));
    }
    for (int i = 0; i < 3; i++) {
        if (dPoseRPY[i + 3] > M_PI) {
            dPoseRPY[i + 3] = dPoseRPY[i + 3] - 2 * M_PI;
        } else if (dPoseRPY[i + 3] < -M_PI) {
            dPoseRPY[i + 3] = dPoseRPY[i + 3] + 2 * M_PI;
        }
    }

}

void CRobotModel::End2RPY(double dPoseEndFrame[6], double dPoseRPY[6], std::vector<double> &measured_q) {
    // dPoseEndFrame: [rad], measured_q: [rad], dPoseRPY [rad]
	// x,y,z 회전행렬
	Eigen::Matrix4f matRotx = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f matRoty = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f matRotz = Eigen::Matrix4f::Identity();

    // Transformation matrix
	Eigen::Matrix4f matTrans = TransformationMatrix(0, measured_q[0]); // [rad]
    for (int i = 1; i < ROBOT_DOF; i++) matTrans = matTrans * TransformationMatrix(i, measured_q[i]); // [rad]

	// TCP 적용
    matTrans = matTrans * m_TCP;

    // 말단 좌표계 업데이트 (이 함수 내에서는 의미없음)
	Eigen::Matrix4f matTransB2E = Eigen::Matrix4f::Identity();
    matTransB2E = matTrans * matRotz;

    Eigen::VectorXf vecPosEF(4);
    for(int i = 0; i < 3; i++) vecPosEF[i] = dPoseEndFrame[i]; // 말단 기준 변위
    vecPosEF[3] = 1.0;

    // 말단 좌표 기준의 목표점을 RPY 기준으로 변환
    Eigen::VectorXf vecPosBF(4);
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
}

void CRobotModel::JogEnd2RPY(double dPoseEndFrame[6], double dPoseRPY[6], std::vector<double> &target_x) {
    // dPoseEndFrame: [rad], measured_q: [rad], dPoseRPY [rad]
	// x,y,z 회전행렬
	Eigen::Matrix4f matRotx = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f matRoty = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f matRotz = Eigen::Matrix4f::Identity();

	// pose 2 HTM
	Eigen::Matrix4f matTrans = Eigen::Matrix4f::Identity();

	Eigen::VectorXf x_current(6);
	for(int i=0; i<6; i++) x_current[i] = target_x[i];
	pose2HTM(x_current, matTrans);

    // 말단 좌표계 업데이트 (이 함수 내에서는 의미없음)
	Eigen::Matrix4f matTransB2E = Eigen::Matrix4f::Identity();
    matTransB2E = matTrans * matRotz;

    Eigen::VectorXf vecPosEF(4);
    for(int i = 0; i < 3; i++) vecPosEF[i] = dPoseEndFrame[i]; // 말단 기준 변위
    vecPosEF[3] = 1.0;

    // 말단 좌표 기준의 목표점을 RPY 기준으로 변환
    Eigen::VectorXf vecPosBF(4);
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
}

Eigen::Matrix4f CRobotModel::poseVector2HTM(const std::vector<double> &pose) 
{
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    double rot[3][3] = {{0.0,},};
    double rpyofTcp[3] = {pose[3], pose[4], pose[5]}; // RPY
    RPY2Rot(rpyofTcp, rot);

    T(0, 0) = rot[0][0]; T(0, 1) = rot[0][1]; T(0, 2) = rot[0][2];
    T(1, 0) = rot[1][0]; T(1, 1) = rot[1][1]; T(1, 2) = rot[1][2];
    T(2, 0) = rot[2][0]; T(2, 1) = rot[2][1]; T(2, 2) = rot[2][2];

    T(0, 3) = pose[0];
    T(1, 3) = pose[1];
    T(2, 3) = pose[2];

    return T;
}

//// Tool frame HTM
void CRobotModel::transformToolFrame(const std::vector<double> &dPoseEndFrame, const std::vector<double> &x_measured, std::vector<double> &output) {
    std::vector<double> dPoseRPY(6);
    // dPoseEndFrame: [rad], measured_q: [rad], dPoseRPY [rad]
	// x,y,z 회전행렬
	Eigen::Matrix4f matRotx = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f matRoty = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f matRotz = Eigen::Matrix4f::Identity();

	// pose 2 HTM
	Eigen::Matrix4f matTrans = Eigen::Matrix4f::Identity();
	Eigen::VectorXf x_current(6);
	for(int i=0; i<6; i++) x_current[i] = x_measured[i];
	pose2HTM(x_current, matTrans);

    // 말단 좌표계 업데이트 (이 함수 내에서는 의미없음)
	Eigen::Matrix4f matTransB2E = Eigen::Matrix4f::Identity();
    matTransB2E = matTrans * matRotz;

    Eigen::VectorXf vecPosEF(4);
    for(int i = 0; i < 3; i++) vecPosEF[i] = dPoseEndFrame[i]; // 말단 기준 변위
    vecPosEF[3] = 1.0;

    // 말단 좌표 기준의 목표점을 RPY 기준으로 변환
    Eigen::VectorXf vecPosBF(4);
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

/** @brief Korean: 로봇의 geometric Jacobian을 계산한다.
 * @param[in] q_deg : 관절각도 [deg]
 * @param[out] J_geo : Geometric Jacobian
 */
void CRobotModel::calJacobianGeo(const Eigen::VectorXf &q_deg, Eigen::MatrixXf &J_geo)
{
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> matTrans;
    matTrans.resize(ROBOT_DOF);
	double q = 0.0;
    for (int i = 0; i < ROBOT_DOF; i++)
	{
		matTrans[i] = Eigen::Matrix4f::Identity();
		q = q_deg[i] * DEGTORAD;
		matTrans[i] = TransformationMatrix(i, q);
	}

    Eigen::MatrixXf matJ(6, ROBOT_DOF);
	matJ.block<3, 3>(0, 0) = Eigen::Matrix3f::Zero();
	matJ.block<3, 3>(3, 3) = Eigen::Matrix3f::Zero();
	matJ.block<3, 3>(0, 3) = Eigen::Matrix3f::Zero();
	matJ.block<3, 3>(3, 0) = Eigen::Matrix3f::Zero();

	Eigen::Matrix4f matU = Eigen::Matrix4f::Identity();
	
    for (int i = ROBOT_DOF-1; i >= 0; i--)
	{
		matU = matTrans[i] * matU;

		Eigen::Vector3f vec_p;
		vec_p[0] = -matU(0, 0)*matU(1, 3) + matU(1, 0)*matU(0, 3);
		vec_p[1] = -matU(0, 1)*matU(1, 3) + matU(1, 1)*matU(0, 3);
		vec_p[2] = -matU(0, 2)*matU(1, 3) + matU(1, 2)*matU(0, 3);

		Eigen::Vector3f vec_r;
		vec_r[0] = matU(2, 0);
		vec_r[1] = matU(2, 1);
		vec_r[2] = matU(2, 2);

		matJ.block<3, 1>(0, i) = vec_p;
		matJ.block<3, 1>(3, i) = vec_r;
	}
	J_geo = matJ;
}

/** @brief Korean: 로봇의 geometric Jacobian을 계산한다.
 * @param[in] q_rad : 관절각도 [rad]
 * @param[out] J_geo : Geometric Jacobian
 */
void CRobotModel::calJacobianGeoRad(const Eigen::VectorXf &q_rad, Eigen::MatrixXf &J_geo)
{
	// std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> matTrans;
    // matTrans.resize(ROBOT_DOF);
	double q = 0.0;
	// 	matTrans[i] = Eigen::Matrix4f::Identity();
	// 	q = q_rad[i];
	// 	matTrans[i] = TransformationMatrix(i, q);
	// }

    Eigen::MatrixXf matJ = Eigen::MatrixXf::Zero(6, ROBOT_DOF);
	Eigen::Matrix4f matU = Eigen::Matrix4f::Identity();
    for (int i = ROBOT_DOF; i > 0; i--)
	{
        q = q_rad[i-1];
		matU = TransformationMatrix(i - 1, q) * matU;

		Eigen::Vector3f vec_p;
		vec_p[0] = -matU(0, 0)*matU(1, 3) + matU(1, 0)*matU(0, 3);
		vec_p[1] = -matU(0, 1)*matU(1, 3) + matU(1, 1)*matU(0, 3);
		vec_p[2] = -matU(0, 2)*matU(1, 3) + matU(1, 2)*matU(0, 3);

		Eigen::Vector3f vec_r;
		vec_r[0] = matU(2, 0);
		vec_r[1] = matU(2, 1);
		vec_r[2] = matU(2, 2);

		matJ.block<3, 1>(0, i-1) = vec_p;
		matJ.block<3, 1>(3, i-1) = vec_r;
	}
	J_geo = matJ;
}