/**
 * @file path_planning.cpp
 * @brief 로봇의 경로계획을 위한 구현파일
 */
// #include "stdafx.h"
#include "module/path_planning.h"
#include <string>
#include <iostream>
#include <math.h>

//const unsigned int ROBOT_DOF_INSP = 6;

PATHPLANNING::PATHPLANNING()
{
}

PATHPLANNING::~PATHPLANNING()
{

}

/** @brief Korean: 입력된 센서 자세로부터 로봇 자세를 생성한다.
 * @param[in] T_in : 입력된 센서 자세
 * @param[out] pose_set : 로봇 자세 set
 */
bool PATHPLANNING::genRobotPose(const std::vector<Eigen::Matrix4f> &T_in, std::vector<Eigen::VectorXf> &pose_set)
{
	std::vector<Eigen::VectorXf> pose_set_final;
	for (int i = 0; i < T_in.size(); i++)
	{
		Eigen::Matrix4f T_B2C_Pose = Eigen::Matrix4f::Identity();
		T_B2C_Pose = T_in[i];
			
		// Camera to End-effector
		Eigen::Matrix4f T_B2T_Pose = T_B2C_Pose*m_T_S2T;
		Eigen::Matrix3f R_B2T_Pose = T_B2T_Pose.block<3, 3>(0, 0);
		Eigen::Vector3f RPY_B2T = m_math.rotm2ZYX(R_B2T_Pose);
			
		Eigen::VectorXf pose_final(6);
		pose_final[0] = T_B2T_Pose(0, 3);
		pose_final[1] = T_B2T_Pose(1, 3);
		pose_final[2] = T_B2T_Pose(2, 3);
		pose_final[3] = RPY_B2T[0];
		pose_final[4] = RPY_B2T[1];
		pose_final[5] = RPY_B2T[2];
		pose_set_final.push_back(pose_final);
	}
	// update
	pose_set = pose_set_final; // robot poses
	Eigen::VectorXf q_init(6);
	q_init[0] = -131.36;
	q_init[1] = -63.39;
	q_init[2] = -91.72;
	q_init[3] = -116.39;
	q_init[4] = -89.7;
	q_init[5] = 222.64;
	if (!checkPoseSetPath(pose_set_final, q_init))
	{
		printf("Path check Success!\n");
		return true;
	}
	else
	{
		printf("Path check Failure!\n");
		return false;
	}
}

/** @brief Korean: 입력된 센서 자세로부터 로봇 자세를 생성한다.
* @param[in] path_data : 경로 생성을 위한 데이터 객체
*/
bool PATHPLANNING::genRobotPose2(CPATHDATA &path_data)
{
	std::vector<Eigen::VectorXf> pose_set_final;
	for (int i = 0; i < path_data.target_sensor_pose.size(); i++)
	{
		Eigen::Matrix4f T_B2C_Pose = Eigen::Matrix4f::Identity();
//		T_B2C_Pose = path_data.target_sensor_pose[i]; // PCL 1.8.1

        // PCL 1.12.1
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k){
                T_B2C_Pose(j,k) = path_data.target_sensor_pose[i][j*4 + k];
            }
        }

		// Camera to End-effector
		//Eigen::Matrix4f T_B2T_Pose = T_B2C_Pose*path_data.T_T2S.inverse();
		Eigen::Matrix4f T_B2T_Pose = T_B2C_Pose*path_data.T_S2T;
		Eigen::Matrix3f R_B2T_Pose = T_B2T_Pose.block<3, 3>(0, 0);
		Eigen::Vector3f RPY_B2T = m_math.rotm2ZYX(R_B2T_Pose);

		Eigen::VectorXf pose_final(6);
		pose_final[0] = T_B2T_Pose(0, 3);
		pose_final[1] = T_B2T_Pose(1, 3);
		pose_final[2] = T_B2T_Pose(2, 3);
		pose_final[3] = RPY_B2T[0];
		pose_final[4] = RPY_B2T[1];
		pose_final[5] = RPY_B2T[2];
		pose_set_final.push_back(pose_final);
	}

	Eigen::VectorXf q_init(6);
	for (int i = 0; i < 6; i++) { q_init[i] = path_data.initial_JS_angle[i]; }

	if (!checkPoseSetPath(pose_set_final, q_init))
	{
		// update
		path_data.target_pose = pose_set_final; // robot poses
        std::cout << "Path check Success!" << std::endl;
		return true;
	}
	else
	{
        std::cout << "Path check Failure!" << std::endl;
		return false;
	}
}

/** @brief Korean: 입력된 센서의 시야점, 법선, 중심을 이용하여 센서 좌표계 기준 자세를 산출한다.
* @details
* @param[in] path_data : 경로계획을 위한 데이터 객체
*/
bool PATHPLANNING::genRobotPoseFromViewpoint(CVIEWPOINTDATA &vp_data, CPATHDATA &path_data, Eigen::VectorXf &q_prev, std::vector<double> &robot_acc, std::vector<double> &robot_vel)
{
	Eigen::Vector3d vp_in = vp_data.VP;
	Eigen::Vector3d normal_in = vp_data.normal;
	Eigen::Vector3d center_in = vp_data.center;
	
	// 1) Cal. rotational angle
	double angle_tmp, rotation_angle = 0.0;
	angle_tmp = m_math.calRotationAngleFromNormal(normal_in, center_in, path_data.angle_rotation_stage2Base, GLOBAL_NORMAL_AXIS);
	rotation_angle = -angle_tmp;

	// 2) Object to base frame
	// Transform Object coordinate to robot base coordinate
	Eigen::Matrix3f rotM_tmp = Eigen::Matrix3f::Identity();
	Eigen::Matrix4f HTM_tmp = Eigen::Matrix4f::Identity();
	m_math.genRotM(rotation_angle, rotM_tmp, HTM_tmp, GLOBAL_NORMAL_AXIS); // 0: Rx, 1: Ry, 2: Rz
	Eigen::Matrix4f T_B2O = path_data.T_B2RS*HTM_tmp;

	// 3) Transformation to rotation stage
	Eigen::Vector3d vp_rot = m_math.transformPoint2A(vp_in, T_B2O.cast<double>()); // viewpoint
	Eigen::Vector3d center_rot = m_math.transformPoint2A(center_in, T_B2O.cast<double>()); // center

	// 4) Sensor poses - transformation matrix
	Eigen::Matrix4d T_B2C_Pose = m_math.transformViewpoint2SensorPose(vp_rot, center_rot, path_data.cam_rx_vector); // cam_rx: {1 0 0}, base기준의 +x와 동일
	Eigen::Matrix4f T_out_tmp = T_B2C_Pose.cast<float>();

	// 5) Camera to end-effector of the robot
	Eigen::Matrix4f T_B2T_Pose = T_B2C_Pose.cast<float>()*path_data.T_T2S.inverse();
	Eigen::Matrix3f R_B2T_Pose = T_B2T_Pose.block<3, 3>(0, 0);
	Eigen::Vector3f RPY_B2T = m_math.rotm2ZYX(R_B2T_Pose);

	Eigen::VectorXf pose_final(6);
	pose_final[0] = T_B2T_Pose(0, 3);
	pose_final[1] = T_B2T_Pose(1, 3);
	pose_final[2] = T_B2T_Pose(2, 3);
	pose_final[3] = RPY_B2T[0];
	pose_final[4] = RPY_B2T[1];
	pose_final[5] = RPY_B2T[2];
	
	if (!checkSinglePosePath(pose_final, q_prev, robot_acc, robot_vel))
	{
		Eigen::VectorXf q_now(6); // 이전 관절각도
		inverseClosedForm(q_prev, pose_final, q_now);
		// update
		vp_data.is_path_feasible = true;
		vp_data.target_CS_pose = pose_final; // robot poses
		vp_data.target_JS_position = q_now; // robot JS angle
		vp_data.target_rs_angle = rotation_angle; // rotation stage target angle
		
		printf("Path check Success!\n");
		return true;
	}
	else
	{
		vp_data.is_path_feasible = false;
		printf("Path check Failure!\n");
		return false;
	}
}

/** @brief Korean: 입력된 자세 set의 측정 경로를 검사한다.
* @details 각 자세 사이의 경로를 생성한 후, 역기구학(inverse kinematics)을 통해 카테시안 공간(cartesian space) 상의 경로를 관절 공간(joint space)으로 변환한다. 여기서 사용된 역기구학은 대상 로봇인 UR5의 closed-form 수식을 사용하였다. 이때 추출된 경로의 관절각도에 대해 자코비안을 계산하여 도달 불가능한 자세를 검사한다.
* @param[in] pose_set : 입력된 자세 set
* @param[in] q_init: 로봇의 초기 관절 각도 [deg]
*/
bool PATHPLANNING::checkPoseSetPath(const std::vector<Eigen::VectorXf> &pose_set, const Eigen::VectorXf q_init)
{
	bool b_singularity = false;
	Eigen::VectorXf P_init(6);
	Eigen::Matrix4f T_init;
	m_math.forwardKinematics(m_DH, q_init, T_init);
	m_math.tr2pose(T_init, P_init);
	for (int j = 3; j < 6; j++) { P_init[j] = (180.0 / M_PI)*P_init[j]; } // deg

	setInitPos(P_init, true);

	Eigen::VectorXf dAcc(6); // [m/s^2], [deg/s^2]
	dAcc[0] = 4.0;
	dAcc[1] = 4.0;
	dAcc[2] = 4.0;
	dAcc[3] = 180.0;
	dAcc[4] = 180.0;
	dAcc[5] = 180.0;

	Eigen::VectorXf dVel(6); // [m/s], [deg/s]
	dVel[0] = 1.0;
	dVel[1] = 1.0;
	dVel[2] = 1.0;
	dVel[3] = 30.0;
	dVel[4] = 30.0;
	dVel[5] = 30.0;

	Eigen::VectorXf q_prev = q_init;
	Eigen::VectorXf q_current(6);
	for (int i = 0; i < pose_set.size(); i++)
	{
		Eigen::VectorXf pose_tgt = pose_set[i];
		for (int j = 0; j < 3; j++) { pose_tgt[j] = 0.001*pose_tgt[j]; } // [m], [rad]

		if (i != 0)
		{
			Eigen::VectorXf pose_prev = pose_set[i - 1];
			for (int j = 0; j < 3; j++) { pose_prev[j] = 0.001*pose_prev[j]; } // [m], [rad]
			P_init = pose_prev;
		}

		// Trajectory planning
		setAcceleration(dAcc);
		setMaxVelocity(dVel);
		std::vector<Eigen::VectorXf> trajectory;
		trajectoryPlanning(pose_tgt, P_init, trajectory); // [m], [rad]
		for (int j = 0; j < trajectory.size(); j++)
		{
			// Inverse kinematics
			inverseClosedForm(q_prev, trajectory[j], q_current);

			if (j == 0 || j == trajectory.size() - 1)
			{
				std::cout << j << "th angle:" << std::endl;
				std::cout << q_current << std::endl << std::endl;
			}

			// Jacobian check
			Eigen::MatrixXf matJ_check;
			calJacobianGeo(q_current, matJ_check);

			if (checkJacobianCondNum(matJ_check, 100.0)) { b_singularity = true; }

			q_prev = q_current;
		}

		// singularity check
		if (b_singularity)
		{
			printf("Singularity between %ith pose and %ith pose\n", i + 1, i + 2);
			//std::stringstream ss;
			//ss << "Singularity between " << i + 1 << "th pose and " << i + 2 << "th pose";
			//AfxMessageBox(ss.str().c_str());
			break;
		}
		else
		{
			printf("Path between %ith pose and %ith pose is feasible\n", i + 1, i + 2);
		}
	}

	return b_singularity;
}

/** @brief Korean: 입력된 자세의 측정 경로를 검사한다.
* @details 각 자세 사이의 경로를 생성한 후, 역기구학(inverse kinematics)을 통해 카테시안 공간(cartesian space) 상의 경로를 관절 공간(joint space)으로 변환한다. 여기서 사용된 역기구학은 대상 로봇인 UR5의 closed-form 수식을 사용하였다. 이때 추출된 경로의 관절각도에 대해 자코비안을 계산하여 도달 불가능한 자세를 검사한다.
* @param[in] pose_set : 입력된 자세 set
* @param[in] q_init: 로봇의 초기 관절 각도 [deg]
*/
bool PATHPLANNING::checkSinglePosePath(const Eigen::VectorXf &pose_in, const Eigen::VectorXf q_init, std::vector<double> &robot_acc, std::vector<double> &robot_vel)
{
	bool b_singularity = false;
	Eigen::VectorXf P_init(6);
	Eigen::Matrix4f T_init;
	Eigen::VectorXf dAcc(6); // [m/s^2], [deg/s^2]
	Eigen::VectorXf dVel(6); // [m/s], [deg/s]
	for (size_t i = 0; i < robot_acc.size(); i++) { dAcc[i] = robot_acc[i]; }
	for (size_t i = 0; i < robot_vel.size(); i++) { dVel[i] = robot_acc[i]; }
	Eigen::VectorXf q_prev = q_init;
	Eigen::VectorXf q_current(6);
	Eigen::VectorXf pose_tgt = pose_in;
	for (int j = 0; j < 3; j++) { pose_tgt[j] = 0.001*pose_tgt[j]; } // [m], [rad]

	// Forward kinematics
	m_math.forwardKinematics(m_DH, q_init, T_init);
	m_math.tr2pose(T_init, P_init); // [m], [rad]

	// Trajectory planning
	setInitPos(P_init, true);
	setAcceleration(dAcc);
	setMaxVelocity(dVel);
	std::vector<Eigen::VectorXf> trajectory;
	trajectoryPlanning(pose_tgt, P_init, trajectory); // [m], [rad]
	for (int j = 0; j < trajectory.size(); j++)
	{
		// Inverse kinematics
		inverseClosedForm(q_prev, trajectory[j], q_current);
		if (j == 0 || j == trajectory.size() - 1) { std::cout << j << "th angle:" << std::endl << q_current << std::endl; }
		// Jacobian check
		Eigen::MatrixXf matJ_check;
		calJacobianGeo(q_current, matJ_check);
		if (checkJacobianCondNum(matJ_check, 100.0)) { b_singularity = true; }
		q_prev = q_current;
	}
	// singularity check
	if (b_singularity) { printf("Singularity!!\n"); }
	else { printf("Path is feasible\n"); }
	return b_singularity;
}

/** @brief Korean: 초기 자세(P_init)로부터 목표 자세(P_cmd)에 도달하기 위한 직선 경로를 생성한다.
 * @details 각 자세 사이의 경로를 생성한 후, 역기구학(inverse kinematics)을 통해 카테시안 공간(cartesian space) 상의 경로를 관절 공간(joint space)으로 변환한다. 여기서 사용된 역기구학은 대상 로봇인 UR5의 closed-form 수식을 사용하였다. 이때 추출된 경로의 관절각도에 대해 자코비안을 계산하여 도달 불가능한 자세를 검사한다.
 * @param[in] pose_set : 입력된 자세 set, [m], [rad]
 */
void PATHPLANNING::trajectoryPlanning(const Eigen::VectorXf &P_cmd, const Eigen::VectorXf &P_init, std::vector<Eigen::VectorXf> &traj)
{
	double controlPeriod = 0.001;
	Eigen::VectorXf dis = P_cmd-P_init;

	m_dCurrentTime = 0.0;
	m_dCurrentPos = P_init;
	m_dTimeInit = m_dCurrentTime;
	m_dTimeFin = m_dTimeInit + 1.0;
	m_iPathMode = NOPATH;

	// genTrapPath(dis); // 사용 X, 코드 동작 X
    printf("---> Target_pose(trajectory): %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", P_cmd[0], P_cmd[1], P_cmd[2], (180.0/M_PI)*P_cmd[3], (180.0/M_PI)*P_cmd[4], (180.0/M_PI)*P_cmd[5]);
    genPolyPath(dis); // [rad]

	std::vector<Eigen::VectorXf> trajectory;
	trajectory.clear();
	int count = 0;
	double time = 0.0;
    std::vector<double> desired_x(6);
	while (m_iPathMode)
	{
		count = count + 1;
		time = 0.001 * static_cast<double>(count);

		calPath(time);
        // printf("Target_path: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", m_dPosDes[0], m_dPosDes[1], m_dPosDes[2], m_dPosDes[3], m_dPosDes[4], m_dPosDes[5]);
		// Eigen::VectorXf PosDes_tmp = (m_dPosDes - P_init).norm()*vec_dis + P_init;
		// trajectory.push_back(PosDes_tmp);

		trajectory.push_back(m_dPosDes);
	}
	traj = trajectory;
}

/** @brief Korean: 초기 자세(P_init)로부터 목표 자세(P_cmd)에 도달하기 위한 직선 경로를 생성한다.
 * @details 각 자세 사이의 경로를 생성한 후, 역기구학(inverse kinematics)을 통해 카테시안 공간(cartesian space) 상의 경로를 관절 공간(joint space)으로 변환한다. 여기서 사용된 역기구학은 대상 로봇인 UR5의 closed-form 수식을 사용하였다. 이때 추출된 경로의 관절각도에 대해 자코비안을 계산하여 도달 불가능한 자세를 검사한다.
 * @param[in] pose_set : 입력된 자세 set, [m], [rad]
 */
void PATHPLANNING::trajectoryPlanning2(double control_period, const Eigen::VectorXf &P_cmd, const Eigen::VectorXf &P_init, std::vector<Eigen::VectorXf> &traj)
{
	// double controlPeriod = 0.001;
	Eigen::VectorXf dis = P_cmd-P_init;

	m_dCurrentTime = 0.0;
	m_dCurrentPos = P_init;
	m_dTimeInit = m_dCurrentTime;
	m_dTimeFin = m_dTimeInit + 1.0;
	m_iPathMode = NOPATH;

	// genTrapPath(dis); // 사용 X, 코드 동작 X
    // printf("---> Target_pose(trajectory): %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", P_cmd[0], P_cmd[1], P_cmd[2], (180.0/M_PI)*P_cmd[3], (180.0/M_PI)*P_cmd[4], (180.0/M_PI)*P_cmd[5]);
    genPolyPath(dis); // [rad]

    double tmp = (m_dTimeFin-m_dTimeInit)/control_period + 1.0;
    tmp = floor(tmp);
    size_t traj_num = static_cast<size_t>(tmp);

    traj.clear();
    traj.reserve(traj_num);
	double count = 0.0;
	double time = 0.0;

	while (m_iPathMode)
	{
		count = count + 1.0;
		time = control_period * count;

		calPath(time);
        // printf("Target_path: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", m_dPosDes[0], m_dPosDes[1], m_dPosDes[2], m_dPosDes[3], m_dPosDes[4], m_dPosDes[5]);
		// Eigen::VectorXf PosDes_tmp = (m_dPosDes - P_init).norm()*vec_dis + P_init;
		// trajectory.push_back(PosDes_tmp);

		traj.push_back(m_dPosDes);
	}
}

/** @brief Korean: 입력된 자세로 초기 자세를 설정하고 파라미터를 초기화한다.
 * @param[in] Pos : 입력 자세
 */
void PATHPLANNING::setInitPos(const Eigen::VectorXf &Pos, bool mode)
{
    Eigen::VectorXf dPosInit(ROBOT_DOF_INSP);
    Eigen::VectorXf dCurrentPos(ROBOT_DOF_INSP);
    Eigen::VectorXf dCurrentVel(ROBOT_DOF_INSP);
    Eigen::VectorXf dCurrentAcc(ROBOT_DOF_INSP);
    Eigen::VectorXf dPosFin(ROBOT_DOF_INSP);
    Eigen::VectorXf dPosDes(ROBOT_DOF_INSP);
    Eigen::VectorXf dVelDes(ROBOT_DOF_INSP);
    Eigen::VectorXf dAccDes(ROBOT_DOF_INSP);
    Eigen::VectorXf dVelMax(ROBOT_DOF_INSP);
    Eigen::VectorXf dVelUni(ROBOT_DOF_INSP);
    Eigen::VectorXf dTimeAcc(ROBOT_DOF_INSP);
    Eigen::VectorXf bDirection(ROBOT_DOF_INSP);

    for (int i = 0; i<ROBOT_DOF_INSP; i++)
	{
		dPosInit[i] = Pos[i];
		dCurrentPos[i] = Pos[i];
		dCurrentVel[i] = 0.0;
		dPosFin[i] = Pos[i];
		dPosDes[i] = Pos[i];
		dVelDes[i] = 0.0;
		dAccDes[i] = 0.0;
		dVelMax[i] = 0.0;
		dTimeAcc[i] = 0.0;
		bDirection[i] = 0.0;
	}

	m_dPosInit = dPosInit;
	m_dCurrentPos = dCurrentPos;
	m_dCurrentVel = dCurrentVel;
	m_dCurrentAcc = dCurrentAcc;
	m_dPosFin = dPosFin;
	m_dPosDes = dPosDes;
	m_dVelDes = dVelDes;
	m_dAccDes = dAccDes;
	m_dVelMax = dVelMax;
    m_dVelUni = dVelUni;
	m_dTimeAcc = dTimeAcc;
	m_bDirection = bDirection;
	m_iPathMode = NOPATH;
	m_dTimeInit = 0.0;
	m_dCurrentTime = 0.0;
	m_dTimeFin = m_dTimeInit + 1.0;
    m_bSpace = mode;
}

/** @brief Korean: 입력된 속도를 설정한다.
 * @param[in] Vel : 입력 속도
 */
void PATHPLANNING::setMaxVelocity(const Eigen::VectorXf &Vel)
{
    m_dVelMax.resize(ROBOT_DOF_INSP);
    for (int i = 0; i < ROBOT_DOF_INSP; i++) { m_dVelMax[i] = Vel[i]; }
}

/** @brief Korean: 입력된 가속도를 설정한다.
 * @param[in] Acc : 입력 가속도
 */
void PATHPLANNING::setAcceleration(const Eigen::VectorXf &Acc)
{
    m_dAcc.resize(ROBOT_DOF_INSP);
    for (int i = 0; i < ROBOT_DOF_INSP; i++) { m_dAcc[i] = Acc[i]; }
}

/** @brief Korean: 두 자세 사이의 변위를 이용하여 trapizoidal path 생성을 위한 파라미터를 설정한다.
 * @param[in] Distance : 두 자세 사이의 변위
 */
void PATHPLANNING::genTrapPath(const Eigen::VectorXf &Distance)
{
	double dDistance[6] = { 0.0 };
	for (int i = 0; i < 6; i++) { dDistance[i] = Distance[i]; }

	// 각도 변경과정에서 발산하는 문제 해결
	for (int i = 3; i<6; i++)
	{
		if (dDistance[i] > 180) { dDistance[i] = dDistance[i] - 360; }
		else if (dDistance[i] < -180) { dDistance[i] = dDistance[i] + 360; }
	}

	// 움직이는 방향검사
	double AbsD[6] = { 0.0 }; // 거리 절대값
	for (int i = 0; i<6; i++)
	{
		if (dDistance[i] >= 0)
		{
			m_bDirection[i] = true;
			AbsD[i] = dDistance[i];
		}
		else
		{
			m_bDirection[i] = false;
			AbsD[i] = -dDistance[i];
		}
	}

	// 최대시간 검사
	double time = 0.0;
	double temp = 0.0;
	for (int i = 0; i<6; i++)
	{
		if ((AbsD[i] / m_dVelMax[i]) >= (m_dVelMax[i] / m_dAcc[i]))
		{
			temp = AbsD[i] / m_dVelMax[i] + m_dVelMax[i] / m_dAcc[i];
			if (temp > time) { time = temp; }
		}
		else
		{
			temp = AbsD[i] / m_dAcc[i];
			temp = 2 * sqrt(temp*temp);
			if (temp > time) { time = temp; }
		}
	}
	// 최대 속도와 가속시간 연산
	for (int i = 0; i<6; i++)
	{
		temp = m_dAcc[i] * m_dAcc[i] * time*time - 4 * m_dAcc[i] * AbsD[i];
		if (temp <= 0.0) { temp = 0.0; }
		m_dVelMax[i] = (m_dAcc[i] * time - sqrt(temp)) / 2.0; //최대 속도
		m_dTimeAcc[i] = m_dVelMax[i] / m_dAcc[i]; //가속 시간
	}

	// Initial and Final time 설정 
	for (int i = 0; i<6; i++)
	{
		m_dPosInit[i] = m_dCurrentPos[i];
		m_dPosFin[i] = m_dPosInit[i] + dDistance[i];
	}
	m_dTimeInit = m_dCurrentTime;
	m_dTimeFin = m_dTimeInit + time;

	// mode 설정
	m_iPathMode = TRAPEZOID; //Path Mode
}

/** @brief Korean: 경로를 계산한다.
 * @param[in] t : 입력 시간(control period)
 */
void PATHPLANNING::calTrapPath(double t)
{
	unsigned int i = 0;
	for (int i = 0; i<6; i++)
	{
		if (t < m_dTimeInit)
		{
			m_dPosDes[i] = m_dPosInit[i];
		}
		else if (t < (m_dTimeInit + m_dTimeAcc[i]))
		{
			if (m_bDirection[i])
			{
				m_dPosDes[i] = m_dPosInit[i] + m_dAcc[i] * (t - m_dTimeInit)*(t - m_dTimeInit) / 2.0;
			}
			else
			{
				m_dPosDes[i] = m_dPosInit[i] - m_dAcc[i] * (t - m_dTimeInit)*(t - m_dTimeInit) / 2.0;
			}
		}
		else if (t < (m_dTimeFin - m_dTimeAcc[i]))
		{
			if (m_bDirection[i])
			{
				m_dPosDes[i] = m_dPosInit[i] + m_dVelMax[i] * (t - m_dTimeInit - m_dTimeAcc[i] / 2.0);
			}
			else
			{
				m_dPosDes[i] = m_dPosInit[i] - m_dVelMax[i] * (t - m_dTimeInit - m_dTimeAcc[i] / 2.0);
			}
		}
		else if (t < (m_dTimeFin))
		{
			if (m_bDirection[i])
			{
				m_dPosDes[i] = m_dPosFin[i] - m_dAcc[i] * (m_dTimeFin - t)*(m_dTimeFin - t) / 2.0;
			}
			else
			{
				m_dPosDes[i] = m_dPosFin[i] + m_dAcc[i] * (m_dTimeFin - t)*(m_dTimeFin - t) / 2.0;
			}
		}
		else
		{
			m_dPosDes[i] = m_dPosFin[i];
			genNoPath(m_dPosFin);
		}
	}
}

/** @brief Korean: S-curve 기반의 궤적을 생성한다.
 * @param[in] Distance : 현재 위치와 목표 위치 사이의 변위
 */
void PATHPLANNING::genPolyPath(const Eigen::VectorXf &Distance) {
    int PathDOF;
    if (m_bSpace == true) PathDOF = 6; // CS 모드면 Path 6자유도
    else PathDOF = m_nPathDOFs; // JS 모드면 Path 사용자 지정 자유도 (최대 20)

    //Function input
    double dDistance[PATH_DOFS] = {0.0};
    for (int i = 0; i < PathDOF; i++) dDistance[i] = Distance[i];

    // CS 모드일 때, 각도 변경과정에서 발산하는 문제 해결
    if (m_bSpace == true) {
        for (int i = 3; i < 6; i++) {
            if (dDistance[i] > M_PI) dDistance[i] = dDistance[i] - 2 * M_PI;
            else if (dDistance[i] < -M_PI) dDistance[i] = dDistance[i] + 2 * M_PI;
        }
    }

    //현재 속도 정보를 획득
    double dVelInit[PATH_DOFS] = {0.0};
    for (int i = 0; i < PathDOF; i++) dVelInit[i] = m_dCurrentVel[i];

    // 최대가속도를 2/3으로 설정
    double dAcc[PATH_DOFS] = {0.0};
    for (int i = 0; i < PathDOF; i++) dAcc[i] = m_dAcc[i] * 2.0 / 3.0;

    //triangular trajectory로 구동했을 시의 최대 속도 연산
    double dVelMax[PATH_DOFS] = {0.0};
    for (int i = 0; i < PathDOF; i++) {
        if (dDistance[i] >= (fabs(dVelInit[i])*dVelInit[i] / 2.0 / dAcc[i]))
            dVelMax[i] =  sqrt(fabs(dVelInit[i] * dVelInit[i] / 2.0 + dAcc[i] * dDistance[i]));
        else
            dVelMax[i] = -sqrt(fabs(dVelInit[i] * dVelInit[i] / 2.0 - dAcc[i] * dDistance[i]));
    }

    //dVelMax와 m_dMaxVel과 비교하여Uniform velocity dVelUni값을 설정
    double dVelUni[PATH_DOFS] = {0.0};
    for (int i = 0; i < PathDOF; i++) {
        if (dVelMax[i] >= m_dVelMax[i])
            dVelUni[i] = m_dVelMax[i];
        else if (dVelMax[i] <= (-m_dVelMax[i]))
            dVelUni[i] = -m_dVelMax[i];
        else
            dVelUni[i] = dVelMax[i];
    }

    //dVelUni에 따라 각 방향 동작시간을 비교하여 최대 동작 시간 dTimeFinal를 구한다.
    double dTime[3] = {0.0001, 0.0001, 0.0001}; // 최소 구동시간 설정 (100 us)
    double dTemp = 0.0;
    for (int i = 0; i < PathDOF; i++) {
        // 가속 시간
        dTemp = fabs(dVelUni[i] - dVelInit[i]) / dAcc[i];
        if (dTemp > dTime[0]) dTime[0] = dTemp;

        // 감속 시간
        dTemp = fabs(dVelUni[i]) / dAcc[i];
        if (dTemp > dTime[2]) dTime[2] = dTemp;

        // 등속 시간
        if (dTemp < 0.0001) { // 감속구간이 짧은 경우 --> 등속구간이 필요없는 경우
            dTemp = 0.0001;
        } else {
            dTemp = (dDistance[i] - (dVelUni[i] + dVelInit[i]) / 2.0 * fabs(dVelUni[i] - dVelInit[i]) / dAcc[i] - dVelUni[i] / 2.0 * fabs(dVelUni[i]) / dAcc[i]) / dVelUni[i];
        }
        if (dTemp > dTime[1]) dTime[1] = dTemp;

    }

    // 최대 동작 시간 dTime[3]에 따라 각 방향의 dVelUni를 다시 구한다.
    for (int i = 0; i < PathDOF; i++) {
        dVelUni[i] = (dDistance[i] - dVelInit[i] * dTime[0] / 2.0) / (dTime[0] / 2.0 + dTime[1] + dTime[2] / 2.0);
    }

    // 실시간 경로 연산을 위해서는 dVelInit, dVelUni, dTimeInit, dTime1, dTime2, dTimeFinal, 을 저장해야 한다.
    for (int i = 0; i < PathDOF; i++) {
        m_dPosInit[i] = m_dCurrentPos[i];
        m_dPosFin[i] = m_dPosInit[i] + dDistance[i];
        m_dVelUni[i] = dVelUni[i];
        // m_dVelInit[i] = dVelInit[i];
    }

    m_dTimeInit = m_dCurrentTime;
    m_dTimeFin = m_dTimeInit + dTime[0] + dTime[1] + dTime[2];

    m_nPolyPathNum = 3;

    m_dPolyPathTime[0][0] = m_dCurrentTime;
    m_dPolyPathTime[0][1] = m_dPolyPathTime[0][0] + dTime[0];

    m_dPolyPathTime[1][0] = m_dPolyPathTime[0][1];
    m_dPolyPathTime[1][1] = m_dPolyPathTime[1][0] + dTime[1];

    m_dPolyPathTime[2][0] = m_dPolyPathTime[1][1];
    m_dPolyPathTime[2][1] = m_dTimeFin;

    for (int i = 0; i < PathDOF; i++) {
        double dTemp0, dTemp1;
        dTemp0 = 0.0;
        dTemp1 = dVelUni[i] - dVelInit[i];
        m_dPolyPathCoff[0][i][0] = m_dPosInit[i];
        m_dPolyPathCoff[0][i][1] = dVelInit[i];
        m_dPolyPathCoff[0][i][2] = 0.0;
        m_dPolyPathCoff[0][i][3] = dTemp1 / pow(dTime[0], 2);
        m_dPolyPathCoff[0][i][4] = -dTemp1 / (2.0 * pow(dTime[0], 3));
        m_dPolyPathCoff[0][i][5] = dTemp0;

        dTemp0 = 0.0;
        dTemp1 = 0.0;
        m_dPolyPathCoff[1][i][0] = (dVelUni[i] + dVelInit[i]) * dTime[0] / 2.0 + m_dPosInit[i];
        m_dPolyPathCoff[1][i][1] = dVelUni[i];
        m_dPolyPathCoff[1][i][2] = 0.0;
        m_dPolyPathCoff[1][i][3] = dTemp0;
        m_dPolyPathCoff[1][i][4] = dTemp0;
        m_dPolyPathCoff[1][i][5] = dTemp0;

        dTemp0 = 0.0;
        dTemp1 = dVelUni[i];
        m_dPolyPathCoff[2][i][0] = m_dPosFin[i] - (dVelUni[i]) * dTime[2] / 2.0;
        m_dPolyPathCoff[2][i][1] = dVelUni[i];
        m_dPolyPathCoff[2][i][2] = 0.0;
        m_dPolyPathCoff[2][i][3] = -dTemp1 / pow(dTime[2], 2);
        m_dPolyPathCoff[2][i][4] = dTemp1 / (2.0 * pow(dTime[2], 3));
        m_dPolyPathCoff[2][i][5] = dTemp0;
    }

    // mode 설정
    m_iPathMode = POLYNOMIAL;    //Path Mode
}

/** @brief Korean: 생성된 S-curve 궤적으로부터 현재 시간의 목표 위치, 속도, 가속도를 산출한다.
 * @param[in] t : 현재 시간, [s]
 */
void PATHPLANNING::calPolyPath(double t) {
    int PathDOF;
    if (m_bSpace == true) PathDOF = 6; 
    else PathDOF = m_nPathDOFs;
    for (int i = 0; i < PathDOF; i++) {
        if (t < m_dTimeInit) {     
            m_dPosDes[i] = m_dPosInit[i];
            m_dVelDes[i] = 0.0;
            m_dAccDes[i] = 0.0;
        } else if (t < (m_dTimeFin)) { 
            for (unsigned int k = 0; k < m_nPolyPathNum; k++) {
                if ((m_dPolyPathTime[k][0] <= t) && (t < m_dPolyPathTime[k][1])) {
                    m_dPosDes[i] = 0;
                    m_dVelDes[i] = 0;
                    m_dAccDes[i] = 0;
                    for (int j = 0; j < 6; j++) m_dPosDes[i] += m_dPolyPathCoff[k][i][j] * pow(t - m_dPolyPathTime[k][0], j); // 위치
                    for (int j = 1; j < 6; j++) m_dVelDes[i] += j * m_dPolyPathCoff[k][i][j] * pow(t - m_dPolyPathTime[k][0], j - 1); // 속도
                    for (int j = 2; j < 6; j++) m_dAccDes[i] += (j * (j - 1)) * m_dPolyPathCoff[k][i][j] * pow(t - m_dPolyPathTime[k][0], j - 2); // 가속도
                }
            }
        } else {
            m_dPosDes[i] = m_dPosFin[i];
            m_dVelDes[i] = 0.0;
            m_dAccDes[i] = 0.0;
            // Stop
            genNoPath(m_dPosFin);
        }
    }
}

/** @brief Korean: 설정된 Path mode에 따라 목표 자세, 속도, 가속도를 계산한다.
 * @param[in] t : 입력 시간(control period)
 */
void PATHPLANNING::calPath(double t)
{
	switch (m_iPathMode)
	{
	case NOPATH:
        for (int i = 0; i<ROBOT_DOF_INSP; i++)
		{
			m_dPosDes[i] = m_dPosInit[i];
			m_dVelDes[i] = 0.0;
			m_dAccDes[i] = 0.0;
		}
		break;
    case POLYNOMIAL: // S-curve 궤적
        calPolyPath(t);
        break;
	case TRAPEZOID:
		calTrapPath(t);
		break;

	default:
		;
	}

    for (int i = 0; i<ROBOT_DOF_INSP; i++)
	{
		m_dCurrentPos[i] = m_dPosDes[i];
		m_dCurrentVel[i] = m_dVelDes[i];
		m_dCurrentAcc[i] = m_dAccDes[i];
	}
	m_dCurrentTime = t;
}

/** @brief Korean: 입력 자세를 초기 자세로 설정한 후, path mode를 NOPATH로 설정(경로생성 x)한다.
 * @param[in] Pos : 입력 자세
 */
void PATHPLANNING::genNoPath(const Eigen::VectorXf &Pos)
{
    for (int i = 0; i < ROBOT_DOF_INSP; i++) { m_dPosInit[i] = Pos[i]; }
	m_iPathMode = NOPATH;
}

/** @brief Korean: 입력된 작업공간(task space)의 자세를 UR5 로봇의 역기구학(closed-form )을 이용하여 관절공간의 자세로 변환한다.
 * @param[in] q_past : 이전 관절 각도
 * @param[in] x_current : 목표 자세
 * @param[out] q_current : 역기구학을 통해 계산된 다음 관절 각도
 */
int PATHPLANNING::inverseClosedForm(const Eigen::VectorXf &q_past, Eigen::VectorXf &x_current, Eigen::VectorXf &q_current)
{
    double q_past_radian[ROBOT_DOF_INSP] = { 0.0, };
    double q_plus[ROBOT_DOF_INSP] = { 0.0, };
    double q_minus[ROBOT_DOF_INSP] = { 0.0, };
    double q_find[8][ROBOT_DOF_INSP] = { { 0.0, }, };
	double q_sum[8] = { 0.0, };
	int reach_out[8] = { 0, };
	int reach_in[8] = { 0, };
	int number = 0;

    for (int i = 0; i < ROBOT_DOF_INSP; i++) { q_past_radian[i] = q_past[i] * (M_PI / 180.0); }
	Eigen::Matrix4f T06;
	pose2HTM(x_current, T06);

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
        double q_temp[ROBOT_DOF_INSP] = { 0.0, };
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
			for (int k = 0; k < 3; k++) 
			{
				p13[k] = T14(k, 3) - m_DH[3][2] * T14(k, 1);
			}

			double theta3_check = 0.0;
			double theta3_constraint = (p13[0] * p13[0] + p13[1] * p13[1] - m_DH[1][0] * m_DH[1][0] - m_DH[2][0] * m_DH[2][0]) / (2 * m_DH[1][0] * m_DH[2][0]);
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

				double theta2_constraint = checkFloatingError(m_DH[2][0] * sin(q_temp[2]), (sqrt(p13[0] * p13[0] + p13[1] * p13[1])));
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
			if (j == 5)
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

	for (int i = 0; i < 6; i++) { q_current[i] = q_find[q_number][i] * (180.0/M_PI); }
	if (reach_in[q_number] == 0 && reach_out[q_number] == 0) { return 0; } // singular error가 없을 경우
	else if ((reach_in[q_number] != 0) && (reach_out[q_number] == 0)) {	return 1; } //q1의 singular error
	else if ((reach_out[q_number] != 0) && (reach_in[q_number] == 0)) { return 2; }
	else { return 3; } // q1, q3의 singular error가 동시에 발생
}

/** @brief Korean: 자세를 변환행렬로 형식으로 변환한다.
 * @param[in] x : 작업공간의 자세 (x, y, z, Rx, Ry, Rz)
 * @param[out] T_out : 변환행렬 형식의 자세
 */
void PATHPLANNING::pose2HTM(const Eigen::VectorXf &x, Eigen::Matrix4f &T_out)
{
	T_out = Eigen::Matrix4f::Identity();
	Eigen::Matrix3f rotM_tmp = Eigen::Matrix3f::Identity();
	T_out(0, 3) = x[0];
	T_out(1, 3) = x[1];
	T_out(2, 3) = x[2];
	RPY2Rot(x.segment(3, 3), rotM_tmp);
	T_out.block<3, 3>(0, 0) = rotM_tmp;
}

/** @brief Korean: RPY를 회전행렬로 변환한다.
 * @param[in] dRPY : Roll-Pitch-Yaw (Rx, Ry, Rz)
 * @param[out] rotM : 회전행렬
 */
void PATHPLANNING::RPY2Rot(const Eigen::Vector3f &dRPY, Eigen::Matrix3f &rotM)
{
	double dRot[3][3] = { 0.0, };
	dRot[0][0] = cos(dRPY[2] * (M_PI/180.0)) * cos(dRPY[1] * (M_PI/180.0));
	dRot[0][1] = cos(dRPY[2] * (M_PI/180.0)) * sin(dRPY[1] * (M_PI/180.0)) * sin(dRPY[0] * (M_PI/180.0)) - sin(dRPY[2] * (M_PI/180.0)) * cos(dRPY[0] * (M_PI/180.0));
	dRot[0][2] = cos(dRPY[2] * (M_PI/180.0)) * sin(dRPY[1] * (M_PI/180.0)) * cos(dRPY[0] * (M_PI/180.0)) + sin(dRPY[2] * (M_PI/180.0)) * sin(dRPY[0] * (M_PI/180.0));

	dRot[1][0] = sin(dRPY[2] * (M_PI/180.0)) * cos(dRPY[1] * (M_PI/180.0));
	dRot[1][1] = sin(dRPY[2] * (M_PI/180.0)) * sin(dRPY[1] * (M_PI/180.0)) * sin(dRPY[0] * (M_PI/180.0)) + cos(dRPY[2] * (M_PI/180.0)) * cos(dRPY[0] * (M_PI/180.0));
	dRot[1][2] = sin(dRPY[2] * (M_PI/180.0)) * sin(dRPY[1] * (M_PI/180.0)) * cos(dRPY[0] * (M_PI/180.0)) - cos(dRPY[2] * (M_PI/180.0)) * sin(dRPY[0] * (M_PI/180.0));

	dRot[2][0] = -sin(dRPY[1] * (M_PI/180.0));
	dRot[2][1] = cos(dRPY[1] * (M_PI/180.0)) * sin(dRPY[0] * (M_PI/180.0));
	dRot[2][2] = cos(dRPY[1] * (M_PI/180.0)) * cos(dRPY[0] * (M_PI/180.0));

	for (int i = 0; i < 3; i++) { for (int j = 0; j < 3; j++) rotM(i, j) = dRot[i][j]; }
}

/** @brief Korean: 로봇의 순기구학(forward kinematics)을 계산한다.
 * @param[in] n : 로봇의 자유도
 * @param[in] q : 관절각도 [rad]
 * @return T_out : 순기구학의 결과로 변환행렬을 반환
 */
Eigen::Matrix4f PATHPLANNING::TransformationMatrix(int n, double q) // [rad]
{
	Eigen::Matrix4f T_out = Eigen::Matrix4f::Identity();
	double dSinQ = sin(q);
	double dCosQ = cos(q);
	double dSinAlpha = sin(m_DH[n][1]);
	double dCosAlpha = cos(m_DH[n][1]);
	double a = m_DH[n][0];
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

/** @brief Korean: 로봇의 geometric Jacobian을 계산한다.
 * @param[in] n : 로봇의 자유도
 * @param[in] q : 관절각도 [rad]
 */
void PATHPLANNING::calJacobianGeo(const Eigen::VectorXf &q_deg, Eigen::MatrixXf &J_geo)
{
    // printf("Here 1-1-0-1!!\n");
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> matTrans;
    // printf("Here 1-1-0-2!!\n");
    matTrans.resize(ROBOT_DOF_INSP);
    // printf("Here 1-1-0-3!!\n");
	double q = 0.0;
    for (int i = 0; i < ROBOT_DOF_INSP; i++)
	{
        // printf("Here 1-1-0-4!!\n");
		matTrans[i] = Eigen::Matrix4f::Identity();
        // printf("Here 1-1-0-5!!\n");
		q = q_deg[i] * (M_PI / 180.0);
        // printf("Here 1-1-0-6!!\n");
		matTrans[i] = TransformationMatrix(i, q);
        // printf("Here 1-1-0-7!!\n");
	}

    // printf("Here 1-1-1!!\n");

    Eigen::MatrixXf matJ(6, ROBOT_DOF_INSP);
	matJ.block<3, 3>(0, 0) = Eigen::Matrix3f::Zero();
	matJ.block<3, 3>(3, 3) = Eigen::Matrix3f::Zero();
	matJ.block<3, 3>(0, 3) = Eigen::Matrix3f::Zero();
	matJ.block<3, 3>(3, 0) = Eigen::Matrix3f::Zero();

	Eigen::Matrix4f matU = Eigen::Matrix4f::Identity();
    // printf("Here 1-1-2!!\n");
	
    for (int i = ROBOT_DOF_INSP-1; i >= 0; i--)
	{

		matU = matTrans[i] * matU;
        // printf("Here 1-1-3!!\n");

		Eigen::Vector3f vec_p;
		vec_p[0] = -matU(0, 0)*matU(1, 3) + matU(1, 0)*matU(0, 3);
		vec_p[1] = -matU(0, 1)*matU(1, 3) + matU(1, 1)*matU(0, 3);
		vec_p[2] = -matU(0, 2)*matU(1, 3) + matU(1, 2)*matU(0, 3);
        // printf("Here 1-1-4!!\n");

		Eigen::Vector3f vec_r;
		vec_r[0] = matU(2, 0);
		vec_r[1] = matU(2, 1);
		vec_r[2] = matU(2, 2);
        // printf("Here 1-1-5!!\n");

		matJ.block<3, 1>(0, i) = vec_p;
		matJ.block<3, 1>(3, i) = vec_r;
        // printf("Here 1-1-5!!\n");

	}
	J_geo = matJ;
}

/** @brief Korean: 로봇의 geometric Jacobian을 이용하여 singularity 여부를 반환한다.
 * @details 특이값 분해(SVD) 방법을 이용하여 입력된 geometric Jacobian의 condition number(최대 특이값을 최소 특이값으로 나눈 값)를 계산한다. 이때, condition number가 주어진 임계값보다 크면 singularity가 발생한 것으로 판별되며 이는 로봇이 도달할 수 없는 자세임을 의미한다.
 * @param[in] matJ : 현재 관절각도에서의 geometric Jacobian
 * @param[in] thre_cond : condition number의 임계 파라미터
 * @return 로봇 자세의 특이점(singularity) 여부를 반환
 * @retval true : singular
 * @retval false : not singular
 */
bool PATHPLANNING::checkJacobianCondNum(const Eigen::MatrixXf &matJ, double thre_cond)
{
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(matJ, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::MatrixXf mat_sv = svd.singularValues(); 
	std::vector<double> sglVal;
    for (int i = 0; i<ROBOT_DOF_INSP; i++) {	sglVal.push_back(mat_sv(i, 0)); }
	int idx_tmp = 0;
	double sgl_max = 0.0;
	double sgl_min = 0.0;
	m_math.genVecMinMax(sglVal, idx_tmp, sgl_max, 1); // max
	m_math.genVecMinMax(sglVal, idx_tmp, sgl_min, 0); // min

	if (sgl_min == 0.0) { return true; }
	double condNum = 0.0;
	condNum = sgl_max / sgl_min;
	if (condNum > thre_cond) {	return true; }
	else { return false; }
}
double PATHPLANNING::checkFloatingError(double v1, double v2)
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
bool PATHPLANNING::checkEquality(double value, double match)
{
	return fabs(value - match) < 0.000001;
}


/** @brief Korean: 시야계획의 CAD 파라미터를 입력한다.
* @param[in] T_T2S :
*/
void PATHPLANNING::setPathPlanningParameters(CPATHDATA &path_data)
{
	m_T_S2T = path_data.T_S2T;
}

// /** @brief Korean: 로봇의 TCP 정보를 입력한다.
//  * @param[in] tcp : 로봇의 TCP 정보, [m], [rad]
//  */
// void PATHPLANNING::setRobotTCP(std::vector<double> tcp) 
// {
//     m_math.SetTcp(tcp);
// }


// /** @brief Korean: 입력된 센서의 시야점, 법선, 중심을 이용하여 센서 좌표계 기준 자세를 산출한다.
// * @details metric: [m], [deg]
// * @param[in] path_data : 경로계획을 위한 데이터 객체
// */
// bool PATHPLANNING::checkRobotSingularPose(CVIEWPOINTDATA &vp_data, CPATHDATA &path_data, const std::vector<double> pose_in, Eigen::VectorXf &q_prev, std::vector<double> &robot_acc, std::vector<double> &robot_vel)
// {
//     printf("target pose input: %f %f %f %f %f %f\n", pose_in[0], pose_in[1], pose_in[2], pose_in[3], pose_in[4], pose_in[5]);
// 	Eigen::VectorXf P_tgt(6);
//     for (int i = 0; i < 6; i++)
//     {
//         if(i<3) P_tgt[i] = pose_in[i]; // [m]
//         else P_tgt[i] = (M_PI/180.0) *pose_in[i]; // [rad]

//     }
//     printf("target pose: %f %f %f %f %f %f\n", P_tgt(0), P_tgt(1), P_tgt(2), P_tgt(3), P_tgt(4), P_tgt(5));

// 	if (!checkPath2TargetPose(P_tgt, q_prev, robot_acc, robot_vel)) // P_tgt: [m], [rad]
// 	{
// 		Eigen::VectorXf q_now(6); // 이전 관절각도
// 		inverseClosedForm(q_prev, P_tgt, q_now);
// 		// update
// 		vp_data.is_path_feasible = true;
// 		vp_data.target_CS_pose = P_tgt; // robot poses
// 		vp_data.target_JS_position = q_now; // robot JS angle
// 		// vp_data.target_rs_angle = rotation_angle; // rotation stage target angle
//         printf("target_JS_position: %f %f %f %f %f %f\n", q_now(0), q_now(1), q_now(2), q_now(3), q_now(4), q_now(5));
		
// 		printf("Path check Success!\n");
// 		return true;
// 	}
// 	else
// 	{
// 		vp_data.is_path_feasible = false;
// 		printf("Path check Failure!\n");
// 		return false;
// 	}

//     return true;
// }

// /** @brief Korean: 입력된 자세의 측정 경로를 검사한다.
// * @details 각 자세 사이의 경로를 생성한 후, 역기구학(inverse kinematics)을 통해 카테시안 공간(cartesian space) 상의 경로를 관절 공간(joint space)으로 변환한다. 여기서 사용된 역기구학은 대상 로봇인 UR5의 closed-form 수식을 사용하였다. 이때 추출된 경로의 관절각도에 대해 자코비안을 계산하여 도달 불가능한 자세를 검사한다.
// * @param[in] pose_set : 입력된 자세 set
// * @param[in] q_init: 로봇의 초기 관절 각도 [deg]
// */
// bool PATHPLANNING::checkPath2TargetPose(const Eigen::VectorXf &pose_in, const Eigen::VectorXf q_init, std::vector<double> &robot_acc, std::vector<double> &robot_vel)
// {
// 	bool b_singularity = false;
// 	Eigen::VectorXf P_init(6);
// 	Eigen::Matrix4f T_init;
// 	Eigen::VectorXf dAcc(6); // [m/s^2], [deg/s^2]
// 	Eigen::VectorXf dVel(6); // [m/s], [deg/s]
// 	for (size_t i = 0; i < robot_acc.size(); i++) { dAcc[i] = robot_acc[i]; }
// 	for (size_t i = 0; i < robot_vel.size(); i++) { dVel[i] = robot_acc[i]; }
// 	Eigen::VectorXf q_prev = q_init;
// 	Eigen::VectorXf q_current(6);
// 	Eigen::VectorXf pose_tgt = pose_in; // [m], [deg]
// 	for (int j = 3; j < 6; j++) { pose_tgt[j] = (180.0 / M_PI)*pose_tgt[j]; } // [deg]

// 	// Forward kinematics
//     printf("q_init: %f %f %f %f %f %f\n", q_init(0), q_init(1), q_init(2), q_init(3), q_init(4), q_init(5));

// 	m_math.forwardKinematics(m_DH, q_init, T_init);
// 	m_math.tr2pose(T_init, P_init); // [m], [deg]
// 	for (int j = 3; j < 6; j++) { P_init[j] = (180.0 / M_PI)*P_init[j]; } // [deg]

//     printf("P_init: %f %f %f %f %f %f\n", P_init(0), P_init(1), P_init(2), P_init(3), P_init(4), P_init(5));

// 	// Trajectory planning
// 	setInitPos(P_init);
// 	setAcceleration(dAcc);
// 	setMaxVelocity(dVel);
// 	std::vector<Eigen::VectorXf> trajectory;
// 	trajectoryPlanning(pose_tgt, P_init, trajectory); // [m], [rad]

//     std::cout << "trajectory num: " << trajectory.size() << std::endl;
//     std::cout << P_init << std::endl;

// 	for (size_t j = 0; j < trajectory.size(); j++)
// 	{
// 		// Inverse kinematics
// 		inverseClosedForm(q_prev, trajectory[j], q_current);
// 		// if (j == 0 || j == trajectory.size() - 1) { std::cout << j << "th angle:" << std::endl << q_current << std::endl; }
// 		// std::cout << j << "th angle:" << std::endl << q_current << std::endl;
//         printf("q_current(#%zu): %f %f %f %f %f %f\n", j, q_current(0), q_current(1), q_current(2), q_current(3), q_current(4), q_current(5));
// 		// Jacobian check
// 		Eigen::MatrixXf matJ_check;
//         // printf("Here 1-1!!\n");
// 		calJacobianGeo(q_current, matJ_check);
//         // printf("Here 1-2!!\n");
// 		if (checkJacobianCondNum(matJ_check, 100.0)) { b_singularity = true; }
// 		q_prev = q_current;
// 	}
// 	// singularity check
// 	if (b_singularity) { printf("Singularity!!\n"); }
// 	else { printf("Path is feasible\n"); }
// 	return b_singularity;
// }
