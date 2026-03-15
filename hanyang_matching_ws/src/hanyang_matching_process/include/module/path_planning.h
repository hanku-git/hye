/**
 * @file path_planning.h
 * @brief 로봇의 경로계획을 위한 헤더파일
 */
#ifdef max
#undef max
#undef min
#endif

#pragma once

// #include "stdafx.h"
#include <iostream>
#include <string>
#include <vector>
//#include <pcl/io/io.h>
//#include <pcl/point_types.h>
//////#include <boost/thread/thread.hpp>
#include "module/KUPCLMath.h"
#include "module/data_class/planning_data.h"
#include "define.h"

const unsigned int NOPATH     = 0;
const unsigned int POLYNOMIAL = 1;
const unsigned int TRAPEZOID  = 2;


const unsigned int PATH_DOFS  = 6;      // 최대8자유도
const unsigned int MAX_WAYPOINTS = 200; // 최대 경유점의 수

/**
 * @class PATHPLANNING
 * @brief 로봇 경로 계획 클래스
 * @ingroup PATH_PLANNING
 */
class PATHPLANNING
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
	PATHPLANNING();
	~PATHPLANNING();

public:
	KUPCLMATH m_math;
	double m_DH[6][4];
	unsigned int m_nPathDOFs;

	bool genRobotPose(const std::vector<Eigen::Matrix4f> &T_in, std::vector<Eigen::VectorXf> &pose_set);
	bool genRobotPose2(CPATHDATA &path_data);

	bool genRobotPoseFromViewpoint(CVIEWPOINTDATA &vp_data, CPATHDATA &path_data, Eigen::VectorXf &q_prev, std::vector<double> &robot_acc, std::vector<double> &robot_vel);

	void trajectoryPlanning(const Eigen::VectorXf &P_cmd, const Eigen::VectorXf &P_init, std::vector<Eigen::VectorXf> &traj);
    void trajectoryPlanning2(double control_period, const Eigen::VectorXf &P_cmd, const Eigen::VectorXf &P_init, std::vector<Eigen::VectorXf> &traj);
	int inverseClosedForm(const Eigen::VectorXf &q_past, Eigen::VectorXf &x_current, Eigen::VectorXf &q_current);
	void setPathPlanningParameters(CPATHDATA &path_data);

	void setAcceleration(const Eigen::VectorXf &Acc);
	void setMaxVelocity(const Eigen::VectorXf &Vel);

	void setInitPos(const Eigen::VectorXf &Pos, bool mode);
	void calJacobianGeo(const Eigen::VectorXf &q_deg, Eigen::MatrixXf &J_geo);
	bool checkJacobianCondNum(const Eigen::MatrixXf &matJ, double thre_cond);

private:
	Eigen::Matrix4f m_T_S2T;
	double m_dTimeInit;
	double m_dCurrentTime;
	double m_dTimeFin;
	int m_iPathMode;
    bool m_bSpace;    // Path의 생성 공간, true: caretesian space, false: joint space

	Eigen::VectorXf m_dPosInit;
	Eigen::VectorXf m_dCurrentPos;
	Eigen::VectorXf m_dCurrentVel;
	Eigen::VectorXf m_dCurrentAcc;
	Eigen::VectorXf m_dPosFin;
	Eigen::VectorXf m_dVelMax;
	Eigen::VectorXf m_dPosDes;
	Eigen::VectorXf m_dVelDes;
	Eigen::VectorXf m_dAccDes;
	Eigen::VectorXf m_dAcc;
	Eigen::VectorXf m_bDirection;
	Eigen::VectorXf m_dVelUni;
	Eigen::VectorXf m_dTimeAcc;
	void genTrapPath(const Eigen::VectorXf &Distance);
	void calTrapPath(double t);

    // Polynomial path생성을 위한 구문
    void genPolyPath(const Eigen::VectorXf &Distance);
    void calPolyPath(double t);
    void calPolyPathCoeff(double InitPos, double FinPos, double InitVel, double FinVel, double PTime, double PolyCoeff[6]);
    double m_dPolyPathTime[MAX_WAYPOINTS * 2 + 1][2];
    double m_dPolyPathCoff[MAX_WAYPOINTS * 2 + 1][PATH_DOFS][6]; // M, a1, a2, a3, a4, a5
    double m_dPolyPathPara[MAX_WAYPOINTS * 2 + 1][PATH_DOFS][5]; // Pi, Pf, Vi, Vf, Tf
    unsigned int m_nPolyPathNum;
    unsigned int m_nPolyPathStep;
    double m_dPolyPathWP[MAX_WAYPOINTS][PATH_DOFS];


	void calPath(double t);

	void genNoPath(const Eigen::VectorXf &Pos);
	void pose2HTM(const Eigen::VectorXf &x, Eigen::Matrix4f &T_out);
	void RPY2Rot(const Eigen::Vector3f &dRPY, Eigen::Matrix3f &dRot);
	double checkFloatingError(double v1, double v2);
	bool checkEquality(double value, double match);
	Eigen::Matrix4f TransformationMatrix(int n, double q);

	bool checkPoseSetPath(const std::vector<Eigen::VectorXf> &pose_set, const Eigen::VectorXf q_init);
	bool checkSinglePosePath(const Eigen::VectorXf &pose_in, const Eigen::VectorXf q_init, std::vector<double> &robot_acc, std::vector<double> &robot_vel);
};
