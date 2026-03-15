/**
 * @file RobotCal.h
 * @brief 로봇 캘리브레이션 알고리즘의 연산을 위한 헤더파일
 */
#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

const unsigned int ROBOT_DOF = 6;
const unsigned int ROBOT_7DOF = 7;

/**
 * @class CMMatrix2
 * @brief 행렬 연산을 위한 클래스
 */
class CMMatrix2
{
public:
	CMMatrix2(void);
	CMMatrix2(unsigned int nColumn, unsigned int nRow);
	~CMMatrix2(void);

	CMMatrix2 CalInverse(void); // Calculation of inverse matrix
	CMMatrix2 Transpose(); // Transposed matrix
	CMMatrix2 Cross(); // Cross product

// Operator overloading
	CMMatrix2 operator+(const CMMatrix2 &); //operator +
	CMMatrix2 operator-(const CMMatrix2 &); //operator -
	CMMatrix2 operator*(const CMMatrix2 &); //operator *

// Element of matrix
	double m_dMatrixElement[180*24];

private:
	unsigned int m_nColumn;
	unsigned int m_nRow;
	void MatrixInversion(double *A, int order, double *Y);   // the inversion of A is put in Y  
};

/**
 * @class CRobotCal
 * @brief 캘리브레이션 알고리즘 연산 위한 클래스
 */
class CRobotCal
{

private:
	double m_dDH[6][4]; //DH notation: alpha a d theta 
	double m_dTcp[6];

	// JS parameter
	double m_dQ[6];    

	// CS Parameter
	double m_dX[6];// Cartesian Space Position and orientation of TCP

	// Homogeneous transformation matrix 연산
	CMMatrix2 TransformationMatrix(int n);
	CMMatrix2 m_dTcpMat;
	Eigen::Matrix4d m_TCP;

public:
	CRobotCal(void);
	~CRobotCal(void);

	CMMatrix2 IdentityMatrix(double lamda);
	void SetDH(double DH[6][4]);
	void SetTcp(double tcp[6]);
	void Setq(double q[6]); 
	void RPY2Rot(double dRPY[3], double dRot[3][3]);
	Eigen::Vector3d rotm2ZYX(Eigen::Matrix3d &RotMat);
	void FwdKine(CMMatrix2 DH_Matrix,double Input_q[ROBOT_DOF], double Output_x[6]);
	void FwdKine_position(CMMatrix2 DH_Matrix,double Input_q[ROBOT_DOF], double Output_x[3]);

	std::vector<double> forwardKinematics(const Eigen::MatrixXd &DH, const Eigen::VectorXd &q);
	void CalCalibrationJacobian(unsigned int n_case, const Eigen::MatrixXd DH, const Eigen::VectorXd dq, double TCP[6], Eigen::MatrixXd &output);
	void CalCalibrationJacobian_7DOF(unsigned int n_case, const Eigen::MatrixXd DH, const Eigen::VectorXd dq, double TCP[6], Eigen::MatrixXd &output);

	// New
	void CalCalibrationJacobianWithTool(unsigned int n_case, const Eigen::MatrixXd DH, const Eigen::VectorXd dq, double TCP[6], Eigen::MatrixXd &output);
	void CalCalibrationJacobianPoseWithTool(unsigned int n_case, const Eigen::MatrixXd DH, const Eigen::VectorXd dq, double TCP[6], Eigen::MatrixXd &output, bool is_tool_calibration);
	void CalCalibrationJacobianPose(unsigned int n_case, const Eigen::MatrixXd DH, const Eigen::VectorXd dq, double TCP[6], Eigen::MatrixXd &output);
	
	
};