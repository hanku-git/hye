#ifndef CAL_DIALOG_HPP
#define CAL_DIALOG_HPP

#include <iostream>
#include <fstream>
#include <bin_picking/nlohmann/json.hpp>
#include <math.h>
#include <random>
#include <QDialog>
#include <QMainWindow>
#include <QFileDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QList>

#include "robot_calibration/RobotCal.h"
#include "control/robot_model.hpp"
#include "qnode.hpp"

using namespace std;

QT_BEGIN_NAMESPACE
namespace Ui { class cal_dialog; }
QT_END_NAMESPACE

class cal_dialog : public QDialog
{
    Q_OBJECT

public:
    cal_dialog(QMainWindow *parent = nullptr, QNode *qnode_ = nullptr);
    ~cal_dialog();

public:
    void testDialog(unsigned int idx);

protected:
    void readRobotParam();

public:

    CRobotCal m_RobotCal;
    // CInverseKinematics m_inv_kine;

	double m_TCP[6];
    double m_dMeasuredPosition[1000][6];
    double m_dMeasuredPose[1000][6];
    double m_dLamda;
	int m_robot_dof;
    int m_iPoseCount;
    int m_iIteration;

    int num_of_pose;
    vector<double> home_pose;
    vector<double> cube_size;
    vector<vector<double>> rnd_points;

    Eigen::VectorXd m_dTCP_Calibrated;
    Eigen::VectorXd m_dDH_Err_threshold;
    Eigen::MatrixXd m_dDH_Init;
    Eigen::MatrixXd m_dDH_Calibrated;
    Eigen::MatrixXd m_dDesiredAngleCali;
    Eigen::MatrixXd MeasPose_Cali;

protected:
    RobotModel robot_model_;

private Q_SLOTS:
    void OpenFolderCallback();
    void CalJSTaskGenerateCallback();
    void CalCSTaskGenerateCallback();
    void CalCheckCallback();
    void CalCapBaseCallback();
    void CalCheckResultCallback();

    void saveToCSV(const std::vector<std::vector<double>>& data, const std::string& filePath);
    void CalPoseGenCallback();
    void CalGetPoseCallback();
    void CalCheckCollisonCallback();

    void LoadDHParameters();
    void LoadTCP();
    void LoadMeasuredPosition();
    void LoadDesiredJointAngle();
    void LoadData();
    void DHcalibrationCallback();
    void RunRobotPoseCalibrationWithTool(unsigned int n_case, bool is_tool_calibration);
    void CalMinMax(const std::vector<double>& input, int &idx_out, double &dis_out, int b_select);
    void CalMeanStd(const std::vector<double>& input, double& m, double& std);

    void pushButtonDoTaskRobotCalibrationClickedCallback();

private:
    Ui::cal_dialog *cal_ui;
    QNode *qnode_cal;

    QList<QLineEdit*> init_dh_params_a;
    QList<QLineEdit*> init_dh_params_ap;
    QList<QLineEdit*> init_dh_params_d;
    QList<QLineEdit*> init_dh_params_q;

    QList<QLineEdit*> cal_dh_params_a;
    QList<QLineEdit*> cal_dh_params_ap;
    QList<QLineEdit*> cal_dh_params_d;
    QList<QLineEdit*> cal_dh_params_q;

    QList<QLineEdit*> cal_tcp_params;

    QList<QLineEdit*> list_tcp_;
    QList<QLineEdit*> list_check_result;
    QList<QLineEdit*> list_init_pose;
    QList<QLineEdit*> list_pose_boundary;
};
#endif // CAL_DIALOG_HPP
