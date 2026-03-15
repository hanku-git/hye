#ifndef MAINWINDOW_CONTROL_HPP
#define MAINWINDOW_CONTROL_HPP

#include "ui_robot_control.h"
#include <QtCore/QTimer>
#include <QtCore/QList>
#include <QtCore/QString>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QFileDialog>
#include "qnode.hpp"

#ifndef CALIBRATION_SKIP
#include "cal_dialog.hpp"
#endif

// #include "llm_dialog.hpp"
#include "bin_picking_dialog.hpp"

using namespace std;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow_control; }
QT_END_NAMESPACE

class MainWindow_control : public QMainWindow
{
    Q_OBJECT

Q_SIGNALS:
    void closeIRLDeveloperWindow();



public:
    MainWindow_control(int argc, char** argv, QNode* qnode, QWidget *parent = nullptr);
    ~MainWindow_control();

public Q_SLOTS:
#if BIN_PICKING_FLAG
    // void dlgDoScanning(taskScanningParameter parameter);
#endif

private Q_SLOTS:
    void timerCallback();

    // Joint space & Cartesian space control button
    void setHomeQBtnCallback();
    void setTaskQBtnCallback();
    void setTaskXBtnCallback();
    void setCurrentQBtnCallback();
    void setCurrentXBtnCallback();
    void moveQBtnCallback();
    void moveXBtnCallback();

    void stepMoveQBtnCallback(uint index, bool is_plus);
    void stepMoveXBtnCallback(uint index, bool is_plus);

    // Jog button
    void jogBtnCallback();
    void jogBtnPressed(uint index, bool is_plus, bool is_joint_space);
    void jogBtnReleased();
    void jogStartBtnCallback();
    void jogEndBtnCallback();

    // DTC button
    void setDtcMode();
    void setDtcInput();

    // Robot setting tab button
    void setPowerBtnCallback();
    void setEnableBtnCallback();
    void jammingStateEnableBtnCallback();
    void setFrictionObserverBtnCallback();
    void setHandGuideBtnCallback();
    void setCollisionDetectionBtnCallback();
    void setCollisionDemoModeBtnCallback();
    void setInvKineMethodBtnCallback(InvKineMethod ik_method);
    void setBrakeBtnCallback(bool flag);
    void clearErrorBtnCallback();
    void jtsBiasBtnCallback();
    void ftsBiasBtnCallback();
    void forceBiasBtnCallback();
    void setTcpBtnCallback();
    void setPayloadBtnCallback();
    void setDriverCtrlModeBtnCallback(DriverCtrlMode driver_ctrl_mode);

    /// Impedance & gripper tab
    // Impedance button
    void impedanceBtnCallback(bool flag);

    // Gripper button
    void grpEnableBtnCallback();
    void grpResetBtnCallback();
    void grpGraspBtnCallback();
    void grpReleaseBtnCallback();
    void grpMoveBtnCallback();
    void setLedBtnCallback();
    void setGrpBtnCallback();
    void grpCtrlWordCallback();

    /// Task planner tab
    // Task related button
    void resetTaskParamsBtnCallback();
    void genTaskBtnCallback();

    /// Development tab
    void updateJsonParamsCallback();
    void rollbackJsonParamsCallback();

    // Recording button
    void recordingStartBtnCallback();
    void recordingEndBtnCallback();

    // Stop button
    void stopAllBtnCallback();
    void stopRobotBtnCallback();
    void stopGrpBtnCallback();
    void stopTaskBtnCallback();
    void pauseTaskBtnCallback();
    void resumeTaskBtnCallback();

    void initNodePtr();

    // step motor plc
    void SetStepMotorVelocity();
    void SetStepMotorPosition();

    //// Bin-picking
#if BIN_PICKING_FLAG
    void openBinPickingDialog();
    // void pushButtonDoScanningZIVIDClickedCallback();
    // void pushButtonDoTemplateMatchingBinPickingClickedCallback();
    // void pushButtonSetPreDetectedPoseClickedCallback();
#endif
    //// Robot Calibration
    int OpenDialogCallback();
    int OpenLLMDialog();

    // void OpenFolderCallback();
    // void CalGenerateCallback();
    // void CalibrateCallback();

    // robot status
    // void servo_on_button_clicked();
    // void jsctrl_button_clicked();
    // void csctrl_button_clicked();
    // void friction_mode_button_clicked();
    // void handguide_button_clicked();
    // void collision_detection_button_clicked();
    // void impedance_button_clicked();

    // void brake_off_button_clicked();
    // void jts_bias_button_clicked();
    // void error_clear_button_clicked();
    // void home_pose_button_clicked();
    // void task_pose_button_clicked();
    // void robot_stop_button_clicked();

    // void joint_angle_button_clicked(int index, int sign);
    // void pose_button_clicked(int index, int sign);

    // // CTRL
    // void set_tcp_payload_button_clicked();

    // // Gain
    // void set_pid_button_clicked();
    // void set_impedance_button_clicked();
    // void get_pid_button_clicked();
    // void get_impedance_button_clicked();

private:
    // void move_joint_space(vector<double> target_q);
    // void move_cartesian_space(vector<double> target_x, uint8_t frame_type);

private:
    Ui::MainWindowDesign *ui;
    BinPickingDialog *bin_picking_dialog_; // dialog

    // llm_dialog *llm_dialog_;

#ifndef CALIBRATION_SKIP
    cal_dialog *cal_dialog_;
#endif


    int init_argc;
    char** init_argv;
    QNode *qnode_;
    QTimer *timer_;

    QList<QLineEdit*> list_q_meas_;
    QList<QLineEdit*> list_q_target_;
    QList<QLineEdit*> list_torque_meas_;

    QList<QLineEdit*> list_x_meas_;
    QList<QLineEdit*> list_x_target_;
    QList<QLineEdit*> list_force_meas_;

    QList<QLineEdit*> list_col_mal_index_js_;
    QList<QLineEdit*> list_col_mal_index_cs_;

    QList<QLineEdit*> list_tcp_;
    QList<QLineEdit*> list_com_;
    QList<QLineEdit*> list_impedance_m_;
    QList<QLineEdit*> list_impedance_b_;
    QList<QLineEdit*> list_impedance_k_;
    QList<QLineEdit*> list_impedance_force_limit_;
    QList<QLineEdit*> list_impedance_force_des_;

    QList<QRadioButton*> list_impedance_selection_;
    QList<QRadioButton*> list_force_selection_;
    QList<QRadioButton*> list_pos_selection_;

    QList<QPushButton*> list_jog_q_plus_;
    QList<QPushButton*> list_jog_q_minus_;
    QList<QPushButton*> list_jog_x_plus_;
    QList<QPushButton*> list_jog_x_minus_;

    QList<QPushButton*> list_step_move_q_plus_;
    QList<QPushButton*> list_step_move_q_minus_;
    QList<QPushButton*> list_step_move_x_plus_;
    QList<QPushButton*> list_step_move_x_minus_;
    QList<QCheckBox*> list_dtc_mode_;
    QList<QDoubleSpinBox*> list_dtc_input_;

    //ZIVID Scanning
    QList <QLineEdit *> taskTargetParts_;
    QList <QLineEdit *> fixedMaskPixelList_;
    QList <QLineEdit *> robotATargetXList_;

    QList <QRadioButton *> bin_picking_target_object_list_;
    // QList<QPushButton*> pose_plus_list_;
    // QList<QPushButton*> pose_minus_list_;

    //Robot Calibration
    QList <QLineEdit *> list_cal_pose_;
};
#endif // MAINWINDOW_CONTROL_HPP
