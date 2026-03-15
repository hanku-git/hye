#ifndef BIN_PICKING_DIALOG_HPP
#define BIN_PICKING_DIALOG_HPP

#include <QDialog>
#include <QMessageBox>

#include "ui_bin_picking_dialog.h"
#include "bin_picking_parameter.hpp"
#include "bin_matching_dialog.hpp"
#include "robot2camera_calibration_dialog.hpp"
#include "qnode.hpp"
#include "ui_define.hpp"

class BinPickingDialog : public QDialog
{
    Q_OBJECT

Q_SIGNALS:
    // void dlgDoScanning(taskScanningParameter parameter);
public:
    BinPickingDialog(QWidget *parent = 0);
    ~BinPickingDialog();


public:
    void initializeDialog();
    void testDialog(unsigned int idx);
    void setNodePtr(QNode *ptr_node) { qnode_ = ptr_node; }
    void timerCallback();
    void timerSubCallback();
    void loadRobotParameters();

public Q_SLOTS:
    void setCustomTrasnformationPose(std::vector<double> &value);
    void slot_getRobot2CameraCalibrationJSPosition(std::vector<double> &js_position, bool do_teaching);
    void slot_JSMove(std::vector<double> &target_js_position);
    void slot_STOPAll();
    void slot_doScanningInitialStage(size_t sampling_num, bool skip_detection_mask, std::string target_name);
    void slot_doClearCloud(bool do_clear);

public Q_SLOTS:
    void setInitListValue();

    // UR JS, CS control
    void pushButtonAbsRelCheckClickedCallback();
    void pushButtonCSMoveClickedCallback();
    void pushButtonJSMoveClickedCallback();

    void pushButtonCSMoveRelativeToolFrameClickedCallback();

    void pushButtonImpedanceOnClickedCallback();
    void pushButtonImpedanceOffClickedCallback();
    void pushButtonForceBiasClickedCallback();
    void pushButtonSetCurrentRobotAPoseClickedCallback();
    void pushButtonSetCurQRobotAClickedCallback();
    void pushButtonGetCurQRobotAClickedCallback();
    void pushButtonGetCurXRobotAClickedCallback();

    ////////////////////////////////////////////////////////////
    /////////////////// ver.2 추가 //////////////////////
    // void pushButtonSetRLTargetJSClickedCallback();
    // void pushButtonSetRLTargetJSClickedCallback2();
    // void pushButtonSetRLTargetCSClickedCallback();
    // void pushButtonSetRLTargetCSClickedCallback2();
    // void pushButtonSetNoRLTargetJSClickedCallback();
    // void pushButtonSetNoRLTargetCSClickedCallback();
    // void pushButtonSetGraspingTargetJSClickedCallback();
    // void pushButtonSetGraspingTargetCSClickedCallback();
    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////

    void pushButtonDoTaskLoadIDClickedCallback();

    // KORAS gripper
    void pushButtonKorasGripperIntializeCallback();
    void pushButtonKorasGripperIntializeVer2Callback();
    void pushButtonKorasDualGripperIntializeCallback();
    void pushButtonKorasGripperSlaveChangeCallback();
    void pushButtonKorasGripperSetInitMinMaxValue();
    void pushButtonKorasGripperOpenCallback();
    void pushButtonKorasGripperCloseCallback();
    void pushButtonKorasGripperPosCtrlCallback();
    void pushButtonKorasGripperPosCtrlOnOffCallback();
    void pushButtonKorasGripperVacuumOnOffCallback();

    void pushButtonKorasGripperToolIndexChangeCallback();
    void pushButtonKorasGripperTipIndexChangeCallback();


    // Task Recognition
    void pushButtonDoTaskInfoPrintCallback();

    void pushButtonToolTeachingProcess();

    //// Modbus
    void pushButtonModbusSetIPAddress();
    void pushButtonModbusConnect();
    void pushButtonModbusClose();
    void pushButtonModbusLatheChuckOnOff();
    void pushButtonModbusMillingChuckOnOff();
    void pushButtonModbusDoorOnOff();
    void pushButtonModbusLEDGreenOnOff();
    void pushButtonModbusLEDYellowOnOff();
    void pushButtonModbusLEDRedOnOff();

    void pushButtonSetDefaultTCP();
    void pushButtonSelectTCP();

    void pushButtonGenerateLoadIDTasksCallback();


    // For Jog
    void pushButtonJogSelectCallback();
    void pushButtonJogModeEndCallback();
    // Jog button
    void jogBtnPressed(uint index, bool is_plus, bool is_joint_space);
    void jogBtnReleased();

    void pushButtonSTOPAllCallback();
    void pushButtonSTOPRobotTaskCallback();
    void pauseTaskBtnCallback();
    void resumeTaskBtnCallback();
    void pushButtonComplianceCallback();
    void pushButtonComplianceOffCallback();
    void pushButtonSetToolWeight();
    void pushButtonMakeToolWeight();

    void pushButtonAddTcpPresetClickedCallback();

    void pushButtonChangeCollisionSensitivityClickedCallback();
    void pushButtonRobotEnableClickedCallback();

    void pushButtonSetUIRobotAccVel();
    void pushButtonLoadIdentification();

    //// 3D scanning
    void pushButtonDoScanningZIVIDClickedCallback();
    void pushButtonDoTemplateMatchingClickedCallback();
    void pushButtonDoTemplateMatchingHoleClickedCallback();
    void pushButtonDoTemplateMatchingBinPickingClickedCallback();
    void pushButtonDoPoseEstimationBinPickingClickedCallback();

    void pushButtonDoTemplateInitializeClickedCallback();
    void pushButtonDoLoadLearningWeightClickedCallback();


    void pushButtonOptimizationBinWorkspaceClickedCallback();
    void pushButtonBinMatchingClickedCallback();
    void pushButtonRobot2CameraCalibrationClickedCallback();
    void pushButtonDoEvaluateGraspingPoseClickedCallback();
    void pushButtonSimulationBinPlacingClickedCallback();
    void pushButtonDoZigPoseInitialTeachingClickedCallback();

    void pushButtonSetGraspingPart1PixelSizeClickedCallback();
    void pushButtonSetGraspingPart2PixelSizeClickedCallback();
    void pushButtonSetPreDetectedPoseClickedCallback();
    void pushButtonSetSubDetectedPoseClickedCallback();

    void pushButtonSetEstimatedPoseClickedCallback();
    void pushButtonSetEstimatedSubPoseClickedCallback();


    void pushButtonSetUIJSPosition1ClickedCallback();
    void pushButtonSetUIJSPosition2ClickedCallback();
    void pushButtonSetUIJSPosition3_1ClickedCallback();
    void pushButtonSetUIJSPosition3_2ClickedCallback();
    void pushButtonSetUIJSPosition4ClickedCallback();
    void pushButtonSetUIJSPosition5_1ClickedCallback();
    void pushButtonSetUIJSPosition5_2ClickedCallback();
    void pushButtonSetUIJSPosition5_3ClickedCallback();
    void pushButtonSetUIJSPosition6ClickedCallback();

    void pushButtonSetUIJSPosition7ClickedCallback();
    void pushButtonSetUIJSPosition8ClickedCallback();
    void pushButtonSetUIJSPosition9ClickedCallback();
    void pushButtonSetUIJSPosition10ClickedCallback();
    void pushButtonSetUIJSPosition11ClickedCallback();
    void pushButtonSetUIJSPosition12ClickedCallback();


    void pushButtonSetUIJSPositionHomeClickedCallback();
    void pushButtonSetUIJSPositionHome2ClickedCallback();


    void pushButtonSetUIZIVIDCalPoseInitClickedCallback();
    void pushButtonSetUIZIVIDCalPose1ClickedCallback();
    void pushButtonSetUIZIVIDCalPose2ClickedCallback();
    void pushButtonSetUIZIVIDCalPose3ClickedCallback();
    void pushButtonSetUIZIVIDCalPose4ClickedCallback();
    void pushButtonSetUIZIVIDCalPose5ClickedCallback();
    void pushButtonSetUIZIVIDCalPose6ClickedCallback();
    void pushButtonSetUIZIVIDCalPose7ClickedCallback();
    void pushButtonSetUIZIVIDCalPose8ClickedCallback();
    void pushButtonSetUIZIVIDCalPose9ClickedCallback();
    void pushButtonSetUIZIVIDCalPose10ClickedCallback();
    void pushButtonSetUIZIVIDCalPose11ClickedCallback();
    void pushButtonSetUIZIVIDCalPose12ClickedCallback();

    void pushButtonSetUIJSPositionGripperExperiment1ClickedCallback();
    void pushButtonSetUIJSPositionGripperExperiment2ClickedCallback();
    void pushButtonSetUIJSPositionGripperExperiment3ClickedCallback();
    void pushButtonSetUIJSPositionGripperExperiment4ClickedCallback();

    void pushButtonSelectTargetObjectClickedCallback();
    void pushButtonSelectRobotTCPClickedCallback();
    void pushButtonSelectToolTipClickedCallback();
    void pushButtonSelectTemplateMatchingSegmentationClickedCallback();

    ////  PCL Function
    void pushButtonDoPCLTestFunctionClickedCallback();
    void pushButtonDoPCLFunctionFeatureExtractionClickedCallback();
    void pushButtonDoPCLFunctionSegmentationClickedCallback();


    //// Data Acquisition - python code
    void FTdataAcqusitionStartOnClickedCallback();
    //// Data Acquisition - UDP
    void UDPFTdataAcqusitionStartOnClickedCallback();
    void UDPConnectOnClickedCallback();
    void UDPCloseOnClickedCallback();


    void pushButtonDoLLMTestFunctionTasksCallback();
    void rightPushButtonDoDrumTotalTasksCallback();
    void pushButtonDoDrumScanTasksCallback();


    void pushButtonDoKorasChemicalGripperInitializeCallback();
    void pushButtonDoKorasChemicalGripperOpenCallback();
    void pushButtonDoKorasChemicalGripperCloseCallback();
    void pushButtonDoKorasChemicalGripperScrewingPoseCallback();
    void pushButtonDoKorasChemicalGripperUnscrewingPoseCallback();
    void pushButtonDoKorasChemicalGripperGraspingPoseCallback();
    void pushButtonDoKorasChemicalGripperExitPoseCallback();

    //// Task Button
    void pushButtonGenerateSingleGraspingTasksCallback();
    void pushButtonDoSingleGraspingTasksCallback();
    void pushButtonDoSequentialDemoTasksCallback2();

    void pushButtonGenerateBinPickingPoseEstimationTasksCallback();
    void pushButtonDoBinPickingPoseEstimationTasksCallback();

    void pushButtonGenerateToolChangingTasksCallback();
    void pushButtonDoToolChangingTasksCallback();
    void pushButtonDoToolAttachingTasksCallback();
    void pushButtonDoToolDetachingTasksCallback();
    void pushButtonDoInitialScanMatchingTasksCallback();

    void pushButtonGenerateTipChangingTasksCallback();
    void pushButtonDoTipChangingTasksCallback();
    void pushButtonDoTipAttachingTasksCallback();
    void pushButtonDoTipDetachingTasksCallback();

    void pushButtonSetToolChangingPoseSensorFrameCallback();
    void pushButtonSetNewToolChangingPoseBaseFrameCallback();

    void pushButtonGenerateTestTasksCallback();
    void pushButtonDoTestTasksCallback();
    void pushButtonTestFunctionCallback();

    void setKORASGripperInitialValue();

    void setInitialTaskRobotParameters();
    void changeRobotTCP(unsigned int tcp_idx);

private:
    Ui::BinPickingDialog ui;


    BinMatchingDialog *bin_matching_dialog; // dialog
    Robot2CameraCalibrationDialog *robot2camera_calibration_dialog; // dialog
    taskTemplateMatchingParameter initial_stage_parameter_;

    QNode *qnode_;

    QList <QLineEdit *> robotAXList_;
    QList <QLineEdit *> robotAQList_;
    QList <QLineEdit *> ATIActualForceSensorList_;
    QList <QLineEdit *> ATIActualForceToolList_;
    QList <QLineEdit *> ATIActualForceTaskList_;
    QList <QLineEdit *> robotATargetXList_;
    QList <QLineEdit *> robotBTargetXList_;
    QList <QLineEdit *> robotATargetQList_;
    QList <QLineEdit *> robotBTargetQList_;

    QList <QLineEdit *> RLTargetJSList_;
    QList <QLineEdit *> RLTargetCSList_;
    QList <QDoubleSpinBox *> RLPoseLimitList_;
    QList <QDoubleSpinBox *> RLForceLimitList_;
    QList <QDoubleSpinBox *> noRLForceLimitList_;
    QList <QDoubleSpinBox *> noRLTargetForceList_;
    QList <QDoubleSpinBox *> noRLContactForceList_;


	QList <QLineEdit *> taskStiffness_;
	QList <QLineEdit *> taskNf_;
	QList <QLineEdit *> taskZeta_;
	QList <QLineEdit *> taskForceLimit_;
	QList <QLineEdit *> taskTargetForce_;
	QList <QLineEdit *> taskContactForce_;
	QList <QLineEdit *> taskTargetParts_;
	QList <QLineEdit *> fixedMaskPixelList_;
    QList <QLineEdit *> taskRecogTaskID_;

    QList <QRadioButton *> feature_extraction_method_list_;
    QList <QRadioButton *> bin_picking_target_object_list_;
    QList <QRadioButton *> robot_tcp_list_;
    QList <QRadioButton *> tool_tip_list_;
    QList <QRadioButton *> UI_template_matching_seg_parameter_list_;
    QList <QRadioButton *> assembly_trajectory_case_list_;

    QList<QPushButton*> list_jog_q_plus_;
    QList<QPushButton*> list_jog_q_minus_;
    QList<QPushButton*> list_jog_x_plus_;
    QList<QPushButton*> list_jog_x_minus_;
};

#endif // BIN_PICKING_DIALOG_HPP
