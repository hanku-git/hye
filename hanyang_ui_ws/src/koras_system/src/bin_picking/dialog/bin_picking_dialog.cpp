#include "bin_picking/dialog/bin_picking_dialog.hpp"



BinPickingDialog::BinPickingDialog(QWidget *parent) :
    QDialog(parent)
{
    ui.setupUi(this);

#if BIN_PICKING_FLAG
    ROS_LOG_WARN("BIN_PICKING_FLAG: true");
#endif

#if BIN_PICKING_FLAG
    //// Bin matching dialog
    bin_matching_dialog = new BinMatchingDialog(this);
    bin_matching_dialog->setModal(true);
    bin_matching_dialog->setWindowTitle(tr("Bin Detection Manager"));
    QObject::connect(bin_matching_dialog, &BinMatchingDialog::setCustomTrasnformationPose , this, &BinPickingDialog::setCustomTrasnformationPose);
    QObject::connect(bin_matching_dialog, &BinMatchingDialog::slot_doClearCloud , this, &BinPickingDialog::slot_doClearCloud);
    QObject::connect(bin_matching_dialog, &BinMatchingDialog::slot_doScanningInitialStage , this, &BinPickingDialog::slot_doScanningInitialStage);

    //// Robot to camera calibration dialog
    robot2camera_calibration_dialog = new Robot2CameraCalibrationDialog(this);
    robot2camera_calibration_dialog->setModal(true);
    robot2camera_calibration_dialog->setWindowTitle(tr("Robot2Camera Calibration Manager"));
    QObject::connect(robot2camera_calibration_dialog, &Robot2CameraCalibrationDialog::setCustomTrasnformationPose , this, &BinPickingDialog::setCustomTrasnformationPose);
    QObject::connect(robot2camera_calibration_dialog, &Robot2CameraCalibrationDialog::slot_getRobot2CameraCalibrationJSPosition , this, &BinPickingDialog::slot_getRobot2CameraCalibrationJSPosition);
    QObject::connect(robot2camera_calibration_dialog, &Robot2CameraCalibrationDialog::slot_JSMove , this, &BinPickingDialog::slot_JSMove);
    QObject::connect(robot2camera_calibration_dialog, &Robot2CameraCalibrationDialog::slot_STOPAll , this, &BinPickingDialog::slot_STOPAll);
    QObject::connect(robot2camera_calibration_dialog, &Robot2CameraCalibrationDialog::slot_doScanningInitialStage , this, &BinPickingDialog::slot_doScanningInitialStage);





    // JS & CS space control
    QObject::connect(ui.pushButton_MoveRobotACS        , SIGNAL(clicked()), this, SLOT(pushButtonCSMoveClickedCallback()));
    QObject::connect(ui.pushButton_jsMove_robotA      , SIGNAL(clicked()), this, SLOT(pushButtonJSMoveClickedCallback()));

    QObject::connect(ui.pushButton_MoveRobotRelativeToolFrame        , SIGNAL(clicked()), this, SLOT(pushButtonCSMoveRelativeToolFrameClickedCallback()));

    QObject::connect(ui.pushButton_setCurrentPoseRobotA, SIGNAL(clicked()), this, SLOT(pushButtonSetCurrentRobotAPoseClickedCallback()));
    QObject::connect(ui.pushButton_setCurQ_robotA     , SIGNAL(clicked()), this, SLOT(pushButtonSetCurQRobotAClickedCallback()));
    QObject::connect(ui.pushButton_getCurQ_robotA     , SIGNAL(clicked()), this, SLOT(pushButtonGetCurQRobotAClickedCallback()));
    QObject::connect(ui.pushButton_getCurX_robotA     , SIGNAL(clicked()), this, SLOT(pushButtonGetCurXRobotAClickedCallback()));


    QObject::connect(ui.checkBox_isrelativeRobotACS     , SIGNAL(clicked()), this, SLOT(pushButtonAbsRelCheckClickedCallback()));
    QObject::connect(ui.pushButtonImpedanceOn , SIGNAL(clicked()), this, SLOT(pushButtonImpedanceOnClickedCallback()));
    QObject::connect(ui.pushButtonImpedanceOff, SIGNAL(clicked()), this, SLOT(pushButtonImpedanceOffClickedCallback()));
    QObject::connect(ui.pushButtonForceBias   , SIGNAL(clicked()), this, SLOT(pushButtonForceBiasClickedCallback()));
    QObject::connect(ui.pushButton_STOPAll         , SIGNAL(clicked()), this, SLOT(pushButtonSTOPAllCallback()));
    QObject::connect(ui.pushButton_STOPTask        , SIGNAL(clicked()), this, SLOT(pushButtonSTOPRobotTaskCallback()));
    QObject::connect(ui.pushButton_Compliance        , SIGNAL(clicked()), this, SLOT(pushButtonComplianceCallback()));
    QObject::connect(ui.pushButton_ComplianceOFF        , SIGNAL(clicked()), this, SLOT(pushButtonComplianceOffCallback()));

    QObject::connect(ui.pushButton_SetToolWeight        , SIGNAL(clicked()), this, SLOT(pushButtonSetToolWeight()));
    QObject::connect(ui.pushButton_MakeToolWeight        , SIGNAL(clicked()), this, SLOT(pushButtonMakeToolWeight()));
    QObject::connect(ui.pushButtonAddTcpPreset        , SIGNAL(clicked()), this, SLOT(pushButtonAddTcpPresetClickedCallback()));

    QObject::connect(ui.pushButton_ChangeCollisionSensitivity        , SIGNAL(clicked()), this, SLOT(pushButtonChangeCollisionSensitivityClickedCallback()));
    QObject::connect(ui.pushButton_RobotEnable        , SIGNAL(clicked()), this, SLOT(pushButtonRobotEnableClickedCallback()));

    QObject::connect(ui.pushButton_task_pause         , SIGNAL(clicked()), this, SLOT(pauseTaskBtnCallback()));
    QObject::connect(ui.pushButton_task_resume         , SIGNAL(clicked()), this, SLOT(resumeTaskBtnCallback()));


    QObject::connect(ui.radioButton_setRobotAccVel_DEBUG   , SIGNAL(clicked()), this, SLOT(pushButtonSetUIRobotAccVel()));
    QObject::connect(ui.radioButton_setRobotAccVel_RUN   , SIGNAL(clicked()), this, SLOT(pushButtonSetUIRobotAccVel()));

    ////////////////////////////////////////////////////////////
    /////////////////// ver.2 추가 //////////////////////

    // QObject::connect(ui.pushButton_setRLCurrentJSPosition     , SIGNAL(clicked()), this, SLOT(pushButtonSetRLTargetJSClickedCallback()));
    // QObject::connect(ui.pushButton_setRLCurrentJSPosition_2  , SIGNAL(clicked()), this, SLOT(pushButtonSetRLTargetJSClickedCallback2()));
    // QObject::connect(ui.pushButton_setRLCurrentCSPose         , SIGNAL(clicked()), this, SLOT(pushButtonSetRLTargetCSClickedCallback()));
    // QObject::connect(ui.pushButton_setRLCurrentCSPose_2      , SIGNAL(clicked()), this, SLOT(pushButtonSetRLTargetCSClickedCallback2()));
    // QObject::connect(ui.pushButton_saveJSONRLUserInputParameters   , SIGNAL(clicked()), this, SLOT(pushButtonSaveJSONRLUserInputParameters()));
    // QObject::connect(ui.radioButton_RL_Power   , SIGNAL(clicked()), this, SLOT(pushButtonSetInitRLUserInputParameters()));
    // QObject::connect(ui.radioButton_RL_USB   , SIGNAL(clicked()), this, SLOT(pushButtonSetInitRLUserInputParameters()));
    // QObject::connect(ui.radioButton_RL_HDMI   , SIGNAL(clicked()), this, SLOT(pushButtonSetInitRLUserInputParameters()));

    // QObject::connect(ui.pushButton_setNoRLCurrentJSPosition     , SIGNAL(clicked()), this, SLOT(pushButtonSetNoRLTargetJSClickedCallback()));
    // QObject::connect(ui.pushButton_setNoRLCurrentCSPose     , SIGNAL(clicked()), this, SLOT(pushButtonSetNoRLTargetCSClickedCallback()));
    // QObject::connect(ui.pushButton_saveJSONNoRLUserInputParameters   , SIGNAL(clicked()), this, SLOT(pushButtonSaveJSONNoRLUserInputParameters()));
    // QObject::connect(ui.radioButton_no_RL_Power   , SIGNAL(clicked()), this, SLOT(pushButtonSetInitNoRLUserInputParameters()));
    // QObject::connect(ui.radioButton_no_RL_USB   , SIGNAL(clicked()), this, SLOT(pushButtonSetInitNoRLUserInputParameters()));
    // QObject::connect(ui.radioButton_no_RL_HDMI   , SIGNAL(clicked()), this, SLOT(pushButtonSetInitNoRLUserInputParameters()));

    // QObject::connect(ui.pushButton_setGraspingCurrentJSPosition     , SIGNAL(clicked()), this, SLOT(pushButtonSetGraspingTargetJSClickedCallback()));
    // QObject::connect(ui.pushButton_setGraspingCurrentCSPose     , SIGNAL(clicked()), this, SLOT(pushButtonSetGraspingTargetCSClickedCallback()));
    // QObject::connect(ui.pushButton_saveJSONGraspingUserInputParameters   , SIGNAL(clicked()), this, SLOT(pushButtonSaveJSONGraspingUserInputParameters()));

    // QObject::connect(ui.radioButton_grasping_Power   , SIGNAL(clicked()), this, SLOT(pushButtonSetInitGraspingUserInputParameters()));
    // QObject::connect(ui.radioButton_grasping_USB   , SIGNAL(clicked()), this, SLOT(pushButtonSetInitGraspingUserInputParameters()));
    // QObject::connect(ui.radioButton_grasping_HDMI   , SIGNAL(clicked()), this, SLOT(pushButtonSetInitGraspingUserInputParameters()));

    ////////////////////////////////////////////////////////////
    //// 3D Scanning
    QObject::connect(ui.pushButton_doScan_ZIVID     , SIGNAL(clicked()), this, SLOT(pushButtonDoScanningZIVIDClickedCallback()));
    QObject::connect(ui.pushButton_doTemplateMatching     , SIGNAL(clicked()), this, SLOT(pushButtonDoTemplateMatchingClickedCallback()));
    QObject::connect(ui.pushButton_doTemplateMatchingHole     , SIGNAL(clicked()), this, SLOT(pushButtonDoTemplateMatchingHoleClickedCallback()));
    QObject::connect(ui.pushButton_doTemplateMatchingBinPicking     , SIGNAL(clicked()), this, SLOT(pushButtonDoTemplateMatchingBinPickingClickedCallback()));

    QObject::connect(ui.pushButton_doPoseEstimationBinPicking     , SIGNAL(clicked()), this, SLOT(pushButtonDoPoseEstimationBinPickingClickedCallback()));


    QObject::connect(ui.pushButton_doTemplateInitialize     , SIGNAL(clicked()), this, SLOT(pushButtonDoTemplateInitializeClickedCallback()));
    QObject::connect(ui.pushButton_doTemplateInitialize_2     , SIGNAL(clicked()), this, SLOT(pushButtonDoTemplateInitializeClickedCallback()));

    QObject::connect(ui.pushButton_doLoadLearningWeight     , SIGNAL(clicked()), this, SLOT(pushButtonDoLoadLearningWeightClickedCallback()));



    QObject::connect(ui.pushButton_doOptimizationBinWorkspace     , SIGNAL(clicked()), this, SLOT(pushButtonOptimizationBinWorkspaceClickedCallback()));
    QObject::connect(ui.pushButton_doBinMatching                  , SIGNAL(clicked()), this, SLOT(pushButtonBinMatchingClickedCallback()));
    QObject::connect(ui.pushButton_doRobot2CameraCalibration      , SIGNAL(clicked()), this, SLOT(pushButtonRobot2CameraCalibrationClickedCallback()));
    QObject::connect(ui.pushButton_doEvaluateGraspingPose      , SIGNAL(clicked()), this, SLOT(pushButtonDoEvaluateGraspingPoseClickedCallback()));
    QObject::connect(ui.pushButton_doSimulationBinPlacing      , SIGNAL(clicked()), this, SLOT(pushButtonSimulationBinPlacingClickedCallback()));

    QObject::connect(ui.pushButton_doZigPoseInitialTeaching      , SIGNAL(clicked()), this, SLOT(pushButtonDoZigPoseInitialTeachingClickedCallback()));



    //// PCL Test Function
    QObject::connect(ui.pushButton_doPCLTestFunction     , SIGNAL(clicked()), this, SLOT(pushButtonDoPCLTestFunctionClickedCallback()));
    QObject::connect(ui.pushButton_doPCLFunctionFeatureExtraction     , SIGNAL(clicked()), this, SLOT(pushButtonDoPCLFunctionFeatureExtractionClickedCallback()));
    QObject::connect(ui.pushButton_doPCLFunctionSegmentation     , SIGNAL(clicked()), this, SLOT(pushButtonDoPCLFunctionSegmentationClickedCallback()));



    QObject::connect(ui.pushButton_setUIJSPosition_1     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition1ClickedCallback()));
    QObject::connect(ui.pushButton_setUIJSPosition_2     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition2ClickedCallback()));
    QObject::connect(ui.pushButton_setUIJSPosition_3_1     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition3_1ClickedCallback()));
    QObject::connect(ui.pushButton_setUIJSPosition_3_2     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition3_2ClickedCallback()));
    QObject::connect(ui.pushButton_setUIJSPosition_4     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition4ClickedCallback()));
    QObject::connect(ui.pushButton_setUIJSPosition_5_1     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition5_1ClickedCallback()));
    QObject::connect(ui.pushButton_setUIJSPosition_5_2     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition5_2ClickedCallback()));
    QObject::connect(ui.pushButton_setUIJSPosition_5_3     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition5_3ClickedCallback()));
    QObject::connect(ui.pushButton_setUIJSPosition_6     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition6ClickedCallback()));

    QObject::connect(ui.pushButton_setUIJSPosition_7     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition7ClickedCallback()));
    QObject::connect(ui.pushButton_setUIJSPosition_8     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition8ClickedCallback()));
    QObject::connect(ui.pushButton_setUIJSPosition_9     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition9ClickedCallback()));
    QObject::connect(ui.pushButton_setUIJSPosition_10     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition10ClickedCallback()));

    //for Doosan
    QObject::connect(ui.pushButton_setUIJSPosition_11     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition11ClickedCallback()));
    QObject::connect(ui.pushButton_setUIJSPosition_12     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition12ClickedCallback()));

    QObject::connect(ui.pushButton_setUIJSPosition_Home     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPositionHomeClickedCallback()));
    QObject::connect(ui.pushButton_setUIJSPosition_Home_2     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPositionHome2ClickedCallback()));



    QObject::connect(ui.pushButton_setUIZIVIDCalPose_init     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIZIVIDCalPoseInitClickedCallback()));
    QObject::connect(ui.pushButton_setUIZIVIDCalPose_1     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIZIVIDCalPose1ClickedCallback()));
    QObject::connect(ui.pushButton_setUIZIVIDCalPose_2     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIZIVIDCalPose2ClickedCallback()));
    QObject::connect(ui.pushButton_setUIZIVIDCalPose_3     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIZIVIDCalPose3ClickedCallback()));
    QObject::connect(ui.pushButton_setUIZIVIDCalPose_4     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIZIVIDCalPose4ClickedCallback()));
    QObject::connect(ui.pushButton_setUIZIVIDCalPose_5     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIZIVIDCalPose5ClickedCallback()));
    QObject::connect(ui.pushButton_setUIZIVIDCalPose_6     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIZIVIDCalPose6ClickedCallback()));
    QObject::connect(ui.pushButton_setUIZIVIDCalPose_7     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIZIVIDCalPose7ClickedCallback()));
    QObject::connect(ui.pushButton_setUIZIVIDCalPose_8     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIZIVIDCalPose8ClickedCallback()));
    QObject::connect(ui.pushButton_setUIZIVIDCalPose_9     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIZIVIDCalPose9ClickedCallback()));
    QObject::connect(ui.pushButton_setUIZIVIDCalPose_10     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIZIVIDCalPose10ClickedCallback()));
    QObject::connect(ui.pushButton_setUIZIVIDCalPose_11     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIZIVIDCalPose11ClickedCallback()));
    QObject::connect(ui.pushButton_setUIZIVIDCalPose_12     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIZIVIDCalPose12ClickedCallback()));




    QObject::connect(ui.pushButton_setUIJSPosition_gripperExperiment_1, SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPositionGripperExperiment1ClickedCallback()));
    QObject::connect(ui.pushButton_setUIJSPosition_gripperExperiment_2, SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPositionGripperExperiment2ClickedCallback()));
    QObject::connect(ui.pushButton_setUIJSPosition_gripperExperiment_3, SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPositionGripperExperiment3ClickedCallback()));
    QObject::connect(ui.pushButton_setUIJSPosition_gripperExperiment_4, SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPositionGripperExperiment4ClickedCallback()));



    QObject::connect(ui.pushButton_graspingPartPixelSize_1     , SIGNAL(clicked()), this, SLOT(pushButtonSetGraspingPart1PixelSizeClickedCallback()));
    QObject::connect(ui.pushButton_graspingPartPixelSize_2     , SIGNAL(clicked()), this, SLOT(pushButtonSetGraspingPart2PixelSizeClickedCallback()));

    QObject::connect(ui.pushButton_setPreDetectedPose     , SIGNAL(clicked()), this, SLOT(pushButtonSetPreDetectedPoseClickedCallback()));
    QObject::connect(ui.pushButton_setSubDetectedPose     , SIGNAL(clicked()), this, SLOT(pushButtonSetSubDetectedPoseClickedCallback()));


    QObject::connect(ui.pushButton_setEstimatedPose     , SIGNAL(clicked()), this, SLOT(pushButtonSetEstimatedPoseClickedCallback()));
    QObject::connect(ui.pushButton_setSubEstimatedPose     , SIGNAL(clicked()), this, SLOT(pushButtonSetEstimatedSubPoseClickedCallback()));


    //// 240901
    QObject::connect(ui.pushButton_doTask_LLM_TestFunction     , SIGNAL(clicked()), this, SLOT(pushButtonDoLLMTestFunctionTasksCallback()));


    QObject::connect(ui.rightPushButton_doTask_drum_total_task     , SIGNAL(clicked()), this, SLOT(rightPushButtonDoDrumTotalTasksCallback()));
    QObject::connect(ui.pushButton_doTask_drum_scan_task     , SIGNAL(clicked()), this, SLOT(pushButtonDoDrumScanTasksCallback()));
    QObject::connect(ui.pushButton_KorasGripperChemicalGripperInitialize     , SIGNAL(clicked()), this, SLOT(pushButtonDoKorasChemicalGripperInitializeCallback()));

    QObject::connect(ui.pushButton_KorasGripperChemicalGripperOpen     , SIGNAL(clicked()), this, SLOT(pushButtonDoKorasChemicalGripperOpenCallback()));
    QObject::connect(ui.pushButton_KorasGripperChemicalGripperClose     , SIGNAL(clicked()), this, SLOT(pushButtonDoKorasChemicalGripperCloseCallback()));


    QObject::connect(ui.pushButton_KorasGripperChemicalGripperGraspingPose     , SIGNAL(clicked()), this, SLOT(pushButtonDoKorasChemicalGripperGraspingPoseCallback()));
    QObject::connect(ui.pushButton_KorasGripperChemicalGripperScrewingPose     , SIGNAL(clicked()), this, SLOT(pushButtonDoKorasChemicalGripperScrewingPoseCallback()));
    QObject::connect(ui.pushButton_KorasGripperChemicalGripperUnscrewingPose     , SIGNAL(clicked()), this, SLOT(pushButtonDoKorasChemicalGripperUnscrewingPoseCallback()));
    QObject::connect(ui.pushButton_KorasGripperChemicalGripperExitPose     , SIGNAL(clicked()), this, SLOT(pushButtonDoKorasChemicalGripperExitPoseCallback()));


    //// 221111, New task definition
    QObject::connect(ui.pushButton_doTask_binPicking     , SIGNAL(clicked()), this, SLOT(pushButtonDoSingleGraspingTasksCallback()));
    QObject::connect(ui.pushButton_doTask_binPicking_sequentialDemo     , SIGNAL(clicked()), this, SLOT(pushButtonDoSequentialDemoTasksCallback2()));
    QObject::connect(ui.task_pushButton_genTask_binPicking     , SIGNAL(clicked()), this, SLOT(pushButtonGenerateSingleGraspingTasksCallback()));

    // QObject::connect(ui.pushButton_doTask_gripperGrasping     , SIGNAL(clicked()), this, SLOT(pushButtonDoGripperGraspingTasksCallback()));
    // QObject::connect(ui.task_pushButton_genTask_gripperGrasping     , SIGNAL(clicked()), this, SLOT(pushButtonGenerateGripperGraspingTasksCallback()));

    // QObject::connect(ui.pushButton_doTask_gripperExperiment     , SIGNAL(clicked()), this, SLOT(pushButtonDoGripperExperimentTasksCallback()));
    // QObject::connect(ui.task_pushButton_genTask_gripperExperiment     , SIGNAL(clicked()), this, SLOT(pushButtonGenerateGripperExperimentTasksCallback()));

    QObject::connect(ui.pushButton_doTask_binPicking_poseEst     , SIGNAL(clicked()), this, SLOT(pushButtonDoBinPickingPoseEstimationTasksCallback()));
    QObject::connect(ui.task_pushButton_genTask_binPicking_poseEst     , SIGNAL(clicked()), this, SLOT(pushButtonGenerateBinPickingPoseEstimationTasksCallback()));


    // QObject::connect(ui.pushButton_doTask_RobotCal60JS     , SIGNAL(clicked()), this, SLOT(pushButtonDoRobotCalibration60JSPositionTasksCallback()));
    // QObject::connect(ui.task_pushButton_genTaskRobotCal60JS, SIGNAL(clicked()), this, SLOT(pushButtonGenerateRobotCalibration60JSPositionTasksCallback()));


    // QObject::connect(ui.pushButton_doTask_RobotCal60JSUsing3DScanner     , SIGNAL(clicked()), this, SLOT(pushButtonDoRobotCalibration60JSPositionUsing3DScannerTasksCallback()));
    // QObject::connect(ui.task_pushButton_genTaskRobotCal60JSUsing3DScanner, SIGNAL(clicked()), this, SLOT(pushButtonGenerateRobotCalibration60JSPositionUsing3DScannerTasksCallback()));



    // QObject::connect(ui.pushButton_doTask_RobotCalISO9283CS     , SIGNAL(clicked()), this, SLOT(pushButtonDoRobotCalibrationISO9283CSPoseTasksCallback()));
    // QObject::connect(ui.task_pushButton_genTaskRobotCalISO9283CS, SIGNAL(clicked()), this, SLOT(pushButtonGenerateRobotCalibrationISO9283CSPoseTasksCallback()));

    // QObject::connect(ui.pushButton_doTask_RobotCalPlaneXYTestToolMove     , SIGNAL(clicked()), this, SLOT(pushButtonDoRobotCalibrationPlaneXYTestToolMoveTasksCallback()));
    // QObject::connect(ui.task_pushButton_genTaskRobotCalPlaneXYTestToolMove, SIGNAL(clicked()), this, SLOT(pushButtonGenerateRobotCalibrationPlaneXYTestToolMoveTasksCallback()));

    // QObject::connect(ui.pushButton_doTask_RobotCalPlaneXZTestToolMove     , SIGNAL(clicked()), this, SLOT(pushButtonDoRobotCalibrationPlaneXZTestToolMoveTasksCallback()));
    // QObject::connect(ui.task_pushButton_genTaskRobotCalPlaneXZTestToolMove, SIGNAL(clicked()), this, SLOT(pushButtonGenerateRobotCalibrationPlaneXZTestToolMoveTasksCallback()));

    // QObject::connect(ui.pushButton_doTask_RobotCalPlaneYZTestToolMove     , SIGNAL(clicked()), this, SLOT(pushButtonDoRobotCalibrationPlaneYZTestToolMoveTasksCallback()));
    // QObject::connect(ui.task_pushButton_genTaskRobotCalPlaneYZTestToolMove, SIGNAL(clicked()), this, SLOT(pushButtonGenerateRobotCalibrationPlaneYZTestToolMoveTasksCallback()));

    // QObject::connect(ui.pushButton_doTask_ZIVIDGlobalCalibration     , SIGNAL(clicked()), this, SLOT(pushButtonDoZIVIDGlobalCalibrationTasksCallback()));
    // QObject::connect(ui.task_pushButton_genTask_ZIVIDGlobalCalibration, SIGNAL(clicked()), this, SLOT(pushButtonGenerateZIVIDGlobalCalibrationTasksCallback()));



    QObject::connect(ui.pushButton_doTask_test     , SIGNAL(clicked()), this, SLOT(pushButtonDoTestTasksCallback()));
    QObject::connect(ui.task_pushButton_genTask_test     , SIGNAL(clicked()), this, SLOT(pushButtonGenerateTestTasksCallback()));
    QObject::connect(ui.pushButton_testFunction     , SIGNAL(clicked()), this, SLOT(pushButtonTestFunctionCallback()));



    //// Tool changing
    QObject::connect(ui.pushButton_doTask_toolAttaching     , SIGNAL(clicked()), this, SLOT(pushButtonDoToolAttachingTasksCallback()));
    QObject::connect(ui.pushButton_doTask_toolDetaching     , SIGNAL(clicked()), this, SLOT(pushButtonDoToolDetachingTasksCallback()));
    QObject::connect(ui.pushButton_doTask_toolChanging     , SIGNAL(clicked()), this, SLOT(pushButtonDoToolChangingTasksCallback()));
    QObject::connect(ui.pushButton_doTask_initialScanMatching     , SIGNAL(clicked()), this, SLOT(pushButtonDoInitialScanMatchingTasksCallback()));
    QObject::connect(ui.task_pushButton_genTask_toolChanging     , SIGNAL(clicked()), this, SLOT(pushButtonGenerateToolChangingTasksCallback()));

    //// Tip changing
    QObject::connect(ui.pushButton_doTask_tipAttaching     , SIGNAL(clicked()), this, SLOT(pushButtonDoTipAttachingTasksCallback()));
    QObject::connect(ui.pushButton_doTask_tipDetaching     , SIGNAL(clicked()), this, SLOT(pushButtonDoTipDetachingTasksCallback()));
    QObject::connect(ui.pushButton_doTask_tipChanging     , SIGNAL(clicked()), this, SLOT(pushButtonDoTipChangingTasksCallback()));
    QObject::connect(ui.task_pushButton_genTask_tipChanging     , SIGNAL(clicked()), this, SLOT(pushButtonGenerateTipChangingTasksCallback()));


    QObject::connect(ui.pushButton_setToolChangingPoseSensorFrame     , SIGNAL(clicked()), this, SLOT(pushButtonSetToolChangingPoseSensorFrameCallback()));
    QObject::connect(ui.pushButton_setNewToolChangingPoseBaseFrame     , SIGNAL(clicked()), this, SLOT(pushButtonSetNewToolChangingPoseBaseFrameCallback()));





    ///////////////////////////////////////////////////////////
    // //// Assembly UI
    // QObject::connect(ui.pushButton_assemblyUITask_setWorkspace, SIGNAL(clicked()), this, SLOT(pushButtonAssemUISetWorkspaceClickedCallback()));
    // QObject::connect(ui.pushButton_assemblyUITask_homePose, SIGNAL(clicked()), this, SLOT(pushButtonAssemUIMoveHomePoseClickedCallback()));
    // QObject::connect(ui.pushButton_assemblyUITask_approaching, SIGNAL(clicked()), this, SLOT(pushButtonAssemUIApproachingClickedCallback()));
    // QObject::connect(ui.pushButton_assemblyUITask_alignment, SIGNAL(clicked()), this, SLOT(pushButtonAssemUIAlignmentClickedCallback()));
    // QObject::connect(ui.pushButton_assemblyUITask_insertion, SIGNAL(clicked()), this, SLOT(pushButtonAssemUIInsertionClickedCallback()));
    // QObject::connect(ui.pushButton_assemblyUITask_saveTeachingPose, SIGNAL(clicked()), this, SLOT(pushButtonAssemUISetTeachingPoseClickedCallback()));

    ///////////////////////////////////////////////////////////
    //// Communication - UDP
    QObject::connect(ui.pushButtonDataAcquisitionStart , SIGNAL(clicked()), this, SLOT(FTdataAcqusitionStartOnClickedCallback()));
    QObject::connect(ui.pushButton_UDP_DataAcquisitionStart , SIGNAL(clicked()), this, SLOT(UDPFTdataAcqusitionStartOnClickedCallback()));
    QObject::connect(ui.pushButton_UDP_CONNECT , SIGNAL(clicked()), this, SLOT(UDPConnectOnClickedCallback()));
    QObject::connect(ui.pushButton_UDP_CLOSE , SIGNAL(clicked()), this, SLOT(UDPCloseOnClickedCallback()));



    // ///////////////////////Blind Search 동작 수행 내용 ///////////////////////
    // QObject::connect(ui.radioButton_Spiral, SIGNAL(clicked()), this, SLOT(pushButtonSetInitGraspingUserInputParameters()));
    // QObject::connect(ui.radioButton_Spike, SIGNAL(clicked()), this, SLOT(pushButtonSetInitGraspingUserInputParameters()));
    // ///////////////////////////////////// Task Related ///////////////////////////////////////
    // // QObject::connect(ui.pushButton_doTask_Assembly     , SIGNAL(clicked()), this, SLOT(pushButtonDoTaskAssemblyClickedCallback()));
    // QObject::connect(ui.pushButton_doTask_Grasping     , SIGNAL(clicked()), this, SLOT(pushButtonDoTaskGraspingClickedCallback()));
    // QObject::connect(ui.pushButton_doTask_Insertion     , SIGNAL(clicked()), this, SLOT(pushButtonDoTaskInsertionClickedCallback()));
    // QObject::connect(ui.pushButton_doTask_Insertion_connector     , SIGNAL(clicked()), this, SLOT(pushButtonDoTaskCableAssemblyClickedCallback()));
    // QObject::connect(ui.pushButton_doTask_RLInsertion_peg     , SIGNAL(clicked()), this, SLOT(pushButtonDoTaskRLInsertionClickedCallback()));
    // QObject::connect(ui.pushButton_doTask_RLInsertion_connector     , SIGNAL(clicked()), this, SLOT(pushButtonDoTaskRLCableAssemblyClickedCallback()));
    QObject::connect(ui.pushButton_doTaskLoadID     , SIGNAL(clicked()), this, SLOT(pushButtonDoTaskLoadIDClickedCallback()));
    // // QObject::connect(ui.task_pushButton_genAssemblyTasks, SIGNAL(clicked()), this, SLOT(pushButtonGenerateAssemblyTasksCallback()));
    // QObject::connect(ui.task_pushButton_genInsertionTasks, SIGNAL(clicked()), this, SLOT(pushButtonGenerateInsertionTasksCallback()));
    // QObject::connect(ui.task_pushButton_genTasks_Grasping, SIGNAL(clicked()), this, SLOT(pushButtonGenerateCableGraspingTasksCallback()));
    // QObject::connect(ui.task_pushButton_genTask_Insertion_connector, SIGNAL(clicked()), this, SLOT(pushButtonGenerateCableAssemblyTasksCallback()));
    // QObject::connect(ui.task_pushButton_genTask_RLInsertion_peg, SIGNAL(clicked()), this, SLOT(pushButtonGenerateRLPegInsertionTasksCallback()));
    // QObject::connect(ui.task_pushButton_genTask_RLInsertion_connector, SIGNAL(clicked()), this, SLOT(pushButtonGenerateRLCableAssemblyTasksCallback()));


    QObject::connect(ui.task_pushButton_genTaskLoadID, SIGNAL(clicked()), this, SLOT(pushButtonGenerateLoadIDTasksCallback()));



    QObject::connect(ui.pushButton_LoadIdentification   , SIGNAL(clicked()), this, SLOT(pushButtonLoadIdentification()));

    // KORAS Gripper

    QObject::connect(ui.pushButton_KorasGripperInitialize, SIGNAL(clicked()), this, SLOT(pushButtonKorasGripperIntializeCallback()));
    QObject::connect(ui.pushButton_KorasGripperInitialize_2, SIGNAL(clicked()), this, SLOT(pushButtonKorasGripperIntializeVer2Callback()));
    QObject::connect(ui.pushButton_KorasDualGripperInitialize, SIGNAL(clicked()), this, SLOT(pushButtonKorasDualGripperIntializeCallback()));
    QObject::connect(ui.pushButton_KorasGripperSlaveChange, SIGNAL(clicked()), this, SLOT(pushButtonKorasGripperSlaveChangeCallback()));
    QObject::connect(ui.pushButton_KorasGripperSetInitMinMaxValue, SIGNAL(clicked()), this, SLOT(pushButtonKorasGripperSetInitMinMaxValue()));
    QObject::connect(ui.pushButton_KorasGripperOpen, SIGNAL(clicked()), this, SLOT(pushButtonKorasGripperOpenCallback()));
    QObject::connect(ui.pushButton_KorasGripperClose, SIGNAL(clicked()), this, SLOT(pushButtonKorasGripperCloseCallback()));
    QObject::connect(ui.pushButton_KorasGripperPosCtrl, SIGNAL(clicked()), this, SLOT(pushButtonKorasGripperPosCtrlCallback()));
    QObject::connect(ui.pushButton_KorasGripperPosCtrlOnOff, SIGNAL(clicked()), this, SLOT(pushButtonKorasGripperPosCtrlOnOffCallback()));
    QObject::connect(ui.pushButton_KorasGripperVacuumOnOff, SIGNAL(clicked()), this, SLOT(pushButtonKorasGripperVacuumOnOffCallback()));

    QObject::connect(ui.pushButton_selectToolIndex, SIGNAL(clicked()), this, SLOT(pushButtonKorasGripperToolIndexChangeCallback()));
    QObject::connect(ui.pushButton_selectTipIndex, SIGNAL(clicked()), this, SLOT(pushButtonKorasGripperTipIndexChangeCallback()));


    // // Relay
    // QObject::connect(ui.pushButton_relay_pnematic_1, SIGNAL(clicked()), this, SLOT(pushButton_relay_pnematic1Callback()));
    // QObject::connect(ui.pushButton_relay_pnematic_2, SIGNAL(clicked()), this, SLOT(pushButton_relay_pnematic2Callback()));
    // QObject::connect(ui.pushButton_relay_vacuum_1, SIGNAL(clicked()), this, SLOT(pushButton_relay_vacuum1Callback()));
    // QObject::connect(ui.pushButton_relay_vacuum_2, SIGNAL(clicked()), this, SLOT(pushButton_relay_vacuum2Callback()));


    //// Task Recognition
    QObject::connect(ui.pushButton_doTaskInfoPrint, SIGNAL(clicked()), this, SLOT(pushButtonDoTaskInfoPrintCallback()));



    QObject::connect(ui.radioButton_assemblyUI_isToolTeachingProcess, SIGNAL(clicked()), this, SLOT(pushButtonToolTeachingProcess()));

    //// Modbus comm.
    QObject::connect(ui.pushButton_MODBUS_SET_IP, SIGNAL(clicked()), this, SLOT(pushButtonModbusSetIPAddress()));
    QObject::connect(ui.pushButton_MODBUS_CONNECT, SIGNAL(clicked()), this, SLOT(pushButtonModbusConnect()));
    QObject::connect(ui.pushButton_MODBUS_CLOSE, SIGNAL(clicked()), this, SLOT(pushButtonModbusClose()));
    QObject::connect(ui.pushButton_MODBUS_latheChuckOnOff, SIGNAL(clicked()), this, SLOT(pushButtonModbusLatheChuckOnOff()));
    QObject::connect(ui.pushButton_MODBUS_millingChuckOnOff, SIGNAL(clicked()), this, SLOT(pushButtonModbusMillingChuckOnOff()));
    QObject::connect(ui.pushButton_MODBUS_doorOnOff, SIGNAL(clicked()), this, SLOT(pushButtonModbusDoorOnOff()));
    QObject::connect(ui.pushButton_MODBUS_ledGreen, SIGNAL(clicked()), this, SLOT(pushButtonModbusLEDGreenOnOff()));
    QObject::connect(ui.pushButton_MODBUS_ledYellow, SIGNAL(clicked()), this, SLOT(pushButtonModbusLEDYellowOnOff()));
    QObject::connect(ui.pushButton_MODBUS_ledRed, SIGNAL(clicked()), this, SLOT(pushButtonModbusLEDRedOnOff()));

    QObject::connect(ui.pushButton_setTCPDefault, SIGNAL(clicked()), this, SLOT(pushButtonSetDefaultTCP()));
    QObject::connect(ui.pushButton_selectTCP, SIGNAL(clicked()), this, SLOT(pushButtonSelectTCP()));


    /////////////////////////////////////// Jog Button ///////////////////////////////////////

    list_jog_q_plus_ << ui.pushButton_jog_j1_plus << ui.pushButton_jog_j2_plus
                     << ui.pushButton_jog_j3_plus << ui.pushButton_jog_j4_plus
                     << ui.pushButton_jog_j5_plus << ui.pushButton_jog_j6_plus
                     << ui.pushButton_jog_j7_plus;

    list_jog_q_minus_ << ui.pushButton_jog_j1_minus << ui.pushButton_jog_j2_minus
                      << ui.pushButton_jog_j3_minus << ui.pushButton_jog_j4_minus
                      << ui.pushButton_jog_j5_minus << ui.pushButton_jog_j6_minus
                      << ui.pushButton_jog_j7_minus;

    list_jog_x_plus_ << ui.pushButton_jog_x1_plus << ui.pushButton_jog_x2_plus
                     << ui.pushButton_jog_x3_plus << ui.pushButton_jog_x4_plus
                     << ui.pushButton_jog_x5_plus << ui.pushButton_jog_x6_plus;

    list_jog_x_minus_ << ui.pushButton_jog_x1_minus << ui.pushButton_jog_x2_minus
                      << ui.pushButton_jog_x3_minus << ui.pushButton_jog_x4_minus
                      << ui.pushButton_jog_x5_minus << ui.pushButton_jog_x6_minus;

    QObject::connect(ui.pushButton_robotAJogSelect, SIGNAL(clicked()), this, SLOT(pushButtonJogSelectCallback()));
    QObject::connect(ui.pushButton_jogModeEnd, SIGNAL(clicked()), this, SLOT(pushButtonJogModeEndCallback()));

    for (int i = 0; i < 7; i++) {
        connect(list_jog_q_plus_[i], &QPushButton::pressed, this, [=] () {
            jogBtnPressed(i, true, true);
        });
        connect(list_jog_q_minus_[i], &QPushButton::pressed, this, [=] () {
            jogBtnPressed(i, false, true);
        });
        connect(list_jog_q_plus_[i], &QPushButton::released, this, [=] () {
            jogBtnReleased();
        });
        connect(list_jog_q_minus_[i], &QPushButton::released, this, [=] () {
            jogBtnReleased();
        });

    }
    for (int i = 0; i < 6; i++) {
        connect(list_jog_x_plus_[i], &QPushButton::pressed, this, [=] () {
            jogBtnPressed(i, true, false);
        });
        connect(list_jog_x_minus_[i], &QPushButton::pressed, this, [=] () {
            jogBtnPressed(i, false, false);
        });
        connect(list_jog_x_plus_[i], &QPushButton::released, this, [=] () {
            jogBtnReleased();
        });
        connect(list_jog_x_minus_[i], &QPushButton::released, this, [=] () {
            jogBtnReleased();
        });
    }


    robotAXList_ << ui.lineEdit_robotAActualX_1
                 << ui.lineEdit_robotAActualX_2
                 << ui.lineEdit_robotAActualX_3
                 << ui.lineEdit_robotAActualX_4
                 << ui.lineEdit_robotAActualX_5
                 << ui.lineEdit_robotAActualX_6;

    robotAQList_ << ui.lineEdit_robotAActualQ_1
                 << ui.lineEdit_robotAActualQ_2
                 << ui.lineEdit_robotAActualQ_3
                 << ui.lineEdit_robotAActualQ_4
                 << ui.lineEdit_robotAActualQ_5
                 << ui.lineEdit_robotAActualQ_6;

    ATIActualForceSensorList_ << ui.lineEdit_actualForceSensor_1
                              << ui.lineEdit_actualForceSensor_2
                              << ui.lineEdit_actualForceSensor_3
                              << ui.lineEdit_actualForceSensor_4
                              << ui.lineEdit_actualForceSensor_5
                              << ui.lineEdit_actualForceSensor_6;

    ATIActualForceToolList_ << ui.lineEdit_actualForceTool_1
                            << ui.lineEdit_actualForceTool_2
                            << ui.lineEdit_actualForceTool_3
                            << ui.lineEdit_actualForceTool_4
                            << ui.lineEdit_actualForceTool_5
                            << ui.lineEdit_actualForceTool_6;

    ATIActualForceTaskList_ << ui.lineEdit_actualForceTask_1
                            << ui.lineEdit_actualForceTask_2
                            << ui.lineEdit_actualForceTask_3
                            << ui.lineEdit_actualForceTask_4
                            << ui.lineEdit_actualForceTask_5
                            << ui.lineEdit_actualForceTask_6;

    robotATargetXList_ << ui.lineEdit_robotATargetX_1
                       << ui.lineEdit_robotATargetX_2
                       << ui.lineEdit_robotATargetX_3
                       << ui.lineEdit_robotATargetX_4
                       << ui.lineEdit_robotATargetX_5
                       << ui.lineEdit_robotATargetX_6;

    robotBTargetXList_ << ui.lineEdit_robotBTargetX_1
                       << ui.lineEdit_robotBTargetX_2
                       << ui.lineEdit_robotBTargetX_3
                       << ui.lineEdit_robotBTargetX_4
                       << ui.lineEdit_robotBTargetX_5
                       << ui.lineEdit_robotBTargetX_6;

    robotATargetQList_ << ui.lineEdit_robotATargetQ_1
                       << ui.lineEdit_robotATargetQ_2
                       << ui.lineEdit_robotATargetQ_3
                       << ui.lineEdit_robotATargetQ_4
                       << ui.lineEdit_robotATargetQ_5
                       << ui.lineEdit_robotATargetQ_6;

    taskStiffness_ << ui.task_lineEdit_stiffnessX
                   << ui.task_lineEdit_stiffnessY
                   << ui.task_lineEdit_stiffnessZ
                   << ui.task_lineEdit_stiffnessR
                   << ui.task_lineEdit_stiffnessP
                   << ui.task_lineEdit_stiffnessYaw;

	taskNf_ << ui.task_lineEdit_nfX
			<< ui.task_lineEdit_nfY
			<< ui.task_lineEdit_nfZ
			<< ui.task_lineEdit_nfR
			<< ui.task_lineEdit_nfP
			<< ui.task_lineEdit_nfYaw;

	taskZeta_ << ui.task_lineEdit_zetaX
			  << ui.task_lineEdit_zetaY
			  << ui.task_lineEdit_zetaZ
			  << ui.task_lineEdit_zetaR
			  << ui.task_lineEdit_zetaP
			  << ui.task_lineEdit_zetaYaw;

	taskForceLimit_ << ui.task_lineEdit_forceLimitX
					<< ui.task_lineEdit_forceLimitY
					<< ui.task_lineEdit_forceLimitZ
					<< ui.task_lineEdit_forceLimitR
					<< ui.task_lineEdit_forceLimitP
					<< ui.task_lineEdit_forceLimitYaw;

	taskTargetForce_ << ui.task_lineEdit_targetForceX
					 << ui.task_lineEdit_targetForceY
					 << ui.task_lineEdit_targetForceZ
					 << ui.task_lineEdit_targetForceR
					 << ui.task_lineEdit_targetForceP
					 << ui.task_lineEdit_targetForceYaw;

	taskContactForce_ << ui.task_lineEdit_contactForceX
					  << ui.task_lineEdit_contactForceY
					  << ui.task_lineEdit_contactForceZ
					  << ui.task_lineEdit_contactForceR
					  << ui.task_lineEdit_contactForceP
					  << ui.task_lineEdit_contactForceYaw;


    ///////////////////
    RLPoseLimitList_ << ui.task_doubleSpinBox_RLInput_poseLimits_1
                     << ui.task_doubleSpinBox_RLInput_poseLimits_2
                     << ui.task_doubleSpinBox_RLInput_poseLimits_3
                     << ui.task_doubleSpinBox_RLInput_poseLimits_4
                     << ui.task_doubleSpinBox_RLInput_poseLimits_5
                     << ui.task_doubleSpinBox_RLInput_poseLimits_6;

    RLForceLimitList_ << ui.task_doubleSpinBox_RLInput_ForceLimits_1
                      << ui.task_doubleSpinBox_RLInput_ForceLimits_2
                      << ui.task_doubleSpinBox_RLInput_ForceLimits_3
                      << ui.task_doubleSpinBox_RLInput_ForceLimits_4
                      << ui.task_doubleSpinBox_RLInput_ForceLimits_5
                      << ui.task_doubleSpinBox_RLInput_ForceLimits_6;

    ///////////////////

    noRLForceLimitList_ << ui.task_doubleSpinBox_noRLInput_ForceLimits_1
                        << ui.task_doubleSpinBox_noRLInput_ForceLimits_2
                        << ui.task_doubleSpinBox_noRLInput_ForceLimits_3
                        << ui.task_doubleSpinBox_noRLInput_ForceLimits_4
                        << ui.task_doubleSpinBox_noRLInput_ForceLimits_5
                        << ui.task_doubleSpinBox_noRLInput_ForceLimits_6;

    noRLTargetForceList_ << ui.task_doubleSpinBox_noRLInput_targetForce_1
                         << ui.task_doubleSpinBox_noRLInput_targetForce_2
                         << ui.task_doubleSpinBox_noRLInput_targetForce_3
                         << ui.task_doubleSpinBox_noRLInput_targetForce_4
                         << ui.task_doubleSpinBox_noRLInput_targetForce_5
                         << ui.task_doubleSpinBox_noRLInput_targetForce_6;


    noRLContactForceList_ << ui.task_doubleSpinBox_noRLInput_contactForce_1
                          << ui.task_doubleSpinBox_noRLInput_contactForce_2
                          << ui.task_doubleSpinBox_noRLInput_contactForce_3
                          << ui.task_doubleSpinBox_noRLInput_contactForce_4
                          << ui.task_doubleSpinBox_noRLInput_contactForce_5
                          << ui.task_doubleSpinBox_noRLInput_contactForce_6;

    //// 3D scanning
    taskTargetParts_ << ui.lineEdit_targetParts_1
                     << ui.lineEdit_targetParts_2;

    fixedMaskPixelList_ << ui.lineEdit_pixelList_1
                        << ui.lineEdit_pixelList_2
                        << ui.lineEdit_pixelList_3
                        << ui.lineEdit_pixelList_4;


    feature_extraction_method_list_ << ui.radioButton_assemblyUI_featureMethod_PFH
                                    << ui.radioButton_assemblyUI_featureMethod_FPFH
                                    << ui.radioButton_assemblyUI_featureMethod_spinImage
                                    << ui.radioButton_assemblyUI_featureMethod_shapeContext;

    //// Bin-picking target object select
    bin_picking_target_object_list_ << ui.radioButton_graspingUI_object_1
                                    << ui.radioButton_graspingUI_object_2
                                    << ui.radioButton_graspingUI_object_3
                                    << ui.radioButton_graspingUI_object_4
                                    << ui.radioButton_graspingUI_object_5
                                    << ui.radioButton_graspingUI_object_6
                                    << ui.radioButton_graspingUI_object_7
                                    << ui.radioButton_graspingUI_object_8
                                    << ui.radioButton_graspingUI_object_9
                                    << ui.radioButton_graspingUI_object_10
                                    << ui.radioButton_graspingUI_object_11
                                    << ui.radioButton_graspingUI_object_12
                                    << ui.radioButton_graspingUI_object_13
                                    << ui.radioButton_graspingUI_object_14
                                    << ui.radioButton_graspingUI_object_15;



    robot_tcp_list_ << ui.radioButton_robotTCP_1
                    << ui.radioButton_robotTCP_2;

    tool_tip_list_ << ui.radioButton_toolTip_1
                   << ui.radioButton_toolTip_2
                   << ui.radioButton_toolTip_3;


    //// Assembly Trajectory case
    assembly_trajectory_case_list_ << ui.radioButton_UI_assemblyTrajectoryCase_1
                                   << ui.radioButton_UI_assemblyTrajectoryCase_2
                                   << ui.radioButton_UI_assemblyTrajectoryCase_3
                                   << ui.radioButton_UI_assemblyTrajectoryCase_4;




    //// bin picking target object list
    for(int i=0; i<bin_picking_target_object_list_.size(); i++) {
        QObject::connect(bin_picking_target_object_list_[i]     , SIGNAL(clicked()), this, SLOT(pushButtonSelectTargetObjectClickedCallback()));
    }


    for(int i=0; i<2; i++) {
        QObject::connect(robot_tcp_list_[i]                     , SIGNAL(clicked()), this, SLOT(pushButtonSelectRobotTCPClickedCallback()));
    }

    for(int i=0; i<3; i++) {
        QObject::connect(tool_tip_list_[i]                     , SIGNAL(clicked()), this, SLOT(pushButtonSelectToolTipClickedCallback()));
    }

    for(int i=0; i<4; i++) {
        // QObject::connect(assembly_trajectory_case_list_[i]     , SIGNAL(clicked()), this, SLOT(pushButtonSelectAssemblyTrajectoryCaseClickedCallback()));
    }


    //// Template matching parameter - Segmentation
    UI_template_matching_seg_parameter_list_ << ui.radioButton_assemblyUI_segmentation_1
                                             << ui.radioButton_assemblyUI_segmentation_2
                                             << ui.radioButton_assemblyUI_segmentation_3
                                             << ui.radioButton_assemblyUI_segmentation_4
                                             << ui.radioButton_assemblyUI_segmentation_5;

    for(int i=0; i<UI_template_matching_seg_parameter_list_.size(); i++) {
        QObject::connect(UI_template_matching_seg_parameter_list_[i]     , SIGNAL(clicked()), this, SLOT(pushButtonSelectTemplateMatchingSegmentationClickedCallback()));
    }



    for (int i = 0; i < 6; i++) {
        robotAXList_[i]->setEnabled(true);
        robotAQList_[i]->setEnabled(true);
        ATIActualForceSensorList_[i]->setEnabled(true);
        ATIActualForceToolList_[i]->setEnabled(true);
        ATIActualForceTaskList_[i]->setEnabled(true);
    }

#endif
}

BinPickingDialog::~BinPickingDialog()
{
    // delete ui;
}


void BinPickingDialog::initializeDialog() {
#if BIN_PICKING_FLAG

    ////////////////////////////////////////////////////////
    //// Target object 데모 순서 정의
    qnode_->cnt_selected_object_ = 0; // initialize

    qnode_->m_bin_picking_node->m_selected_object_index_.clear();
    qnode_->m_bin_picking_node->m_selected_object_index_.push_back(TargetObject::OBJECT_BOLT_BUSH);
    qnode_->m_bin_picking_node->m_selected_object_index_.push_back(TargetObject::OBJECT_SQUARE_PEG);
    qnode_->m_bin_picking_node->m_selected_object_index_.push_back(TargetObject::OBJECT_WELDING_T_JOINT_GRP_2F);
    qnode_->m_bin_picking_node->m_selected_object_index_.push_back(TargetObject::OBJECT_CYLINDER);
    ////////////////////////////////////////////////////////


    //// Object data pointer initialize
    qnode_->m_bin_picking_node->m_bin_picking_node_target_object_list.clear();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list.clear();
    qnode_->m_bin_picking_node->m_bin_picking_node_target_object_list.resize(bin_picking_target_object_list_.size());
    for (size_t i = 0; i < qnode_->m_bin_picking_node->m_bin_picking_node_target_object_list.size(); i++)
    {
        CTARGET_OBJECT_DATA tmp;
        qnode_->m_bin_picking_node->m_bin_picking_node_target_object_list[i] = tmp;
        CTARGET_OBJECT_DATA* ptr_object_data = &qnode_->m_bin_picking_node->m_bin_picking_node_target_object_list[i];
        qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list.push_back(ptr_object_data);
    }

    for (size_t i = 0; i < bin_picking_target_object_list_.size(); i++)
    {
        bin_picking_target_object_list_[i]->setChecked(true);
        // qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[i]->is_template_initialized_ = false; // 처음엔 무조건 초기화
        qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[i]->is_template_initialized_ = true; // CAD는 matching node에서 초기화하므로 여기선 그냥 true
        pushButtonSelectTargetObjectClickedCallback();
    }

    // Rebuild name->index map from actual UI order
    qnode_->rebuildTargetObjectIndexMap();


    ////////////////////////////////////////////////////////////
    //// KUAIS
    bin_picking_target_object_list_[10]->setChecked(true);
    pushButtonSelectTargetObjectClickedCallback();
    ////////////////////////////////////////////////////////////



    // for (size_t i = 0; i < bin_picking_target_object_list_.size(); i++)
    // {
    //     ROS_LOG_WARN("[Object #%zu] tcp_changing_id: #%zu", i+1, qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[i]->tcp_changing_id);
    //     ROS_LOG_WARN("              target_object_: %s", qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[i]->m_scan_parameter.target_name.c_str());
    //     ROS_LOG_WARN("              stacking_mode_: %zu", qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[i]->stacking_mode_);
    //     ROS_LOG_WARN("              max_cnt_detaching_: %zu", qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[i]->max_cnt_detaching_);
    //     ROS_LOG_WARN("              stacking_trans_scale_: [%0.4f, %0.4f, %0.4f]", qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[i]->stacking_trans_scale_[0], qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[i]->stacking_trans_scale_[1], qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[i]->stacking_trans_scale_[2]);
    //     ROS_LOG_WARN("              stacking_single_stack_num_: %zu", qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[i]->stacking_single_stack_num_);
    //     ROS_LOG_WARN("              stacking_line_stack_num_: %zu", qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[i]->stacking_line_stack_num_);
    // }

    for (size_t i = 0; i < qnode_->m_bin_picking_node->m_selected_object_index_.size(); i++)
    {
        size_t l_idx_now = qnode_->m_bin_picking_node->m_selected_object_index_[i];
        ROS_LOG_WARN("[Object #%zu] tcp_changing_id: #%zu", l_idx_now+1, qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[l_idx_now]->tcp_changing_id);
        ROS_LOG_WARN("              target_object_: %s", qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[l_idx_now]->m_scan_parameter.target_name.c_str());
        ROS_LOG_WARN("              stacking_mode_: %zu", qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[l_idx_now]->stacking_mode_);
        ROS_LOG_WARN("              max_cnt_detaching_: %zu", qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[l_idx_now]->max_cnt_detaching_);
        ROS_LOG_WARN("              stacking_trans_scale_: [%0.4f, %0.4f, %0.4f]", qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[l_idx_now]->stacking_trans_scale_[0], qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[i]->stacking_trans_scale_[1], qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[i]->stacking_trans_scale_[2]);
        ROS_LOG_WARN("              stacking_single_stack_num_: %zu", qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[l_idx_now]->stacking_single_stack_num_);
        ROS_LOG_WARN("              stacking_line_stack_num_: %zu", qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[l_idx_now]->stacking_line_stack_num_);
    }




    // initialize
    setInitListValue();
    ROS_LOG_WARN("CODE1");
    loadRobotParameters(); // load robot dh, tcp
    ROS_LOG_WARN("CODE2");
    // pushButtonSelectTargetObjectClickedCallback();
    // pushButtonSelectAssemblyTrajectoryCaseClickedCallback();
    pushButtonSelectTemplateMatchingSegmentationClickedCallback();
    ROS_LOG_WARN("CODE3");

    qnode_->is_tcp_initialized = true;

    // initialize Modbus TCP/IP
    pushButtonModbusSetIPAddress();

    // Set current Tool index
    qnode_->setToolChangingCurrentIndex(qnode_->tool_changing_attach_id);
    qnode_->setTipChangingCurrentIndex(qnode_->tip_changing_attach_id);
    // 툴이 장착된 채로 시작되는 것으로 가정
    qnode_->is_tool_attached = true; // update
    qnode_->is_tip_attached = true; // update


    // // Set default gripper driver address
    // pushButtonKorasGripperSlaveChangeCallback();
#endif
}

void BinPickingDialog::testDialog(unsigned int idx) {
    ////
    if(idx != 0) {
        qnode_->test_cnt_ = idx;
    }
    ROS_LOG_INFO("(after)bin_picking_dialog_->qnode_->test_cnt_: %i", qnode_->test_cnt_);

    // ROS_LOG_INFO("testDialog - tool_changing_attach_id: %u", qnode_->tool_changing_attach_id);
    // qnode_->tool_changing_attach_id = idx;
    // ROS_LOG_INFO("testDialog - tool_changing_attach_id: %u", qnode_->tool_changing_attach_id);

    // ROS_LOG_INFO("Code here 333");
    // //// Robot TCP selection
    // if(qnode_->is_tcp_initialized) changeRobotTCP(idx); // TCP #1

    // ROS_LOG_INFO("Code here 444");
}

/** @brief Korean: Qt GUI timer callback 함수
 */
void BinPickingDialog::timerCallback() {
    for (std::size_t i = 0; i < 6; i++) {
        robotAXList_[i]->setText(QString::number(qnode_->params_.meas.x[i], 'f', 4));
        robotAQList_[i]->setText(QString::number(qnode_->params_.meas.q[i], 'f', 3));
        ATIActualForceSensorList_[i]->setText(QString::number(qnode_->params_.meas.force_sensor[i], 'f', 2));
        ATIActualForceToolList_[i]->setText(QString::number(qnode_->params_.meas.force_tool[i], 'f', 2));
        ATIActualForceTaskList_[i]->setText(QString::number(qnode_->params_.meas.force[i], 'f', 2));
    }
}

/** @brief Korean: Qt GUI sub. timer callback 함수
 */
void BinPickingDialog::timerSubCallback() {

    if(qnode_->m_bin_picking_node->ui_flag_is_initial_scan_done) {
        ROS_LOG_INFO("Initial Scanning & Matching Finished!!!");
        ROS_LOG_INFO("Initial Scanning & Matching Finished!!!");
        ROS_LOG_INFO("Initial Scanning & Matching Finished!!!");
        ui.pushButton_doTask_initialScanMatching->setChecked(false);
        qnode_->m_bin_picking_node->ui_flag_is_initial_scan_done = false; // only GUI flag
    }

    ///////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////
    ///////////////////////////
    //// TODO: 주의해서 사용
    //// 빈피킹 반복 수행 - Mask가 없을 때까지 반복 수행
    if(ui.pushButton_doTask_binPicking->isChecked())
    {
        if(!qnode_->is_task_mode_) // 작업이 끝난 경우
        {
            // ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            // ROS_LOG_INFO("Bin-Picking Task Finished!(Current mask count: %i)", qnode_->m_bin_picking_node->getDetectedMaskNum());
            // ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            // ui.pushButton_doTask_binPicking->setChecked(false);

            //// TODO: 아래 주석 풀기
            //// TODO: 아래 주석 풀기
            //// TODO: 아래 주석 풀기
            if(qnode_->m_bin_picking_node->getDetectedMaskNum() > 0)
            {
                qnode_->m_bin_picking_node->bin_scan_fail = false; // flag initialize
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                ROS_LOG_INFO("Bin-Picking Task Start!(Current mask count: %i)", qnode_->m_bin_picking_node->getDetectedMaskNum());
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                pushButtonDoSingleGraspingTasksCallback();
            }
            else
            {
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                ROS_LOG_INFO("Bin-Picking Task Stop!(Current mask count: %i)", qnode_->m_bin_picking_node->getDetectedMaskNum());
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                pushButtonSTOPAllCallback();
                ui.pushButton_doTask_binPicking->setChecked(false);
                qnode_->m_bin_picking_node->bin_scan_fail = false; // flag initialize
            }
        } else { // 작업 도중 모니터링 부분
            //// TODO: 아래 주석 풀기
            //// TODO: 아래 주석 풀기
            //// TODO: 아래 주석 풀기
            if(qnode_->m_bin_picking_node->bin_scan_fail)
            {
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                ROS_LOG_INFO("Bin-Picking Task Stop! - Scan failure!(Current mask count: %i)", qnode_->m_bin_picking_node->getDetectedMaskNum());
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                pushButtonSTOPAllCallback();
                ui.pushButton_doTask_binPicking->setChecked(false);
                qnode_->m_bin_picking_node->bin_scan_fail = false; // flag initialize
            }

            // Scan 성공, Mask 없는 경우 --> 물체 파지를 모두 완료한 상황
            // getFlagMatchingProcess() --> is_matching_finished 반환
            // is_matching_finished이 false면, 중단
            //// 현재(23.09.04)는 시간 내에 자세가 들어오지 않으면 정지하도록 설정되어 있음
            //// TODO: 어떻게든 자세를 찾을 때까지 기다리도록 변경

            //// 24.02.12 아래 부분 수정 (qnode_->m_bin_picking_node->is_check_matching_finished 추가)
            if(qnode_->m_bin_picking_node->getFlagMatchingProcess() &&
                qnode_->m_bin_picking_node->getDetectedMaskNum() == 0 &&
                qnode_->m_bin_picking_node->is_check_matching_finished)
            {
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                ROS_LOG_INFO("Bin-Picking Task Stop! - No mask!(Current mask count: %i)", qnode_->m_bin_picking_node->getDetectedMaskNum());
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                pushButtonSTOPAllCallback();
                ui.pushButton_doTask_binPicking->setChecked(false);
                qnode_->m_bin_picking_node->bin_scan_fail = false; // flag initialize


                //// TODO: 반복 수행하도록 처리



            }


            if(qnode_->is_task_blending_path_infeasible_) {
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                ROS_LOG_INFO("Bin-Picking Task Stop! - Blending path infeasible(check the waypoints)");
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                pushButtonSTOPAllCallback();
                ui.pushButton_doTask_binPicking->setChecked(false);
                qnode_->m_bin_picking_node->bin_scan_fail = false; // flag initialize
                qnode_->is_task_blending_path_infeasible_ = false; // flag initialize
            }

            // 그리퍼 동작 거리 이용하여, 파지 성공/실패 판별 - 실패면 다시 수행
            if(qnode_->m_bin_picking_node->is_grasping_fail)
            {
                // ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                // ROS_LOG_INFO("Bin-Picking Task Re-start! - Grasping failure!");
                // ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                // qnode_->m_bin_picking_node->is_grasping_fail = false; // flag initialize
                // pushButtonSTOPAllCallback();
                // pushButtonDoSingleGraspingTasksCallback();
            }


        }
    }
    ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////
    ///////////////////////////
    //// TODO: 주의해서 사용
    //// 빈피킹 반복 수행 - Mask가 없을 때까지 반복 수행
    if(ui.pushButton_doTask_binPicking_sequentialDemo->isChecked())
    {
        if(!qnode_->is_task_mode_) // 작업이 끝난 경우
        {
            // ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            // ROS_LOG_INFO("Bin-Picking Task Finished!(Current mask count: %i)", qnode_->m_bin_picking_node->getDetectedMaskNum());
            // ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            // ui.pushButton_doTask_binPicking_sequentialDemo->setChecked(false);

            //// TODO: 아래 주석 풀기
            //// TODO: 아래 주석 풀기
            //// TODO: 아래 주석 풀기
            if(qnode_->m_bin_picking_node->getDetectedMaskNum() > 0)
            {
                qnode_->m_bin_picking_node->bin_scan_fail = false; // flag initialize
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                ROS_LOG_INFO("[Sequential] Bin-Picking Task Start!(Current mask count: %i)", qnode_->m_bin_picking_node->getDetectedMaskNum());
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                pushButtonDoSequentialDemoTasksCallback2();
            }
            else
            {
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                ROS_LOG_INFO("[Sequential] Bin-Picking Task Stop!(Current mask count: %i)", qnode_->m_bin_picking_node->getDetectedMaskNum());
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                pushButtonSTOPAllCallback();
                ui.pushButton_doTask_binPicking_sequentialDemo->setChecked(false);
                qnode_->m_bin_picking_node->bin_scan_fail = false; // flag initialize
            }
        } else { // 작업 도중 모니터링 부분
            //// TODO: 아래 주석 풀기
            //// TODO: 아래 주석 풀기
            //// TODO: 아래 주석 풀기
            if(qnode_->m_bin_picking_node->bin_scan_fail)
            {
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                ROS_LOG_INFO("[Sequential] Bin-Picking Task Stop! - Scan failure!(Current mask count: %i)", qnode_->m_bin_picking_node->getDetectedMaskNum());
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                pushButtonSTOPAllCallback();
                ui.pushButton_doTask_binPicking_sequentialDemo->setChecked(false);
                qnode_->m_bin_picking_node->bin_scan_fail = false; // flag initialize
            }

            // Scan 성공, Mask 없는 경우 --> 물체 파지를 모두 완료한 상황
            // getFlagMatchingProcess() --> is_matching_finished 반환
            // is_matching_finished이 false면, 중단
            //// 현재(23.09.04)는 시간 내에 자세가 들어오지 않으면 정지하도록 설정되어 있음
            //// TODO: 어떻게든 자세를 찾을 때까지 기다리도록 변경

            //// 24.02.12 아래 부분 수정 (qnode_->m_bin_picking_node->is_check_matching_finished 추가)
            if(qnode_->m_bin_picking_node->getFlagMatchingProcess() &&
                qnode_->m_bin_picking_node->getDetectedMaskNum() == 0 &&
                qnode_->m_bin_picking_node->is_check_matching_finished)
            {
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                ROS_LOG_INFO("[Sequential] Bin-Picking Task Stop! - No mask!(Current mask count: %i)", qnode_->m_bin_picking_node->getDetectedMaskNum());
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                pushButtonSTOPAllCallback();
                ui.pushButton_doTask_binPicking_sequentialDemo->setChecked(false);
                qnode_->m_bin_picking_node->bin_scan_fail = false; // flag initialize


                //// TODO: 반복 수행하도록 처리


            }

            if(qnode_->is_task_blending_path_infeasible_) {
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                ROS_LOG_INFO("[Sequential] Bin-Picking Task Stop! - Blending path infeasible(check the waypoints)");
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                pushButtonSTOPAllCallback();
                ui.pushButton_doTask_binPicking_sequentialDemo->setChecked(false);
                qnode_->m_bin_picking_node->bin_scan_fail = false; // flag initialize
                qnode_->is_task_blending_path_infeasible_ = false; // flag initialize
            }
        }
    }
    ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////


    switch(qnode_->m_bin_picking_node->current_target_object_idx_) {
        case TargetObject::OBJECT_GRILL : {
            ui.lineEdit_currentTargetObject->setText("OBJECT_GRILL");
            break;
        }
        case TargetObject::OBJECT_SQUARE_PEG : {
            ui.lineEdit_currentTargetObject->setText("OBJECT_SQUARE_PEG");
            break;
        }
        case TargetObject::OBJECT_WELDING_T_JOINT_GRP_2F : {
            ui.lineEdit_currentTargetObject->setText("OBJECT_WELDING_T_JOINT_GRP_2F");
            break;
        }
        case TargetObject::OBJECT_WELDING_ELBOW_JOINT : {
            ui.lineEdit_currentTargetObject->setText("OBJECT_WELDING_ELBOW_JOINT");
            break;
        }
        case TargetObject::OBJECT_BOLT_BUSH : {
            ui.lineEdit_currentTargetObject->setText("OBJECT_BOLT_BUSH");
            break;
        }
        case TargetObject::OBJECT_CYLINDER : {
            ui.lineEdit_currentTargetObject->setText("OBJECT_CYLINDER");
            break;
        }
        case TargetObject::OBJECT_STRIKER : {
            ui.lineEdit_currentTargetObject->setText("OBJECT_STRIKER");
            break;
        }
        case TargetObject::OBJECT_HINGE : {
            ui.lineEdit_currentTargetObject->setText("OBJECT_HINGE");
            break;
        }
        case TargetObject::OBJECT_SCU : {
            ui.lineEdit_currentTargetObject->setText("OBJECT_SCU");
            break;
        }
        case TargetObject::OBJECT_BOLT : {
            ui.lineEdit_currentTargetObject->setText("OBJECT_BOLT");
            break;
        }
        default: {
            ui.lineEdit_currentTargetObject->setText("NONE");
            break;
        }
    }
}

void BinPickingDialog::loadRobotParameters() {

    //// FILE PATH
    std::string file_name("bin_picking_dialog.cpp");
    std::string config_path_ = __FILE__;
    config_path_.resize(config_path_.length() - file_name.length());
    config_path_ += "../../../../robot_control_config/controller_config/";

    // config_path_ = "/home/" + USER_NAME + "/bp_gui_ws_sim/src/robot_control_config/controller_config/";  kshan 2026.03.15

    std::string dh_config_path = config_path_ + "dynamics_param.json";

    ifstream file(dh_config_path.c_str());
    json Config = json::parse(file);
    std::vector<double> a = Config["dh"]["a"].get<std::vector<double> >();
    std::vector<double> ap = Config["dh"]["alpha"].get<std::vector<double> >();
    std::vector<double> d = Config["dh"]["d"].get<std::vector<double> >();
    std::vector<double> q = Config["dh"]["theta"].get<std::vector<double> >();

    Eigen::MatrixXd dDH(JS_DOF,4);
    std::vector<double> dh_vec(JS_DOF * 4);
    for(int i = 0; i < JS_DOF; i++) { // [m], [deg]
        dDH(i,1) = 0.001*a[i];
        dDH(i,0) = ap[i];
        dDH(i,2) = 0.001*d[i];
        dDH(i,3) = q[i];
	}

    printf("\n\n\nDH: ");
    for (int i = 0; i < JS_DOF; i++) {
        for(int j = 0; j < 4; j++) { // [m], [deg] -- a, alpha, d, theta
            if(j == 0) {
                dh_vec[i * 4 + j] = dDH(i, j+1); // a
            } else if(j == 1) {
                dh_vec[i * 4 + j] = dDH(i, j-1); // alpha
            } else {
                dh_vec[i * 4 + j] = dDH(i, j); // d, theta
            }
            printf("%0.6f ", dh_vec[i * 4 + j]);
        }
        printf("\n");
    }

    std::string tcp_config_path_ = __FILE__;
    tcp_config_path_.resize(tcp_config_path_.length() - file_name.length());
    tcp_config_path_ += "../../../config/robot_config/";

    std::string tcp_config_path = tcp_config_path_ + "robot_tcp.json";
    //// JSON INPUT
    CsDouble tcp;
    std::vector<std::vector<double>> robot_tcp_set; // 정의된 tcp를 순서대로 저장
    std::ifstream ifs_robot_tcp_config(tcp_config_path.c_str());
    json j_robot_tcp = json::parse(ifs_robot_tcp_config);
    int cnt_tcp = j_robot_tcp["tcp_count"];
    std::stringstream ss;
    for(int i = 0; i < cnt_tcp; i++) {
        ss.str("");
        ss << "tcp";
        ss << std::setw(3) << std::setfill('0') << i+1;
        std::vector<double> vec_tcp = j_robot_tcp[ss.str()].get<std::vector<double>>();
        robot_tcp_set.push_back(vec_tcp); // [m], [deg]
        ROS_LOG_INFO("tcp00%i: %0.4f, %0.4f, %0.4f, %0.1f, %0.1f, %0.1f", i+1, robot_tcp_set[i][0], robot_tcp_set[i][1], robot_tcp_set[i][2], robot_tcp_set[i][3], robot_tcp_set[i][4], robot_tcp_set[i][5]);
    }

    std::vector<double> tcp_default_vec(CS_DOF); // default tcp idx #7
    std::vector<double> tcp_vec(CS_DOF);
    for (int i = 0; i < CS_DOF; i++) {
        tcp_default_vec[i] = robot_tcp_set[7 - 1][i];
        tcp_vec[i] = robot_tcp_set[qnode_->robot_current_tcp_idx_ - 1][i];
    }
    printf("\n\n\nRobot TCP default[m, deg]: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n\n\n", tcp_default_vec[0], tcp_default_vec[1], tcp_default_vec[2], tcp_default_vec[3], tcp_default_vec[4], tcp_default_vec[5] );
    printf("\n\n\nRobot TCP[m, deg]: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n\n\n", tcp_vec[0], tcp_vec[1], tcp_vec[2], tcp_vec[3], tcp_vec[4], tcp_vec[5] );

    //// update
    qnode_->setRobotDHParameters(dh_vec);
    qnode_->setRobotDefaultTCP(tcp_default_vec);
    qnode_->setRobotTCP(tcp_vec);
}

void BinPickingDialog::jogBtnPressed(uint index, bool is_plus, bool is_joint_space) {
    if (qnode_->is_jog_mode_) {
        if ((qnode_->jog_space_ == JS_JOG && is_joint_space) || (qnode_->jog_space_ == CS_JOG && !is_joint_space)) {
            double vel, acc;

            if (!is_joint_space && index < 3) {
                vel = ui.doubleSpinBox_JogSpeed_trans->value();
                vel *= is_plus ? +1 : -1;
                acc = fabs(vel) * 2.0;
            } else {
                vel = ui.doubleSpinBox_JogSpeed_ori->value();
                vel *= is_plus ? +1 : -1;
                acc = fabs(vel) * 2.0;
            }

            qnode_->jogStart(index, vel, acc, is_joint_space, !ui.radioButton_rotationToolFrame->isChecked());
        }
    }
}

void BinPickingDialog::jogBtnReleased() {
    qnode_->jogEnd(0.5);
}


/** @brief Korean: UI 버튼함수, 현재 직교공간 상의 자세를 받아서 UI에 업데이트한다.
 */
void BinPickingDialog::pushButtonSetCurrentRobotAPoseClickedCallback() {
#if BIN_PICKING_FLAG

#if DRFL_CONTROL
    /////////////////////////////////////////////////
    qnode_->getDrflCurrentPose(); //// 두산 로봇 CS pose 할당
    /////////////////////////////////////////////////
#endif
    std::vector<double> currentRobotAPose(6);
    arr2Vec(qnode_->params_.meas.x, currentRobotAPose);
    std::stringstream ss;
    for (std::size_t i = 0; i < currentRobotAPose.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(currentRobotAPose[i], 'f', 5));
        QString pose_str;
        if(i<3) pose_str = QString::number(currentRobotAPose[i], 'f', 5);
        else pose_str = QString::number(currentRobotAPose[i], 'f', 3);
        std::string str = pose_str.toStdString();
        if(i < currentRobotAPose.size()-1) ss << str << ", ";
        else ss << str;
    }
    QString pose_str_out = QString(ss.str().c_str());
    ui.lineEdit_controlView_targetCS->setText(pose_str_out);
#endif
}

/** @brief Korean: UI 버튼함수, 현재 관절공간 상의 관절각도를 받아서 UI에 업데이트한다.
 */
void BinPickingDialog::pushButtonSetCurQRobotAClickedCallback() {
#if BIN_PICKING_FLAG
    std::vector<double> curQRobotA(6);
    arr2Vec(qnode_->params_.meas.q, curQRobotA);
    std::stringstream ss;
    for (std::size_t i = 0; i < curQRobotA.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(curQRobotA[i], 'f', 3));
        QString q_str = QString::number(curQRobotA[i], 'f', 3);
        std::string str = q_str.toStdString();
        if(i < curQRobotA.size()-1) ss << str << ", ";
        else ss << str;
    }
    QString q_str_out = QString(ss.str().c_str());
    ui.lineEdit_controlView_targetJS->setText(q_str_out);
#endif
}

/** @brief Korean: UI 버튼함수, 현재 관절공간 상의 관절각도를 받아서 UI에 업데이트한다.
 */
void BinPickingDialog::pushButtonGetCurQRobotAClickedCallback() {
#if BIN_PICKING_FLAG
    QString q_str_in = ui.lineEdit_controlView_targetJS->text();
    QStringList temp_q_str = q_str_in.split(",");
    std::vector<double> curQRobotA(6);
    for (std::size_t i = 0; i < curQRobotA.size(); i++) {
        curQRobotA[i] = QString(temp_q_str[i]).toDouble();
        robotATargetQList_[i]->setText(QString::number(curQRobotA[i], 'f', 3));
    }
#endif
}

/** @brief Korean: UI 버튼함수, 현재 직교공간 상의 자세를 받아서 UI에 업데이트한다.
 */
void BinPickingDialog::pushButtonGetCurXRobotAClickedCallback() {
#if BIN_PICKING_FLAG
    QString x_str_in = ui.lineEdit_controlView_targetCS->text();
    QStringList temp_x_str = x_str_in.split(",");
    std::vector<double> curXRobotA(6);
    for (std::size_t i = 0; i < curXRobotA.size(); i++) {
        curXRobotA[i] = QString(temp_x_str[i]).toDouble();
        if(i < 3) {
            robotATargetXList_[i]->setText(QString::number(curXRobotA[i], 'f', 5));
        } else {
            robotATargetXList_[i]->setText(QString::number(curXRobotA[i], 'f', 3));
        }
    }
#endif
}

/** @brief Korean: UI 버튼함수, 부하동정(load identification)의 수행을 위해 정의된 작업계획을 실행한다.
 */
void BinPickingDialog::pushButtonDoTaskLoadIDClickedCallback() {
#if BIN_PICKING_FLAG
    qnode_->current_task_list_ = qnode_->task_planner_->module_task_load_id_; // Task planner
    qnode_->beforeBinPickingTaskStart(true);
    qnode_->task_cycle_ = 0;
#endif
}

/** @brief Korean: UI 버튼함수, 직교공간 상의 목표 자세를 현재자세(absolute mode) 또는 0.0(relative mode)으로 초기화하여 UI에 업데이트한다.
 */
void BinPickingDialog::pushButtonAbsRelCheckClickedCallback() {
#if BIN_PICKING_FLAG
    std::vector<double> currentRobotAPose(6);
    arr2Vec(qnode_->params_.meas.x, currentRobotAPose);
    for (std::size_t i = 0; i < currentRobotAPose.size(); i++)
    {
        if(ui.checkBox_isrelativeRobotACS->isChecked()) // relative pose
        {
            robotATargetXList_[i]->setText(QString::number(0.0, 'f', 1));
            robotBTargetXList_[i]->setText(QString::number(0.0, 'f', 1));

        }
        else // absolute pose
        {
            robotATargetXList_[i]->setText(QString::number(currentRobotAPose[i], 'f', 5));
            robotBTargetXList_[i]->setText(QString::number(currentRobotAPose[i], 'f', 5));

        }
    }
#endif
}


/** @brief Korean: UI 버튼함수, 직교공간 상의 목표 자세로 로봇을 이동한다.
 */
void BinPickingDialog::pushButtonCSMoveClickedCallback() {
#if BIN_PICKING_FLAG
    std::vector<double> pose;
    double xd  = ui.doubleSpinBox_robotATargetVelocityCS->value();
    double xdd = ui.doubleSpinBox_robotATargetAccelerationCS->value();
    for (std::size_t i = 0; i < robotATargetXList_.size(); i++) {
        pose.push_back(robotATargetXList_[i]->text().toDouble());
    }
    bool is_relative = ui.checkBox_isrelativeRobotACS->isChecked();
    // bool is_base_frame = !ui.checkBox_is_tcp_move_cs->isChecked();
    bool is_base_frame = true;
    if(fabs(pose[0]) > 1.3 || fabs(pose[1]) > 1.3 || fabs(pose[2]) > 1.0) {
        ROS_LOG_INFO("Check position[x, y, z] scale!");
    } else {
        CsDouble x;
        vec2Arr(pose, x);
        if (JS_DOF == 6 || ((JS_DOF == 7) && !(qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION))) {
            qnode_->moveX(x, xd, xdd, is_base_frame, is_relative);
            // qnode_->moveX(x, xd, xdd, is_base_frame, rotation_frame, is_relative);
        } else if ((JS_DOF == 7) && (qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION)) {
            // double q_redundant = list_x_target_[6]->text().toDouble();
            // qnode_->moveX(x, xd, xdd, is_base_frame, is_relative, q_redundant);
        }
    }
#endif
}

/** @brief Korean: UI 버튼함수, 관절공간 상의 목표 관절각도로 로봇을 이동한다.
 */
void BinPickingDialog::pushButtonJSMoveClickedCallback() {
#if BIN_PICKING_FLAG
    double qd  = ui.task_doubleSpinBox_JSvelRobotA->value();
    double qdd = ui.task_doubleSpinBox_JSaccRobotA->value();
    bool is_relative = false;
    JsDouble q;
    for (std::size_t i = 0; i < robotATargetQList_.size(); i++) {
        q[i] = robotATargetQList_[i]->text().toDouble();
    }
    qnode_->moveQ(q, qd, qdd, is_relative);
    // std::vector<double> jointPosition;
    // double maxVelocity  = ui.task_doubleSpinBox_JSvelRobotA->value();
    // double acceleration = ui.task_doubleSpinBox_JSaccRobotA->value();
    // for (std::size_t i = 0; i < robotATargetQList_.size(); i++) {
    //     jointPosition.push_back(robotATargetQList_[i]->text().toDouble());
    // }
    // bool isrelative = false;

    // qnode_->setTargetJointPosition(jointPosition, maxVelocity, acceleration, isrelative, true);
#endif
}

/** @brief Korean: UI 버튼함수, 직교공간 상의 목표 자세로 로봇을 이동한다.
 */
void BinPickingDialog::pushButtonCSMoveRelativeToolFrameClickedCallback() {
#if BIN_PICKING_FLAG

    //// Relative인 경우만
    if(ui.checkBox_isrelativeRobotACS->isChecked())
    {
        ROS_LOG_INFO("UI_CSMOVE_RELATIVE_TOOL_FRAME!");

        std::vector<double> pose;
        double xd  = ui.doubleSpinBox_robotATargetVelocityCS->value();
        double xdd = ui.doubleSpinBox_robotATargetAccelerationCS->value();
        for (std::size_t i = 0; i < robotATargetXList_.size(); i++) {
            pose.push_back(robotATargetXList_[i]->text().toDouble());
        }
        bool is_relative = true;
        bool is_base_frame = false;
        std::string rotation_mode = "tool_frame";
        if(fabs(pose[0]) > 1.3 || fabs(pose[1]) > 1.3 || fabs(pose[2]) > 1.0) {
            ROS_LOG_INFO("Check position[x, y, z] scale!");
        } else {
            CsDouble x;
            vec2Arr(pose, x);
            if (JS_DOF == 6 || ((JS_DOF == 7) && !(qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION))) {
                qnode_->moveX(x, xd, xdd, is_base_frame, is_relative);
                // qnode_->moveX(x, xd, xdd, is_base_frame, rotation_frame, is_relative);
            } else if ((JS_DOF == 7) && (qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION)) {
                // double q_redundant = list_x_target_[6]->text().toDouble();
                // qnode_->moveX(x, xd, xdd, is_base_frame, is_relative, q_redundant);
            }
        }
        // qnode_->setTargetRelativePose(rotation_mode, pose, velocity, acceleration, isRelative, true);
    } else {
        ROS_LOG_INFO("CURRENT MODE IS NOT RELATIVE MODE!");
    }
#endif
}

/** @brief Korean: UI 버튼함수, 임피던스 제어 모드를 시작한다.
 */
void BinPickingDialog::pushButtonImpedanceOnClickedCallback() {
	std::vector<double> stiffness(CS_DOF);
	std::vector<double> nf(CS_DOF);
	std::vector<double> zeta(CS_DOF);
	std::vector<double> forceLimit(CS_DOF);
	std::vector<double> damping(CS_DOF);
	std::vector<double> mass(CS_DOF);
    for (std::size_t i = 0; i < CS_DOF; i++) {
        stiffness [i] = taskStiffness_ [i]->text().toDouble();
        nf        [i] = taskNf_        [i]->text().toDouble();
        zeta      [i] = taskZeta_      [i]->text().toDouble();
        forceLimit[i] = taskForceLimit_[i]->text().toDouble();

        damping[i] = 2.0 * stiffness[i] * zeta[i] / nf[i];
        mass[i] = stiffness[i] / nf[i] / nf[i];
    }

    Impedance imped;
    for (int i = 0; i < CS_DOF; i++) {
        imped.m[i] = mass      [i];
        imped.b[i] = damping   [i];
        imped.k[i] = stiffness [i];
        imped.force_limit[i] = forceLimit[i];
        // imped.force_selection[i] = list_impedance_selection_[i]->isChecked();
        imped.force_selection[i] = true;
    }

    qnode_->setImpedance(imped, true);

    ROS_LOG_WARN("Impedance ON!");
}

/** @brief Korean: UI 버튼함수, 임피던스 제어 모드를 종료한다.
 */
void BinPickingDialog::pushButtonImpedanceOffClickedCallback() {
	std::vector<double> stiffness(CS_DOF);
	std::vector<double> nf(CS_DOF);
	std::vector<double> zeta(CS_DOF);
	std::vector<double> forceLimit(CS_DOF);
	std::vector<double> damping(CS_DOF);
	std::vector<double> mass(CS_DOF);
    for (std::size_t i = 0; i < CS_DOF; i++) {
        stiffness [i] = taskStiffness_ [i]->text().toDouble();
        nf        [i] = taskNf_        [i]->text().toDouble();
        zeta      [i] = taskZeta_      [i]->text().toDouble();
        forceLimit[i] = taskForceLimit_[i]->text().toDouble();

        damping[i] = 2.0 * stiffness[i] * zeta[i] / nf[i];
        mass[i] = stiffness[i] / nf[i] / nf[i];
    }

    Impedance imped;
    for (int i = 0; i < CS_DOF; i++) {
        imped.m[i] = mass      [i];
        imped.b[i] = damping   [i];
        imped.k[i] = stiffness [i];
        imped.force_limit[i] = forceLimit[i];
        // imped.force_selection[i] = list_impedance_selection_[i]->isChecked();
        imped.force_selection[i] = true;
    }

    qnode_->setImpedance(imped, false);
    ROS_LOG_WARN("Impedance OFF!");
}

/** @brief Korean: UI 버튼함수, ATI F/T sensor의 sensor bias를 수행한다.
 */
void BinPickingDialog::pushButtonForceBiasClickedCallback() {
    ROS_LOG_INFO("Force bias!");
    qnode_->setCommand("force_bias");
}

/** @brief Korean: UI 버튼함수, 로봇의 Jog 동작을 수행한다.
 */
void BinPickingDialog::pushButtonJogSelectCallback() {
#if BIN_PICKING_FLAG
    if (ui.radioButton_JSJog->isChecked()) {
        qnode_->jog_space_ = JS_JOG;
    } else if (ui.radioButton_CSJog->isChecked()) {
        qnode_->jog_space_ = CS_JOG;
    }
    qnode_->is_jog_mode_ = true;
#endif
}

/** @brief Korean: UI 버튼함수, 로봇의 Jog 동작을 종료한다.
 */
void BinPickingDialog::pushButtonJogModeEndCallback() {
#if BIN_PICKING_FLAG
    qnode_->is_jog_mode_ = false;
#endif
}

// void BinPickingDialog::pushButtonSetCSJogRotationMode() {
// #if BIN_PICKING_FLAG
//     if (ui.radioButton_rotationBaseFrame->isChecked())
//     {
//         qnode_->isRPYRotation_ = false;
//         qnode_->isToolFrameRotation_ = false;
//         ROS_LOG_INFO("CS Jog mode: rotation w.r.t base frame");
//     }
//     else if (ui.radioButton_rotationToolFrame->isChecked())
//     {
//         qnode_->isRPYRotation_ = false;
//         qnode_->isToolFrameRotation_ = true;
//         ROS_LOG_INFO("CS Jog mode: rotation w.r.t tool frame");
//     }
//     else if (ui.radioButton_rotationRPY->isChecked())
//     {
//         qnode_->isRPYRotation_ = true;
//         qnode_->isToolFrameRotation_ = false;
//         ROS_LOG_INFO("CS Jog mode: rotation w.r.t RPY");
//     }
// #endif
// }

/** @brief Korean: UI 버튼함수, 로봇의 현재 수행 작업을 정지한다.
 */
void BinPickingDialog::pushButtonSTOPRobotTaskCallback() {
#if BIN_PICKING_FLAG
    // qnode_->taskParam_robotA_.task_step = 0;
    // qnode_->taskParam_robotB_.task_step = 0;

    // qnode_->taskParam_robotA_.task_mode      = TASK_DEFAULT;
    // qnode_->taskParam_robotA_.is_unit_task_fin = false;
    // qnode_->is_task_mode_ = false;
#endif
}

/** @brief Korean: UI 버튼함수, 현재 로봇의 모든 동작을 정지한다.
 */
void BinPickingDialog::pushButtonSTOPAllCallback() {
#if BIN_PICKING_FLAG
    qnode_->is_task_mode_ = false;
#if DRFL_CONTROL
    qnode_->drflMoveStop(2);
#else
    qnode_->stopRobot();
#endif
#endif
}

void BinPickingDialog::pushButtonComplianceCallback() {
#if DRFL_CONTROL

    qnode_->drflSetRobotMode(0);

    qnode_->is_task_mode_ = false;
    bool success = qnode_->drflTaskComplianceCtrl();
    if (success) {
        ROS_LOG_INFO("Compliance control enabled.");
    } else {
        ROS_LOG_ERROR("Failed to enable compliance control.");
    }

    qnode_->drflSetRobotMode(1);

#endif
}

void BinPickingDialog::pushButtonComplianceOffCallback() {
#if DRFL_CONTROL
    qnode_->drflSetRobotMode(0);

    qnode_->is_task_mode_ = false;
    bool success = qnode_->drflReleaseComplianceCtrl();
    if (success) {
        ROS_LOG_INFO("Compliance control enabled.");
    } else {
        ROS_LOG_ERROR("Failed to enable compliance control.");
    }

    qnode_->drflSetRobotMode(1);
#endif
}


void BinPickingDialog::pushButtonSetToolWeight() {
#if DRFL_CONTROL

    qnode_->drflSetRobotMode(0);

    qnode_->is_task_mode_ = false;
    bool success = qnode_->drflSetCurrentTool("t1");
    if (success) {
        ROS_LOG_INFO("Tool Weight SET! /// 5KG tool(0, 0.030 , 0.132)");
    } else {
        ROS_LOG_ERROR("Failed to Set Tool Weight");
    }

    qnode_->drflSetRobotMode(1);

#endif
}
void BinPickingDialog::pushButtonMakeToolWeight() {
#if DRFL_CONTROL

    qnode_->drflSetRobotMode(0);

    qnode_->is_task_mode_ = false;
    bool success = qnode_->drflConfigCreateTool(
        "t1",        // tool name
        8.00,            // tool weight in kg
        std::array<double, 3>{-1.00, 0.0177, 0.095},   // center of gravity (COG) in meters
        std::array<double, 6>{0.00, 0.00, 0.00, 0.00, 0.00, 0.00} // tool inertia (kg·m²) - 예시 값
    );

    if (success) {
        ROS_LOG_INFO("Tool Weight SET! /// 5KG tool(0, 0.030 , 0.132)");
    } else {
        ROS_LOG_ERROR("Failed to Set Tool Weight");
    }

    qnode_->drflSetRobotMode(1);

#endif
}

void BinPickingDialog::pushButtonChangeCollisionSensitivityClickedCallback() {
#if DRFL_CONTROL
    // 로봇 모드를 비활성화 (예: MANUAL 모드)
    qnode_->drflSetRobotMode(0);

    // 충돌 민감도 변경 (예: sensitivity 값을 100으로 설정)
    bool success = qnode_->drflChangeCollisionSensitivity(1);
    if (success) {
        ROS_LOG_INFO("Collision sensitivity changed successfully.");
    } else {
        ROS_LOG_ERROR("Failed to change collision sensitivity.");
    }

    // 로봇 모드를 다시 활성화 (예: AUTONOMOUS 모드)
    qnode_->drflSetRobotMode(1);
#endif
}

void BinPickingDialog::pushButtonRobotEnableClickedCallback() {
#if DRFL_CONTROL
    qnode_->drflSetRobotMode(0);
    qnode_->drflSetRobotMode(1);
#endif
}

void BinPickingDialog::pushButtonAddTcpPresetClickedCallback() {
#if DRFL_CONTROL
    // 로봇 모드를 비활성화합니다.
    qnode_->drflSetRobotMode(0);

    // Preset TCP 1 등록
    std::string tcp_name1 = "tcp01";
    CsDouble tcp1;
    tcp1[0] = 0.0;  // X (mm)
    tcp1[1] = 45.0;  // Y (mm)
    tcp1[2] = 220.0;  // Z (mm)
    tcp1[3] = 0.0;   // Rx (deg)
    tcp1[4] = 0.0;   // Ry (deg)
    tcp1[5] = 90.0;   // Rz (deg)
    bool success1 = qnode_->drflCreateTcp(tcp_name1, tcp1);
    // 내부 TCP 목록에 저장 (UI 업데이트 없이)
    qnode_->list_tcp[tcp_name1] = tcp1;

    // Preset TCP 2 등록
    std::string tcp_name2 = "tcp02";
    CsDouble tcp2;
    tcp2[0] = 0.0;  // X (mm)
    tcp2[1] = 0.0;  // Y (mm)
    tcp2[2] = 0.0;  // Z (mm)
    tcp2[3] = 0.0;   // Rx (deg)
    tcp2[4] = 0.0;   // Ry (deg)
    tcp2[5] = 90.0;   // Rz (deg)
    bool success2 = qnode_->drflCreateTcp(tcp_name2, tcp2);
    qnode_->list_tcp[tcp_name2] = tcp2;

    // 로봇 모드를 다시 활성화합니다.
    qnode_->drflSetRobotMode(1);
#endif
}




void BinPickingDialog::pauseTaskBtnCallback() {
    qnode_->pauseTask();
}

void BinPickingDialog::resumeTaskBtnCallback() {
    qnode_->resumeTask();
}

/** @brief Korean: UI 버튼함수, 부하동정(load identification)을 수행하기 위한 작업계획을 수행한다.
 */
void BinPickingDialog::pushButtonGenerateLoadIDTasksCallback() {
#if BIN_PICKING_FLAG
    setInitialTaskRobotParameters(); // Initialze task and parameters
    qnode_->setLoadIDTargetJSPosition(); // Load ID initialize
    ROS_LOG_INFO("Task list generation!!!");
    ROS_LOG_INFO("forceLimit: %f, %f, %f, %f, %f, %f", qnode_->task_planner_->impedance_default_.force_limit[0], qnode_->task_planner_->impedance_default_.force_limit[1], qnode_->task_planner_->impedance_default_.force_limit[2], qnode_->task_planner_->impedance_default_.force_limit[3], qnode_->task_planner_->impedance_default_.force_limit[4], qnode_->task_planner_->impedance_default_.force_limit[5]);
    ROS_LOG_INFO("stiffness: %f, %f, %f, %f, %f, %f", qnode_->task_planner_->impedance_default_.m[0], qnode_->task_planner_->impedance_default_.m[1], qnode_->task_planner_->impedance_default_.m[2], qnode_->task_planner_->impedance_default_.m[3], qnode_->task_planner_->impedance_default_.m[4], qnode_->task_planner_->impedance_default_.m[5]);
#endif
}

/** @brief Korean: UI 버튼함수, Debug, Run mode에 맞는 로봇의 속도 및 가속도를 설정한다.
 */
void BinPickingDialog::pushButtonSetUIRobotAccVel() {
#if BIN_PICKING_FLAG
    if(ui.radioButton_setRobotAccVel_DEBUG->isChecked())
    {
        ui.task_doubleSpinBox_CSvelRobotA->setValue(0.1); // [m/s]
        ui.task_doubleSpinBox_CSaccRobotA->setValue(0.2); // [m/s^2]
        ui.task_doubleSpinBox_JSvelRobotA->setValue(30.0); // [deg/s]
        ui.task_doubleSpinBox_JSaccRobotA->setValue(45.0); // [deg/s^2]
    }

    if(ui.radioButton_setRobotAccVel_RUN->isChecked())
    {
        ui.task_doubleSpinBox_CSvelRobotA->setValue(0.3); // [m/s]
        ui.task_doubleSpinBox_CSaccRobotA->setValue(0.6); // [m/s^2]
        ui.task_doubleSpinBox_JSvelRobotA->setValue(110.0); // [deg/s]
        ui.task_doubleSpinBox_JSaccRobotA->setValue(150.0); // [deg/s^2]
    }
#endif
}

/** @brief Korean: UI 버튼함수, 부하동정(load identification)을 실행한다.
 */
void BinPickingDialog::pushButtonLoadIdentification() {
#if BIN_PICKING_FLAG
    qnode_->doLoadIdentification(true);
#endif
}

//// KORAS gripper
void BinPickingDialog::pushButtonKorasGripperIntializeCallback()
{
#if BIN_PICKING_FLAG
    uint16_t position = 0;
    ROS_LOG_INFO("Current Driver address #%u", qnode_->grp_driver_address_);
    qnode_->setGrp((uint16_t)KR_GRP::INIT, position, qnode_->grp_driver_address_);
#endif
}

//// KORAS gripper
void BinPickingDialog::pushButtonKorasGripperIntializeVer2Callback()
{
#if BIN_PICKING_FLAG
    uint16_t position = qnode_->grp_init_2_grp_cmd_;
    ROS_LOG_INFO("[Init. 2 - %u]Current Driver address #%u", position, qnode_->grp_driver_address_);
    qnode_->setGrp((uint16_t)KR_GRP::INIT_2, position, qnode_->grp_driver_address_);
#endif
}





//// KORAS gripper
void BinPickingDialog::pushButtonKorasDualGripperIntializeCallback()
{
#if BIN_PICKING_FLAG
    // uint16_t position = 0;
    // qnode_->setGrp((uint16_t)KR_GRP::INIT, position, 1);
    // qnode_->setGrp((uint16_t)KR_GRP::INIT, position, 2);

    // Dual tool initialize
    ROS_LOG_INFO("Dual gripper initialize!");
    qnode_->current_task_list_ = qnode_->task_planner_->module_task_dual_tool_initialize_;
    qnode_->beforeBinPickingTaskStart(true);
    qnode_->task_cycle_ = 0;

#endif
}

void BinPickingDialog::pushButtonKorasGripperSlaveChangeCallback()
{
#if BIN_PICKING_FLAG
    qnode_->grp_driver_address_ = ui.task_spinBox_gripperSlaveNum->value(); // 1, 2, ...
    qnode_->setGrp((uint16_t)KR_GRP::CHANGE_ADDRESS, 0, qnode_->grp_driver_address_);
#endif
}

void BinPickingDialog::pushButtonKorasGripperSetInitMinMaxValue()
{
#if BIN_PICKING_FLAG
    setKORASGripperInitialValue();
#endif
}

void BinPickingDialog::pushButtonKorasGripperOpenCallback()
{
#if BIN_PICKING_FLAG
    uint16_t position = ui.task_spinBox_gripperOpenValue->value(); // (scale: 0(Open)~10000(Close))
    ROS_LOG_INFO("Open!");
    ROS_LOG_INFO("Current Driver address #%u", qnode_->grp_driver_address_);
    qnode_->setGrp((uint16_t)KR_GRP::OPEN, position, qnode_->grp_driver_address_);
#endif
}

void BinPickingDialog::pushButtonKorasGripperCloseCallback()
{
#if BIN_PICKING_FLAG
    uint16_t position = ui.task_spinBox_gripperCloseValue->value(); // (scale: 0(Open)~10000(Close))
    ROS_LOG_INFO("Close!");
    ROS_LOG_INFO("Current Driver address #%u", qnode_->grp_driver_address_);
    qnode_->setGrp((uint16_t)KR_GRP::CLOSE, position, qnode_->grp_driver_address_);
#endif
}

void BinPickingDialog::pushButtonKorasGripperPosCtrlCallback() { // (scale: 0(Open)~10000(Close))
#if BIN_PICKING_FLAG
    uint16_t position = ui.task_spinBox_gripperPosValue->value(); // (scale: 0~10000)
    std::cout << "Pos.: " << position << std::endl;
    ROS_LOG_INFO("Current Driver address #%u", qnode_->grp_driver_address_);
    qnode_->setGrp((uint16_t)KR_GRP::POS_CTRL, position, qnode_->grp_driver_address_);
#endif
}

void BinPickingDialog::pushButtonKorasGripperPosCtrlOnOffCallback() { // (scale: 0(Open)~10000(Close))
#if BIN_PICKING_FLAG
    uint16_t position = ui.task_spinBox_gripperPosValue->value(); // (scale: 0~10000)
    uint16_t duration = ui.task_spinBox_gripperPosDuration->value();
    std::cout << "Pos.: " << position << std::endl;
    if(ui.pushButton_KorasGripperPosCtrlOnOff->isChecked()) {
        qnode_->setGrp((uint16_t)KR_GRP::CLOSE, position, qnode_->grp_driver_address_); // Close
    } else {
        qnode_->setGrp((uint16_t)KR_GRP::POS_CTRL, position, qnode_->grp_driver_address_); // Position control
    }
#endif
}

void BinPickingDialog::pushButtonKorasGripperVacuumOnOffCallback() {
#if BIN_PICKING_FLAG
    uint16_t position = ui.task_spinBox_gripperPosValue->value();
    // std::cout << "Pos.: " << position << std::endl;

#if NEW_VER_KORAS_GRIPPER_PACKAGE
    if(ui.pushButton_KorasGripperVacuumOnOff->isChecked()) {
        qnode_->setGrpVacOn(); // Vac ON
    } else {
        qnode_->setGrpVacOff(); // Vac OFF
    }
#else
    if(ui.pushButton_KorasGripperVacuumOnOff->isChecked()) {
        qnode_->setGrp((uint16_t)KR_GRP::VACUUM_ON, position, qnode_->grp_driver_address_); // Vac ON
    } else {
        qnode_->setGrp((uint16_t)KR_GRP::VACUUM_OFF, position, qnode_->grp_driver_address_); // Vac OFF
    }
#endif

#endif
}


void BinPickingDialog::pushButtonKorasGripperToolIndexChangeCallback()
{
#if BIN_PICKING_FLAG
    qnode_->setToolChangingCurrentIndex(ui.task_spinBox_toolIndex->value()); // tool slot #1, 2, ...
#endif
}

void BinPickingDialog::pushButtonKorasGripperTipIndexChangeCallback()
{
#if BIN_PICKING_FLAG
    qnode_->setTipChangingCurrentIndex(ui.task_spinBox_tipIndex->value()); // tool slot #1, 2, ...
#endif
}



void BinPickingDialog::pushButtonDoTaskInfoPrintCallback() {
#if BIN_PICKING_FLAG
    if(ui.pushButton_doTaskInfoPrint->isChecked()) qnode_->m_bin_picking_node->is_task_recognition_on = true;
    else qnode_->m_bin_picking_node->is_task_recognition_on = false;
#endif
}

void BinPickingDialog::pushButtonToolTeachingProcess() {

#if BIN_PICKING_FLAG
    if(ui.radioButton_assemblyUI_isToolTeachingProcess->isChecked()) ui.pushButton_setToolChangingPoseSensorFrame->setEnabled(true);
    else ui.pushButton_setToolChangingPoseSensorFrame->setEnabled(false);
#endif
}

void BinPickingDialog::pushButtonModbusSetIPAddress() {
#if BIN_PICKING_FLAG
    std::string ip = ui.lineEdit_MODBUS_IP->text().toStdString();
    int portno = static_cast<int>(ui.lineEdit_MODBUS_PORT->text().toDouble());
    qnode_->ip_modbus_ = ip;
    qnode_->port_modbus_ = portno;
#endif
}

void BinPickingDialog::pushButtonModbusConnect() {
#if BIN_PICKING_FLAG
    pushButtonModbusSetIPAddress();
    qnode_->setMODBUSComm(true);
#endif
}

void BinPickingDialog::pushButtonModbusClose() {
#if BIN_PICKING_FLAG
    qnode_->setMODBUSComm(false);
#endif
}

void BinPickingDialog::pushButtonModbusLatheChuckOnOff() {
#if BIN_PICKING_FLAG
    if(ui.pushButton_MODBUS_latheChuckOnOff->isChecked())
    {
        qnode_->setMODBUSComm(true);
        qnode_->sendMessageMODBUS(0, 1); // CLOSE
        qnode_->setMODBUSComm(false);
        ui.pushButton_MODBUS_latheChuckOnOff->setText("LATHE CHUCK OPEN");
    }
    else
    {
        qnode_->setMODBUSComm(true);
        qnode_->sendMessageMODBUS(1, 1); // OPEN
        qnode_->setMODBUSComm(false);
        ui.pushButton_MODBUS_latheChuckOnOff->setText("LATHE CHUCK CLOSE");
    }
#endif
}

void BinPickingDialog::pushButtonModbusMillingChuckOnOff() {
#if BIN_PICKING_FLAG
    if(ui.pushButton_MODBUS_millingChuckOnOff->isChecked())
    {
        qnode_->setMODBUSComm(true);
        qnode_->sendMessageMODBUS(2, 1); // CLOSE
        qnode_->setMODBUSComm(false);
        ui.pushButton_MODBUS_millingChuckOnOff->setText("MILLING CHUCK OPEN");
    }
    else
    {
        qnode_->setMODBUSComm(true);
        qnode_->sendMessageMODBUS(3, 1); // OPEN
        qnode_->setMODBUSComm(false);
        ui.pushButton_MODBUS_millingChuckOnOff->setText("MILLING CHUCK CLOSE");
    }
#endif
}

void BinPickingDialog::pushButtonModbusDoorOnOff() {
#if BIN_PICKING_FLAG
    if(ui.pushButton_MODBUS_doorOnOff->isChecked())
    {
        qnode_->setMODBUSComm(true);
        qnode_->sendMessageMODBUS(5, 1); // OPEN
        qnode_->setMODBUSComm(false);
        ui.pushButton_MODBUS_doorOnOff->setText("CNC DOOR CLOSE");
    }
    else
    {
        qnode_->setMODBUSComm(true);
        qnode_->sendMessageMODBUS(4, 1); // CLOSE
        qnode_->setMODBUSComm(false);
        ui.pushButton_MODBUS_doorOnOff->setText("CNC DOOR OPEN");
    }
#endif
}

void BinPickingDialog::pushButtonModbusLEDGreenOnOff() {
#if BIN_PICKING_FLAG
    if(ui.pushButton_MODBUS_ledGreen->isChecked())
    {
        qnode_->setMODBUSComm(true);
        qnode_->sendMessageMODBUS(0x20, 1, true); // ON
        qnode_->setMODBUSComm(false);
        ui.pushButton_MODBUS_ledGreen->setText("LED GREEN ON");
    }
    else
    {
        qnode_->setMODBUSComm(true);
        qnode_->sendMessageMODBUS(0x20, 1, false); // OFF
        qnode_->setMODBUSComm(false);
        ui.pushButton_MODBUS_ledGreen->setText("LED GREEN OFF");
    }
#endif
}

void BinPickingDialog::pushButtonModbusLEDYellowOnOff() {
#if BIN_PICKING_FLAG
    if(ui.pushButton_MODBUS_ledYellow->isChecked())
    {
        qnode_->setMODBUSComm(true);
        qnode_->sendMessageMODBUS(0x21, 1, true); // ON
        qnode_->setMODBUSComm(false);
        ui.pushButton_MODBUS_ledYellow->setText("LED YELLOW ON");
    }
    else
    {
        qnode_->setMODBUSComm(true);
        qnode_->sendMessageMODBUS(0x21, 1, false); // OFF
        qnode_->setMODBUSComm(false);
        ui.pushButton_MODBUS_ledYellow->setText("LED YELLOW OFF");
    }
#endif
}

void BinPickingDialog::pushButtonModbusLEDRedOnOff() {
#if BIN_PICKING_FLAG
    if(ui.pushButton_MODBUS_ledRed->isChecked())
    {
        qnode_->setMODBUSComm(true);
        qnode_->sendMessageMODBUS(0x22, 1, true); // ON
        qnode_->setMODBUSComm(false);
        ui.pushButton_MODBUS_ledRed->setText("LED RED ON");
    }
    else
    {
        qnode_->setMODBUSComm(true);
        qnode_->sendMessageMODBUS(0x22, 1, false); // CLOSE
        qnode_->setMODBUSComm(false);
        ui.pushButton_MODBUS_ledRed->setText("LED RED OFF");
    }
#endif
}

void BinPickingDialog::pushButtonSetDefaultTCP() {
#if BIN_PICKING_FLAG
    changeRobotTCP(7); // TCP #7, tool master
#endif
}

void BinPickingDialog::pushButtonSelectTCP()
{
#if BIN_PICKING_FLAG
    uint16_t tcp_num = ui.task_spinBox_robotTCPNum->value(); // 1, 2, ...
    changeRobotTCP(tcp_num);
#endif
}
/** @brief Korean: UI의 관절공간 및 직교공간 상의 목표 관절각도/자세를 UI에 업데이트한다.
 */
void BinPickingDialog::setInitListValue() {
#if BIN_PICKING_FLAG
    // Initial CS pose
    std::vector<double> currentRobotAPose = { 0.48000, -0.08000, 0.33000, -180.0, 0.0, -180.0 };
    // std::vector<double> currentRobotAPose = { 0.0, -0.20000, 0.33000, -180.0, 0.0, -180.0 };
    for (std::size_t i = 0; i < currentRobotAPose.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(currentRobotAPose[i], 'f', 5));
    }
    // Initial JS pose
    std::vector<double> starting_pose_q = { 94.273, -68.145, -88.882, -112.889, 90.362, 3.922 };
    // std::vector<double> starting_pose_q = { -90, 110, -210, 0, -80, 0 };


    for (std::size_t i = 0; i < starting_pose_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(starting_pose_q[i], 'f', 3));
    }
#endif
}

////////////////// 3D scanning

void BinPickingDialog::pushButtonDoScanningZIVIDClickedCallback() {
#if BIN_PICKING_FLAG
#if DRFL_CONTROL
    /////////////////////////////////////////////////
    qnode_->getDrflCurrentPose(); //// 두산 로봇 CS pose 할당
    /////////////////////////////////////////////////
#endif
    taskScanningParameter scan_parameter;
    scan_parameter.target_id = taskTargetParts_[0]->text().toInt();
    scan_parameter.target_name = taskTargetParts_[1]->text().toStdString();
    scan_parameter.is_mask_pixel_fixed = ui.radioButton_assemblyUI_isMaskFixed->isChecked();
    std::vector<int> mask_pixel_list(4,10);
    mask_pixel_list[0] = fixedMaskPixelList_[0]->text().toInt();
    mask_pixel_list[1] = fixedMaskPixelList_[1]->text().toInt();
    mask_pixel_list[2] = fixedMaskPixelList_[2]->text().toInt();
    mask_pixel_list[3] = fixedMaskPixelList_[3]->text().toInt();
    scan_parameter.mask_pixel_list = mask_pixel_list;
    scan_parameter.sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();
    scan_parameter.is_base_frame_unknown = false;
    scan_parameter.do_scan_sampling = ui.radioButton_assemblyUI_scanSampling->isChecked();
    scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    scan_parameter.do_single_matching = true; // matching service
    scan_parameter.do_not_scan_do_load_data = ui.radioButton_assemblyUI_scanDataLoad->isChecked();
    scan_parameter.skip_detection_mask = ui.radioButton_assemblyUI_skipDetectionMask->isChecked();


    //// 241125 추가
    scan_parameter.robot_dh_vec = qnode_->getRobotDHParameters(); // [m], [deg]
    scan_parameter.robot_tcp_default = qnode_->getRobotDefaultTCP(); // [m], [deg]
    scan_parameter.robot_tcp = qnode_->getRobotTCP(); // [m], [deg]



    arr2Vec(qnode_->params_.meas.q , scan_parameter.scan_position); // scanning joint position
    std::vector<double> scan_position_q = scan_parameter.scan_position;
    ROS_LOG_WARN("Current JS position [deg]: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", scan_position_q[0], scan_position_q[1], scan_position_q[2], scan_position_q[3], scan_position_q[4], scan_position_q[5]);


    // // dlgDoScanning(scan_parameter); // signal to BinPickingDialog
    // if(scan_parameter.target_id >= 31 && scan_parameter.target_id < 45) {
    //     qnode_->m_bin_picking_node->scanZIVID(scan_parameter);
    // } else {
    //     ROS_LOG_INFO("Wrong target id!");
    // }
    qnode_->m_bin_picking_node->scanZIVID(scan_parameter);

#endif
}

void BinPickingDialog::pushButtonDoTemplateMatchingClickedCallback() { // 공유작업
#if BIN_PICKING_FLAG
    // size_t sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();
    // bool do_scan_sampling = ui.radioButton_assemblyUI_scanSampling->isChecked();
    // qnode_->m_bin_picking_node->doTemplateMatching(true, do_scan_sampling, sampling_num);
    // pushButtonSetPreDetectedPoseClickedCallback();
#endif
}

void BinPickingDialog::pushButtonDoTemplateMatchingHoleClickedCallback() { // connector hole pose
#if BIN_PICKING_FLAG
    // size_t sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();
    // if(ui.radioButton_assemblyUI_scanSampling->isChecked()) {
    //     qnode_->m_bin_picking_node->doTemplateMatchingHole(true, true, sampling_num);
    // }
    // else {
    //     qnode_->m_bin_picking_node->doTemplateMatchingHole(true, false, sampling_num);
    // }
    // pushButtonSetPreDetectedPoseClickedCallback();
#endif
}

void BinPickingDialog::pushButtonDoTemplateMatchingBinPickingClickedCallback() { // bin picking
#if BIN_PICKING_FLAG

#if DRFL_CONTROL
    /////////////////////////////////////////////////
    qnode_->getDrflCurrentPose(); //// 두산 로봇 CS pose 할당
    /////////////////////////////////////////////////
#endif
    taskTemplateMatchingParameter matching_parameter;
    matching_parameter.debug_mode = true;
    matching_parameter.is_base_frame_unknown = false;
    matching_parameter.do_scan_sampling = ui.radioButton_assemblyUI_scanSampling->isChecked();
    matching_parameter.sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();
    qnode_->m_bin_picking_node->doCADMatchingService(matching_parameter);
    pushButtonSetPreDetectedPoseClickedCallback();
#endif
}

void BinPickingDialog::pushButtonDoPoseEstimationBinPickingClickedCallback() { // bin picking
#if BIN_PICKING_FLAG
    taskTemplateMatchingParameter matching_parameter;
    matching_parameter.debug_mode = true;
    matching_parameter.is_base_frame_unknown = false;
    matching_parameter.do_scan_sampling = ui.radioButton_assemblyUI_scanSampling->isChecked();
    matching_parameter.sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();
    qnode_->m_bin_picking_node->doPoseEstimationAI(matching_parameter);
    pushButtonSetEstimatedPoseClickedCallback();
#endif
}

void BinPickingDialog::pushButtonDoTemplateInitializeClickedCallback() { // Template initialize
#if BIN_PICKING_FLAG

    taskTemplateMatchingParameter parameter;
    // Target object
    parameter.target_id = taskTargetParts_[0]->text().toInt();
    parameter.target_name = taskTargetParts_[1]->text().toStdString();

    parameter.robot_dh_vec = qnode_->getRobotDHParameters(); // [m], [deg]
    parameter.robot_tcp_default = qnode_->getRobotDefaultTCP(); // [m], [deg]
    parameter.robot_tcp = qnode_->getRobotTCP(); // [m], [deg]

    bool do_overwrite_JSON = true;
    if(ui.radioButton_assemblyUI_setTemplateParameters->isChecked()) {
        qnode_->m_bin_picking_node->doTemplateInitialize(parameter, true, do_overwrite_JSON);
    }
    else {
        qnode_->m_bin_picking_node->doTemplateInitialize(parameter, false, do_overwrite_JSON);
    }
#endif
}

void BinPickingDialog::pushButtonDoLoadLearningWeightClickedCallback() {
#if BIN_PICKING_FLAG
    taskScanningParameter ui_cmd_parameter;
    ui_cmd_parameter.target_name = ui.lineEdit_targetName->text().toStdString();
    ui_cmd_parameter.weight_number = ui.lineEdit_targetWeightNumber->text().toStdString();
    ui_cmd_parameter.is_sam_mean_size_assigned = false; // SAM paramter
    ui_cmd_parameter.sam_mean_size = 0; // SAM paramter
    ui_cmd_parameter.sam_mask_min_area = 6000;
    ui_cmd_parameter.sam_mask_max_area = 300000;
    qnode_->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);
#endif
}

void BinPickingDialog::pushButtonOptimizationBinWorkspaceClickedCallback() {
#if BIN_PICKING_FLAG
    size_t sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();

    taskTemplateMatchingParameter matching_parameter;
    matching_parameter.debug_mode = true;
    matching_parameter.do_scan_sampling = true;
    matching_parameter.do_scan_sampling = ui.radioButton_assemblyUI_scanSampling->isChecked();
    matching_parameter.sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();

    matching_parameter.robot_dh_vec = qnode_->getRobotDHParameters();
    matching_parameter.robot_tcp_default = qnode_->getRobotDefaultTCP();
    matching_parameter.robot_tcp = qnode_->getRobotTCP();

    qnode_->m_bin_picking_node->doOptimizationBinPickingWorkspace(matching_parameter);
#endif
}


void BinPickingDialog::pushButtonBinMatchingClickedCallback() {
#if BIN_PICKING_FLAG

    bin_matching_dialog->exec(); // dialog open
    ui.pushButton_doBinMatching->setChecked(false);
#endif
}

void BinPickingDialog::pushButtonRobot2CameraCalibrationClickedCallback() {
#if BIN_PICKING_FLAG

    robot2camera_calibration_dialog->exec(); // dialog open
    ui.pushButton_doRobot2CameraCalibration->setChecked(false);
#endif
}

void BinPickingDialog::pushButtonDoEvaluateGraspingPoseClickedCallback() {
#if BIN_PICKING_FLAG

    robot2camera_calibration_dialog->exec(); // dialog open
    ui.pushButton_doEvaluateGraspingPose->setChecked(false);
#endif
}

void BinPickingDialog::pushButtonDoZigPoseInitialTeachingClickedCallback() {
#if BIN_PICKING_FLAG
    taskTemplateMatchingParameter parameter;
    // Target object
    parameter.target_id = taskTargetParts_[0]->text().toInt();
    parameter.target_name = taskTargetParts_[1]->text().toStdString();
    parameter.robot_dh_vec = qnode_->getRobotDHParameters(); // [m], [deg]
    parameter.robot_tcp_default = qnode_->getRobotDefaultTCP(); // [m], [deg]
    parameter.robot_tcp = qnode_->getRobotTCP(); // [m], [deg]
    bool do_overwrite_JSON = true;
    bool do_set_only_parameters = true;
    //// a) Set Parameters - JSON에 티칭한 zig_pose_to_be_set_by_user_from_initial_teaching 값을 불러와야 함.
    qnode_->m_bin_picking_node->doTemplateInitialize(parameter, do_set_only_parameters, do_overwrite_JSON);
    //// b) Do process - 여기서 출력된 zig_pose_to_be_saved_from_initial_teaching을 JSON에 입력하면 끝.
    qnode_->m_bin_picking_node->doZigPoseInitialTeaching(parameter);

#endif
}


void BinPickingDialog::pushButtonSimulationBinPlacingClickedCallback() {
#if BIN_PICKING_FLAG

    bin_matching_dialog->exec(); // dialog open
    ui.pushButton_doSimulationBinPlacing->setChecked(false);
#endif
}


void BinPickingDialog::pushButtonDoPCLTestFunctionClickedCallback() { // bin picking
#if BIN_PICKING_FLAG

    size_t segmentation_method = 0;
    for(int i=0; i<UI_template_matching_seg_parameter_list_.size(); i++) {
        if(UI_template_matching_seg_parameter_list_[i]->isChecked()) { segmentation_method = i; }
    }
    taskTemplateMatchingParameter parameter;
    // Target object
    parameter.target_id = taskTargetParts_[0]->text().toInt();
    parameter.target_name = taskTargetParts_[1]->text().toStdString();

    parameter.robot_dh_vec = qnode_->getRobotDHParameters();
    parameter.robot_tcp_default = qnode_->getRobotDefaultTCP();
    parameter.robot_tcp = qnode_->getRobotTCP();

    parameter.segmentation_method = segmentation_method;
	// Region growing segmentation
	parameter.rgs_normal_K = ui.task_spinBox_segParam_RGS_normalK->value();
	parameter.rgs_min_size = ui.task_spinBox_segParam_RGS_minSize->value();
	parameter.rgs_max_size = ui.task_spinBox_segParam_RGS_maxSize->value();
	parameter.rgs_thre_angle = ui.task_doubleSpinBox_segParam_RGS_normal->value();
	parameter.rgs_thre_curvature = ui.task_doubleSpinBox_segParam_RGS_curv->value();
	parameter.rgs_neighbor_K = ui.task_spinBox_segParam_RGS_neighborK->value();
	// Color-based region growing segmentation
	parameter.color_rgs_neighbor_R = ui.task_doubleSpinBox_segParam__colorRGS_neighborR->value();
	parameter.color_rgs_thre_1st_color = ui.task_doubleSpinBox_segParam_colorRGS_thre1stColor->value();
	parameter.color_rgs_thre_2nd_color = ui.task_doubleSpinBox_segParam_colorRGS_thre2ndColor->value();
	parameter.color_rgs_min_cluster_size = ui.task_spinBox_segParam_colorRGS_minClusterSize->value();

	parameter.color_rgs_neighbor_R_2 = ui.task_doubleSpinBox_segParam__colorRGS_neighborR_2->value();
	parameter.color_rgs_thre_1st_color_2 = ui.task_doubleSpinBox_segParam_colorRGS_thre1stColor_2->value();
	parameter.color_rgs_thre_2nd_color_2 = ui.task_doubleSpinBox_segParam_colorRGS_thre2ndColor_2->value();
	parameter.color_rgs_min_cluster_size_2 = ui.task_spinBox_segParam_colorRGS_minClusterSize_2->value();

	// Euclidean cluster segmentation
	parameter.euclidean_cluster_tol = ui.task_doubleSpinBox_segParam_EuclideanCluster_tolerance->value();
	parameter.euclidean_min_cluster_size = ui.task_spinBox_segParam_EuclideanCluster_minClusterSize_2->value();
	parameter.euclidean_max_cluster_size = ui.task_spinBox_segParam_EuclideanCluster_maxClusterSize_2->value();


    size_t sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();
    double voxel_downsampling_size = ui.task_doubleSpinBox_featureVoxelDownsamplingSize->value();

    size_t feature_ext_method = 0;
    for(int i=0; i<feature_extraction_method_list_.size(); i++) {
        if(feature_extraction_method_list_[i]->isChecked()) { feature_ext_method = i; }
    }

    if(ui.radioButton_assemblyUI_scanSampling->isChecked()) {
        qnode_->m_bin_picking_node->doPCLTestFunction(parameter, true, true, sampling_num, feature_ext_method, voxel_downsampling_size);
    }
    else {
        qnode_->m_bin_picking_node->doPCLTestFunction(parameter, true, false, sampling_num, feature_ext_method, voxel_downsampling_size);
    }
#endif
}

void BinPickingDialog::pushButtonDoPCLFunctionFeatureExtractionClickedCallback() { // bin picking
#if BIN_PICKING_FLAG
    size_t feature_ext_method = 0;
    for(int i=0; i<feature_extraction_method_list_.size(); i++) {
        if(feature_extraction_method_list_[i]->isChecked()) { feature_ext_method = i; }
    }

    taskTemplateMatchingParameter parameter;
    if(ui.radioButton_assemblyUI_scanSampling->isChecked()) { parameter.do_scan_sampling = true; }
    else { parameter.do_scan_sampling = false; }
    parameter.sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();
    parameter.voxel_downsampling_size = ui.task_doubleSpinBox_featureVoxelDownsamplingSize->value();
    parameter.feature_ext_method = feature_ext_method;

    qnode_->m_bin_picking_node->doPCLFunctionFeatureExtraction(parameter);
#endif
}

void BinPickingDialog::pushButtonDoPCLFunctionSegmentationClickedCallback() { // bin picking
#if BIN_PICKING_FLAG

    size_t segmentation_method = 0;
    for(int i=0; i<UI_template_matching_seg_parameter_list_.size(); i++) {
        if(UI_template_matching_seg_parameter_list_[i]->isChecked()) { segmentation_method = i; }
    }
    taskTemplateMatchingParameter parameter;
    // Target object
    parameter.target_id = taskTargetParts_[0]->text().toInt();
    parameter.target_name = taskTargetParts_[1]->text().toStdString();

    parameter.robot_dh_vec = qnode_->getRobotDHParameters();
    parameter.robot_tcp_default = qnode_->getRobotDefaultTCP();
    parameter.robot_tcp = qnode_->getRobotTCP();

    parameter.segmentation_method = segmentation_method;
	// Region growing segmentation
	parameter.rgs_normal_K = ui.task_spinBox_segParam_RGS_normalK->value();
	parameter.rgs_min_size = ui.task_spinBox_segParam_RGS_minSize->value();
	parameter.rgs_max_size = ui.task_spinBox_segParam_RGS_maxSize->value();
	parameter.rgs_thre_angle = ui.task_doubleSpinBox_segParam_RGS_normal->value();
	parameter.rgs_thre_curvature = ui.task_doubleSpinBox_segParam_RGS_curv->value();
	parameter.rgs_neighbor_K = ui.task_spinBox_segParam_RGS_neighborK->value();
	// Color-based region growing segmentation
	parameter.color_rgs_neighbor_R = ui.task_doubleSpinBox_segParam__colorRGS_neighborR->value();
	parameter.color_rgs_thre_1st_color = ui.task_doubleSpinBox_segParam_colorRGS_thre1stColor->value();
	parameter.color_rgs_thre_2nd_color = ui.task_doubleSpinBox_segParam_colorRGS_thre2ndColor->value();
	parameter.color_rgs_min_cluster_size = ui.task_spinBox_segParam_colorRGS_minClusterSize->value();

	parameter.color_rgs_neighbor_R_2 = ui.task_doubleSpinBox_segParam__colorRGS_neighborR_2->value();
	parameter.color_rgs_thre_1st_color_2 = ui.task_doubleSpinBox_segParam_colorRGS_thre1stColor_2->value();
	parameter.color_rgs_thre_2nd_color_2 = ui.task_doubleSpinBox_segParam_colorRGS_thre2ndColor_2->value();
	parameter.color_rgs_min_cluster_size_2 = ui.task_spinBox_segParam_colorRGS_minClusterSize_2->value();

	// Euclidean cluster segmentation
	parameter.euclidean_cluster_tol = ui.task_doubleSpinBox_segParam_EuclideanCluster_tolerance->value();
	parameter.euclidean_min_cluster_size = ui.task_spinBox_segParam_EuclideanCluster_minClusterSize_2->value();
	parameter.euclidean_max_cluster_size = ui.task_spinBox_segParam_EuclideanCluster_maxClusterSize_2->value();

    if(ui.radioButton_assemblyUI_scanSampling->isChecked()) { parameter.do_scan_sampling = true; }
    else { parameter.do_scan_sampling = false; }
    parameter.sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();


    qnode_->m_bin_picking_node->doPCLFunctionSegmentation(parameter);


#endif
}


void BinPickingDialog::pushButtonSetGraspingPart1PixelSizeClickedCallback() {
#if BIN_PICKING_FLAG
    taskTargetParts_[0]->setText("31"); // harting connector hole

    ui.lineEdit_pixelList_1->setText("0");
    ui.lineEdit_pixelList_2->setText("0");
    ui.lineEdit_pixelList_3->setText("1920");
    ui.lineEdit_pixelList_4->setText("1200");
#endif
}

void BinPickingDialog::pushButtonSetGraspingPart2PixelSizeClickedCallback() {
#if BIN_PICKING_FLAG
    taskTargetParts_[0]->setText("31"); // gear

    ui.lineEdit_pixelList_1->setText("220");
    ui.lineEdit_pixelList_2->setText("400");
    ui.lineEdit_pixelList_3->setText("400");
    ui.lineEdit_pixelList_4->setText("410");
#endif
}

void BinPickingDialog::pushButtonSetPreDetectedPoseClickedCallback() {
#if BIN_PICKING_FLAG
    std::vector<double> detected_pose = qnode_->m_bin_picking_node->getDetectedPose();
    std::vector<double> tool_relative_pose(6, 0.0);
    double approach_distance = qnode_->m_bin_picking_node->getApproachDistance(); // [m]
    tool_relative_pose[2] = -approach_distance; // Tool 기준이므로 부호 반대
    // gripper open length
    uint16_t gripper_open_length = qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->getGripperOpenLength();
    ui.task_spinBox_gripperPosValue->setValue(gripper_open_length);

    std::vector<double> current_pose = detected_pose;
    std::vector<double> approach_pose;
    // detected_pose로부터 Tool frame 기준의 상대 이동 (approach distance 고려)
    for (std::size_t i = 0; i < 3; i++) { current_pose[i + 3] = current_pose[i + 3] * kDeg2Rad; }
    qnode_->m_bin_picking_node->m_bp_math.transformToolFrame(tool_relative_pose, current_pose, approach_pose); // [m], [rad]
    for (std::size_t i = 0; i < 3; i++) { approach_pose[i + 3] = approach_pose[i + 3] * kRad2Deg; }
    for (std::size_t i = 0; i < detected_pose.size(); i++)
    {
        if(ui.pushButton_setPreDetectedPose->isChecked())
        {
            robotATargetXList_[i]->setText(QString::number(approach_pose[i], 'f', 5)); // Approach distance 고려
        }
        else
        {
            robotATargetXList_[i]->setText(QString::number(detected_pose[i], 'f', 5)); // detected grasping pose
        }
    }
#endif
}

void BinPickingDialog::pushButtonSetSubDetectedPoseClickedCallback() {
#if BIN_PICKING_FLAG
    std::vector<double> detected_sub_pose = qnode_->m_bin_picking_node->getDetectedSubPose();
    std::vector<double> tool_relative_pose(6, 0.0);
    double approach_distance = qnode_->m_bin_picking_node->getApproachDistance(); // [m]
    tool_relative_pose[2] = -approach_distance; // Tool 기준이므로 부호 반대
    // gripper open length
    uint16_t gripper_open_length = qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->getGripperOpenLength();
    ui.task_spinBox_gripperPosValue->setValue(gripper_open_length);

    std::vector<double> current_pose = detected_sub_pose;
    std::vector<double> approach_pose;
    // detected_pose로부터 Tool frame 기준의 상대 이동 (approach distance 고려)
    for (std::size_t i = 0; i < 3; i++) { current_pose[i + 3] = current_pose[i + 3] * kDeg2Rad; }
    qnode_->m_bin_picking_node->m_bp_math.transformToolFrame(tool_relative_pose, current_pose, approach_pose); // [m], [rad]
    for (std::size_t i = 0; i < 3; i++) { approach_pose[i + 3] = approach_pose[i + 3] * kRad2Deg; }
    for (std::size_t i = 0; i < detected_sub_pose.size(); i++)
    {
        if(ui.pushButton_setSubDetectedPose->isChecked())
        {
            robotATargetXList_[i]->setText(QString::number(approach_pose[i], 'f', 5)); // Approach distance 고려
        }
        else
        {
            robotATargetXList_[i]->setText(QString::number(detected_sub_pose[i], 'f', 5)); // detected grasping pose
        }
    }
#endif
}


void BinPickingDialog::pushButtonSetEstimatedPoseClickedCallback() {
#if BIN_PICKING_FLAG
    std::vector<double> detected_pose = qnode_->m_bin_picking_node->getEstimatedPose();
    std::vector<double> tool_relative_pose(6, 0.0);
    double approach_distance = qnode_->m_bin_picking_node->getApproachDistance(); // [m]
    // tool_relative_pose[2] = -approach_distance; // Tool 기준이므로 부호 반대
    tool_relative_pose[2] = -approach_distance - 0.06; // Tool 기준이므로 부호 반대
    // gripper open length
    uint16_t gripper_open_length = qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->getGripperOpenLength();
    ui.task_spinBox_gripperPosValue->setValue(gripper_open_length);

    std::vector<double> current_pose = detected_pose;
    std::vector<double> approach_pose;
    // detected_pose로부터 Tool frame 기준의 상대 이동 (approach distance 고려)
    for (std::size_t i = 0; i < 3; i++) { current_pose[i + 3] = current_pose[i + 3] * kDeg2Rad; }
    qnode_->m_bin_picking_node->m_bp_math.transformToolFrame(tool_relative_pose, current_pose, approach_pose); // [m], [rad]
    for (std::size_t i = 0; i < 3; i++) { approach_pose[i + 3] = approach_pose[i + 3] * kRad2Deg; }
    for (std::size_t i = 0; i < detected_pose.size(); i++)
    {
        if(ui.pushButton_setEstimatedPose->isChecked())
        {
            robotATargetXList_[i]->setText(QString::number(approach_pose[i], 'f', 5)); // Approach distance 고려
        }
        else
        {
            robotATargetXList_[i]->setText(QString::number(detected_pose[i], 'f', 5)); // detected grasping pose
        }
    }
#endif
}

void BinPickingDialog::pushButtonSetEstimatedSubPoseClickedCallback() {
#if BIN_PICKING_FLAG
    std::vector<double> detected_pose = qnode_->m_bin_picking_node->getEstimatedSubPose();
    std::vector<double> tool_relative_pose(6, 0.0);
    double approach_distance = qnode_->m_bin_picking_node->getApproachDistance(); // [m]
    // tool_relative_pose[2] = -approach_distance; // Tool 기준이므로 부호 반대
    tool_relative_pose[2] = -approach_distance - 0.06; // Tool 기준이므로 부호 반대
    // gripper open length
    uint16_t gripper_open_length = qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->getGripperOpenLength();
    ui.task_spinBox_gripperPosValue->setValue(gripper_open_length);

    std::vector<double> current_pose = detected_pose;
    std::vector<double> approach_pose;
    // detected_pose로부터 Tool frame 기준의 상대 이동 (approach distance 고려)
    for (std::size_t i = 0; i < 3; i++) { current_pose[i + 3] = current_pose[i + 3] * kDeg2Rad; }
    qnode_->m_bin_picking_node->m_bp_math.transformToolFrame(tool_relative_pose, current_pose, approach_pose); // [m], [rad]
    for (std::size_t i = 0; i < 3; i++) { approach_pose[i + 3] = approach_pose[i + 3] * kRad2Deg; }
    for (std::size_t i = 0; i < detected_pose.size(); i++)
    {
        if(ui.pushButton_setSubEstimatedPose->isChecked())
        {
            robotATargetXList_[i]->setText(QString::number(approach_pose[i], 'f', 5)); // Approach distance 고려
        }
        else
        {
            robotATargetXList_[i]->setText(QString::number(detected_pose[i], 'f', 5)); // detected grasping pose
        }
    }
#endif
}


/** @brief Korean: UI 버튼함수, 정의된 관절공간 상의 목표 관절각도를 UI에 업데이트한다.
 */
void BinPickingDialog::pushButtonSetUIJSPosition1ClickedCallback() {
#if BIN_PICKING_FLAG
    std::vector<double> position_q = { 94.273, -68.145, -88.882, -112.889, 90.362, 3.922 }; // (CS: 0.48000, -0.08, 0.33000, -180.0, 0.0, -180.0)
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPosition2ClickedCallback() {
#if BIN_PICKING_FLAG
    // JS position
    // std::vector<double> position_q_cylinder_demo = { -59.528, 118.811, -200.909, -43.154, -117.118, 17.200 }; // Cylinder feeding JS 자세
    // std::vector<double> position_q_eb_joint_demo = { -59.528, 118.811, -200.909, -43.154, -117.118, 17.200 }; // eb joint feeding JS 자세
    std::vector<double> position_q_cylinder_demo = { 98.193, -61.121, -119.278, -44.839, 83.722, 6.157 }; // Cylinder feeding JS 자세
    std::vector<double> position_q_eb_joint_demo = { 98.193, -61.121, -119.278, -44.839, 83.722, 6.157 }; // eb joint feeding JS 자세
    std::vector<double> position_q;

    if(bin_picking_target_object_list_[5]->isChecked()) { position_q = position_q_cylinder_demo; }
    else if(bin_picking_target_object_list_[2]->isChecked()) { position_q = position_q_eb_joint_demo; }
    else if(bin_picking_target_object_list_[3]->isChecked()) { position_q = position_q_eb_joint_demo; }
    else if(bin_picking_target_object_list_[4]->isChecked()) { position_q = position_q_eb_joint_demo; }
    else { position_q = position_q_cylinder_demo; }

    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPosition3_1ClickedCallback() {
#if BIN_PICKING_FLAG

    // CS pose: 0.15648, 0.31521, 0.50002, 179.935, 0.002, 179.999
    // std::vector<double> position_q = { 48.858, -72.125, -101.632, -96.104, 90.024, 48.934 }; // 파지 대기 자세, ZIVID (eye-to-hand)
    std::vector<double> position_q = { 103.298, -42.450, -135.403, -57.847, 82.598, 11.343 }; // dual gripper, 파지 대기 자세, ZIVID (eye-to-hand)


    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPosition3_2ClickedCallback() {
#if BIN_PICKING_FLAG

    std::vector<double> position_q = { -105.429, -65.126, -93.997, -100.813, 93.297, -14.272 }; // dual gripper, 파지 대기 자세, ZIVID (eye-to-hand)
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPosition4ClickedCallback() {
#if BIN_PICKING_FLAG
    std::vector<double> position_q = { -86.540, -68.144, -88.879, -112.888, 90.360, 3.926 }; // ZIVID 측정 자세 2 (드럼통 구멍 측정)
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPosition5_1ClickedCallback() {
#if BIN_PICKING_FLAG
    // JS position
    std::vector<double> position_q_cylinder_demo = { -112.762, -63.835, -125.147, 8.196, 108.738, -0.867 }; // Cylinder feeding JS 자세
    std::vector<double> position_q_eb_joint_demo = { -112.762, -63.835, -125.147, 8.196, 108.738, -0.867 }; // eb joint feeding JS 자세

    std::vector<double> position_q;

    if(bin_picking_target_object_list_[5]->isChecked()) { position_q = position_q_cylinder_demo; }
    else if(bin_picking_target_object_list_[2]->isChecked()) { position_q = position_q_eb_joint_demo; }
    else if(bin_picking_target_object_list_[3]->isChecked()) { position_q = position_q_eb_joint_demo; }
    else if(bin_picking_target_object_list_[4]->isChecked()) { position_q = position_q_eb_joint_demo; }
    else { position_q = position_q_cylinder_demo; }

    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
    // // CS pose
    // std::vector<double> pose_x = { 0.90000, -0.02233, 0.25852, 179.991, 0.498, 179.223 };
    // for (std::size_t i = 0; i < pose_x.size(); i++) {
    //     robotATargetXList_[i]->setText(QString::number(pose_x[i], 'f', 5));
    // }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPosition5_2ClickedCallback() {
#if BIN_PICKING_FLAG
    // JS position
    std::vector<double> position_q_cylinder_demo = { 17.468, -110.583, -68.974, -134.589, 103.231, -77.610 }; // Cylinder Unloading 포즈 위 4cm JS 자세
    std::vector<double> position_q_eb_joint_demo = { -10.514, -87.328, -101.651, -80.596, 90.098, -9.650 }; // eb joint feeding JS 자세
    // std::vector<double> position_q_eb_joint_demo = { 9.619, -108.315, -79.235, -81.895, 89.906, 10.458 }; // Cylinder feeding JS 자세

    std::vector<double> position_q;

    if(bin_picking_target_object_list_[5]->isChecked()) { position_q = position_q_cylinder_demo; }
    else if(bin_picking_target_object_list_[2]->isChecked()) { position_q = position_q_eb_joint_demo; }
    else if(bin_picking_target_object_list_[3]->isChecked()) { position_q = position_q_eb_joint_demo; }
    else if(bin_picking_target_object_list_[4]->isChecked()) { position_q = position_q_eb_joint_demo; }
    else { position_q = position_q_cylinder_demo; }

    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
    // CS pose
    std::vector<double> pose_x = { 0.90000, -0.02233, 0.25852, 179.991, 0.498, 179.223 };
    for (std::size_t i = 0; i < pose_x.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(pose_x[i], 'f', 5));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPosition5_3ClickedCallback() {
#if BIN_PICKING_FLAG
    // JS position
    std::vector<double> position_q = { -0.510, -113.416, -69.432, -134.141, 92.437, -87.860 }; // Cylinder 적재 JS 자세
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPosition6ClickedCallback() {
#if BIN_PICKING_FLAG
    std::vector<double> position_q = { 53.155, -86.286, -56.958, -126.789, 90.087, 53.167 }; // ZIVID 측정 자세 3 (scanning distance: 850mm, right angle)
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPosition7ClickedCallback() {
#if BIN_PICKING_FLAG
    // 접근 JS position // CS pose
    // 17.321, -76.710, -108.365, -102.034, 46.510, 114.203 // 0.57094, -0.30504, 0.39975, 179.983, 46.106, 90.071
    // 삽입 직전 JS position // CS pose
    // 14.467, -76.695, -108.913, -98.872, 45.724, 110.389 // 0.57087, -0.33327, 0.39972, 179.999, 46.111, 90.084
    std::vector<double> position_q = { 14.467, -76.695, -108.913, -98.872, 45.724, 110.389 }; // Assembly trajectory - case 1, 아래로 휘어짐, 전방 삽입
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPosition8ClickedCallback() {
#if BIN_PICKING_FLAG
    std::vector<double> position_q = { 23.017, -42.048, -131.333, -53.940, 73.889, 16.772 }; // Assembly trajectory - case 2, 아래로 휘어짐, 후방 삽입
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPosition9ClickedCallback() { // Case 3
#if BIN_PICKING_FLAG
    // 접근 JS position // CS pose
    // -14.428, -92.776, -91.246, -85.904, 89.984, -59.316 // 0.65512, -0.34605, 0.33218, 179.999, 0.006, -135.043
    // 삽입 직전 JS position // CS pose
    // -14.233, -93.619, -90.347, -85.962, 89.988, -59.124 // 0.66513, -0.34604, 0.33227, -179.993, 0.010, -135.043
    std::vector<double> position_q = { -14.233, -93.619, -90.347, -85.962, 89.988, -59.124 }; // Assembly trajectory - case 3, 우측 휘어짐, 전방 삽입
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPosition10ClickedCallback() { // Case 4
#if BIN_PICKING_FLAG
    // 접근 JS position // CS pose
    // -7.446, -111.793, -62.807, -95.375, 89.976, 37.618 // 0.88712, -0.28919, 0.38275, -179.971, -0.026, 135.000
    // 삽입 직전 JS position // CS pose
    // -11.167, -112.419, -67.036, -90.519, 89.976, 33.883 // 0.87505, -0.34780, 0.33181, -179.975, -0.028, 135.019
    std::vector<double> position_q = { -11.167, -112.419, -67.036, -90.519, 89.976, 33.883 }; // Assembly trajectory - case 4, 우측 휘어짐, 후방 삽입
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPosition11ClickedCallback() { // Case 4
#if BIN_PICKING_FLAG
    // 접근 JS position // CS pose
    // -7.446, -111.793, -62.807, -95.375, 89.976, 37.618 // 0.88712, -0.28919, 0.38275, -179.971, -0.026, 135.000
    // 삽입 직전 JS position // CS pose
    // -11.167, -112.419, -67.036, -90.519, 89.976, 33.883 // 0.87505, -0.34780, 0.33181, -179.975, -0.028, 135.019
    std::vector<double> position_q = {-270, 0, 90, 0, 90, 270}; // Assembly trajectory - case 4, 우측 휘어짐, 후방 삽입
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPosition12ClickedCallback() { // Case 4
#if BIN_PICKING_FLAG
    // 접근 JS position // CS pose
    // -7.446, -111.793, -62.807, -95.375, 89.976, 37.618 // 0.88712, -0.28919, 0.38275, -179.971, -0.026, 135.000
    // 삽입 직전 JS position // CS pose
    // -11.167, -112.419, -67.036, -90.519, 89.976, 33.883 // 0.87505, -0.34780, 0.33181, -179.975, -0.028, 135.019
    std::vector<double> position_q = {-288.520, -16.511, 103.542, 2.425, 79.684, 269.132}; // Assembly trajectory - case 4, 우측 휘어짐, 후방 삽입
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}
/** @brief Korean: UI 버튼함수, 정의된 관절공간 상의 목표 관절각도를 UI에 업데이트한다.
 */
void BinPickingDialog::pushButtonSetUIJSPositionHomeClickedCallback() {
#if BIN_PICKING_FLAG
    std::vector<double> position_q = { 0.0, -90.0, 0.0, -90.0, 0.0, 0.0 }; // (CS: 0.48000, -0.08, 0.33000, -180.0, 0.0, -180.0)
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

/** @brief Korean: UI 버튼함수, 정의된 관절공간 상의 목표 관절각도를 UI에 업데이트한다.
 */
void BinPickingDialog::pushButtonSetUIJSPositionHome2ClickedCallback() {
#if BIN_PICKING_FLAG
    double angle_scale = 0.5;
    std::vector<double> position_q = { 0.0, -90.0, 0.0, -90.0, 0.0, 0.0 }; // (CS: 0.48000, -0.08, 0.33000, -180.0, 0.0, -180.0)
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i] + angle_scale, 'f', 3));
    }
#endif
}




void BinPickingDialog::pushButtonSetUIZIVIDCalPoseInitClickedCallback() {
#if BIN_PICKING_FLAG
    // CS pose
    std::vector<double> pose_x = { 0.67811, 0.13712, 0.43241, 179.983, 0.019, -179.995 };
    for (std::size_t i = 0; i < pose_x.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(pose_x[i], 'f', 5));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIZIVIDCalPose1ClickedCallback() {
#if BIN_PICKING_FLAG
    // CS pose
    std::vector<double> pose_x = { 0.67811, 0.13712, 0.43241, 179.983, 0.019, -179.995 };
    for (std::size_t i = 0; i < pose_x.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(pose_x[i], 'f', 5));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIZIVIDCalPose2ClickedCallback() {
#if BIN_PICKING_FLAG
    // CS pose
    std::vector<double> pose_x = { 0.72130, 0.20144, 0.47259, 179.981, -14.561, -133.419 };
    for (std::size_t i = 0; i < pose_x.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(pose_x[i], 'f', 5));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIZIVIDCalPose3ClickedCallback() {
#if BIN_PICKING_FLAG
    // CS pose
    std::vector<double> pose_x = { 0.67820, 0.23088, 0.37689, -164.590, 0.040, -179.999 };
    for (std::size_t i = 0; i < pose_x.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(pose_x[i], 'f', 5));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIZIVIDCalPose4ClickedCallback() {
#if BIN_PICKING_FLAG
    // CS pose
    std::vector<double> pose_x = { 0.49795, 0.16613, 0.50394, -167.884, 8.555, -138.378 };
    for (std::size_t i = 0; i < pose_x.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(pose_x[i], 'f', 5));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIZIVIDCalPose5ClickedCallback() {
#if BIN_PICKING_FLAG
    // CS pose
    std::vector<double> pose_x = { 0.46505, 0.13706, 0.62891, 179.963, 20.900, 179.990 };
    for (std::size_t i = 0; i < pose_x.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(pose_x[i], 'f', 5));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIZIVIDCalPose6ClickedCallback() {
#if BIN_PICKING_FLAG
    // CS pose
    std::vector<double> pose_x = { 0.73910, -0.08172, 0.26257, 140.458, -26.142, 151.874 };
    for (std::size_t i = 0; i < pose_x.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(pose_x[i], 'f', 5));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIZIVIDCalPose7ClickedCallback() {
#if BIN_PICKING_FLAG
    // CS pose
    std::vector<double> pose_x = { 0.67820, -0.03076, 0.23448, 133.817, 0.036, -179.994 };
    for (std::size_t i = 0; i < pose_x.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(pose_x[i], 'f', 5));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIZIVIDCalPose8ClickedCallback() {
#if BIN_PICKING_FLAG
    // CS pose
    std::vector<double> pose_x = { 0.80161, -0.00248, 0.19734, 149.494, -25.165, 163.820 };
    for (std::size_t i = 0; i < pose_x.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(pose_x[i], 'f', 5));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIZIVIDCalPose9ClickedCallback() {
#if BIN_PICKING_FLAG
    // CS pose
    std::vector<double> pose_x = { 0.83995, 0.13716, 0.29987, 173.275, -25.218, 178.798 };
    for (std::size_t i = 0; i < pose_x.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(pose_x[i], 'f', 5));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIZIVIDCalPose10ClickedCallback() {
#if BIN_PICKING_FLAG
    // CS pose
    std::vector<double> pose_x = { 0.53818, 0.13699, 0.03567, 179.987, 0.407, 179.190 };
    for (std::size_t i = 0; i < pose_x.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(pose_x[i], 'f', 5));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIZIVIDCalPose11ClickedCallback() {
#if BIN_PICKING_FLAG
    // CS pose
    std::vector<double> pose_x = { 0.53818, 0.13699, 0.03567, 179.987, 0.407, 179.190 };
    for (std::size_t i = 0; i < pose_x.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(pose_x[i], 'f', 5));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIZIVIDCalPose12ClickedCallback() {
#if BIN_PICKING_FLAG
    // CS pose
    std::vector<double> pose_x = { 0.53818, 0.13699, 0.03567, 179.987, 0.407, 179.190 };
    for (std::size_t i = 0; i < pose_x.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(pose_x[i], 'f', 5));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPositionGripperExperiment1ClickedCallback() {
#if BIN_PICKING_FLAG
    std::vector<double> position_q = { 12.282, -100.896, -96.195, -72.502, 89.913, 13.144 }; // 그리퍼 실험 Initial position
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPositionGripperExperiment2ClickedCallback() {
#if BIN_PICKING_FLAG
    std::vector<double> position_q = { 10.309, -120.919, -69.863, -78.977, 89.659, 10.537 }; // 그리퍼 실험 Approach position
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPositionGripperExperiment3ClickedCallback() {
#if BIN_PICKING_FLAG
    std::vector<double> position_q = { 10.309, -123.099, -75.490, -71.181, 89.660, 10.537 }; // 그리퍼 실험 Set position
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

void BinPickingDialog::pushButtonSetUIJSPositionGripperExperiment4ClickedCallback() {
#if BIN_PICKING_FLAG
    std::vector<double> position_q = { 10.310, -123.741, -76.573, -69.456, 89.657, 10.535 }; // 그리퍼 실험 Grasping position
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }
#endif
}

void BinPickingDialog::pushButtonSelectTargetObjectClickedCallback() {
#if BIN_PICKING_FLAG

    double voxel_downsampling_size = ui.task_doubleSpinBox_featureVoxelDownsamplingSize->value();

    size_t target_object_idx = 0;
    for(int i=0; i<bin_picking_target_object_list_.size(); i++) {
        if(bin_picking_target_object_list_[i]->isChecked()) { target_object_idx = i; }
    }


    // if(bin_picking_target_object_list_[10]->isChecked() || bin_picking_target_object_list_[11]->isChecked()) {
    //     qnode_->m_bin_picking_node->current_target_object_idx_ = target_object_idx - 10;
    //     qnode_->m_bin_picking_node->before_target_object_idx_ = target_object_idx - 10;
    // } else {
    //     qnode_->m_bin_picking_node->current_target_object_idx_ = target_object_idx;
    //     qnode_->m_bin_picking_node->before_target_object_idx_ = target_object_idx;
    // }
    qnode_->m_bin_picking_node->current_target_object_idx_ = target_object_idx;
    qnode_->m_bin_picking_node->before_target_object_idx_ = target_object_idx;

    CTARGET_OBJECT_DATA* ptr_object_now = qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_];


    //// # of gripper driver: 2
    ptr_object_now->grp_initial_min_position.resize(2);
    ptr_object_now->grp_initial_max_position.resize(2);
    ptr_object_now->is_grp_set_min_value_finished.resize(2);
    ptr_object_now->is_grp_set_max_value_finished.resize(2);

    for(int i=0; i<2; i++)
    {
        ptr_object_now->grp_initial_min_position[i] = 0;
        ptr_object_now->grp_initial_max_position[i] = 1;
        ptr_object_now->is_grp_set_min_value_finished[i] = false;
        ptr_object_now->is_grp_set_max_value_finished[i] = false;
    }

    uint16_t gripper_close_length = 0;
    taskScanningParameter ui_cmd_parameter;

    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    ///// Default///////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////

    ui.radioButton_assemblyUI_isMaskFixed->setChecked(false);
    ui.lineEdit_pixelList_1->setText("0");
    ui.lineEdit_pixelList_2->setText("0");
    ui.lineEdit_pixelList_3->setText("1920");
    ui.lineEdit_pixelList_4->setText("1200");
    ui.task_spinBox_gripperPosValue->setValue(7500); // tip ver. 3
    gripper_close_length = 8250;

    qnode_->grp_do_initialize_ = true;
    qnode_->grp_is_vacuum_ = false;
    qnode_->grp_is_vacuum_before_ = false;
    qnode_->grp_init_2_grp_cmd_ = 4953; // for initialize_2

    //// Tip index setting
    ptr_object_now->m_grasping_parameter.gripper_tip_index = 1; // default index: 1
    ptr_object_now->m_grasping_parameter.is_tip_changing_applied = false;

    //// 파지 성공/실패 여부 확인을 위한 변수
    //// TODO: JSON 파일로 설정하기
    // Gripper #1
    ptr_object_now->grp_initial_min_position[0] = -4735; // [deg]
    ptr_object_now->grp_initial_max_position[0] = 44; // [deg]
    ptr_object_now->is_grp_set_min_value_finished[0] = true;
    ptr_object_now->is_grp_set_max_value_finished[0] = true;
    // Gripper #2
    ptr_object_now->grp_initial_min_position[1] = -4557; // [deg]
    ptr_object_now->grp_initial_max_position[1] = 187; // [deg]
    ptr_object_now->is_grp_set_min_value_finished[1] = true;
    ptr_object_now->is_grp_set_max_value_finished[1] = true;

    ////
    // Target mrcnn weight number
    if(ui.radioButton_use_automatic_mrcnn_weight_1->isChecked()) ui.lineEdit_targetWeightNumber->setText("udr");
    else if(ui.radioButton_use_automatic_mrcnn_weight_2->isChecked()) ui.lineEdit_targetWeightNumber->setText("udrv");
    else ui.lineEdit_targetWeightNumber->setText("29");
    //// Tool changing slave id
    ptr_object_now->tool_changing_attach_id = 2; // # slave #1, 2, ...
    ptr_object_now->tool_changing_detach_id = 2; // # slave #1, 2, ...
    qnode_->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
    qnode_->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

    ui.task_spinBox_toolIndex->setValue(qnode_->tool_changing_attach_id); // tool slot #1, 2, ...

    //// Tip changing id
    ptr_object_now->is_tip_changing_allowed_ = true;
    ptr_object_now->tip_changing_attach_id = 2; // tip #1, 2, ...
    ptr_object_now->tip_changing_detach_id = 2; // tip #1, 2, ...
    qnode_->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
    qnode_->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

    ui.task_spinBox_tipIndex->setValue(qnode_->tip_changing_attach_id); // tool slot #1, 2, ...

    //// Detaching count initialize
    //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
    ptr_object_now->detaching_cnt_ = 0;
    ptr_object_now->stacking_z_idx_ = 0;
    qnode_->test_cnt_ = 0;

    std::vector<double> stacking_trans_scale = {0.225, 0.050, 0.045}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
    ptr_object_now->stacking_single_stack_num_ = 4; // 1개씩 놓는 진행 방향의 물체 개수
    ptr_object_now->stacking_line_stack_num_ = 1; // stacking_single_stack_num_만큼 놓는 선의 개수
    ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
    ptr_object_now->stacking_mode_ = StackingType::CYLINDER_STACKING;

    //// 1단의 실린더 개수
    ptr_object_now->stack_part_cnt_ = ((ptr_object_now->stacking_single_stack_num_ * (ptr_object_now->stacking_single_stack_num_ + 1)) / 2) * ptr_object_now->stacking_line_stack_num_;
    //// counting 최대 개수
    ptr_object_now->max_cnt_detaching_ = ptr_object_now->stack_part_cnt_; // 한 단의 실린더 스택, ex) 한 개의 단은 10개씩, 두 단은 총 20개
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////


    switch(target_object_idx)
    {
        case TargetObject::OBJECT_SQUARE_PEG :
        {
            ROS_LOG_WARN("Target grasping object: Square peg!");
            taskTargetParts_[0]->setText(QString::number(ON_PART_SQUARE_PEG));
            taskTargetParts_[1]->setText("square_peg");
            ui.radioButton_assemblyUI_isMaskFixed->setChecked(false);
            ui.lineEdit_pixelList_1->setText("800");
            ui.lineEdit_pixelList_2->setText("180");
            ui.lineEdit_pixelList_3->setText("250");
            ui.lineEdit_pixelList_4->setText("920");
            ui.task_spinBox_gripperPosValue->setValue(0);
            gripper_close_length = 9000;

            qnode_->grp_do_initialize_ = false;
            qnode_->grp_is_vacuum_ = true;
            qnode_->grp_is_vacuum_before_ = true;
            qnode_->grp_init_2_grp_cmd_ = 4953; // for initialize_2

            //// Tip index setting
            ptr_object_now->m_grasping_parameter.gripper_tip_index = 1; // default index: 1
            ptr_object_now->m_grasping_parameter.is_tip_changing_applied = false;

            //// 파지 성공/실패 여부 확인을 위한 변수
            //// TODO: JSON 파일로 설정하기
            // Gripper #1
            ptr_object_now->grp_initial_min_position[0] = -4735; // [deg]
            ptr_object_now->grp_initial_max_position[0] = 44; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[0] = true;
            ptr_object_now->is_grp_set_max_value_finished[0] = true;
            // Gripper #2
            ptr_object_now->grp_initial_min_position[1] = -4557; // [deg]
            ptr_object_now->grp_initial_max_position[1] = 187; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[1] = true;
            ptr_object_now->is_grp_set_max_value_finished[1] = true;
            ////

            // Target mrcnn weight number
            ui.lineEdit_targetWeightNumber->setText("ud");
            ui.lineEdit_targetName->setText("square_peg");
            //// load mrcnn weight
            ui_cmd_parameter.target_name = ui.lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = ui.lineEdit_targetWeightNumber->text().toStdString();
            ui_cmd_parameter.is_sam_mean_size_assigned = true; // SAM paramter
            ui_cmd_parameter.sam_mean_size = 35736; // SAM paramter
            ui_cmd_parameter.sam_mask_min_area = 29000;
            ui_cmd_parameter.sam_mask_max_area = 43000;
            // qnode_->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            if(qnode_->is_tcp_initialized) changeRobotTCP(11); // TCP #11, square peg vacuum
            ptr_object_now->tcp_changing_id = 11;

	        ui.task_doubleSpinBox_segParam__colorRGS_neighborR->setValue(1.0);
	        ui.task_doubleSpinBox_segParam_colorRGS_thre1stColor->setValue(20.0);
	        ui.task_doubleSpinBox_segParam_colorRGS_thre2ndColor->setValue(8.0);
	        ui.task_spinBox_segParam_colorRGS_minClusterSize->setValue(20000);

            //// Tool changing slave id
            // qnode_->is_tool_attached = false;
            ptr_object_now->tool_changing_attach_id = 1; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 1; // # slave #1, 2, ...

            qnode_->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            qnode_->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_toolIndex->setValue(qnode_->tool_changing_attach_id); // tool slot #1, 2, ...


            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = false;
            ptr_object_now->tip_changing_attach_id = 1; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 1; // tip #1, 2, ...
            qnode_->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            qnode_->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_tipIndex->setValue(qnode_->tip_changing_attach_id); // tool slot #1, 2, ...


            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            qnode_->test_cnt_ = 0;

            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = 2;

            std::vector<double> stacking_trans_scale = {0.0, 0.14, 0.082}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	ptr_object_now->stacking_single_stack_num_ = 2; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 1; // stacking_single_stack_num_만큼 놓는 선의 개수
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::GRID_STACKING;

            qnode_->m_bin_picking_node->is_task_in_progress = false;

            //// Scan sampling setting
            ui.radioButton_assemblyUI_scanSampling->setChecked(true);
            ui.lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObject::OBJECT_WELDING_ELBOW_JOINT :
        {
            ROS_LOG_WARN("Target grasping object: Welding elbow joint!");
            taskTargetParts_[0]->setText(QString::number(ON_PART_WELDING_ELBOW_JOINT));
            taskTargetParts_[1]->setText("eb_joint");
            ui.radioButton_assemblyUI_isMaskFixed->setChecked(false);
            ui.lineEdit_pixelList_1->setText("0");
            ui.lineEdit_pixelList_2->setText("0");
            ui.lineEdit_pixelList_3->setText("1920");
            ui.lineEdit_pixelList_4->setText("1200");
            // ui.task_spinBox_gripperPosValue->setValue(8250); // 60mm
            ui.task_spinBox_gripperPosValue->setValue(7000); // 60mm
            gripper_close_length = 8000;

            qnode_->grp_do_initialize_ = true;
            qnode_->grp_is_vacuum_ = false;
            qnode_->grp_is_vacuum_before_ = false;
            qnode_->grp_init_2_grp_cmd_ = 4953; // for initialize_2

            //// Tip index setting
            ptr_object_now->m_grasping_parameter.gripper_tip_index = 1; // default index: 1
            ptr_object_now->m_grasping_parameter.is_tip_changing_applied = false;

            //// 파지 성공/실패 여부 확인을 위한 변수
            //// TODO: JSON 파일로 설정하기
            // Gripper #1
            ptr_object_now->grp_initial_min_position[0] = -4735; // [deg]
            ptr_object_now->grp_initial_max_position[0] = 44; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[0] = true;
            ptr_object_now->is_grp_set_max_value_finished[0] = true;
            // Gripper #2
            ptr_object_now->grp_initial_min_position[1] = -4557; // [deg]
            ptr_object_now->grp_initial_max_position[1] = 187; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[1] = true;
            ptr_object_now->is_grp_set_max_value_finished[1] = true;
            ////

            // Target mrcnn weight number
            if(ui.radioButton_use_automatic_mrcnn_weight_1->isChecked()) ui.lineEdit_targetWeightNumber->setText("udr");
            else if(ui.radioButton_use_automatic_mrcnn_weight_2->isChecked()) ui.lineEdit_targetWeightNumber->setText("udrv");
            else ui.lineEdit_targetWeightNumber->setText("29");
            ui.lineEdit_targetName->setText("eb_joint");
            //// load mrcnn weight
            ui_cmd_parameter.target_name = ui.lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = ui.lineEdit_targetWeightNumber->text().toStdString();
            ui_cmd_parameter.is_sam_mean_size_assigned = false; // SAM paramter
            ui_cmd_parameter.sam_mean_size = 0; // SAM paramter
            ui_cmd_parameter.sam_mask_min_area = 6000;
            ui_cmd_parameter.sam_mask_max_area = 300000;
            // qnode_->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            if(qnode_->is_tcp_initialized) changeRobotTCP(2); // TCP #2
            ptr_object_now->tcp_changing_id = 2;

            //// Tool changing slave id
            // qnode_->is_tool_attached = false;
            ptr_object_now->tool_changing_attach_id = 2; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 2; // # slave #1, 2, ...

            qnode_->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            qnode_->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_toolIndex->setValue(qnode_->tool_changing_attach_id); // tool slot #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = false;
            ptr_object_now->tip_changing_attach_id = 1; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 1; // tip #1, 2, ...
            qnode_->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            qnode_->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_tipIndex->setValue(qnode_->tip_changing_attach_id); // tool slot #1, 2, ...

            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            qnode_->test_cnt_ = 0;

            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = 9;

            std::vector<double> stacking_trans_scale = {0.14, 0.12, 0.082}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	ptr_object_now->stacking_single_stack_num_ = 2; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 3; // stacking_single_stack_num_만큼 놓는 선의 개수
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::GRID_STACKING;

            qnode_->m_bin_picking_node->is_task_in_progress = false;

            //// Scan sampling setting
            ui.radioButton_assemblyUI_scanSampling->setChecked(true);
            ui.lineEdit_scanSamplingNum->setText("8");

        }
        break;

        case TargetObject::OBJECT_WELDING_T_JOINT_GRP_2F :
        {
            ROS_LOG_WARN("Target grasping object: Welding T joint (2F Gripper)!");
            taskTargetParts_[0]->setText(QString::number(ON_PART_WELDING_T_JOINT_2F));
            taskTargetParts_[1]->setText("t_joint");
            ui.radioButton_assemblyUI_isMaskFixed->setChecked(false);
            ui.lineEdit_pixelList_1->setText("0");
            ui.lineEdit_pixelList_2->setText("0");
            ui.lineEdit_pixelList_3->setText("1920");
            ui.lineEdit_pixelList_4->setText("1200");
            ui.task_spinBox_gripperPosValue->setValue(0); // 0, 4000, 7000
            gripper_close_length = 1000;

            qnode_->grp_do_initialize_ = true;
            qnode_->grp_is_vacuum_ = false;
            qnode_->grp_is_vacuum_before_ = false;
            qnode_->grp_init_2_grp_cmd_ = 4953; // for initialize_2

            //// Tip index setting
            ptr_object_now->m_grasping_parameter.gripper_tip_index = 1; // default index: 1
            ptr_object_now->m_grasping_parameter.is_tip_changing_applied = false;

            //// 파지 성공/실패 여부 확인을 위한 변수
            //// TODO: JSON 파일로 설정하기
            // Gripper #1
            ptr_object_now->grp_initial_min_position[0] = -4735; // [deg]
            ptr_object_now->grp_initial_max_position[0] = 44; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[0] = true;
            ptr_object_now->is_grp_set_max_value_finished[0] = true;
            // Gripper #2
            ptr_object_now->grp_initial_min_position[1] = -4557; // [deg]
            ptr_object_now->grp_initial_max_position[1] = 187; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[1] = true;
            ptr_object_now->is_grp_set_max_value_finished[1] = true;
            ////

            // Target mrcnn weight number
            if(ui.radioButton_use_automatic_mrcnn_weight_1->isChecked()) ui.lineEdit_targetWeightNumber->setText("udr");
            else if(ui.radioButton_use_automatic_mrcnn_weight_2->isChecked()) ui.lineEdit_targetWeightNumber->setText("udrv");
            else ui.lineEdit_targetWeightNumber->setText("29");
            ui.lineEdit_targetName->setText("t_joint");
            //// load mrcnn weight
            ui_cmd_parameter.target_name = ui.lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = ui.lineEdit_targetWeightNumber->text().toStdString();
            ui_cmd_parameter.is_sam_mean_size_assigned = false; // SAM paramter
            ui_cmd_parameter.sam_mean_size = 0; // SAM paramter
            ui_cmd_parameter.sam_mask_min_area = 6000;
            ui_cmd_parameter.sam_mask_max_area = 300000;
            // qnode_->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            //// 2F gripper - before
            if(qnode_->is_tcp_initialized) changeRobotTCP(3); // TCP #3
            ptr_object_now->tcp_changing_id = 3;

            // 2F gripper - new short version
            // if(qnode_->is_tcp_initialized) changeRobotTCP(17); // TCP #17
            // ptr_object_now->tcp_changing_id = 17;


            //// Vacuum 1cup
            // if(qnode_->is_tcp_initialized) changeRobotTCP(15); // TCP #15

            //// Tool changing slave id
            // qnode_->is_tool_attached = false;
            ptr_object_now->tool_changing_attach_id = 2; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 2; // # slave #1, 2, ...

            qnode_->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            qnode_->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_toolIndex->setValue(qnode_->tool_changing_attach_id); // tool slot #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = true;
            ptr_object_now->tip_changing_attach_id = 1; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 1; // tip #1, 2, ...
            qnode_->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            qnode_->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_tipIndex->setValue(qnode_->tip_changing_attach_id); // tool slot #1, 2, ...

            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            qnode_->test_cnt_ = 0;

            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = 2;

            std::vector<double> stacking_trans_scale = {0.0, 0.125, 0.082}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	ptr_object_now->stacking_single_stack_num_ = 2; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 1; // stacking_single_stack_num_만큼 놓는 선의 개수
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::GRID_STACKING;

            qnode_->m_bin_picking_node->is_task_in_progress = false;

            //// Scan sampling setting
            ui.radioButton_assemblyUI_scanSampling->setChecked(true);
            ui.lineEdit_scanSamplingNum->setText("8");

        }
        break;

        case TargetObject::OBJECT_BOLT_BUSH :
        {
            ROS_LOG_WARN("Target grasping object: Bolt bush!");
            taskTargetParts_[0]->setText(QString::number(ON_PART_BOLT_BUSH));
            taskTargetParts_[1]->setText("bolt_bush");
            ui.radioButton_assemblyUI_isMaskFixed->setChecked(false);
            ui.lineEdit_pixelList_1->setText("0");
            ui.lineEdit_pixelList_2->setText("0");
            ui.lineEdit_pixelList_3->setText("1920");
            ui.lineEdit_pixelList_4->setText("1200");
            ui.task_spinBox_gripperPosValue->setValue(5000); // inner plane grasping - grasping length
            gripper_close_length = 5000;

            qnode_->grp_do_initialize_ = true;
            qnode_->grp_is_vacuum_ = false;
            qnode_->grp_is_vacuum_before_ = false;
            qnode_->grp_init_2_grp_cmd_ = 1323; // for initialize_2

            //// Tip index setting
            ptr_object_now->m_grasping_parameter.gripper_tip_index = 1; // default index: 1
            ptr_object_now->m_grasping_parameter.is_tip_changing_applied = false;

            //// 파지 성공/실패 여부 확인을 위한 변수
            //// TODO: JSON 파일로 설정하기
            // Gripper #1
            ptr_object_now->grp_initial_min_position[0] = -4735; // [deg]
            ptr_object_now->grp_initial_max_position[0] = 44; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[0] = true;
            ptr_object_now->is_grp_set_max_value_finished[0] = true;
            // Gripper #2
            ptr_object_now->grp_initial_min_position[1] = -4557; // [deg]
            ptr_object_now->grp_initial_max_position[1] = 187; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[1] = true;
            ptr_object_now->is_grp_set_max_value_finished[1] = true;
            ////

            // Target mrcnn weight number
            if(ui.radioButton_use_automatic_mrcnn_weight_1->isChecked()) ui.lineEdit_targetWeightNumber->setText("udr");
            else if(ui.radioButton_use_automatic_mrcnn_weight_2->isChecked()) ui.lineEdit_targetWeightNumber->setText("udrv");
            else ui.lineEdit_targetWeightNumber->setText("29");
            ui.lineEdit_targetName->setText("bolt_bush");
            //// load mrcnn weight
            ui_cmd_parameter.target_name = ui.lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = ui.lineEdit_targetWeightNumber->text().toStdString();
            ui_cmd_parameter.is_sam_mean_size_assigned = true; // SAM paramter
            ui_cmd_parameter.sam_mean_size = 26000; // SAM paramter
            ui_cmd_parameter.sam_mask_min_area = 15000;
            ui_cmd_parameter.sam_mask_max_area = 38000;
            // qnode_->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            if(qnode_->is_tcp_initialized) changeRobotTCP(4); // TCP #4, 평행그리퍼
            ptr_object_now->tcp_changing_id = 4;

            //// Tool changing slave id
            // qnode_->is_tool_attached = false;
            ptr_object_now->tool_changing_attach_id = 3; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 3; // # slave #1, 2, ...

            qnode_->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            qnode_->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_toolIndex->setValue(qnode_->tool_changing_attach_id); // tool slot #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = false;
            ptr_object_now->tip_changing_attach_id = 1; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 1; // tip #1, 2, ...
            qnode_->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            qnode_->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_tipIndex->setValue(qnode_->tip_changing_attach_id); // tool slot #1, 2, ...


            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            qnode_->test_cnt_ = 0;

            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = 3;

            std::vector<double> stacking_trans_scale = {0.0, 0.100, 0.02}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	ptr_object_now->stacking_single_stack_num_ = 3; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 1; // stacking_single_stack_num_만큼 놓는 선의 개수
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::GRID_STACKING;

            qnode_->m_bin_picking_node->is_task_in_progress = false;

            //// Scan sampling setting
            ui.radioButton_assemblyUI_scanSampling->setChecked(true);
            ui.lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObject::OBJECT_CYLINDER :
        {
            ROS_LOG_WARN("Target grasping object: Cylinder!");
            taskTargetParts_[0]->setText(QString::number(ON_PART_ROTATION_AXIS_ONE_SIDED));
            taskTargetParts_[1]->setText("peg");
            ui.radioButton_assemblyUI_isMaskFixed->setChecked(false);
            ui.lineEdit_pixelList_1->setText("0");
            ui.lineEdit_pixelList_2->setText("0");
            ui.lineEdit_pixelList_3->setText("1920");
            ui.lineEdit_pixelList_4->setText("1200");
            ui.task_spinBox_gripperPosValue->setValue(7500); // tip ver. 3
            gripper_close_length = 8250;

            qnode_->grp_do_initialize_ = true;
            qnode_->grp_is_vacuum_ = false;
            qnode_->grp_is_vacuum_before_ = false;
            qnode_->grp_init_2_grp_cmd_ = 4953; // for initialize_2

            //// Tip index setting
            ptr_object_now->m_grasping_parameter.gripper_tip_index = 1; // default index: 1
            ptr_object_now->m_grasping_parameter.is_tip_changing_applied = false;

            //// 파지 성공/실패 여부 확인을 위한 변수
            //// TODO: JSON 파일로 설정하기
            // Gripper #1
            ptr_object_now->grp_initial_min_position[0] = -4735; // [deg]
            ptr_object_now->grp_initial_max_position[0] = 44; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[0] = true;
            ptr_object_now->is_grp_set_max_value_finished[0] = true;
            // Gripper #2
            ptr_object_now->grp_initial_min_position[1] = -4557; // [deg]
            ptr_object_now->grp_initial_max_position[1] = 187; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[1] = true;
            ptr_object_now->is_grp_set_max_value_finished[1] = true;

            ////
            // Target mrcnn weight number
            if(ui.radioButton_use_automatic_mrcnn_weight_1->isChecked()) ui.lineEdit_targetWeightNumber->setText("udr");
            else if(ui.radioButton_use_automatic_mrcnn_weight_2->isChecked()) ui.lineEdit_targetWeightNumber->setText("udrv");
            else ui.lineEdit_targetWeightNumber->setText("29");

            ui.lineEdit_targetName->setText("peg");
            //// load mrcnn weight
            ui_cmd_parameter.target_name = ui.lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = ui.lineEdit_targetWeightNumber->text().toStdString();
            ui_cmd_parameter.is_sam_mean_size_assigned = false; // SAM paramter
            ui_cmd_parameter.sam_mean_size = 0; // SAM paramter
            ui_cmd_parameter.sam_mask_min_area = 6000;
            ui_cmd_parameter.sam_mask_max_area = 300000;

            // qnode_->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            //// [2F 그리퍼] - before
            // if(qnode_->is_tcp_initialized) changeRobotTCP(13); // TCP #13(tip changing - round tip)
            // ptr_object_now->tcp_changing_id = 13;

            // //// [2F 그리퍼] - new short version
            // if(qnode_->is_tcp_initialized) changeRobotTCP(18); // TCP #18(tip changing - round tip)
            // ptr_object_now->tcp_changing_id = 18;

            //// 드럼통 과제
            if(qnode_->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;


            //// Tool changing slave id
            ptr_object_now->tool_changing_attach_id = 2; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 2; // # slave #1, 2, ...

            qnode_->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            qnode_->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_toolIndex->setValue(qnode_->tool_changing_attach_id); // tool slot #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = true;
            ptr_object_now->tip_changing_attach_id = 2; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 2; // tip #1, 2, ...
            qnode_->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            qnode_->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_tipIndex->setValue(qnode_->tip_changing_attach_id); // tool slot #1, 2, ...

            //// [흡착 그리퍼]
            // if(qnode_->is_tcp_initialized) changeRobotTCP(16); // TCP #11, square peg vacuum
            // //// Tool changing slave id
            // qnode_->tool_changing_attach_id = 4; // # slave #1, 2, ...
            // qnode_->tool_changing_detach_id = 4; // # slave #1, 2, ...


            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            qnode_->test_cnt_ = 0;

            std::vector<double> stacking_trans_scale = {0.225, 0.050, 0.045}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	ptr_object_now->stacking_single_stack_num_ = 4; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 1; // stacking_single_stack_num_만큼 놓는 선의 개수
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::CYLINDER_STACKING;

            //// 1단의 실린더 개수
            ptr_object_now->stack_part_cnt_ = ((ptr_object_now->stacking_single_stack_num_ * (ptr_object_now->stacking_single_stack_num_ + 1)) / 2) * ptr_object_now->stacking_line_stack_num_;
            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = ptr_object_now->stack_part_cnt_; // 한 단의 실린더 스택, ex) 한 개의 단은 10개씩, 두 단은 총 20개


            qnode_->m_bin_picking_node->is_task_in_progress = false;

            //// Scan sampling setting
            ui.radioButton_assemblyUI_scanSampling->setChecked(true);
            ui.lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObject::OBJECT_STRIKER :
        {
            ROS_LOG_WARN("OBJECT_STRIKER!");
            taskTargetParts_[0]->setText(QString::number(ON_PART_STRIKER));
            taskTargetParts_[1]->setText("striker");
            ui.radioButton_assemblyUI_isMaskFixed->setChecked(false);
            ui.lineEdit_pixelList_1->setText("0");
            ui.lineEdit_pixelList_2->setText("0");
            ui.lineEdit_pixelList_3->setText("1920");
            ui.lineEdit_pixelList_4->setText("1200");
            ui.task_spinBox_gripperPosValue->setValue(7500); // tip ver. 3
            gripper_close_length = 8250;

            qnode_->grp_do_initialize_ = false;
            qnode_->grp_is_vacuum_ = false;
            qnode_->grp_is_vacuum_before_ = false;
            qnode_->grp_init_2_grp_cmd_ = 4953; // for initialize_2

            //// Tip index setting
            ptr_object_now->m_grasping_parameter.gripper_tip_index = 1; // default index: 1
            ptr_object_now->m_grasping_parameter.is_tip_changing_applied = false;

            //// 파지 성공/실패 여부 확인을 위한 변수
            //// TODO: JSON 파일로 설정하기
            // Gripper #1
            ptr_object_now->grp_initial_min_position[0] = -4735; // [deg]
            ptr_object_now->grp_initial_max_position[0] = 44; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[0] = true;
            ptr_object_now->is_grp_set_max_value_finished[0] = true;
            // Gripper #2
            ptr_object_now->grp_initial_min_position[1] = -4557; // [deg]
            ptr_object_now->grp_initial_max_position[1] = 187; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[1] = true;
            ptr_object_now->is_grp_set_max_value_finished[1] = true;

            ////
            // Target mrcnn weight number
            if(ui.radioButton_use_automatic_mrcnn_weight_1->isChecked()) ui.lineEdit_targetWeightNumber->setText("udr");
            else if(ui.radioButton_use_automatic_mrcnn_weight_2->isChecked()) ui.lineEdit_targetWeightNumber->setText("udrv");
            else ui.lineEdit_targetWeightNumber->setText("29");

            ui.lineEdit_targetName->setText("striker");
            //// load mrcnn weight
            ui_cmd_parameter.target_name = ui.lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = ui.lineEdit_targetWeightNumber->text().toStdString();
            ui_cmd_parameter.is_sam_mean_size_assigned = false; // SAM paramter
            ui_cmd_parameter.sam_mean_size = 0; // SAM paramter
            ui_cmd_parameter.sam_mask_min_area = 6000;
            ui_cmd_parameter.sam_mask_max_area = 300000;

            // qnode_->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            if(qnode_->is_tcp_initialized) changeRobotTCP(11); // TCP #11
            ptr_object_now->tcp_changing_id = 11;

            //// Tool changing slave id
            // qnode_->is_tool_attached = false;

            ptr_object_now->tool_changing_attach_id = 4; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 4; // # slave #1, 2, ...

            qnode_->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            qnode_->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_toolIndex->setValue(qnode_->tool_changing_attach_id); // tool slot #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = false;
            ptr_object_now->tip_changing_attach_id = 1; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 1; // tip #1, 2, ...
            qnode_->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            qnode_->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_tipIndex->setValue(qnode_->tip_changing_attach_id); // tool slot #1, 2, ...

            //// Detaching count initialize
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;

            std::vector<double> stacking_trans_scale = {0.225, 0.050, 0.045}; // row, col, z
        	ptr_object_now->stacking_single_stack_num_ = 4; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 1; // 열 개수 (cylinder stacking의 경우 무조건 1)
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::CYLINDER_STACKING;

            ptr_object_now->stack_part_cnt_ = ((ptr_object_now->stacking_single_stack_num_ * (ptr_object_now->stacking_single_stack_num_ + 1)) / 2) * ptr_object_now->stacking_line_stack_num_;
            ptr_object_now->max_cnt_detaching_ = 2*ptr_object_now->stack_part_cnt_; // 두 단의 실린더 스택, ex) 한 개의 단은 10개씩, 두 단은 총 20개
            qnode_->m_bin_picking_node->is_task_in_progress = false;

            //// Scan sampling setting
            ui.radioButton_assemblyUI_scanSampling->setChecked(false);
            ui.lineEdit_scanSamplingNum->setText("4");
        }
        break;

        case TargetObject::OBJECT_SCU :
        {
            ROS_LOG_WARN("Target grasping object: OBJECT_SCU!");
            taskTargetParts_[0]->setText(QString::number(ON_PART_SCU));
            taskTargetParts_[1]->setText("scu");
            ui.radioButton_assemblyUI_isMaskFixed->setChecked(false);
            ui.lineEdit_pixelList_1->setText("0");
            ui.lineEdit_pixelList_2->setText("0");
            ui.lineEdit_pixelList_3->setText("1920");
            ui.lineEdit_pixelList_4->setText("1200");
            ui.task_spinBox_gripperPosValue->setValue(9500); // tip ver. 3
            gripper_close_length = 8250;

            qnode_->grp_do_initialize_ = true;
            qnode_->grp_is_vacuum_ = false;
            qnode_->grp_is_vacuum_before_ = false;
            qnode_->grp_init_2_grp_cmd_ = 4953; // for initialize_2

            //// Tip index setting
            ptr_object_now->m_grasping_parameter.gripper_tip_index = 1; // default index: 1
            ptr_object_now->m_grasping_parameter.is_tip_changing_applied = false;

            //// 파지 성공/실패 여부 확인을 위한 변수
            //// TODO: JSON 파일로 설정하기
            // Gripper #1
            ptr_object_now->grp_initial_min_position[0] = -4735; // [deg]
            ptr_object_now->grp_initial_max_position[0] = 44; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[0] = true;
            ptr_object_now->is_grp_set_max_value_finished[0] = true;
            // Gripper #2
            ptr_object_now->grp_initial_min_position[1] = -4557; // [deg]
            ptr_object_now->grp_initial_max_position[1] = 187; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[1] = true;
            ptr_object_now->is_grp_set_max_value_finished[1] = true;

            ////
            // Target mrcnn weight number
            if(ui.radioButton_use_automatic_mrcnn_weight_1->isChecked()) ui.lineEdit_targetWeightNumber->setText("udr");
            else if(ui.radioButton_use_automatic_mrcnn_weight_2->isChecked()) ui.lineEdit_targetWeightNumber->setText("udrv");
            else ui.lineEdit_targetWeightNumber->setText("29");

            ui.lineEdit_targetName->setText("scu");
            //// load mrcnn weight
            ui_cmd_parameter.target_name = ui.lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = ui.lineEdit_targetWeightNumber->text().toStdString();
            ui_cmd_parameter.is_sam_mean_size_assigned = false; // SAM paramter
            ui_cmd_parameter.sam_mean_size = 0; // SAM paramter
            ui_cmd_parameter.sam_mask_min_area = 6000;
            ui_cmd_parameter.sam_mask_max_area = 300000;

            // qnode_->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            //// [2F 그리퍼] - before
            if(qnode_->is_tcp_initialized) changeRobotTCP(13); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 13;

            // //// [2F 그리퍼] - new short version
            // if(qnode_->is_tcp_initialized) changeRobotTCP(18); // TCP #18(tip changing - round tip)
            // ptr_object_now->tcp_changing_id = 18;


            //// Tool changing slave id
            ptr_object_now->tool_changing_attach_id = 2; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 2; // # slave #1, 2, ...

            qnode_->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            qnode_->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_toolIndex->setValue(qnode_->tool_changing_attach_id); // tool slot #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = true;
            ptr_object_now->tip_changing_attach_id = 2; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 2; // tip #1, 2, ...
            qnode_->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            qnode_->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_tipIndex->setValue(qnode_->tip_changing_attach_id); // tool slot #1, 2, ...

            //// [흡착 그리퍼]
            // if(qnode_->is_tcp_initialized) changeRobotTCP(16); // TCP #11, square peg vacuum
            // //// Tool changing slave id
            // qnode_->tool_changing_attach_id = 4; // # slave #1, 2, ...
            // qnode_->tool_changing_detach_id = 4; // # slave #1, 2, ...


            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            qnode_->test_cnt_ = 0;

            std::vector<double> stacking_trans_scale = {0.225, 0.050, 0.045}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	ptr_object_now->stacking_single_stack_num_ = 4; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 1; // stacking_single_stack_num_만큼 놓는 선의 개수
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::CYLINDER_STACKING;

            //// 1단의 실린더 개수
            ptr_object_now->stack_part_cnt_ = ((ptr_object_now->stacking_single_stack_num_ * (ptr_object_now->stacking_single_stack_num_ + 1)) / 2) * ptr_object_now->stacking_line_stack_num_;
            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = ptr_object_now->stack_part_cnt_; // 한 단의 실린더 스택, ex) 한 개의 단은 10개씩, 두 단은 총 20개


            qnode_->m_bin_picking_node->is_task_in_progress = false;

            //// Scan sampling setting
            ui.radioButton_assemblyUI_scanSampling->setChecked(true);
            ui.lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObject::OBJECT_GRILL :
        {
            ROS_LOG_WARN("Target grasping object: Grill!");
            taskTargetParts_[0]->setText(QString::number(ON_PART_GRILL));
            taskTargetParts_[1]->setText("grill");
            ui.radioButton_assemblyUI_isMaskFixed->setChecked(false);
            ui.lineEdit_pixelList_1->setText("0");
            ui.lineEdit_pixelList_2->setText("0");
            ui.lineEdit_pixelList_3->setText("1920");
            ui.lineEdit_pixelList_4->setText("1200");
            ui.task_spinBox_gripperPosValue->setValue(6000);
            gripper_close_length = 7000;

            qnode_->grp_do_initialize_ = true;
            qnode_->grp_is_vacuum_ = false;
            qnode_->grp_is_vacuum_before_ = false;
            qnode_->grp_init_2_grp_cmd_ = 4953; // for initialize_2

            //// Tip index setting
            ptr_object_now->m_grasping_parameter.gripper_tip_index = 1; // default index: 1
            ptr_object_now->m_grasping_parameter.is_tip_changing_applied = false;

            //// 파지 성공/실패 여부 확인을 위한 변수
            //// TODO: JSON 파일로 설정하기
            // Gripper #1
            ptr_object_now->grp_initial_min_position[0] = -4735; // [deg]
            ptr_object_now->grp_initial_max_position[0] = 44; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[0] = true;
            ptr_object_now->is_grp_set_max_value_finished[0] = true;
            // Gripper #2
            ptr_object_now->grp_initial_min_position[1] = -4557; // [deg]
            ptr_object_now->grp_initial_max_position[1] = 187; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[1] = true;
            ptr_object_now->is_grp_set_max_value_finished[1] = true;
            ////

            // Target mrcnn weight number
            if(ui.radioButton_use_automatic_mrcnn_weight_1->isChecked()) ui.lineEdit_targetWeightNumber->setText("udr");
            else if(ui.radioButton_use_automatic_mrcnn_weight_2->isChecked()) ui.lineEdit_targetWeightNumber->setText("udrv");
            else ui.lineEdit_targetWeightNumber->setText("39");
            ui.lineEdit_targetName->setText("grill");
            //// load mrcnn weight
            ui_cmd_parameter.target_name = ui.lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = ui.lineEdit_targetWeightNumber->text().toStdString();
            ui_cmd_parameter.is_sam_mean_size_assigned = false; // SAM paramter
            ui_cmd_parameter.sam_mean_size = 0; // SAM paramter
            ui_cmd_parameter.sam_mask_min_area = 6000;
            ui_cmd_parameter.sam_mask_max_area = 300000;
            // qnode_->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            if(qnode_->is_tcp_initialized) changeRobotTCP(1); // TCP #1
            ptr_object_now->tcp_changing_id = 1;

            //// Tool changing slave id
            // qnode_->is_tool_attached = false;
            ptr_object_now->tool_changing_attach_id = 2; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 2; // # slave #1, 2, ...

            qnode_->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            qnode_->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...


            ui.task_spinBox_toolIndex->setValue(qnode_->tool_changing_attach_id); // tool slot #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = false;
            ptr_object_now->tip_changing_attach_id = 1; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 1; // tip #1, 2, ...
            qnode_->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            qnode_->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_tipIndex->setValue(qnode_->tip_changing_attach_id); // tool slot #1, 2, ...


            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            qnode_->test_cnt_ = 0;

            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = 9;

            std::vector<double> stacking_trans_scale = {0.14, 0.12, 0.082}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	ptr_object_now->stacking_single_stack_num_ = 2; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 3; // stacking_single_stack_num_만큼 놓는 선의 개수
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::GRID_STACKING;

            qnode_->m_bin_picking_node->is_task_in_progress = false;

            //// Scan sampling setting
            ui.radioButton_assemblyUI_scanSampling->setChecked(true);
            ui.lineEdit_scanSamplingNum->setText("4");
        }
        break;

        case TargetObject::OBJECT_BOLT :
        {
            ROS_LOG_WARN("Target grasping object: Bolt!");
            taskTargetParts_[0]->setText(QString::number(ON_PART_BOLT));
            taskTargetParts_[1]->setText("bolt");
            ui.radioButton_assemblyUI_isMaskFixed->setChecked(false);
            ui.lineEdit_pixelList_1->setText("0");
            ui.lineEdit_pixelList_2->setText("0");
            ui.lineEdit_pixelList_3->setText("1920");
            ui.lineEdit_pixelList_4->setText("1200");
            ui.task_spinBox_gripperPosValue->setValue(0); // 0, 4000, 7000
            gripper_close_length = 1000;

            qnode_->grp_do_initialize_ = false;
            qnode_->grp_is_vacuum_ = true;
            qnode_->grp_is_vacuum_before_ = true;
            qnode_->grp_init_2_grp_cmd_ = 4953; // for initialize_2

            //// Tip index setting
            ptr_object_now->m_grasping_parameter.gripper_tip_index = 1; // default index: 1
            ptr_object_now->m_grasping_parameter.is_tip_changing_applied = false;

            //// 파지 성공/실패 여부 확인을 위한 변수
            //// TODO: JSON 파일로 설정하기
            // Gripper #1
            ptr_object_now->grp_initial_min_position[0] = -4735; // [deg]
            ptr_object_now->grp_initial_max_position[0] = 44; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[0] = true;
            ptr_object_now->is_grp_set_max_value_finished[0] = true;
            // Gripper #2
            ptr_object_now->grp_initial_min_position[1] = -4557; // [deg]
            ptr_object_now->grp_initial_max_position[1] = 187; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[1] = true;
            ptr_object_now->is_grp_set_max_value_finished[1] = true;
            ////

            // Target mrcnn weight number
            if(ui.radioButton_use_automatic_mrcnn_weight_1->isChecked()) ui.lineEdit_targetWeightNumber->setText("udr");
            else if(ui.radioButton_use_automatic_mrcnn_weight_2->isChecked()) ui.lineEdit_targetWeightNumber->setText("udrv");
            else ui.lineEdit_targetWeightNumber->setText("38");
            ui.lineEdit_targetName->setText("bolt");
            //// load mrcnn weight
            ui_cmd_parameter.target_name = ui.lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = ui.lineEdit_targetWeightNumber->text().toStdString();
            ui_cmd_parameter.is_sam_mean_size_assigned = false; // SAM paramter
            ui_cmd_parameter.sam_mean_size = 0; // SAM paramter
            ui_cmd_parameter.sam_mask_min_area = 6000;
            ui_cmd_parameter.sam_mask_max_area = 300000;
            // qnode_->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            //// 2F gripper
            // if(qnode_->is_tcp_initialized) changeRobotTCP(3); // TCP #3
            // //// 2F gripper - new short version
            // if(qnode_->is_tcp_initialized) changeRobotTCP(17); // TCP #17
            // ptr_object_now->tcp_changing_id = 17;

            //// Vacuum 1cup

            if(qnode_->is_tcp_initialized) changeRobotTCP(12); // TCP #12
            ptr_object_now->tcp_changing_id = 12;

            //// Tool changing slave id
            // qnode_->is_tool_attached = false;

            ptr_object_now->tool_changing_attach_id = 1; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 1; // # slave #1, 2, ...

            qnode_->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            qnode_->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_toolIndex->setValue(qnode_->tool_changing_attach_id); // tool slot #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = false;
            ptr_object_now->tip_changing_attach_id = 1; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 1; // tip #1, 2, ...
            qnode_->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            qnode_->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_tipIndex->setValue(qnode_->tip_changing_attach_id); // tool slot #1, 2, ...

            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            qnode_->test_cnt_ = 0;

            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = 9;

            std::vector<double> stacking_trans_scale = {0.14, 0.12, 0.082}; // row, col, z
        	ptr_object_now->stacking_single_stack_num_ = 2; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 3; // stacking_single_stack_num_만큼 놓는 선의 개수
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::GRID_STACKING;

            qnode_->m_bin_picking_node->is_task_in_progress = false;

            //// Scan sampling setting
            ui.radioButton_assemblyUI_scanSampling->setChecked(true);
            ui.lineEdit_scanSamplingNum->setText("1");

        }
        break;


        case TargetObject::OBJECT_HINGE :
        {
            ROS_LOG_WARN("Target grasping object: OBJECT_HINGE!");
            taskTargetParts_[0]->setText(QString::number(ON_PART_HINGE));
            taskTargetParts_[1]->setText("hinge");
            ui.radioButton_assemblyUI_isMaskFixed->setChecked(false);
            ui.lineEdit_pixelList_1->setText("0");
            ui.lineEdit_pixelList_2->setText("0");
            ui.lineEdit_pixelList_3->setText("1920");
            ui.lineEdit_pixelList_4->setText("1200");
            ui.task_spinBox_gripperPosValue->setValue(9500); // tip ver. 3
            gripper_close_length = 8250;

            qnode_->grp_do_initialize_ = true;
            qnode_->grp_is_vacuum_ = false;
            qnode_->grp_is_vacuum_before_ = false;
            qnode_->grp_init_2_grp_cmd_ = 4953; // for initialize_2

            //// Tip index setting
            ptr_object_now->m_grasping_parameter.gripper_tip_index = 1; // default index: 1
            ptr_object_now->m_grasping_parameter.is_tip_changing_applied = false;

            //// 파지 성공/실패 여부 확인을 위한 변수
            //// TODO: JSON 파일로 설정하기
            // Gripper #1
            ptr_object_now->grp_initial_min_position[0] = -4735; // [deg]
            ptr_object_now->grp_initial_max_position[0] = 44; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[0] = true;
            ptr_object_now->is_grp_set_max_value_finished[0] = true;
            // Gripper #2
            ptr_object_now->grp_initial_min_position[1] = -4557; // [deg]
            ptr_object_now->grp_initial_max_position[1] = 187; // [deg]
            ptr_object_now->is_grp_set_min_value_finished[1] = true;
            ptr_object_now->is_grp_set_max_value_finished[1] = true;

            ////
            // Target mrcnn weight number
            if(ui.radioButton_use_automatic_mrcnn_weight_1->isChecked()) ui.lineEdit_targetWeightNumber->setText("udr");
            else if(ui.radioButton_use_automatic_mrcnn_weight_2->isChecked()) ui.lineEdit_targetWeightNumber->setText("udrv");
            else ui.lineEdit_targetWeightNumber->setText("29");

            ui.lineEdit_targetName->setText("hinge");
            //// load mrcnn weight
            ui_cmd_parameter.target_name = ui.lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = ui.lineEdit_targetWeightNumber->text().toStdString();
            ui_cmd_parameter.is_sam_mean_size_assigned = false; // SAM paramter
            ui_cmd_parameter.sam_mean_size = 0; // SAM paramter
            ui_cmd_parameter.sam_mask_min_area = 6000;
            ui_cmd_parameter.sam_mask_max_area = 300000;

            // qnode_->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            //// [2F 그리퍼] - before
            if(qnode_->is_tcp_initialized) changeRobotTCP(13); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 13;

            // //// [2F 그리퍼] - new short version
            // if(qnode_->is_tcp_initialized) changeRobotTCP(18); // TCP #18(tip changing - round tip)
            // ptr_object_now->tcp_changing_id = 18;


            //// Tool changing slave id
            ptr_object_now->tool_changing_attach_id = 2; // # slave #1, 2, ...
            ptr_object_now->tool_changing_detach_id = 2; // # slave #1, 2, ...

            qnode_->tool_changing_attach_id = ptr_object_now->tool_changing_attach_id; // # slave #1, 2, ...
            qnode_->tool_changing_detach_id = ptr_object_now->tool_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_toolIndex->setValue(qnode_->tool_changing_attach_id); // tool slot #1, 2, ...

            //// Tip changing id
            ptr_object_now->is_tip_changing_allowed_ = true;
            ptr_object_now->tip_changing_attach_id = 2; // tip #1, 2, ...
            ptr_object_now->tip_changing_detach_id = 2; // tip #1, 2, ...
            qnode_->tip_changing_attach_id = ptr_object_now->tip_changing_attach_id; // # slave #1, 2, ...
            qnode_->tip_changing_detach_id = ptr_object_now->tip_changing_detach_id; // # slave #1, 2, ...

            ui.task_spinBox_tipIndex->setValue(qnode_->tip_changing_attach_id); // tool slot #1, 2, ...

            //// [흡착 그리퍼]
            // if(qnode_->is_tcp_initialized) changeRobotTCP(16); // TCP #11, square peg vacuum
            // //// Tool changing slave id
            // qnode_->tool_changing_attach_id = 4; // # slave #1, 2, ...
            // qnode_->tool_changing_detach_id = 4; // # slave #1, 2, ...


            //// Detaching count initialize
            //// NOTICE: [주의] 물체 radio button을 누르면 0으로 초기화됨.
            ptr_object_now->detaching_cnt_ = 0;
            ptr_object_now->stacking_z_idx_ = 0;
            qnode_->test_cnt_ = 0;

            std::vector<double> stacking_trans_scale = {0.225, 0.050, 0.045}; // the scale of (stacking_line_stack_num_, stacking_single_stack_num_, z)
        	ptr_object_now->stacking_single_stack_num_ = 4; // 1개씩 놓는 진행 방향의 물체 개수
	        ptr_object_now->stacking_line_stack_num_ = 1; // stacking_single_stack_num_만큼 놓는 선의 개수
            ptr_object_now->stacking_trans_scale_ = stacking_trans_scale;
            ptr_object_now->stacking_mode_ = StackingType::CYLINDER_STACKING;

            //// 1단의 실린더 개수
            ptr_object_now->stack_part_cnt_ = ((ptr_object_now->stacking_single_stack_num_ * (ptr_object_now->stacking_single_stack_num_ + 1)) / 2) * ptr_object_now->stacking_line_stack_num_;
            //// counting 최대 개수
            ptr_object_now->max_cnt_detaching_ = ptr_object_now->stack_part_cnt_; // 한 단의 실린더 스택, ex) 한 개의 단은 10개씩, 두 단은 총 20개


            qnode_->m_bin_picking_node->is_task_in_progress = false;

            //// Scan sampling setting
            ui.radioButton_assemblyUI_scanSampling->setChecked(true);
            ui.lineEdit_scanSamplingNum->setText("4");
        }
        break;

        case TargetObject::OBJECT_CHEMICAL_COUPLER_HOLDER :
        {
            ROS_LOG_WARN("Target grasping object: DRUM HOSE!");
            taskTargetParts_[0]->setText(QString::number(ON_PART_RIGHT_CHEMICAL_COUPLER_HOLDER));
            taskTargetParts_[1]->setText("right_holder_with_coupler");
            ui.lineEdit_targetName->setText("right_holder_with_coupler");
            //// load mrcnn weight
            ui_cmd_parameter.target_name = ui.lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = ui.lineEdit_targetWeightNumber->text().toStdString();
            // ui_cmd_parameter.is_sam_mean_size_assigned = false; // SAM paramter
            // ui_cmd_parameter.sam_mean_size = 0; // SAM paramter
            // ui_cmd_parameter.sam_mask_min_area = 6000;
            // ui_cmd_parameter.sam_mask_max_area = 300000;
            // qnode_->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            //// 드럼통 과제
            if(qnode_->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;

            qnode_->m_bin_picking_node->is_task_in_progress = false;

            //// Scan sampling setting
            ui.radioButton_assemblyUI_scanSampling->setChecked(true);
            ui.lineEdit_scanSamplingNum->setText("6");
        }
        break;

        case TargetObject::OBJECT_DRUM_HOLE_SURFACE :
        {
            ROS_LOG_WARN("Target grasping object: DRUM HOLE SURFACE!");
            taskTargetParts_[0]->setText(QString::number(ON_PART_RIGHT_DRUM_HOLE_SURFACE));
            taskTargetParts_[1]->setText("right_drum_hole_surface");
            ui.lineEdit_targetName->setText("right_drum_hole_surface");
            //// load mrcnn weight
            ui_cmd_parameter.target_name = ui.lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = ui.lineEdit_targetWeightNumber->text().toStdString();
            // ui_cmd_parameter.is_sam_mean_size_assigned = false; // SAM paramter
            // ui_cmd_parameter.sam_mean_size = 0; // SAM paramter
            // ui_cmd_parameter.sam_mask_min_area = 6000;
            // ui_cmd_parameter.sam_mask_max_area = 300000;
            // qnode_->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter);

            //// Robot TCP selection
            //// 드럼통 과제
            if(qnode_->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;

            qnode_->m_bin_picking_node->is_task_in_progress = false;

            //// Scan sampling setting
            ui.radioButton_assemblyUI_scanSampling->setChecked(true);
            ui.lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObject::OBJECT_DRUM_HOLE_UNSCREWING :
        {
            ROS_LOG_WARN("Target grasping object: DRUM_HOLE_UNSCREWING!");
            taskTargetParts_[0]->setText(QString::number(ON_PART_RIGHT_DRUM_COUPLER_UNSCREWING));
            taskTargetParts_[1]->setText("right_drum_coupler_unscrewing");
            ui.lineEdit_targetName->setText("right_drum_coupler_unscrewing");
            //// load mrcnn weight
            ui_cmd_parameter.target_name = ui.lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = ui.lineEdit_targetWeightNumber->text().toStdString();
            //// Robot TCP selection
            //// 드럼통 과제
            if(qnode_->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;

            qnode_->m_bin_picking_node->is_task_in_progress = false;

            //// Scan sampling setting
            ui.radioButton_assemblyUI_scanSampling->setChecked(true);
            ui.lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObject::OBJECT_DRUM_LID_CAP_UNSCREWING :
        {
            ROS_LOG_WARN("Target grasping object: DRUM_HOLE_STAR!");
            taskTargetParts_[0]->setText(QString::number(ON_PART_RIGHT_DRUM_LID_CAP_UNSCREWING));
            taskTargetParts_[1]->setText("right_drum_lid_cap_unscrewing");
            ui.lineEdit_targetName->setText("right_drum_lid_cap_unscrewing");
            //// load mrcnn weight
            ui_cmd_parameter.target_name = ui.lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = ui.lineEdit_targetWeightNumber->text().toStdString();
            //// Robot TCP selection
            //// 드럼통 과제
            if(qnode_->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;

            qnode_->m_bin_picking_node->is_task_in_progress = false;

            //// Scan sampling setting
            ui.radioButton_assemblyUI_scanSampling->setChecked(true);
            ui.lineEdit_scanSamplingNum->setText("8");
        }
        break;

        case TargetObject::OBJECT_DRUM_LID_CAP_HOLDER :
        {
            ROS_LOG_WARN("Target grasping object: DRUM_LID_HOLDER!");
            taskTargetParts_[0]->setText(QString::number(ON_PART_RIGHT_DRUM_LID_CAP_HOLDER));
            taskTargetParts_[1]->setText("right_holder_with_lid_cap");
            ui.lineEdit_targetName->setText("right_holder_with_lid_cap");
            //// load mrcnn weight
            ui_cmd_parameter.target_name = ui.lineEdit_targetName->text().toStdString();
            ui_cmd_parameter.weight_number = ui.lineEdit_targetWeightNumber->text().toStdString();
            //// Robot TCP selection
            //// 드럼통 과제
            if(qnode_->is_tcp_initialized) changeRobotTCP(1); // TCP #13(tip changing - round tip)
            ptr_object_now->tcp_changing_id = 1;

            qnode_->m_bin_picking_node->is_task_in_progress = false;

            //// Scan sampling setting
            ui.radioButton_assemblyUI_scanSampling->setChecked(true);
            ui.lineEdit_scanSamplingNum->setText("8");
        }
        break;
    }

    //// 초기 TCP는 tool master 기준
    if(!qnode_->is_tcp_initialized) changeRobotTCP(7); // TCP #7 // Robot end-effector
    // if(!qnode_->is_tcp_initialized) changeRobotTCP(1); // TCP #1, 2지그리퍼
    // if(!qnode_->is_tcp_initialized) changeRobotTCP(8); // TCP #8 --> Robot Calibration
    // if(!qnode_->is_tcp_initialized) changeRobotTCP(9); // TCP #9 --> (0.0, 0.0, 0.0)
    // if(!qnode_->is_tcp_initialized) changeRobotTCP(10); // TCP #10, Connector gripper

    // Set scanning parameters
    ptr_object_now->m_scan_parameter.target_id = taskTargetParts_[0]->text().toInt();
    ptr_object_now->m_scan_parameter.target_name = taskTargetParts_[1]->text().toStdString();
    ptr_object_now->m_scan_parameter.do_scan_sampling = ui.radioButton_assemblyUI_scanSampling->isChecked();
    ptr_object_now->m_scan_parameter.sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();
    ptr_object_now->m_scan_parameter.is_mask_pixel_fixed = ui.radioButton_assemblyUI_isMaskFixed->isChecked();
    ptr_object_now->m_scan_parameter.do_save_data = false;
    ptr_object_now->m_scan_parameter.do_not_scan_do_load_data = false;
    ptr_object_now->m_scan_parameter.skip_detection_mask = false;

    ptr_object_now->m_scan_parameter.do_single_matching = false; // matching topic


    // SAM,
    // 추후 아래 파라미터는 삭제
    ptr_object_now->m_scan_parameter.weight_number = ui_cmd_parameter.weight_number;
    ptr_object_now->m_scan_parameter.is_sam_mean_size_assigned = ui_cmd_parameter.is_sam_mean_size_assigned;
    ptr_object_now->m_scan_parameter.sam_mean_size = ui_cmd_parameter.sam_mean_size;
    ptr_object_now->m_scan_parameter.sam_mask_min_area = ui_cmd_parameter.sam_mask_min_area;
    ptr_object_now->m_scan_parameter.sam_mask_max_area = ui_cmd_parameter.sam_mask_max_area;


    // Set matching parameters
    ptr_object_now->m_matching_parameter.debug_mode = true;
    ptr_object_now->m_matching_parameter.is_base_frame_unknown = false;
    ptr_object_now->m_matching_parameter.do_scan_sampling = ui.radioButton_assemblyUI_scanSampling->isChecked();
    ptr_object_now->m_matching_parameter.sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();
    // Set grasping parameters
    ptr_object_now->m_grasping_parameter.gripper_open_length = ui.task_spinBox_gripperPosValue->value();
    ptr_object_now->m_grasping_parameter.gripper_close_length = gripper_close_length;

    std::vector<int> mask_pixel_list(4,10);
    for(int i=0; i<4; i++) mask_pixel_list[i] = fixedMaskPixelList_[i]->text().toInt();
    ptr_object_now->m_scan_parameter.mask_pixel_list = mask_pixel_list;

    ////



    // ////////////////////////////////////////////////////////
    // //// Tool Change - gripper initialize
    // if(qnode_->checkConnectionKorasGripperDriver())
    // {
    //     if(0) // dual gripper
    //     {
    //         for(int i=0; i<2; i++)
    //         {
    //             ui.task_spinBox_gripperSlaveNum->setValue(i+1); // Gripper driver #
    //             pushButtonKorasGripperSlaveChangeCallback(); // slave change
    //             // pushButtonKorasGripperIntializeCallback();
    //         }
    //     }
    //     else // single gripper
    //     {
    //         // ui.task_spinBox_gripperSlaveNum->setValue(2); // Gripper driver #
    //         ui.task_spinBox_gripperSlaveNum->setValue(1); // Gripper driver #
    //         pushButtonKorasGripperSlaveChangeCallback(); // slave change
    //         // pushButtonKorasGripperIntializeCallback();
    //     }
    // }

    ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////
    //// NOTICE: 240220, 실행 초기에만 실행되는 부분
    //// NOTICE: 240302, 파라미터 세팅으로만 설정하도록 항상 is_template_initialized_ = true
    //// Template JSON import
    taskTemplateMatchingParameter template_parameter;
    template_parameter.target_id = taskTargetParts_[0]->text().toInt();
    template_parameter.target_name = taskTargetParts_[1]->text().toStdString();
    template_parameter.robot_dh_vec = qnode_->getRobotDHParameters();
    template_parameter.robot_tcp_default = qnode_->getRobotDefaultTCP();
    template_parameter.robot_tcp = qnode_->getRobotTCP();

    if(!ptr_object_now->is_template_initialized_) { // CAD importing
        ROS_LOG_WARN("***** Template_initialized *****");
        ptr_object_now->is_template_initialized_ = true;
        bool do_set_only_parameters = false;
        bool do_overwrite_JSON = true;
        // qnode_->m_bin_picking_node->doTemplateInitialize(template_parameter, do_set_only_parameters, do_overwrite_JSON);
    } else { // only set parameters for JSON
        bool do_set_only_parameters = true;
        bool do_overwrite_JSON = true;
        // qnode_->m_bin_picking_node->doTemplateInitialize(template_parameter, do_set_only_parameters, do_overwrite_JSON);
    }


    //// Drum 과제
    bool do_set_only_parameters = true;
    bool do_overwrite_JSON = true;
    if(target_object_idx == TargetObject::OBJECT_CHEMICAL_COUPLER_HOLDER || target_object_idx == TargetObject::OBJECT_DRUM_HOLE_SURFACE || target_object_idx == TargetObject::OBJECT_DRUM_HOLE_UNSCREWING || target_object_idx == TargetObject::OBJECT_DRUM_LID_CAP_UNSCREWING || target_object_idx == TargetObject::OBJECT_DRUM_LID_CAP_HOLDER) {
        qnode_->m_bin_picking_node->publishUICommandLearningMsg(ui_cmd_parameter); // sam
        qnode_->m_bin_picking_node->doTemplateInitialize(template_parameter, do_set_only_parameters, do_overwrite_JSON); // matching_node
    }

    ////

    ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////


    //// Enable task button
    if(qnode_->is_task_generated_) {
        ui.pushButton_doTask_binPicking->setEnabled(true);
        ui.pushButton_doTask_binPicking_sequentialDemo->setEnabled(true);
    }
#endif
}

void BinPickingDialog::pushButtonSelectRobotTCPClickedCallback() {
#if BIN_PICKING_FLAG

    size_t robot_tcp_idx = 0;
    for(int i=0; i<robot_tcp_list_.size(); i++) {
        if(robot_tcp_list_[i]->isChecked()) { robot_tcp_idx = i; }
    }
    uint16_t gripper_close_length = 0;
    switch(robot_tcp_idx)
    {
        case 0 :
        {
            ROS_LOG_INFO("Robot TCP 1 (main hand)");
            ui.task_spinBox_gripperPosValue->setValue(7500);
            gripper_close_length = 8250;
            ui.task_spinBox_gripperSlaveNum->setValue(2); // Gripper driver #2
            pushButtonKorasGripperSlaveChangeCallback(); // slave change

            //// Robot TCP selection
            if(qnode_->is_tcp_initialized) changeRobotTCP(1); // TCP #1
        }
        break;
        case 1 :
        {
            ROS_LOG_INFO("Robot TCP 2 (sub hand)");
            ui.task_spinBox_gripperPosValue->setValue(7500);
            gripper_close_length = 8250;
            ui.task_spinBox_gripperSlaveNum->setValue(1); // Gripper driver #1
            pushButtonKorasGripperSlaveChangeCallback(); // slave change

            //// Robot TCP selection
            if(qnode_->is_tcp_initialized) changeRobotTCP(2); // TCP #2
        }
        break;

    }
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.gripper_open_length = ui.task_spinBox_gripperPosValue->value();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.gripper_close_length = gripper_close_length;

    //// Tool Change - gripper initialize
    // pushButtonKorasGripperIntializeCallback();
#endif
}

void BinPickingDialog::pushButtonSelectToolTipClickedCallback() {
#if BIN_PICKING_FLAG

    size_t tool_tip_idx = 0;
    for(int i=0; i<tool_tip_list_.size(); i++) {
        if(tool_tip_list_[i]->isChecked()) { tool_tip_idx = i; }
    }
    switch(tool_tip_idx)
    {
        case 0 :
        {
            ROS_LOG_INFO("Tool Tip 1");
            qnode_->tip_changing_attach_id = 1; // tip #1, 2, ...
            qnode_->tip_changing_detach_id = 1; // tip #1, 2, ...
            qnode_->setTipChangingCurrentIndex(qnode_->tip_changing_attach_id);
        }
        break;
        case 1 :
        {
            ROS_LOG_INFO("Tool Tip 2");
            qnode_->tip_changing_attach_id = 2; // tip #1, 2, ...
            qnode_->tip_changing_detach_id = 2; // tip #1, 2, ...
            qnode_->setTipChangingCurrentIndex(qnode_->tip_changing_attach_id);
        }
        break;
        case 2 :
        {
            ROS_LOG_INFO("Tool Tip 3");
            qnode_->tip_changing_attach_id = 3; // tip #1, 2, ...
            qnode_->tip_changing_detach_id = 3; // tip #1, 2, ...
            qnode_->setTipChangingCurrentIndex(qnode_->tip_changing_attach_id);
        }
        break;
    }
#endif
}


void BinPickingDialog::pushButtonSelectTemplateMatchingSegmentationClickedCallback() {
#if BIN_PICKING_FLAG

    size_t target_idx = 0;
    for(int i=0; i<UI_template_matching_seg_parameter_list_.size(); i++) {
        if(UI_template_matching_seg_parameter_list_[i]->isChecked()) { target_idx = i; }
    }
    switch(target_idx)
    {
        case 0 :
        {
            // ROS_LOG_INFO("RGS!");
            ui.stackedWidgetSegParam->setCurrentIndex(0);
        }
        break;

        case 1 :
        {
            // ROS_LOG_INFO("VCCS!");
            // ui.stackedWidgetSegParam->setCurrentIndex(1);
            ui.stackedWidgetSegParam->setCurrentIndex(3);
        }
        break;

        case 2 :
        {
            // ROS_LOG_INFO("Color-based RGS!");
            ui.stackedWidgetSegParam->setCurrentIndex(2);
        }
        break;

        case 3 :
        {
            // ROS_LOG_INFO("Euclidean cluster extraction!");
            ui.stackedWidgetSegParam->setCurrentIndex(3);
        }
        break;

        case 4 :
        {
            // ROS_LOG_INFO("Conditional Euclidean Clustering!");
            ui.stackedWidgetSegParam->setCurrentIndex(4);
        }
        break;
    }
#endif
}

void BinPickingDialog::FTdataAcqusitionStartOnClickedCallback() {

#if BIN_PICKING_FLAG
    //// 기존 파이썬 코드
    if(ui.pushButtonDataAcquisitionStart->isChecked())
    {
        qnode_->acquisition_mode = 2; // data acqusition start
        qnode_->data_index++;
        ROS_LOG_INFO("(UI button)Data acquisition start! (data index: %i)", qnode_->data_index);
    }
    else
    {
        qnode_->acquisition_mode = 1; // data acqusition end
        ROS_LOG_INFO("(UI button)Data acquisition End!(data index: %i)", qnode_->data_index);
    }

    // //// UDP 코드
    // std::string ip = ui.lineEdit_UDP_Port->text().toStdString();
    // int portno = static_cast<int>(ui.lineEdit_UDP_Port->text().toDouble());
    // if(ui.pushButtonDataAcquisitionStart->isChecked())
    // {
    //     qnode_->setUDPComm("udp_comm_acquisition_start", ip, portno);
    // }
    // else
    // {
    //     qnode_->setUDPComm("udp_comm_acquisition_stop", ip, portno);
    // }
#endif
}

void BinPickingDialog::UDPFTdataAcqusitionStartOnClickedCallback() {
#if BIN_PICKING_FLAG
    //// UDP 코드
    std::string ip = ui.lineEdit_UDP_IP->text().toStdString();
    int portno = static_cast<int>(ui.lineEdit_UDP_Port->text().toDouble());
    qnode_->ip_udp = ip;
    qnode_->port_udp = portno;
    if(ui.pushButton_UDP_DataAcquisitionStart->isChecked())
    {
        qnode_->setUDPComm("udp_comm_acquisition_start", ip, portno);
    }
    else
    {
        qnode_->setUDPComm("udp_comm_acquisition_stop", ip, portno);
    }
#endif
}

void BinPickingDialog::UDPConnectOnClickedCallback() {
#if BIN_PICKING_FLAG
    std::string ip = ui.lineEdit_UDP_IP->text().toStdString();
    int portno = static_cast<int>(ui.lineEdit_UDP_Port->text().toDouble());
    qnode_->ip_udp = ip;
    qnode_->port_udp = portno;
    qnode_->setUDPComm("udp_comm_connect", ip, portno);

#endif
}

void BinPickingDialog::UDPCloseOnClickedCallback() {
#if BIN_PICKING_FLAG
    std::string ip = ui.lineEdit_UDP_IP->text().toStdString();
    int portno = static_cast<int>(ui.lineEdit_UDP_Port->text().toDouble());
    qnode_->ip_udp = ip;
    qnode_->port_udp = portno;
    qnode_->setUDPComm("udp_comm_close", ip, portno);
#endif
}

void BinPickingDialog::pushButtonGenerateSingleGraspingTasksCallback() {
#if BIN_PICKING_FLAG
    setInitialTaskRobotParameters(); // Initialze task and parameters

    bool is_slow_mode;
    if(ui.radioButton_setRobotAccVel_DEBUG->isChecked()) {
        ROS_LOG_WARN("TASK SLOW MODE!");
        is_slow_mode = true;
    }
    if(ui.radioButton_setRobotAccVel_RUN->isChecked()) {
        ROS_LOG_WARN("TASK FAST MODE!");
        is_slow_mode = false;
    }

    //// Default
    // qnode_->task_planner_->makeRecogOnlyGraspingTaskList(is_slow_mode);
    //// 반복 TEST
    qnode_->task_planner_->makeTestRecogOnlyGraspingTaskList(is_slow_mode);

    // Tool changing task generation
    pushButtonGenerateToolChangingTasksCallback();

    // Tip changing task generation
    pushButtonGenerateTipChangingTasksCallback();

    qnode_->is_task_generated_ = true; // button enable
#endif
}

void BinPickingDialog::pushButtonDoLLMTestFunctionTasksCallback() {

    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching = false; // matching topic
    if(ui.pushButton_doTask_LLM_TestFunction->isChecked()) {
        // Flag initialize
        ROS_LOG_WARN("LLM Test Function!");

        int current_obj = 0;
        int velocity_level = 1;
        switch(qnode_->m_bin_picking_node->current_target_object_idx_)
        {
            case TargetObject::OBJECT_STRIKER :
            {
                ROS_LOG_WARN("Target grasping object: OBJECT_STRIKER!");
                current_obj = 39;
            }
            break;
            case TargetObject::OBJECT_SCU :
            {
                ROS_LOG_WARN("Target grasping object: OBJECT_SCU!");
                current_obj = 40;
            }
            break;
            case TargetObject::OBJECT_GRILL :
            {
                ROS_LOG_WARN("Target grasping object: OBJECT_GRILL!");
                current_obj = 41;
            }
            break;
            case TargetObject::OBJECT_HINGE :
            {
                ROS_LOG_WARN("Target grasping object: OBJECT_HINGE!");
                current_obj = 38;
            }
            break;
            case TargetObject::OBJECT_BOLT :
            {
                ROS_LOG_WARN("Target grasping object: OBJECT_BOLT!");
                current_obj = 37;
            }
            break;
        }
        // qnode_->task_planner_->LLMScanMatchingGrasping(current_obj, velocity_level);
        // qnode_->current_task_list_ = qnode_->task_planner_->llm_task_zivid_;

        qnode_->task_planner_->LLMTestTask(current_obj, velocity_level);
        qnode_->current_task_list_ = qnode_->task_planner_->llm_task_test_;

        qnode_->beforeBinPickingTaskStart(true);
        qnode_->task_cycle_ = 0;
    } else {
        pushButtonSTOPAllCallback();
    }
}

void BinPickingDialog::rightPushButtonDoDrumTotalTasksCallback() {

    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching = false; // matching topic
    if(ui.rightPushButton_doTask_drum_total_task->isChecked()) {
        // Flag initialize
        ROS_LOG_WARN("LLM Test Function!");

        qnode_->task_planner_->rightGenDrumTask();
        qnode_->current_task_list_ = qnode_->task_planner_->drum_total_task_;

        qnode_->beforeBinPickingTaskStart(true);
        qnode_->task_cycle_ = 0;

    } else {
        pushButtonSTOPAllCallback();
    }


// #if DRFL_CONTROL
//                 //// 두산 로봇의 경우, ZYZ angle
//                 ROS_LOG_WARN("*******************************************");
//                 ROS_LOG_WARN("[%s]DRFL_CONTROL MODE! - ZYX to ZYZ euler angle", __func__);
//                 ROS_LOG_WARN("[%s]DRFL_CONTROL MODE! - ZYX to ZYZ euler angle", __func__);
//                 ROS_LOG_WARN("[%s]DRFL_CONTROL MODE! - ZYX to ZYZ euler angle", __func__);


//                 // input: [rad]
//                 Eigen::Matrix3f Rx = qnode_->m_bin_picking_node->m_bp_math.genRotMf(180.0 * kDeg2Rad, 0); // 0: rx, 1:ry, 2:rz
//                 Eigen::Matrix3f Ry = qnode_->m_bin_picking_node->m_bp_math.genRotMf(0.0 * kDeg2Rad, 1); // 0: rx, 1:ry, 2:rz
//                 Eigen::Matrix3f Rz = qnode_->m_bin_picking_node->m_bp_math.genRotMf(90.0 * kDeg2Rad, 2); // 0: rx, 1:ry, 2:rz

//                 Eigen::Matrix3f R = Rz*Ry*Rx;

//                 // Convert back to ZYZ Euler angles
//                 Eigen::Vector3f euler_angles_zyz = qnode_->m_bin_picking_node->m_bp_math.rotationMatrixToZYZEulerAngles(R);
//                 cout << "Rotation Matrix:\n" << R << endl;
//                 cout << "ZYZ Euler Angles (degrees):\n" << euler_angles_zyz.transpose() << endl;


//                 ROS_LOG_WARN("*******************************************");
//                 // R_ret = Eigen::AngleAxisf(euler_angles_zyz(0)*(M_PI/180.0), Eigen::Vector3f::UnitZ())
//                 //     * Eigen::AngleAxisf(euler_angles_zyz(1)*(M_PI/180.0), Eigen::Vector3f::UnitY())
//                 //     * Eigen::AngleAxisf(euler_angles_zyz(2)*(M_PI/180.0), Eigen::Vector3f::UnitZ());
//                 Eigen::Matrix3f Rz_ret1 = qnode_->m_bin_picking_node->m_bp_math.genRotMf(euler_angles_zyz[0] * kDeg2Rad, 0); // 0: rx, 1:ry, 2:rz
//                 Eigen::Matrix3f Ry_ret = qnode_->m_bin_picking_node->m_bp_math.genRotMf(euler_angles_zyz[1] * kDeg2Rad, 1); // 0: rx, 1:ry, 2:rz
//                 Eigen::Matrix3f Rz_ret2 = qnode_->m_bin_picking_node->m_bp_math.genRotMf(euler_angles_zyz[2] * kDeg2Rad, 2); // 0: rx, 1:ry, 2:rz
//                 Eigen::Matrix3f R_ret = Rz_ret1*Ry_ret*Rz_ret2;
//                 cout << "[For Checking] Rotation Matrix (from ZYZ):\n" << R_ret << endl;


//                 ROS_LOG_WARN("*******************************************");


// #endif

}

void BinPickingDialog::pushButtonDoDrumScanTasksCallback() {

    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching = false; // matching topic
    if(ui.pushButton_doTask_drum_scan_task->isChecked()) {
        // Flag initialize
        ROS_LOG_WARN("Drum Scanning Task Function!");

        qnode_->task_planner_->rightGenDrumTask();
        qnode_->current_task_list_ = qnode_->task_planner_->drum_scan_sub_task_;

        qnode_->beforeBinPickingTaskStart(true);
        qnode_->task_cycle_ = 0;
    } else {
        pushButtonSTOPAllCallback();
    }
}




void BinPickingDialog::pushButtonDoKorasChemicalGripperInitializeCallback() {

    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching = false; // matching topic


    ROS_LOG_WARN("Chemical Gripper Initialize!");


    qnode_->task_planner_->rightGenDrumTask();
    ROS_LOG_WARN("Chemical Gripper Initialize!1111");

    qnode_->current_task_list_ = qnode_->task_planner_->drum_grp_initialize_task_;
    ROS_LOG_WARN("Chemical Gripper Initialize!22222");

    qnode_->beforeBinPickingTaskStart(true);
    ROS_LOG_WARN("Chemical Gripper Initialize!333");

    qnode_->task_cycle_ = 0;
    ROS_LOG_WARN("Chemical Gripper Initialize!444");

}

void BinPickingDialog::pushButtonDoKorasChemicalGripperOpenCallback() {

    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching = false; // matching topic


    ROS_LOG_WARN("Chemical Gripper Initialize!");


    qnode_->task_planner_->rightGenDrumTask();
    qnode_->current_task_list_ = qnode_->task_planner_->drum_grp_lid_cap_close_;

    qnode_->beforeBinPickingTaskStart(true);
    qnode_->task_cycle_ = 0;

}

void BinPickingDialog::pushButtonDoKorasChemicalGripperCloseCallback() {

    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching = false; // matching topic


    ROS_LOG_WARN("Chemical Gripper Initialize!");


    qnode_->task_planner_->rightGenDrumTask();
    ROS_LOG_WARN("Chemical Gripper Initialize!1111111111111");

    qnode_->current_task_list_ = qnode_->task_planner_->drum_grp_lid_cap_open_;

    ROS_LOG_WARN("Chemical Gripper Initialize!2222222222222");

    qnode_->beforeBinPickingTaskStart(true);
    qnode_->task_cycle_ = 0;
    ROS_LOG_WARN("Chemical Gripper Initialize!33333333333333");

}

void BinPickingDialog::pushButtonDoKorasChemicalGripperGraspingPoseCallback() {

    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching = false; // matching topic

    ROS_LOG_WARN("Chemical Gripper Grasping Pose!");

    qnode_->task_planner_->rightGenDrumTask();
    qnode_->current_task_list_ = qnode_->task_planner_->drum_grp_grasping_pose_task_;

    qnode_->beforeBinPickingTaskStart(true);
    qnode_->task_cycle_ = 0;
}

void BinPickingDialog::pushButtonDoKorasChemicalGripperScrewingPoseCallback() {

    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching = false; // matching topic


    ROS_LOG_WARN("Chemical Gripper Screwing Pose!");


    qnode_->task_planner_->rightGenDrumTask();
    qnode_->current_task_list_ = qnode_->task_planner_->drum_grp_screwing_pose_task_;

    qnode_->beforeBinPickingTaskStart(true);
    qnode_->task_cycle_ = 0;

}

void BinPickingDialog::pushButtonDoKorasChemicalGripperUnscrewingPoseCallback() {

    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching = false; // matching topic


    ROS_LOG_WARN("Chemical Gripper Unscrewing Pose!");

    qnode_->task_planner_->rightGenDrumTask();
    qnode_->current_task_list_ = qnode_->task_planner_->drum_grp_unscrewing_pose_task_;

    qnode_->beforeBinPickingTaskStart(true);
    qnode_->task_cycle_ = 0;

}

void BinPickingDialog::pushButtonDoKorasChemicalGripperExitPoseCallback() {

    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching = false; // matching topic

    ROS_LOG_WARN("Chemical Gripper Exit Pose!");

    qnode_->task_planner_->rightGenDrumTask();
    qnode_->current_task_list_ = qnode_->task_planner_->drum_grp_exit_pose_task_;

    qnode_->beforeBinPickingTaskStart(true);
    qnode_->task_cycle_ = 0;
}


void BinPickingDialog::pushButtonDoSingleGraspingTasksCallback() {
#if BIN_PICKING_FLAG

    qnode_->is_sequential_demo_ = false;

    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching = false; // matching topic
    if(ui.pushButton_doTask_binPicking->isChecked()) {
        // Flag initialize
        qnode_->m_bin_picking_node->bin_scan_fail = false; // flag initialize
        qnode_->m_bin_picking_node->is_grasping_fail = false;
        if(bin_picking_target_object_list_[5]->isChecked()) {
            //// Pick and place
            ROS_LOG_INFO("Pick-and-Place Task - cylinder (2F Gripper)!");
            // qnode_->current_task_list_ = qnode_->task_planner_->module_task_bin_picking_pick_and_place_cylinder_;

            //// Graphy only grasping task
            qnode_->current_task_list_ = qnode_->task_planner_->module_task_graphy_stage5_only_grasping_test_;

        } else if(bin_picking_target_object_list_[1]->isChecked()) {
            //// Pick and place
            ROS_LOG_INFO("Pick-and-Place Task - square peg (Vacuum gripper)!");
            qnode_->current_task_list_ = qnode_->task_planner_->module_task_bin_picking_pick_and_place_square_peg_;

            //// Milling
            // ROS_LOG_INFO("Single Machine Tending (Milling) Task - square peg!");
            // qnode_->current_task_list_ = qnode_->task_planner_->module_task_bin_picking_single_machine_tending_milling_;
        } else if(bin_picking_target_object_list_[2]->isChecked()) {
            //// Pick and place
            ROS_LOG_INFO("Pick-and-Place Task - eb joint (2F Gripper)!");
            qnode_->current_task_list_ = qnode_->task_planner_->module_task_bin_picking_pick_and_place_eb_joint_;
        } else if(bin_picking_target_object_list_[3]->isChecked()) {
            //// Pick and place
            ROS_LOG_INFO("Pick-and-Place Task! - t joint (2F Gripper)");
            qnode_->current_task_list_ = qnode_->task_planner_->module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_;
        } else if(bin_picking_target_object_list_[4]->isChecked()) {
            //// Pick and place
            ROS_LOG_INFO("Pick-and-Place Task (inner plane grasping)!");
            // qnode_->current_task_list_ = qnode_->task_planner_->module_task_bin_picking_machine_tending_bolt_bush_;
            qnode_->current_task_list_ = qnode_->task_planner_->module_task_bin_picking_pick_and_place_bolt_bush_;
        } else if(bin_picking_target_object_list_[7]->isChecked()) {
            //// Pick and place
            ROS_LOG_INFO("Pick-and-Place Task! - t joint (Vacuum Gripper)");
            qnode_->current_task_list_ = qnode_->task_planner_->module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_;


        } else if(bin_picking_target_object_list_[8]->isChecked()) {
            //// Pick and place
            ROS_LOG_INFO("Graphy Task - Test 1!");

            //// NOTICE: Graphy Test Task!!!
            //// NOTICE: Graphy Test Task!!!
            //// NOTICE: Graphy Test Task!!!

            qnode_->current_task_list_.clear();
            // 5-a) grasping
            qnode_->current_task_list_ = qnode_->task_planner_->module_task_graphy_stage5_grasping_test_1_;
            // 5-b) tip changing 1 to 2
            qnode_->task_planner_->makeSetTipIndexTask(1); // tip index #1
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_set_current_tip_idx_.begin(), qnode_->task_planner_->module_task_tip_changing_set_current_tip_idx_.end());
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_detach_motion_.begin(), qnode_->task_planner_->module_task_tip_changing_detach_motion_.end());
            qnode_->task_planner_->makeSetTipIndexTask(2); // tip index #2
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_set_current_tip_idx_.begin(), qnode_->task_planner_->module_task_tip_changing_set_current_tip_idx_.end());
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_attach_motion_.begin(), qnode_->task_planner_->module_task_tip_changing_attach_motion_.end());
            // 5-c) rotating plate
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_graphy_stage5_plate_rotation_test_1_.begin(), qnode_->task_planner_->module_task_graphy_stage5_plate_rotation_test_1_.end());
            // 5-d) tip changing 2 to 1
            qnode_->task_planner_->makeSetTipIndexTask(2); // tip index #1
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_set_current_tip_idx_.begin(), qnode_->task_planner_->module_task_tip_changing_set_current_tip_idx_.end());
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_detach_motion_.begin(), qnode_->task_planner_->module_task_tip_changing_detach_motion_.end());
            qnode_->task_planner_->makeSetTipIndexTask(1); // tip index #2
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_set_current_tip_idx_.begin(), qnode_->task_planner_->module_task_tip_changing_set_current_tip_idx_.end());
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_attach_motion_.begin(), qnode_->task_planner_->module_task_tip_changing_attach_motion_.end());


        } else if(bin_picking_target_object_list_[9]->isChecked()) {
            //// Pick and place
            ROS_LOG_INFO("Graphy Task - Test 2!");

            //// NOTICE: Graphy Test Task!!!
            //// NOTICE: Graphy Test Task!!!
            //// NOTICE: Graphy Test Task!!!

            qnode_->current_task_list_.clear();
            // 5-a) grasping
            qnode_->current_task_list_ = qnode_->task_planner_->module_task_graphy_stage5_grasping_test_2_;


            ///////////////////////////////////
            //// NOTICE: 장원 이형
            // 5-b) tip changing 1 to 2
            qnode_->task_planner_->makeSetTipIndexTask(1); // tip index #1 (왼)
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_set_current_tip_idx_.begin(), qnode_->task_planner_->module_task_tip_changing_set_current_tip_idx_.end());
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_detach_motion_.begin(), qnode_->task_planner_->module_task_tip_changing_detach_motion_.end());
            qnode_->task_planner_->makeSetTipIndexTask(2); // tip index #2 (오)
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_set_current_tip_idx_.begin(), qnode_->task_planner_->module_task_tip_changing_set_current_tip_idx_.end());
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_attach_motion_.begin(), qnode_->task_planner_->module_task_tip_changing_attach_motion_.end());
            // 5-c) rotating plate
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_graphy_stage5_plate_rotation_test_2_.begin(), qnode_->task_planner_->module_task_graphy_stage5_plate_rotation_test_2_.end());
            // 5-d) tip changing 2 to 1
            qnode_->task_planner_->makeSetTipIndexTask(2); // tip index #1 (오)
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_set_current_tip_idx_.begin(), qnode_->task_planner_->module_task_tip_changing_set_current_tip_idx_.end());
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_detach_motion_.begin(), qnode_->task_planner_->module_task_tip_changing_detach_motion_.end());
            qnode_->task_planner_->makeSetTipIndexTask(1); // tip index #2 (왼)
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_set_current_tip_idx_.begin(), qnode_->task_planner_->module_task_tip_changing_set_current_tip_idx_.end());
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_attach_motion_.begin(), qnode_->task_planner_->module_task_tip_changing_attach_motion_.end());
            ///////////////////////////////////

        } else {
            // ROS_LOG_INFO("Pick-and-Place Task!");
            // qnode_->current_task_list_ = qnode_->task_planner_->module_task_bin_picking_pick_and_place_;
        }
        //// Initialize unit task
        if(!qnode_->m_bin_picking_node->is_grasping_pose_cal_finished) {
            ROS_LOG_WARN("Not performed - initial scan & matching");
            ROS_LOG_WARN("Not performed - initial scan & matching");
            ROS_LOG_WARN("Not performed - initial scan & matching");
            qnode_->beforeBinPickingTaskStart(true); // unit task initialize
        } else {
            qnode_->beforeBinPickingTaskStart(false); // in process
        }
        qnode_->task_cycle_ = 0;
    } else {
        pushButtonSTOPAllCallback();
        qnode_->m_bin_picking_node->is_task_in_progress = false;
        qnode_->m_bin_picking_node->is_initial_scan_done = false;
    }
#endif
}


void BinPickingDialog::pushButtonDoSequentialDemoTasksCallback2() {
#if BIN_PICKING_FLAG
    qnode_->is_sequential_demo_ = true;
    CTARGET_OBJECT_DATA* ptr_object_now = qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_];
    ptr_object_now->m_scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    ptr_object_now->m_scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    ptr_object_now->m_scan_parameter.do_single_matching = false; // matching topic


    if(ui.pushButton_doTask_binPicking_sequentialDemo->isChecked()) {
        // Flag initialize
        qnode_->m_bin_picking_node->bin_scan_fail = false; // flag initialize
        qnode_->m_bin_picking_node->is_grasping_fail = false;

        size_t target_object_idx = qnode_->m_bin_picking_node->current_target_object_idx_;

        //// Robot TCP update (qnode_->m_bin_picking_node->current_target_object_idx_에 저장된 정보)
        qnode_->setRobotInfoUpdateCurrentObject(); // robot tcp update and tool/tip changing idx update

        qnode_->current_task_list_.clear();
        std::vector<UnitTask> target_task_now;
        switch(target_object_idx)
        {
            case TargetObject::OBJECT_BOLT_BUSH :
            {
                ROS_LOG_WARN("Do Task Start! - TargetObject::OBJECT_BOLT_BUSH");
                target_task_now = qnode_->task_planner_->module_task_bin_picking_pick_and_place_bolt_bush_;
                qnode_->grp_do_initialize_ = true;
                qnode_->grp_init_2_grp_cmd_ = 1323; // for initialize_2
                qnode_->grp_is_vacuum_= false;
            }
            break;

            case TargetObject::OBJECT_SQUARE_PEG :
            {
                ROS_LOG_WARN("Do Task Start! - TargetObject::OBJECT_SQUARE_PEG");
                target_task_now = qnode_->task_planner_->module_task_bin_picking_pick_and_place_square_peg_;
                qnode_->grp_do_initialize_ = false;
                qnode_->grp_init_2_grp_cmd_ = 4953; // for initialize_2
                qnode_->grp_is_vacuum_= true;
            }
            break;

            case TargetObject::OBJECT_WELDING_T_JOINT_GRP_2F :
            {
                ROS_LOG_WARN("Do Task Start! - TargetObject::OBJECT_WELDING_T_JOINT_GRP_2F");
                target_task_now = qnode_->task_planner_->module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_;
                qnode_->grp_do_initialize_ = true;
                qnode_->grp_init_2_grp_cmd_ = 4953; // for initialize_2
                qnode_->grp_is_vacuum_= false;
            }
            break;

            case TargetObject::OBJECT_CYLINDER :
            {
                ROS_LOG_WARN("Do Task Start! - TargetObject::OBJECT_CYLINDER");
                target_task_now = qnode_->task_planner_->module_task_bin_picking_pick_and_place_cylinder_;
                qnode_->grp_do_initialize_ = true;
                qnode_->grp_init_2_grp_cmd_ = 4953; // for initialize_2
                qnode_->grp_is_vacuum_= false;

            }
            break;

        }

        //// Task allocation
        // a) tool changing task
        if(qnode_->getToolChangingCurrentIndex() != qnode_->tool_changing_attach_id) { // tool index가 현재 index와 다르면
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tool_changing_current_index_detach_and_attach_motion_.begin(), qnode_->task_planner_->module_task_tool_changing_current_index_detach_and_attach_motion_.end());
        }
        // b) tip changing task
        if(ptr_object_now->is_tip_changing_allowed_) {
            if(qnode_->getTipChangingCurrentIndex() != qnode_->tip_changing_attach_id) { // tip index가 현재 index와 다르면
                qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_tip_changing_current_index_detach_and_attach_motion_.begin(), qnode_->task_planner_->module_task_tip_changing_current_index_detach_and_attach_motion_.end());
            }
        }

        // c) Target task
        qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), target_task_now.begin(), target_task_now.end());


        //// Initialize unit task
        if(!qnode_->m_bin_picking_node->is_grasping_pose_cal_finished) {
            ROS_LOG_WARN("Not performed - initial scan & matching");
            ROS_LOG_WARN("Not performed - initial scan & matching");
            ROS_LOG_WARN("Not performed - initial scan & matching");
            qnode_->beforeBinPickingTaskStart(true); // unit task initialize
        } else {
            qnode_->beforeBinPickingTaskStart(false); // in process
        }
        qnode_->task_cycle_ = 0;
    } else {
        pushButtonSTOPAllCallback();
        qnode_->m_bin_picking_node->is_task_in_progress = false;
        qnode_->m_bin_picking_node->is_initial_scan_done = false;
    }
#endif
}


void BinPickingDialog::pushButtonGenerateBinPickingPoseEstimationTasksCallback() {
#if BIN_PICKING_FLAG
    setInitialTaskRobotParameters(); // Initialze task and parameters
    qnode_->task_planner_->makeRecogOnlyPoseEstimationTaskList();
#endif
}

void BinPickingDialog::pushButtonDoBinPickingPoseEstimationTasksCallback() {
#if BIN_PICKING_FLAG

    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching = false; // matching topic
    if(ui.pushButton_doTask_binPicking_poseEst->isChecked())
    {
        qnode_->beforeTaskStart(); // unit task initialize

        qnode_->current_task_list_ = qnode_->task_planner_->module_recog_only_grasping_pose_estimation_; // Task planner




    }
    else
    {
        pushButtonSTOPAllCallback();
    }

#endif
}

//// Tool Changing Task
void BinPickingDialog::pushButtonGenerateToolChangingTasksCallback() {
#if BIN_PICKING_FLAG
    setInitialTaskRobotParameters(); // Initialze task and parameters
    qnode_->task_planner_->makeToolChangeMotionTaskList();
    qnode_->task_planner_->makeInitialScanMatchingWithToolChangingMotionTaskList();

    //// Sequential demo
    qnode_->task_planner_->makeCurrentToolChangingMotionTaskList();
    qnode_->task_planner_->makeCurrentTipChangingMotionTaskList();

    ui.pushButton_doTask_toolChanging->setEnabled(true);
    ui.pushButton_doTask_toolAttaching->setEnabled(true);
    ui.pushButton_doTask_toolDetaching->setEnabled(true);
    ui.pushButton_doTask_initialScanMatching->setEnabled(true);
#endif
}

void BinPickingDialog::pushButtonDoToolChangingTasksCallback() {
#if BIN_PICKING_FLAG
    if(ui.pushButton_doTask_toolChanging->isChecked())  {
        if(!qnode_->is_tool_attached) { // Tool attaching
            ROS_LOG_INFO("Tool Attaching Task!");
            qnode_->current_task_list_ = qnode_->task_planner_->module_task_tool_changing_attach_motion_;
        } else { // Tool detaching
            ROS_LOG_INFO("Tool Detaching Task!");
            qnode_->current_task_list_ = qnode_->task_planner_->module_task_tool_changing_detach_motion_;
        }
        qnode_->beforeBinPickingTaskStart(true);
        qnode_->task_cycle_ = 0;
    } else {
        pushButtonSTOPAllCallback();
    }
#endif
}

void BinPickingDialog::pushButtonDoInitialScanMatchingTasksCallback() {
#if BIN_PICKING_FLAG

    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching = false; // matching topic
    if(ui.pushButton_doTask_initialScanMatching->isChecked()) {
        // Flag initialize
        qnode_->m_bin_picking_node->bin_scan_fail = false; // flag initialize
        qnode_->m_bin_picking_node->is_grasping_fail = false;
        if(qnode_->getToolChangingCurrentIndex() != qnode_->tool_changing_attach_id) { // with tool changing
            ROS_LOG_INFO("Initial Scan & Matching Task! - with tool changing motion");
            ROS_LOG_INFO("Current tool index: #%u", qnode_->getToolChangingCurrentIndex());
            ROS_LOG_INFO("Next tool index: #%u", qnode_->tool_changing_attach_id);
            qnode_->current_task_list_ = qnode_->task_planner_->module_task_initial_scan_matching_with_tool_changing_motion_;
            // qnode_->current_task_list_ = qnode_->task_planner_->module_task_initial_scan_matching_;
        } else { // w/o tool changing
            ROS_LOG_INFO("Initial Scan & Matching Task! - w/o tool changing motion");
            ROS_LOG_INFO("Current tool index: #%u", qnode_->getToolChangingCurrentIndex());
            ROS_LOG_INFO("Only Scan & Matching");
            qnode_->current_task_list_ = qnode_->task_planner_->module_task_initial_scan_matching_;
        }
        qnode_->beforeBinPickingTaskStart(true);
        qnode_->task_cycle_ = 0;
    } else {
        qnode_->m_bin_picking_node->is_task_in_progress = false;
        qnode_->m_bin_picking_node->is_initial_scan_done = false;
        pushButtonSTOPAllCallback();
    }
#endif
}

void BinPickingDialog::pushButtonDoToolAttachingTasksCallback() {
#if BIN_PICKING_FLAG
    if(ui.pushButton_doTask_toolAttaching->isChecked()) {
        ROS_LOG_INFO("Tool Attaching Task!");
        qnode_->is_tool_attached = false;
        qnode_->current_task_list_ = qnode_->task_planner_->module_task_tool_changing_attach_motion_;
        // for(int i=0; i<bin_picking_target_object_list_.size(); i++) bin_picking_target_object_list_[i]->setEnabled(false);
        qnode_->beforeBinPickingTaskStart(true);
        qnode_->task_cycle_ = 0;
    } else {
        pushButtonSTOPAllCallback();
    }
#endif
}

void BinPickingDialog::pushButtonDoToolDetachingTasksCallback() {
#if BIN_PICKING_FLAG
    if(ui.pushButton_doTask_toolDetaching->isChecked()) {
        ROS_LOG_INFO("Tool Detaching Task!");
        qnode_->is_tool_attached = true;
        qnode_->current_task_list_ = qnode_->task_planner_->module_task_tool_changing_detach_motion_;
        // for(int i=0; i<bin_picking_target_object_list_.size(); i++) bin_picking_target_object_list_[i]->setEnabled(true);
        qnode_->beforeBinPickingTaskStart(true);
        qnode_->task_cycle_ = 0;
    } else {
        pushButtonSTOPAllCallback();
    }
#endif
}

//// Tip Changing Task
void BinPickingDialog::pushButtonGenerateTipChangingTasksCallback() {
#if BIN_PICKING_FLAG
    setInitialTaskRobotParameters(); // Initialze task and parameters
    qnode_->task_planner_->makeTipChangeMotionTaskList();
    ui.pushButton_doTask_tipChanging->setEnabled(true);
    ui.pushButton_doTask_tipAttaching->setEnabled(true);
    ui.pushButton_doTask_tipDetaching->setEnabled(true);
#endif
}

void BinPickingDialog::pushButtonDoTipChangingTasksCallback() {
#if BIN_PICKING_FLAG
    if(ui.pushButton_doTask_tipChanging->isChecked()) {
        if(!qnode_->is_tip_attached) { // Tip attaching
            ROS_LOG_INFO("Tip Attaching Task!");
            qnode_->current_task_list_ = qnode_->task_planner_->module_ui_task_tip_changing_attach_motion_;
        } else  { // Tip detaching
            ROS_LOG_INFO("Tip Detaching Task!");
            qnode_->current_task_list_ = qnode_->task_planner_->module_ui_task_tip_changing_detach_motion_;
        }
        qnode_->beforeBinPickingTaskStart(true);
        qnode_->task_cycle_ = 0;
    } else {
        pushButtonSTOPAllCallback();
    }
#endif
}

void BinPickingDialog::pushButtonDoTipAttachingTasksCallback() {
#if BIN_PICKING_FLAG
    if(ui.pushButton_doTask_tipAttaching->isChecked()) {
        ROS_LOG_INFO("Tip Attaching Task!");
        qnode_->is_tip_attached = false;
        qnode_->current_task_list_ = qnode_->task_planner_->module_ui_task_tip_changing_attach_motion_;
        qnode_->beforeBinPickingTaskStart(true);
        qnode_->task_cycle_ = 0;
    } else {
        pushButtonSTOPAllCallback();
    }
#endif
}

void BinPickingDialog::pushButtonDoTipDetachingTasksCallback() {
#if BIN_PICKING_FLAG
    if(ui.pushButton_doTask_tipDetaching->isChecked()) {
        ROS_LOG_INFO("Tip Detaching Task!");
        qnode_->is_tip_attached = true;
        qnode_->current_task_list_ = qnode_->task_planner_->module_ui_task_tip_changing_detach_motion_;
        qnode_->beforeBinPickingTaskStart(true);
        qnode_->task_cycle_ = 0;
    } else {
        pushButtonSTOPAllCallback();
    }
#endif
}

//// 툴체인징 Pose 변환 기능
void BinPickingDialog::pushButtonSetToolChangingPoseSensorFrameCallback() {
#if BIN_PICKING_FLAG
    //// TODO: 22.04
    //// TODO: 22.04
    //// TODO: 22.04
    //// TODO: 22.04
    initial_stage_parameter_.target_id = taskTargetParts_[0]->text().toInt();

    initial_stage_parameter_.robot_dh_vec = qnode_->getRobotDHParameters();
    initial_stage_parameter_.robot_tcp_default = qnode_->getRobotDefaultTCP();
    initial_stage_parameter_.robot_tcp = qnode_->getRobotTCP();

    if(qnode_->m_bin_picking_node->doGetEnvParameters(initial_stage_parameter_))
    {
        qnode_->m_bin_picking_node->transformToolChangingPose2SensorPose(initial_stage_parameter_);
    }
    else
    {
        printf("pushButtonSetToolChangingPoseSensorFrameCallback --> Transforming Tool Changing Poses Process Failure!\n");
    }

#endif
}

void BinPickingDialog::pushButtonSetNewToolChangingPoseBaseFrameCallback() {
#if BIN_PICKING_FLAG
    //// TODO: 22.04
    //// TODO: 22.04
    //// TODO: 22.04
    //// TODO: 22.04
    initial_stage_parameter_.target_id = taskTargetParts_[0]->text().toInt();

    initial_stage_parameter_.robot_dh_vec = qnode_->getRobotDHParameters();
    initial_stage_parameter_.robot_tcp_default = qnode_->getRobotDefaultTCP();
    initial_stage_parameter_.robot_tcp = qnode_->getRobotTCP();

    if(qnode_->m_bin_picking_node->doGetEnvParameters(initial_stage_parameter_))
    {
        qnode_->m_bin_picking_node->transformToolChangingPose2NewBaseFrame(initial_stage_parameter_);
    }
    else
    {
        printf("pushButtonSetNewToolChangingPoseBaseFrameCallback --> Transforming Tool Changing Poses Process Failure!\n");
    }

#endif
}


//// Task Button
void BinPickingDialog::pushButtonGenerateTestTasksCallback() {
#if BIN_PICKING_FLAG
    setInitialTaskRobotParameters(); // Initialze task and parameters
    qnode_->task_planner_->makeBinPickingTestTaskList();
    ui.pushButton_doTask_test->setEnabled(true);
#endif
}

void BinPickingDialog::pushButtonDoTestTasksCallback() {
#if BIN_PICKING_FLAG

    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching = true; // matching service
    if(ui.pushButton_doTask_test->isChecked()) {
        ROS_LOG_INFO("# of Tasks: %zu", qnode_->task_planner_->module_bin_picking_test_.size());
        qnode_->current_task_list_ = qnode_->task_planner_->module_bin_picking_test_; // Task planner
        ROS_LOG_INFO("# of qnode_cal->current_task_list_: %zu", qnode_->current_task_list_.size());
        qnode_->beforeBinPickingTaskStart(true);
        qnode_->task_cycle_ = 0;
        ROS_LOG_INFO("# of qnode_cal->current_task_list_: %zu", qnode_->current_task_list_.size());

    } else {
        pushButtonSTOPAllCallback();
    }
#endif
}


void BinPickingDialog::setKORASGripperInitialValue()
{
#if BIN_PICKING_FLAG
    //// Gen. task
    setInitialTaskRobotParameters(); // Initialze task and parameters
    qnode_->task_planner_->makeKORASGripperSetInitialValueTask();
    //// Do task
    qnode_->current_task_list_ = qnode_->task_planner_->module_koras_gripper_set_initial_value_task_; // Task planner
    qnode_->beforeBinPickingTaskStart(true);
    qnode_->task_cycle_ = 0;
#endif
}

void BinPickingDialog::setInitialTaskRobotParameters()
{
#if BIN_PICKING_FLAG
    // qnode_->task_planner_->setPosePosition();
    //// UI vel. & acc. (task independent)
    qnode_->task_planner_->ref_unit_task_.vel_js = ui.task_doubleSpinBox_JSvelRobotA->value();
    qnode_->task_planner_->ref_unit_task_.acc_js = ui.task_doubleSpinBox_JSaccRobotA->value();
    qnode_->task_planner_->ref_unit_task_.vel_cs = ui.task_doubleSpinBox_CSvelRobotA->value();
    qnode_->task_planner_->ref_unit_task_.acc_cs = ui.task_doubleSpinBox_CSaccRobotA->value();

    //// Custom vel. & acc. (task dependent)
    qnode_->task_planner_->ref_unit_task_.vel_js_custom = ui.task_doubleSpinBox_JSvelRobotA->value();
    qnode_->task_planner_->ref_unit_task_.acc_js_custom = ui.task_doubleSpinBox_JSaccRobotA->value();
    qnode_->task_planner_->ref_unit_task_.vel_cs_custom = ui.task_doubleSpinBox_CSvelRobotA->value();
    qnode_->task_planner_->ref_unit_task_.acc_cs_custom = ui.task_doubleSpinBox_CSaccRobotA->value();

	std::vector<double> stiffness(CS_DOF);
	std::vector<double> nf(CS_DOF);
	std::vector<double> zeta(CS_DOF);
	std::vector<double> forceLimit(CS_DOF);
	std::vector<double> damping(CS_DOF);
	std::vector<double> mass(CS_DOF);
    for (std::size_t i = 0; i < CS_DOF; i++) {
        stiffness [i] = taskStiffness_ [i]->text().toDouble();
        nf        [i] = taskNf_        [i]->text().toDouble();
        zeta      [i] = taskZeta_      [i]->text().toDouble();
        forceLimit[i] = taskForceLimit_[i]->text().toDouble();
        damping[i] = 2.0 * stiffness[i] * zeta[i] / nf[i];
        mass[i] = stiffness[i] / nf[i] / nf[i];

        //// update
        qnode_->task_planner_->impedance_default_.m [i]              = mass      [i];
        qnode_->task_planner_->impedance_default_.b [i]              = damping   [i];
        qnode_->task_planner_->impedance_default_.k [i]              = stiffness [i];
        qnode_->task_planner_->impedance_default_.force_limit[i]     = forceLimit[i];
        // qnode_->task_planner_->impedance_default_.force_selection[i] = list_impedance_selection_[i]->isChecked();
        qnode_->task_planner_->impedance_default_.force_selection[i] = true;

        qnode_->task_planner_->ref_unit_task_.bp_param.assemble.target_force [i] = taskTargetForce_ [i]->text().toDouble();
        qnode_->task_planner_->ref_unit_task_.bp_param.assemble.contact_force[i] = taskContactForce_[i]->text().toDouble();
    }
    for (std::size_t i = 0; i < 3; i++) {
        qnode_->task_planner_->ref_unit_task_.bp_param.assemble.move_vel   [i] = ui.task_doubleSpinBox_CSvelRobotA->value();
        qnode_->task_planner_->ref_unit_task_.bp_param.assemble.contact_vel[i] = ui.task_doubleSpinBox_CSvelRobotA->value()/6.0;
        qnode_->task_planner_->ref_unit_task_.bp_param.assemble.insert_vel [i] = ui.task_doubleSpinBox_CSvelRobotA->value()/6.0;

        qnode_->task_planner_->ref_unit_task_.bp_param.assemble.move_vel   [i + 3] = 20.0;
        qnode_->task_planner_->ref_unit_task_.bp_param.assemble.contact_vel[i + 3] = 2.0;
        qnode_->task_planner_->ref_unit_task_.bp_param.assemble.insert_vel [i + 3] = 10.0;
    }

    ROS_LOG_INFO("Task list generation! - Bin-picking task initializing and parameter setting");
    ROS_LOG_INFO("forceLimit: %f, %f, %f, %f, %f, %f", qnode_->task_planner_->impedance_default_.force_limit[0], qnode_->task_planner_->impedance_default_.force_limit[1], qnode_->task_planner_->impedance_default_.force_limit[2], qnode_->task_planner_->impedance_default_.force_limit[3], qnode_->task_planner_->impedance_default_.force_limit[4], qnode_->task_planner_->impedance_default_.force_limit[5]);
    ROS_LOG_INFO("stiffness: %f, %f, %f, %f, %f, %f", qnode_->task_planner_->impedance_default_.m[0], qnode_->task_planner_->impedance_default_.m[1], qnode_->task_planner_->impedance_default_.m[2], qnode_->task_planner_->impedance_default_.m[3], qnode_->task_planner_->impedance_default_.m[4], qnode_->task_planner_->impedance_default_.m[5]);
    ROS_LOG_INFO("move velocity: %f, %f, %f, %f, %f, %f", qnode_->task_planner_->ref_unit_task_.bp_param.assemble.move_vel[0], qnode_->task_planner_->ref_unit_task_.bp_param.assemble.move_vel[1], qnode_->task_planner_->ref_unit_task_.bp_param.assemble.move_vel[2], qnode_->task_planner_->ref_unit_task_.bp_param.assemble.move_vel[3], qnode_->task_planner_->ref_unit_task_.bp_param.assemble.move_vel[4],qnode_->task_planner_->ref_unit_task_.bp_param.assemble.move_vel[5]);
    ROS_LOG_INFO("insert velocity: %f, %f, %f, %f, %f, %f", qnode_->task_planner_->ref_unit_task_.bp_param.assemble.insert_vel[0], qnode_->task_planner_->ref_unit_task_.bp_param.assemble.insert_vel[1], qnode_->task_planner_->ref_unit_task_.bp_param.assemble.insert_vel[2], qnode_->task_planner_->ref_unit_task_.bp_param.assemble.insert_vel[3], qnode_->task_planner_->ref_unit_task_.bp_param.assemble.insert_vel[4],qnode_->task_planner_->ref_unit_task_.bp_param.assemble.insert_vel[5]);

#endif
}

void BinPickingDialog::changeRobotTCP(unsigned int tcp_idx) {
#if BIN_PICKING_FLAG
    qnode_->sendChangeRobotTCPCommand(tcp_idx);
#endif
}

void BinPickingDialog::pushButtonTestFunctionCallback() {
#if BIN_PICKING_FLAG
    qnode_->test_cnt_++;
    ROS_LOG_WARN("Stacking Test Trial #%u", qnode_->test_cnt_);

    // std::vector<double> pose_in = {0.89144, 0.43383, 0.05854, -179.786, -0.135, 179.575};
    // std::vector<double> pose_in = {0.85772, 0.59973, 0.10510, -177.501, 44.579, -179.298};
    // std::vector<double> pose_in = {0.61315, 0.17036, 0.12562, 179.793, -0.058, -135.002};
    std::vector<double> pose_in = {0.87647, 0.31923, 0.06501, 179.882, -0.223, -88.979};

    for(int i=0; i<qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->max_cnt_detaching_; i++) {

        //// generateStackingPose
        std::vector<double> pose_now = pose_in;
        qnode_->m_bin_picking_node->generateStackingPose(pose_now, qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->detaching_cnt_, qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->stacking_trans_scale_, qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->stacking_single_stack_num_, qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->stacking_line_stack_num_, qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->stacking_mode_);
        qnode_->m_bin_picking_node->is_detaching_stack_pose_assigned = false;

        //// TASK_DETACH_COUNTING
        if(qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->detaching_cnt_ < qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->max_cnt_detaching_ - 1) {
            qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->detaching_cnt_++;
        } else {
            qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->detaching_cnt_ = 0;
            qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->stacking_z_idx_ = 0;
            ROS_LOG_WARN("Stacking Test Trial #%u Finished!!!\n\n", qnode_->test_cnt_);
        }
    }
#endif
}

//// Q Slot Func.
void BinPickingDialog::setCustomTrasnformationPose(std::vector<double> &value) {
#if BIN_PICKING_FLAG
    //// TODO: 22.04
    //// TODO: 22.04
    //// TODO: 22.04
    //// TODO: 22.04
    // ROS_LOG_INFO("value: %0.5f, %0.5f, %0.5f, %0.3f, %0.3f, %0.3f", value[0], value[1], value[2], value[3], value[4], value[5]);
    //// Scanning
    // pushButtonDoScanningZIVIDClickedCallback();
    //// Matching
    // taskTemplateMatchingParameter matching_parameter;

    initial_stage_parameter_.debug_mode = true;
    initial_stage_parameter_.custom_transformation_pose = value;
    initial_stage_parameter_.do_scan_sampling = true;
    initial_stage_parameter_.do_scan_sampling = ui.radioButton_assemblyUI_scanSampling->isChecked();
    initial_stage_parameter_.sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();

    // Target object
    initial_stage_parameter_.target_id = taskTargetParts_[0]->text().toInt();
    initial_stage_parameter_.target_name = taskTargetParts_[1]->text().toStdString();

    initial_stage_parameter_.robot_dh_vec = qnode_->getRobotDHParameters();
    initial_stage_parameter_.robot_tcp_default = qnode_->getRobotDefaultTCP();
    initial_stage_parameter_.robot_tcp = qnode_->getRobotTCP();

    arr2Vec(qnode_->params_.meas.q , initial_stage_parameter_.robot_camera_calibration_js_position); // scanning joint position
    std::vector<double> scan_position_q = initial_stage_parameter_.robot_camera_calibration_js_position;
    ROS_LOG_WARN("Current JS position [deg]: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", scan_position_q[0], scan_position_q[1], scan_position_q[2], scan_position_q[3], scan_position_q[4], scan_position_q[5]);


    initial_stage_parameter_.do_cloud_clear = false;

    std::stringstream ss;
    ss.str("");

    if(ui.pushButton_doBinMatching->isChecked()) {

        if(bin_matching_dialog->ui.radioButton_matching_bin->isChecked()) {
            qnode_->m_bin_picking_node->doBinDetectionTemplateMatching(initial_stage_parameter_);
            ss << "Bin matching accuracy: " << 0.1*floor(1000.0*initial_stage_parameter_.matching_accuracy) << "%" << std::endl;
        } else if(bin_matching_dialog->ui.radioButton_matching_base_zig->isChecked()) {
            qnode_->m_bin_picking_node->doBaseZigDetectionTemplateMatching(initial_stage_parameter_);
            ss << "Base zig matching accuracy: " << 0.1*floor(1000.0*initial_stage_parameter_.matching_accuracy) << "%" << std::endl;
        } else if(bin_matching_dialog->ui.radioButton_matching_eyeInHand->isChecked()) {

            changeRobotTCP(7); // default TCP #7
            arr2Vec(qnode_->params_.meas.x, initial_stage_parameter_.pose_B2E_measured);
            std::vector<double> tmp = initial_stage_parameter_.pose_B2E_measured;
            ROS_LOG_INFO("initial_stage_parameter_.pose_B2E_measured: %0.5f, %0.5f, %0.5f, %0.3f, %0.3f, %0.3f", tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]);

            qnode_->m_bin_picking_node->doEyeInHandDetectionTemplateMatching(initial_stage_parameter_);
            ss << "Eye-in-hand detection matching accuracy: " << 0.1*floor(1000.0*initial_stage_parameter_.matching_accuracy) << "%" << std::endl;
        }

    } else if(ui.pushButton_doRobot2CameraCalibration->isChecked()) {
        if(0) { // 16.04
            qnode_->m_bin_picking_node->getCSPoseWithDefaultTCPCommandUsingCurrentJS(7, initial_stage_parameter_.pose_B2E_measured); // default tcp idx: 7, target JS position
        } else { // new
            changeRobotTCP(7); // default TCP #7
            arr2Vec(qnode_->params_.meas.x, initial_stage_parameter_.pose_B2E_measured);
            std::vector<double> tmp = initial_stage_parameter_.pose_B2E_measured;
            ROS_LOG_INFO("initial_stage_parameter_.pose_B2E_measured: %0.5f, %0.5f, %0.5f, %0.3f, %0.3f, %0.3f", tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]);
        }
        qnode_->m_bin_picking_node->doRobot2CameraCalibrationTemplateMatching(initial_stage_parameter_);
        ss << "Robot2Camera calibration matching accuracy: " << 0.1*floor(1000.0*initial_stage_parameter_.matching_accuracy) << "%" << std::endl;
    } else if(ui.pushButton_doEvaluateGraspingPose->isChecked()) {
        qnode_->m_bin_picking_node->getCSPoseWithDefaultTCPCommandUsingCurrentJS(7, initial_stage_parameter_.pose_B2E_measured); // default tcp idx: 7, target JS position
        qnode_->m_bin_picking_node->doEvaluateGraspingPose(initial_stage_parameter_);
        ss << "pushButton_doEvaluateGraspingPose: " << std::endl;
    } else if(ui.pushButton_doSimulationBinPlacing->isChecked()) {
        qnode_->m_bin_picking_node->doSimulationBinPlacing(initial_stage_parameter_);
        ss << "doSimulationBinPlacing: " << std::endl;
        return;
    }

    QMessageBox msgBox;
    size_t cnt = 0;
    bool is_bin_matching_success = false;

    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Ok);
    msgBox.setStyleSheet(
        "   QLabel {"
        "font: bold 24px;"
        "}"
        "QPushButton {"
        "font: bold 36px;"
        "}"
    );

    msgBox.setWindowTitle(" ");
    msgBox.setIcon(QMessageBox::Warning);
    msgBox.setText("Click OK when matching is successful");
    msgBox.setInformativeText(ss.str().c_str());

    int ret = msgBox.exec();
    switch(ret)
    {
        case QMessageBox::Ok:
            is_bin_matching_success = true;
            ROS_LOG_INFO("Matching is successful!");
        break;

        case QMessageBox::Cancel:
            is_bin_matching_success = false;
            ROS_LOG_INFO("Matching is failure!");
        break;
    }

    if(is_bin_matching_success)
    {
        bin_matching_dialog->close();
        robot2camera_calibration_dialog->close();
        ui.pushButton_doBinMatching->setChecked(false);
        ui.pushButton_doRobot2CameraCalibration->setChecked(false);
        ui.pushButton_doEvaluateGraspingPose->setChecked(false);
        ui.pushButton_doSimulationBinPlacing->setChecked(false);
    }
#endif
}

void BinPickingDialog::slot_doClearCloud(bool do_clear) {
#if BIN_PICKING_FLAG
    std::vector<double> value(6, 0.0);
    initial_stage_parameter_.debug_mode = true;
    initial_stage_parameter_.custom_transformation_pose = value;
    initial_stage_parameter_.do_scan_sampling = true;
    initial_stage_parameter_.do_scan_sampling = ui.radioButton_assemblyUI_scanSampling->isChecked();
    initial_stage_parameter_.sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();

    // Target object
    initial_stage_parameter_.target_id = taskTargetParts_[0]->text().toInt();
    initial_stage_parameter_.target_name = taskTargetParts_[1]->text().toStdString();

    initial_stage_parameter_.robot_dh_vec = qnode_->getRobotDHParameters();
    initial_stage_parameter_.robot_tcp_default = qnode_->getRobotDefaultTCP();
    initial_stage_parameter_.robot_tcp = qnode_->getRobotTCP();

    initial_stage_parameter_.do_cloud_clear = do_clear;

    if(ui.pushButton_doSimulationBinPlacing->isChecked())
    {
        qnode_->m_bin_picking_node->doSimulationBinPlacing(initial_stage_parameter_);
        return;
    }
#endif
}


void BinPickingDialog::slot_getRobot2CameraCalibrationJSPosition(std::vector<double> &js_position, bool do_teaching) {
#if BIN_PICKING_FLAG
    initial_stage_parameter_.debug_mode = true;
    // initial_stage_parameter_.custom_transformation_pose = js_position;
    initial_stage_parameter_.do_scan_sampling = true;
    initial_stage_parameter_.do_scan_sampling = ui.radioButton_assemblyUI_scanSampling->isChecked();
    initial_stage_parameter_.sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();

    // Target object
    initial_stage_parameter_.target_id = taskTargetParts_[0]->text().toInt();
    initial_stage_parameter_.target_name = taskTargetParts_[1]->text().toStdString();

    initial_stage_parameter_.robot_dh_vec = qnode_->getRobotDHParameters();
    initial_stage_parameter_.robot_tcp_default = qnode_->getRobotDefaultTCP();
    initial_stage_parameter_.robot_tcp = qnode_->getRobotTCP();

    initial_stage_parameter_.is_r2cam_calibration_js_position_set_by_teaching = do_teaching;
    if(do_teaching) {
        initial_stage_parameter_.robot_camera_calibration_js_position = js_position;
    }

    std::stringstream ss;
    ss.str("");
    qnode_->m_bin_picking_node->doGetRobot2CameraCalibrationJSPosition(initial_stage_parameter_);
    // // qnode_->m_bin_picking_node->getCSPoseWithDefaultTCPCommand(7, initial_stage_parameter_.robot_camera_calibration_js_position, initial_stage_parameter_.pose_B2E_measured); // default tcp idx: 7, target JS position
    // qnode_->m_bin_picking_node->getCSPoseWithDefaultTCPCommandUsingCurrentJS(7, initial_stage_parameter_.robot_camera_calibration_js_position, initial_stage_parameter_.pose_B2E_measured); // default tcp idx: 7, target JS position

    if(!do_teaching) {
        js_position = initial_stage_parameter_.robot_camera_calibration_js_position;
    }
#endif
}


void BinPickingDialog::slot_JSMove(std::vector<double> &target_js_position) {
#if BIN_PICKING_FLAG
    std::vector<double> jointPosition = target_js_position; // [deg]
    qnode_->m_bin_picking_node->printVector(jointPosition, "jointPosition");
    double qd  = ui.task_doubleSpinBox_JSvelRobotA->value();
    double qdd = ui.task_doubleSpinBox_JSaccRobotA->value();
    bool is_relative = false;
    JsDouble q;
    for (std::size_t i = 0; i < jointPosition.size(); i++) {
        q[i] = jointPosition[i];
    }
    qnode_->moveQ(q, qd, qdd, is_relative);
    // qnode_->setTargetJointPosition(jointPosition, maxVelocity, acceleration, isrelative, true);
#endif
}

void BinPickingDialog::slot_STOPAll(){
#if BIN_PICKING_FLAG
    qnode_->is_task_mode_ = false;
    qnode_->stopRobot();
#endif
}

void BinPickingDialog::slot_doScanningInitialStage(size_t sampling_num, bool skip_detection_mask, std::string target_name) {
#if BIN_PICKING_FLAG
    taskScanningParameter scan_parameter;
    scan_parameter.target_id = taskTargetParts_[0]->text().toInt();
    // scan_parameter.target_name = taskTargetParts_[1]->text().toStdString();
    scan_parameter.target_name = target_name;
    scan_parameter.is_mask_pixel_fixed = ui.radioButton_assemblyUI_isMaskFixed->isChecked();
    std::vector<int> mask_pixel_list(4,10);
    mask_pixel_list[0] = fixedMaskPixelList_[0]->text().toInt();
    mask_pixel_list[1] = fixedMaskPixelList_[1]->text().toInt();
    mask_pixel_list[2] = fixedMaskPixelList_[2]->text().toInt();
    mask_pixel_list[3] = fixedMaskPixelList_[3]->text().toInt();
    scan_parameter.mask_pixel_list = mask_pixel_list;
    scan_parameter.sampling_num = sampling_num;
    scan_parameter.is_base_frame_unknown = true;
    scan_parameter.do_scan_sampling = ui.radioButton_assemblyUI_scanSampling->isChecked();
    scan_parameter.do_save_data = ui.radioButton_assemblyUI_scanDataSave->isChecked();
    scan_parameter.do_image_processing = ui.radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    scan_parameter.do_single_matching = true; // matching service

    scan_parameter.do_not_scan_do_load_data = ui.radioButton_assemblyUI_scanDataLoad->isChecked();
    scan_parameter.skip_detection_mask = skip_detection_mask;


    //// 241125 추가
    scan_parameter.robot_dh_vec = qnode_->getRobotDHParameters(); // [m], [deg]
    scan_parameter.robot_tcp_default = qnode_->getRobotDefaultTCP(); // [m], [deg]
    scan_parameter.robot_tcp = qnode_->getRobotTCP(); // [m], [deg]

    arr2Vec(qnode_->params_.meas.q , scan_parameter.scan_position); // scanning joint position
    std::vector<double> scan_position_q = scan_parameter.scan_position;
    ROS_LOG_WARN("Current JS position [deg]: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", scan_position_q[0], scan_position_q[1], scan_position_q[2], scan_position_q[3], scan_position_q[4], scan_position_q[5]);


    // if(scan_parameter.target_id >= 31 && scan_parameter.target_id < 45) // bin picking(31~36)
    // {
    //     qnode_->m_bin_picking_node->scanZIVID(scan_parameter);
    // }
    // else
    // {
    //     ROS_LOG_INFO("Wrong target id!");
    // }
    qnode_->m_bin_picking_node->scanZIVID(scan_parameter);
#endif
}





