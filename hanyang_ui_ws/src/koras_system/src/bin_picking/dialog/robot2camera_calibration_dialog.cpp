#include "bin_picking/dialog/robot2camera_calibration_dialog.hpp"

Robot2CameraCalibrationDialog::Robot2CameraCalibrationDialog(QWidget *parent) :
    QDialog(parent)
{
    ui.setupUi(this);

    QObject::connect(ui.pushButton_customMatchingTransformationValueClear     , SIGNAL(clicked()), this, SLOT(pushButtonCustomMatchingTransformationValueClearClickedCallback()));    
    QObject::connect(ui.pushButton_setCustomMatchingTransformationValue     , SIGNAL(clicked()), this, SLOT(pushButtonSetCustomMatchingTransformationValueClickedCallback()));    
    QObject::connect(ui.pushButton_getRobot2CameraCalibrationJSPosition     , SIGNAL(clicked()), this, SLOT(pushButtonGetRobot2CameraCalibrationJSPositionClickedCallback()));    
    QObject::connect(ui.pushButton_jsMove_robotA                            , SIGNAL(clicked()), this, SLOT(pushButtonJSMoveClickedCallback()));
    QObject::connect(ui.pushButton_STOPAll                                  , SIGNAL(clicked()), this, SLOT(pushButtonSTOPAllCallback()));
    QObject::connect(ui.pushButton_doScan_ZIVID                             , SIGNAL(clicked()), this, SLOT(pushButtonDoScanningZIVIDClickedCallback()));

    customMatchingTransformationPoseList_ << ui.task_doubleSpinBox_customMatchingTransformation_1
                                            << ui.task_doubleSpinBox_customMatchingTransformation_2
                                            << ui.task_doubleSpinBox_customMatchingTransformation_3
                                            << ui.task_doubleSpinBox_customMatchingTransformation_4
                                            << ui.task_doubleSpinBox_customMatchingTransformation_5
                                            << ui.task_doubleSpinBox_customMatchingTransformation_6;


    robotATargetQList_ << ui.lineEdit_robotATargetQ_1
                       << ui.lineEdit_robotATargetQ_2
                       << ui.lineEdit_robotATargetQ_3
                       << ui.lineEdit_robotATargetQ_4
                       << ui.lineEdit_robotATargetQ_5
                       << ui.lineEdit_robotATargetQ_6;


    setInitListValue();
}

Robot2CameraCalibrationDialog::~Robot2CameraCalibrationDialog()
{
    // delete ui;
}

void Robot2CameraCalibrationDialog::setInitListValue() {

    // Initial JS pose
    // std::vector<double> starting_pose_q = { 226.532, -109.569, 141.020, -168.260, -64.239, -119.084 };
    std::vector<double> starting_pose_q = { 232.768, -99.550, 153.553, -157.855, -32.238, -17.980 }; // ur10e rvbust test
    for (std::size_t i = 0; i < starting_pose_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(starting_pose_q[i], 'f', 3));
    }
}

void Robot2CameraCalibrationDialog::pushButtonCustomMatchingTransformationValueClearClickedCallback()
{
    std::vector<double> custom_transformation_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < customMatchingTransformationPoseList_.size(); i++) { customMatchingTransformationPoseList_[i]->setValue(custom_transformation_pose[i]); }
}

void Robot2CameraCalibrationDialog::pushButtonSetCustomMatchingTransformationValueClickedCallback()
{
    std::vector<double> custom_transformation_pose;
    for (std::size_t i = 0; i < customMatchingTransformationPoseList_.size(); i++) { custom_transformation_pose.push_back(customMatchingTransformationPoseList_[i]->value()); }
    for(int i = 0; i < 3; i++) custom_transformation_pose[i] = 0.001*custom_transformation_pose[i]; // [m]
    emit setCustomTrasnformationPose(custom_transformation_pose);
}

void Robot2CameraCalibrationDialog::pushButtonGetRobot2CameraCalibrationJSPositionClickedCallback()
{
    std::vector<double> robot_camera_calibration_js_position(6);
    bool do_teaching = ui.radioButton_robot2SensorCal_doTeaching->isChecked();
    if(do_teaching) {
        printf("Do teaching JS position for robot camera calibration\n");
        for (std::size_t i = 0; i < 6; i++) {
            robot_camera_calibration_js_position[i] = robotATargetQList_[i]->text().toDouble();
        }
    } else {
        printf("Automatic generating JS position for robot camera calibration\n");
    }
    emit slot_getRobot2CameraCalibrationJSPosition(robot_camera_calibration_js_position, do_teaching);
    if(!do_teaching) {
        for (std::size_t i = 0; i < robot_camera_calibration_js_position.size(); i++) {
            robotATargetQList_[i]->setText(QString::number(robot_camera_calibration_js_position[i], 'f', 3));
        }
    }
}

/** @brief Korean: UI 버튼함수, 관절공간 상의 목표 관절각도로 로봇을 이동한다.
 */
void Robot2CameraCalibrationDialog::pushButtonJSMoveClickedCallback() 
{
    std::vector<double> jointPosition;
    for (std::size_t i = 0; i < robotATargetQList_.size(); i++) {
        jointPosition.push_back(robotATargetQList_[i]->text().toDouble());
    }
    emit slot_JSMove(jointPosition);
}

void Robot2CameraCalibrationDialog::pushButtonSTOPAllCallback()
{
    printf("STOP ALL!\n");
    emit slot_STOPAll();
}

void Robot2CameraCalibrationDialog::pushButtonDoScanningZIVIDClickedCallback()
{
    size_t sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();
    std::string target_name = "cal_tool";
    emit slot_doScanningInitialStage(sampling_num, true, target_name);
}

