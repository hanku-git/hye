#include "bin_picking/dialog/bin_matching_dialog.hpp"

BinMatchingDialog::BinMatchingDialog(QWidget *parent) :
    QDialog(parent)
{
    ui.setupUi(this);

    QObject::connect(ui.pushButton_customMatchingTransformationValueClear     , SIGNAL(clicked()), this, SLOT(pushButtonCustomMatchingTransformationValueClearClickedCallback()));    
    QObject::connect(ui.pushButton_setCustomMatchingTransformationValue     , SIGNAL(clicked()), this, SLOT(pushButtonSetCustomMatchingTransformationValueClickedCallback()));    
    QObject::connect(ui.pushButton_doInitializeCloud     , SIGNAL(clicked()), this, SLOT(pushButtonDoInitializeCloudClickedCallback()));    
    QObject::connect(ui.pushButton_doScan_ZIVID                             , SIGNAL(clicked()), this, SLOT(pushButtonDoScanningZIVIDClickedCallback()));

    QObject::connect(ui.radioButton_matching_bin   , SIGNAL(clicked()), this, SLOT(pushButtonSetInitialTransformationValue()));
    QObject::connect(ui.radioButton_matching_base_zig   , SIGNAL(clicked()), this, SLOT(pushButtonSetInitialTransformationValue()));
    QObject::connect(ui.radioButton_matching_eyeInHand   , SIGNAL(clicked()), this, SLOT(pushButtonSetInitialTransformationValue()));


    customMatchingTransformationPoseList_ << ui.task_doubleSpinBox_customMatchingTransformation_1
                                            << ui.task_doubleSpinBox_customMatchingTransformation_2
                                            << ui.task_doubleSpinBox_customMatchingTransformation_3
                                            << ui.task_doubleSpinBox_customMatchingTransformation_4
                                            << ui.task_doubleSpinBox_customMatchingTransformation_5
                                            << ui.task_doubleSpinBox_customMatchingTransformation_6;


}

BinMatchingDialog::~BinMatchingDialog()
{
    // delete ui;
}



void BinMatchingDialog::pushButtonSetInitialTransformationValue()
{
    std::vector<double> custom_transformation_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if(ui.radioButton_matching_bin->isChecked()) {

    } else if(ui.radioButton_matching_base_zig->isChecked()) {

    } else if(ui.radioButton_matching_eyeInHand->isChecked()) {
        // //// 241125, rvbust with FR
        // custom_transformation_pose[0] = -40.0;
        // custom_transformation_pose[1] = 100.0;
        // custom_transformation_pose[2] = -60.0;

        //// 250224, zivid2+mr60
        custom_transformation_pose[0] = -60.0;
        custom_transformation_pose[1] = 25.0;
        custom_transformation_pose[2] = -150.0;
    }
    for (std::size_t i = 0; i < customMatchingTransformationPoseList_.size(); i++) { customMatchingTransformationPoseList_[i]->setValue(custom_transformation_pose[i]); }
}


void BinMatchingDialog::pushButtonCustomMatchingTransformationValueClearClickedCallback()
{
    std::vector<double> custom_transformation_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < customMatchingTransformationPoseList_.size(); i++) { customMatchingTransformationPoseList_[i]->setValue(custom_transformation_pose[i]); }
}

void BinMatchingDialog::pushButtonSetCustomMatchingTransformationValueClickedCallback()
{
    std::vector<double> custom_transformation_pose;
    for (std::size_t i = 0; i < customMatchingTransformationPoseList_.size(); i++) { custom_transformation_pose.push_back(customMatchingTransformationPoseList_[i]->value()); }
    for(int i = 0; i < 3; i++) custom_transformation_pose[i] = 0.001*custom_transformation_pose[i]; // [m]
    emit setCustomTrasnformationPose(custom_transformation_pose);
}

void BinMatchingDialog::pushButtonDoInitializeCloudClickedCallback()
{
    emit slot_doClearCloud(true);
}

void BinMatchingDialog::pushButtonDoScanningZIVIDClickedCallback()
{
    size_t sampling_num = ui.lineEdit_scanSamplingNum->text().toInt();
    std::string target_name = "";
    if(ui.radioButton_matching_bin->isChecked()) {
        target_name = "bin";
    } else if(ui.radioButton_matching_base_zig->isChecked()) {
        target_name = "base_zig";
    } else if(ui.radioButton_matching_eyeInHand->isChecked()) {
        target_name = "eye_in_hand";
    }
    
    emit slot_doScanningInitialStage(sampling_num, true, target_name);
}


