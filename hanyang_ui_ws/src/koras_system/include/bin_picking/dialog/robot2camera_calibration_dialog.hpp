#ifndef ROBOT2CAMERA_CALIBRATION_DIALOG_HPP
#define ROBOT2CAMERA_CALIBRATION_DIALOG_HPP

#include <QDialog>
#include "ui_robot2camera_calibration_dialog.h"

class Robot2CameraCalibrationDialog : public QDialog
{
    Q_OBJECT

Q_SIGNALS:
    void setCustomTrasnformationPose(std::vector<double> &data);
    void slot_getRobot2CameraCalibrationJSPosition(std::vector<double> &js_position, bool do_teaching);
    void slot_JSMove(std::vector<double> &target_js_position);
    void slot_STOPAll();
    void slot_doScanningInitialStage(size_t sampling_num, bool skip_detection_mask, std::string target_name);

public:
    Robot2CameraCalibrationDialog(QWidget *parent = 0);
    ~Robot2CameraCalibrationDialog();

public Q_SLOTS:

    void setInitListValue();

    void pushButtonCustomMatchingTransformationValueClearClickedCallback();
    void pushButtonSetCustomMatchingTransformationValueClickedCallback();
    void pushButtonGetRobot2CameraCalibrationJSPositionClickedCallback();
    void pushButtonJSMoveClickedCallback();
    void pushButtonSTOPAllCallback();
    void pushButtonDoScanningZIVIDClickedCallback();


private:
    Ui::Robot2CameraCalibrationDialog ui;

    QList <QDoubleSpinBox *> customMatchingTransformationPoseList_;
    QList <QLineEdit *> robotATargetQList_;

};

#endif // ROBOT2CAMERA_CALIBRATION_DIALOG_HPP
