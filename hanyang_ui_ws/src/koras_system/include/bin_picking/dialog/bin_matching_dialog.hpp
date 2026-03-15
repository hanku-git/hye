#ifndef BIN_MATCHING_DIALOG_HPP
#define BIN_MATCHING_DIALOG_HPP

#include <QDialog>
#include "ui_bin_matching_dialog.h"

class BinMatchingDialog : public QDialog
{
    Q_OBJECT

Q_SIGNALS:
    void setCustomTrasnformationPose(std::vector<double> &data);
    void slot_doClearCloud(bool do_clear);
    void slot_doScanningInitialStage(size_t sampling_num, bool skip_detection_mask, std::string target_name);

public:
    BinMatchingDialog(QWidget *parent = 0);
    ~BinMatchingDialog();

public:
    Ui::BinMatchingDialog ui;


public Q_SLOTS:

    void pushButtonSetInitialTransformationValue();
    void pushButtonCustomMatchingTransformationValueClearClickedCallback();
    void pushButtonSetCustomMatchingTransformationValueClickedCallback();
    void pushButtonDoInitializeCloudClickedCallback();
    void pushButtonDoScanningZIVIDClickedCallback();

private:

    QList <QDoubleSpinBox *> customMatchingTransformationPoseList_;

};

#endif // BIN_MATCHING_DIALOG_HPP
