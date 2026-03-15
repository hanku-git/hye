#ifndef LLM_DIALOG_HPP
#define LLM_DIALOG_HPP

#include <iostream>
#include <QDialog>
#include <QtCore/QTimer>
#include <QMainWindow>
#include <QFileDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QList>
#include <QString>
#include <QStringList>

#include "qnode.hpp"
#include "bin_picking_parameter.hpp"

using namespace std;

QT_BEGIN_NAMESPACE
namespace Ui { class llm_dialog; }
QT_END_NAMESPACE

class llm_dialog : public QDialog
{
    Q_OBJECT

public:
    llm_dialog(QMainWindow *parent = nullptr, QNode *qnode_ = nullptr);
    ~llm_dialog();

    bool is_placement_task = false;
    int current_gripper = 1; // 현재 장착된 그리퍼 (기본값: 2지 그리퍼)
    int current_tip = 1; // default: regular tip
    int current_obj = 36; // striker
    string target_name = "peg";
    string llm_text;

    int tool_change_vel;
    int grasping_vel;
    int passing_vel;
    int loading_vel;

    JsDouble grp_home_pose_;
    JsDouble home_pose_;
    JsDouble mid_pose_;
    JsDouble scan_pose_;

public Q_SLOTS: // 슬롯을 정의하는 영역
    void updateLLMText(const QString &target, const QString &goal, const QString &passes);
    void UpdateVelocites(const QString &velocities);
private Q_SLOTS:
    void timerCallback();
    void TargetObjectParamSetting();
    void RecordingCallback();
    void SetPoseCallback(const std::string& position);
    void GraspingTargetObject();
    void GripperAttachCallback();
    void GripperDetachCallback();
    void llm_node_Callback(LLM_PARAM &llm_param);
    void target_obj_Callback();
    void BoxSelectionCallback();
    void onUpdateImage(const QImage &image, const QString &current_pass);
    void onUpdate3DCoordinates(const QString &coordinates, const QString &current_pass);

    // 기존 이미지 및 좌표 업데이트 슬롯 제거
private:
    Ui::llm_dialog *llm_ui;
    QNode *qnode_llm;
    QTimer *timer_;

    QList <QRadioButton *> target_object_list;
    QList <QLineEdit *> waypoint_list;
    QList <QLineEdit *> pass_list;
};
#endif // LLM_DIALOG_HPP