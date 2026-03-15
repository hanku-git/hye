#ifndef MAINWINDOW_INITIAL_SETUP_H
#define MAINWINDOW_INITIAL_SETUP_H

#include <QMainWindow>
#include <QLabel>
#include <QSlider>
#include <QWidget>
#include <QGridLayout>
#include <QProcess>
#include <QCloseEvent>

#include <QFile>
#include <QTextStream>
#include <QImage>
#include <thread>
#include "qnode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "task_parameter.hpp"
#include "task_planner_default.hpp"
#include <bin_picking/nlohmann/json.hpp>
#include <functional>
#include <map>

nlohmann::json array_to_json(const std::array<double, 6>& vec);
JsDouble parseJsDouble(const std::string& str);
CsDouble parseCsDouble(const std::string& str);
nlohmann::json params_to_json(const _unitTask& task);
nlohmann::json vector_to_json(const CsDouble& vec);
std::vector<double> parseVectorDouble(const nlohmann::json& json_array) ;


using json = nlohmann::json;
// 함수 포인터 타입 정의
using TaskPushBackFunction = std::function<void(std::vector<UnitTask>&, const nlohmann::json&)>;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow_initialSetup; }
QT_END_NAMESPACE

// const double pi = 3.14159265358979;

class MainWindow_initialSetup : public QMainWindow
{
    Q_OBJECT

Q_SIGNALS:
    void closeDeveloperWindow();
    void openTaskGeneratorWindow();

public:
    MainWindow_initialSetup(QNode* qnode, QWidget *parent = 0);
    ~MainWindow_initialSetup();
    void initializeTaskMap();

    JsDouble home_pose_;
    JsDouble mid_pose_;
    JsDouble mid_2f_pose_;
    JsDouble mid_tag_pose_;

// struct TaskModeData {
//     std::string name;
//     std::function<void(std::vector<UnitTask>&, nlohmann::json&)> handler;

// };
// struct TaskModeData {
//     std::string name;
//     void (TaskPlannerDefault::*handler)(std::vector<UnitTask>&, nlohmann::json&);
// };
struct TaskModeData {
    std::string name;
    std::function<void(std::vector<UnitTask>&, nlohmann::json&)> handler;

    // 생성자 추가
    TaskModeData(const std::string& n, const std::function<void(std::vector<UnitTask>&, nlohmann::json&)>& h)
        : name(n), handler(h) {}

    // 기본 생성자
    TaskModeData() = default;

    // 복사 생성자와 할당 연산자를 기본 제공
    TaskModeData(const TaskModeData&) = default;
    TaskModeData& operator=(const TaskModeData&) = default;
};
private:
    Ui::MainWindow_initialSetup *ui;

    QList <QLineEdit *> robotAXList_;
    QList <QLineEdit *> robotAQList_;
    QList <QLineEdit *> robotATargetXList_;
    QList <QLineEdit *> robotBTargetXList_;
    QList <QLineEdit *> robotATargetQList_;
    QList <QLineEdit *> robotBTargetQList_;

    QList<QPushButton*> list_jog_q_plus_;
    QList<QPushButton*> list_jog_q_minus_;
    QList<QPushButton*> list_jog_x_plus_;
    QList<QPushButton*> list_jog_x_minus_;

    QList<QPushButton*> list_step_move_q_plus_;
    QList<QPushButton*> list_step_move_q_minus_;
    QList<QPushButton*> list_step_move_x_plus_;
    QList<QPushButton*> list_step_move_x_minus_;
    QList<QPushButton*> list_step_move_x_plus_tool_;
    QList<QPushButton*> list_step_move_x_minus_tool_;

    QList<QLineEdit *> list_set_tcp_x_mm_;
    QList<QLineEdit *> list_add_tcp_x_mm_;

    void closeEvent(QCloseEvent *bar);

private:
    size_t task_trial_num_ = 0;
    size_t cnt_repeat_task_ = 0;


private Q_SLOTS:
    void timerCallback();

    void setEnableBtnCallback();
    void setTcpBtnCallback();

    void stepMoveQBtnCallback(uint index, bool is_plus);
    void stepMoveXBtnCallback(uint index, bool is_plus);
    void stepMoveXToolFrameBtnCallback(uint index, bool is_plus);

    void setDrflTcpBtnCallback();

    void setStepMoveTranslationScale_1();
    void setStepMoveTranslationScale_2();
    void setStepMoveTranslationScale_3();
    void setStepMoveTranslationScale_4();
    void setStepMoveTranslationScale_5();
    void setStepMoveTranslationScale_6();
    void setStepMoveTranslationScale_7();
    void setStepMoveTranslationScale_8();

    void setStepMoveRotationScale_1();
    void setStepMoveRotationScale_2();
    void setStepMoveRotationScale_3();
    void setStepMoveRotationScale_4();
    void setStepMoveRotationScale_5();
    void setStepMoveRotationScale_6();
    void setStepMoveRotationScale_7();
    void setStepMoveRotationScale_8();

    void taskStartTotalTaskBtnCallback();
    void taskStartForTeachingBtnCallback();
    void taskStartNoTeachingBtnCallback();
    void taskStartRepeatedlyForWithoutTeachingBtnCallback();
    void setTeachingPoseBtnCallback();

    void setTeachingPoseWithMarkerDetectionBtnCallback();
    void taskStartForTeachingWithMarkerBtnCallback();
    void taskStartNoTeachingWithMarkerBtnCallback();

    void genGraphyBtnCallback();

    void do3DScanningBtnCallback();
    void doKUAISTaskBtnCallback();

    void graphyTestFunctionBtnCallback();
    void graphyRobotPowerOnOffBtnCallback();

    //// Modbus
    void pushButtonModbusSetIPAddress();
    void pushButtonModbusConnect();
    void pushButtonModbusClose();
    void SetStepMotorPosition();
    void SetStepMotorVelocity();

    void pushButtonSelectPlcWriteAddressClickedCallback();
    void pushButtonSelectPlcReadAddressClickedCallback();
    void pushButtonModbusWriteDataBoolType();
    void pushButtonModbusWriteDataInt16Type();
    void pushButtonModbusWriteDataASCIIType();

    void pushButtonModbusReadDataBoolType();
    void pushButtonModbusMonitoringPLC();

    void pushButtonModbusTest();

    // Stop button
    void stopAllBtnCallback();
    void pauseTaskBtnCallback();
    void resumeTaskBtnCallback();


private Q_SLOTS:

    void pushButtonKorasGripperConnectionCallback();
    void pushButtonKorasGripperIntializeCallback();
    void pushButtonKorasDualGripperIntializeCallback();
    void pushButtonKorasGripperSlaveChangeCallback();
    void pushButtonKorasGripperSetInitMinMaxValue();
    void pushButtonKorasGripperOpenCallback();
    void pushButtonKorasGripperCloseCallback();
    void pushButtonKorasGripperPosCtrlCallback();
    void pushButtonKorasGripperPosCtrlOnOffCallback();
    void pushButtonKorasGripperVacuumOnOffCallback();

    // UR JS, CS control
    void pushButtonAbsRelCheckClickedCallback();
    void pushButtonCSMoveClickedCallback();
    void pushButtonJSMoveClickedCallback();

    void pushButtonCSMoveRelativeToolFrameClickedCallback();
    void pushButtonSetCurrentRobotAPoseClickedCallback();
    void pushButtonSetCurQRobotAClickedCallback();
    void pushButtonGetCurQRobotAClickedCallback();
    void pushButtonGetCurXRobotAClickedCallback();

    void pushButtonSetUIRobotAccVel();
    // For Jog
    void pushButtonJogSelectCallback();
    void pushButtonJogModeEndCallback();
    // Jog button
    void jogBtnPressed(uint index, bool is_plus, bool is_joint_space);
    void jogBtnReleased();

    void pushButtonSetUIJSPosition0ClickedCallback();
    void pushButtonSetUIJSPosition1ClickedCallback();
    void pushButtonSetUIJSPosition2ClickedCallback();
    void pushButtonSetUIJSPosition3ClickedCallback();
    void pushButtonSetUIJSPositionPackingClickedCallback();

    void on_pushButton_export_json_clicked();
    void on_pushButton_import_json_clicked();
    void on_pushButton_import_tw_clicked();
    // void on_pushButton_start_json_clicked();

    void on_pushButton_doosan_task_start_clicked();
    void comboBoxIndexChangedCallback(int index);
    void pushButtonSetTcpClickedCallback();
    void pushButtonAddTcpClickedCallback();

    //////////////////Cooking robot////////////////////////
    // 버튼 클릭 이벤트
    void onFryerButtonClicked();
    void onPotButtonClicked();
    void onBowlButtonClicked();
    void onMoveToFryerClicked();
    void onMoveToPotClicked();
    void onMoveToBowlClicked();
    void onMoveToCenterClicked();
    void SetPoseCallback(const std::string& position);
    void onTestButtonClicked();

    // UI 업데이트
    void updateRealsenseImage(const QImage &image);
    void updateRobotPose(const QString &toolName, double x, double y, double z, double roll, double pitch, double yaw);
    void onEnterButtonClicked(); // Enter 버튼 동작
    void updateLangSAMCoordinates(double x, double y, double z); // 좌표 업데이트
    void updateLangSAMImage(const QImage &image); // 이미지 업데이트



private:
    QWidget* central_widget;
    QVBoxLayout* main_layout;
    // BinMatchingDialog *bin_matching_dialog; // dialog
    // Robot2CameraCalibrationDialog *robot2camera_calibration_dialog; // dialog
    QNode *qnode_;

// #if SW_MODE_COOKING
    TaskPlannerCooking *cookingPlanner_;  // 전역 변수로 선언
// #endif
    void setupPlanner(); // TaskPlannerCooking 초기화 함수


    QTimer *timer_;
    int current_task_index = 0;
    std::vector<UnitTask> module_task_json;
    std::vector<std::vector<UnitTask>> module_tasks_json;

    QString current_tool_;//Cooking robot
    QString current_food_prompt_; // 입력된 음식 프롬프트 저장


};
#endif // MAINWINDOW_GRIPPER_H
