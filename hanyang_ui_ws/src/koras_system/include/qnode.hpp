#ifndef KCR_CONTROL_UI_QNODE_HPP_
#define KCR_CONTROL_UI_QNODE_HPP_


#if ROBOT_CALIBRATION_MODE
#include "robot_calibration/RobotCal.h"
#endif

#include <rclcpp/rclcpp.hpp>
#include <QThread>
#include <QObject>
#include <QString>

#include "ui_define.hpp"
#include <time.h>

#include "task_planner.hpp"

#include "task_planner_hyundai_llm.hpp"
#include "task_planner_cooking.hpp"
#include "task_planner_hanyang_eng.hpp"


#include <jsoncpp/json/json.h>
#include <mutex>
#include <regex>
#include "model_matrix.hpp"
#include <cmath>   // sqrt() 사용
#include <cfloat>  // DBL_MAX 사용을 위해 추가
#include <QQueue>
#include <QPoint>
#include <QColor>
#include <QImage>
#include <QMap>
#include <QVector>
#include <climits>  // INT_MAX 사용을 위한 헤더 추가


// ROS msg
#include <kcr_control_msg/msg/robot_state.hpp>
#include <kcr_control_msg/msg/rsc_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

// ROS srv
#include <kcr_control_msg/srv/move_q.hpp>
#include <kcr_control_msg/srv/move_x.hpp>
#include <kcr_control_msg/srv/stop_robot.hpp>
#include <kcr_control_msg/srv/single_bool.hpp>
#include <kcr_control_msg/srv/single_string.hpp>
#include <kcr_control_msg/srv/impedance.hpp>
#include <kcr_control_msg/srv/tcp_payload.hpp>
#include <kcr_control_msg/srv/operating_status.hpp>
#include <kcr_control_msg/srv/blending.hpp>
#include <kcr_control_msg/srv/jog.hpp>
#include <kcr_control_msg/srv/xpad_jog.hpp>
#include <kcr_control_msg/srv/direct_torque_control.hpp>
#include <kcr_control_msg/srv/command.hpp>

// Gripper
#include <grp_control_msg/srv/gripper_command.hpp>
#include <grp_control_msg/srv/stop_motor.hpp>
#include "grp_control_msg/srv/driver_enable.hpp"
#include "grp_control_msg/msg/gripper_msg.hpp"

#if NEW_VER_KORAS_GRIPPER_PACKAGE
// New
#include <grp_control_msg/srv/single_int.hpp>
#include <grp_control_msg/srv/pos_vel_cur_ctrl.hpp>
#include <grp_control_msg/srv/void.hpp>
#endif


// DS robot
#include <sensor_msgs/msg/joint_state.hpp>
#include "dsr_msgs2/msg/robot_state.hpp"

#include <dsr_msgs2/srv/set_robot_mode.hpp>
#include <dsr_msgs2/srv/get_current_pose.hpp>
#include <dsr_msgs2/srv/move_joint.hpp>
#include <dsr_msgs2/srv/move_line.hpp>
#include <dsr_msgs2/srv/move_spline_joint.hpp>
#include <dsr_msgs2/srv/move_spline_task.hpp>
#include <dsr_msgs2/srv/jog.hpp>
#include <dsr_msgs2/srv/drl_start.hpp>
#include <dsr_msgs2/srv/set_current_tcp.hpp>
#include <dsr_msgs2/srv/config_create_tcp.hpp>
#include <dsr_msgs2/srv/servo_off.hpp>
#include <dsr_msgs2/srv/set_robot_control.hpp>
#include <dsr_msgs2/srv/move_stop.hpp>

#include <dsr_msgs2/srv/task_compliance_ctrl.hpp>
#include <dsr_msgs2/srv/release_compliance_ctrl.hpp>
#include <dsr_msgs2/srv/get_joint_torque.hpp>

#include <dsr_msgs2/srv/set_current_tool.hpp>
#include <dsr_msgs2/srv/config_create_tool.hpp>


#include <dsr_msgs2/srv/change_collision_sensitivity.hpp>
#include <dsr_msgs2/msg/robot_state.hpp>
#include <dsr_msgs2/srv/get_robot_state.hpp>

// #include <dsr_msgs2/srv/set_safe_stop_reset_type.hpp>
// #include <dsr_msgs2/srv/move_stop.hpp>

// Joy node
#include <sensor_msgs/msg/joy.hpp>
// ROS image
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// Robot Calibration
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// LLM
#include <QImage>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// #include <hanyang_matching_msgs/msg/llm_cmd.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/vector3.hpp>


//// Bin picking
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#if BIN_PICKING_FLAG
// #include <hanyang_matching_msgs/srv/SetCommand.hpp>
#include <hanyang_matching_msgs/srv/zivid_do_scan.hpp>
// #include <hanyang_matching_msgs/srv/do_template_matching.hpp>

//// Bin picking class
#include "bin_picking_node.hpp"
// #include "py_ui_bridge_node.hpp"

//// Bin picking class
#include "bin_picking_node.hpp"

//// MODBUS TCP
#include <memory>
#include "modbus/modbusTcpDefine.hpp"

//// Monitor ui
#include "mainwindow_widgets_monitoring.hpp"

#endif
//// Camera View
#include "mainwindow_widgets_camera_view.hpp"

//// Cooking
// #include "mainwindow_cook_server.hpp"

// python binding
#undef slots  // Qt의 slots 키워드 비활성화
#include <pybind11/embed.h>
#define slots Q_SLOTS  // Qt의 slots 키워드를 다시 활성화
#include <iostream>

// Apriltag
#include <tf2_msgs/msg/tf_message.hpp>


#include <QMetaType>


#include "hanyang_matching_msgs/srv/do_pcl_test.hpp"

#include "hanyang_matching_msgs/srv/send_command.hpp"

#include <variant>

extern int task_index_;
#if DRFL_CONTROL
#include "libdrfl/DRFLEx.h"
#endif
namespace py = pybind11;

// Apriltag related
typedef struct _TagInfo {
    std::string tag_name;
    ModelMatrix pose_camera_tag;
    ModelMatrix pose_base_tag;
    ModelMatrix tr_camera_tag;
    ModelMatrix tr_base_tag;

    _TagInfo() {
        tag_name = "null";
        pose_camera_tag = ModelMatrix(6, 1);
        pose_base_tag   = ModelMatrix(6, 1);
        tr_camera_tag   = ModelMatrix(4, 4);
        tr_base_tag     = ModelMatrix(4, 4);
    }
} TagInfo;

using json = nlohmann::json;

typedef struct _ApriltagData {
    ModelMatrix pose_base_tag;
    ModelMatrix pose_ee_tag;
    std::string id_str;
    bool is_tag_detected;

    _ApriltagData() {
        pose_base_tag = ModelMatrix(6, 1);
        pose_ee_tag = ModelMatrix(6, 1);
    }
} ApriltagData;

using namespace std;
struct TaskInfo {
    using TASK_FIN = bool;
    std::vector<std::vector<TASK_FIN>> taskInfo;
    int compelete_index;
    int compelete_tag;
};

struct Coordinates {
    float x;
    float y;
    float z;
};

enum LogLevel {
    INFO,
    WARN,
    ERROR
};
typedef struct  {
    LogLevel _log_level;
    std::string _log_message;
} LogInfo;

class QNode : public QThread {
    Q_OBJECT

Q_SIGNALS:
    void sendLogMessageStr(std::string &str);
    void newImageAvailable(QImage image);

private:
    // // Private constructor to prevent external instantiation
    // QNode(int argc = 0, char** argv = nullptr);
    // QNode(const QNode&) = delete;
    // QNode& operator=(const QNode&) = delete;

    QNode(int argc, char** argv, const std::string& argument = ""); // 선언만!
    static QNode* qnodePtr;
    static std::mutex mutex_;


public:
    static QNode& getInstance(int argc = 0, char** argv = nullptr, const std::string& argument = "") {
        if (qnodePtr == nullptr) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (qnodePtr == nullptr) {
                qnodePtr = new QNode(argc, argv, argument);
            }
        }
        return *qnodePtr;
    }
    virtual ~QNode();

public Q_SLOTS:
    void setTaskMode(bool is_task);

Q_SIGNALS:
    void rosShutdown();
    void UpdateLLMText(const QString &target, const QString &goal, const QString &passes);
    void UpdateGoalPose(const geometry_msgs::msg::Pose2D &goal_pose);
    void passPointUpdated(const QString &pass_point);
    void UpdateVelocites(const QString &velocities);
    void taskFinLogger();
    void taskEndLogger();
    void pauseLogger();
    void taskStartedSignal();
    void TaskInfoSet();
    void beforeTaskSignal();
    void sendLogMessage(LogInfo logInfo);
    void UpdateImage(const QImage &image, const QString &current_pass);
    void UpdateWaypoints(const QString &coordinates, const QString &current_pass);
    void LLMNodeReady();
    void updateRecordingStatus(bool isRecording);
    void requestStopAll();
    void newPoseReceived(const QString &toolName, double x, double y, double z, double roll, double pitch, double yaw); // Cooking robot
    void newImageReceived(const QImage &image);
    void newLangSAMCoordinatesReceived(double x, double y, double z);
    void newLangSAMImageReceived(const QImage &image);

public:
    bool init();
    void run();

    bool setPower(bool flag);
    bool setEnable(bool flag);
    bool jammingStateEnable();
    bool clearError();
    bool setAutomaticMode(bool flag);
    bool setManualMode(bool flag);

    bool setHandGuide(bool flag);
    bool setFrictionObserver(bool flag);
    bool setCollisionDetection(bool flag);
    bool setCollisionDemoMode(bool flag);
    bool setInvKineMethod(string ik_method);
    bool setDriverCtrlMode(string driver_ctrl_mode);

    bool updateParam();
    bool rollbackParam();

    bool recordingStart(std::string record_name);
    bool recordingEnd();

    MoveErrorType moveQ(JsDouble q_target, double qd_target, double qdd_target,
                        bool is_relative = false);
    MoveErrorType moveX(CsDouble x_target, double xd_target, double xdd_target,
                        bool is_base_frame = true, bool is_relative = false,
                        double q_redundant = 0);
    MoveErrorType moveSplineQ(std::vector<JsDouble>& q_targets, double qd_target, double qdd_target, bool is_relative);
    MoveErrorType moveSplineX(std::vector<CsDouble>& x_targets, double xd_target, double xdd_target, bool is_base_frame, bool is_relative, double q_redundant);

    MoveErrorType jogStart(uint8_t index, double vel, double acc, bool is_joint_space, bool is_base_frame = true);
    MoveErrorType jogEnd(double stop_time);

    bool drflDrlStart(std::string code);
    bool drflSetRobotMode(int mode);
    bool drflServoOff(int type);
    bool drflMoveStop(int type);
    bool drflSetRobotControl(int type);
    bool drflSetTcp(string tcp_name);
    bool drflCreateTcp(string tcp_name, CsDouble tcp);
    bool drflTaskComplianceCtrl();
    bool drflReleaseComplianceCtrl();
    bool drflSetCurrentTool(const std::string &tool_name);
    bool drflGetJointTorque();

    bool drflChangeCollisionSensitivity(int sensitivity);


    bool drflConfigCreateTool(const std::string &tool_name, double weight, const std::array<double, 3> &cog, const std::array<double, 6> &inertia);

    void robotStatePrintCallback(const dsr_msgs2::msg::RobotState::SharedPtr msg);
    void robotStatePrintTimerCb();

    void drfl_currentpose(const dsr_msgs2::msg::RobotState::SharedPtr msg);

    MoveErrorType moveBlending(BlendingTraj traj_blending);

    bool setDtcMode(JsBool dtc_mode);
    bool setDtcInput(JsDouble dtc_input);

    bool stopRobot(double time_stop = 0.2);
    bool setCommand(string cmd);
    bool setImpedance(Impedance imped, bool flag);
    bool setTcp(CsDouble tcp);
    bool setPayload(double payload, std::vector<double> com);
    bool saveForceData(string index);

    // Gripper related
    bool setGrpEnable(bool enable);
    bool setGrpCmd(uint8_t command, int16_t value);
    bool setGrpStop(uint16_t duration);
    bool setLed(uint16_t command, uint16_t color);
    bool setGrp(uint16_t command, int16_t value, uint16_t address);
    bool setGrpGraphy(uint16_t command, uint16_t value, uint16_t address);
    bool setGrpMotorCtrl(uint16_t command, int16_t value, uint16_t address);
    bool setGrpCtrlWord(uint16_t grp_ctrl_word);

    bool setGrpMotorEnable();
    bool setGrpMotorResetPose();

    bool setGrpInitialize();
    bool setGrpOpen();
    bool setGrpClose();
    bool setGrpVacOn();
    bool setGrpVacOff();
    
    bool drflSetGrp(string script);

    /// Task related
    void pauseTask();
    void resumeTask();
    void beforeTaskStart();
    void loadTaskList();
    void doControl();
    void doTask(vector<UnitTask> &current_task_list, TaskParameter &task_param);
    bool aprilTagDetection(uint tag_num);

    bool checkWorkspace(const std::vector<double> pose, const Eigen::MatrixXd &workspace);

    void setLogMessageStr(std::string &str);

    void sendRealsenseRequest(const QString &request); //Cooking robot
    void requestMoveTask(const QString &tool_name, double x, double y, double z, double roll, double pitch, double yaw);
    void requestLangSAMMoveTask(double x, double y, double z, double roll, double pitch, double yaw);
    void executeTasks();
    void setCurrentTool(const std::string &tool) { current_tool_ = tool; } //Cooking robot

    // void setCommand(uint32_t command);

    // void setPidParameter(const uint8_t joint, const double p_gain, const double i_gain, const double d_gain, const double observer_gain);
    // void setImpedanceParameter(const uint8_t axis, const double m_gain, const double d_gain, const double k_gain);
    // void getPidParameter(const uint8_t joint, double *p_gain, double *i_gain, double *d_gain, double *observer_gain);
    // void getImpedanceParameter(const uint8_t axis, double *m_gain, double *d_gain, double *k_gain);

    // std::string dec2hex(uint16_t num);
    // std::string create_crc(std::string hex_num);

public:
    ControlParameters params_;
    kcr_control_msg::msg::RscState rsc_params_;

    bool is_package_launched_ = false;

    // For task
    // TaskPlanner task_planner_;
    std::unique_ptr<TaskPlanner> task_planner_;

	// significant
    bool is_parallel_task_mode_  = false;
    bool is_jog_mode_  = false;
    int  jog_index_ = -1;
    int  jog_frame = -1;
    bool is_task_mode_ = false;
    bool is_simul_mode_ = false;
    bool is_developer_mode_ = false; // false: developer UI (Legacy), true: BP GUI (Graphic)
    bool is_task_finished_ = false;

    bool is_task_robot_disable_and_enable = false;
    bool do_open_developer_window_ = false;
    bool is_developer_window_open_ = false;

    bool is_task_button_clicked_ = false;

    bool is_marker_task_mode_ = false;
    bool is_total_task_in_progress_ = false;
    size_t repeat_cnt_total_task_ = 0;
    int repeat_Max = 0;
    
    bool is_box_pose_received_ = false;

    JogSpace jog_space_ = JS_JOG;

    vector<UnitTask> current_task_list_;
    vector<UnitTask> current_task_list_origin_;

    TaskParameter task_param_;

    struct timespec task_start_time_;

    //mutex mutex_;
    mutex mutex_4tag_;

    // Joy node realted
    vector<int32_t> joy_button_;
    vector<float>   joy_stick_axis_;

    double jog_xpad_trans_;
    double jog_xpad_ori_;

    uint task_cycle_ = 0;

    int test_cnt_ = 0;

    ApriltagData tag_info_temp;
    std::vector<ApriltagData> tag_info_;

    std::string task_log_str_;
    std::string current_task_log_;
    
//// TODO: calibration node

public: // Python UI class
    // PyUiBridgeNode *m_py_ui_bridge_node;

public:
    void LLMRecording();
    void LLMTest();
    void LLMStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    void LLMStartRecording();
    void LLMStopRecording();

    void toggleAutoRecording(bool enabled);

    bool is_llm_task_fin = true;

    QImage current_image_; // 현재 이미지를 저장할 변수
    QString current_pass_;
    geometry_msgs::msg::Point current_coordinates_; // 현재 좌표를 저장할 변수
    std::map<std::string, Coordinates> pass_point_data_;  // 경유지 데이터를 저장할 맵

    // Robot Calibration
    bool is_save_fin_ = false;
    std::vector<double> cal_check_err;
    void doB2Scalibration();
    void CalSaveCallback(std_msgs::msg::Bool is_saved);
    void CalCapBaseCallback(std::string folder_name);
    void CalCheckCalibration(std::string folder_name, std::array<double, JS_DOF> q_current);
    void CalCheckResultCallback(std_msgs::msg::Float64MultiArray mesured_err);

////////////////////////////////////////////////////////////////////////////////////////

public: // Bin picking node
	BinPickingNode *m_bin_picking_node; //// Bin picking class

	BinPickingMath m_bp_math; //// Bin picking math class


    std::string m_package_path;

	int acquisition_mode;
	int data_index;

	//// TCP setting
	bool is_tcp_initialized = false;

    unsigned int robot_current_tcp_idx_ = 7;
	//// Tool changing slave id
	unsigned int tool_changing_attach_id = 0; // # slave 1, 2, ..
	unsigned int tool_changing_detach_id = 0; // # slave 1, 2, ..
	bool is_tool_attached = false;
	int gripper_state = 0;

	//// Tip changing id
	unsigned int tip_changing_attach_id = 0; // # slave 1, 2, ..
	unsigned int tip_changing_detach_id = 0; // # slave 1, 2, ..
	bool is_tip_attached = false;
    bool tip_changing_task_flag_ = false;



	//// Task flag
	bool is_task_generated_ = false;
	bool is_task_blending_path_infeasible_ = false;
    bool do_skip_task = false;

    // int controlMode_ = TASK_DEFAULT;
    unsigned int operatingModeProcess_ = 2;
    unsigned int operatingModeTask_    = 2;
    bool isJogRobotA_;		// true : robotA, false : robotB

	///////////////////////////////// for task ///////////////////////////////////
    // // TaskPlanner taskPlanner_;
    // bool isTaskMode_         = false;
    // bool isParallelTaskMode_ = false;

    // unsigned int taskStepVerify_ = 0;
    // std::vector<UnitTask> currentTaskList_robotA_;
    // std::vector<UnitTask> currentTaskList_robotB_;
    // TaskParameter taskParam_robotA_;
    // TaskParameter taskParam_robotB_;

	// //// Task recog. parameter
	// // bool bin_scan_fail = false;
	// // bool is_grasping_fail = false;
	// std::vector<int16_t> grp_initial_min_position;
	// std::vector<int16_t> grp_initial_max_position;
	// std::vector<bool> is_grp_set_min_value_finished;
	// std::vector<bool> is_grp_set_max_value_finished;

	std::vector<double> jogXDot_;
	std::vector<double> jogQDot_;

    bool isJogBtnPressed_;
    bool isToolFrameRotation_ = false;
    bool isRPYRotation_ = true;

	//// Comm. UDP
    std::string ip_udp;
    int port_udp;
    // MODBUD TCP
    bool is_plc_simulation_mode_ = false;
    bool is_plc_task_mode_monitoring_ = false;
    bool is_plc_task_in_progress_ = false;



    bool is_monitoring_plc_read_amr_to_robot_status_ = false;
	std::shared_ptr<modbus> _modbus;
	std::string ip_modbus_;
    int port_modbus_;

    std::vector<bool> plc_status_word_41000_ = std::vector<bool>(16, false);
    std::vector<bool> plc_status_word_41002_ = std::vector<bool>(16, false);
    std::vector<bool> plc_status_word_41003_ = std::vector<bool>(16, false);
    std::vector<bool> plc_status_word_41004_ = std::vector<bool>(16, false);
    std::vector<bool> plc_status_word_42000_ = std::vector<bool>(16, false);
    // std::vector<bool> plc_status_word_42001_ = std::vector<bool>(16, false);
    std::vector<bool> plc_status_word_42002_ = std::vector<bool>(16, false);

    //// 경태님 두산 로봇 상태
    bool is_doosan_robot_collision_ = false;


	//// Robot TCP index
	int robot_tcp_idx_ = 0;

    //// Gripper
    uint16_t grp_driver_address_ = 1;

    bool grp_do_initialize_ = true;
    bool grp_is_vacuum_ = false;
    bool grp_is_vacuum_before_ = false;
    uint16_t grp_init_2_grp_cmd_ = 4500;

    //// Demo count
    size_t cnt_selected_object_ = 0;
    bool is_sequential_demo_ = false;

    bool is_stacking_mode_ = false;

    bool is_teaching_finished_ = false;
    bool is_task_teaching_mode_ = false;
    bool is_in_task_teaching_stage_ = false;

    //// UNIZ DATA LOAD FLAG
    bool is_uniz_file_loaded_ = false;

    //// MARKER FLAG
    bool is_tag_detecting_in_progress_ = false;
    bool is_tag_received_ = false;
    bool is_marker_detection_converged_ = false;
    bool is_marker_detection_approached_ = false;

    size_t tag_detection_delay_ = 0;

    QPixmap graphy_logo_img_;


public:
    //// Bin picking task
    void beforeBinPickingTaskStart(bool is_initial_process);

	void publishTaskState(unsigned int target_stage, unsigned int target_id);
	void publishTaskEndState(std::string str);
	void setLoadIDTargetJSPosition();
	void setMeasuredFT2Json();

    std::vector<double> getRobotAActualX()     {return robotAActualX_;}
    std::vector<double> getRobotAActualQ()     {return robotAActualQ_;}
    std::vector<double> getFTForceRaw() {return dataForceRaw_;}
    std::vector<double> getFTForceSensor() {return dataForceSensor_;}
    std::vector<double> getFTForceTool() {return dataForceTool_;}
    std::vector<double> getFTForceTask() {return dataForceTask_;}

    std::vector<double> getRobotDHParameters() {return robot_dh_vec_;}
    std::vector<double> getRobotDefaultTCP() {return robot_tcp_default_;}
    std::vector<double> getRobotTCP() {return robot_tcp_;}

    void setRobotDHParameters(std::vector<double> &vec_in) {robot_dh_vec_ = vec_in;}
    void setRobotDefaultTCP(std::vector<double> &vec_in) {robot_tcp_default_ = vec_in;}
    void setRobotTCP(std::vector<double> &vec_in) {robot_tcp_ = vec_in;}

    // Torque guard getter
    bool getTorqueGuardEnabled() const {return torque_guard_enabled_;}
    bool getTorquePollingEnabled() const {return torque_polling_enabled_;}

	bool sendChangeRobotTCPCommand(unsigned int tcp_idx);
	// bool setTargetJointPosition    (std::vector<double> jointPosition, double maxVelocity, double acceleration, bool isRelative = false, bool isRobotA = true, bool forcebias = false);
	// bool setTargetPose    (std::vector<double> pose, double maxVelocity, double acceleration, bool isRelative = false, bool isRobotA = true);
	bool setTargetRelativePose(std::string rotation_frame, std::vector<double> pose, double maxVelocity, double acceleration, bool isRelative = false, bool isRobotA = true);
	// bool setTargetPoint   (std::vector<double> pose, double maxVelocity, double acceleration, bool isRelative = false, bool isRobotA = true);
	// bool setTargetUIPose    (std::vector<double> pose, double maxVelocity, double acceleration, bool isRelative = false, bool isRobotA = true);
	// bool setImpedanceControl(bool enable, std::vector<double> stiffness, std::vector<double> nf, std::vector<double> zeta, std::vector<double> forceLimit, bool isRobotA);
	// bool setCommandATIStart();
	// bool setForceBias();
	// bool setSubForceBias(bool isRobotA);
	bool setInitSensorBias();
	bool setUDPComm(const std::string cmd, const std::string &ip, const int &port);
	bool setMODBUSComm(bool do_connect);
	bool sendMessageMODBUS(uint16_t address, uint16_t amount, bool flag_coil = true);
	bool setPLCModbusCommand(int plc_command);
    bool setPLCModbusCommandWriteRegister(int plc_command, int32_t write_data);

    bool PLCModbusWriteStatusFromRobotToAMR(uint16_t register_address, bool is_lsb, uint16_t bit_address, bool status);
    bool PLCModbusInitializeStatusFromRobotToAMR(uint16_t register_address, bool is_lsb, uint16_t bit_address, bool status);
    bool setPLCModbusRotationAngle(uint16_t register_address, uint16_t bit_address, double angle);
    bool setPLCModbusBarcodeWrite(uint16_t register_address, uint16_t bit_address, const std::string& barcode);

    bool PLCModbusReadStatusFromAMR(uint16_t register_address, uint16_t bit_address, bool &status_read);
    bool PLCModbusReadStatusFromAMR2(const uint16_t register_address, uint16_t &current_value);
    // void monitorPLCStatus();


	//// KORAS gripper
	bool checkConnectionKorasGripperDriver();
	// bool sendKorasGripperDriverCommand(uint16_t cmd, uint16_t position, uint16_t grp_address);
    // MoveErrorType moveBlending(BlendingTraj traj_blending);

	void doLoadIdentification(bool b_id);
	void jogDesXControl();
	void jogDesQControl();
    // bool robotAControlStatusCallback(assem_control_msgs::IsControlFin::Request &request, assem_control_msgs::IsControlFin::Response &response);

	//// Monitoring
    void setTaskInfo(TaskInfo taskInfo);

	//// Monitoring
    void setTaskInfoForMonitorAtStart(TaskInfo taskInfo);
    void setTaskInfoForMonitor(TaskInfo taskInfo);

    void sendTaskInfoLog(std::string &str);
    void sendTaskErrorLog(std::string &str);

    void setCameraAutoFocus(bool is_auto_focus_on);

    void executeHyundaiLlmTask();
    void executeCookingTask();
    void executeHanyangEngTask();

    void getDrflCurrentPose();

    size_t getTargetObjectIndex(const std::string& object_name);
    void rebuildTargetObjectIndexMap();


	TaskInfo getTaskInfo();
public:
	void setToolChangingCurrentIndex(unsigned int id) {tool_changing_current_id = id;}
	void setTipChangingCurrentIndex(unsigned int id) {tip_changing_current_id = id;}
	unsigned int getToolChangingCurrentIndex() {return tool_changing_current_id;}
	unsigned int getTipChangingCurrentIndex() {return tip_changing_current_id;}

    bool setCurrentObjectIndex(size_t idx_now);
    bool setSamInfoUpdateCurrentObject();
    bool setRobotInfoUpdateCurrentObject();


    QImage getCameraImage() {return q_image_;}
    QImage getDownsampledCameraImage() {return q_image_downsampled_;}

    bool getCameraState() {return is_camera_mode_on_;}
    void setCameraState(bool cam_on) { is_camera_mode_on_ = cam_on;}

    std::vector<double> getTargetTeachingPose() { return teaching_target_pose_; }
    std::string getTaskTagTeachingPose() { return tag_teaching_pose_; }
    std::string getIdxTeachingPose() { return pose_idx_teaching_pose_; }

    // Apriltag related
    void tagRecogCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    bool saveTagInfo();
    bool setCurrentTag(std::string tag_name);
    bool setCurrentTagFromJSON(std::string tag_name);
    bool getTagBasedPose(CsDouble input_pose, CsDouble &output_pose);
    bool getTagBasedPose(CsDouble input_pose, CsDouble &output_pose, std::string tag_name);

    TagInfo getCurrentTagInfo() { return tag_info_current_; };

    MoveErrorType moveX_tagBased(CsDouble pose_tag_target, double xd_target, double xdd_target,
                                 double q_redundant = 0);


    bool sendUNIZData(std::vector<std::vector<std::vector<double>>> &l_bbox_set);
    bool sendRelayCommand(const std::string cmd);

    void sendLangSAMTextPrompt(const std::string& text_prompt);//Cooking robot


public: //Cooking
    inline bool is_base64(unsigned char c);
    std::string base64_decode(std::string& encoded_string);
    bool setCustomCode(std::string);

    bool is_data_request_active_; //Cooking robot
    bool is_image_request_active_ = false;


public: // Shared Task

    std::string setTaskRecogArduinoLED(unsigned int tgt_id, unsigned int led_command);
    void getTaskFinishFlag();


	void setTaskTargetPosePosition(unsigned int target_stage, unsigned int target_id);

	//// Task Recognition value
	unsigned int task_target_id = 0;
	unsigned int task_stage_id = 0;

    std::vector<std::vector<double>> task_vec_target_JS_position_;
    std::vector<std::vector<double>> task_vec_target_CS_pose_;

	bool isConveyorSystemON = false;
	bool isTaskRecognitionON = false;
	bool isRLGetActionON = false;


	bool b_A1_CPU = false;
	bool b_A1_RAM = false;
	bool b_A1_SSD = false;
	bool b_A1_Cooler = false;
	bool b_A2_CPU = false;
	bool b_A2_RAM = false;
	bool b_A2_SSD = false;
	bool b_A2_Cooler = false;
	bool b_B1_HDMI = false;
	bool b_B1_USB = false;
	bool b_B1_Power = false;
	bool b_B2_HDMI = false;
	bool b_B2_USB = false;
	bool b_B2_Power = false;

	bool b_select_stage_default = true;
	bool b_select_stage_init = false;
	bool b_select_stage_A1 = false;
	bool b_select_stage_A2 = false;
	bool b_select_stage_B1 = false;
	bool b_select_stage_B2 = false;

	bool b_finish_stage_init = false;
	bool b_finish_stage_A1 = false;
	bool b_finish_stage_A2 = false;
	bool b_finish_stage_B1 = false;
	bool b_finish_stage_B2 = false;

	bool is_part_on_stage_init = false;
	bool is_part_on_stage_A1 = false;
	bool is_part_on_stage_A2 = false;
	bool is_part_on_stage_B1 = false;
	bool is_part_on_stage_B2 = true; // Last stage (not on the conveyor)

	int getRecogStageID() {return task_stage_id;}
	int getRecogPartID() {return task_target_id;}

    bool is_scanning_and_detection_finished = false;
    bool is_open3d_processing_finished = false;

private: // Bin picking
	//// Robot data
    std::vector<double> robotAActualQ_;
    std::vector<double> robotAActualX_;
	std::vector<double> dataForceRaw_;
    std::vector<double> dataForceSensor_;
    std::vector<double> dataForceTool_;
    std::vector<double> dataForceTask_;

	std::vector<double> robot_dh_vec_;
	std::vector<double> robot_tcp_default_;
	std::vector<double> robot_tcp_;

	std::vector<double> teaching_target_pose_;
	std::vector<double> uniz_top_right_initial_pose_;

    std::string tag_teaching_pose_;
    std::string pose_idx_teaching_pose_;
    std::string task_log_;

	// //// Gripper state
	// std::vector<int16_t> grp_measured_count;
	// std::vector<int16_t> grp_measured_position;
	std::vector<bool> is_grp_initialized;
	// uint16_t grp_driver_idx = 0;
    uint16_t grp_meas_motor_current_ = 0;
    uint16_t grp_meas_pos_ = 0;

	//// Load Identification
    std::vector<std::vector<double>> load_id_vec_target_JS_pose_;
    std::vector<std::vector<double>> load_id_vec_meas_FT_data_;

    bool isBtnPressedBefore_ = false;

	bool is_sensor_bias = false;
	bool is_robot_state_connected = false;

	unsigned int tool_changing_current_id = 0; // # slave 1, 2, ..
	unsigned int tip_changing_current_id = 0; // # slave 1, 2, ..





    QImage q_image_;
    QImage q_image_downsampled_;
    bool is_camera_mode_on_ = false;

    // Apriltag related
    ModelMatrix pose_ee_camera_;
    ModelMatrix tr_ee_camera_;
    ModelMatrix tr_camera_ee_;

    std::vector<TagInfo> tag_info_vec_temp_;
    std::vector<TagInfo> tag_info_vec_;
    TagInfo tag_info_current_;
    std::atomic<bool> flag_tag_recog_;

    // Name to index map for target objects (built from actual UI order)
    std::unordered_map<std::string, size_t> name_to_index_map_;

    /// sb 3 24.11.25
    JsDouble q_target_reverse_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool relative_reverse_ = 0;
    
    // sb 25.04.11
    double teaching_to_init_RZ_ = 0.0;




    std::string current_tool_; //Cooking robot

    //// Doosan Robot
    std::vector<double> js_position_before_;
    std::vector<double> js_position_now_;
    bool do_check_js_err_ = false;
    bool is_robot_moving_begin_ = false;
    size_t cnt_for_is_js_err_ok_ = 0;
    size_t cnt_for_is_js_err_exceeds_ = 0;

    // Torque guard state
    bool torque_guard_enabled_ = false;
    int  torque_guard_joint_index_ = -1; // -1: use vector norm
    double torque_guard_threshold_ = 0.0; // threshold in joint torque unit

    // Torque polling state (for controlling periodic torque service calls)
    bool torque_polling_enabled_ = false;
    
    //// Drum Rotation Angle
    double drum_rotation_angle_ = 0.0;
    bool keycode_angle_received_ = false;
    double drum_rotation_angle_holder_ = 0.0;
    bool keycode_holder_angle_received_ = false;

    // Torque guard polling
    std::chrono::steady_clock::time_point last_torque_guard_poll_{};


////////////////////////////////////////////////////////////////////////////////////////

private:
    void robotStateCallback(const kcr_control_msg::msg::RobotState::SharedPtr msg);
    void rscStateCallback(const kcr_control_msg::msg::RscState::SharedPtr msg);
    void gripperStateCallback(const grp_control_msg::msg::GripperMsg::SharedPtr msg);
    void getJoyData(sensor_msgs::msg::Joy::SharedPtr msg);
    bool getOperatingStatus(const shared_ptr<kcr_control_msg::srv::OperatingStatus::Request> req,
                            shared_ptr<kcr_control_msg::srv::OperatingStatus::Response> res);
    bool xpadJogControl();
    // void setLLMText(const hanyang_matching_msgs::msg::LlmCmd::SharedPtr msg);
    void LLMPointCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void LLMPassImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void LLMGoalPoseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);

    void scannerImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // Robot calibration
    void calSaveCallback(std_msgs::msg::Bool is_saved);

    void ScanningResultCallback(const hanyang_matching_msgs::msg::MatchingResultMsg &msg);
	void templateMatchingCallback(const hanyang_matching_msgs::msg::MatchingResultMsg &msg);
    void imageCallback(const sensor_msgs::msg::Image &msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void CookGoalPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);//Cooking robot
    void CookImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    // LangSAM 데이터 처리 콜백 함수
    void LangSAMCoordCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg); // 좌표 콜백
    void LangSAMImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);         // 이미지 콜백
    void KeycodeAngleCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void KeycodeHolderAngleCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);

private:
    int init_argc;
    char** init_argv;

    // ROS node
    shared_ptr<rclcpp::Node> node_;

public:
#if ROBOT_CALIBRATION_MODE
    CRobotCal m_RobotCal;
#endif

#if DRFL_CONTROL
public:
    bool try_connect();
    void drlfPathFinished();
    void drflRobotStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    bool drflGrpConnect(bool flag);
    string grpCmd;
    array<double, 6> getRobotStateCallback(int space_type);
    std::map<std::string, CsDouble> list_tcp;

    bool is_drfl_connected = false;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_robot_state_;
    rclcpp::Client<dsr_msgs2::srv::GetCurrentPose>::SharedPtr client_robot_state_;

    rclcpp::Subscription<dsr_msgs2::msg::RobotState>::SharedPtr dsr_state_sub_;
    rclcpp::TimerBase::SharedPtr                                 dsr_state_timer_;

    dsr_msgs2::msg::RobotState dsr_last_state_;
    bool                       dsr_state_ready_ = false;

    /* ▼▼ ② 콜백 선언 ▼▼ */
    void dsrRobotStateCb (const dsr_msgs2::msg::RobotState::SharedPtr msg);
    void dsrStatePrintTimerCb();


    rclcpp::Client<dsr_msgs2::srv::GetRobotState>::SharedPtr client_get_robot_state_;
    rclcpp::TimerBase::SharedPtr                              timer_get_robot_state_;

    /* ② 콜백 프로토타입 */
    void requestRobotState();   // 5초마다 실행
    void robotStateRespCb(rclcpp::Client<
        dsr_msgs2::srv::GetRobotState>::SharedFuture future);

        

    bool plc_safe_stop_sent_ = false;

    rclcpp::Client<dsr_msgs2::srv::MoveJoint>::SharedPtr client_move_q_;
    rclcpp::Client<dsr_msgs2::srv::MoveLine>::SharedPtr client_move_x_;
    rclcpp::Client<dsr_msgs2::srv::MoveSplineJoint>::SharedPtr client_move_spline_q_;
    rclcpp::Client<dsr_msgs2::srv::MoveSplineTask>::SharedPtr client_move_spline_x_;

    rclcpp::Client<dsr_msgs2::srv::Jog>::SharedPtr client_jog_;
    rclcpp::Client<dsr_msgs2::srv::DrlStart>::SharedPtr client_drl_start_;

    rclcpp::Client<dsr_msgs2::srv::SetCurrentTcp>::SharedPtr client_set_tcp_;
    rclcpp::Client<dsr_msgs2::srv::ConfigCreateTcp>::SharedPtr client_create_tcp_;
    rclcpp::Client<dsr_msgs2::srv::SetRobotMode>::SharedPtr client_set_robot_mode_;
    rclcpp::Client<dsr_msgs2::srv::ServoOff>::SharedPtr client_servo_off_;
    rclcpp::Client<dsr_msgs2::srv::SetRobotControl>::SharedPtr client_set_robot_control_;
    rclcpp::Client<dsr_msgs2::srv::MoveStop>::SharedPtr client_stop_robot;

    rclcpp::Subscription<dsr_msgs2::msg::RobotState>::SharedPtr robot_state_sub_;
    rclcpp::Node::SharedPtr internal_node_;

    bool compliance_initialized_ = false;
    rclcpp::Client<dsr_msgs2::srv::TaskComplianceCtrl>::SharedPtr client_compliance_ctrl;
    rclcpp::Client<dsr_msgs2::srv::ReleaseComplianceCtrl>::SharedPtr client_release_compliance_ctrl;
    rclcpp::Client<dsr_msgs2::srv::SetCurrentTool>::SharedPtr client_set_current_tool_;
    rclcpp::Client<dsr_msgs2::srv::ConfigCreateTool>::SharedPtr client_config_create_tool_;
    rclcpp::Client<dsr_msgs2::srv::ChangeCollisionSensitivity>::SharedPtr client_change_collision_sensitivity_;
    rclcpp::Client<dsr_msgs2::srv::GetJointTorque>::SharedPtr client_get_dsr_torque;



#else
    // Subscription
    rclcpp::Subscription<kcr_control_msg::msg::RobotState>::SharedPtr subscription_robot_state_;


    // Service client
    rclcpp::Client<kcr_control_msg::srv::MoveQ>::SharedPtr client_move_q_;
    rclcpp::Client<kcr_control_msg::srv::MoveX>::SharedPtr client_move_x_;

    rclcpp::Client<kcr_control_msg::srv::Jog>::SharedPtr client_jog_;

#endif

    // Publisher
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_tag_;

    // Subscription
    // rclcpp::Subscription<kcr_control_msg::msg::RobotState>::SharedPtr subscription_robot_state_;
    rclcpp::Subscription<kcr_control_msg::msg::RscState>::SharedPtr   subscription_rsc_state_;
    rclcpp::Subscription<grp_control_msg::msg::GripperMsg>::SharedPtr subscriber_gripper_state_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_joy_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscriber_tag_;

    // Service client
    rclcpp::Client<kcr_control_msg::srv::SingleBool>::SharedPtr client_power_on_off_;
    rclcpp::Client<kcr_control_msg::srv::SingleBool>::SharedPtr client_set_enable_;
    rclcpp::Client<kcr_control_msg::srv::SingleBool>::SharedPtr client_set_AutomaticMode_;
    rclcpp::Client<kcr_control_msg::srv::SingleBool>::SharedPtr client_set_ManualMode_;
    rclcpp::Client<kcr_control_msg::srv::SingleBool>::SharedPtr client_jamming_state_enable_;
    rclcpp::Client<kcr_control_msg::srv::SingleBool>::SharedPtr client_clear_error_;
    rclcpp::Client<kcr_control_msg::srv::SingleBool>::SharedPtr client_set_hand_guide_;
    rclcpp::Client<kcr_control_msg::srv::SingleBool>::SharedPtr client_set_friction_observer_;
    rclcpp::Client<kcr_control_msg::srv::SingleBool>::SharedPtr client_set_collision_detection_;
    rclcpp::Client<kcr_control_msg::srv::SingleBool>::SharedPtr client_set_collision_demo_mode_;
    rclcpp::Client<kcr_control_msg::srv::SingleString>::SharedPtr client_set_inv_kine_method_;
    rclcpp::Client<kcr_control_msg::srv::SingleString>::SharedPtr client_set_driver_ctrl_mode_;

    rclcpp::Client<kcr_control_msg::srv::SingleBool>::SharedPtr client_update_param_;
    rclcpp::Client<kcr_control_msg::srv::SingleBool>::SharedPtr client_rollback_param_;

    rclcpp::Client<kcr_control_msg::srv::SingleString>::SharedPtr client_record_start_;
    rclcpp::Client<kcr_control_msg::srv::SingleBool>::SharedPtr   client_record_end_;

    // rclcpp::Client<kcr_control_msg::srv::MoveQ>::SharedPtr client_move_q_;
    // rclcpp::Client<kcr_control_msg::srv::MoveX>::SharedPtr client_move_x_;

    // rclcpp::Client<kcr_control_msg::srv::Jog>::SharedPtr client_jog_;

    rclcpp::Client<kcr_control_msg::srv::Blending>::SharedPtr client_blending_;

    rclcpp::Client<kcr_control_msg::srv::DirectTorqueControl>::SharedPtr client_dtc_;

    rclcpp::Client<kcr_control_msg::srv::Command>::SharedPtr client_led_;
    rclcpp::Client<kcr_control_msg::srv::Command>::SharedPtr client_grp_;
    rclcpp::Client<kcr_control_msg::srv::Command>::SharedPtr client_grp_ctrl_word_;

    rclcpp::Client<kcr_control_msg::srv::StopRobot   >::SharedPtr client_stop_robot_;
    rclcpp::Client<kcr_control_msg::srv::SingleString>::SharedPtr client_set_command_;
    rclcpp::Client<kcr_control_msg::srv::Impedance   >::SharedPtr client_set_impedance_;
    rclcpp::Client<kcr_control_msg::srv::TcpPayload  >::SharedPtr client_set_tcp_payload_;
    rclcpp::Client<kcr_control_msg::srv::SingleString>::SharedPtr client_save_force_data_;

    rclcpp::Service<kcr_control_msg::srv::OperatingStatus>::SharedPtr server_operating_status_;

    // LLM
    // rclcpp::Subscription<hanyang_matching_msgs::msg::LlmCmd>::SharedPtr subscriber_llm_text_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_record_voice_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_start_record_voice_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_stop_record_voice_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscriber_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_toggle_auto_rec_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr realsense_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_subscriber_;
    std::mutex data_mutex_;

#if NEW_VER_KORAS_GRIPPER_PACKAGE

    //// New version
    rclcpp::Client<grp_control_msg::srv::SingleInt>::SharedPtr new_client_gripper_cmd_;
    rclcpp::Client<grp_control_msg::srv::PosVelCurCtrl>::SharedPtr new_client_gripper_motor_pos_ctrl_;
    rclcpp::Client<grp_control_msg::srv::Void>::SharedPtr new_client_gripper_motor_enable_;
    rclcpp::Client<grp_control_msg::srv::Void>::SharedPtr new_client_gripper_motor_reset_pose_;

    rclcpp::Client<grp_control_msg::srv::Void>::SharedPtr new_client_gripper_cmd_initialize_;
    rclcpp::Client<grp_control_msg::srv::Void>::SharedPtr new_client_gripper_cmd_open_;
    rclcpp::Client<grp_control_msg::srv::Void>::SharedPtr new_client_gripper_cmd_close_;
    rclcpp::Client<grp_control_msg::srv::Void>::SharedPtr new_client_gripper_cmd_vac_on_;
    rclcpp::Client<grp_control_msg::srv::Void>::SharedPtr new_client_gripper_cmd_vac_off_;

#endif
    // Gripper
    rclcpp::Client<grp_control_msg::srv::GripperCommand>::SharedPtr client_gripper_cmd_;
    rclcpp::Client<grp_control_msg::srv::StopMotor>::SharedPtr client_stop_motor_;
    rclcpp::Client<grp_control_msg::srv::DriverEnable>::SharedPtr client_grp_enable_;

    rclcpp::Client<kcr_control_msg::srv::XpadJog>::SharedPtr client_xpad_jog_;

    //// Robot calibration
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cal_scan_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cal_scan_sub;
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cal_check_sub;
    std::string fname;

    //// Bin picking
    rclcpp::Subscription<hanyang_matching_msgs::msg::MatchingResultMsg>::SharedPtr isScanningResultSubscriber_;
    rclcpp::Subscription<hanyang_matching_msgs::msg::MatchingResultMsg>::SharedPtr matchingPoseResultSubscriber_;


    rclcpp::Publisher<hanyang_matching_msgs::msg::RobotState>::SharedPtr bp_robot_state_pub_;

    //// Camera Image
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr keycode_angle_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr keycode_holder_angle_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher_;

    ////Camera info
    // sb 25.03.06
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_infoSubscriber_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_infoPublisher_;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_ = nullptr;

    // sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_ = nullptr;







	//// Graphy PCL Test
	rclcpp::Client<hanyang_matching_msgs::srv::DoPclTest>::SharedPtr graphySendUNIZClient_;

    //// Relay Arduino
	rclcpp::Client<hanyang_matching_msgs::srv::SendCommand>::SharedPtr client_relay_command_;

    ////Cooking robot
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pose_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;  // 이미지 구독자
    // LangSAM 노드 통신 관련
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr langsam_request_publisher_; // 텍스트 프롬프트 발행
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr langsam_coord_subscriber_; // 중심 좌표 수신
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr langsam_image_subscriber_; // 분할 이미지 수신



    TaskInfo taskInfo;
    int taskCount;

    py::object ros_thread;

};
Q_DECLARE_METATYPE(LogLevel)
Q_DECLARE_METATYPE(LogInfo)

#endif /* KCR_CONTROL_UI_QNODE_HPP_ */
