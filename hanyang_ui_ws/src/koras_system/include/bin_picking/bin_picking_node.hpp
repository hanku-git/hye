/**
 * @file bin_picking_node.hpp
 * @brief ROS Node 관리를 위한 헤더파일
 */

#ifndef BIN_PICKING_NODE_HPP_
#define BIN_PICKING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

// #include <ros/ros.h>
// #include <ros/callback_queue.h>
// #include <rosrt/rosrt.h>
#include <string>
#include <QThread>
#include <unordered_map>

#include "ui_define.hpp"
// #include <eigen3/Eigen/Eigen>

// Msg
#include <hanyang_matching_msgs/msg/robot_state.hpp>
#include <hanyang_matching_msgs/msg/bp_ui_command.hpp>
#include <hanyang_matching_msgs/msg/gripper_cmd.hpp>
#include <hanyang_matching_msgs/msg/bp_ui_command_learning.hpp>
// #include <hanyang_matching_msgs/msg/gripper_msg.hpp> // KORAS gripper state msg
#include <hanyang_matching_msgs/msg/do_template_matching_msg.hpp>
#include <hanyang_matching_msgs/msg/matching_result_msg.hpp>


// // Srv
// #include <hanyang_matching_msgs/srv/SetTargetJointPosition.hpp>
// #include <hanyang_matching_msgs/srv/SetTargetPose.hpp>
// #include <hanyang_matching_msgs/srv/SetTargetUIPose.hpp>
// #include <hanyang_matching_msgs/srv/SetTargetPoint.hpp>
#include <hanyang_matching_msgs/srv/set_tcp.hpp>

// #include <hanyang_matching_msgs/srv/JogCommand.hpp>
// #include <hanyang_matching_msgs/srv/StopRobot.hpp>
// #include <hanyang_matching_msgs/srv/SetCommand.hpp>
// #include <hanyang_matching_msgs/srv/SetRLAction.hpp>
// #include <hanyang_matching_msgs/srv/ImpedanceControl.hpp>
// #include <hanyang_matching_msgs/srv/DoAssembly.hpp>
// #include <hanyang_matching_msgs/srv/DoRLAssembly.hpp>
// #include <hanyang_matching_msgs/srv/DoRL_connectorAssembly.hpp>
// #include <hanyang_matching_msgs/srv/DoLoadID.hpp>
// #include <hanyang_matching_msgs/srv/IsControlFin.hpp>
// #include <hanyang_matching_msgs/srv/RLInsert.hpp>

// #include <hanyang_matching_msgs/srv/zivid_do_scan.hpp>

//// 3D Scanning
#include <hanyang_matching_msgs/srv/zivid_do_scan.hpp>
#include <hanyang_matching_msgs/srv/part_scanning.hpp>
#include <hanyang_matching_msgs/srv/part_detection.hpp>
#include <hanyang_matching_msgs/srv/grasping_pose.hpp>
#include <hanyang_matching_msgs/srv/do_template_matching.hpp>
#include <hanyang_matching_msgs/srv/do_pose_estimation.hpp>

//// Task Recognition
#include <hanyang_matching_msgs/srv/do_task_recognition.hpp>

//// KORAS gripper
#include <hanyang_matching_msgs/srv/gripper_command.hpp>

// //// Arduino
// #include <hanyang_matching_msgs/srv/relay.hpp>
// #include <hanyang_matching_msgs/srv/arduino_led.hpp>
// #include <hanyang_matching_msgs/srv/conveyor.hpp>
// #include <hanyang_matching_msgs/srv/limit_switch.hpp>

//// Blending
#include <hanyang_matching_msgs/srv/blending.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
////
// #include <sensor_msgs/Joy.hpp>
// #include <std_msgs/Int8MultiArray.hpp>
// #include <std_msgs/String.hpp>

// #include <ur_msgs/SetIO.hpp>

#include <bin_picking/nlohmann/json.hpp>
#include <time.h>

//// MODBUS TCP
#include "modbus/modbusTcpDefine.hpp"
#include "bin_picking/data_class/bin_picking_parameter.hpp"
#include "bin_picking/math/bin_picking_math.hpp"

// #include "task_parameter.hpp"
#include "task_planner.hpp"



/**
 * @class BinPickingNode
 * @brief 패키지 간 통신을 위한 ROS Node 정의 클래스
 * @ingroup USER_INTERFACE
 */
class BinPickingNode : public QThread {
	Q_OBJECT
public:
	BinPickingNode(int argc, char **argv );
	virtual ~BinPickingNode();

Q_SIGNALS:
	void rosShutdown();

public:
	BinPickingMath m_bp_math; //// Bin picking math class
    std::vector<CTARGET_OBJECT_DATA, Eigen::aligned_allocator<CTARGET_OBJECT_DATA>> m_bin_picking_node_target_object_list;
    std::vector<CTARGET_OBJECT_DATA*> m_ptr_bin_picking_node_target_object_list;
    std::vector<size_t> m_selected_object_index_;

public:

    //// Object data index
    size_t current_target_object_idx_ = 0;
    size_t before_target_object_idx_ = 0;

    //// Bin picking flag
	bool is_initial_scan_done = false; // 초기 스캔 및 매칭 여부
	bool is_grasping_pose_cal_finished = false;
	bool is_grasping_pose_assigned = false;
	bool is_grasping_pose_feasible_ = false;
	bool is_zig_pose_assigned = false;
	bool is_blending_approach_grasp_pose_assigned = false;
	bool is_blending_path_in_process_for_scan_and_matching = false;
	bool is_blending_scan_and_matching_conducted = false;
	bool is_detaching_stack_pose_assigned = false;
	bool bin_scan_fail = false;
	bool is_grasping_fail = false;
	bool is_matching_finished = false;
	bool is_task_in_progress = false;
	bool is_task_recognition_on = false;

	bool ui_flag_is_initial_scan_done = false;

	bool is_check_matching_finished = false;

	bool is_hinge_pose_recived = false;

	int m_robot_id = 0;
	int cnt_scanning_failure = 0;
	int detected_mask_num = 0;
    unsigned int operatingModeProcess_ = 2;
    unsigned int operatingModeTask_    = 2;

	double matching_accuracy = 0.0;
	double matching_accuracy_limit = 0.0;
	std::unordered_map<std::string, double> map_object_name_approach_distance_;
	double approach_distance = 0.0;
	double grp_width = 0.0;

	// //// Stacking count
	// StackingType stacking_mode_;
	// size_t detaching_cnt_ = 0;
	// size_t max_cnt_detaching_ = 0;
	// size_t stack_part_cnt_ = 0;
	// size_t stacking_single_stack_num_ = 0;
	// size_t stacking_line_stack_num_ = 0;
	// std::vector<double> stacking_trans_scale_;
	// size_t stacking_z_idx_ = 0;

	//// Zivid scanning parameters
	// taskScanningParameter m_scan_parameter;
    // taskTemplateMatchingParameter m_matching_parameter;
	// taskGraspingParameter m_grasping_parameter;
	//// Grasping pose estimation
	std::unordered_map<std::string, std::vector<double>> map_object_name_detected_pose_;
    std::vector<double> detected_pose_;
    std::vector<double> before_detected_pose_;
    std::vector<double> detected_approach_pose_;
    std::vector<double> before_detected_approach_pose_;
    std::vector<double> detected_zig_pose_;
    std::vector<double> before_detected_zig_pose_;

	std::unordered_map<std::string, std::vector<double>> map_object_name_detected_sub_pose_;
    std::vector<double> detected_sub_pose_;
	std::unordered_map<std::string, std::vector<double>> map_object_name_estimated_pose_;
	std::vector<double> estimated_pose_;
	std::vector<double> estimated_sub_pose_;
	std::vector<double> assigned_stacking_pose_;
	std::vector<double> m_measured_pose;

	//// Gripper state
	// uint16_t grp_driver_idx = 0;
	std::vector<int16_t> grp_measured_count;
	std::vector<int16_t> grp_measured_position;
	// std::vector<bool> is_grp_initialized;
	std::vector<int16_t> grp_initial_min_position;
	std::vector<int16_t> grp_initial_max_position;
	std::vector<bool> is_grp_set_min_value_finished;
	std::vector<bool> is_grp_set_max_value_finished;

    //// Tool changing JS position
	std::vector<unsigned int> task_tcp_idx_set_tool_changing_attach_;
    std::vector<std::vector<double>> task_vec_set_tool_changing_attach_JS_position_;
    std::vector<std::vector<double>> task_vec_set_tool_changing_detach_JS_position_;
    std::vector<std::vector<double>> task_vec_tool_changing_goal_cs_pose_;
	std::vector<double> tc_robot_left_side_attach_default_js_position_;
	std::vector<double> tc_robot_left_side_detach_default_js_position_;
	std::vector<double> tc_robot_right_side_attach_default_js_position_;
	std::vector<double> tc_robot_right_side_detach_default_js_position_;
	std::vector<bool> is_tool_left_side_;


    //// Tip changing JS position
	std::vector<unsigned int> task_tcp_idx_set_tip_changing_attach_;
    std::vector<std::vector<double>> task_vec_set_tip_changing_attach_JS_position_;
    std::vector<std::vector<double>> task_vec_set_tip_changing_detach_JS_position_;
	std::vector<double> task_vec_tip_changing_default_attach_JS_position_;
	std::vector<double> task_vec_tip_changing_default_detach_JS_position_;
	//// Robot Calibration
    std::vector<std::vector<double>> robot_calibration_vec_target_JS_position_;
	std::vector<std::vector<double>> robot_calibration_vec_target_CS_pose_;

public:
    shared_ptr<rclcpp::Node> node_;
	bool init();
	void spinNode();
	void publishUICommandLearningMsg(taskScanningParameter &parameter);
	void printVector(const std::vector<double> &vec, std::string str);
	bool checkWorkspace(const std::vector<double> pose, const Eigen::MatrixXd &workspace);

	void setTaskToolChangingJSPosition();
	void setTaskTipChangingJSPosition();
	void setRobotCalibrationUsingLaserTrackerTarget60JSPosition();
	void setRobotCalibrationISO9283Pose();
	void setRobotCalibrationUsing3DScannerTarget60JSPosition();
	void setZIVIDGlobalCalibrationTargetJSPosition();

	//////////////////////////////////////////////////////////////////////////////
	//// 3D scanning
    std::vector<double> getDetectedPose() {return detected_pose_;}
    std::vector<double> getBeforeDetectedPose() {return before_detected_pose_;}
    void setBeforeDetectedPose(std::vector<double> vec_in) { before_detected_pose_= vec_in; }
    std::vector<double> getDetectedApproachPose() {return detected_approach_pose_;}
    std::vector<double> getBeforeDetectedApproachPose() {return before_detected_approach_pose_;}
    void setBeforeDetectedApproachPose(std::vector<double> vec_in) { before_detected_approach_pose_= vec_in; }
    std::vector<double> getDetectedZigPose() {return detected_zig_pose_;}
    std::vector<double> getBeforeDetectedZigPose() {return before_detected_zig_pose_;}
    void setBeforeDetectedZigPose(std::vector<double> vec_in) { before_detected_zig_pose_= vec_in; }



    std::vector<double> getDetectedSubPose() {return detected_sub_pose_;}
    std::vector<double> getStackingPose() {return assigned_stacking_pose_;}
    double getMatchingAccuracy() {return matching_accuracy;}
    double getApproachDistance() {return approach_distance;}
	// uint16_t getGripperOpenLength() {return m_grasping_parameter.gripper_open_length;}
	// uint16_t getGripperTipIndex() {return m_grasping_parameter.gripper_tip_index;}
	int getDetectedMaskNum() {return detected_mask_num;}
	bool getFlagMatchingProcess() {return is_matching_finished;}
	std::vector<double> getEstimatedPose() {return estimated_pose_;}
    std::vector<double> getEstimatedSubPose() {return estimated_sub_pose_;}

	bool getMatchingProcessStatus() {return is_matching_finished && is_grasping_pose_cal_finished;}

	//// For bin detection
	bool getCSPoseWithDefaultTCPCommand(unsigned int tcp_idx, const std::vector<double> &js_position, std::vector<double> &pose);
	bool getCSPoseWithDefaultTCPCommandUsingCurrentJS(unsigned int tcp_idx, std::vector<double> &pose);
	bool getJSPositionWithDefaultTCPCommand(unsigned int tcp_idx, const std::vector<double> &cs_pose, std::vector<double> &js_position);
	//// 3D scanning
	bool scanZIVID(taskScanningParameter &parameter);
	bool doScanning(taskScanningParameter &parameter);
	bool doScanningAndMatching(taskScanningParameter &scan_param, taskTemplateMatchingParameter &matching_param);
	void doCADMatchingTopic(taskTemplateMatchingParameter &parameter);
	bool doCADMatchingService(taskTemplateMatchingParameter &parameter);
	bool doPoseEstimationAI(taskTemplateMatchingParameter &parameter);
	bool doTemplateInitialize(taskTemplateMatchingParameter &parameter, bool do_set_only_parameters, bool do_overwrite_JSON = false);
	bool doBinDetectionTemplateMatching(taskTemplateMatchingParameter &parameter);
	bool doBaseZigDetectionTemplateMatching(taskTemplateMatchingParameter &parameter);
	bool doEyeInHandDetectionTemplateMatching(taskTemplateMatchingParameter &parameter);

	bool doOptimizationBinPickingWorkspace(taskTemplateMatchingParameter &parameter);
	bool doGetRobot2CameraCalibrationJSPosition(taskTemplateMatchingParameter &parameter);
	bool doRobot2CameraCalibrationTemplateMatching(taskTemplateMatchingParameter &parameter);
	bool doEvaluateGraspingPose(taskTemplateMatchingParameter &parameter);
	bool doZigPoseInitialTeaching(taskTemplateMatchingParameter &parameter);

	bool doSimulationBinPlacing(taskTemplateMatchingParameter &parameter);
	bool doGetEnvParameters(taskTemplateMatchingParameter &parameter);
	bool transformToolChangingPose2SensorPose(taskTemplateMatchingParameter &parameter);
	bool transformToolChangingPose2NewBaseFrame(taskTemplateMatchingParameter &parameter);
	//// PCL Test function
	bool doPCLTestFunction(taskTemplateMatchingParameter &parameter, bool b_matching, bool do_scan_sampling, size_t sampling_num, size_t feature_ext_method, double voxel_downsampling_size);
	bool doPCLFunctionFeatureExtraction(taskTemplateMatchingParameter &parameter);
	bool doPCLFunctionSegmentation(taskTemplateMatchingParameter &parameter);

	//// Demo JSON
	bool setDemoJSONCSPose(std::vector<double> &output, std::string demo_name, uint16_t motion_tag);
	bool setBlendingPath(BlendingTraj &blending_traj, std::string demo_name, uint16_t motion_tag);
	bool setDemoRecogCSPose(std::vector<double> &output, std::string demo_name, uint16_t motion_tag);
	std::string getDemoName(uint16_t demo_tag);

	//// Task Monitoring
	void getTaskInfo(bool is_task_mode);

	////
	bool doGrasping(std::vector<double> &output_pose, enum GraspingPoseType pose_type, std::string demo_name = "default", std::string object_name = "default");
	bool checkMatchingResult(enum GraspingPoseType pose_type, std::string demo_name = "default");

	bool generateStackingPose(std::vector<double> &output_pose, size_t stacking_cnt, const std::vector<double> &trans_scale, size_t single_stack_num, size_t line_stack_num, enum StackingType stack_type);
	std::vector<double> transformRelativePoseToolFrame(const std::vector<double> &pose, const std::vector<double> &relative_pose);

	void hinge_pose_callback(const std_msgs::msg::Float32MultiArray &msg);

private:
	// void templateMatchingCallback(const hanyang_matching_msgs::msg::MatchingResultMsg &msg);
	TaskPlanner task_planner_;

private:
	int init_argc;
	char** init_argv;


    rclcpp::Publisher<hanyang_matching_msgs::msg::BpUiCommandLearning>::SharedPtr uiCommandLearningPublisher_;
    rclcpp::Publisher<hanyang_matching_msgs::msg::DoTemplateMatchingMsg>::SharedPtr doTemplateMatchingBinPickingPublisher_;
    // rclcpp::Subscription<hanyang_matching_msgs::msg::MatchingResultMsg>::SharedPtr matchingPoseResultSubscriber_;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr hinge_pose_sub_;

    rclcpp::Client<hanyang_matching_msgs::srv::ZividDoScan>::SharedPtr zividScanningClient_;
    rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doTemplateMatchingClient_;
    rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doTemplateMatchingHoleClient_;
	rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doTemplateMatchingBinPickingClient_;
	rclcpp::Client<hanyang_matching_msgs::srv::DoPoseEstimation>::SharedPtr doPoseEstimationBinPickingClient_;
	rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doTemplateInitializeClient_;
	rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doBinTemplateMatchingClient_;
	rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doBaseZigTemplateMatchingClient_;
	rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doEyeInHandTemplateMatchingClient_;


	rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doOptimizationBinPickingWorkspaceClient_;
	rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doGetRobot2CameraCalJSPositionClient_;
	rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doRobot2CameraCalTemplateMatchingClient_;
	rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doEvaluateGraspingPoseClient_;
	rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doZigPoseInitialTeachingClient_;
	rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doSimulationBinPlacingClient_;
	rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doGetEnvParametersClient_;

	rclcpp::Client<hanyang_matching_msgs::srv::SetTcp>::SharedPtr robotACalCSPoseWithDefaultTCPClient_;
	rclcpp::Client<hanyang_matching_msgs::srv::SetTcp>::SharedPtr robotACalJSPositionWithDefaultTCPClient_;

	//// PCL Test
	rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doPCLTestClient_;
	rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doPCLFunctionFeatureExtractionClient_;
    rclcpp::Client<hanyang_matching_msgs::srv::DoTemplateMatching>::SharedPtr doPCLFunctionSegmentationClient_;
};

// }  // namespace robot_control_ui

#endif
