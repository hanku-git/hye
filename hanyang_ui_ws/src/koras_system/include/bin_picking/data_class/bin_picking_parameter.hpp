#pragma once
#include <iostream>
#include <array>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

const int UR5CB = 7001;
const int UR10e = 7002;

// 221212, bin picking target object
const int ON_PART_GEAR = 31;
const int ON_PART_SQUARE_PEG = 32;
const int ON_PART_WELDING_ELBOW_JOINT = 33;
const int ON_PART_WELDING_T_JOINT_2F = 34;
const int ON_PART_BOLT_BUSH = 35;
const int ON_PART_ROTATION_AXIS_ONE_SIDED = 36;


const int ON_PART_BOLT = 37;
const int ON_PART_HINGE = 38;
const int ON_PART_STRIKER = 39;
const int ON_PART_SCU = 40;
const int ON_PART_GRILL = 41;

const int ON_PART_RIGHT_CHEMICAL_COUPLER_HOLDER = 42;
const int ON_PART_RIGHT_DRUM_HOLE_SURFACE = 43;
const int ON_PART_RIGHT_DRUM_COUPLER_UNSCREWING = 44;
const int ON_PART_RIGHT_DRUM_LID_CAP_UNSCREWING = 45;
const int ON_PART_RIGHT_DRUM_LID_CAP_HOLDER = 46;
const int ON_PART_RIGHT_DRUM_LID_CAP_HOLDER_EMPTY = 47;
const int ON_PART_RIGHT_CHEMICAL_COUPLER_HOLDER_EMPTY = 48;
const int ON_PART_RIGHT_DRUM_LID_CAP_SCREWING = 49;
const int ON_PART_DRUM_LID_TOTAL = 50;
const int ON_PART_LEFT_CHEMICAL_COUPLER_HOLDER = 51;
const int ON_PART_LEFT_DRUM_HOLE_SURFACE = 52;
const int ON_PART_LEFT_DRUM_COUPLER_UNSCREWING = 53;
const int ON_PART_LEFT_DRUM_LID_CAP_UNSCREWING = 54;
const int ON_PART_LEFT_DRUM_LID_CAP_HOLDER = 55;
const int ON_PART_LEFT_DRUM_LID_CAP_HOLDER_EMPTY = 56;
const int ON_PART_LEFT_CHEMICAL_COUPLER_HOLDER_EMPTY = 57;
const int ON_PART_LEFT_DRUM_LID_CAP_SCREWING = 58;
const int ON_PART_RIGHT_DRUM_KEY_CODE = 59;
const int ON_PART_RIGHT_DRUM_HOLDER_WITH_KEY_CODE = 60;
const int ON_PART_RIGHT_DRUM_HOLDER_WITHOUT_KEY_CODE = 61;
const int ON_PART_RIGHT_DRUM_KEY_CODE_UNSCREWING = 62;


const int ON_PART_SQUARE_PEG_NEW = 51;
const int ON_PART_WELDING_T_JOINT_2F_NEW = 52;
const int ON_PART_BOLT_BUSH_NEW = 53;
const int ON_PART_ROTATION_AXIS_ONE_SIDED_NEW = 54;
const int ON_PART_BOLT_NEW = 55;
const int ON_PART_RAM_NEW = 56;
const int ON_PART_RAM_SLOT_NEW = 57;



// #define ROS_LOG_INFO(...)  RCLCPP_INFO (rclcpp::get_logger("BinPickingPackage"), __VA_ARGS__);
// #define ROS_LOG_WARN(...)  RCLCPP_WARN (rclcpp::get_logger("BinPickingPackage"), __VA_ARGS__);
// #define ROS_LOG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("BinPickingPackage"), __VA_ARGS__);

enum GraspingPoseType
{
	OPTIMAL_POSE = 1,
	SUB_OPTIMAL_POSE,
	AI_ESTIMATED_POSE,
};

enum StackingType
{
	GRID_STACKING = 1,
	CYLINDER_STACKING,
};

// namespace robot_control_ui {

struct taskScanningParameter {
	unsigned int target_id = 0;
	std::string target_name;
	bool do_scan_sampling  = false;
	size_t sampling_num = 8;
	bool is_base_frame_unknown = false;

	bool is_mask_pixel_fixed = false;
    std::vector<int> mask_pixel_list;
	bool do_save_data = false;
	bool do_image_processing = true;
	bool do_single_matching = true;

	bool do_save_mrcnn_learning_data = false;
	bool do_not_scan_do_load_data = false;
	bool skip_detection_mask = false;
	unsigned int learning_data_idx = 0;

	std::vector<double> scan_position;

	//// detection parameters
	std::string weight_number;
	bool is_sam_mean_size_assigned = false;
	int sam_mean_size = 0;
	int sam_mask_min_area = 0;
	int sam_mask_max_area = 0;


	std::vector<double> robot_dh_vec;
	std::vector<double> robot_tcp_default;
	std::vector<double> robot_tcp;

};

struct taskTemplateMatchingParameter {

	size_t target_id;
	std::string target_name;
	std::string object_name;

	bool do_scan_sampling = true;
	bool debug_mode = false;
	size_t sampling_num;
	bool is_base_frame_unknown = false;

	bool do_cloud_clear = false;

	double voxel_downsampling_size;
	size_t feature_ext_method;

	// Segmentation method
	size_t segmentation_method;
	// Region growing segmentation
	int rgs_normal_K;
	int rgs_min_size;
	int rgs_max_size;
	double rgs_thre_angle;
	double rgs_thre_curvature;
	int rgs_neighbor_K;
	// Color-based region growing segmentation
	double color_rgs_neighbor_R;
	double color_rgs_thre_1st_color;
	double color_rgs_thre_2nd_color;
	size_t color_rgs_min_cluster_size;
	double color_rgs_neighbor_R_2;
	double color_rgs_thre_1st_color_2;
	double color_rgs_thre_2nd_color_2;
	size_t color_rgs_min_cluster_size_2;
	// Euclidean cluster segmentation
	double euclidean_cluster_tol;
	size_t euclidean_min_cluster_size;
	size_t euclidean_max_cluster_size;

	std::vector<double> custom_transformation_pose;


	// process data
	double matching_accuracy;
	std::vector<double> robot_camera_calibration_js_position;
	std::vector<double> pose_B2E_measured;
	std::vector<double> htm_vec_B2S_global;

	std::vector<double> robot_dh_vec;
	std::vector<double> robot_tcp_default;
	std::vector<double> robot_tcp;

	// r2s calibration
	bool is_r2cam_calibration_js_position_set_by_teaching = false;

	bool is_symmetric = false;
};

struct taskGraspingParameter {
	uint16_t gripper_open_length = 0;
	uint16_t gripper_close_length = 0;
	uint16_t gripper_tip_index = 1;
	bool is_tip_changing_applied = false;
	bool do_tip_changing_for_grasping = false;
};


class CTARGET_OBJECT_DATA
{
public:
	CTARGET_OBJECT_DATA()
    {
        // std::cout << "CTARGET_OBJECT_DATA Construct!" << std::endl;
    };

	~CTARGET_OBJECT_DATA()
    {
        // std::cout << "자원을 해제함!" << std::endl;
    };

	CTARGET_OBJECT_DATA(const CTARGET_OBJECT_DATA& s)
	{
	}

public:
	////
	bool is_template_initialized_ = false;

	//// Robot TCP changing id
	size_t tcp_changing_id = 7; // tcp #1, 2, ...

	//// Tool changing slave id
	unsigned int tool_changing_attach_id = 0; // # slave 1, 2, ..
	unsigned int tool_changing_detach_id = 0; // # slave 1, 2, ..

	//// Tip changing id
	bool is_tip_changing_allowed_ = false;
	unsigned int tip_changing_attach_id = 0; // # slave 1, 2, ..
	unsigned int tip_changing_detach_id = 0; // # slave 1, 2, ..

	//// Stacking count
	StackingType stacking_mode_;
	size_t detaching_cnt_ = 0;
	size_t max_cnt_detaching_ = 0;
	size_t stack_part_cnt_ = 0;
	size_t stacking_single_stack_num_ = 0;
	size_t stacking_line_stack_num_ = 0;
	std::vector<double> stacking_trans_scale_;
	size_t stacking_z_idx_ = 0;

	std::vector<int16_t> grp_initial_min_position;
	std::vector<int16_t> grp_initial_max_position;
	std::vector<bool> is_grp_set_min_value_finished;
	std::vector<bool> is_grp_set_max_value_finished;

public:
	taskScanningParameter m_scan_parameter;
    taskTemplateMatchingParameter m_matching_parameter;
	taskGraspingParameter m_grasping_parameter;


public:
	uint16_t getGripperOpenLength() {return m_grasping_parameter.gripper_open_length;}
	uint16_t getGripperTipIndex() {return m_grasping_parameter.gripper_tip_index;}
};


/***************************Bin_Picking********************************/

// const int UR10e = 7002;

// struct taskParameter {

// 	            bool isParallelTaskFin  = false;
// 	            bool isSimRobotMoveFin  = false;


// 	bool isUnitTaskFin      = false;
// 	bool isRealRobotMoveFin = false;
// 	bool isAssemblyFin = false;


//     int time_10ms = 0;
// 	unsigned int taskStep = 0;
// 	unsigned int taskEndNum;
// 	unsigned int task_mode = TASK_DEFAULT;



// 	bool isDetectionFin = false; // flag for service between NUC and mando PC
// 	bool isRoughAlignFin = false; // Rough alignment using Yolo
// 	bool isGripperFin = false;
// 	bool isTaskSeqFin = false;
// 	bool isTaskFail = false;
// 	bool isReadyMatching = false;
// 	bool isReadyTaskRecog = false;

// 	bool isRecoveryTask = false;
// 	bool isScanningRecoveryTask = false;
//     bool is_json_trajectory = false;
// 	////
// 	bool isAssemblyFailureFlag_ = false;
// 	bool isScanningFailureFlag_ = false;
// };

/***********************************************************************/

// /**
// * @class MASK_DATA
// * @brief 물체의 Mask 데이터 클래스
// * @ingroup BIN_PICKING_DATA_CLASS
// */
// class MASK_DATA
// {
// public:
// 	MASK_DATA() {};
// 	~MASK_DATA() {};
// public:
//     // Integer class IDs for each bounding box
//     std::vector<int> class_ids;
//     // # String class IDs for each bouding box
//     std::vector<std::string> class_names;
//     // # Float probability scores of the class_id
//     std::vector<float> scores;

//     // Detected class Count
//     size_t class_cnt;
//     std::vector<sensor_msgs::Image> mask_imgs;
//     std::vector<sensor_msgs::RegionOfInterest> boxes;

//     ////
//     size_t idx_target_mask_in;
//     size_t idx_target_mask_process;
//     std::vector<size_t> mask_confident_idx_set;
//     std::vector<size_t> pre_selected_idx_set;
// 	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PointXYZRGBA>> cloud_mask_rgba_list;
// 	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>, Eigen::aligned_allocator<pcl::PointXYZRGBNormal>> cloud_mask_rgbn_list;
// 	std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ>> mask_cloud_base_aligned_list;
// 	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>, Eigen::aligned_allocator<pcl::PointXYZRGBNormal>> mask_cloud_base_aligned_rgbn_list;

//     std::vector<Eigen::Vector4f> xyz_centroid_list;

// };
