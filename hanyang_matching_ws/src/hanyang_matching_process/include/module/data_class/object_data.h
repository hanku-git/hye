#pragma once
#include <iostream>
#include <array>
#include <string>
#include <vector>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/pcl_exports.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h> // normal estimation
#include <pcl/features/pfh.h> // PFH
#include <pcl/features/fpfh.h> // FPFH
#include <pcl/features/spin_image.h> // SpinImage
#include <pcl/features/3dsc.h> // shape context
#include <pcl/features/impl/3dsc.hpp> // shape context

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nlohmann/json.hpp> // JSON

#include <octomap/OcTree.h>

#include "control/RobotModel.hpp"
#include "module/data_class/optimization_data.h"
#include "module/data_class/planning_data.h"
#include "module/data_class/simulation_data.h"

#include "define.h"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "hanyang_matching_msgs/srv/do_learning_workspace.hpp"

/**
* @class MASK_DATA
* @brief 물체의 Mask 데이터 클래스
* @ingroup BIN_PICKING_DATA_CLASS
*/
class MASK_DATA
{
public:
	MASK_DATA() {};
	~MASK_DATA() {};
public:
    int target_id = 0;
    std::string detection_mode;
    // Integer class IDs for each bounding box
    std::vector<int> class_ids;
    // # String class IDs for each bouding box
    std::vector<std::string> class_names;
    // # Float probability scores of the class_id
    std::vector<float> scores;

    // Detected class Count
    size_t class_cnt;
    std::vector<sensor_msgs::msg::Image> mask_imgs;
    std::vector<sensor_msgs::msg::RegionOfInterest> boxes;

    // SAM - 주변 물체의 빽빽함 정도    
    std::vector<float> ext_nearby_ratio;

    // FastSAM - 물체의 중복여부
    std::vector<bool> is_multiple_masks_set;

    // Bolt
    std::vector<geometry_msgs::msg::Pose2D> picking_poses;

    //// Parameters
    double thre_score = 0.0;
    size_t pixel_margin = 15;
    size_t target_detection_id = 0;
    size_t sampling_num = 1;
    double thre_dist_z = 0.005; // 5mm 

    //// Output
    size_t idx_target_mask = 0;
    
    ////
    size_t idx_target_mask_in;
    size_t idx_target_mask_process;
    std::vector<size_t> mask_confident_idx_set;
    std::vector<size_t> pre_selected_idx_set;
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PointXYZRGBA>> cloud_mask_rgba_list; 
	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>, Eigen::aligned_allocator<pcl::PointXYZRGBNormal>> cloud_mask_rgbn_list; 
	std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ>> mask_cloud_base_aligned_list; 
	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>, Eigen::aligned_allocator<pcl::PointXYZRGBNormal>> mask_cloud_base_aligned_rgbn_list; 
    
    std::vector<Eigen::Vector4f> xyz_centroid_list;

    std::vector<std::vector<double>> set_image_picking_position_xyq;
    std::vector<double> hinge_pose_transformation;

};

/**
* @class GRIPPER_DATA
* @brief 그리퍼 데이터 클래스
* @ingroup BIN_PICKING_DATA_CLASS
*/
class GRIPPER_DATA
{
public:
	GRIPPER_DATA() {};
	~GRIPPER_DATA() {};
public:
    std::string file_path;
    std::vector<std::string> gripper_list;
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ>>> set_cloud_gripper_object_frame_list;
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ>>> set_cloud_gripper_tool_frame_list;
    std::vector<std::vector<std::vector<size_t>>> set_gripper_tip_surface_idx_set;
    std::vector<std::vector<uint16_t>> set_gripper_CAD_open_length_list;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> set_T_E2GP;
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ>>> set_cloud_evaluate_grasping_pose_CAD_tool_frame;
};

/**
* @class BIN_DATA
* @brief 상자 데이터 클래스
* @ingroup BIN_PICKING_DATA_CLASS
*/
class BIN_DATA
{
public:
	BIN_DATA() {};
	~BIN_DATA() {};
public:
    std::vector<std::string> bin_list;
    std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ>> cloud_bin_CAD_bin_frame_raw_list;
    std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ>> cloud_bin_CAD_bin_frame_list;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>, Eigen::aligned_allocator<pcl::PointXYZRGBNormal>> cloud_bin_CAD_bin_frame_with_normal_list;
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ>> cloud_bin_CAD_base_frame_list;
    std::vector<octomap::OcTree*> set_OctoMap_bin_CAD;
	std::vector<pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>>> set_md_KdTree_octoMap_bin_CAD;

    Eigen::Matrix4f T_bin_CAD2RobotBase_nominal_JSON;
    Eigen::Matrix4f T_bin_CAD2RobotBase_JSON;
    Eigen::Matrix4f T_bin_CAD2Sensor_JSON;

    double map_bin_voxel_length;
    double query_base_frame_translation_scale;
    std::vector<double> query_base_frame_range;
};

/**
* @class CTARGET_OBJECT_DATA
* @brief 파지 대상 물체의 데이터 클래스
* @ingroup BIN_PICKING_DATA_CLASS
*/
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

    // input
    bool is_JSON_imported = false;
    std::string target_name; // detection target name (JSON file name)
    std::string file_path;
    std::string target_object_name; // object name (object CAD name)
    bool do_load_symmetric_CAD_file = false;
    std::string target_bin_name; // bin name (bin CAD name)
    std::string target_gripper_name; // gripper name (gripper CAD name)
    
    size_t target_id;

    bool is_zig_pose_initial_teaching_process = false;
    std::vector<double> zig_pose_to_be_set_by_user_from_initial_teaching;
    std::vector<double> zig_pose_to_be_saved_from_initial_teaching;
    std::vector<double> object_bin_to_object_teach_pose_to_be_saved_from_initial_teaching;
    std::vector<double> object_bin_to_base_pose_to_be_saved_from_initial_teaching;
    
    std::vector<bool> is_symmetric_plane_object_frame; // ex) xy plane --> [true, true, false]
    
    
    bool do_downsample_CAD_cloud = false;
    bool do_downsample_measured_cloud = false; // in scanning phase
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scan_raw;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_measured;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_measured_rgbn;
    
    Eigen::VectorXf robot_scan_JS_position;
    Eigen::VectorXf robot_scan_CS_pose;
    
    Eigen::VectorXf calibration_delta_q;
    size_t sampling_num;

    // output
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_CAD;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_CAD_for_quick_align;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_CAD_uniformly_downsampled;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_CAD_quick_align_with_mesh_normal;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_CAD_quick_align_with_est_normal;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_CAD_uniformly_downsampled_with_mesh_normal;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_CAD_uniformly_downsampled_with_est_normal;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_CAD_with_normal;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_CAD_with_normal_downsampled;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_CAD;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_CAD_for_quick_align;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_CAD_uniformly_downsampled;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_raw_CAD_quick_align_with_mesh_normal;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_raw_CAD_quick_align_with_est_normal;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_raw_CAD_uniformly_downsampled_with_mesh_normal;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_raw_CAD_uniformly_downsampled_with_est_normal;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_raw_CAD_with_normal;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_raw_CAD_with_normal_downsampled;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_symmetric_CAD;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_symmetric_CAD_for_quick_align;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_symmetric_CAD_uniformly_downsampled;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_symmetric_CAD_quick_align_with_mesh_normal;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_symmetric_CAD_quick_align_with_est_normal;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_symmetric_CAD_uniformly_downsampled_with_mesh_normal;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_symmetric_CAD_uniformly_downsampled_with_est_normal;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_symmetric_CAD_with_normal;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_symmetric_CAD_with_normal_downsampled;

    ////
    std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ>> set_cloud_gripper_CAD_tool_frame_downsampled;
    std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ>> set_cloud_gripper_CAD_uniformly_downsampled;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bin_CAD_uniformly_downsampled_base_frame;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bin_CAD_uniformly_downsampled_bin_frame;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bin_CAD_uniformly_downsampled_bin_frame_raw;
    
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_bin_CAD_uniformly_downsampled_bin_frame_with_normal;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_zig_CAD_uniformly_downsampled_object_frame;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_zig_CAD_uniformly_downsampled_object_frame_raw;


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bin_measured_sensor_frame;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_zig_measured_sensor_frame;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_calibration_tool_CAD_uniformly_downsampled_object_frame;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_calibration_tool_measured_sensor_frame;
    
    // evaluate grasping pose
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_evaluate_grasping_pose_CAD_object_frame;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_evaluate_grasping_pose_CAD_base_aligned;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_evaluate_grasping_pose_measured_sensor_frame;
    std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ>> set_cloud_evaluate_grasping_pose_CAD_tool_frame;



    Eigen::Matrix4f T_bin_CAD2RobotBase_nominal_JSON;
    Eigen::Matrix4f T_bin_CAD2RobotBase_JSON;
    Eigen::Matrix4f T_bin_CAD2Sensor_JSON;

    Eigen::Matrix4f T_base_zig_B2S_JSON;
    Eigen::Matrix4f T_base_zig_B2S_nominal_JSON;   

    Eigen::Matrix4f T_base_zig_E2S_JSON;
    Eigen::Matrix4f T_base_zig_E2S_nominal_JSON;   

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_aligned;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_base_aligned_rgbn;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pre_matching_results;
    Eigen::Matrix4f T_pre_matching;
    Eigen::Matrix4f T_process_matching;
    Eigen::Matrix4f T_process_quick_for_json;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_matching_results;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_matching_results_filtered;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final_results_for_collision_check;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final_multiple_results_for_collision_check;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final_results_for_workspace_optimization;

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_simulation_merged_object_with_bin;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_simulation_merged_object_only;

    // Check collision
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scan_base_aligned_with_bin_collision_check_process;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scan_base_aligned_with_bin_collision_check_raw;

    //
    pcl::PointXYZ cad_min_pt;
    pcl::PointXYZ cad_max_pt;

    bool is_object_pose_flipped = false;
    Eigen::Vector3f measured_object_normal_for_flip_check;
    
    bool is_grasping_pose_flipped = false;
    bool is_feasible_pose = false;
    bool is_collision = false;

    bool is_matching_success = false;

    bool is_base_frame_unknown = false;

    bool is_simulation_object_collision = false;
    bool is_simulation_object_contact_stable = false;
    size_t simulation_contact_stable_cnt = 0;

    
    std::vector<std::vector<size_t>> set_gripper_tip_surface_idx;
    std::vector<uint16_t> set_gripper_CAD_open_length;
    std::vector<double> gripper_tip_dimension;

    // 1) DH parameters
    Eigen::MatrixXf DH_parameters;
    // 2) TCP parameters
    Eigen::Matrix4f T_E2E;
    Eigen::VectorXf robot_tcp;
    Eigen::VectorXf robot_tcp_default;
    // 3) Sensor
    Eigen::Matrix4f T_E2S;
    Eigen::Matrix4f T_B2S_global;
    Eigen::Matrix4f T_B2S_global_nominal;

    std::vector<double> htm_base2sensor_vec;
    
    bool is_eye_in_hand = false;
    
    Eigen::Matrix4f T_B2S;
    Eigen::Matrix4f T_B2E;
    Eigen::Matrix4f T_E2GP;
    

    double robot_min_workspace;
    double robot_max_workspace;

    //// Path planning parameters
    Eigen::VectorXf q_initial;
    double ik_lambda;
    int ik_iteration;
    double ik_position_tolerance;
    double ik_orientation_tolerance;
    double path_planning_control_period;
    std::vector<double> path_planning_acceleration;
    std::vector<double> path_planning_velocity;
    std::vector<double> robot_workspace;

    std::vector<double> bin_ready_cs_pose;

    std::vector<std::vector<double>> set_grp_bounding_box_CAD_frame;
    std::vector<double> object_bounding_box_CAD_frame;
    std::vector<double> object_bounding_box_CAD_symmetric_frame;
    std::vector<double> object_bounding_box_CAD_raw_frame;

    


    Eigen::Matrix4f T_Default2CAD;

    size_t img_rows;
    size_t img_cols;
    // 4) CAD
    Eigen::Matrix4f T_O2GP;
    Eigen::Matrix4f T_O2GP_process;
    Eigen::Matrix4f T_O2GP_final;
    Eigen::Matrix4f T_GP2O;
    Eigen::Vector4f grasping_point_O2GP; // grasping point w.r.t object frame
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> sub_pose_set_T_O2GP;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> sub_pose_set_T_GP2O;

    std::vector<int> sub_pose_set_raw_idx;
    std::vector<uint16_t> sub_pose_gripper_open_length;
    std::vector<uint16_t> sub_pose_gripper_close_length;
    std::vector<uint16_t> sub_pose_gripper_tip_index;


    Eigen::Vector3f vec_E2O_x;
    Eigen::Vector3f vec_E2O_y;
    std::string shape_name;
    std::string grasp_type;

    std::string bin_short_axis;

    float thre_dist_bin_inner_floor_to_mask_centroid;
    double bin_floor_z_value;
    bool do_plane_removal_filtering = false;
    bool do_statistical_filtering = false;
    int statistical_filter_param_K;
    double statistical_filter_param_std;

    bool use_total_scan_data_for_collision_check = false;
    bool do_statistical_filtering_for_collision_check = false;
    int statistical_filter_param_K_for_collision_check;
    double statistical_filter_param_std_for_collision_check;

    double cad_quick_align_voxel_filter_size;
    double cad_raw_voxel_filter_size;

    // evaluate grasping pose
    double eval_gp_cad_raw_voxel_filter_size;
    double eval_gp_map_voxel_length;
    double eval_gp_icp_corr_threshold;

    bool is_orientation_norm_limit_applied = false;
    double orientation_norm_limit;
    bool is_regrasping_required = false;

    bool do_post_processing_for_template_matching = false;
    bool do_post_eucidean_filtering_for_matching_accuracy = false;
    bool do_post_outlier_matching_for_matching_accuracy = false;


    bool mode_learning_based_quick_matching = false;
    bool do_quick_align = false;
    bool quick_align_use_cloud_with_mesh_normal = false;
    bool quick_align_use_feature_with_normal = true;
    double quick_align_z_threshold;
    float quick_align_min_sample_distance;
    float quick_align_max_correspondence_distance;
    int quick_align_iteration;
    int quick_align_sample_num;
    int quick_align_corr_rnd_k;

    FeatureCloud quick_align_cloud_CAD_feature;
    FeatureCloudNormal quick_align_cloud_CAD_feature_with_normal;
    FeatureCloud quick_align_cloud_symmetric_CAD_feature;
    FeatureCloudNormal quick_align_cloud_symmetric_CAD_feature_with_normal;
    FeatureCloud quick_align_cloud_raw_CAD_feature;
    FeatureCloudNormal quick_align_cloud_raw_CAD_feature_with_normal;

    FeatureCloud quick_align_cloud_measured_feature;
    FeatureCloudNormal quick_align_cloud_measured_feature_with_normal;

    
    double icp_corr_threshold;
    double icp_euclidean_fitness_eps; // the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the algorithm is considered to have converged. The error is estimated as the sum of the differences between correspondences in an Euclidean sense, divided by the number of correspondences.
    double icp_transformation_eps; // the transformation epsilon (maximum allowable translation squared difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution. 
    int icp_max_iter_inner;
    int icp_max_iter_outer;

    double matching_accuracy_limit = 0.0;

    float sam_nearby_ratio_threshold = 0.0;

    int detected_mask_num = 0;

    int multi_thread_id = 0;
    int multi_thread_test_num = 0;

    std::vector<uint16_t> object_sub_grasping_pose_select_only_this_pose_with_gripper_open_length;
    bool do_additional_translation_wrt_object_frame = false;

    double cad_height_z = 0.0;
    float cad_cross_length = 0.0;
    double cad_symmetric_height_z = 0.0;
    float cad_symmetric_cross_length = 0.0;
    double cad_raw_height_z = 0.0;
    float cad_raw_cross_length = 0.0;


    bool is_object_complex_shaped = false;
    bool is_orientation_customized = false;
    Eigen::Vector3f custom_vector_major_axis;
    double approach_distance = 0.0;
    Eigen::Matrix4f T_approach_distance;
    size_t trans_count_for_check_collision_approach2escape = 0;
    bool do_check_collision_approach2escape = false;

    bool plot_base_frame_optimization = false;
    bool plot_mask_results = false;
    bool plot_measured_scan_processing_results = false;
    bool plot_measured_euclidean_filtering_results = false;
    bool plot_collision_check_euclidean_filtering_results = false;
    bool plot_collision_check_rgs_filtering_results = false;
    bool plot_collision_check_bin_bounding_box = false;
    bool plot_scan_cloud_for_collision_check = false;
    bool plot_base_aligned_cloud_results = false;
    bool plot_matching_before_icp_results = false;
    bool plot_matching_icp_results = false;
    bool plot_each_mask_bin_bounding_box = false;
    bool plot_post_processing_euclidean_filtering_results = false;
    bool plot_collision_results = false;
    bool plot_approach2escape_collision_results = false;

    Eigen::Matrix4f T_matching_O2B;
    Eigen::Matrix4f T_matching_B2O;
    Eigen::Matrix4f T_matching_B2O_process;
    Eigen::Matrix4f T_matching_B2O_sub_process;
    Eigen::Matrix4f T_matching_B2T; // Tool frame
    
    std::vector<double> grasping_pose; // [m], [deg]
    std::vector<double> grasping_sub_pose; // [m], [deg]
    std::vector<double> zig_pose; // [m], [deg]

    //// Multiple matching results
    std::vector<std::vector<double>> bp_result_set_grasping_pose;
    std::vector<std::vector<std::vector<double>>> bp_result_set_sub_grasping_pose_set;
    std::vector<std::vector<double>> bp_result_set_zig_pose;
    std::vector<double> bp_result_set_matching_accuracy;
    std::vector<size_t> bp_result_set_mask_idx;
    std::vector<bool> bp_result_set_is_pose_feasible; 

    std::vector<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> candidate_approach_pose_set_for_each_mask;
    std::vector<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> candidate_grasping_pose_set_for_each_mask;


    uint16_t gripper_open_length = 0;
    uint16_t gripper_close_length = 0;
    uint16_t gripper_tip_index = 1;
    uint16_t gripper_default_open_length = 0;
    uint16_t gripper_default_close_length = 0;
    uint16_t gripper_default_tip_index = 1;

    //// Sensor view frustum
    std::vector<double> view_frustum_3d_scanner;

    // Bin box bounding box
    bool do_bin_box_filtering = false;
    bool do_collision_check_using_voxel_map = false;
    size_t collision_pt_threshold;
    size_t collision_pt_threshold_escape;

    
    std::vector<double> bin_outward_short_axis_direction;
    std::vector<double> bin_bounding_box; // [m]
    std::vector<double> bin_bounding_box_CAD_frame; // [m]

    double query_base_frame_translation_scale;
    std::vector<double> query_base_frame_range; // [m]
    
    Eigen::Vector3f bin_center_point_bin_frame;
    Eigen::Vector3f bin_center_point_base_frame;
    std::vector<std::vector<double>> bin_minmax;
    std::vector<double> bin_length;

    // Voxel map
    double map_voxel_length;
    double map_gripper_voxel_length;
    
    bool do_euclidean_filtering = false;
    double euclidean_cluster_tolerance;
    size_t euclidean_cluster_min_size;
    size_t euclidean_cluster_max_size;

    bool do_euclidean_filtering_for_collision_check = false;
    double euclidean_cluster_tolerance_for_collision_check;
    size_t euclidean_cluster_min_size_for_collision_check;
    size_t euclidean_cluster_max_size_for_collision_check;

    bool do_euclidean_filtering_for_multiple_masks = false;
    double euclidean_cluster_tolerance_for_multiple_masks;
    size_t euclidean_cluster_min_size_for_multiple_masks;
    size_t euclidean_cluster_max_size_for_multiple_masks;

    // rgs filtering
    bool do_rgs_filtering_for_collision_check = false;
    int rgs_normal_K_for_collision_check;
    int rgs_min_size_for_collision_check;
    int rgs_max_size_for_collision_check; 
    double rgs_thre_angle_for_collision_check; 
    double rgs_thre_curvature_for_collision_check; 
    int rgs_neighbor_K_for_collision_check;
    
    octomap::OcTree* m_OctoMap_CAD;
    octomap::OcTree* m_OctoMap_raw_CAD;
    octomap::OcTree* m_OctoMap_symmetric_CAD;
    
    octomap::OcTree* m_OctoMap_gripper_CAD;
    octomap::OcTree* m_OctoMap_bin_CAD;
    octomap::OcTree* m_OctoMap_calibration_tool_CAD;
    octomap::OcTree* m_OctoMap_base_zig_CAD;
    octomap::OcTree* m_OctoMap_simulation_process;

	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> md_KdTree_octoMap_CAD;
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> md_KdTree_octoMap_raw_CAD;
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> md_KdTree_octoMap_symmetric_CAD;
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> md_KdTree_octoMap_bin_CAD;
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> md_KdTree_octoMap_calibration_tool_CAD;
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> md_KdTree_octoMap_base_zig_CAD;
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> md_KdTree_octoMap_objects_with_bin_simulation_process;
    
    // evaluate grasping pose
    octomap::OcTree* m_OctoMap_evaluate_grasping_pose_CAD;
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> md_KdTree_octoMap_evaluate_grasping_pose_CAD;


    std::vector<octomap::OcTree*> m_set_OctoMap_gripper_CAD;
    std::vector<octomap::OcTree*> m_set_OctoMap_gripper_CAD_tool_frame;
	std::vector<pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>>> set_md_KdTree_octoMap_gripper_CAD;
	std::vector<pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>>> set_md_KdTree_octoMap_gripper_CAD_tool_frame;


	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal, flann::L2_Simple<float>> md_KdTree_scan_for_image_based_picking;


    int gripper_idx_now = 0;

    double pre_matching_accuracy;
    double matching_accuracy;

    std::vector<double> custom_transformation_pose;
    std::vector<double> process_transformation_pose;
    
    // Bin matching
    bool initial_phase_bin_proc_do_filtering = false;
    bool initial_phase_bin_proc_plot_filtering_result = false;

    // Base zig matching
    bool initial_phase_base_zig_proc_do_filtering = false;
    bool initial_phase_base_zig_proc_plot_filtering_result = false;


    
    

    // Robot-camera calibration
    std::vector<double> r2cam_calibration_JS_position;
    Eigen::Matrix4f T_bin_robot_camera_calibration_nominal;
    std::vector<double> pose_B2E_measured;
    
    
    ///////////////////////
    // Mask RCNN data
    MASK_DATA mask_data;

    // Quick CAD learning data
    std::vector<std::vector<double>> color_map;

    // Optimization data
    COPTCONDITIONLIST optimization_input;

    bool do_re_matching = false;
    int cnt_re_matching = 0;
    bool select_sub_pose = false;

    bool debug_mode = false;
    size_t test_cnt_1 = 0;
    size_t test_cnt_2 = 0;

    bool print_log = false;

    bool is_image_based_picking_process = false;

    bool do_matching_with_image_info = false;

    bool is_symmetric_CAD = false;

    std::string matching_json_total_folder_path;
    std::string json_path_for_quick_align;

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////// SIMULATION /////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////

    //// Simulation
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_process_object_CAD_simulation;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr collision_cloud_in;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr collision_cloud_map;
    pcl::PointIndices::Ptr simulation_collision_idx_cloud_in;
    pcl::PointIndices::Ptr simulation_collision_idx_cloud_map;

    // size_t idx_simulation_object_center_of_mass;
    Eigen::Vector4f simulation_center_of_mass_object_frame;
    Eigen::Vector3f simulation_center_of_mass_process;
    Eigen::Vector3f simulation_center_of_mass_process_before;

    Eigen::Vector3f r_vector_com2contact;
    Eigen::Vector3f contact_pt;
    Eigen::Vector3f contact_normal;

    RigidBody simulation_object;

    float mass_simulation_object = 0.0;
    Eigen::Matrix3f inertia_matrix_simulation_object;
    Eigen::Matrix4f T_process_simulation;
    Eigen::Matrix4f T_bin2Contact;
    Eigen::Matrix4f T_contact2Bin;
    Eigen::Matrix4f T_gravity;

    std::vector<size_t> idx_set_contact_point;

    // Contact nearest cloud
	// std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>, Eigen::aligned_allocator<pcl::PointXYZRGBNormal> > m_set_cloud_contact_nearest;
	std::vector<pcl::PointIndices, Eigen::aligned_allocator<pcl::PointIndices> > m_set_idx_contact_nearest;

    // Robot model
	CRobotModel robot_model;

    // PhysicsWorld physics_word;
    PhysicsObject physics_object;


    //// Put object
    std::vector<hanyang_matching_msgs::msg::LearningInput> learning_input_set;

};

