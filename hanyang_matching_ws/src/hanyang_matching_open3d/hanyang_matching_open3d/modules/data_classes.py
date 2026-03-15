"""
Data classes for Open3D-based template matching.
Equivalent to object_data.h in PCL version.
"""

import numpy as np
import open3d as o3d
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from enum import Enum


class ShapeType(Enum):
    """Object shape types"""
    CYLINDER = "cylinder"
    COMPLEX = "complex"
    BOX = "box"


class GraspType(Enum):
    """Grasp type"""
    TOP = "top"
    SIDE = "side"
    CUSTOM = "custom"


@dataclass
class MaskData:
    """Mask detection data from SAM/Mask-RCNN"""
    target_id: int = 0
    detection_mode: str = "sam"
    class_ids: List[int] = field(default_factory=list)
    class_names: List[str] = field(default_factory=list)
    scores: List[float] = field(default_factory=list)
    class_cnt: int = 0
    
    # SAM-specific
    ext_nearby_ratio: List[float] = field(default_factory=list)
    is_multiple_masks_set: List[bool] = field(default_factory=list)
    
    # Parameters
    thre_score: float = 0.0
    pixel_margin: int = 15
    target_detection_id: int = 0
    sampling_num: int = 1
    thre_dist_z: float = 0.005  # 5mm
    
    # Output
    idx_target_mask: int = 0
    idx_target_mask_in: int = 0
    idx_target_mask_process: int = 0
    mask_confident_idx_set: List[int] = field(default_factory=list)
    pre_selected_idx_set: List[int] = field(default_factory=list)
    
    # Point clouds for each mask
    cloud_mask_list: List[o3d.geometry.PointCloud] = field(default_factory=list)
    mask_cloud_base_aligned_list: List[o3d.geometry.PointCloud] = field(default_factory=list)
    xyz_centroid_list: List[np.ndarray] = field(default_factory=list)


@dataclass
class GripperData:
    """Gripper configuration data"""
    file_path: str = ""
    gripper_list: List[str] = field(default_factory=list)
    cloud_gripper_object_frame_list: List[List[o3d.geometry.PointCloud]] = field(default_factory=list)
    cloud_gripper_tool_frame_list: List[List[o3d.geometry.PointCloud]] = field(default_factory=list)
    gripper_tip_surface_idx_set: List[List[List[int]]] = field(default_factory=list)
    gripper_CAD_open_length_list: List[List[int]] = field(default_factory=list)
    T_E2GP_set: List[np.ndarray] = field(default_factory=list)  # 4x4 transformation matrices
    cloud_evaluate_grasping_pose_CAD_tool_frame: List[List[o3d.geometry.PointCloud]] = field(default_factory=list)


@dataclass
class BinData:
    """Bin configuration data"""
    bin_list: List[str] = field(default_factory=list)
    cloud_bin_CAD_bin_frame_raw_list: List[o3d.geometry.PointCloud] = field(default_factory=list)
    cloud_bin_CAD_bin_frame_list: List[o3d.geometry.PointCloud] = field(default_factory=list)
    cloud_bin_CAD_base_frame_list: List[o3d.geometry.PointCloud] = field(default_factory=list)
    
    # Voxel grids for collision checking
    voxel_grid_bin_CAD_list: List[o3d.geometry.VoxelGrid] = field(default_factory=list)
    
    # Transformations
    T_bin_CAD2RobotBase_nominal_JSON: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_bin_CAD2RobotBase_JSON: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_bin_CAD2Sensor_JSON: np.ndarray = field(default_factory=lambda: np.eye(4))
    
    map_bin_voxel_length: float = 0.001  # 1mm
    query_base_frame_translation_scale: float = 1.0
    query_base_frame_range: List[float] = field(default_factory=list)


@dataclass
class AlignParameters:
    """Alignment/Registration parameters"""
    # Quick align (RANSAC-based feature matching)
    do_quick_align: bool = False
    quick_align_use_feature_with_normal: bool = True
    quick_align_z_threshold: float = 0.1
    quick_align_min_sample_distance: float = 0.005
    quick_align_max_correspondence_distance: float = 0.05
    quick_align_iteration: int = 1000
    quick_align_sample_num: int = 4
    quick_align_corr_rnd_k: int = 5
    
    # ICP parameters
    icp_corr_threshold: float = 0.05  # 50mm correspondence distance
    icp_euclidean_fitness_eps: float = 0.001
    icp_transformation_eps: float = 0.001
    icp_max_iter_inner: int = 50
    icp_max_iter_outer: int = 10
    
    # Normal estimation
    normal_radius: float = 0.01  # 10mm
    normal_max_nn: int = 30
    
    # FPFH feature parameters
    fpfh_radius: float = 0.05  # 50mm
    fpfh_max_nn: int = 100


@dataclass
class TargetObjectData:
    """
    Main data class for target object.
    Equivalent to CTARGET_OBJECT_DATA in PCL version.
    """
    # Input configuration
    is_JSON_imported: bool = False
    target_name: str = ""
    file_path: str = ""
    target_object_name: str = ""
    target_bin_name: str = ""
    target_gripper_name: str = ""
    target_id: int = 0
    
    # Symmetric handling
    do_load_symmetric_CAD_file: bool = False
    is_symmetric_plane_object_frame: List[bool] = field(default_factory=lambda: [False, False, False])
    is_symmetric_CAD: bool = False
    
    # Scan data
    do_downsample_CAD_cloud: bool = False
    do_downsample_measured_cloud: bool = False
    cloud_scan_raw: Optional[o3d.geometry.PointCloud] = None
    cloud_measured: Optional[o3d.geometry.PointCloud] = None
    sampling_num: int = 1
    
    # Robot pose
    robot_scan_JS_position: np.ndarray = field(default_factory=lambda: np.zeros(6))
    robot_scan_CS_pose: np.ndarray = field(default_factory=lambda: np.zeros(6))
    
    # CAD models
    cloud_CAD: Optional[o3d.geometry.PointCloud] = None
    cloud_CAD_for_quick_align: Optional[o3d.geometry.PointCloud] = None
    cloud_CAD_uniformly_downsampled: Optional[o3d.geometry.PointCloud] = None
    cloud_CAD_with_normal: Optional[o3d.geometry.PointCloud] = None
    cloud_CAD_with_normal_downsampled: Optional[o3d.geometry.PointCloud] = None
    
    # Symmetric CAD
    cloud_symmetric_CAD: Optional[o3d.geometry.PointCloud] = None
    cloud_symmetric_CAD_for_quick_align: Optional[o3d.geometry.PointCloud] = None
    cloud_symmetric_CAD_uniformly_downsampled: Optional[o3d.geometry.PointCloud] = None
    cloud_symmetric_CAD_with_normal: Optional[o3d.geometry.PointCloud] = None
    
    # Features
    cloud_CAD_fpfh: Optional[o3d.pipelines.registration.Feature] = None
    cloud_symmetric_CAD_fpfh: Optional[o3d.pipelines.registration.Feature] = None
    cloud_measured_fpfh: Optional[o3d.pipelines.registration.Feature] = None
    
    # Bin/Gripper CAD
    cloud_bin_CAD_uniformly_downsampled_base_frame: Optional[o3d.geometry.PointCloud] = None
    cloud_bin_CAD_uniformly_downsampled_bin_frame: Optional[o3d.geometry.PointCloud] = None
    cloud_base_zig_CAD_uniformly_downsampled_object_frame: Optional[o3d.geometry.PointCloud] = None
    
    # Gripper clouds
    set_cloud_gripper_CAD_tool_frame_downsampled: List[o3d.geometry.PointCloud] = field(default_factory=list)
    set_cloud_gripper_CAD_uniformly_downsampled: List[o3d.geometry.PointCloud] = field(default_factory=list)
    
    # Transformations
    T_bin_CAD2RobotBase_nominal_JSON: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_bin_CAD2RobotBase_JSON: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_bin_CAD2Sensor_JSON: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_base_zig_B2S_JSON: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_base_zig_B2S_nominal_JSON: np.ndarray = field(default_factory=lambda: np.eye(4))
    
    # Processed clouds
    cloud_base_aligned: Optional[o3d.geometry.PointCloud] = None
    cloud_pre_matching_results: Optional[o3d.geometry.PointCloud] = None
    T_pre_matching: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_process_matching: np.ndarray = field(default_factory=lambda: np.eye(4))
    
    cloud_matching_results: Optional[o3d.geometry.PointCloud] = None
    cloud_matching_results_filtered: Optional[o3d.geometry.PointCloud] = None
    cloud_final_results_for_collision_check: Optional[o3d.geometry.PointCloud] = None
    
    # Collision check cloud
    cloud_scan_base_aligned_with_bin_collision_check_process: Optional[o3d.geometry.PointCloud] = None
    cloud_scan_base_aligned_with_bin_collision_check_raw: Optional[o3d.geometry.PointCloud] = None
    
    # Bounding boxes
    cad_min_pt: np.ndarray = field(default_factory=lambda: np.zeros(3))
    cad_max_pt: np.ndarray = field(default_factory=lambda: np.zeros(3))
    object_bounding_box_CAD_frame: List[float] = field(default_factory=list)
    object_bounding_box_CAD_symmetric_frame: List[float] = field(default_factory=list)
    
    # Status flags
    is_object_pose_flipped: bool = False
    measured_object_normal_for_flip_check: np.ndarray = field(default_factory=lambda: np.zeros(3))
    is_grasping_pose_flipped: bool = False
    is_feasible_pose: bool = False
    is_collision: bool = False
    is_matching_success: bool = False
    is_base_frame_unknown: bool = False
    
    # Gripper parameters
    gripper_open_length: int = 0
    gripper_close_length: int = 0
    gripper_tip_index: int = 1
    gripper_default_open_length: int = 0
    gripper_default_close_length: int = 0
    gripper_default_tip_index: int = 1
    gripper_tip_dimension: List[float] = field(default_factory=list)
    set_gripper_tip_surface_idx: List[List[int]] = field(default_factory=list)
    set_gripper_CAD_open_length: List[int] = field(default_factory=list)
    
    # Robot parameters
    DH_parameters: np.ndarray = field(default_factory=lambda: np.zeros((6, 4)))
    T_E2E: np.ndarray = field(default_factory=lambda: np.eye(4))  # TCP transformation
    robot_tcp: np.ndarray = field(default_factory=lambda: np.zeros(6))
    robot_tcp_default: np.ndarray = field(default_factory=lambda: np.zeros(6))
    T_E2S: np.ndarray = field(default_factory=lambda: np.eye(4))  # End-effector to Sensor
    T_B2S_global: np.ndarray = field(default_factory=lambda: np.eye(4))  # Base to Sensor
    T_B2S_global_nominal: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_B2S: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_B2E: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_E2GP: np.ndarray = field(default_factory=lambda: np.eye(4))  # End-effector to Grasp Point
    
    is_eye_in_hand: bool = False
    htm_base2sensor_vec: List[float] = field(default_factory=list)
    
    robot_min_workspace: float = 0.0
    robot_max_workspace: float = 0.0
    robot_workspace: List[float] = field(default_factory=list)
    
    # Path planning parameters
    q_initial: np.ndarray = field(default_factory=lambda: np.zeros(6))
    ik_lambda: float = 0.1
    ik_iteration: int = 100
    ik_position_tolerance: float = 0.001
    ik_orientation_tolerance: float = 0.01
    path_planning_control_period: float = 0.01
    path_planning_acceleration: List[float] = field(default_factory=list)
    path_planning_velocity: List[float] = field(default_factory=list)
    
    bin_ready_cs_pose: List[float] = field(default_factory=list)
    
    # Grasp pose
    T_Default2CAD: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_O2GP: np.ndarray = field(default_factory=lambda: np.eye(4))  # Object to Grasp Point
    T_O2GP_process: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_O2GP_final: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_GP2O: np.ndarray = field(default_factory=lambda: np.eye(4))
    grasping_point_O2GP: np.ndarray = field(default_factory=lambda: np.zeros(4))
    sub_pose_set_T_O2GP: List[np.ndarray] = field(default_factory=list)
    sub_pose_set_T_GP2O: List[np.ndarray] = field(default_factory=list)
    
    sub_pose_set_raw_idx: List[int] = field(default_factory=list)
    sub_pose_gripper_open_length: List[int] = field(default_factory=list)
    sub_pose_gripper_close_length: List[int] = field(default_factory=list)
    sub_pose_gripper_tip_index: List[int] = field(default_factory=list)
    
    # Shape and grasp type
    shape_name: str = ShapeType.COMPLEX.value
    grasp_type: str = GraspType.TOP.value
    bin_short_axis: str = "z"
    
    # Filtering parameters
    thre_dist_bin_inner_floor_to_mask_centroid: float = 0.0
    bin_floor_z_value: float = 0.0
    do_plane_removal_filtering: bool = False
    do_statistical_filtering: bool = False
    statistical_filter_param_K: int = 50
    statistical_filter_param_std: float = 1.0
    
    use_total_scan_data_for_collision_check: bool = False
    do_statistical_filtering_for_collision_check: bool = False
    statistical_filter_param_K_for_collision_check: int = 50
    statistical_filter_param_std_for_collision_check: float = 1.0
    
    # Voxel parameters
    cad_quick_align_voxel_filter_size: float = 0.005  # 5mm
    cad_raw_voxel_filter_size: float = 0.001  # 1mm
    
    # Post-processing
    do_post_processing_for_template_matching: bool = False
    do_post_eucidean_filtering_for_matching_accuracy: bool = False
    do_post_outlier_matching_for_matching_accuracy: bool = False
    
    # Alignment parameters
    align_params: AlignParameters = field(default_factory=AlignParameters)
    
    matching_accuracy_limit: float = 0.0
    sam_nearby_ratio_threshold: float = 0.0
    detected_mask_num: int = 0
    
    # Multi-threading
    multi_thread_id: int = 0
    multi_thread_test_num: int = 0
    
    # CAD dimensions
    cad_height_z: float = 0.0
    cad_cross_length: float = 0.0
    cad_symmetric_height_z: float = 0.0
    cad_symmetric_cross_length: float = 0.0
    
    # Complex shape handling
    is_object_complex_shaped: bool = False
    is_orientation_customized: bool = False
    custom_vector_major_axis: np.ndarray = field(default_factory=lambda: np.zeros(3))
    approach_distance: float = 0.0
    T_approach_distance: np.ndarray = field(default_factory=lambda: np.eye(4))
    trans_count_for_check_collision_approach2escape: int = 0
    do_check_collision_approach2escape: bool = False
    
    # Visualization flags
    plot_base_frame_optimization: bool = False
    plot_mask_results: bool = False
    plot_measured_scan_processing_results: bool = False
    plot_measured_euclidean_filtering_results: bool = False
    plot_collision_check_euclidean_filtering_results: bool = False
    plot_base_aligned_cloud_results: bool = False
    plot_matching_before_icp_results: bool = False
    plot_matching_icp_results: bool = False
    plot_each_mask_bin_bounding_box: bool = False
    plot_post_processing_euclidean_filtering_results: bool = False
    plot_collision_results: bool = False
    plot_approach2escape_collision_results: bool = False
    
    # Matching results
    T_matching_O2B: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_matching_B2O: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_matching_B2O_process: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_matching_B2T: np.ndarray = field(default_factory=lambda: np.eye(4))  # Tool frame
    
    grasping_pose: List[float] = field(default_factory=list)  # [m], [deg]
    grasping_sub_pose: List[float] = field(default_factory=list)
    zig_pose: List[float] = field(default_factory=list)
    
    # Multiple matching results
    bp_result_set_grasping_pose: List[List[float]] = field(default_factory=list)
    bp_result_set_sub_grasping_pose_set: List[List[List[float]]] = field(default_factory=list)
    bp_result_set_zig_pose: List[List[float]] = field(default_factory=list)
    bp_result_set_matching_accuracy: List[float] = field(default_factory=list)
    bp_result_set_mask_idx: List[int] = field(default_factory=list)
    bp_result_set_is_pose_feasible: List[bool] = field(default_factory=list)
    
    # Sensor view frustum
    view_frustum_3d_scanner: List[float] = field(default_factory=list)
    
    # Bin bounding box
    do_bin_box_filtering: bool = False
    do_collision_check_using_voxel_map: bool = False
    collision_pt_threshold: int = 100
    collision_pt_threshold_escape: int = 50
    
    bin_outward_short_axis_direction: List[float] = field(default_factory=list)
    bin_bounding_box: List[float] = field(default_factory=list)  # [m]
    bin_bounding_box_CAD_frame: List[float] = field(default_factory=list)
    
    query_base_frame_translation_scale: float = 1.0
    query_base_frame_range: List[float] = field(default_factory=list)
    
    bin_center_point_bin_frame: np.ndarray = field(default_factory=lambda: np.zeros(3))
    bin_center_point_base_frame: np.ndarray = field(default_factory=lambda: np.zeros(3))
    bin_minmax: List[List[float]] = field(default_factory=list)
    bin_length: List[float] = field(default_factory=list)
    
    # Voxel map
    map_voxel_length: float = 0.001  # 1mm
    map_gripper_voxel_length: float = 0.001
    voxel_grid_CAD: Optional[o3d.geometry.VoxelGrid] = None
    voxel_grid_symmetric_CAD: Optional[o3d.geometry.VoxelGrid] = None
    voxel_grid_bin_CAD: Optional[o3d.geometry.VoxelGrid] = None
    voxel_grid_gripper_CAD: Optional[o3d.geometry.VoxelGrid] = None
    set_voxel_grid_gripper_CAD: List[o3d.geometry.VoxelGrid] = field(default_factory=list)
    
    # Euclidean filtering
    do_euclidean_filtering: bool = False
    euclidean_cluster_tolerance: float = 0.01  # 10mm
    euclidean_cluster_min_size: int = 100
    euclidean_cluster_max_size: int = 25000
    
    do_euclidean_filtering_for_collision_check: bool = False
    euclidean_cluster_tolerance_for_collision_check: float = 0.02
    euclidean_cluster_min_size_for_collision_check: int = 100
    euclidean_cluster_max_size_for_collision_check: int = 25000
    
    do_euclidean_filtering_for_multiple_masks: bool = False
    euclidean_cluster_tolerance_for_multiple_masks: float = 0.01
    euclidean_cluster_min_size_for_multiple_masks: int = 100
    euclidean_cluster_max_size_for_multiple_masks: int = 25000
    
    # Accuracy
    pre_matching_accuracy: float = 0.0
    matching_accuracy: float = 0.0
    
    # Custom transformation
    custom_transformation_pose: List[float] = field(default_factory=list)
    process_transformation_pose: List[float] = field(default_factory=list)
    
    # Mask data
    mask_data: MaskData = field(default_factory=MaskData)
    
    # Debug
    do_re_matching: bool = False
    cnt_re_matching: int = 0
    select_sub_pose: bool = False
    debug_mode: bool = False
    test_cnt_1: int = 0
    test_cnt_2: int = 0
    print_log: bool = False
    
    is_image_based_picking_process: bool = False
    do_matching_with_image_info: bool = False

