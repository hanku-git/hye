"""
Template matching module - equivalent to template_matching.cpp
Main class for bin picking template matching using Open3D.
"""

import numpy as np
import open3d as o3d
import json
import os
import copy
from typing import List, Tuple, Optional, Dict
from pathlib import Path

from .data_classes import (
    TargetObjectData, MaskData, GripperData, BinData,
    AlignParameters, ShapeType, GraspType
)
from .pcl_processing import PCLProcessing
from .mask_processing import MaskProcessing
from .grasp_planning import GraspPlanning
from .collision_checker import CollisionChecker
from .utils import (
    pose_vec_to_matrix, matrix_to_pose_vec,
    remove_nan_points, filter_by_view_frustum,
    cloud_scaling, compute_transform_from_dh_and_js,
    estimate_normals_with_sensor_origin, evaluate_matching_accuracy,
    evaluate_matching_accuracy_improved, paint_uniform_color, merge_point_clouds, compute_centroid
)


class TemplateMatching:
    """
    Main template matching class for bin picking.
    Equivalent to TEMPLATE_MATCHING in PCL version.
    """
    
    def __init__(self, package_path: str = ""):
        """
        Initialize template matching module.
        
        Args:
            package_path: Path to package share directory
        """
        self.package_path = package_path
        self.home_folder_path = str(Path.home())
        
        # Processing modules
        self.pcl_proc = PCLProcessing()
        self.mask_proc = MaskProcessing()
        self.grasp_plan = GraspPlanning()
        self.collision_check = CollisionChecker()
        
        # Template lists
        self.template_list: List[TargetObjectData] = []
        self.template_tool_list: List[TargetObjectData] = []
        
        # Gripper and bin data
        self.gripper_data = GripperData()
        self.bin_data = BinData()
        
        # Visualization control
        self.enable_visualization = False
        
        print("TemplateMatching initialized with all modules")
    
    def set_visualization_enabled(self, enabled: bool) -> None:
        """Enable or disable visualization"""
        self.enable_visualization = enabled
        print(f"Visualization {'enabled' if enabled else 'disabled'}")
    
    def initialize_template(self, target_idx: int, 
                          robot_dh: List[float],
                          robot_tcp_default: List[float],
                          robot_tcp: List[float]) -> TargetObjectData:
        """
        Initialize target object template.
        
        Args:
            target_idx: Target object index
            robot_dh: DH parameters (24 values: 6 joints x 4 params)
            robot_tcp_default: Default TCP [x, y, z, rx, ry, rz]
            robot_tcp: Current TCP [x, y, z, rx, ry, rz]
            
        Returns:
            Initialized target object data
        """
        target_data = TargetObjectData()
        target_data.target_id = target_idx
        
        # Set robot parameters
        target_data.DH_parameters = np.array(robot_dh).reshape(6, 4)
        target_data.robot_tcp_default = np.array(robot_tcp_default)
        target_data.robot_tcp = np.array(robot_tcp)
        
        # Initialize point clouds
        target_data.cloud_scan_raw = o3d.geometry.PointCloud()
        target_data.cloud_measured = o3d.geometry.PointCloud()
        target_data.cloud_CAD = o3d.geometry.PointCloud()
        target_data.cloud_base_aligned = o3d.geometry.PointCloud()
        target_data.cloud_matching_results = o3d.geometry.PointCloud()
        
        # Add to template list (ensure proper indexing)
        while len(self.template_list) <= target_idx:
            self.template_list.append(None)
        
        self.template_list[target_idx] = target_data
        
        print(f"Template {target_idx} initialized")
        
        return target_data
    
    def load_cad_model(self, target_data: TargetObjectData,
                      cad_file_path: str,
                      voxel_size: float = 0.005) -> bool:
        """
        Load CAD model from file.
        
        Args:
            target_data: Target object data
            cad_file_path: Path to CAD file (PLY, PCD, etc.)
            voxel_size: Voxel size for downsampling
            
        Returns:
            True if successful
        """
        try:
            # Load CAD model
            if cad_file_path.endswith('.ply'):
                cloud_cad = o3d.io.read_point_cloud(cad_file_path)
            elif cad_file_path.endswith('.pcd'):
                cloud_cad = o3d.io.read_point_cloud(cad_file_path)
            else:
                print(f"Unsupported file format: {cad_file_path}")
                return False
            
            if len(cloud_cad.points) == 0:
                print(f"Empty point cloud: {cad_file_path}")
                return False
            
            print(f"Loaded CAD model: {len(cloud_cad.points)} points")
            
            # Store original CAD
            target_data.cloud_CAD = cloud_cad
            
            # Create downsampled version for quick align
            target_data.cloud_CAD_for_quick_align = cloud_cad.voxel_down_sample(voxel_size)
            
            # Create uniformly downsampled version
            if target_data.do_downsample_CAD_cloud:
                target_data.cloud_CAD_uniformly_downsampled = cloud_cad.voxel_down_sample(
                    target_data.cad_raw_voxel_filter_size
                )
            else:
                target_data.cloud_CAD_uniformly_downsampled = cloud_cad
            
            # Estimate normals for CAD model
            target_data.cloud_CAD_with_normal = target_data.cloud_CAD_uniformly_downsampled
            target_data.cloud_CAD_with_normal = self.pcl_proc.estimate_normals(
                target_data.cloud_CAD_with_normal,
                radius=target_data.align_params.normal_radius,
                max_nn=target_data.align_params.normal_max_nn
            )
            
            # Compute FPFH features for quick align
            if target_data.align_params.do_quick_align:
                target_data.cloud_CAD_fpfh = self.pcl_proc.compute_fpfh_feature(
                    target_data.cloud_CAD_for_quick_align,
                    radius=target_data.align_params.fpfh_radius,
                    max_nn=target_data.align_params.fpfh_max_nn
                )
                print(f"FPFH features computed: {target_data.cloud_CAD_fpfh.num()} features")
            
            # Compute bounding box
            bbox = target_data.cloud_CAD.get_axis_aligned_bounding_box()
            target_data.cad_min_pt = np.asarray(bbox.min_bound)
            target_data.cad_max_pt = np.asarray(bbox.max_bound)
            
            # Compute dimensions
            dimensions = target_data.cad_max_pt - target_data.cad_min_pt
            target_data.cad_height_z = dimensions[2]
            target_data.cad_cross_length = max(dimensions[0], dimensions[1])
            
            print(f"CAD dimensions: {dimensions}")
            
            # Create voxel grid for collision checking
            if target_data.do_collision_check_using_voxel_map:
                target_data.voxel_grid_CAD = self.pcl_proc.create_voxel_grid(
                    target_data.cloud_CAD_uniformly_downsampled,
                    voxel_size=target_data.map_voxel_length
                )
            
            return True
            
        except Exception as e:
            print(f"Error loading CAD model: {e}")
            return False
    
    def import_target_json_files(self, target_data: TargetObjectData,
                                json_dir: str) -> bool:
        """
        Import target object configuration from JSON files.
        
        Args:
            target_data: Target object data
            json_dir: Directory containing JSON configuration files
            
        Returns:
            True if successful
        """
        try:
            # Load target object config
            target_json_path = os.path.join(json_dir, f"{target_data.target_name}.json")
            
            if not os.path.exists(target_json_path):
                print(f"Target JSON not found: {target_json_path}")
                return False
            
            with open(target_json_path, 'r') as f:
                config = json.load(f)
            
            # Parse configuration
            target_data.target_object_name = config.get('object_name', '')
            target_data.target_bin_name = config.get('bin_name', '')
            target_data.target_gripper_name = config.get('gripper_name', '')
            target_data.shape_name = config.get('shape_name', 'complex')
            target_data.grasp_type = config.get('grasp_type', 'top')
            
            # Alignment parameters
            if 'alignment' in config:
                align_cfg = config['alignment']
                target_data.align_params.do_quick_align = align_cfg.get('do_quick_align', False)
                target_data.align_params.icp_corr_threshold = align_cfg.get('icp_corr_threshold', 0.05)
                target_data.align_params.icp_max_iter_inner = align_cfg.get('icp_max_iter_inner', 50)
                target_data.align_params.quick_align_max_correspondence_distance = align_cfg.get(
                    'quick_align_max_correspondence_distance', 0.05
                )
            
            # Filtering parameters
            if 'filtering' in config:
                filt_cfg = config['filtering']
                target_data.do_euclidean_filtering = filt_cfg.get('do_euclidean_filtering', False)
                target_data.euclidean_cluster_tolerance = filt_cfg.get('euclidean_cluster_tolerance', 0.01)
                target_data.euclidean_cluster_min_size = filt_cfg.get('euclidean_cluster_min_size', 100)
                target_data.euclidean_cluster_max_size = filt_cfg.get('euclidean_cluster_max_size', 25000)
            
            # Voxel parameters
            if 'voxel' in config:
                voxel_cfg = config['voxel']
                target_data.map_voxel_length = voxel_cfg.get('map_voxel_length', 0.001)
                target_data.cad_raw_voxel_filter_size = voxel_cfg.get('cad_raw_voxel_filter_size', 0.001)
            
            # View frustum
            if 'view_frustum' in config:
                target_data.view_frustum_3d_scanner = config['view_frustum']
            
            # Bin bounding box
            if 'bin_bounding_box' in config:
                target_data.bin_bounding_box = config['bin_bounding_box']
                target_data.do_bin_box_filtering = True
            
            # Collision check
            if 'collision' in config:
                coll_cfg = config['collision']
                target_data.do_collision_check_using_voxel_map = coll_cfg.get('enabled', False)
                target_data.collision_pt_threshold = coll_cfg.get('threshold', 100)
            
            target_data.is_JSON_imported = True
            print(f"JSON configuration loaded for {target_data.target_name}")
            
            return True
            
        except Exception as e:
            print(f"Error loading JSON configuration: {e}")
            return False
    
    def do_measured_cloud_processing(self, target_data: TargetObjectData) -> None:
        """
        Process measured point cloud (filtering, alignment).
        
        Args:
            target_data: Target object data with cloud_measured set
        """
        print("Processing measured cloud...")
        
        # Remove NaN points
        target_data.cloud_measured = self.pcl_proc.remove_nan_points(target_data.cloud_measured)
        
        # Filter using view frustum if available
        if len(target_data.view_frustum_3d_scanner) == 6:
            print(f"Applying view frustum filter: {target_data.view_frustum_3d_scanner}")
            print(f"Points before view frustum: {len(target_data.cloud_measured.points)}")
            target_data.cloud_measured = self.pcl_proc.filter_cloud_using_view_frustum(
                target_data.cloud_measured,
                target_data.view_frustum_3d_scanner
            )
            print(f"Points after view frustum: {len(target_data.cloud_measured.points)}")
        else:
            print(f"View frustum not applied: length={len(target_data.view_frustum_3d_scanner)}")
        
        # Statistical outlier removal if enabled
        if target_data.do_statistical_filtering:
            target_data.cloud_measured = self.pcl_proc.statistical_outlier_removal(
                target_data.cloud_measured,
                nb_neighbors=target_data.statistical_filter_param_K,
                std_ratio=target_data.statistical_filter_param_std
            )
        
        # Downsampling if enabled
        if target_data.do_downsample_measured_cloud:
            voxel_size = target_data.cad_raw_voxel_filter_size
            target_data.cloud_measured = self.pcl_proc.voxel_down_sample(
                target_data.cloud_measured, voxel_size
            )
        
        print(f"Measured cloud processed: {len(target_data.cloud_measured.points)} points")
    
    def do_alignment_scan2robot_base_frame(self, target_data: TargetObjectData) -> None:
        """
        Transform scan cloud from sensor frame to robot base frame.
        
        Args:
            target_data: Target object data
        """
        # Compute transformation from sensor to base
        # T_B2E: base to end-effector (from forward kinematics)
        T_B2E = compute_transform_from_dh_and_js(
            target_data.DH_parameters,
            target_data.robot_scan_JS_position,
            target_data.robot_tcp
        )
        
        target_data.T_B2E = T_B2E
        
        # T_E2S: end-effector to sensor (from calibration or config)
        T_E2S = target_data.T_E2S
        
        # T_B2S = T_B2E @ T_E2S
        T_B2S = T_B2E @ T_E2S
        target_data.T_B2S = T_B2S
        
        # Transform cloud
        target_data.cloud_base_aligned = target_data.cloud_measured.transform(T_B2S)
        
        print("Cloud aligned to robot base frame")
    
    def do_pre_matching_process(self, target_data: TargetObjectData) -> None:
        """
        Pre-matching process: Euclidean filtering to extract major segment.
        
        Args:
            target_data: Target object data
        """
        if not target_data.do_euclidean_filtering:
            target_data.cloud_pre_matching_results = target_data.cloud_base_aligned
            return
        
        print("Extracting major segment...")
        
        # Extract largest cluster
        major_segment = self.pcl_proc.extract_major_segment(
            target_data.cloud_base_aligned,
            eps=target_data.euclidean_cluster_tolerance,
            min_points=target_data.euclidean_cluster_min_size,
            max_points=target_data.euclidean_cluster_max_size
        )
        
        # Paint with color for visualization
        major_segment = paint_uniform_color(major_segment, [1.0, 0.0, 0.0])  # Red
        
        target_data.cloud_pre_matching_results = major_segment
        
        print(f"Major segment extracted: {len(major_segment.points)} points")
    
    def cal_measured_cloud_feature_for_quick_align(self, target_data: TargetObjectData) -> None:
        """
        Compute features for measured cloud (for quick alignment).
        
        Args:
            target_data: Target object data
        """
        if not target_data.align_params.do_quick_align:
            return
        
        print("Computing features for measured cloud...")
        
        # Downsample for feature computation
        cloud_for_feature = target_data.cloud_pre_matching_results.voxel_down_sample(
            target_data.cad_quick_align_voxel_filter_size
        )
        
        # Estimate normals
        cloud_for_feature = self.pcl_proc.estimate_normals(
            cloud_for_feature,
            radius=target_data.align_params.normal_radius,
            max_nn=target_data.align_params.normal_max_nn,
            sensor_origin=np.array([0, 0, 0])
        )
        
        # Compute FPFH features
        target_data.cloud_measured_fpfh = self.pcl_proc.compute_fpfh_feature(
            cloud_for_feature,
            radius=target_data.align_params.fpfh_radius,
            max_nn=target_data.align_params.fpfh_max_nn
        )
        
        print(f"Measured FPFH features: {target_data.cloud_measured_fpfh.num()} features")
    
    def do_template_matching_process(self, target_data: TargetObjectData) -> bool:
        """
        Main template matching process: quick align + ICP.
        
        Args:
            target_data: Target object data
            
        Returns:
            True if matching successful
        """
        print("Starting template matching process...")
        
        source = target_data.cloud_measured  # Use measured cloud directly
        target = target_data.cloud_CAD_with_normal
        
        print(f"Source cloud points: {len(source.points)}")
        print(f"Target cloud points: {len(target.points)}")
        
        # Debug: Check coordinate ranges
        source_points = np.asarray(source.points)
        target_points = np.asarray(target.points)
        
        print(f"Source cloud bounds: min={source_points.min(axis=0)}, max={source_points.max(axis=0)}")
        print(f"Target cloud bounds: min={target_points.min(axis=0)}, max={target_points.max(axis=0)}")
        print(f"Source cloud centroid: {source_points.mean(axis=0)}")
        print(f"Target cloud centroid: {target_points.mean(axis=0)}")
        
        # Scale and coordinate transformation
        print("Applying coordinate transformation...")
        
        # Scale source from meters to millimeters (1000x)
        source_scaled = source.scale(1000.0, center=source.get_center())
        
        # Translate source to match target centroid approximately
        target_centroid = target_points.mean(axis=0)
        source_centroid = np.asarray(source_scaled.points).mean(axis=0)
        translation = target_centroid - source_centroid
        
        # Apply translation
        source_transformed = source_scaled.translate(translation)
        
        print(f"After transformation:")
        source_transformed_points = np.asarray(source_transformed.points)
        print(f"Source transformed bounds: min={source_transformed_points.min(axis=0)}, max={source_transformed_points.max(axis=0)}")
        print(f"Source transformed centroid: {source_transformed_points.mean(axis=0)}")
        
        # Use transformed source for matching
        source = source_transformed
        
        # Visualize initial state
        self.visualize_matching_process(source, target, "Initial State", show=self.enable_visualization)
        
        # Initial transformation
        init_transform = np.eye(4)
        
        # Quick alignment if enabled
        if target_data.align_params.do_quick_align:
            print("Performing quick alignment...")
            
            # Downsample for quick align
            source_down = source.voxel_down_sample(target_data.cad_quick_align_voxel_filter_size)
            target_down = target_data.cloud_CAD_for_quick_align
            
            # Compute features if not already computed
            if target_data.cloud_measured_fpfh is None:
                source_down = self.pcl_proc.estimate_normals(source_down)
                source_fpfh = self.pcl_proc.compute_fpfh_feature(
                    source_down,
                    radius=target_data.align_params.fpfh_radius,
                    max_nn=target_data.align_params.fpfh_max_nn
                )
            else:
                source_fpfh = target_data.cloud_measured_fpfh
            
            target_fpfh = target_data.cloud_CAD_fpfh
            
            # Quick align
            init_transform, fitness = self.pcl_proc.quick_align_using_features(
                source_down, target_down, source_fpfh, target_fpfh,
                target_data.align_params
            )
            
            target_data.T_pre_matching = init_transform
            target_data.pre_matching_accuracy = fitness * 100.0
            
            print(f"Quick align completed: accuracy={target_data.pre_matching_accuracy:.2f}%")
        
        # ICP refinement
        print("Performing ICP refinement...")
        
        # Visualize before ICP
        self.visualize_matching_process(source, target, "Before ICP", show=self.enable_visualization)
        
        # Advanced ICP algorithms based on latest research
        print("Trying multiple advanced ICP algorithms...")
        
        # Try different algorithms and pick the best result
        algorithms = []
        
        # 1. Robust ICP with outlier rejection
        try:
            robust_transform, robust_fitness = self.pcl_proc.robust_icp_with_outlier_rejection(
                source, target, init_transform, target_data.align_params, outlier_threshold=0.1
            )
            algorithms.append(("Robust ICP", robust_transform, robust_fitness))
        except Exception as e:
            print(f"Robust ICP failed: {e}")
        
        # 2. Fast Global Registration + ICP
        try:
            fgr_transform, fgr_fitness = self.pcl_proc.fast_global_registration_icp(
                source, target, target_data.align_params
            )
            algorithms.append(("FGR+ICP", fgr_transform, fgr_fitness))
        except Exception as e:
            print(f"FGR+ICP failed: {e}")
        
        # 3. Adaptive multi-scale ICP
        try:
            adaptive_transform, adaptive_fitness = self.pcl_proc.adaptive_multi_scale_icp(
                source, target, init_transform, target_data.align_params
            )
            algorithms.append(("Adaptive Multi-scale ICP", adaptive_transform, adaptive_fitness))
        except Exception as e:
            print(f"Adaptive Multi-scale ICP failed: {e}")
        
        # 4. Colored ICP (if colors available)
        try:
            colored_transform, colored_fitness = self.pcl_proc.colored_icp_registration(
                source, target, init_transform, target_data.align_params
            )
            algorithms.append(("Colored ICP", colored_transform, colored_fitness))
        except Exception as e:
            print(f"Colored ICP failed: {e}")
        
        # 5. Original multi-scale ICP as fallback
        try:
            original_transform, original_fitness = self.pcl_proc.multi_scale_icp(
                source, target, init_transform, target_data.align_params
            )
            algorithms.append(("Original Multi-scale ICP", original_transform, original_fitness))
        except Exception as e:
            print(f"Original Multi-scale ICP failed: {e}")
        
        # Select the best algorithm based on fitness
        if algorithms:
            best_algorithm = max(algorithms, key=lambda x: x[2])  # x[2] is fitness
            algorithm_name, final_transform, fitness = best_algorithm
            print(f"Best algorithm: {algorithm_name} with fitness: {fitness:.4f}")
        else:
            print("All algorithms failed, using identity transformation")
            final_transform = np.eye(4)
            fitness = 0.0
        
        # PCL-style 후처리 (Major Segment Extraction + 추가 정합)
        print("Performing PCL-style post-processing...")
        
        # Major segment extraction (PCL의 핵심 후처리)
        major_segment = o3d.geometry.PointCloud()
        if self.pcl_proc.extract_major_segment_using_euclidean_clustering_with_normals(
            source, major_segment, 
            cluster_tolerance=10.0,  # 10mm
            min_cluster_size=100,
            max_cluster_size=100000,
            b_plot=False
        ):
            print(f"Major segment extracted: {len(major_segment.points)} points")
            
            # Major axis translation (PCL의 추가 정렬)
            major_axis_transformation = self.pcl_proc.major_axis_translation(
                major_segment, target, final_transform, b_plot=False
            )
            
            # Plane rotation using OBB (PCL의 회전 정렬)
            obb_transformation = self.pcl_proc.plane_rotation_using_obb(
                major_segment, target, major_axis_transformation, b_plot=False
            )
            
            # 최종 ICP 정합으로 세밀한 조정
            print("Final ICP refinement with major segment...")
            final_icp_transform, final_icp_fitness = self.pcl_proc.multi_scale_icp(
                major_segment, target,
                init_transformation=obb_transformation,
                params=target_data.align_params
            )
            
            # 최종 변환 행렬 업데이트
            final_transform = final_icp_transform
            fitness = final_icp_fitness
            
            print(f"Post-processing ICP fitness: {fitness:.6f}")
        else:
            print("Major segment extraction failed, using original ICP result")
        
        # Visualize after ICP
        source_final = source.transform(final_transform)
        self.visualize_matching_process(source_final, target, "After ICP", show=self.enable_visualization)
        
        target_data.T_process_matching = final_transform
        
        # Evaluate matching accuracy using PCL-style approach
        source_transformed = source.transform(final_transform)
        accuracy = evaluate_matching_accuracy_improved(
            source_transformed, target, voxel_size=0.0006  # 0.6mm voxel size like PCL
        )
        
        target_data.matching_accuracy = accuracy
        
        print(f"ICP completed: accuracy={accuracy:.2f}%")
        
        # Check if matching is successful
        target_data.is_matching_success = accuracy >= target_data.matching_accuracy_limit
        
        # Store matching results
        target_data.cloud_matching_results = source_transformed
        paint_uniform_color(target_data.cloud_matching_results, [0.0, 1.0, 0.0])  # Green
        
        return target_data.is_matching_success
    
    def visualize_matching_process(self, source: o3d.geometry.PointCloud, 
                                 target: o3d.geometry.PointCloud, 
                                 title: str = "ICP Matching", 
                                 show: bool = True) -> None:
        """
        Visualize ICP matching process with source and target point clouds.
        
        Args:
            source: Source point cloud (scanned data)
            target: Target point cloud (CAD model)
            title: Window title
            show: Whether to show the visualization
        """
        if not show:
            return
            
        try:
            import open3d.visualization as vis
            import time
            
            # Create visualization
            vis_clouds = []
            
            # Source cloud (scanned data) - Red
            source_vis = copy.deepcopy(source)
            source_vis.paint_uniform_color([1.0, 0.0, 0.0])  # Red
            vis_clouds.append(source_vis)
            
            # Target cloud (CAD model) - Blue
            target_vis = copy.deepcopy(target)
            target_vis.paint_uniform_color([0.0, 0.0, 1.0])  # Blue
            vis_clouds.append(target_vis)
            
            # Create visualizer
            vis_obj = vis.Visualizer()
            vis_obj.create_window(window_name=title, width=1200, height=800)
            
            # Add geometries
            for cloud in vis_clouds:
                vis_obj.add_geometry(cloud)
            
            # Set view point
            ctr = vis_obj.get_view_control()
            ctr.set_front([0, 0, -1])
            ctr.set_up([0, -1, 0])
            ctr.set_lookat([0, 0, 0])
            ctr.set_zoom(0.8)
            
            # Initial render
            vis_obj.poll_events()
            vis_obj.update_renderer()
            
            print(f"Visualization window opened: {title}")
            print("Press 'Q' or close window to continue...")
            
            # Simple blocking approach - wait for user input
            input("Press Enter to close visualization window...")
            
            vis_obj.destroy_window()
            print(f"Visualization window closed: {title}")
            
        except Exception as e:
            print(f"Visualization error: {e}")
            print("Continuing without visualization...")
    
    def icp_matching_single_process(self, target_data: TargetObjectData,
                                   mask_idx: int = 0) -> bool:
        """
        Single-threaded ICP matching process for one mask.
        
        Args:
            target_data: Target object data
            mask_idx: Mask index to process
            
        Returns:
            True if matching successful
        """
        print(f"Processing mask #{mask_idx}...")
        
        # Process measured cloud
        self.do_measured_cloud_processing(target_data)
        
        # Align to base frame
        self.do_alignment_scan2robot_base_frame(target_data)
        
        # Pre-matching (segmentation)
        self.do_pre_matching_process(target_data)
        
        # Feature extraction for quick align
        self.cal_measured_cloud_feature_for_quick_align(target_data)
        
        # Template matching
        success = self.do_template_matching_process(target_data)
        
        if success:
            print(f"Mask #{mask_idx} matching SUCCESS")
        else:
            print(f"Mask #{mask_idx} matching FAILED")
        
        return success
    
    def cal_grasping_pose(self, target_data: TargetObjectData) -> bool:
        """
        Calculate grasping pose from matching result.
        
        Args:
            target_data: Target object data
            
        Returns:
            True if pose calculation successful
        """
        if not target_data.is_matching_success:
            return False
        
        print("Calculating grasping pose...")
        
        # Get object-to-base transformation from matching
        T_O2B = target_data.T_process_matching
        target_data.T_matching_O2B = T_O2B
        target_data.T_matching_B2O = np.linalg.inv(T_O2B)
        
        # Calculate grasp point transformation
        # T_B2GP = T_B2O @ T_O2GP
        T_B2GP = target_data.T_matching_B2O @ target_data.T_O2GP
        
        # Apply approach distance if needed
        if target_data.approach_distance > 0:
            # Move along z-axis of grasp frame
            T_approach = np.eye(4)
            T_approach[2, 3] = -target_data.approach_distance
            T_B2GP = T_B2GP @ T_approach
        
        # Convert to pose vector
        grasp_pose = matrix_to_pose_vec(T_B2GP)
        target_data.grasping_pose = grasp_pose
        
        # Calculate tool frame pose
        # T_B2T = T_B2GP @ T_GP2E
        T_B2T = T_B2GP @ np.linalg.inv(target_data.T_E2GP)
        target_data.T_matching_B2T = T_B2T
        
        target_data.is_feasible_pose = True
        
        print(f"Grasping pose: {grasp_pose}")
        
        return True
    
    def icp_matching_bin_picking(self, cloud_mask: o3d.geometry.PointCloud,
                                 cloud_scan: o3d.geometry.PointCloud,
                                 target_id: int,
                                 target_name: str,
                                 q_scan: np.ndarray,
                                 do_scan_sampling: bool = False,
                                 sampling_num: int = 1) -> Tuple[List[float], List[List[float]], List[float], float]:
        """
        Main ICP matching for bin picking (single target).
        
        Args:
            cloud_mask: Masked point cloud (from SAM/Mask-RCNN)
            cloud_scan: Full scan point cloud
            target_id: Target object ID
            target_name: Target object name
            q_scan: Robot joint angles at scan [6]
            do_scan_sampling: Whether to downsample scan
            sampling_num: Downsampling ratio
            
        Returns:
            Tuple of (grasp_pose, grasp_sub_pose_set, zig_pose, matching_accuracy)
        """
        print(f"ICP matching bin picking: target={target_name} (ID={target_id})")
        
        # Get or create target data
        if target_id - 1 < len(self.template_list):
            target_data = self.template_list[target_id - 1]
        else:
            print(f"Target {target_id} not found in template list")
            return [], [], [], 0.0
        
        # Set measured cloud
        target_data.cloud_measured = cloud_mask
        target_data.cloud_scan_raw = cloud_scan
        target_data.robot_scan_JS_position = q_scan
        
        # Process
        success = self.icp_matching_single_process(target_data, mask_idx=0)
        
        if success:
            # Calculate grasping pose
            self.cal_grasping_pose(target_data)
            
            grasp_pose = target_data.grasping_pose
            grasp_sub_pose_set = []  # TODO: implement sub-poses
            zig_pose = target_data.zig_pose if len(target_data.zig_pose) > 0 else [0, 0, 0, 0, 0, 0]
            matching_accuracy = target_data.matching_accuracy
            
            return grasp_pose, grasp_sub_pose_set, zig_pose, matching_accuracy
        else:
            return [], [], [], 0.0
    
    def save_matching_results(self, target_data: TargetObjectData, 
                             output_dir: str) -> None:
        """
        Save matching results to file.
        
        Args:
            target_data: Target object data
            output_dir: Output directory
        """
        os.makedirs(output_dir, exist_ok=True)
        
        # Save matching point cloud
        if target_data.cloud_matching_results is not None:
            output_path = os.path.join(output_dir, "matching_result.pcd")
            o3d.io.write_point_cloud(output_path, target_data.cloud_matching_results)
            print(f"Saved matching result to {output_path}")
        
        # Save transformation
        transform_path = os.path.join(output_dir, "transformation.json")
        transform_data = {
            'T_process_matching': target_data.T_process_matching.tolist(),
            'matching_accuracy': target_data.matching_accuracy,
            'grasping_pose': target_data.grasping_pose,
            'is_matching_success': target_data.is_matching_success
        }
        
        with open(transform_path, 'w') as f:
            json.dump(transform_data, f, indent=2)
        
        print(f"Saved transformation to {transform_path}")

