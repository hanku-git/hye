"""
Extended template matching methods for complete pipeline.
This file contains additional methods to be added to TemplateMatching class.
"""

import numpy as np
import open3d as o3d
from typing import List, Tuple, Optional
import concurrent.futures
import threading

from .data_classes import TargetObjectData, MaskData
from .utils import evaluate_matching_accuracy_improved


def initial_mask_processing_ver2(self, mask_data: MaskData, 
                                 cloud_scan: o3d.geometry.PointCloud,
                                 image_width: int = 1920,
                                 image_height: int = 1200) -> bool:
    """
    Process SAM detection results.
    Equivalent to initialMaskProcessingVer2 in PCL version.
    
    Args:
        mask_data: Mask detection data
        cloud_scan: Full scan point cloud
        image_width: Image width (default for Zivid)
        image_height: Image height
        
    Returns:
        True if processing successful
    """
    return self.mask_proc.initial_mask_processing_sam(
        mask_data, cloud_scan, image_width, image_height
    )


def icp_matching_single_process_ver3(self,
                                    ptr_is_pose_assigned: threading.Event,
                                    mask_confident_idx_set: List[int],
                                    pre_selected_idx_set: List[int],
                                    thread_id: int,
                                    target_data: TargetObjectData,
                                    b_plot: bool = False) -> bool:
    """
    Single-threaded ICP matching process for one mask (Ver3).
    Equivalent to icp_matchingSingleProcessVer3 in PCL version.
    
    Args:
        ptr_is_pose_assigned: Threading event for early termination
        mask_confident_idx_set: Set of confident mask indices
        pre_selected_idx_set: Pre-selected mask indices
        thread_id: Thread ID
        target_data: Target object data
        b_plot: Enable plotting
        
    Returns:
        True if matching successful
    """
    print(f"[Thread {thread_id}] ICP matching single process Ver3")
    print(f"[Thread {thread_id}] Mask confident idx set: {mask_confident_idx_set}")
    print(f"[Thread {thread_id}] Pre-selected idx set: {pre_selected_idx_set}")
    
    # Check if already assigned by another thread
    if ptr_is_pose_assigned.is_set():
        print(f"[Thread {thread_id}] Pose already assigned, skipping")
        return False
    
    # Get mask index
    if len(pre_selected_idx_set) == 0:
        print(f"[Thread {thread_id}] No mask selected")
        return False
    
    mask_idx = pre_selected_idx_set[-1]
    print(f"[Thread {thread_id}] Processing mask #{mask_idx}")
    
    # Get masked cloud
    if mask_idx >= len(target_data.mask_data.cloud_mask_list):
        print(f"[Thread {thread_id}] Invalid mask index: {mask_idx}, available masks: {len(target_data.mask_data.cloud_mask_list)}")
        return False
    
    cloud_mask = target_data.mask_data.cloud_mask_list[mask_idx]
    print(f"[Thread {thread_id}] Mask cloud points: {len(cloud_mask.points)}")
    
    # Set as measured cloud
    target_data.cloud_measured = cloud_mask
    
    print(f"[Thread {thread_id}] Starting measured cloud processing...")
    # Process measured cloud
    self.do_measured_cloud_processing(target_data)
    print(f"[Thread {thread_id}] Measured cloud processing completed. Points: {len(target_data.cloud_measured.points)}")
    
    print(f"[Thread {thread_id}] Starting alignment...")
    # Align to base frame
    self.do_alignment_scan2robot_base_frame(target_data)
    print(f"[Thread {thread_id}] Alignment completed. Points: {len(target_data.cloud_measured.points)}")
    
    print(f"[Thread {thread_id}] Starting pre-matching...")
    # Pre-matching (segmentation)
    self.do_pre_matching_process(target_data)
    print(f"[Thread {thread_id}] Pre-matching completed. Points: {len(target_data.cloud_measured.points)}")
    
    print(f"[Thread {thread_id}] Starting feature extraction...")
    # Feature extraction
    self.cal_measured_cloud_feature_for_quick_align(target_data)
    print(f"[Thread {thread_id}] Feature extraction completed. Points: {len(target_data.cloud_measured.points)}")
    
    print(f"[Thread {thread_id}] Starting template matching...")
    # Template matching (ICP)
    success = self.do_template_matching_process(target_data)
    print(f"[Thread {thread_id}] Template matching result: {success}")
    print(f"[Thread {thread_id}] After template matching. Points: {len(target_data.cloud_measured.points)}")
    
    if success:
        print(f"[Thread {thread_id}] Matching SUCCESS")
        
        print(f"[Thread {thread_id}] Calculating grasping pose...")
        # Calculate grasping pose
        self.cal_grasping_pose_multi_threaded(target_data)
        print(f"[Thread {thread_id}] Grasping pose calculated")
        
        print(f"[Thread {thread_id}] Checking feasibility...")
        # Check collision
        self.check_feasibility_multi_threaded(target_data)
        print(f"[Thread {thread_id}] Feasibility check result: {target_data.is_feasible_pose}")
        
        if target_data.is_feasible_pose:
            print(f"[Thread {thread_id}] Feasible pose found!")
            ptr_is_pose_assigned.set()
            return True
    else:
        print(f"[Thread {thread_id}] Matching FAILED")
    
    return False


def gicp_matching_single_process_ver1(self,
                                     ptr_is_pose_assigned: threading.Event,
                                     mask_confident_idx_set: List[int],
                                     pre_selected_idx_set: List[int],
                                     thread_id: int,
                                     target_data: TargetObjectData,
                                     b_plot: bool = False) -> bool:
    """
    Single-threaded GICP matching process for one mask (Ver1).
    Equivalent to gicp_matchingSingleProcessVer1 in PCL version.
    
    Uses multi-scale ICP to approximate GICP.
    
    Args:
        ptr_is_pose_assigned: Threading event for early termination
        mask_confident_idx_set: Set of confident mask indices
        pre_selected_idx_set: Pre-selected mask indices
        thread_id: Thread ID
        target_data: Target object data
        b_plot: Enable plotting
        
    Returns:
        True if matching successful
    """
    print(f"[Thread {thread_id}] GICP matching single process Ver1")
    
    # Check if already assigned
    if ptr_is_pose_assigned.is_set():
        print(f"[Thread {thread_id}] Pose already assigned, skipping")
        return False
    
    # Get mask index
    if len(pre_selected_idx_set) == 0:
        print(f"[Thread {thread_id}] No mask selected")
        return False
    
    mask_idx = pre_selected_idx_set[-1]
    print(f"[Thread {thread_id}] Processing mask #{mask_idx} with GICP")
    
    # Get masked cloud
    if mask_idx >= len(target_data.mask_data.cloud_mask_list):
        print(f"[Thread {thread_id}] Invalid mask index")
        return False
    
    cloud_mask = target_data.mask_data.cloud_mask_list[mask_idx]
    target_data.cloud_measured = cloud_mask
    
    # Process
    self.do_measured_cloud_processing(target_data)
    self.do_alignment_scan2robot_base_frame(target_data)
    self.do_pre_matching_process(target_data)
    self.cal_measured_cloud_feature_for_quick_align(target_data)
    
    # Template matching with multi-scale ICP (GICP approximation)
    success = self.do_template_matching_process_gicp(target_data)
    
    if success:
        print(f"[Thread {thread_id}] GICP matching SUCCESS")
        
        self.cal_grasping_pose_multi_threaded(target_data)
        self.check_feasibility_multi_threaded(target_data)
        
        if target_data.is_feasible_pose:
            print(f"[Thread {thread_id}] Feasible pose found!")
            ptr_is_pose_assigned.set()
            return True
    else:
        print(f"[Thread {thread_id}] GICP matching FAILED")
    
    return False


def do_template_matching_process_gicp(self, target_data: TargetObjectData) -> bool:
    """
    Template matching using multi-scale ICP (GICP approximation).
    
    Args:
        target_data: Target object data
        
    Returns:
        True if matching successful
    """
    print("Template matching with GICP (multi-scale ICP)...")
    
    source = target_data.cloud_pre_matching_results
    target = target_data.cloud_CAD_with_normal
    
    # Initial transformation
    init_transform = np.eye(4)
    
    # Quick alignment if enabled
    if target_data.align_params.do_quick_align:
        print("Performing quick alignment...")
        
        source_down = source.voxel_down_sample(target_data.cad_quick_align_voxel_filter_size)
        target_down = target_data.cloud_CAD_for_quick_align
        
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
        
        init_transform, fitness = self.pcl_proc.quick_align_using_features(
            source_down, target_down, source_fpfh, target_fpfh,
            target_data.align_params
        )
        
        target_data.T_pre_matching = init_transform
        target_data.pre_matching_accuracy = fitness * 100.0
        
        print(f"Quick align completed: accuracy={target_data.pre_matching_accuracy:.2f}%")
    
    # Multi-scale ICP (GICP approximation)
    print("Performing multi-scale ICP (GICP)...")
    
    final_transform, fitness = self.pcl_proc.multi_scale_icp(
        source, target, init_transform, target_data.align_params,
        voxel_sizes=[0.05, 0.02, 0.01, 0.005]  # More scales for better approximation
    )
    
    target_data.T_process_matching = final_transform
    
    # Evaluate matching accuracy using PCL-style approach
    source_transformed = source.transform(final_transform)
    accuracy = evaluate_matching_accuracy_improved(
        source_transformed, target, voxel_size=0.0006  # 0.6mm voxel size like PCL
    )
    
    target_data.matching_accuracy = accuracy
    
    print(f"GICP completed: accuracy={accuracy:.2f}%")
    
    # Check if matching is successful
    target_data.is_matching_success = accuracy >= target_data.matching_accuracy_limit
    
    # Store matching results
    target_data.cloud_matching_results = source_transformed
    paint_uniform_color(target_data.cloud_matching_results, [0.0, 1.0, 0.0])  # Green
    
    return target_data.is_matching_success


def cal_grasping_pose_multi_threaded(self, target_data: TargetObjectData) -> bool:
    """
    Calculate grasping pose (thread-safe version).
    
    Args:
        target_data: Target object data
        
    Returns:
        True if successful
    """
    return self.grasp_plan.calculate_grasping_pose(target_data)


def check_feasibility_multi_threaded(self, target_data: TargetObjectData) -> bool:
    """
    Check pose feasibility including collision check (thread-safe version).
    
    Args:
        target_data: Target object data
        
    Returns:
        True if feasible
    """
    print("Checking pose feasibility...")
    
    # Check workspace limits
    if len(target_data.grasping_pose) > 0:
        in_workspace = self.grasp_plan.check_workspace_limits(
            target_data, target_data.grasping_pose
        )
        
        if not in_workspace:
            target_data.is_feasible_pose = False
            return False
    
    # Check collision
    if target_data.do_collision_check_using_voxel_map:
        T_grasp = target_data.T_O2GP_final
        
        is_collision, count = self.collision_check.check_gripper_collision(
            target_data, T_grasp, target_data.gripper_idx_now
        )
        
        target_data.is_collision = is_collision
        
        if is_collision:
            print(f"Collision detected: {count} points")
            target_data.is_feasible_pose = False
            return False
        
        # Check approach path
        if target_data.do_check_collision_approach2escape:
            approach_collision = self.collision_check.check_approach_path_collision(
                target_data, T_grasp, approach_distance=0.1, num_steps=10
            )
            
            if approach_collision:
                print("Collision along approach path")
                target_data.is_feasible_pose = False
                return False
    
    # Calculate gripper width
    if len(target_data.mask_data.xyz_centroid_list) > 0:
        grasp_point = target_data.mask_data.xyz_centroid_list[0]
        open_len, close_len = self.grasp_plan.calculate_gripper_width(
            target_data, grasp_point
        )
        target_data.gripper_open_length = open_len
        target_data.gripper_close_length = close_len
    
    # All checks passed
    target_data.is_feasible_pose = True
    print("Pose is FEASIBLE")
    
    return True


def matching_process_multi_threaded(self,
                                   target_data: TargetObjectData,
                                   num_threads: int = 4,
                                   method: int = 1) -> bool:
    """
    Multi-threaded matching process for multiple mask candidates.
    
    Args:
        target_data: Target object data
        num_threads: Number of parallel threads
        method: 1=ICP, 2=GICP
        
    Returns:
        True if at least one successful match found
    """
    print(f"Multi-threaded matching with {num_threads} threads (method={method})")
    
    mask_data = target_data.mask_data
    
    if len(mask_data.mask_confident_idx_set) == 0:
        print("No confident masks to process")
        return False
    
    # Create event for early termination
    is_pose_assigned = threading.Event()
    
    # Prepare mask indices to process
    mask_indices = mask_data.mask_confident_idx_set.copy()
    
    print(f"Processing {len(mask_indices)} mask candidates")
    
    # Thread-safe data copies
    thread_data_list = []
    for i, mask_idx in enumerate(mask_indices):
        # Create copy of target_data for each thread
        thread_data = TargetObjectData()
        # Copy necessary fields
        thread_data.cloud_measured = mask_data.cloud_mask_list[mask_idx]
        thread_data.mask_data = mask_data
        thread_data.align_params = target_data.align_params
        thread_data.cloud_CAD = target_data.cloud_CAD
        thread_data.cloud_CAD_with_normal = target_data.cloud_CAD_with_normal
        thread_data.cloud_CAD_for_quick_align = target_data.cloud_CAD_for_quick_align
        thread_data.cloud_CAD_fpfh = target_data.cloud_CAD_fpfh
        # Copy robot parameters
        thread_data.DH_parameters = target_data.DH_parameters
        thread_data.robot_tcp = target_data.robot_tcp
        thread_data.robot_scan_JS_position = target_data.robot_scan_JS_position
        thread_data.T_E2S = target_data.T_E2S
        thread_data.T_E2GP = target_data.T_E2GP
        # Copy other parameters
        thread_data.view_frustum_3d_scanner = target_data.view_frustum_3d_scanner
        thread_data.do_collision_check_using_voxel_map = target_data.do_collision_check_using_voxel_map
        thread_data.voxel_grid_bin_CAD = target_data.voxel_grid_bin_CAD
        thread_data.matching_accuracy_limit = target_data.matching_accuracy_limit
        
        thread_data_list.append(thread_data)
    
    # Process masks in parallel
    with concurrent.futures.ThreadPoolExecutor(max_workers=num_threads) as executor:
        futures = []
        
        for i, (mask_idx, thread_data) in enumerate(zip(mask_indices, thread_data_list)):
            pre_selected = [mask_idx]
            
            if method == 1:  # ICP
                future = executor.submit(
                    self.icp_matching_single_process_ver3,
                    is_pose_assigned,
                    mask_data.mask_confident_idx_set,
                    pre_selected,
                    i,
                    thread_data,
                    False
                )
            else:  # GICP
                future = executor.submit(
                    self.gicp_matching_single_process_ver1,
                    is_pose_assigned,
                    mask_data.mask_confident_idx_set,
                    pre_selected,
                    i,
                    thread_data,
                    False
                )
            
            futures.append((future, i, mask_idx, thread_data))
        
        # Wait for results
        for future, thread_id, mask_idx, thread_data in futures:
            try:
                success = future.result(timeout=30)  # 30 second timeout per thread
                
                if success and thread_data.is_feasible_pose:
                    print(f"Thread {thread_id} found feasible pose for mask {mask_idx}!")
                    
                    # Copy results back to main target_data
                    target_data.is_matching_success = True
                    target_data.is_feasible_pose = True
                    target_data.matching_accuracy = thread_data.matching_accuracy
                    target_data.grasping_pose = thread_data.grasping_pose
                    target_data.T_O2GP_final = thread_data.T_O2GP_final
                    target_data.T_process_matching = thread_data.T_process_matching
                    target_data.cloud_matching_results = thread_data.cloud_matching_results
                    target_data.gripper_open_length = thread_data.gripper_open_length
                    target_data.gripper_close_length = thread_data.gripper_close_length
                    
                    return True
                    
            except concurrent.futures.TimeoutError:
                print(f"Thread {thread_id} timed out")
            except Exception as e:
                print(f"Thread {thread_id} error: {e}")
    
    print("Multi-threaded matching completed, no feasible pose found")
    return False


# Add these methods to TemplateMatching class
def extend_template_matching_class():
    """Helper function to add methods to TemplateMatching class"""
    from .template_matching import TemplateMatching
    
    TemplateMatching.initial_mask_processing_ver2 = initial_mask_processing_ver2
    TemplateMatching.icp_matching_single_process_ver3 = icp_matching_single_process_ver3
    TemplateMatching.gicp_matching_single_process_ver1 = gicp_matching_single_process_ver1
    TemplateMatching.do_template_matching_process_gicp = do_template_matching_process_gicp
    TemplateMatching.cal_grasping_pose_multi_threaded = cal_grasping_pose_multi_threaded
    TemplateMatching.check_feasibility_multi_threaded = check_feasibility_multi_threaded
    TemplateMatching.matching_process_multi_threaded = matching_process_multi_threaded


