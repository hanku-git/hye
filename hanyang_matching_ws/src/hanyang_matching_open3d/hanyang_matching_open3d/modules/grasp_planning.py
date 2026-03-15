"""
Grasp planning module - calculates grasping pose and checks feasibility.
Equivalent to calGraspingPose functions in PCL version.
"""

import numpy as np
import open3d as o3d
from typing import List, Tuple, Optional

from .data_classes import TargetObjectData
from .utils import matrix_to_pose_vec, pose_vec_to_matrix


class GraspPlanning:
    """
    Grasp planning and pose calculation class.
    """
    
    def __init__(self):
        """Initialize grasp planning module"""
        pass
    
    def calculate_grasping_pose(self, target_data: TargetObjectData) -> bool:
        """
        Calculate grasping pose from matching result.
        Equivalent to calGraspingPose in PCL version.
        
        Args:
            target_data: Target object data with matching results
            
        Returns:
            True if grasping pose is calculated successfully
        """
        if not target_data.is_matching_success:
            print("Matching failed, cannot calculate grasping pose")
            return False
        
        print("Calculating grasping pose...")
        
        # Get object-to-base transformation from matching result
        T_O2B = target_data.T_process_matching  # Object frame to Base frame
        target_data.T_matching_O2B = T_O2B
        target_data.T_matching_B2O = np.linalg.inv(T_O2B)
        
        print(f"Object to Base transformation:\n{T_O2B}")
        
        # Get object-to-grasp-point transformation (from CAD/config)
        T_O2GP = target_data.T_O2GP  # Object frame to Grasp Point
        
        # Calculate base-to-grasp-point transformation
        # T_B2GP = T_B2O @ T_O2GP
        T_B2O = target_data.T_matching_B2O
        T_B2GP = T_B2O @ T_O2GP
        
        print(f"Base to Grasp Point transformation:\n{T_B2GP}")
        
        # Apply approach distance if needed
        if target_data.approach_distance > 0:
            print(f"Applying approach distance: {target_data.approach_distance} mm")
            
            # Create approach transformation (move along z-axis of grasp frame)
            T_approach = np.eye(4)
            T_approach[2, 3] = -target_data.approach_distance  # mm, negative z direction
            
            # Apply approach
            T_B2GP_approach = T_B2GP @ T_approach
            
            # Store approach transformation
            target_data.T_approach_distance = T_approach
        else:
            T_B2GP_approach = T_B2GP
        
        # Convert transformation matrix to pose vector
        grasp_pose = matrix_to_pose_vec(T_B2GP_approach)  # [x, y, z, rx, ry, rz] in [m, deg]
        
        # Store grasping pose
        target_data.grasping_pose = grasp_pose
        target_data.T_O2GP_final = T_B2GP_approach
        
        print(f"Grasping pose: [{grasp_pose[0]:.4f}, {grasp_pose[1]:.4f}, {grasp_pose[2]:.4f}, "
              f"{grasp_pose[3]:.2f}, {grasp_pose[4]:.2f}, {grasp_pose[5]:.2f}]")
        
        # Calculate tool frame pose (for robot control)
        # T_B2T = T_B2GP @ T_GP2E
        T_GP2E = np.linalg.inv(target_data.T_E2GP)  # Grasp Point to End-Effector
        T_B2T = T_B2GP_approach @ T_GP2E
        target_data.T_matching_B2T = T_B2T
        
        return True
    
    def generate_sub_grasping_poses(self, 
                                   target_data: TargetObjectData,
                                   num_rotations: int = 4) -> List[np.ndarray]:
        """
        Generate alternative grasping poses by rotation.
        
        Args:
            target_data: Target object data
            num_rotations: Number of rotation steps
            
        Returns:
            List of alternative transformation matrices
        """
        sub_poses = []
        
        if target_data.shape_name == "cylinder":
            # For cylinder, generate rotations around z-axis
            T_base = target_data.T_O2GP_final
            
            for i in range(num_rotations):
                angle_deg = (360.0 / num_rotations) * i
                angle_rad = np.deg2rad(angle_deg)
                
                # Create rotation matrix around z-axis
                T_rot = np.eye(4)
                c = np.cos(angle_rad)
                s = np.sin(angle_rad)
                T_rot[0, 0] = c
                T_rot[0, 1] = -s
                T_rot[1, 0] = s
                T_rot[1, 1] = c
                
                # Apply rotation
                T_sub = T_base @ T_rot
                sub_poses.append(T_sub)
                
                print(f"Sub-pose {i}: rotation {angle_deg:.1f} deg")
        
        else:
            # For other shapes, just use the main pose
            sub_poses.append(target_data.T_O2GP_final)
        
        # Store sub poses
        target_data.sub_pose_set_T_O2GP = sub_poses
        
        return sub_poses
    
    def check_orientation_flip(self, 
                              target_data: TargetObjectData,
                              measured_normal: np.ndarray,
                              threshold_angle: float = 90.0) -> bool:
        """
        Check if grasping orientation needs to be flipped.
        
        Args:
            target_data: Target object data
            measured_normal: Measured surface normal
            threshold_angle: Threshold angle for flip decision [degrees]
            
        Returns:
            True if orientation should be flipped
        """
        # Get expected grasp approach direction (negative z-axis of grasp frame)
        T_grasp = target_data.T_O2GP_final
        approach_dir = T_grasp[:3, 2]  # z-axis
        
        # Calculate angle between measured normal and approach direction
        cos_angle = np.dot(measured_normal, approach_dir)
        angle_deg = np.rad2deg(np.arccos(np.clip(cos_angle, -1.0, 1.0)))
        
        print(f"Angle between measured normal and approach: {angle_deg:.2f} deg")
        
        # Check if flip is needed
        should_flip = angle_deg > threshold_angle
        
        if should_flip:
            print("Orientation flip detected")
            target_data.is_grasping_pose_flipped = True
            target_data.measured_object_normal_for_flip_check = measured_normal
        
        return should_flip
    
    def apply_custom_orientation(self,
                                target_data: TargetObjectData,
                                custom_vector: np.ndarray) -> np.ndarray:
        """
        Apply custom orientation constraint to grasping pose.
        
        Args:
            target_data: Target object data
            custom_vector: Custom major axis direction
            
        Returns:
            Modified transformation matrix
        """
        T_current = target_data.T_O2GP_final
        
        # Extract translation
        translation = T_current[:3, 3]
        
        # Create new rotation matrix aligned with custom vector
        # Assume custom vector is the new z-axis
        z_axis = custom_vector / np.linalg.norm(custom_vector)
        
        # Choose arbitrary x-axis perpendicular to z
        if abs(z_axis[2]) < 0.9:
            x_axis = np.cross(np.array([0, 0, 1]), z_axis)
        else:
            x_axis = np.cross(np.array([1, 0, 0]), z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        # y-axis completes the right-handed frame
        y_axis = np.cross(z_axis, x_axis)
        
        # Construct rotation matrix
        R_new = np.column_stack([x_axis, y_axis, z_axis])
        
        # Construct new transformation
        T_new = np.eye(4)
        T_new[:3, :3] = R_new
        T_new[:3, 3] = translation
        
        print("Applied custom orientation")
        
        return T_new
    
    def check_workspace_limits(self,
                               target_data: TargetObjectData,
                               pose: List[float]) -> bool:
        """
        Check if pose is within robot workspace limits.
        
        Args:
            target_data: Target object data with workspace definition
            pose: Pose [x, y, z, rx, ry, rz] in [m, deg]
            
        Returns:
            True if within workspace
        """
        if len(target_data.robot_workspace) < 6:
            print("Warning: Robot workspace not defined")
            return True
        
        x, y, z = pose[0], pose[1], pose[2]
        
        x_min, x_max = target_data.robot_workspace[0], target_data.robot_workspace[1]
        y_min, y_max = target_data.robot_workspace[2], target_data.robot_workspace[3]
        z_min, z_max = target_data.robot_workspace[4], target_data.robot_workspace[5]
        
        in_workspace = (x_min <= x <= x_max and
                       y_min <= y <= y_max and
                       z_min <= z <= z_max)
        
        if not in_workspace:
            print(f"Pose out of workspace: [{x:.3f}, {y:.3f}, {z:.3f}]")
            print(f"Workspace limits: x[{x_min:.3f}, {x_max:.3f}], "
                  f"y[{y_min:.3f}, {y_max:.3f}], z[{z_min:.3f}, {z_max:.3f}]")
        
        return in_workspace
    
    def calculate_gripper_width(self,
                               target_data: TargetObjectData,
                               grasp_point: np.ndarray) -> Tuple[int, int]:
        """
        Calculate gripper open/close width based on object size.
        
        Args:
            target_data: Target object data
            grasp_point: Grasp point position
            
        Returns:
            Tuple of (open_length, close_length) in device units
        """
        # Get object dimensions
        if target_data.shape_name == "cylinder":
            # For cylinder, use cross length (diameter)
            object_width = target_data.cad_cross_length  # mm
        else:
            # For other shapes, use bounding box
            bbox = target_data.object_bounding_box_CAD_frame
            if len(bbox) >= 6:
                # Assume gripper approaches along z, so use x or y dimension
                object_width = min(bbox[1] - bbox[0], bbox[3] - bbox[2])  # mm
            else:
                object_width = 50.0  # default
        
        # Calculate gripper widths
        # Open width: object width + margin
        margin = 10.0  # mm
        open_length = int(object_width + margin)
        
        # Close width: object width - grip
        grip = 5.0  # mm
        close_length = int(max(0, object_width - grip))
        
        # Clamp to gripper limits (device-specific)
        max_open = 85  # mm (typical for parallel gripper)
        open_length = min(open_length, max_open)
        close_length = min(close_length, max_open)
        
        print(f"Gripper width: open={open_length}mm, close={close_length}mm "
              f"(object_width={object_width:.1f}mm)")
        
        return open_length, close_length
    
    def transform_pose_sensor_to_robot(self,
                                      target_data: TargetObjectData,
                                      T_sensor: np.ndarray) -> np.ndarray:
        """
        Transform pose from sensor frame to robot base frame.
        
        Args:
            target_data: Target object data with calibration
            T_sensor: Transformation in sensor frame
            
        Returns:
            Transformation in robot base frame
        """
        # T_base = T_B2S @ T_sensor
        T_B2S = target_data.T_B2S
        T_base = T_B2S @ T_sensor
        
        return T_base
    
    def evaluate_grasp_quality(self,
                              target_data: TargetObjectData,
                              grasp_pose: np.ndarray) -> float:
        """
        Evaluate grasp quality based on multiple criteria.
        
        Args:
            target_data: Target object data
            grasp_pose: Grasp transformation matrix
            
        Returns:
            Quality score (0-1, higher is better)
        """
        score = 0.0
        weights = []
        
        # 1. Matching accuracy
        matching_score = target_data.matching_accuracy / 100.0
        score += matching_score * 0.4
        weights.append(0.4)
        print(f"  Matching accuracy: {matching_score:.3f} (weight: 0.4)")
        
        # 2. Workspace position (prefer center)
        pose_vec = matrix_to_pose_vec(grasp_pose)
        x, y, z = pose_vec[0], pose_vec[1], pose_vec[2]
        
        if len(target_data.robot_workspace) >= 6:
            x_center = (target_data.robot_workspace[0] + target_data.robot_workspace[1]) / 2
            y_center = (target_data.robot_workspace[2] + target_data.robot_workspace[3]) / 2
            z_center = (target_data.robot_workspace[4] + target_data.robot_workspace[5]) / 2
            
            x_range = target_data.robot_workspace[1] - target_data.robot_workspace[0]
            y_range = target_data.robot_workspace[3] - target_data.robot_workspace[2]
            z_range = target_data.robot_workspace[5] - target_data.robot_workspace[4]
            
            # Normalized distance from center (0=center, 1=edge)
            dist_x = abs(x - x_center) / (x_range / 2) if x_range > 0 else 0
            dist_y = abs(y - y_center) / (y_range / 2) if y_range > 0 else 0
            dist_z = abs(z - z_center) / (z_range / 2) if z_range > 0 else 0
            
            workspace_score = 1.0 - min(1.0, (dist_x + dist_y + dist_z) / 3.0)
            score += workspace_score * 0.2
            weights.append(0.2)
            print(f"  Workspace position: {workspace_score:.3f} (weight: 0.2)")
        
        # 3. Orientation (prefer vertical approach)
        z_axis = grasp_pose[:3, 2]
        vertical_alignment = abs(z_axis[2])  # 0=horizontal, 1=vertical
        score += vertical_alignment * 0.2
        weights.append(0.2)
        print(f"  Orientation: {vertical_alignment:.3f} (weight: 0.2)")
        
        # 4. Object height (prefer higher objects)
        if len(target_data.mask_data.xyz_centroid_list) > 0:
            object_z = target_data.mask_data.xyz_centroid_list[0][2]
            # Normalize to 0-1 (assuming z range 0-1m)
            height_score = min(1.0, max(0.0, object_z))
            score += height_score * 0.2
            weights.append(0.2)
            print(f"  Object height: {height_score:.3f} (weight: 0.2)")
        
        # Normalize score
        total_weight = sum(weights)
        if total_weight > 0:
            score = score / total_weight
        
        print(f"Overall grasp quality: {score:.3f}")
        
        return score


