"""
Collision checking module using voxel grid and KDTree.
Approximates OctoMap-based collision checking from PCL version.
"""

import numpy as np
import open3d as o3d
from typing import List, Tuple, Optional

from .data_classes import TargetObjectData


class CollisionChecker:
    """
    Collision checking class using voxel grid.
    """
    
    def __init__(self):
        """Initialize collision checker"""
        self.collision_threshold = 100  # Number of collision points
    
    def check_gripper_collision(self,
                               target_data: TargetObjectData,
                               T_grasp: np.ndarray,
                               gripper_idx: int = 0) -> Tuple[bool, int]:
        """
        Check collision between gripper and environment using voxel grid.
        Equivalent to checkGripperCollisionUsingVoxelMap in PCL version.
        
        Args:
            target_data: Target object data
            T_grasp: Grasp transformation matrix
            gripper_idx: Gripper configuration index
            
        Returns:
            Tuple of (is_collision, collision_count)
        """
        if not target_data.do_collision_check_using_voxel_map:
            return False, 0
        
        print("Checking gripper collision...")
        
        # Get gripper point cloud
        if gripper_idx >= len(target_data.set_cloud_gripper_CAD_uniformly_downsampled):
            print(f"Warning: Gripper index {gripper_idx} out of range")
            return False, 0
        
        gripper_cloud = target_data.set_cloud_gripper_CAD_uniformly_downsampled[gripper_idx]
        
        # Transform gripper to grasp pose
        gripper_transformed = gripper_cloud.transform(T_grasp)
        
        # Get environment voxel grid
        env_voxel_grid = target_data.voxel_grid_bin_CAD
        
        if env_voxel_grid is None:
            print("Warning: Environment voxel grid not available")
            return False, 0
        
        # Check collision using voxel occupancy
        collision_count = self.count_voxel_collisions(
            gripper_transformed,
            env_voxel_grid,
            target_data.map_voxel_length
        )
        
        threshold = target_data.collision_pt_threshold
        is_collision = collision_count >= threshold
        
        print(f"Collision check: {collision_count} points (threshold: {threshold})")
        
        if is_collision:
            print("COLLISION DETECTED")
        else:
            print("No collision")
        
        return is_collision, collision_count
    
    def count_voxel_collisions(self,
                              cloud: o3d.geometry.PointCloud,
                              voxel_grid: o3d.geometry.VoxelGrid,
                              voxel_size: float) -> int:
        """
        Count number of points in cloud that collide with occupied voxels.
        
        Args:
            cloud: Point cloud to check
            voxel_grid: Occupied voxel grid
            voxel_size: Voxel size [m]
            
        Returns:
            Number of collision points
        """
        if len(cloud.points) == 0:
            return 0
        
        points = np.asarray(cloud.points)
        
        # Get voxel grid parameters
        origin = voxel_grid.origin
        
        # Convert points to voxel indices
        voxel_indices = ((points - origin) / voxel_size).astype(np.int32)
        
        # Get occupied voxels from voxel grid
        occupied_voxels = set()
        for voxel in voxel_grid.get_voxels():
            grid_index = voxel.grid_index
            occupied_voxels.add(tuple(grid_index))
        
        # Count collisions
        collision_count = 0
        for voxel_idx in voxel_indices:
            if tuple(voxel_idx) in occupied_voxels:
                collision_count += 1
        
        return collision_count
    
    def check_collision_with_kdtree(self,
                                   cloud_gripper: o3d.geometry.PointCloud,
                                   cloud_environment: o3d.geometry.PointCloud,
                                   distance_threshold: float = 0.001) -> Tuple[bool, int]:
        """
        Check collision using KDTree nearest neighbor search.
        Alternative method when voxel grid is not available.
        
        Args:
            cloud_gripper: Gripper point cloud
            cloud_environment: Environment point cloud
            distance_threshold: Distance threshold for collision [m]
            
        Returns:
            Tuple of (is_collision, collision_count)
        """
        if len(cloud_gripper.points) == 0 or len(cloud_environment.points) == 0:
            return False, 0
        
        # Build KDTree for environment
        pcd_tree = o3d.geometry.KDTreeFlann(cloud_environment)
        
        # Check each gripper point
        collision_count = 0
        gripper_points = np.asarray(cloud_gripper.points)
        
        for point in gripper_points:
            [k, idx, dist] = pcd_tree.search_knn_vector_3d(point, 1)
            
            if len(dist) > 0 and np.sqrt(dist[0]) < distance_threshold:
                collision_count += 1
        
        is_collision = collision_count >= self.collision_threshold
        
        return is_collision, collision_count
    
    def check_approach_path_collision(self,
                                     target_data: TargetObjectData,
                                     T_grasp: np.ndarray,
                                     approach_distance: float = 0.1,
                                     num_steps: int = 10) -> bool:
        """
        Check collision along approach path.
        
        Args:
            target_data: Target object data
            T_grasp: Final grasp pose
            approach_distance: Distance to check [m]
            num_steps: Number of steps along path
            
        Returns:
            True if collision detected along path
        """
        if not target_data.do_check_collision_approach2escape:
            return False
        
        print(f"Checking approach path collision ({num_steps} steps)...")
        
        # Create approach path
        for step in range(num_steps):
            # Distance from final grasp pose
            dist = approach_distance * (step / num_steps)
            
            # Create transformation at this step
            T_step = T_grasp.copy()
            T_step[2, 3] += dist  # Move along z-axis
            
            # Check collision at this step
            is_collision, count = self.check_gripper_collision(
                target_data, T_step, target_data.gripper_idx_now
            )
            
            if is_collision:
                print(f"Collision detected at step {step} (distance: {dist:.3f}m)")
                return True
        
        print("No collision along approach path")
        return False
    
    def remove_object_from_environment(self,
                                      cloud_environment: o3d.geometry.PointCloud,
                                      cloud_object: o3d.geometry.PointCloud,
                                      removal_radius: float = 0.01) -> o3d.geometry.PointCloud:
        """
        Remove object points from environment cloud for collision checking.
        
        Args:
            cloud_environment: Environment point cloud
            cloud_object: Object point cloud to remove
            removal_radius: Radius for removal [m]
            
        Returns:
            Filtered environment cloud
        """
        if len(cloud_object.points) == 0:
            return cloud_environment
        
        print("Removing object from environment...")
        
        # Build KDTree for object
        pcd_tree = o3d.geometry.KDTreeFlann(cloud_object)
        
        # Check each environment point
        env_points = np.asarray(cloud_environment.points)
        valid_indices = []
        
        for i, point in enumerate(env_points):
            [k, idx, dist] = pcd_tree.search_radius_vector_3d(point, removal_radius)
            
            # Keep point if not near object
            if k == 0:
                valid_indices.append(i)
        
        # Create filtered cloud
        cloud_filtered = cloud_environment.select_by_index(valid_indices)
        
        print(f"Environment cloud: {len(cloud_environment.points)} -> {len(cloud_filtered.points)} points")
        
        return cloud_filtered
    
    def create_collision_environment(self,
                                    target_data: TargetObjectData,
                                    include_bin: bool = True,
                                    include_objects: bool = True) -> o3d.geometry.PointCloud:
        """
        Create complete collision checking environment.
        
        Args:
            target_data: Target object data
            include_bin: Include bin CAD
            include_objects: Include other detected objects
            
        Returns:
            Combined environment point cloud
        """
        print("Creating collision environment...")
        
        env_cloud = o3d.geometry.PointCloud()
        
        # Add bin
        if include_bin and target_data.cloud_bin_CAD_uniformly_downsampled_base_frame is not None:
            env_cloud += target_data.cloud_bin_CAD_uniformly_downsampled_base_frame
            print(f"  Added bin: {len(target_data.cloud_bin_CAD_uniformly_downsampled_base_frame.points)} points")
        
        # Add scanned environment (excluding target object)
        if target_data.cloud_scan_base_aligned_with_bin_collision_check_process is not None:
            scan_cloud = target_data.cloud_scan_base_aligned_with_bin_collision_check_process
            
            # Remove target object region
            if target_data.cloud_base_aligned is not None:
                scan_cloud = self.remove_object_from_environment(
                    scan_cloud,
                    target_data.cloud_base_aligned,
                    removal_radius=0.02  # 20mm
                )
            
            env_cloud += scan_cloud
            print(f"  Added scan: {len(scan_cloud.points)} points")
        
        print(f"Total environment: {len(env_cloud.points)} points")
        
        return env_cloud
    
    def create_voxel_grid_from_cloud(self,
                                    cloud: o3d.geometry.PointCloud,
                                    voxel_size: float) -> o3d.geometry.VoxelGrid:
        """
        Create voxel grid from point cloud.
        
        Args:
            cloud: Input point cloud
            voxel_size: Voxel size [m]
            
        Returns:
            Voxel grid
        """
        if len(cloud.points) == 0:
            # Return empty voxel grid
            return o3d.geometry.VoxelGrid()
        
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(
            cloud,
            voxel_size=voxel_size
        )
        
        print(f"Created voxel grid: {len(voxel_grid.get_voxels())} voxels")
        
        return voxel_grid
    
    def check_self_collision(self,
                            gripper_clouds: List[o3d.geometry.PointCloud],
                            T_grasp: np.ndarray,
                            distance_threshold: float = 0.001) -> bool:
        """
        Check self-collision between gripper parts.
        
        Args:
            gripper_clouds: List of gripper part point clouds
            T_grasp: Grasp transformation
            distance_threshold: Distance threshold [m]
            
        Returns:
            True if self-collision detected
        """
        if len(gripper_clouds) < 2:
            return False
        
        # Transform all gripper parts
        transformed_clouds = []
        for cloud in gripper_clouds:
            transformed_clouds.append(cloud.transform(T_grasp))
        
        # Check collision between each pair
        for i in range(len(transformed_clouds)):
            for j in range(i + 1, len(transformed_clouds)):
                is_collision, count = self.check_collision_with_kdtree(
                    transformed_clouds[i],
                    transformed_clouds[j],
                    distance_threshold
                )
                
                if is_collision:
                    print(f"Self-collision detected between gripper parts {i} and {j}")
                    return True
        
        return False
    
    def estimate_collision_free_pose(self,
                                    target_data: TargetObjectData,
                                    T_initial: np.ndarray,
                                    max_iterations: int = 20,
                                    step_size: float = 0.01) -> Optional[np.ndarray]:
        """
        Try to find collision-free pose near initial pose.
        
        Args:
            target_data: Target object data
            T_initial: Initial transformation
            max_iterations: Maximum search iterations
            step_size: Step size for search [m]
            
        Returns:
            Collision-free transformation, or None if not found
        """
        print("Searching for collision-free pose...")
        
        # Try small perturbations
        search_directions = [
            np.array([step_size, 0, 0]),
            np.array([-step_size, 0, 0]),
            np.array([0, step_size, 0]),
            np.array([0, -step_size, 0]),
            np.array([0, 0, step_size]),
            np.array([0, 0, -step_size]),
        ]
        
        for iteration in range(max_iterations):
            for direction in search_directions:
                # Create perturbed pose
                T_test = T_initial.copy()
                T_test[:3, 3] += direction * iteration
                
                # Check collision
                is_collision, count = self.check_gripper_collision(
                    target_data, T_test, target_data.gripper_idx_now
                )
                
                if not is_collision:
                    print(f"Found collision-free pose after {iteration} iterations")
                    return T_test
        
        print("Could not find collision-free pose")
        return None


