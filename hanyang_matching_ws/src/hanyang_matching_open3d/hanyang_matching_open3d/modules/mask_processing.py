"""
Mask processing module - handles SAM/Mask-RCNN detection results.
Equivalent to initialMaskProcessing functions in PCL version.
"""

import numpy as np
import open3d as o3d
import cv2
from typing import List, Tuple, Optional
from sensor_msgs.msg import Image, RegionOfInterest

from .data_classes import MaskData, TargetObjectData
from .utils import compute_centroid


class MaskProcessing:
    """
    Mask processing class for SAM/Mask-RCNN integration.
    """
    
    def __init__(self):
        """Initialize mask processing module"""
        self.pixel_margin = 15  # Margin for mask expansion
    
    def initial_mask_processing_sam(self, 
                                   mask_data: MaskData,
                                   cloud_scan: o3d.geometry.PointCloud,
                                   image_width: int,
                                   image_height: int) -> bool:
        """
        Process SAM detection results and extract masked point clouds.
        Equivalent to initialMaskProcessingVer2 in PCL version.
        
        Args:
            mask_data: Mask detection data from SAM
            cloud_scan: Full scan point cloud
            image_width: Image width
            image_height: Image height
            
        Returns:
            True if processing successful
        """
        print(f"Processing SAM masks: {mask_data.class_cnt} masks detected")
        
        if mask_data.class_cnt == 0:
            print("No masks detected")
            return False
        
        # Get scan points as numpy array
        scan_points = np.asarray(cloud_scan.points)
        scan_colors = np.asarray(cloud_scan.colors) if cloud_scan.has_colors() else None
        
        # Clear previous results
        mask_data.cloud_mask_list = []
        mask_data.mask_cloud_base_aligned_list = []
        mask_data.xyz_centroid_list = []
        mask_data.mask_confident_idx_set = []
        
        # Process each mask
        for mask_idx in range(mask_data.class_cnt):
            print(f"Processing mask {mask_idx + 1}/{mask_data.class_cnt}")
            
            # Check score threshold
            if mask_data.scores[mask_idx] < mask_data.thre_score:
                print(f"  Score {mask_data.scores[mask_idx]:.3f} below threshold {mask_data.thre_score}")
                continue
            
            # Get mask image
            mask_img = mask_data.mask_imgs[mask_idx]
            
            # Convert ROS Image to numpy array
            mask_array = self.ros_image_to_numpy(mask_img, image_width, image_height)
            
            if mask_array is None:
                print(f"  Failed to convert mask image")
                continue
            
            # Extract points from mask region
            masked_cloud = self.extract_cloud_from_mask(
                cloud_scan, mask_array, image_width, image_height
            )
            
            if len(masked_cloud.points) == 0:
                print(f"  No points in masked region")
                continue
            
            # Store masked cloud
            mask_data.cloud_mask_list.append(masked_cloud)
            
            # Compute centroid
            centroid = compute_centroid(masked_cloud)
            mask_data.xyz_centroid_list.append(centroid)
            
            # Add to confident mask set
            mask_data.mask_confident_idx_set.append(mask_idx)
            
            print(f"  Mask {mask_idx}: {len(masked_cloud.points)} points, "
                  f"centroid: [{centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}]")
        
        print(f"Processed {len(mask_data.mask_confident_idx_set)} confident masks")
        
        return len(mask_data.mask_confident_idx_set) > 0
    
    def ros_image_to_numpy(self, 
                          ros_image: Image,
                          width: int,
                          height: int) -> Optional[np.ndarray]:
        """
        Convert ROS Image message to numpy array.
        
        Args:
            ros_image: ROS Image message
            width: Image width
            height: Image height
            
        Returns:
            Numpy array of mask (binary image)
        """
        try:
            # Get image data
            if ros_image.encoding == "mono8":
                mask_array = np.frombuffer(ros_image.data, dtype=np.uint8)
                mask_array = mask_array.reshape((height, width))
            elif ros_image.encoding == "8UC1":
                mask_array = np.frombuffer(ros_image.data, dtype=np.uint8)
                mask_array = mask_array.reshape((height, width))
            elif ros_image.encoding == "bgr8":
                mask_array = np.frombuffer(ros_image.data, dtype=np.uint8)
                mask_array = mask_array.reshape((height, width, 3))
                mask_array = cv2.cvtColor(mask_array, cv2.COLOR_BGR2GRAY)
            else:
                print(f"Unsupported encoding: {ros_image.encoding}")
                return None
            
            # Binarize mask
            _, mask_binary = cv2.threshold(mask_array, 127, 255, cv2.THRESH_BINARY)
            
            return mask_binary
            
        except Exception as e:
            print(f"Error converting ROS image: {e}")
            return None
    
    def extract_cloud_from_mask(self,
                                cloud_scan: o3d.geometry.PointCloud,
                                mask_array: np.ndarray,
                                image_width: int,
                                image_height: int,
                                apply_margin: bool = True) -> o3d.geometry.PointCloud:
        """
        Extract point cloud from mask region.
        
        Args:
            cloud_scan: Full scan point cloud
            mask_array: Binary mask array
            image_width: Image width
            image_height: Image height
            
        Returns:
            Masked point cloud
        """
        # Get scan data
        scan_points = np.asarray(cloud_scan.points)
        scan_colors = np.asarray(cloud_scan.colors) if cloud_scan.has_colors() else None
        
        # Apply margin to mask if requested
        if apply_margin:
            mask_dilated = self.dilate_mask(mask_array, self.pixel_margin)
        else:
            mask_dilated = mask_array
        
        # Create point cloud from organized data
        # Assume point cloud is organized (width x height)
        if len(scan_points) == image_width * image_height:
            # Organized point cloud
            mask_flat = mask_dilated.flatten()
            valid_indices = np.where(mask_flat > 0)[0]
            
            # Extract points
            masked_points = scan_points[valid_indices]
            
            # Remove invalid points (NaN, inf)
            valid_mask = ~np.isnan(masked_points).any(axis=1)
            valid_mask &= ~np.isinf(masked_points).any(axis=1)
            masked_points = masked_points[valid_mask]
            
            # Create masked cloud
            masked_cloud = o3d.geometry.PointCloud()
            masked_cloud.points = o3d.utility.Vector3dVector(masked_points)
            
            # Copy colors if available
            if scan_colors is not None:
                masked_colors = scan_colors[valid_indices][valid_mask]
                masked_cloud.colors = o3d.utility.Vector3dVector(masked_colors)
        
        else:
            # Unorganized point cloud - use projection
            print("Warning: Unorganized point cloud, using simple filtering")
            masked_cloud = cloud_scan
        
        return masked_cloud
    
    def dilate_mask(self, mask: np.ndarray, margin: int) -> np.ndarray:
        """
        Dilate mask by margin pixels.
        
        Args:
            mask: Binary mask
            margin: Dilation margin in pixels
            
        Returns:
            Dilated mask
        """
        if margin <= 0:
            return mask
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (margin*2+1, margin*2+1))
        dilated = cv2.dilate(mask, kernel, iterations=1)
        
        return dilated
    
    def select_best_mask(self,
                        mask_data: MaskData,
                        target_data: TargetObjectData,
                        selection_criteria: str = "highest_z") -> Optional[int]:
        """
        Select the best mask based on criteria.
        
        Args:
            mask_data: Mask data with multiple candidates
            target_data: Target object data
            selection_criteria: Selection method
                - "highest_z": Select highest point
                - "nearest_center": Select nearest to bin center
                - "largest": Select largest cluster
                
        Returns:
            Index of best mask, or None if no valid mask
        """
        if len(mask_data.mask_confident_idx_set) == 0:
            return None
        
        if len(mask_data.mask_confident_idx_set) == 1:
            return mask_data.mask_confident_idx_set[0]
        
        print(f"Selecting best mask from {len(mask_data.mask_confident_idx_set)} candidates")
        print(f"Selection criteria: {selection_criteria}")
        
        if selection_criteria == "highest_z":
            # Select mask with highest z coordinate
            z_values = [centroid[2] for centroid in mask_data.xyz_centroid_list]
            best_idx = np.argmax(z_values)
            selected_mask_idx = mask_data.mask_confident_idx_set[best_idx]
            print(f"Selected mask {selected_mask_idx} with highest z: {z_values[best_idx]:.3f}")
            return selected_mask_idx
        
        elif selection_criteria == "nearest_center":
            # Select mask nearest to bin center
            bin_center = target_data.bin_center_point_base_frame
            
            distances = []
            for centroid in mask_data.xyz_centroid_list:
                dist = np.linalg.norm(centroid[:2] - bin_center[:2])  # xy distance only
                distances.append(dist)
            
            best_idx = np.argmin(distances)
            selected_mask_idx = mask_data.mask_confident_idx_set[best_idx]
            print(f"Selected mask {selected_mask_idx} nearest to center: dist={distances[best_idx]:.3f}")
            return selected_mask_idx
        
        elif selection_criteria == "largest":
            # Select largest cluster
            sizes = [len(cloud.points) for cloud in mask_data.cloud_mask_list]
            best_idx = np.argmax(sizes)
            selected_mask_idx = mask_data.mask_confident_idx_set[best_idx]
            print(f"Selected mask {selected_mask_idx} with most points: {sizes[best_idx]}")
            return selected_mask_idx
        
        else:
            print(f"Unknown selection criteria: {selection_criteria}")
            return mask_data.mask_confident_idx_set[0]
    
    def merge_masks(self,
                   mask_data: MaskData,
                   merge_distance_threshold: float = 0.01) -> o3d.geometry.PointCloud:
        """
        Merge multiple mask point clouds.
        
        Args:
            mask_data: Mask data with multiple clouds
            merge_distance_threshold: Distance threshold for merging [m]
            
        Returns:
            Merged point cloud
        """
        if len(mask_data.cloud_mask_list) == 0:
            return o3d.geometry.PointCloud()
        
        if len(mask_data.cloud_mask_list) == 1:
            return mask_data.cloud_mask_list[0]
        
        print(f"Merging {len(mask_data.cloud_mask_list)} mask clouds")
        
        # Merge all clouds
        merged_cloud = o3d.geometry.PointCloud()
        for cloud in mask_data.cloud_mask_list:
            merged_cloud += cloud
        
        # Remove duplicate points
        merged_cloud = merged_cloud.voxel_down_sample(voxel_size=merge_distance_threshold)
        
        print(f"Merged cloud: {len(merged_cloud.points)} points")
        
        return merged_cloud
    
    def filter_masks_by_nearby_ratio(self,
                                     mask_data: MaskData,
                                     threshold: float = 0.5) -> List[int]:
        """
        Filter masks by nearby object ratio (SAM-specific).
        
        Args:
            mask_data: Mask data with nearby ratios
            threshold: Threshold for nearby ratio
            
        Returns:
            List of valid mask indices
        """
        if len(mask_data.ext_nearby_ratio) == 0:
            return mask_data.mask_confident_idx_set
        
        valid_indices = []
        
        for i, mask_idx in enumerate(mask_data.mask_confident_idx_set):
            if mask_idx < len(mask_data.ext_nearby_ratio):
                nearby_ratio = mask_data.ext_nearby_ratio[mask_idx]
                
                if nearby_ratio < threshold:
                    valid_indices.append(mask_idx)
                    print(f"Mask {mask_idx}: nearby_ratio={nearby_ratio:.3f} (PASS)")
                else:
                    print(f"Mask {mask_idx}: nearby_ratio={nearby_ratio:.3f} (FILTERED)")
            else:
                valid_indices.append(mask_idx)
        
        print(f"Filtered by nearby ratio: {len(valid_indices)}/{len(mask_data.mask_confident_idx_set)}")
        
        return valid_indices
    
    def extract_mask_region_from_bounding_box(self,
                                             cloud_scan: o3d.geometry.PointCloud,
                                             bbox: RegionOfInterest,
                                             image_width: int,
                                             image_height: int) -> o3d.geometry.PointCloud:
        """
        Extract point cloud from bounding box region.
        
        Args:
            cloud_scan: Full scan point cloud
            bbox: Bounding box (ROS RegionOfInterest)
            image_width: Image width
            image_height: Image height
            
        Returns:
            Point cloud in bounding box region
        """
        # Create mask from bounding box
        mask_array = np.zeros((image_height, image_width), dtype=np.uint8)
        
        x_min = max(0, int(bbox.x_offset))
        y_min = max(0, int(bbox.y_offset))
        x_max = min(image_width, int(bbox.x_offset + bbox.width))
        y_max = min(image_height, int(bbox.y_offset + bbox.height))
        
        mask_array[y_min:y_max, x_min:x_max] = 255
        
        # Extract cloud from mask
        return self.extract_cloud_from_mask(cloud_scan, mask_array, image_width, image_height)

