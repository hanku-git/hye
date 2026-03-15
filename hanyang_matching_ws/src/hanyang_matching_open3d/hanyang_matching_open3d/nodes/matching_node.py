#!/usr/bin/env python3
"""
Open3D-based matching node for ROS2.
Equivalent to matching_node.cpp in PCL version.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import open3d as o3d
import time
import threading
from typing import List, Optional

# ROS2 messages
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import sensor_msgs_py.point_cloud2 as pc2

# Service messages (assuming hanyang_matching_msgs exists)
try:
    from hanyang_matching_msgs.srv import DoTemplateMatching
    from hanyang_matching_msgs.msg import MatchingResultMsg, MaskCloud
except ImportError:
    print("Warning: hanyang_matching_msgs not found. Using mock messages.")
    DoTemplateMatching = None
    MatchingResultMsg = None
    MaskCloud = None

# OpenCV for image handling
import cv2
from cv_bridge import CvBridge

# Our modules
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from modules.template_matching import TemplateMatching
from modules.data_classes import TargetObjectData, MaskData
from modules.utils import ros_to_open3d, open3d_to_ros, pose_vec_to_matrix, matrix_to_pose_vec


class MatchingNode(Node):
    """
    ROS2 node for Open3D-based template matching.
    """
    
    def __init__(self):
        super().__init__('hanyang_matching_open3d_node')
        
        self.get_logger().info("Initializing Open3D Matching Node...")
        
        # Callback group for parallel processing
        self.callback_group = ReentrantCallbackGroup()
        
        # Template matching module
        package_share_dir = self.get_package_share_directory()
        self.template_matching = TemplateMatching(package_path=package_share_dir)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # State variables
        self.cloud_scan: Optional[o3d.geometry.PointCloud] = None
        self.cloud_input: Optional[o3d.geometry.PointCloud] = None
        self.scan_position_q: np.ndarray = np.zeros(6)
        self.target_id: int = -1
        self.target_name: str = ""
        self.is_scan_finished: bool = False
        self.matching_debug_mode: bool = False
        
        # View frustum for filtering
        self.view_frustum: List[float] = []
        
        # Robot parameters
        self.robot_dh_vec: List[float] = []
        self.robot_tcp_default: List[float] = []
        self.robot_tcp: List[float] = []
        
        # Mask data
        self.mask_data = MaskData()
        self.detected_mask_num: int = 0
        
        # Multi-threading for mask processing
        self.is_pose_assigned: bool = False
        self.pose_assigned_lock = threading.Lock()
        
        # Parameters
        self.declare_parameters()
        self.load_parameters()
        
        # Subscribers
        self.create_subscriptions()
        
        # Publishers
        self.create_publishers()
        
        # Services
        self.create_services()
        
        self.get_logger().info("Open3D Matching Node initialized successfully")
    
    def get_package_share_directory(self) -> str:
        """Get package share directory"""
        try:
            from ament_index_python.packages import get_package_share_directory
            return get_package_share_directory('hanyang_matching_open3d')
        except:
            return ""
    
    def declare_parameters(self):
        """Declare ROS parameters"""
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('matching_accuracy_limit', 50.0)
        self.declare_parameter('do_quick_align', True)
        self.declare_parameter('icp_max_iterations', 50)
        self.declare_parameter('icp_correspondence_distance', 0.05)
        self.declare_parameter('voxel_size_downsample', 0.005)
    
    def load_parameters(self):
        """Load ROS parameters"""
        self.matching_debug_mode = self.get_parameter('debug_mode').value
        
        # Default view frustum (can be overridden by service calls)
        self.view_frustum = [-1.0, 1.0, -1.0, 1.0, 0.0, 2.0]  # [m]
        
        self.get_logger().info(f"Debug mode: {self.matching_debug_mode}")
    
    def create_subscriptions(self):
        """Create ROS2 subscribers"""
        # Point cloud subscription
        self.scan_sub = self.create_subscription(
            PointCloud2,
            '/raw_scanned_points',
            self.scan_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Mask cloud subscription (if using SAM/Mask-RCNN)
        if MaskCloud is not None:
            self.mask_sub = self.create_subscription(
                MaskCloud,
                '/mask_cloud',
                self.mask_callback,
                10,
                callback_group=self.callback_group
            )
        
        self.get_logger().info("Subscriptions created")
    
    def create_publishers(self):
        """Create ROS2 publishers"""
        # Matching results publisher
        self.matching_results_pub = self.create_publisher(
            PointCloud2,
            '/cad_matching_result_cloud',
            10
        )
        
        # Pre-matching results publisher
        self.pre_matching_results_pub = self.create_publisher(
            PointCloud2,
            '/pre_matching_result_cloud',
            10
        )
        
        # Segmentation results publisher
        self.segmentation_results_pub = self.create_publisher(
            PointCloud2,
            '/segmentation_result_cloud',
            10
        )
        
        # Matching result message publisher
        if MatchingResultMsg is not None:
            self.matching_result_msg_pub = self.create_publisher(
                MatchingResultMsg,
                '/cad_matching_result',
                10
            )
        
        self.get_logger().info("Publishers created")
    
    def create_services(self):
        """Create ROS2 services"""
        if DoTemplateMatching is not None:
            # Main matching service
            self.matching_srv = self.create_service(
                DoTemplateMatching,
                '/do_template_matching_bin_picking',
                self.do_matching_bin_picking_callback,
                callback_group=self.callback_group
            )
            
            self.get_logger().info("Services created")
        else:
            self.get_logger().warn("DoTemplateMatching service not available")
    
    def scan_callback(self, msg: PointCloud2):
        """
        Callback for raw scan point cloud.
        
        Args:
            msg: PointCloud2 message
        """
        try:
            # Convert ROS message to Open3D
            self.cloud_scan = ros_to_open3d(msg)
            
            self.get_logger().info(f"Received scan cloud: {len(self.cloud_scan.points)} points")
            
            # Mark scan as finished
            self.is_scan_finished = True
            
        except Exception as e:
            self.get_logger().error(f"Error in scan_callback: {e}")
    
    def mask_callback(self, msg):
        """
        Callback for mask cloud data.
        
        Args:
            msg: MaskCloud message
        """
        try:
            # Parse mask data
            self.target_id = msg.target_id
            self.detected_mask_num = msg.detected_mask_num
            
            # Convert mask cloud
            if hasattr(msg, 'cloud'):
                self.cloud_input = ros_to_open3d(msg.cloud)
            
            self.get_logger().info(f"Received mask data: target_id={self.target_id}, masks={self.detected_mask_num}")
            
        except Exception as e:
            self.get_logger().error(f"Error in mask_callback: {e}")
    
    def do_matching_bin_picking_callback(self, request, response):
        """
        Service callback for bin picking matching.
        
        Args:
            request: Service request
            response: Service response
            
        Returns:
            Response with matching results
        """
        try:
            self.get_logger().info("Matching service called")
            
            # Parse request
            self.target_id = request.target_id
            self.target_name = request.target_name
            
            # Robot parameters
            if len(request.robot_dh_parameters) > 0:
                self.robot_dh_vec = list(request.robot_dh_parameters)
            if len(request.robot_tcp_default) > 0:
                self.robot_tcp_default = list(request.robot_tcp_default)
            if len(request.robot_tcp) > 0:
                self.robot_tcp = list(request.robot_tcp)
            
            # Scan position
            if len(request.scan_position_q) > 0:
                self.scan_position_q = np.array(request.scan_position_q)
            
            # View frustum
            if len(request.view_frustum) == 6:
                self.view_frustum = list(request.view_frustum)
            
            # Debug mode
            if request.debug_mode:
                self.matching_debug_mode = True
            
            # Check if scan is ready
            if not self.is_scan_finished and not self.matching_debug_mode:
                self.get_logger().warn("Scan not finished yet")
                response.is_pose = False
                response.matching_accuracy = 0.0
                return response
            
            # Check if clouds are available
            if self.cloud_scan is None:
                self.get_logger().error("No scan cloud available")
                response.is_pose = False
                response.matching_accuracy = 0.0
                return response
            
            # Use mask cloud if available, otherwise use full scan
            if self.cloud_input is not None:
                cloud_mask = self.cloud_input
            else:
                cloud_mask = self.cloud_scan
            
            # Initialize template if needed
            if self.target_id - 1 >= len(self.template_matching.template_list):
                target_data = self.template_matching.initialize_template(
                    self.target_id - 1,
                    self.robot_dh_vec,
                    self.robot_tcp_default,
                    self.robot_tcp
                )
                
                # Load CAD model
                cad_path = self.get_cad_path(self.target_name)
                if os.path.exists(cad_path):
                    self.template_matching.load_cad_model(target_data, cad_path)
                else:
                    self.get_logger().warn(f"CAD file not found: {cad_path}")
            
            # Perform matching
            start_time = time.time()
            
            grasp_pose, grasp_sub_pose_set, zig_pose, matching_accuracy = \
                self.template_matching.icp_matching_bin_picking(
                    cloud_mask,
                    self.cloud_scan,
                    self.target_id,
                    self.target_name,
                    self.scan_position_q,
                    do_scan_sampling=request.do_scan_sampling,
                    sampling_num=request.sampling_num
                )
            
            elapsed_time = (time.time() - start_time) * 1000  # ms
            
            self.get_logger().info(f"Matching completed in {elapsed_time:.1f} ms")
            self.get_logger().info(f"Matching accuracy: {matching_accuracy:.2f}%")
            
            # Fill response
            response.is_pose = len(grasp_pose) > 0
            response.matching_accuracy = matching_accuracy
            
            if response.is_pose:
                response.pose = grasp_pose
                response.zig_pose = zig_pose if len(zig_pose) > 0 else [0, 0, 0, 0, 0, 0]
                
                # Gripper parameters
                if self.target_id - 1 < len(self.template_matching.template_list):
                    target_data = self.template_matching.template_list[self.target_id - 1]
                    response.gripper_open_length = float(target_data.gripper_open_length)
                    response.gripper_close_length = float(target_data.gripper_close_length)
                
                self.get_logger().info(f"Grasp pose: {grasp_pose}")
            else:
                self.get_logger().warn("Matching failed")
            
            # Publish visualization
            self.publish_visualization_results()
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error in matching service: {e}")
            import traceback
            traceback.print_exc()
            
            response.is_pose = False
            response.matching_accuracy = 0.0
            return response
    
    def get_cad_path(self, target_name: str) -> str:
        """
        Get CAD file path for target object.
        Compatible with PCL version path structure.
        
        Args:
            target_name: Target object name
            
        Returns:
            Path to CAD file
        """
        # PCL compatible paths (same as original system)
        search_paths = [
            # Original PCL path structure
            os.path.expanduser(f"~/scanDataHanyang/CAD/bin picking/target_object/{target_name}.ply"),
            os.path.expanduser(f"~/scanDataHanyang/CAD/bin picking/target_object/{target_name}.pcd"),
            # Try package share directory
            os.path.join(self.get_package_share_directory(), 'cad_models', f"{target_name}.ply") if self.get_package_share_directory() else "",
            # Try home directory
            os.path.expanduser(f"~/cad_models/{target_name}.ply"),
            # Default path
            f"/tmp/{target_name}.ply",
        ]
        
        # Filter out empty paths
        search_paths = [path for path in search_paths if path]
        
        for path in search_paths:
            if os.path.exists(path):
                self.get_logger().info(f"Found CAD model: {path}")
                return path
        
        self.get_logger().warn(f"CAD model not found for {target_name}")
        self.get_logger().warn(f"Searched paths: {search_paths}")
        return search_paths[0] if search_paths else f"/tmp/{target_name}.ply"
    
    def publish_visualization_results(self):
        """Publish visualization point clouds"""
        try:
            if self.target_id - 1 >= len(self.template_matching.template_list):
                return
            
            target_data = self.template_matching.template_list[self.target_id - 1]
            
            # Publish matching results
            if target_data.cloud_matching_results is not None:
                msg = open3d_to_ros(target_data.cloud_matching_results, "base_link")
                self.matching_results_pub.publish(msg)
            
            # Publish pre-matching results
            if target_data.cloud_pre_matching_results is not None:
                msg = open3d_to_ros(target_data.cloud_pre_matching_results, "base_link")
                self.pre_matching_results_pub.publish(msg)
            
            # Publish base-aligned cloud
            if target_data.cloud_base_aligned is not None:
                msg = open3d_to_ros(target_data.cloud_base_aligned, "base_link")
                self.segmentation_results_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing visualization: {e}")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = MatchingNode()
        
        # Use multi-threaded executor for parallel processing
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        node.get_logger().info("Node spinning...")
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

