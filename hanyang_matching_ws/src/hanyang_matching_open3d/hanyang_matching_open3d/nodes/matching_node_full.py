#!/usr/bin/env python3
"""
Complete Open3D-based matching node with full pipeline.
Fully compatible with PCL version's workflow: Zivid scan → SAM mask → Matching → Robot pose
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
import copy

# ROS2 messages
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import sensor_msgs_py.point_cloud2 as pc2

# Service messages
try:
    from hanyang_matching_msgs.srv import DoTemplateMatching, ZividDoScan
    from hanyang_matching_msgs.msg import (
        MatchingResultMsg, MaskCloud, Resultsam,
        DoTemplateMatchingMsg, ScanCommandMsg
    )
    MSGS_AVAILABLE = True
except ImportError:
    print("Warning: hanyang_matching_msgs not found")
    MSGS_AVAILABLE = False
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

from hanyang_matching_open3d.modules.template_matching import TemplateMatching
from hanyang_matching_open3d.modules.template_matching_extended import extend_template_matching_class
from hanyang_matching_open3d.modules.data_classes import TargetObjectData, MaskData
from hanyang_matching_open3d.modules.utils import ros_to_open3d, open3d_to_ros, pose_vec_to_matrix, matrix_to_pose_vec

def compute_centroid(point_cloud):
    """Compute centroid of a point cloud"""
    if len(point_cloud.points) == 0:
        return np.array([0.0, 0.0, 0.0])
    return np.mean(np.asarray(point_cloud.points), axis=0)

# Extend TemplateMatching class with additional methods
extend_template_matching_class()


class MatchingNodeFull(Node):
    """
    Complete ROS2 node for Open3D-based template matching.
    Implements full pipeline compatible with PCL version.
    """
    
    def __init__(self):
        super().__init__('hanyang_matching_open3d_node')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Initializing Open3D Matching Node (Full Pipeline)...")
        self.get_logger().info("=" * 60)
        
        # Callback group for parallel processing
        self.callback_group = ReentrantCallbackGroup()
        
        # Template matching module
        package_share_dir = self.get_package_share_directory()
        self.package_path = package_share_dir  # Store package path for CAD model access
        self.template_matching = TemplateMatching(package_path=package_share_dir)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # State variables - matching node
        self.cloud_scan: Optional[o3d.geometry.PointCloud] = None
        self.cloud_input: Optional[o3d.geometry.PointCloud] = None
        self.scan_position_q: np.ndarray = np.zeros(6)
        self.scan_position_x: np.ndarray = np.zeros(6)
        self.target_id: int = -1
        self.target_name: str = "none"
        self.is_scan_finished: bool = False
        self.matching_debug_mode: bool = False
        
        # Image data
        self.img_raw: Optional[np.ndarray] = None
        self.img_width: int = 1920
        self.img_height: int = 1200
        
        # View frustum for filtering
        self.view_frustum: List[float] = [-3.0, 3.0, -3.0, 3.0, -3.0, 3.0]  # default
        
        # Robot parameters
        self.robot_dh_vec: List[float] = []
        self.robot_tcp_default: List[float] = []
        self.robot_tcp: List[float] = []
        
        # Mask data
        self.mask_data = MaskData()
        self.detected_mask_num: int = 0
        
        # Multi-threading for mask processing
        self.is_pose_assigned: bool = False
        self.pose_assigned_event = threading.Event()
        self.pose_lock = threading.Lock()
        
        # Matching results (global for multi-threading)
        self.grasp_pose: List[float] = []
        self.grasp_sub_pose_set: List[List[float]] = []
        self.zig_pose: List[float] = []
        self.matching_accuracy: float = 0.0
        self.is_feasible_pose: bool = False
        self.gripper_open_length: int = 0
        self.gripper_close_length: int = 0
        self.gripper_tip_index: int = 0
        
        # Parameters
        self.declare_node_parameters()
        self.load_parameters()
        
        # Visualization control
        self.enable_visualization = self.get_parameter('enable_visualization').value
        
        # Set visualization option in template matching
        self.template_matching.set_visualization_enabled(self.enable_visualization)
        
        # Subscribers
        self.create_subscriptions()
        
        # Publishers
        self.create_publishers()
        
        # Services
        self.create_services()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Open3D Matching Node Initialized Successfully")
        self.get_logger().info("=" * 60)
    
    def get_package_share_directory(self) -> str:
        """Get package share directory"""
        try:
            from ament_index_python.packages import get_package_share_directory
            return get_package_share_directory('hanyang_matching_open3d')
        except:
            return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    
    def declare_node_parameters(self):
        """Declare ROS parameters"""
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('matching_accuracy_limit', 50.0)
        self.declare_parameter('do_quick_align', True)
        self.declare_parameter('icp_max_iterations', 50)
        self.declare_parameter('icp_correspondence_distance', 0.05)
        self.declare_parameter('voxel_size_downsample', 0.005)
        self.declare_parameter('num_threads', 4)
        self.declare_parameter('matching_method', 1)  # 1=ICP, 2=GICP
        self.declare_parameter('enable_visualization', False)
    
    def load_parameters(self):
        """Load ROS parameters"""
        self.matching_debug_mode = self.get_parameter('debug_mode').value
        self.num_threads = self.get_parameter('num_threads').value
        self.matching_method = self.get_parameter('matching_method').value
        self.enable_visualization = self.get_parameter('enable_visualization').value
        
        self.get_logger().info(f"Parameters loaded:")
        self.get_logger().info(f"  Debug mode: {self.matching_debug_mode}")
        self.get_logger().info(f"  Num threads: {self.num_threads}")
        self.get_logger().info(f"  Method: {'ICP' if self.matching_method == 1 else 'GICP'}")
        self.get_logger().info(f"  Visualization: {'Enabled' if self.enable_visualization else 'Disabled'}")
    
    def create_subscriptions(self):
        """Create ROS2 subscribers"""
        # Main mask cloud subscription (from scan node)
        if MSGS_AVAILABLE:
            self.mask_cloud_sub = self.create_subscription(
                MaskCloud,
                '/cloud_mask_results',
                self.on_receive_cloud,
                10,
                callback_group=self.callback_group
            )
            
            self.get_logger().info("Subscribed to /cloud_mask_results")
        
        # Raw image subscription (for debugging)
        self.raw_image_sub = self.create_subscription(
            Image,
            '/zivid/color/image_color',
            self.on_raw_image,
            10,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("Subscriptions created")
    
    def create_publishers(self):
        """Create ROS2 publishers"""
        # Matching results publisher (main output)
        if MSGS_AVAILABLE:
            self.matching_pose_results_pub = self.create_publisher(
                MatchingResultMsg,
                '/cad_matching_result',
                10
            )
            
            self.scanning_results_pub = self.create_publisher(
                MatchingResultMsg,
                '/scanning_result',
                10
            )
        
        # Visualization publishers
        self.cloud_matching_results_pub = self.create_publisher(
            PointCloud2,
            '/cloud_matching_results',
            10
        )
        
        self.cloud_pre_matching_results_pub = self.create_publisher(
            PointCloud2,
            '/cloud_pre_matching_results',
            10
        )
        
        self.cloud_segmentation_results_pub = self.create_publisher(
            PointCloud2,
            '/cloud_segmentation_results',
            10
        )
        
        self.aligned_points_pub = self.create_publisher(
            PointCloud2,
            '/aligned_points_xyz',
            10
        )
        
        self.get_logger().info("Publishers created")
    
    def create_services(self):
        """Create ROS2 services"""
        if MSGS_AVAILABLE:
            # Main matching service
            self.matching_srv = self.create_service(
                DoTemplateMatching,
                '/do_template_matching_bin_picking',
                self.do_matching_bin_picking_callback,
                callback_group=self.callback_group
            )
            
            self.get_logger().info("Services created:")
            self.get_logger().info("  /do_template_matching_bin_picking")
        else:
            self.get_logger().warn("DoTemplateMatching service not available")
    
    def on_raw_image(self, msg: Image):
        """
        Callback for raw RGB image from Zivid.
        
        Args:
            msg: Image message
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.img_raw = cv_image
            self.img_height, self.img_width = cv_image.shape[:2]
            
            self.get_logger().debug(f"Received image: {self.img_width}x{self.img_height}")
            
        except Exception as e:
            self.get_logger().error(f"Error in on_raw_image: {e}")
    
    def on_receive_cloud(self, msg):
        """
        Main callback for receiving mask cloud from scan node.
        Equivalent to do_receive_cloud in PCL version.
        
        Args:
            msg: MaskCloud message
        """
        try:
            detection_mode = msg.detection_mode
            
            self.get_logger().info("*" * 50)
            self.get_logger().info(f"Received cloud (mode: {detection_mode})")
            self.get_logger().info("*" * 50)
            
            # Parse target info
            self.target_id = msg.class_ids
            self.target_name = msg.class_name
            
            self.get_logger().warn(f"Target ID: {self.target_id}, Name: {self.target_name}")
            
            # Parse robot parameters
            self.robot_dh_vec = list(msg.robot_dh_parameters)
            self.robot_tcp_default = list(msg.robot_tcp_default)
            self.robot_tcp = list(msg.robot_tcp)
            
            # Parse robot state (if available)
            if hasattr(msg, 'scan_position_q'):
                self.scan_position_q = np.array(msg.scan_position_q)
            else:
                self.scan_position_q = np.zeros(6)  # Default values
                
            if hasattr(msg, 'scan_position_x'):
                self.scan_position_x = np.array(msg.scan_position_x)
            else:
                self.scan_position_x = np.zeros(6)  # Default values
            
            self.get_logger().info(f"JS position: {self.scan_position_q}")
            self.get_logger().info(f"CS pose: {self.scan_position_x}")
            
            # Convert point clouds
            self.cloud_scan = ros_to_open3d(msg.scan_cloud)
            self.cloud_input = ros_to_open3d(msg.mask_cloud) if hasattr(msg, 'mask_cloud') else None
            
            self.get_logger().info(f"Scan cloud: {len(self.cloud_scan.points)} points")
            if self.cloud_input is not None:
                self.get_logger().info(f"Mask cloud: {len(self.cloud_input.points)} points")
            
            # Parse mask data
            mask_set_in = MaskData()
            
            if detection_mode == "mask_sam_detection":
                self.get_logger().info("Processing SAM detection results")
                
                mask_set_in.target_id = self.target_id
                mask_set_in.detection_mode = detection_mode
                mask_set_in.class_cnt = msg.sam_result.class_cnt
                mask_set_in.class_ids = list(msg.sam_result.class_ids)
                mask_set_in.scores = list(msg.sam_result.scores)
                mask_set_in.class_names = list(msg.sam_result.class_names)
                mask_set_in.mask_imgs = list(msg.sam_result.masks)
                mask_set_in.boxes = list(msg.sam_result.boxes)
                mask_set_in.ext_nearby_ratio = list(msg.sam_result.nearby_ratio_set)
            
            self.detected_mask_num = mask_set_in.class_cnt
            self.mask_data = mask_set_in
            
            self.get_logger().info(f"Detected masks: {self.detected_mask_num}")
            
            if self.detected_mask_num == 0:
                self.get_logger().warn("No masks detected!")
                self.publish_scanning_result(False, 0.0)
                return
            
            # Initialize template if needed
            if self.target_id > len(self.template_matching.template_list):
                self.get_logger().info(f"Initializing template for target {self.target_id}")
                self.get_logger().info(f"Current template_list size: {len(self.template_matching.template_list)}")
                
                target_data = self.template_matching.initialize_template(
                    self.target_id - 1,  # Convert to 0-based index
                    self.robot_dh_vec,
                    self.robot_tcp_default,
                    self.robot_tcp
                )
                
                self.get_logger().info(f"After initialization, template_list size: {len(self.template_matching.template_list)}")
                
                # Set robot state
                target_data.robot_scan_JS_position = self.scan_position_q
                target_data.robot_scan_CS_pose = self.scan_position_x
                
                # Load CAD model
                cad_path = self.get_cad_path(self.target_name)
                if os.path.exists(cad_path):
                    self.template_matching.load_cad_model(target_data, cad_path)
                else:
                    self.get_logger().warn(f"CAD file not found: {cad_path}")
            
            # Get target data (safe index access)
            self.get_logger().info(f"Accessing template at index {self.target_id - 1}, template_list size: {len(self.template_matching.template_list)}")
            if self.target_id > 0 and self.target_id <= len(self.template_matching.template_list):
                target_data = self.template_matching.template_list[self.target_id - 1]
                self.get_logger().info(f"Successfully accessed target data for target_id {self.target_id}")
            else:
                self.get_logger().error(f"Invalid target_id: {self.target_id}, template_list size: {len(self.template_matching.template_list)}")
                self.get_logger().error("Template initialization failed. Please check target_id and CAD model availability.")
                return
            
            # Update robot state
            target_data.robot_scan_JS_position = self.scan_position_q
            target_data.robot_scan_CS_pose = self.scan_position_x
            
            # Set view frustum
            target_data.view_frustum_3d_scanner = self.view_frustum
            
            # Process masks or use full scan
            self.get_logger().info("Processing masks...")
            
            if detection_mode == "mask_sam_detection" and len(mask_set_in.mask_confident_idx_set) > 0:
                # Use SAM mask processing
                mask_success = self.template_matching.initial_mask_processing_ver2(
                    mask_set_in,
                    self.cloud_scan,
                    self.img_width,
                    self.img_height
                )
                
                if not mask_success:
                    self.get_logger().warn("Mask processing failed, using full scan")
                    self._use_full_scan_for_matching(target_data)
                else:
                    # Update target data with mask results
                    target_data.mask_data = mask_set_in
                    target_data.detected_mask_num = len(mask_set_in.mask_confident_idx_set)
                    self.detected_mask_num = target_data.detected_mask_num
                    self.get_logger().info(f"Confident masks: {target_data.detected_mask_num}")
            else:
                # No SAM masks or SAM processing failed, use full scan
                self.get_logger().info("No SAM masks available, using full scan for matching")
                self._use_full_scan_for_matching(target_data)
            
            # Mark scan as finished
            self.is_scan_finished = True
            
            # Publish scanning result
            self.publish_scanning_result(True, 0.0)
            
            # Trigger matching if topic-based mode enabled
            if hasattr(msg, 'do_matching_process_topic'):
                if msg.do_matching_process_topic and not msg.do_single_matching:
                    self.get_logger().info("Starting matching process (topic mode)...")
                    self.matching_process_multi_threaded_bin_picking()
            
        except Exception as e:
            self.get_logger().error(f"Error in on_receive_cloud: {e}")
            import traceback
            traceback.print_exc()
    
    def _use_full_scan_for_matching(self, target_data: TargetObjectData):
        """
        Use full scan point cloud for matching when SAM masks are not available.
        
        Args:
            target_data: Target object data to update
        """
        self.get_logger().info("Setting up full scan for matching...")
        
        # Create a dummy mask data structure
        mask_data = MaskData()
        mask_data.target_id = self.target_id
        mask_data.detection_mode = "full_scan"
        
        # Use the full scan cloud as a single mask
        mask_data.cloud_mask_list = [self.cloud_scan]
        mask_data.mask_confident_idx_set = [0]  # Single mask with index 0
        mask_data.xyz_centroid_list = [compute_centroid(self.cloud_scan)]
        
        # Update target data
        target_data.mask_data = mask_data
        target_data.detected_mask_num = 1
        self.detected_mask_num = 1
        
        self.get_logger().info(f"Full scan setup complete: {len(self.cloud_scan.points)} points")
    
    def matching_process_multi_threaded_bin_picking(self):
        """
        Multi-threaded matching process for bin picking.
        Processes multiple mask candidates in parallel.
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("MULTI-THREADED MATCHING PROCESS START")
        self.get_logger().info("=" * 60)
        
        if self.target_id - 1 >= len(self.template_matching.template_list):
            self.get_logger().error("Target not initialized")
            return
        
        target_data = self.template_matching.template_list[self.target_id - 1]
        
        if target_data is None:
            self.get_logger().error(f"Target data is None for target_id {self.target_id}")
            return
        
        # Reset pose assignment flag
        with self.pose_lock:
            self.is_pose_assigned = False
            self.pose_assigned_event.clear()
        
        start_time = time.time()
        
        # Run multi-threaded matching
        self.get_logger().info(f"Starting matching process...")
        self.get_logger().info(f"Target data mask_confident_idx_set: {target_data.mask_data.mask_confident_idx_set}")
        self.get_logger().info(f"Target data mask_confident_idx_set length: {len(target_data.mask_data.mask_confident_idx_set)}")
        
        success = self.template_matching.matching_process_multi_threaded(
            target_data,
            num_threads=self.num_threads,
            method=self.matching_method
        )
        
        self.get_logger().info(f"Matching process result: success={success}, is_feasible_pose={target_data.is_feasible_pose}")
        self.get_logger().info(f"Target data details:")
        self.get_logger().info(f"  - is_matching_success: {getattr(target_data, 'is_matching_success', 'N/A')}")
        self.get_logger().info(f"  - matching_accuracy: {getattr(target_data, 'matching_accuracy', 'N/A')}")
        self.get_logger().info(f"  - grasping_pose: {getattr(target_data, 'grasping_pose', 'N/A')}")
        self.get_logger().info(f"  - T_process_matching exists: {hasattr(target_data, 'T_process_matching') and target_data.T_process_matching is not None}")
        
        elapsed = (time.time() - start_time) * 1000  # ms
        
        if success and target_data.is_feasible_pose:
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"MATCHING SUCCESS (time: {elapsed:.1f} ms)")
            self.get_logger().info("=" * 60)
            
            # Copy results to global variables
            with self.pose_lock:
                self.is_pose_assigned = True
                self.grasp_pose = target_data.grasping_pose
                self.grasp_sub_pose_set = [target_data.grasping_sub_pose] if len(target_data.grasping_sub_pose) > 0 else []
                self.zig_pose = target_data.zig_pose if len(target_data.zig_pose) > 0 else [0, 0, 0, 0, 0, 0]
                self.matching_accuracy = target_data.matching_accuracy
                self.is_feasible_pose = target_data.is_feasible_pose
                self.gripper_open_length = target_data.gripper_open_length
                self.gripper_close_length = target_data.gripper_close_length
                self.gripper_tip_index = target_data.gripper_tip_index
                
                # Debug: Log detailed matching results
                self.get_logger().info(f"=== DETAILED MATCHING RESULTS ===")
                self.get_logger().info(f"Grasping pose: {self.grasp_pose}")
                self.get_logger().info(f"Matching accuracy: {self.matching_accuracy:.2f}%")
                self.get_logger().info(f"Is feasible pose: {self.is_feasible_pose}")
                if hasattr(target_data, 'T_process_matching') and target_data.T_process_matching is not None:
                    self.get_logger().info(f"Transformation matrix:")
                    self.get_logger().info(f"{target_data.T_process_matching}")
                else:
                    self.get_logger().warn("No transformation matrix found!")
                if hasattr(target_data, 'cloud_matching_results') and target_data.cloud_matching_results is not None:
                    self.get_logger().info(f"Matched cloud points: {len(target_data.cloud_matching_results.points)}")
                else:
                    self.get_logger().warn("No matched cloud found!")
                
                # Debug: Check point cloud properties
                if hasattr(target_data, 'cloud_measured') and target_data.cloud_measured is not None:
                    measured_points = np.asarray(target_data.cloud_measured.points)
                    if len(measured_points) > 0:
                        self.get_logger().info(f"Measured cloud bounds: min={measured_points.min(axis=0)}, max={measured_points.max(axis=0)}")
                        self.get_logger().info(f"Measured cloud centroid: {measured_points.mean(axis=0)}")
                    else:
                        self.get_logger().warn("Measured cloud is empty!")
                
                if hasattr(target_data, 'cloud_CAD_with_normal') and target_data.cloud_CAD_with_normal is not None:
                    cad_points = np.asarray(target_data.cloud_CAD_with_normal.points)
                    if len(cad_points) > 0:
                        self.get_logger().info(f"CAD cloud bounds: min={cad_points.min(axis=0)}, max={cad_points.max(axis=0)}")
                        self.get_logger().info(f"CAD cloud centroid: {cad_points.mean(axis=0)}")
                    else:
                        self.get_logger().warn("CAD cloud is empty!")
            
            self.get_logger().info(f"Grasp pose: {self.grasp_pose}")
            self.get_logger().info(f"Matching accuracy: {self.matching_accuracy:.2f}%")
            self.get_logger().info(f"Gripper open: {self.gripper_open_length} mm")
            
            # Publish results
            self.publish_matching_result(True)
            self.publish_visualization_results()
            
        else:
            self.get_logger().warn("=" * 60)
            self.get_logger().warn(f"MATCHING FAILED (time: {elapsed:.1f} ms)")
            self.get_logger().warn("=" * 60)
            
            # Publish failure
            self.publish_matching_result(False)
        
        # Reset scan finished flag
        self.is_scan_finished = False
    
    def do_matching_bin_picking_callback(self, request, response):
        """
        Service callback for bin picking matching.
        Compatible with PCL version's service interface.
        
        Args:
            request: DoTemplateMatching.Request
            response: DoTemplateMatching.Response
            
        Returns:
            Response with matching results
        """
        try:
            self.get_logger().info("=" * 60)
            self.get_logger().info("MATCHING SERVICE CALLED")
            self.get_logger().info("=" * 60)
            
            # Parse request (don't override target_id from cloud data)
            request_target_id = request.target_id
            request_target_name = request.target_name
            
            self.get_logger().info(f"Target: {request_target_name} (ID: {request_target_id})")
            self.get_logger().info(f"Using initialized target_id: {self.target_id}")
            
            # Update robot parameters if provided
            if len(request.robot_dh_parameters) > 0:
                self.robot_dh_vec = list(request.robot_dh_parameters)
            if len(request.robot_tcp_default) > 0:
                self.robot_tcp_default = list(request.robot_tcp_default)
            if len(request.robot_tcp) > 0:
                self.robot_tcp = list(request.robot_tcp)
            
            # Update scan position if provided
            if hasattr(request, 'scan_position_q') and len(request.scan_position_q) > 0:
                self.scan_position_q = np.array(request.scan_position_q)
            
            # Update view frustum if provided
            if hasattr(request, 'view_frustum') and len(request.view_frustum) == 6:
                self.view_frustum = list(request.view_frustum)
                self.get_logger().info(f"View frustum: {self.view_frustum}")
            
            # Debug mode
            if request.debug_mode:
                self.matching_debug_mode = True
            
            # Check if scan is ready
            if not self.is_scan_finished and not self.matching_debug_mode:
                self.get_logger().warn("Scan not finished yet")
                response.is_pose = False
                response.matching_accuracy = 0.0
                response.detected_mask_num = 0
                return response
            
            # Check if mask num matches
            if self.detected_mask_num == 0:
                self.get_logger().warn("No masks detected")
                response.is_pose = False
                response.matching_accuracy = 0.0
                response.detected_mask_num = 0
                return response
            
            # Use existing matching results if available
            with self.pose_lock:
                if self.is_pose_assigned and len(self.grasp_pose) > 0:
                    self.get_logger().info("Using existing matching results")
                    
                    response.is_pose = True
                    response.pose = self.grasp_pose
                    response.sub_pose = list(np.array(self.grasp_sub_pose_set[0]).astype(float)) if len(self.grasp_sub_pose_set) > 0 else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    response.zig_pose = list(np.array(self.zig_pose).astype(float)) if hasattr(self, 'zig_pose') and self.zig_pose is not None else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    response.matching_accuracy = self.matching_accuracy
                    response.approach_distance = 0.0  # TODO: implement
                    response.is_grasping_pose_flipped = False  # TODO: implement
                    response.gripper_open_length = int(self.gripper_open_length)
                    response.gripper_close_length = int(self.gripper_close_length)
                    response.gripper_tip_index = self.gripper_tip_index
                    response.detected_mask_num = self.detected_mask_num
                    
                    return response
            
            # No existing results, perform matching now
            self.get_logger().info("No existing results, performing matching...")
            
            # Check if we have scan data
            if self.cloud_scan is None:
                self.get_logger().warn("No scan data available")
                response.is_pose = False
                response.matching_accuracy = 0.0
                response.detected_mask_num = 0
                return response
            
            # Perform matching
            self.matching_process_multi_threaded_bin_picking()
            
            # Check results after matching
            with self.pose_lock:
                if self.is_pose_assigned and len(self.grasp_pose) > 0:
                    self.get_logger().info("Matching completed successfully")
                    
                    response.is_pose = True
                    response.pose = self.grasp_pose
                    response.sub_pose = list(np.array(self.grasp_sub_pose_set[0]).astype(float)) if len(self.grasp_sub_pose_set) > 0 else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    response.zig_pose = list(np.array(self.zig_pose).astype(float)) if hasattr(self, 'zig_pose') and self.zig_pose is not None else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    response.matching_accuracy = self.matching_accuracy
                    response.approach_distance = 0.0  # TODO: implement
                    response.is_grasping_pose_flipped = False  # TODO: implement
                    response.gripper_open_length = int(self.gripper_open_length)
                    response.gripper_close_length = int(self.gripper_close_length)
                    response.gripper_tip_index = self.gripper_tip_index
                    response.detected_mask_num = self.detected_mask_num
                    
                    return response
                else:
                    self.get_logger().warn("Matching failed - no feasible pose found")
                    response.is_pose = False
                    response.matching_accuracy = 0.0
                    response.detected_mask_num = self.detected_mask_num
                    
                    return response
            
        except Exception as e:
            self.get_logger().error(f"Error in matching service: {e}")
            import traceback
            traceback.print_exc()
            
            response.is_pose = False
            response.matching_accuracy = 0.0
            response.detected_mask_num = 0
            return response
    
    def publish_scanning_result(self, is_detected: bool, accuracy: float):
        """
        Publish scanning and detection results.
        
        Args:
            is_detected: Detection success
            accuracy: Detection accuracy
        """
        if not MSGS_AVAILABLE:
            return
        
        try:
            msg = MatchingResultMsg()
            msg.is_pose = is_detected
            msg.matching_accuracy = accuracy
            msg.detected_mask_num = self.detected_mask_num
            
            self.scanning_results_pub.publish(msg)
            
            self.get_logger().debug("Scanning result published")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing scanning result: {e}")
    
    def publish_matching_result(self, success: bool):
        """
        Publish matching pose results.
        
        Args:
            success: Matching success
        """
        if not MSGS_AVAILABLE:
            return
        
        try:
            msg = MatchingResultMsg()
            
            if success:
                msg.is_pose = True
                msg.pose = self.grasp_pose
                msg.sub_pose = self.grasp_sub_pose_set[0] if len(self.grasp_sub_pose_set) > 0 else [0, 0, 0, 0, 0, 0]
                msg.zig_pose = self.zig_pose
                msg.matching_accuracy = self.matching_accuracy
                msg.approach_distance = 0.0  # TODO
                msg.is_grasping_pose_flipped = False  # TODO
                msg.gripper_open_length = float(self.gripper_open_length)
                msg.gripper_close_length = float(self.gripper_close_length)
                msg.gripper_tip_index = self.gripper_tip_index
                msg.detected_mask_num = self.detected_mask_num
            else:
                msg.is_pose = False
                msg.matching_accuracy = 0.0
                msg.detected_mask_num = self.detected_mask_num
            
            self.matching_pose_results_pub.publish(msg)
            
            self.get_logger().info("Matching result published")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing matching result: {e}")
    
    def publish_visualization_results(self):
        """Publish visualization point clouds for RViz"""
        try:
            if self.target_id - 1 >= len(self.template_matching.template_list):
                return
            
            target_data = self.template_matching.template_list[self.target_id - 1]
            
            # Publish matching results (CAD + matched cloud)
            if target_data.cloud_matching_results is not None:
                # Merge with CAD for visualization
                cloud_vis = o3d.geometry.PointCloud()
                cloud_vis += target_data.cloud_matching_results
                
                if target_data.cloud_CAD_with_normal is not None:
                    cad_vis = copy.deepcopy(target_data.cloud_CAD_with_normal)
                    cad_vis = cad_vis.transform(target_data.T_process_matching)
                    cad_vis = cad_vis.paint_uniform_color([0.0, 0.0, 1.0])  # Blue
                    cloud_vis += cad_vis
                
                msg = open3d_to_ros(cloud_vis, "base_link")
                self.cloud_matching_results_pub.publish(msg)
            
            # Publish pre-matching results
            if target_data.cloud_pre_matching_results is not None:
                msg = open3d_to_ros(target_data.cloud_pre_matching_results, "base_link")
                self.cloud_pre_matching_results_pub.publish(msg)
            
            # Publish base-aligned cloud
            if target_data.cloud_base_aligned is not None:
                msg = open3d_to_ros(target_data.cloud_base_aligned, "base_link")
                self.cloud_segmentation_results_pub.publish(msg)
            
            self.get_logger().debug("Visualization clouds published")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing visualization: {e}")
    
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
            # Original PCL path structure (corrected)
            os.path.expanduser(f"~/[scanDataHanyang]/CAD/[bin picking]/target_object/{target_name}_CAD.ply"),
            os.path.expanduser(f"~/[scanDataHanyang]/CAD/[bin picking]/target_object/{target_name}_CAD.pcd"),
            # Alternative naming patterns
            os.path.expanduser(f"~/[scanDataHanyang]/CAD/[bin picking]/target_object/{target_name}.ply"),
            os.path.expanduser(f"~/[scanDataHanyang]/CAD/[bin picking]/target_object/{target_name}.pcd"),
            # Alternative paths for flexibility
            os.path.join(self.package_path, 'cad_models', f"{target_name}.ply"),
            os.path.join(self.package_path, 'cad_models', f"{target_name}.pcd"),
            os.path.expanduser(f"~/cad_models/{target_name}.ply"),
            os.path.expanduser(f"~/cad_models/{target_name}.pcd"),
            f"/tmp/{target_name}.ply",
        ]
        
        for path in search_paths:
            if os.path.exists(path):
                self.get_logger().info(f"Found CAD model: {path}")
                return path
        
        self.get_logger().warn(f"CAD model not found for {target_name}")
        self.get_logger().warn(f"Searched paths: {search_paths}")
        return search_paths[0]  # Return first path as default


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = MatchingNodeFull()
        
        # Use multi-threaded executor
        executor = MultiThreadedExecutor(num_threads=8)
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


