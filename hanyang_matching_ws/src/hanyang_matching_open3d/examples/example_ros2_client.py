#!/usr/bin/env python3
"""
ROS2 service client example for template matching.
Shows how to call the matching service from a robot control system.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import sys

try:
    from hanyang_matching_msgs.srv import DoTemplateMatching
    from hanyang_matching_msgs.msg import MatchingResultMsg
    MSGS_AVAILABLE = True
except ImportError:
    print("Warning: hanyang_matching_msgs not available")
    MSGS_AVAILABLE = False
    sys.exit(1)


class MatchingClient(Node):
    """
    Example client for template matching service.
    """
    
    def __init__(self):
        super().__init__('matching_client_example')
        
        self.get_logger().info("Creating matching service client...")
        
        # Create service client
        self.client = self.create_client(
            DoTemplateMatching,
            '/do_template_matching_bin_picking'
        )
        
        # Subscribe to Open3D results
        self.result_sub = self.create_subscription(
            MatchingResultMsg,
            '/open3d/cad_matching_result',
            self.result_callback,
            10
        )
        
        # Wait for service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for matching service...')
        
        self.get_logger().info("Service is available!")
    
    def result_callback(self, msg: MatchingResultMsg):
        """Callback for Open3D matching results"""
        self.get_logger().info(f"Received Open3D result: accuracy={msg.matching_accuracy:.2f}%, pose={msg.is_pose}")
    
    def call_matching_service(self,
                             target_id: int = 1,
                             target_name: str = "bolt",
                             robot_dh: list = None,
                             robot_tcp_default: list = None,
                             robot_tcp: list = None,
                             scan_position_q: list = None,
                             view_frustum: list = None,
                             debug_mode: bool = False):
        """
        Call template matching service.
        
        Args:
            target_id: Target object ID
            target_name: Target object name
            robot_dh: DH parameters [24 values]
            robot_tcp_default: Default TCP [6 values]
            robot_tcp: Current TCP [6 values]
            scan_position_q: Joint angles at scan [6 values]
            view_frustum: View frustum [6 values]
            debug_mode: Enable debug mode
            
        Returns:
            Service response
        """
        # Create request
        request = DoTemplateMatching.Request()
        
        request.target_id = target_id
        request.target_name = target_name
        request.debug_mode = debug_mode
        
        # Robot parameters
        if robot_dh is not None:
            request.robot_dh_parameters = robot_dh
        else:
            # Default UR10e DH parameters
            request.robot_dh_parameters = [
                0.0, 0.1807, 0.0, 1.5708,
                0.0, 0.0, -0.6127, 0.0,
                0.0, 0.0, -0.57155, 0.0,
                0.0, 0.17415, 0.0, 1.5708,
                0.0, 0.11985, 0.0, -1.5708,
                0.0, 0.11655, 0.0, 0.0
            ]
        
        if robot_tcp_default is not None:
            request.robot_tcp_default = robot_tcp_default
        else:
            request.robot_tcp_default = [0.0, 0.0, 0.15, 0.0, 0.0, 0.0]
        
        if robot_tcp is not None:
            request.robot_tcp = robot_tcp
        else:
            request.robot_tcp = [0.0, 0.0, 0.15, 0.0, 0.0, 0.0]
        
        # Scan position
        if scan_position_q is not None:
            request.scan_position_q = scan_position_q
        else:
            # Home position
            request.scan_position_q = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
        
        # View frustum
        if view_frustum is not None:
            request.view_frustum = view_frustum
        else:
            request.view_frustum = [-1.0, 1.0, -1.0, 1.0, 0.0, 2.0]
        
        # Other parameters
        request.do_scan_sampling = False
        request.sampling_num = 1
        
        # Call service
        self.get_logger().info(f"Calling matching service for {target_name} (ID: {target_id})...")
        
        future = self.client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.done():
            try:
                response = future.result()
                return response
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
                return None
        else:
            self.get_logger().error('Service call timed out')
            return None
    
    def print_response(self, response):
        """Print service response"""
        print("\n" + "=" * 80)
        print("MATCHING SERVICE RESPONSE")
        print("=" * 80)
        
        if response is None:
            print("❌ No response received")
            return
        
        if response.is_pose:
            print("✅ POSE FOUND")
            print(f"\nGrasping Pose [m, deg]:")
            print(f"  Position: [{response.pose[0]:.4f}, {response.pose[1]:.4f}, {response.pose[2]:.4f}]")
            print(f"  Orientation: [{response.pose[3]:.2f}, {response.pose[4]:.2f}, {response.pose[5]:.2f}]")
            
            if len(response.sub_pose) > 0:
                print(f"\nSub Pose:")
                print(f"  Position: [{response.sub_pose[0]:.4f}, {response.sub_pose[1]:.4f}, {response.sub_pose[2]:.4f}]")
                print(f"  Orientation: [{response.sub_pose[3]:.2f}, {response.sub_pose[4]:.2f}, {response.sub_pose[5]:.2f}]")
            
            if len(response.zig_pose) > 0:
                print(f"\nZig Pose:")
                print(f"  Position: [{response.zig_pose[0]:.4f}, {response.zig_pose[1]:.4f}, {response.zig_pose[2]:.4f}]")
            
            print(f"\nMatching Quality:")
            print(f"  Accuracy: {response.matching_accuracy:.2f}%")
            print(f"  Approach distance: {response.approach_distance * 1000:.1f} mm")
            print(f"  Flipped: {response.is_grasping_pose_flipped}")
            
            print(f"\nGripper Parameters:")
            print(f"  Open length: {response.gripper_open_length:.0f} mm")
            print(f"  Close length: {response.gripper_close_length:.0f} mm")
            print(f"  Tip index: {response.gripper_tip_index}")
            
            print(f"\nDetection Info:")
            print(f"  Detected masks: {response.detected_mask_num}")
        else:
            print("❌ NO POSE FOUND")
            print(f"  Matching accuracy: {response.matching_accuracy:.2f}%")
            print(f"  Detected masks: {response.detected_mask_num}")
        
        print("=" * 80)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        # Create client node
        client = MatchingClient()
        
        # Example 1: Basic matching request
        print("\n" + "=" * 80)
        print("EXAMPLE 1: Basic Matching Request")
        print("=" * 80)
        
        response = client.call_matching_service(
            target_id=1,
            target_name="bolt",
            debug_mode=True
        )
        
        client.print_response(response)
        
        # Example 2: Custom parameters
        print("\n" + "=" * 80)
        print("EXAMPLE 2: Custom Parameters")
        print("=" * 80)
        
        # Custom scan position
        scan_q = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # deg
        
        # Custom view frustum (smaller region)
        view_frustum = [-0.5, 0.5, -0.5, 0.5, 0.0, 1.0]
        
        response2 = client.call_matching_service(
            target_id=2,
            target_name="hinge",
            scan_position_q=scan_q,
            view_frustum=view_frustum,
            debug_mode=False
        )
        
        client.print_response(response2)
        
        # Cleanup
        client.destroy_node()
        
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

