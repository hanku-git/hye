#!/usr/bin/env python3
"""
Record test data for offline testing and debugging.
Records scan data, masks, and results to files.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
import json
import os
from datetime import datetime
from sensor_msgs.msg import PointCloud2

try:
    from hanyang_matching_msgs.msg import MaskCloud, MatchingResultMsg
    MSGS_AVAILABLE = True
except ImportError:
    print("Error: hanyang_matching_msgs not available")
    MSGS_AVAILABLE = False

from hanyang_matching_open3d.modules.utils import ros_to_open3d


class DataRecorder(Node):
    """
    Record matching data for testing and analysis.
    """
    
    def __init__(self, output_dir: str = None):
        super().__init__('data_recorder')
        
        # Create output directory
        if output_dir is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = f"/tmp/matching_test_data_{timestamp}"
        
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info(f"Recording data to: {self.output_dir}")
        
        # Counters
        self.scan_count = 0
        self.result_count = 0
        
        # Create subscribers
        if MSGS_AVAILABLE:
            self.mask_cloud_sub = self.create_subscription(
                MaskCloud,
                '/cloud_mask_results',
                self.mask_cloud_callback,
                10
            )
            
            self.matching_result_sub = self.create_subscription(
                MatchingResultMsg,
                '/open3d/cad_matching_result',
                self.matching_result_callback,
                10
            )
        
        self.get_logger().info("Data recorder ready")
    
    def mask_cloud_callback(self, msg: MaskCloud):
        """Record mask cloud data"""
        try:
            self.scan_count += 1
            
            self.get_logger().info(f"Recording scan #{self.scan_count}")
            
            # Create scan directory
            scan_dir = os.path.join(self.output_dir, f"scan_{self.scan_count:04d}")
            os.makedirs(scan_dir, exist_ok=True)
            
            # Save scan cloud
            cloud_scan = ros_to_open3d(msg.scan_cloud)
            scan_path = os.path.join(scan_dir, "scan_cloud.pcd")
            o3d.io.write_point_cloud(scan_path, cloud_scan)
            self.get_logger().info(f"  Saved scan: {len(cloud_scan.points)} points")
            
            # Save mask cloud (if available)
            if hasattr(msg, 'mask_cloud'):
                cloud_mask = ros_to_open3d(msg.mask_cloud)
                mask_path = os.path.join(scan_dir, "mask_cloud.pcd")
                o3d.io.write_point_cloud(mask_path, cloud_mask)
                self.get_logger().info(f"  Saved mask: {len(cloud_mask.points)} points")
            
            # Save metadata
            metadata = {
                'target_id': msg.class_ids,
                'target_name': msg.class_name,
                'detection_mode': msg.detection_mode,
                'scan_position_q': list(msg.scan_position_q),
                'scan_position_x': list(msg.scan_position_x),
                'robot_dh_parameters': list(msg.robot_dh_parameters),
                'robot_tcp_default': list(msg.robot_tcp_default),
                'robot_tcp': list(msg.robot_tcp),
            }
            
            # SAM results
            if hasattr(msg, 'sam_result'):
                metadata['sam_result'] = {
                    'class_cnt': msg.sam_result.class_cnt,
                    'class_ids': list(msg.sam_result.class_ids),
                    'class_names': list(msg.sam_result.class_names),
                    'scores': list(msg.sam_result.scores),
                    'nearby_ratio_set': list(msg.sam_result.nearby_ratio_set),
                }
            
            meta_path = os.path.join(scan_dir, "metadata.json")
            with open(meta_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            
            self.get_logger().info(f"  Saved metadata")
            
        except Exception as e:
            self.get_logger().error(f"Error recording scan: {e}")
    
    def matching_result_callback(self, msg: MatchingResultMsg):
        """Record matching results"""
        try:
            self.result_count += 1
            
            # Save to latest scan directory
            if self.scan_count > 0:
                scan_dir = os.path.join(self.output_dir, f"scan_{self.scan_count:04d}")
                
                result = {
                    'timestamp': datetime.now().isoformat(),
                    'is_pose': msg.is_pose,
                    'pose': list(msg.pose) if len(msg.pose) > 0 else [],
                    'sub_pose': list(msg.sub_pose) if len(msg.sub_pose) > 0 else [],
                    'zig_pose': list(msg.zig_pose) if len(msg.zig_pose) > 0 else [],
                    'matching_accuracy': msg.matching_accuracy,
                    'approach_distance': msg.approach_distance,
                    'is_grasping_pose_flipped': msg.is_grasping_pose_flipped,
                    'gripper_open_length': float(msg.gripper_open_length),
                    'gripper_close_length': float(msg.gripper_close_length),
                    'gripper_tip_index': msg.gripper_tip_index,
                    'detected_mask_num': msg.detected_mask_num,
                }
                
                result_path = os.path.join(scan_dir, "matching_result.json")
                with open(result_path, 'w') as f:
                    json.dump(result, f, indent=2)
                
                self.get_logger().info(f"Saved result #{self.result_count}")
        
        except Exception as e:
            self.get_logger().error(f"Error recording result: {e}")
    
    def print_summary(self):
        """Print recording summary"""
        print("\n" + "=" * 80)
        print("RECORDING SUMMARY")
        print("=" * 80)
        print(f"Output directory: {self.output_dir}")
        print(f"Scans recorded: {self.scan_count}")
        print(f"Results recorded: {self.result_count}")
        print("=" * 80)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        # Create recorder
        recorder = DataRecorder()
        
        print("\n" + "=" * 80)
        print("DATA RECORDER STARTED")
        print("Press Ctrl+C to stop recording")
        print("=" * 80 + "\n")
        
        try:
            rclpy.spin(recorder)
        except KeyboardInterrupt:
            print("\n\nStopping recorder...")
        
        # Print summary
        recorder.print_summary()
        
        # Cleanup
        recorder.destroy_node()
        
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

