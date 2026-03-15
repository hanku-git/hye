#!/usr/bin/env python3
"""
Compare PCL and Open3D matching performance.
Subscribe to both nodes' outputs and compare results.
"""

import rclpy
from rclpy.node import Node
import time
import numpy as np
from typing import Dict, List
import json

try:
    from hanyang_matching_msgs.msg import MatchingResultMsg
    MSGS_AVAILABLE = True
except ImportError:
    print("Error: hanyang_matching_msgs not available")
    MSGS_AVAILABLE = False


class PerformanceComparator(Node):
    """
    Node to compare PCL and Open3D matching performance.
    """
    
    def __init__(self):
        super().__init__('performance_comparator')
        
        self.get_logger().info("=" * 80)
        self.get_logger().info("PCL vs Open3D Performance Comparator")
        self.get_logger().info("=" * 80)
        
        # Results storage
        self.pcl_results: List[Dict] = []
        self.open3d_results: List[Dict] = []
        
        # Create subscribers
        if MSGS_AVAILABLE:
            # PCL results (assuming remapped topic)
            self.pcl_sub = self.create_subscription(
                MatchingResultMsg,
                '/cad_matching_result_pcl',
                self.pcl_callback,
                10
            )
            
            # Open3D results (new topic name)
            self.open3d_sub = self.create_subscription(
                MatchingResultMsg,
                '/open3d/cad_matching_result',
                self.open3d_callback,
                10
            )
            
            self.get_logger().info("Subscribed to matching result topics")
            self.get_logger().info("  PCL: /cad_matching_result_pcl")
            self.get_logger().info("  Open3D: /open3d/cad_matching_result")
        
        # Statistics
        self.stats = {
            'pcl': {'count': 0, 'success': 0, 'accuracy_sum': 0.0, 'times': []},
            'open3d': {'count': 0, 'success': 0, 'accuracy_sum': 0.0, 'times': []}
        }
        
        # Timer for periodic reporting
        self.report_timer = self.create_timer(5.0, self.print_statistics)
    
    def pcl_callback(self, msg: MatchingResultMsg):
        """Callback for PCL matching results"""
        timestamp = time.time()
        
        result = {
            'timestamp': timestamp,
            'is_pose': msg.is_pose,
            'pose': list(msg.pose) if len(msg.pose) > 0 else [],
            'matching_accuracy': msg.matching_accuracy,
            'detected_mask_num': msg.detected_mask_num,
            'gripper_open_length': msg.gripper_open_length,
        }
        
        self.pcl_results.append(result)
        
        # Update statistics
        self.stats['pcl']['count'] += 1
        if msg.is_pose:
            self.stats['pcl']['success'] += 1
            self.stats['pcl']['accuracy_sum'] += msg.matching_accuracy
        
        # Calculate processing time (if paired)
        if len(self.open3d_results) > 0:
            time_diff = timestamp - self.open3d_results[-1]['timestamp']
            self.stats['pcl']['times'].append(abs(time_diff))
        
        self.get_logger().info(f"PCL result: accuracy={msg.matching_accuracy:.2f}%, pose={msg.is_pose}")
    
    def open3d_callback(self, msg: MatchingResultMsg):
        """Callback for Open3D matching results"""
        timestamp = time.time()
        
        result = {
            'timestamp': timestamp,
            'is_pose': msg.is_pose,
            'pose': list(msg.pose) if len(msg.pose) > 0 else [],
            'matching_accuracy': msg.matching_accuracy,
            'detected_mask_num': msg.detected_mask_num,
            'gripper_open_length': msg.gripper_open_length,
        }
        
        self.open3d_results.append(result)
        
        # Update statistics
        self.stats['open3d']['count'] += 1
        if msg.is_pose:
            self.stats['open3d']['success'] += 1
            self.stats['open3d']['accuracy_sum'] += msg.matching_accuracy
        
        # Calculate processing time
        if len(self.pcl_results) > 0:
            time_diff = timestamp - self.pcl_results[-1]['timestamp']
            self.stats['open3d']['times'].append(abs(time_diff))
        
        self.get_logger().info(f"Open3D result: accuracy={msg.matching_accuracy:.2f}%, pose={msg.is_pose}")
    
    def print_statistics(self):
        """Print periodic statistics"""
        print("\n" + "=" * 80)
        print("PERFORMANCE COMPARISON STATISTICS")
        print("=" * 80)
        
        # PCL statistics
        pcl_count = self.stats['pcl']['count']
        pcl_success = self.stats['pcl']['success']
        pcl_success_rate = (pcl_success / pcl_count * 100) if pcl_count > 0 else 0
        pcl_avg_accuracy = (self.stats['pcl']['accuracy_sum'] / pcl_success) if pcl_success > 0 else 0
        
        print("\nPCL (C++):")
        print(f"  Total runs: {pcl_count}")
        print(f"  Successful: {pcl_success} ({pcl_success_rate:.1f}%)")
        print(f"  Avg accuracy: {pcl_avg_accuracy:.2f}%")
        
        # Open3D statistics
        open3d_count = self.stats['open3d']['count']
        open3d_success = self.stats['open3d']['success']
        open3d_success_rate = (open3d_success / open3d_count * 100) if open3d_count > 0 else 0
        open3d_avg_accuracy = (self.stats['open3d']['accuracy_sum'] / open3d_success) if open3d_success > 0 else 0
        
        print("\nOpen3D (Python):")
        print(f"  Total runs: {open3d_count}")
        print(f"  Successful: {open3d_success} ({open3d_success_rate:.1f}%)")
        print(f"  Avg accuracy: {open3d_avg_accuracy:.2f}%")
        
        # Comparison
        if pcl_count > 0 and open3d_count > 0:
            print("\nComparison:")
            
            accuracy_diff = open3d_avg_accuracy - pcl_avg_accuracy
            print(f"  Accuracy difference: {accuracy_diff:+.2f}% "
                  f"({'Open3D better' if accuracy_diff > 0 else 'PCL better'})")
            
            success_diff = open3d_success_rate - pcl_success_rate
            print(f"  Success rate difference: {success_diff:+.1f}% "
                  f"({'Open3D better' if success_diff > 0 else 'PCL better'})")
        
        print("=" * 80)
    
    def compare_poses(self):
        """Compare pose accuracy between PCL and Open3D"""
        if len(self.pcl_results) == 0 or len(self.open3d_results) == 0:
            print("Not enough data for pose comparison")
            return
        
        print("\n" + "=" * 80)
        print("POSE COMPARISON")
        print("=" * 80)
        
        # Find matching pairs (same timestamp within 1 second)
        pairs = []
        for pcl_res in self.pcl_results:
            for open3d_res in self.open3d_results:
                time_diff = abs(pcl_res['timestamp'] - open3d_res['timestamp'])
                if time_diff < 1.0:  # Within 1 second
                    pairs.append((pcl_res, open3d_res))
                    break
        
        if len(pairs) == 0:
            print("No matching pairs found")
            return
        
        print(f"Found {len(pairs)} matching pairs\n")
        
        position_errors = []
        orientation_errors = []
        
        for i, (pcl_res, open3d_res) in enumerate(pairs):
            if not pcl_res['is_pose'] or not open3d_res['is_pose']:
                continue
            
            pcl_pose = np.array(pcl_res['pose'])
            open3d_pose = np.array(open3d_res['pose'])
            
            # Position error (mm)
            pos_error = np.linalg.norm(pcl_pose[:3] - open3d_pose[:3]) * 1000
            position_errors.append(pos_error)
            
            # Orientation error (deg)
            ori_error = np.linalg.norm(pcl_pose[3:] - open3d_pose[3:])
            orientation_errors.append(ori_error)
            
            print(f"Pair {i + 1}:")
            print(f"  Position error: {pos_error:.2f} mm")
            print(f"  Orientation error: {ori_error:.2f} deg")
            print(f"  PCL accuracy: {pcl_res['matching_accuracy']:.2f}%")
            print(f"  Open3D accuracy: {open3d_res['matching_accuracy']:.2f}%")
            print()
        
        if len(position_errors) > 0:
            print("Summary:")
            print(f"  Avg position error: {np.mean(position_errors):.2f} mm")
            print(f"  Max position error: {np.max(position_errors):.2f} mm")
            print(f"  Avg orientation error: {np.mean(orientation_errors):.2f} deg")
            print(f"  Max orientation error: {np.max(orientation_errors):.2f} deg")
        
        print("=" * 80)
    
    def save_results(self, filename: str = "comparison_results.json"):
        """Save comparison results to file"""
        output = {
            'pcl_results': self.pcl_results,
            'open3d_results': self.open3d_results,
            'statistics': self.stats
        }
        
        output_path = f"/tmp/{filename}"
        
        with open(output_path, 'w') as f:
            json.dump(output, f, indent=2)
        
        self.get_logger().info(f"Results saved to {output_path}")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        comparator = PerformanceComparator()
        
        print("\nMonitoring matching results...")
        print("Press Ctrl+C to stop and show final comparison\n")
        
        try:
            rclpy.spin(comparator)
        except KeyboardInterrupt:
            print("\n\nStopping...")
        
        # Final comparison
        comparator.print_statistics()
        comparator.compare_poses()
        
        # Save results
        comparator.save_results()
        
        # Cleanup
        comparator.destroy_node()
        
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

