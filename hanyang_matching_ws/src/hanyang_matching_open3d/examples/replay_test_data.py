#!/usr/bin/env python3
"""
Replay recorded test data for offline testing.
Useful for debugging and algorithm development without hardware.
"""

import numpy as np
import open3d as o3d
import json
import os
import sys
from pathlib import Path

# Add package to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hanyang_matching_open3d.modules.template_matching import TemplateMatching
from hanyang_matching_open3d.modules.template_matching_extended import extend_template_matching_class
from hanyang_matching_open3d.modules.data_classes import TargetObjectData, MaskData

# Extend TemplateMatching class
extend_template_matching_class()


class DataReplayer:
    """
    Replay recorded test data.
    """
    
    def __init__(self, data_dir: str):
        """
        Initialize replayer.
        
        Args:
            data_dir: Directory containing recorded data
        """
        self.data_dir = data_dir
        self.template_matching = TemplateMatching()
        
        print(f"Data replayer initialized")
        print(f"Data directory: {data_dir}")
    
    def list_scans(self) -> list:
        """List all available scans"""
        scans = []
        
        for item in sorted(os.listdir(self.data_dir)):
            item_path = os.path.join(self.data_dir, item)
            if os.path.isdir(item_path) and item.startswith('scan_'):
                scans.append(item_path)
        
        return scans
    
    def load_scan_data(self, scan_dir: str) -> dict:
        """
        Load scan data from directory.
        
        Args:
            scan_dir: Scan directory path
            
        Returns:
            Dictionary with scan data
        """
        print(f"\nLoading scan from: {scan_dir}")
        
        data = {}
        
        # Load metadata
        meta_path = os.path.join(scan_dir, "metadata.json")
        if os.path.exists(meta_path):
            with open(meta_path, 'r') as f:
                data['metadata'] = json.load(f)
            print(f"  Metadata loaded")
        
        # Load scan cloud
        scan_path = os.path.join(scan_dir, "scan_cloud.pcd")
        if os.path.exists(scan_path):
            data['scan_cloud'] = o3d.io.read_point_cloud(scan_path)
            print(f"  Scan cloud: {len(data['scan_cloud'].points)} points")
        
        # Load mask cloud
        mask_path = os.path.join(scan_dir, "mask_cloud.pcd")
        if os.path.exists(mask_path):
            data['mask_cloud'] = o3d.io.read_point_cloud(mask_path)
            print(f"  Mask cloud: {len(data['mask_cloud'].points)} points")
        
        # Load matching result (if exists)
        result_path = os.path.join(scan_dir, "matching_result.json")
        if os.path.exists(result_path):
            with open(result_path, 'r') as f:
                data['matching_result'] = json.load(f)
            print(f"  Matching result loaded")
        
        return data
    
    def replay_scan(self, scan_data: dict, visualize: bool = True) -> dict:
        """
        Replay matching process on recorded scan.
        
        Args:
            scan_data: Loaded scan data
            visualize: Show visualization
            
        Returns:
            Matching results
        """
        print("\n" + "=" * 80)
        print("REPLAYING SCAN")
        print("=" * 80)
        
        metadata = scan_data.get('metadata', {})
        
        # Get target info
        target_id = metadata.get('target_id', 1)
        target_name = metadata.get('target_name', 'unknown')
        
        print(f"\nTarget: {target_name} (ID: {target_id})")
        
        # Initialize template
        dh = metadata.get('robot_dh_parameters', [0.0] * 24)
        tcp_default = metadata.get('robot_tcp_default', [0.0] * 6)
        tcp = metadata.get('robot_tcp', [0.0] * 6)
        
        target_data = self.template_matching.initialize_template(
            target_id - 1, dh, tcp_default, tcp
        )
        
        # Load CAD model
        # Try PCL compatible path first
        cad_paths = [
            os.path.expanduser(f"~/scanDataHanyang/CAD/bin picking/target_object/{target_name}.ply"),
            os.path.expanduser(f"~/scanDataHanyang/CAD/bin picking/target_object/{target_name}.pcd"),
            os.path.expanduser(f"~/cad_models/{target_name}.ply"),
            os.path.expanduser(f"~/cad_models/{target_name}.pcd"),
        ]
        
        cad_path = None
        for path in cad_paths:
            if os.path.exists(path):
                cad_path = path
                break
        
        if cad_path:
            self.template_matching.load_cad_model(target_data, cad_path)
            print(f"CAD model loaded from: {cad_path}")
        else:
            print(f"Warning: CAD model not found in any of these paths:")
            for path in cad_paths:
                print(f"  - {path}")
            print("Creating synthetic CAD for testing...")
            mesh = o3d.geometry.TriangleMesh.create_cylinder(0.015, 0.05)
            target_data.cloud_CAD = mesh.sample_points_uniformly(5000)
        
        # Set scan data
        if 'mask_cloud' in scan_data:
            target_data.cloud_measured = scan_data['mask_cloud']
        elif 'scan_cloud' in scan_data:
            target_data.cloud_measured = scan_data['scan_cloud']
        else:
            print("Error: No cloud data found")
            return {}
        
        # Set robot state
        scan_q = metadata.get('scan_position_q', [0.0] * 6)
        target_data.robot_scan_JS_position = np.array(scan_q)
        
        # Configure parameters
        target_data.align_params.do_quick_align = True
        target_data.do_euclidean_filtering = True
        target_data.matching_accuracy_limit = 50.0
        
        # Run matching
        print("\nRunning matching process...")
        print("-" * 80)
        
        import time
        start_time = time.time()
        
        success = self.template_matching.icp_matching_single_process(
            target_data, mask_idx=0
        )
        
        elapsed = (time.time() - start_time) * 1000  # ms
        
        print("-" * 80)
        
        # Results
        results = {
            'success': success,
            'matching_accuracy': target_data.matching_accuracy,
            'grasping_pose': target_data.grasping_pose,
            'is_feasible_pose': target_data.is_feasible_pose,
            'processing_time_ms': elapsed,
        }
        
        # Compare with recorded result if available
        if 'matching_result' in scan_data:
            recorded = scan_data['matching_result']
            
            print("\nComparison with recorded result:")
            print(f"  Recorded accuracy: {recorded.get('matching_accuracy', 0):.2f}%")
            print(f"  Replayed accuracy: {results['matching_accuracy']:.2f}%")
            
            if 'pose' in recorded and len(recorded['pose']) > 0 and len(results['grasping_pose']) > 0:
                recorded_pose = np.array(recorded['pose'])
                replayed_pose = np.array(results['grasping_pose'])
                
                pos_error = np.linalg.norm(recorded_pose[:3] - replayed_pose[:3]) * 1000  # mm
                ori_error = np.linalg.norm(recorded_pose[3:] - replayed_pose[3:])  # deg
                
                print(f"  Position error: {pos_error:.2f} mm")
                print(f"  Orientation error: {ori_error:.2f} deg")
        
        # Visualize
        if visualize and target_data.cloud_matching_results is not None:
            print("\nVisualizing results...")
            self.visualize_results(target_data)
        
        return results
    
    def visualize_results(self, target_data: TargetObjectData):
        """
        Visualize matching results.
        
        Args:
            target_data: Target object data with results
        """
        vis_clouds = []
        
        # Measured cloud (red)
        if target_data.cloud_pre_matching_results is not None:
            measured_vis = target_data.cloud_pre_matching_results
            measured_vis.paint_uniform_color([1, 0, 0])
            vis_clouds.append(measured_vis)
        
        # Matched cloud (green)
        if target_data.cloud_matching_results is not None:
            matched_vis = target_data.cloud_matching_results
            matched_vis.paint_uniform_color([0, 1, 0])
            vis_clouds.append(matched_vis)
        
        # CAD model (blue)
        if target_data.cloud_CAD_with_normal is not None:
            cad_vis = target_data.cloud_CAD_with_normal.transform(target_data.T_process_matching)
            cad_vis.paint_uniform_color([0, 0, 1])
            vis_clouds.append(cad_vis)
        
        # Grasp frame
        if len(target_data.grasping_pose) > 0:
            from hanyang_matching_open3d.modules.utils import pose_vec_to_matrix
            T_grasp = pose_vec_to_matrix(target_data.grasping_pose)
            
            coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
            coord_frame.transform(T_grasp)
            vis_clouds.append(coord_frame)
        
        # Visualize
        o3d.visualization.draw_geometries(
            vis_clouds,
            window_name="Replay Results",
            width=1280,
            height=720
        )


def main(args=None):
    """Main function"""
    import sys
    
    # Parse command line arguments
    if len(sys.argv) < 2:
        print("Usage: replay_test_data.py <data_directory> [--no-vis]")
        print("\nExample:")
        print("  replay_test_data.py /tmp/matching_test_data_20251022_143000")
        print("  replay_test_data.py /tmp/matching_test_data_20251022_143000 --no-vis")
        sys.exit(1)
    
    data_dir = sys.argv[1]
    visualize = '--no-vis' not in sys.argv
    
    if not os.path.exists(data_dir):
        print(f"Error: Directory not found: {data_dir}")
        sys.exit(1)
    
    # Create replayer
    replayer = DataReplayer(data_dir)
    
    # List available scans
    scans = replayer.list_scans()
    
    print(f"\nFound {len(scans)} scans")
    
    if len(scans) == 0:
        print("No scans found in directory")
        sys.exit(1)
    
    # Replay each scan
    all_results = []
    
    for i, scan_dir in enumerate(scans):
        print(f"\n{'=' * 80}")
        print(f"SCAN {i + 1}/{len(scans)}")
        print(f"{'=' * 80}")
        
        # Load scan data
        scan_data = replayer.load_scan_data(scan_dir)
        
        # Replay matching
        results = replayer.replay_scan(scan_data, visualize=visualize)
        
        all_results.append(results)
        
        print(f"\nResults:")
        print(f"  Success: {results.get('success', False)}")
        print(f"  Accuracy: {results.get('matching_accuracy', 0):.2f}%")
        print(f"  Processing time: {results.get('processing_time_ms', 0):.1f} ms")
    
    # Summary
    print("\n" + "=" * 80)
    print("REPLAY SUMMARY")
    print("=" * 80)
    
    success_count = sum(1 for r in all_results if r.get('success', False))
    total_count = len(all_results)
    success_rate = (success_count / total_count * 100) if total_count > 0 else 0
    
    accuracies = [r.get('matching_accuracy', 0) for r in all_results if r.get('success', False)]
    avg_accuracy = np.mean(accuracies) if len(accuracies) > 0 else 0
    
    times = [r.get('processing_time_ms', 0) for r in all_results]
    avg_time = np.mean(times) if len(times) > 0 else 0
    
    print(f"Total scans: {total_count}")
    print(f"Successful: {success_count} ({success_rate:.1f}%)")
    print(f"Average accuracy: {avg_accuracy:.2f}%")
    print(f"Average processing time: {avg_time:.1f} ms")
    print("=" * 80)


if __name__ == '__main__':
    main()

