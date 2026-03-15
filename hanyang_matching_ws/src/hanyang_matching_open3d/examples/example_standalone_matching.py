#!/usr/bin/env python3
"""
Standalone matching example without ROS.
Shows how to use the matching modules directly.
"""

import numpy as np
import open3d as o3d
import sys
import os

# Add package to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hanyang_matching_open3d.modules.template_matching import TemplateMatching
from hanyang_matching_open3d.modules.data_classes import TargetObjectData
from hanyang_matching_open3d.modules.utils import pose_vec_to_matrix, matrix_to_pose_vec


def main():
    """Main example function"""
    
    print("=" * 80)
    print("STANDALONE MATCHING EXAMPLE")
    print("=" * 80)
    
    # 1. Create template matching instance
    print("\n1. Initializing template matching...")
    tm = TemplateMatching()
    
    # 2. Initialize template
    print("\n2. Initializing template for target object...")
    
    # Robot parameters (example for UR10e)
    dh_params = [
        0, 0.1807, 0, 1.5708,      # Joint 1
        0, 0, -0.6127, 0,          # Joint 2
        0, 0, -0.57155, 0,         # Joint 3
        0, 0.17415, 0, 1.5708,     # Joint 4
        0, 0.11985, 0, -1.5708,    # Joint 5
        0, 0.11655, 0, 0           # Joint 6
    ]
    
    tcp_default = [0.0, 0.0, 0.15, 0.0, 0.0, 0.0]  # 150mm TCP
    tcp = [0.0, 0.0, 0.15, 0.0, 0.0, 0.0]
    
    target_data = tm.initialize_template(0, dh_params, tcp_default, tcp)
    
    # 3. Load or create CAD model
    print("\n3. Loading CAD model...")
    
    # Option A: Load from file (PCL compatible path)
    cad_paths = [
        os.path.expanduser(f"~/scanDataHanyang/CAD/bin picking/target_object/bolt.ply"),
        os.path.expanduser(f"~/scanDataHanyang/CAD/bin picking/target_object/bolt.pcd"),
        os.path.expanduser("~/cad_models/bolt.ply"),
        os.path.expanduser("~/cad_models/bolt.pcd"),
    ]
    
    cad_path = None
    for path in cad_paths:
        if os.path.exists(path):
            cad_path = path
            break
    
    if cad_path:
        tm.load_cad_model(target_data, cad_path, voxel_size=0.005)
        print(f"   Loaded from file: {cad_path}")
    else:
        # Option B: Create synthetic model
        print("   Creating synthetic CAD model...")
        mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=0.015, height=0.05)
        cloud_cad = mesh.sample_points_uniformly(number_of_points=10000)
        cloud_cad.translate(-cloud_cad.get_center())
        
        target_data.cloud_CAD = cloud_cad
        target_data.cloud_CAD_for_quick_align = cloud_cad.voxel_down_sample(0.005)
        target_data.cloud_CAD_with_normal = cloud_cad
        
        # Estimate normals
        tm.pcl_proc.estimate_normals(target_data.cloud_CAD_with_normal)
        
        print(f"   Created synthetic CAD: {len(cloud_cad.points)} points")
    
    # 4. Create or load measured scan
    print("\n4. Loading measured scan...")
    
    scan_file = "~/scan_data/test_scan.pcd"
    scan_file_expanded = os.path.expanduser(scan_file)
    
    if os.path.exists(scan_file_expanded):
        cloud_measured = o3d.io.read_point_cloud(scan_file_expanded)
        print(f"   Loaded from file: {scan_file_expanded}")
    else:
        # Create synthetic measured data
        print("   Creating synthetic measured data...")
        
        # Transform CAD to simulate measurement
        T_measurement = np.eye(4)
        T_measurement[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz([0.1, 0.2, 0.3])
        T_measurement[:3, 3] = [0.3, 0.2, 0.15]
        
        cloud_measured = target_data.cloud_CAD.transform(T_measurement)
        
        # Add noise
        points = np.asarray(cloud_measured.points)
        points += np.random.normal(0, 0.003, points.shape)  # 3mm noise
        cloud_measured.points = o3d.utility.Vector3dVector(points)
        
        # Add outliers
        num_outliers = 500
        outlier_points = np.random.rand(num_outliers, 3) * 0.8
        all_points = np.vstack([points, outlier_points])
        cloud_measured.points = o3d.utility.Vector3dVector(all_points)
        
        print(f"   Created synthetic scan: {len(cloud_measured.points)} points")
    
    target_data.cloud_measured = cloud_measured
    
    # 5. Set robot scan position
    print("\n5. Setting robot scan position...")
    target_data.robot_scan_JS_position = np.deg2rad(np.array([0, -90, 90, 0, 90, 0]))
    
    # 6. Set grasp parameters
    print("\n6. Setting grasp parameters...")
    target_data.T_O2GP = np.eye(4)
    target_data.T_O2GP[:3, 3] = [0, 0, 0.03]  # 30mm above object center
    target_data.approach_distance = 50.0  # 50mm approach distance
    
    # 7. Configure matching parameters
    print("\n7. Configuring matching parameters...")
    target_data.align_params.do_quick_align = True
    target_data.align_params.icp_max_iter_inner = 50
    target_data.do_euclidean_filtering = True
    target_data.matching_accuracy_limit = 30.0  # Lower for synthetic data
    
    # 8. Run matching pipeline
    print("\n8. Running matching pipeline...")
    print("-" * 80)
    
    success = tm.icp_matching_single_process(target_data, mask_idx=0)
    
    print("-" * 80)
    
    # 9. Display results
    print("\n9. Matching results:")
    print("=" * 80)
    
    if target_data.is_matching_success:
        print(f"✅ MATCHING SUCCESS")
        print(f"   Accuracy: {target_data.matching_accuracy:.2f}%")
        print(f"   Grasping pose: {target_data.grasping_pose}")
        print(f"   Gripper open: {target_data.gripper_open_length} mm")
        print(f"   Gripper close: {target_data.gripper_close_length} mm")
        print(f"   Feasible: {target_data.is_feasible_pose}")
    else:
        print(f"❌ MATCHING FAILED")
        print(f"   Accuracy: {target_data.matching_accuracy:.2f}%")
        print(f"   Threshold: {target_data.matching_accuracy_limit:.2f}%")
    
    print("=" * 80)
    
    # 10. Visualize results (optional)
    print("\n10. Visualization (press 'q' to close)...")
    
    if target_data.cloud_matching_results is not None:
        # Prepare visualization
        vis_clouds = []
        
        # Matched cloud (green)
        matched = target_data.cloud_matching_results
        matched.paint_uniform_color([0, 1, 0])
        vis_clouds.append(matched)
        
        # CAD model (blue)
        cad_vis = target_data.cloud_CAD_with_normal.transform(target_data.T_process_matching)
        cad_vis.paint_uniform_color([0, 0, 1])
        vis_clouds.append(cad_vis)
        
        # Show coordinate frame at grasp point
        if len(target_data.grasping_pose) > 0:
            T_grasp = target_data.T_O2GP_final
            coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
            coord_frame.transform(T_grasp)
            vis_clouds.append(coord_frame)
        
        # Visualize
        o3d.visualization.draw_geometries(
            vis_clouds,
            window_name="Matching Results",
            width=1280,
            height=720
        )
    
    print("\nExample completed!")
    
    return 0 if target_data.is_matching_success else 1


if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code)

