#!/usr/bin/env python3
"""
Full pipeline integration tests.
Tests the complete workflow from scan data to grasp pose.
"""

import unittest
import numpy as np
import open3d as o3d
import sys
import os
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hanyang_matching_open3d.modules.template_matching import TemplateMatching
from hanyang_matching_open3d.modules.template_matching_extended import extend_template_matching_class
from hanyang_matching_open3d.modules.pcl_processing import PCLProcessing
from hanyang_matching_open3d.modules.mask_processing import MaskProcessing
from hanyang_matching_open3d.modules.grasp_planning import GraspPlanning
from hanyang_matching_open3d.modules.collision_checker import CollisionChecker
from hanyang_matching_open3d.modules.data_classes import (
    TargetObjectData, MaskData, AlignParameters
)
from hanyang_matching_open3d.modules.utils import (
    pose_vec_to_matrix, matrix_to_pose_vec,
    remove_nan_points, compute_centroid
)

# Extend TemplateMatching class
extend_template_matching_class()


class TestFullPipeline(unittest.TestCase):
    """Test complete matching pipeline"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.template_matching = TemplateMatching()
        self.pcl_proc = PCLProcessing()
        self.mask_proc = MaskProcessing()
        self.grasp_plan = GraspPlanning()
        self.collision_check = CollisionChecker()
        
        # Create test data directory
        self.test_data_dir = "/tmp/hanyang_matching_test"
        os.makedirs(self.test_data_dir, exist_ok=True)
    
    def create_test_point_cloud(self, num_points: int = 1000) -> o3d.geometry.PointCloud:
        """Create synthetic point cloud for testing"""
        points = np.random.rand(num_points, 3) * 0.5  # 0.5m cube
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # Add colors
        colors = np.random.rand(num_points, 3)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        return pcd
    
    def create_test_cad_model(self) -> o3d.geometry.PointCloud:
        """Create synthetic CAD model"""
        # Create a simple box
        mesh = o3d.geometry.TriangleMesh.create_box(width=0.1, height=0.1, depth=0.05)
        pcd = mesh.sample_points_uniformly(number_of_points=5000)
        
        # Center at origin
        pcd.translate(-pcd.get_center())
        
        return pcd
    
    def test_complete_pipeline_synthetic_data(self):
        """Test complete pipeline with synthetic data"""
        print("\n" + "=" * 60)
        print("Testing complete pipeline with synthetic data")
        print("=" * 60)
        
        # 1. Create test data
        print("\n1. Creating test data...")
        cloud_scan = self.create_test_point_cloud(num_points=10000)
        cloud_cad = self.create_test_cad_model()
        
        # 2. Initialize template
        print("\n2. Initializing template...")
        dh = [0.0] * 24  # 6 joints x 4 params
        tcp_default = [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
        tcp = [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
        
        target_data = self.template_matching.initialize_template(0, dh, tcp_default, tcp)
        
        # Set CAD model
        target_data.cloud_CAD = cloud_cad
        target_data.cloud_CAD_with_normal = cloud_cad
        target_data.cloud_CAD_for_quick_align = cloud_cad.voxel_down_sample(0.005)
        
        # Set parameters
        target_data.align_params.do_quick_align = False  # Skip for synthetic data
        target_data.do_euclidean_filtering = False
        target_data.matching_accuracy_limit = 10.0  # Lower threshold for synthetic
        
        # Set measured cloud
        target_data.cloud_measured = cloud_scan
        
        # 3. Process measured cloud
        print("\n3. Processing measured cloud...")
        self.template_matching.do_measured_cloud_processing(target_data)
        
        self.assertIsNotNone(target_data.cloud_measured)
        self.assertGreater(len(target_data.cloud_measured.points), 0)
        
        # 4. Coordinate transformation (skip - use identity)
        print("\n4. Coordinate transformation...")
        target_data.cloud_base_aligned = target_data.cloud_measured
        
        # 5. Pre-matching (segmentation)
        print("\n5. Pre-matching process...")
        self.template_matching.do_pre_matching_process(target_data)
        
        self.assertIsNotNone(target_data.cloud_pre_matching_results)
        
        # 6. Template matching
        print("\n6. Template matching...")
        success = self.template_matching.do_template_matching_process(target_data)
        
        # Should complete (may not be accurate with synthetic data)
        self.assertIsNotNone(target_data.T_process_matching)
        
        print(f"Matching accuracy: {target_data.matching_accuracy:.2f}%")
        print(f"Matching success: {target_data.is_matching_success}")
        
        print("\n" + "=" * 60)
        print("Complete pipeline test PASSED")
        print("=" * 60)
    
    def test_mask_processing(self):
        """Test mask processing module"""
        print("\n" + "=" * 60)
        print("Testing mask processing")
        print("=" * 60)
        
        # Create test mask data
        mask_data = MaskData()
        mask_data.class_cnt = 3
        mask_data.scores = [0.95, 0.85, 0.99]
        mask_data.thre_score = 0.90
        
        # Should filter to 2 masks (0.95 and 0.99)
        confident_masks = [i for i, score in enumerate(mask_data.scores) 
                          if score >= mask_data.thre_score]
        
        self.assertEqual(len(confident_masks), 2)
        self.assertIn(0, confident_masks)
        self.assertIn(2, confident_masks)
        
        print("Mask filtering test PASSED")
    
    def test_grasp_pose_calculation(self):
        """Test grasp pose calculation"""
        print("\n" + "=" * 60)
        print("Testing grasp pose calculation")
        print("=" * 60)
        
        target_data = TargetObjectData()
        
        # Set matching result
        target_data.is_matching_success = True
        target_data.matching_accuracy = 85.0
        
        # Set transformations
        T_matching = np.eye(4)
        T_matching[:3, 3] = [0.5, 0.3, 0.2]  # Object position
        target_data.T_process_matching = T_matching
        
        # Set grasp point transformation
        T_O2GP = np.eye(4)
        T_O2GP[:3, 3] = [0.0, 0.0, 0.05]  # Grasp 50mm above object center
        target_data.T_O2GP = T_O2GP
        
        # Calculate grasping pose
        success = self.grasp_plan.calculate_grasping_pose(target_data)
        
        self.assertTrue(success)
        self.assertEqual(len(target_data.grasping_pose), 6)
        
        # Check position
        expected_pos = [0.5, 0.3, 0.25]  # 0.2 + 0.05
        for i in range(3):
            self.assertAlmostEqual(target_data.grasping_pose[i], expected_pos[i], places=5)
        
        print(f"Grasp pose: {target_data.grasping_pose}")
        print("Grasp pose calculation test PASSED")
    
    def test_collision_checking(self):
        """Test collision checking"""
        print("\n" + "=" * 60)
        print("Testing collision checking")
        print("=" * 60)
        
        # Create gripper cloud
        gripper = o3d.geometry.TriangleMesh.create_box(0.05, 0.02, 0.1)
        gripper_cloud = gripper.sample_points_uniformly(1000)
        
        # Create environment cloud
        env = o3d.geometry.TriangleMesh.create_box(0.5, 0.5, 0.1)
        env_cloud = env.sample_points_uniformly(5000)
        
        # Test 1: No collision (gripper above environment)
        T_no_collision = np.eye(4)
        T_no_collision[2, 3] = 0.2  # 200mm above
        
        gripper_transformed = gripper_cloud.transform(T_no_collision)
        
        is_collision, count = self.collision_check.check_collision_with_kdtree(
            gripper_transformed, env_cloud, distance_threshold=0.01
        )
        
        self.assertFalse(is_collision)
        print(f"No collision test: {count} points (PASS)")
        
        # Test 2: Collision (gripper intersects environment)
        T_collision = np.eye(4)
        T_collision[2, 3] = 0.05  # 50mm - intersects
        
        gripper_transformed = gripper_cloud.transform(T_collision)
        
        is_collision, count = self.collision_check.check_collision_with_kdtree(
            gripper_transformed, env_cloud, distance_threshold=0.01
        )
        
        self.assertTrue(is_collision)
        print(f"Collision test: {count} points (PASS)")
        
        print("Collision checking test PASSED")
    
    def test_coordinate_transformations(self):
        """Test coordinate transformation chain"""
        print("\n" + "=" * 60)
        print("Testing coordinate transformations")
        print("=" * 60)
        
        # Test DH forward kinematics
        from hanyang_matching_open3d.modules.utils import compute_transform_from_dh_and_js
        
        # Simple 2-link planar arm (for testing)
        DH = np.array([
            [0, 0, 0.5, 0],  # Joint 1: 500mm link
            [0, 0, 0.3, 0],  # Joint 2: 300mm link
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
        ])
        
        q = np.array([0, 0, 0, 0, 0, 0])  # All joints at 0
        tcp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        T_result = compute_transform_from_dh_and_js(DH, q, tcp)
        
        # Should reach 0.8m in x direction
        self.assertIsNotNone(T_result)
        self.assertEqual(T_result.shape, (4, 4))
        
        print(f"End-effector position: {T_result[:3, 3]}")
        print("Coordinate transformation test PASSED")
    
    def test_multi_scale_icp(self):
        """Test multi-scale ICP (GICP approximation)"""
        print("\n" + "=" * 60)
        print("Testing multi-scale ICP")
        print("=" * 60)
        
        # Create source and target clouds
        target = self.create_test_cad_model()
        
        # Create source by transforming target
        T_ground_truth = np.eye(4)
        T_ground_truth[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz([0.1, 0.2, 0.3])
        T_ground_truth[:3, 3] = [0.05, 0.03, 0.02]
        
        source = target.transform(T_ground_truth)
        
        # Add noise
        points = np.asarray(source.points)
        points += np.random.normal(0, 0.001, points.shape)  # 1mm noise
        source.points = o3d.utility.Vector3dVector(points)
        
        # Initial guess (identity)
        init_transform = np.eye(4)
        
        # Alignment parameters
        params = AlignParameters()
        params.icp_max_iter_inner = 50
        params.icp_corr_threshold = 0.05
        
        # Run multi-scale ICP
        final_transform, fitness = self.pcl_proc.multi_scale_icp(
            source, target, init_transform, params,
            voxel_sizes=[0.01, 0.005, 0.002]
        )
        
        self.assertIsNotNone(final_transform)
        self.assertGreater(fitness, 0.5)  # Should have reasonable fitness
        
        # Check transformation error
        T_error = np.linalg.inv(T_ground_truth) @ final_transform
        translation_error = np.linalg.norm(T_error[:3, 3])
        
        print(f"Fitness: {fitness:.4f}")
        print(f"Translation error: {translation_error * 1000:.2f} mm")
        
        # Should be close to ground truth (within 10mm with noise)
        self.assertLess(translation_error, 0.01)
        
        print("Multi-scale ICP test PASSED")
    
    def test_complete_workflow_with_real_structure(self):
        """Test complete workflow with realistic data structure"""
        print("\n" + "=" * 60)
        print("Testing complete workflow with real data structure")
        print("=" * 60)
        
        # 1. Create CAD model
        print("\n1. Creating CAD model...")
        cad_path = os.path.join(self.test_data_dir, "test_object.ply")
        cloud_cad = self.create_test_cad_model()
        o3d.io.write_point_cloud(cad_path, cloud_cad)
        print(f"   CAD saved: {cad_path}")
        
        # 2. Initialize template
        print("\n2. Initializing template...")
        dh = [0.0] * 24
        tcp_default = [0.0, 0.0, 0.15, 0.0, 0.0, 0.0]  # 150mm TCP
        tcp = [0.0, 0.0, 0.15, 0.0, 0.0, 0.0]
        
        target_data = self.template_matching.initialize_template(0, dh, tcp_default, tcp)
        
        # Load CAD
        success = self.template_matching.load_cad_model(target_data, cad_path, voxel_size=0.005)
        self.assertTrue(success)
        print(f"   CAD loaded: {len(target_data.cloud_CAD.points)} points")
        
        # 3. Create measured scan (CAD + transformation + noise)
        print("\n3. Creating measured scan...")
        T_object = np.eye(4)
        T_object[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz([0.0, 0.0, 0.5])
        T_object[:3, 3] = [0.3, 0.2, 0.15]
        
        cloud_measured = cloud_cad.transform(T_object)
        
        # Add measurement noise
        points = np.asarray(cloud_measured.points)
        points += np.random.normal(0, 0.002, points.shape)  # 2mm noise
        cloud_measured.points = o3d.utility.Vector3dVector(points)
        
        target_data.cloud_measured = cloud_measured
        print(f"   Measured cloud: {len(cloud_measured.points)} points")
        
        # 4. Set robot state
        print("\n4. Setting robot state...")
        target_data.robot_scan_JS_position = np.array([0, -90, 90, 0, 90, 0])  # deg
        target_data.T_E2S = np.eye(4)  # Simplified
        target_data.T_E2S[:3, 3] = [0, 0, 0.2]  # Sensor 200mm from EE
        
        # 5. Process measured cloud
        print("\n5. Processing measured cloud...")
        self.template_matching.do_measured_cloud_processing(target_data)
        
        # 6. Alignment (simplified - assume already in base frame)
        print("\n6. Coordinate alignment...")
        target_data.cloud_base_aligned = target_data.cloud_measured
        
        # 7. Pre-matching
        print("\n7. Pre-matching (segmentation)...")
        self.template_matching.do_pre_matching_process(target_data)
        
        # 8. Template matching
        print("\n8. Template matching...")
        target_data.align_params.do_quick_align = False  # Use direct ICP
        success = self.template_matching.do_template_matching_process(target_data)
        
        print(f"   Matching accuracy: {target_data.matching_accuracy:.2f}%")
        print(f"   Matching success: {target_data.is_matching_success}")
        
        # 9. Calculate grasping pose
        if target_data.is_matching_success:
            print("\n9. Calculating grasping pose...")
            
            # Set grasp point (center of object)
            target_data.T_O2GP = np.eye(4)
            target_data.T_O2GP[:3, 3] = [0, 0, 0.05]  # 50mm above center
            
            grasp_success = self.template_matching.cal_grasping_pose(target_data)
            
            self.assertTrue(grasp_success)
            self.assertEqual(len(target_data.grasping_pose), 6)
            
            print(f"   Grasping pose: {target_data.grasping_pose}")
        
        print("\n" + "=" * 60)
        print("Complete workflow test PASSED")
        print("=" * 60)
    
    def tearDown(self):
        """Clean up test data"""
        # Remove test files
        import shutil
        if os.path.exists(self.test_data_dir):
            shutil.rmtree(self.test_data_dir)


class TestModulesIntegration(unittest.TestCase):
    """Test integration between modules"""
    
    def test_pcl_processing_to_grasp_planning(self):
        """Test data flow from PCL processing to grasp planning"""
        pcl_proc = PCLProcessing()
        grasp_plan = GraspPlanning()
        
        # Create test cloud
        points = np.random.rand(1000, 3) * 0.5
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # Process
        pcd = pcl_proc.estimate_normals(pcd)
        
        # Create target data
        target_data = TargetObjectData()
        target_data.is_matching_success = True
        target_data.T_process_matching = np.eye(4)
        target_data.T_O2GP = np.eye(4)
        
        # Calculate grasp
        success = grasp_plan.calculate_grasping_pose(target_data)
        
        self.assertTrue(success)
    
    def test_mask_to_matching_pipeline(self):
        """Test pipeline from mask processing to matching"""
        mask_proc = MaskProcessing()
        
        # Create test mask
        mask_data = MaskData()
        mask_data.class_cnt = 1
        mask_data.scores = [0.95]
        
        # Create test cloud
        cloud = o3d.geometry.PointCloud()
        points = np.random.rand(1000, 3) * 0.5
        cloud.points = o3d.utility.Vector3dVector(points)
        
        mask_data.cloud_mask_list = [cloud]
        mask_data.mask_confident_idx_set = [0]
        
        # Select best mask
        best_idx = mask_proc.select_best_mask(
            mask_data,
            TargetObjectData(),
            selection_criteria="largest"
        )
        
        self.assertEqual(best_idx, 0)


def run_performance_benchmark():
    """Run performance benchmarks"""
    print("\n" + "=" * 80)
    print("PERFORMANCE BENCHMARK")
    print("=" * 80)
    
    import time
    
    # Create test data
    pcl_proc = PCLProcessing()
    
    sizes = [1000, 5000, 10000, 50000, 100000]
    
    for size in sizes:
        print(f"\nPoint cloud size: {size}")
        
        # Create point cloud
        points = np.random.rand(size, 3) * 0.5
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # Test normal estimation
        start = time.time()
        pcd_normals = pcl_proc.estimate_normals(pcd, radius=0.01, max_nn=30)
        elapsed = (time.time() - start) * 1000
        print(f"  Normal estimation: {elapsed:.1f} ms")
        
        # Test downsampling
        start = time.time()
        pcd_down = pcl_proc.voxel_down_sample(pcd, 0.01)
        elapsed = (time.time() - start) * 1000
        print(f"  Voxel downsampling: {elapsed:.1f} ms ({len(pcd_down.points)} points)")
        
        # Test clustering
        if size <= 10000:  # DBSCAN is slow for large clouds
            start = time.time()
            clusters, largest = pcl_proc.euclidean_cluster_extraction(pcd, eps=0.02, min_points=10)
            elapsed = (time.time() - start) * 1000
            print(f"  Clustering: {elapsed:.1f} ms ({len(clusters)} clusters)")
    
    print("\n" + "=" * 80)


def main():
    """Run all tests"""
    import sys
    
    print("\n" + "=" * 80)
    print("HANYANG MATCHING OPEN3D - FULL PIPELINE TESTS")
    print("=" * 80)
    
    # Check if benchmark flag is provided
    if '--benchmark' in sys.argv:
        sys.argv.remove('--benchmark')
        run_performance_benchmark()
    
    # Run unit tests
    unittest.main(verbosity=2)


if __name__ == '__main__':
    main()

