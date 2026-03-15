#!/usr/bin/env python3
"""
Basic functionality tests for Open3D matching modules.
"""

import unittest
import numpy as np
import open3d as o3d
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hanyang_matching_open3d.modules.pcl_processing import PCLProcessing
from hanyang_matching_open3d.modules.data_classes import TargetObjectData, AlignParameters
from hanyang_matching_open3d.modules.utils import (
    pose_vec_to_matrix, matrix_to_pose_vec,
    remove_nan_points, compute_centroid
)


class TestUtils(unittest.TestCase):
    """Test utility functions"""
    
    def test_pose_conversion(self):
        """Test pose vector to matrix conversion and back"""
        pose_vec = [1.0, 2.0, 3.0, 45.0, 30.0, 60.0]  # x, y, z, rx, ry, rz (deg)
        
        # Convert to matrix
        T = pose_vec_to_matrix(pose_vec)
        
        # Check translation
        self.assertAlmostEqual(T[0, 3], 1.0, places=5)
        self.assertAlmostEqual(T[1, 3], 2.0, places=5)
        self.assertAlmostEqual(T[2, 3], 3.0, places=5)
        
        # Convert back
        pose_vec_back = matrix_to_pose_vec(T)
        
        # Check roundtrip
        for i in range(6):
            self.assertAlmostEqual(pose_vec[i], pose_vec_back[i], places=3)
    
    def test_remove_nan_points(self):
        """Test NaN point removal"""
        # Create point cloud with NaN
        points = np.array([
            [1.0, 2.0, 3.0],
            [np.nan, 5.0, 6.0],
            [7.0, 8.0, 9.0],
            [10.0, np.nan, 12.0]
        ])
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # Remove NaN
        pcd_clean = remove_nan_points(pcd)
        
        # Should have 2 valid points
        self.assertEqual(len(pcd_clean.points), 2)
    
    def test_compute_centroid(self):
        """Test centroid computation"""
        points = np.array([
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [0.0, 2.0, 0.0],
            [0.0, 0.0, 2.0]
        ])
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        centroid = compute_centroid(pcd)
        
        # Centroid should be (0.5, 0.5, 0.5)
        expected = np.array([0.5, 0.5, 0.5])
        np.testing.assert_array_almost_equal(centroid, expected, decimal=5)


class TestPCLProcessing(unittest.TestCase):
    """Test PCL processing functions"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.pcl_proc = PCLProcessing()
        
        # Create test point cloud
        points = np.random.rand(1000, 3)
        self.test_cloud = o3d.geometry.PointCloud()
        self.test_cloud.points = o3d.utility.Vector3dVector(points)
    
    def test_voxel_downsample(self):
        """Test voxel downsampling"""
        voxel_size = 0.1
        downsampled = self.pcl_proc.voxel_down_sample(self.test_cloud, voxel_size)
        
        # Should have fewer points
        self.assertLess(len(downsampled.points), len(self.test_cloud.points))
        self.assertGreater(len(downsampled.points), 0)
    
    def test_normal_estimation(self):
        """Test normal estimation"""
        pcd_with_normals = self.pcl_proc.estimate_normals(
            self.test_cloud, radius=0.1, max_nn=30
        )
        
        # Should have normals
        self.assertTrue(pcd_with_normals.has_normals())
        self.assertEqual(len(pcd_with_normals.normals), len(pcd_with_normals.points))
    
    def test_fpfh_feature(self):
        """Test FPFH feature computation"""
        # Estimate normals first
        pcd = self.pcl_proc.estimate_normals(self.test_cloud, radius=0.1, max_nn=30)
        
        # Compute FPFH
        fpfh = self.pcl_proc.compute_fpfh_feature(pcd, radius=0.2, max_nn=100)
        
        # Should have features
        self.assertGreater(fpfh.num(), 0)
    
    def test_euclidean_clustering(self):
        """Test Euclidean clustering"""
        clusters, largest = self.pcl_proc.euclidean_cluster_extraction(
            self.test_cloud, eps=0.1, min_points=10, max_points=500
        )
        
        # Should find at least one cluster
        self.assertGreater(len(clusters), 0)
        self.assertGreater(len(largest.points), 0)


class TestDataClasses(unittest.TestCase):
    """Test data classes"""
    
    def test_target_object_data_initialization(self):
        """Test TargetObjectData initialization"""
        data = TargetObjectData()
        
        # Check default values
        self.assertEqual(data.target_id, 0)
        self.assertFalse(data.is_matching_success)
        self.assertEqual(data.matching_accuracy, 0.0)
        
        # Check transformation matrices are identity
        np.testing.assert_array_almost_equal(data.T_B2S, np.eye(4))
    
    def test_align_parameters(self):
        """Test AlignParameters"""
        params = AlignParameters()
        
        # Check defaults
        self.assertFalse(params.do_quick_align)
        self.assertEqual(params.icp_max_iter_inner, 50)
        self.assertGreater(params.icp_corr_threshold, 0)


class TestTemplateMatching(unittest.TestCase):
    """Test template matching functionality"""
    
    def test_initialization(self):
        """Test TemplateMatching initialization"""
        from hanyang_matching_open3d.modules.template_matching import TemplateMatching
        
        tm = TemplateMatching()
        
        # Should have empty template lists
        self.assertEqual(len(tm.template_list), 0)
        self.assertEqual(len(tm.template_tool_list), 0)
    
    def test_initialize_template(self):
        """Test template initialization"""
        from hanyang_matching_open3d.modules.template_matching import TemplateMatching
        
        tm = TemplateMatching()
        
        # Initialize template
        dh = [0.0] * 24  # 6 joints x 4 params
        tcp_default = [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
        tcp = [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
        
        target_data = tm.initialize_template(0, dh, tcp_default, tcp)
        
        # Check initialization
        self.assertEqual(target_data.target_id, 0)
        self.assertEqual(len(tm.template_list), 1)


def main():
    """Run tests"""
    unittest.main()


if __name__ == '__main__':
    main()

