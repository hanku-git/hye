"""
Point cloud processing module - equivalent to pcl_processing.cpp
Provides filtering, segmentation, registration, and feature extraction.
"""

import numpy as np
import open3d as o3d
from typing import List, Tuple, Optional
from .data_classes import AlignParameters


class PCLProcessing:
    """Point cloud processing class"""
    
    def __init__(self):
        """Initialize PCL processing module"""
        pass
    
    def remove_nan_points(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """
        Remove NaN points from point cloud.
        
        Args:
            pcd: Input point cloud
            
        Returns:
            Cleaned point cloud
        """
        points = np.asarray(pcd.points)
        valid_mask = ~np.isnan(points).any(axis=1)
        
        pcd_clean = o3d.geometry.PointCloud()
        pcd_clean.points = o3d.utility.Vector3dVector(points[valid_mask])
        
        if pcd.has_colors():
            colors = np.asarray(pcd.colors)
            pcd_clean.colors = o3d.utility.Vector3dVector(colors[valid_mask])
        
        if pcd.has_normals():
            normals = np.asarray(pcd.normals)
            pcd_clean.normals = o3d.utility.Vector3dVector(normals[valid_mask])
        
        return pcd_clean
    
    def filter_cloud_using_view_frustum(self, pcd: o3d.geometry.PointCloud,
                                       view_frustum: List[float]) -> o3d.geometry.PointCloud:
        """
        Filter point cloud using PassThrough-like AABB filtering.
        Equivalent to PCL's PassThrough filter.
        
        Args:
            pcd: Input point cloud
            view_frustum: [min_x, max_x, min_y, max_y, min_z, max_z]
            
        Returns:
            Filtered point cloud
        """
        if len(view_frustum) != 6:
            print(f"Warning: view_frustum has {len(view_frustum)} elements, expected 6")
            return pcd
        
        min_bound = np.array([view_frustum[0], view_frustum[2], view_frustum[4]])
        max_bound = np.array([view_frustum[1], view_frustum[3], view_frustum[5]])
        
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
        pcd_filtered = pcd.crop(bbox)
        
        print(f"Filtered cloud size: {len(pcd_filtered.points)} (from {len(pcd.points)})")
        
        return pcd_filtered
    
    def voxel_down_sample(self, pcd: o3d.geometry.PointCloud,
                         voxel_size: float) -> o3d.geometry.PointCloud:
        """
        Voxel grid downsampling.
        
        Args:
            pcd: Input point cloud
            voxel_size: Voxel size (e.g., 0.005 for 5mm)
            
        Returns:
            Downsampled point cloud
        """
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        print(f"Downsampled: {len(pcd.points)} -> {len(pcd_down.points)} points")
        return pcd_down
    
    def estimate_normals(self, pcd: o3d.geometry.PointCloud,
                        radius: float = 0.01,
                        max_nn: int = 30,
                        sensor_origin: Optional[np.ndarray] = None) -> o3d.geometry.PointCloud:
        """
        Estimate normals using KDTree search.
        Equivalent to PCL's NormalEstimation.
        
        Args:
            pcd: Input point cloud
            radius: Search radius
            max_nn: Maximum number of neighbors
            sensor_origin: If provided, orient normals towards this point
            
        Returns:
            Point cloud with normals
        """
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
        )
        
        # Orient normals towards sensor if provided
        if sensor_origin is not None:
            pcd.orient_normals_towards_camera_location(camera_location=sensor_origin)
        
        return pcd
    
    def statistical_outlier_removal(self, pcd: o3d.geometry.PointCloud,
                                    nb_neighbors: int = 20,
                                    std_ratio: float = 2.0) -> o3d.geometry.PointCloud:
        """
        Statistical outlier removal.
        Equivalent to PCL's StatisticalOutlierRemoval.
        
        Args:
            pcd: Input point cloud
            nb_neighbors: Number of neighbors to analyze
            std_ratio: Standard deviation multiplier
            
        Returns:
            Filtered point cloud
        """
        pcd_filtered, ind = pcd.remove_statistical_outlier(
            nb_neighbors=nb_neighbors,
            std_ratio=std_ratio
        )
        
        print(f"Statistical outlier removal: {len(pcd.points)} -> {len(pcd_filtered.points)}")
        
        return pcd_filtered
    
    def euclidean_cluster_extraction(self, pcd: o3d.geometry.PointCloud,
                                    eps: float = 0.02,
                                    min_points: int = 100,
                                    max_points: int = 25000) -> Tuple[List[o3d.geometry.PointCloud], o3d.geometry.PointCloud]:
        """
        Euclidean cluster extraction using DBSCAN.
        Equivalent to PCL's EuclideanClusterExtraction.
        
        Args:
            pcd: Input point cloud
            eps: Maximum distance between points in a cluster
            min_points: Minimum number of points in a cluster
            max_points: Maximum number of points in a cluster
            
        Returns:
            Tuple of (list of clusters, largest cluster)
        """
        # DBSCAN clustering
        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
        
        max_label = labels.max()
        print(f"Point cloud has {max_label + 1} clusters")
        
        clusters = []
        cluster_sizes = []
        
        for i in range(max_label + 1):
            cluster_indices = np.where(labels == i)[0]
            cluster_size = len(cluster_indices)
            
            # Filter by max_points
            if cluster_size > max_points:
                continue
            
            cluster = pcd.select_by_index(cluster_indices)
            clusters.append(cluster)
            cluster_sizes.append(cluster_size)
        
        # Find largest cluster
        if len(clusters) > 0:
            largest_idx = np.argmax(cluster_sizes)
            largest_cluster = clusters[largest_idx]
            print(f"Largest cluster has {cluster_sizes[largest_idx]} points")
        else:
            largest_cluster = o3d.geometry.PointCloud()
        
        return clusters, largest_cluster
    
    def extract_major_segment(self, pcd: o3d.geometry.PointCloud,
                             eps: float = 0.02,
                             min_points: int = 100,
                             max_points: int = 25000) -> o3d.geometry.PointCloud:
        """
        Extract the largest cluster from point cloud.
        
        Args:
            pcd: Input point cloud
            eps: Cluster tolerance
            min_points: Minimum cluster size
            max_points: Maximum cluster size
            
        Returns:
            Largest cluster
        """
        _, largest_cluster = self.euclidean_cluster_extraction(
            pcd, eps, min_points, max_points
        )
        
        return largest_cluster
    
    def compute_fpfh_feature(self, pcd: o3d.geometry.PointCloud,
                            radius: float = 0.05,
                            max_nn: int = 100) -> o3d.pipelines.registration.Feature:
        """
        Compute FPFH (Fast Point Feature Histograms) features.
        Equivalent to PCL's FPFHEstimation.
        
        Args:
            pcd: Input point cloud (must have normals)
            radius: Feature search radius
            max_nn: Maximum number of neighbors
            
        Returns:
            FPFH features
        """
        # Ensure normals are computed
        if not pcd.has_normals():
            self.estimate_normals(pcd, radius=radius/5.0, max_nn=30)
        
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
        )
        
        return fpfh
    
    def quick_align_using_features(self, source: o3d.geometry.PointCloud,
                                   target: o3d.geometry.PointCloud,
                                   source_fpfh: o3d.pipelines.registration.Feature,
                                   target_fpfh: o3d.pipelines.registration.Feature,
                                   params: AlignParameters) -> Tuple[np.ndarray, float]:
        """
        Quick alignment using RANSAC-based feature matching.
        Equivalent to PCL's SampleConsensusInitialAlignment (SAC-IA).
        
        Args:
            source: Source point cloud
            target: Target point cloud
            source_fpfh: Source FPFH features
            target_fpfh: Target FPFH features
            params: Alignment parameters
            
        Returns:
            Tuple of (transformation matrix, fitness score)
        """
        # RANSAC-based registration
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source, target, source_fpfh, target_fpfh,
            mutual_filter=True,
            max_correspondence_distance=params.quick_align_max_correspondence_distance,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            ransac_n=params.quick_align_sample_num,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    params.quick_align_max_correspondence_distance
                )
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
                max_iteration=params.quick_align_iteration,
                confidence=0.999
            )
        )
        
        print(f"Quick align fitness: {result.fitness}")
        
        return result.transformation, result.fitness
    
    def icp_registration(self, source: o3d.geometry.PointCloud,
                        target: o3d.geometry.PointCloud,
                        init_transformation: np.ndarray,
                        params: AlignParameters,
                        use_point_to_plane: bool = True) -> Tuple[np.ndarray, float]:
        """
        ICP (Iterative Closest Point) registration.
        Equivalent to PCL's IterativeClosestPointWithNormals.
        
        Args:
            source: Source point cloud
            target: Target point cloud
            init_transformation: Initial transformation guess
            params: Alignment parameters
            use_point_to_plane: If True, use point-to-plane ICP; otherwise point-to-point
            
        Returns:
            Tuple of (transformation matrix, fitness score)
        """
        # Ensure normals are computed for point-to-plane ICP
        if use_point_to_plane:
            if not source.has_normals():
                self.estimate_normals(source, radius=params.normal_radius, max_nn=params.normal_max_nn)
            if not target.has_normals():
                self.estimate_normals(target, radius=params.normal_radius, max_nn=params.normal_max_nn)
            
            estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()
        else:
            estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        
        # ICP registration
        result = o3d.pipelines.registration.registration_icp(
            source, target,
            max_correspondence_distance=params.icp_corr_threshold,
            init=init_transformation,
            estimation_method=estimation,
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=params.icp_max_iter_inner,
                relative_fitness=params.icp_euclidean_fitness_eps,
                relative_rmse=params.icp_transformation_eps
            )
        )
        
        print(f"ICP converged: fitness={result.fitness:.6f}, inlier_rmse={result.inlier_rmse:.6f}")
        
        return result.transformation, result.fitness
    
    def multi_scale_icp(self, source: o3d.geometry.PointCloud,
                       target: o3d.geometry.PointCloud,
                       init_transformation: np.ndarray,
                       params: AlignParameters,
                       voxel_sizes: List[float] = [0.05, 0.02, 0.01]) -> Tuple[np.ndarray, float]:
        """
        Multi-scale ICP registration (approximation of GICP).
        Uses multiple voxel grid resolutions for robustness.
        
        Args:
            source: Source point cloud
            target: Target point cloud
            init_transformation: Initial transformation
            params: Alignment parameters
            voxel_sizes: List of voxel sizes from coarse to fine
            
        Returns:
            Tuple of (transformation matrix, fitness score)
        """
        current_transformation = init_transformation.copy()
        best_fitness = 0.0
        best_transformation = current_transformation.copy()
        
        # PCL-style ICP with accumulative transformation (핵심!)
        print("PCL-style ICP with accumulative transformation...")
        
        # Estimate normals for both clouds (PCL uses K=20)
        source_with_normals = self.estimate_normals(source, radius=0.01, max_nn=20)
        target_with_normals = self.estimate_normals(target, radius=0.01, max_nn=20)
        
        # Initial ICP with original parameters
        print("Initial ICP...")
        icp_result = o3d.pipelines.registration.registration_icp(
            source_with_normals, target_with_normals,
            max_correspondence_distance=params.icp_corr_threshold,
            init=current_transformation,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=params.icp_max_iter_inner,
                relative_fitness=params.icp_euclidean_fitness_eps,
                relative_rmse=params.icp_transformation_eps
            )
        )
        
        # Start with initial transformation
        transformation_matrix = np.eye(4)
        if icp_result.fitness > 0:
            transformation_matrix = icp_result.transformation
            print(f"Initial ICP fitness: {icp_result.fitness:.4f}")
        else:
            print("Initial ICP failed")
        
        # PCL-style progressive ICP with accumulative transformation
        corres = 5.0  # Start with 5mm correspondence distance
        iter_step = 1.0
        iter_cnt = 0
        max_iterations = 2  # PCL uses 2 additional iterations
        
        print("Progressive ICP with accumulative transformation...")
        while iter_cnt <= max_iterations:
            iter_cnt += 1
            new_corres = corres - iter_step
            print(f"Iteration {iter_cnt}: correspondence distance = {new_corres:.1f}mm")
            
            # Apply current transformation to source
            source_transformed = source_with_normals.transform(transformation_matrix)
            
            # ICP with reduced correspondence distance
            icp_result = o3d.pipelines.registration.registration_icp(
                source_transformed, target_with_normals,
                max_correspondence_distance=new_corres,
                init=np.eye(4),  # Start from identity since source is already transformed
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=params.icp_max_iter_inner,
                    relative_fitness=params.icp_euclidean_fitness_eps,
                    relative_rmse=params.icp_transformation_eps
                )
            )
            
            if icp_result.fitness > 0:
                # PCL 핵심: 누적 변환 (Accumulative Transformation)
                transformation_matrix = icp_result.transformation @ transformation_matrix
                print(f"Iteration {iter_cnt} fitness: {icp_result.fitness:.4f}, accumulative transformation applied")
            else:
                print(f"Iteration {iter_cnt} failed")
            
            iter_step += 0.5  # PCL uses 0.5 increment
        
        # Final fitness evaluation
        source_final = source_with_normals.transform(transformation_matrix)
        final_fitness = self.evaluate_registration(source_final, target_with_normals, np.eye(4))[0]
        
        print(f"Final accumulative transformation fitness: {final_fitness:.4f}")
        
        return transformation_matrix, final_fitness
    
    def robust_icp_with_outlier_rejection(self, 
                                        source: o3d.geometry.PointCloud,
                                        target: o3d.geometry.PointCloud,
                                        init_transformation: np.ndarray,
                                        params: AlignParameters,
                                        outlier_threshold: float = 0.1) -> Tuple[np.ndarray, float]:
        """
        Robust ICP with outlier rejection for improved accuracy
        Based on latest research on robust point cloud registration
        """
        print("Performing robust ICP with outlier rejection...")
        
        # Estimate normals
        source_with_normals = self.estimate_normals(source, radius=0.01, max_nn=20)
        target_with_normals = self.estimate_normals(target, radius=0.01, max_nn=20)
        
        # Multi-stage robust ICP with outlier rejection
        current_transformation = init_transformation.copy()
        best_fitness = 0.0
        best_transformation = current_transformation.copy()
        
        # Stage 1: Coarse alignment with high outlier threshold
        print("Stage 1: Coarse robust alignment...")
        coarse_result = o3d.pipelines.registration.registration_icp(
            source_with_normals, target_with_normals,
            max_correspondence_distance=params.icp_corr_threshold * 2.0,  # Larger threshold
            init=current_transformation,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=params.icp_max_iter_inner // 2,
                relative_fitness=params.icp_euclidean_fitness_eps * 10,
                relative_rmse=params.icp_transformation_eps * 10
            )
        )
        
        if coarse_result.fitness > 0:
            current_transformation = coarse_result.transformation
            best_fitness = coarse_result.fitness
            best_transformation = current_transformation.copy()
            print(f"Coarse alignment fitness: {coarse_result.fitness:.4f}")
        
        # Stage 2: Outlier rejection and refinement
        print("Stage 2: Outlier rejection and refinement...")
        
        # Transform source and find correspondences
        source_transformed = source_with_normals.transform(current_transformation)
        
        # Find correspondences with outlier rejection
        distances = source_transformed.compute_point_cloud_distance(target_with_normals)
        distances = np.asarray(distances)
        
        # Reject outliers based on distance threshold
        inlier_mask = distances < outlier_threshold
        inlier_indices = np.where(inlier_mask)[0]
        
        if len(inlier_indices) > 0:
            print(f"Outlier rejection: {len(inlier_indices)}/{len(distances)} points kept")
            
            # Create inlier-only source cloud
            source_inliers = source_with_normals.select_by_index(inlier_indices)
            
            # Refined ICP with inliers only
            refined_result = o3d.pipelines.registration.registration_icp(
                source_inliers, target_with_normals,
                max_correspondence_distance=params.icp_corr_threshold,
                init=current_transformation,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=params.icp_max_iter_inner,
                    relative_fitness=params.icp_euclidean_fitness_eps,
                    relative_rmse=params.icp_transformation_eps
                )
            )
            
            if refined_result.fitness > best_fitness:
                best_fitness = refined_result.fitness
                best_transformation = refined_result.transformation
                print(f"Refined alignment fitness: {refined_result.fitness:.4f}")
        
        # Stage 3: Final refinement with original clouds
        print("Stage 3: Final refinement...")
        final_result = o3d.pipelines.registration.registration_icp(
            source_with_normals, target_with_normals,
            max_correspondence_distance=params.icp_corr_threshold * 0.5,  # Tighter threshold
            init=best_transformation,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=params.icp_max_iter_inner,
                relative_fitness=params.icp_euclidean_fitness_eps,
                relative_rmse=params.icp_transformation_eps
            )
        )
        
        if final_result.fitness > best_fitness:
            best_fitness = final_result.fitness
            best_transformation = final_result.transformation
            print(f"Final refinement fitness: {final_result.fitness:.4f}")
        
        return best_transformation, best_fitness
    
    def fast_global_registration_icp(self, 
                                   source: o3d.geometry.PointCloud,
                                   target: o3d.geometry.PointCloud,
                                   params: AlignParameters) -> Tuple[np.ndarray, float]:
        """
        Fast Global Registration + ICP for improved initial alignment
        Based on FGR algorithm for better global registration
        """
        print("Performing Fast Global Registration + ICP...")
        
        # Estimate normals
        source_with_normals = self.estimate_normals(source, radius=0.01, max_nn=20)
        target_with_normals = self.estimate_normals(target, radius=0.01, max_nn=20)
        
        # Compute FPFH features
        print("Computing FPFH features...")
        source_fpfh = self.compute_fpfh_features(source_with_normals, radius=0.05)
        target_fpfh = self.compute_fpfh_features(target_with_normals, radius=0.05)
        
        # Fast Global Registration
        print("Performing Fast Global Registration...")
        fgr_result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            source_with_normals, target_with_normals,
            source_fpfh, target_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                division_factor=1.5,
                use_absolute_scale=False,
                decrease_mu=False,
                maximum_correspondence_distance=params.icp_corr_threshold,
                iteration_number=100
            )
        )
        
        print(f"FGR fitness: {fgr_result.fitness:.4f}")
        
        # Refine with ICP
        print("Refining with ICP...")
        icp_result = o3d.pipelines.registration.registration_icp(
            source_with_normals, target_with_normals,
            max_correspondence_distance=params.icp_corr_threshold,
            init=fgr_result.transformation,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=params.icp_max_iter_inner,
                relative_fitness=params.icp_euclidean_fitness_eps,
                relative_rmse=params.icp_transformation_eps
            )
        )
        
        print(f"Final ICP fitness: {icp_result.fitness:.4f}")
        return icp_result.transformation, icp_result.fitness
    
    def compute_fpfh_features(self, 
                            cloud: o3d.geometry.PointCloud, 
                            radius: float = 0.05) -> o3d.pipelines.registration.Feature:
        """
        Compute FPFH features for point cloud
        """
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            cloud, o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=100)
        )
        return fpfh
    
    def colored_icp_registration(self, 
                               source: o3d.geometry.PointCloud,
                               target: o3d.geometry.PointCloud,
                               init_transformation: np.ndarray,
                               params: AlignParameters) -> Tuple[np.ndarray, float]:
        """
        Colored ICP registration for improved accuracy
        Uses both geometry and color information
        """
        print("Performing Colored ICP registration...")
        
        # Ensure clouds have colors
        if not source.has_colors():
            source.paint_uniform_color([0.5, 0.5, 0.5])
        if not target.has_colors():
            target.paint_uniform_color([0.5, 0.5, 0.5])
        
        # Estimate normals
        source_with_normals = self.estimate_normals(source, radius=0.01, max_nn=20)
        target_with_normals = self.estimate_normals(target, radius=0.01, max_nn=20)
        
        # Colored ICP
        colored_icp_result = o3d.pipelines.registration.registration_colored_icp(
            source_with_normals, target_with_normals,
            max_correspondence_distance=params.icp_corr_threshold,
            init=init_transformation,
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=params.icp_euclidean_fitness_eps,
                relative_rmse=params.icp_transformation_eps,
                max_iteration=params.icp_max_iter_inner
            ),
            estimation_method=o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            lambda_geometric=0.968
        )
        
        print(f"Colored ICP fitness: {colored_icp_result.fitness:.4f}")
        return colored_icp_result.transformation, colored_icp_result.fitness
    
    def adaptive_multi_scale_icp(self, 
                               source: o3d.geometry.PointCloud,
                               target: o3d.geometry.PointCloud,
                               init_transformation: np.ndarray,
                               params: AlignParameters) -> Tuple[np.ndarray, float]:
        """
        Adaptive multi-scale ICP with dynamic parameter adjustment
        Based on latest research on adaptive registration
        """
        print("Performing adaptive multi-scale ICP...")
        
        current_transformation = init_transformation.copy()
        best_fitness = 0.0
        best_transformation = current_transformation.copy()
        
        # Adaptive scales based on cloud size and density
        source_points = len(source.points)
        target_points = len(target.points)
        
        # Calculate adaptive scales
        if source_points > 10000 and target_points > 10000:
            scales = [0.05, 0.02, 0.01, 0.005]  # High density
            correspondence_distances = [5.0, 2.0, 1.0, 0.5]
        elif source_points > 5000 and target_points > 5000:
            scales = [0.03, 0.015, 0.008]  # Medium density
            correspondence_distances = [3.0, 1.5, 0.8]
        else:
            scales = [0.02, 0.01]  # Low density
            correspondence_distances = [2.0, 1.0]
        
        print(f"Adaptive scales: {scales}")
        print(f"Correspondence distances: {correspondence_distances}")
        
        for i, (scale, corr_dist) in enumerate(zip(scales, correspondence_distances)):
            print(f"Scale {i+1}/{len(scales)}: {scale}m, corr_dist={corr_dist}mm")
            
            # Downsample clouds
            source_down = source.voxel_down_sample(scale)
            target_down = target.voxel_down_sample(scale)
            
            if len(source_down.points) < 3 or len(target_down.points) < 3:
                print(f"Skipping scale {i+1}: insufficient points")
                continue
            
            # Estimate normals
            source_down = self.estimate_normals(source_down, radius=scale*2, max_nn=30)
            target_down = self.estimate_normals(target_down, radius=scale*2, max_nn=30)
            
            # Adaptive ICP parameters
            max_iter = min(params.icp_max_iter_inner, 50)  # Limit iterations for efficiency
            fitness_eps = params.icp_euclidean_fitness_eps * (1.0 + i * 0.1)  # Relaxed for early stages
            trans_eps = params.icp_transformation_eps * (1.0 + i * 0.1)
            
            # ICP registration
            result = o3d.pipelines.registration.registration_icp(
                source_down, target_down,
                max_correspondence_distance=corr_dist,
                init=current_transformation,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=max_iter,
                    relative_fitness=fitness_eps,
                    relative_rmse=trans_eps
                )
            )
            
            if result.fitness > best_fitness:
                best_fitness = result.fitness
                best_transformation = result.transformation
                current_transformation = result.transformation
                print(f"Scale {i+1} improved fitness: {result.fitness:.4f}")
            else:
                print(f"Scale {i+1} fitness: {result.fitness:.4f} (no improvement)")
        
        return best_transformation, best_fitness
    
    def extract_major_segment_using_euclidean_clustering_with_normals(self, 
                                                                    cloud_in: o3d.geometry.PointCloud,
                                                                    cloud_out: o3d.geometry.PointCloud,
                                                                    cluster_tolerance: float = 10.0,
                                                                    min_cluster_size: int = 100,
                                                                    max_cluster_size: int = 100000,
                                                                    b_plot: bool = False) -> bool:
        """
        PCL의 extractMajorSegmentUsingEuclideanClusteringWithNormals 구현
        Euclidean clustering을 사용하여 주요 세그먼트를 추출
        """
        print(f"Extracting major segment using Euclidean clustering...")
        print(f"Cluster tolerance: {cluster_tolerance}mm, Min size: {min_cluster_size}, Max size: {max_cluster_size}")
        
        if len(cloud_in.points) == 0:
            print("Input cloud is empty")
            return False
        
        # Convert to numpy for processing
        points = np.asarray(cloud_in.points)
        if len(points) == 0:
            print("No points in cloud")
            return False
        
        # PCL-style Euclidean clustering
        from sklearn.cluster import DBSCAN
        
        # Convert mm to m for clustering (PCL uses mm internally)
        cluster_tolerance_m = cluster_tolerance / 1000.0
        
        # DBSCAN clustering (similar to PCL's EuclideanClusterExtraction)
        clustering = DBSCAN(eps=cluster_tolerance_m, min_samples=min_cluster_size).fit(points)
        labels = clustering.labels_
        
        # Find the largest cluster
        unique_labels = np.unique(labels)
        largest_cluster_size = 0
        largest_cluster_label = -1
        
        for label in unique_labels:
            if label == -1:  # Skip noise points
                continue
            cluster_mask = labels == label
            cluster_size = np.sum(cluster_mask)
            
            if min_cluster_size <= cluster_size <= max_cluster_size:
                if cluster_size > largest_cluster_size:
                    largest_cluster_size = cluster_size
                    largest_cluster_label = label
        
        if largest_cluster_label == -1:
            print("No valid cluster found")
            return False
        
        # Extract the largest cluster
        cluster_mask = labels == largest_cluster_label
        major_segment_points = points[cluster_mask]
        
        print(f"Major segment extracted: {len(major_segment_points)} points")
        
        # Create output cloud
        cloud_out.points = o3d.utility.Vector3dVector(major_segment_points)
        
        # Estimate normals for the major segment
        cloud_out = self.estimate_normals(cloud_out, radius=0.01, max_nn=20)
        
        if b_plot:
            print("Visualizing major segment extraction...")
            o3d.visualization.draw_geometries([cloud_in, cloud_out], 
                                            window_name="Major Segment Extraction",
                                            width=800, height=600)
        
        return True
    
    def euclidean_clustering_with_normals(self, 
                                        cloud_in: o3d.geometry.PointCloud,
                                        cluster_tolerance: float = 0.02,  # 2cm in meters
                                        min_cluster_size: int = 100,
                                        max_cluster_size: int = 25000,
                                        do_plane_removal: bool = False,
                                        b_plot: bool = False) -> List[o3d.geometry.PointCloud]:
        """
        PCL의 segmentationEuclideanClusterExtraction 구현
        Euclidean clustering을 사용하여 점군을 클러스터로 분할
        """
        print(f"Euclidean clustering with normals...")
        print(f"Cluster tolerance: {cluster_tolerance}m, Min size: {min_cluster_size}, Max size: {max_cluster_size}")
        
        if len(cloud_in.points) == 0:
            print("Input cloud is empty")
            return []
        
        # Convert to numpy for processing
        points = np.asarray(cloud_in.points)
        if len(points) == 0:
            print("No points in cloud")
            return []
        
        # Optional plane removal (like PCL)
        if do_plane_removal:
            print("Performing plane removal...")
            # Simple plane removal using RANSAC
            plane_model, inliers = cloud_in.segment_plane(distance_threshold=0.01, 
                                                        ransac_n=3, 
                                                        num_iterations=1000)
            if len(inliers) > 0:
                cloud_in = cloud_in.select_by_index(inliers, invert=True)
                points = np.asarray(cloud_in.points)
                print(f"After plane removal: {len(points)} points")
        
        # DBSCAN clustering (similar to PCL's EuclideanClusterExtraction)
        from sklearn.cluster import DBSCAN
        
        clustering = DBSCAN(eps=cluster_tolerance, min_samples=min_cluster_size).fit(points)
        labels = clustering.labels_
        
        # Extract clusters
        clusters = []
        unique_labels = np.unique(labels)
        
        for label in unique_labels:
            if label == -1:  # Skip noise points
                continue
                
            cluster_mask = labels == label
            cluster_points = points[cluster_mask]
            cluster_size = len(cluster_points)
            
            if min_cluster_size <= cluster_size <= max_cluster_size:
                # Create cluster cloud
                cluster_cloud = o3d.geometry.PointCloud()
                cluster_cloud.points = o3d.utility.Vector3dVector(cluster_points)
                
                # Estimate normals
                cluster_cloud = self.estimate_normals(cluster_cloud, radius=0.01, max_nn=20)
                
                clusters.append(cluster_cloud)
                print(f"Cluster {label}: {cluster_size} points")
        
        print(f"Found {len(clusters)} valid clusters")
        
        if b_plot and len(clusters) > 0:
            print("Visualizing clustering results...")
            # Color each cluster differently
            colored_clusters = []
            colors = np.random.rand(len(clusters), 3)
            
            for i, cluster in enumerate(clusters):
                cluster.paint_uniform_color(colors[i])
                colored_clusters.append(cluster)
            
            o3d.visualization.draw_geometries(colored_clusters, 
                                            window_name="Euclidean Clustering Results",
                                            width=800, height=600)
        
        return clusters
    
    def major_axis_translation(self, 
                             cloud_reg: o3d.geometry.PointCloud,
                             cloud_cad: o3d.geometry.PointCloud,
                             T_init: np.ndarray,
                             b_plot: bool = False) -> np.ndarray:
        """
        PCL의 matchingMajorAxisTranslation 구현
        주요 축을 기준으로 점군을 정렬
        """
        print("Major axis translation...")
        
        if len(cloud_reg.points) == 0 or len(cloud_cad.points) == 0:
            print("Empty point clouds")
            return T_init
        
        # Get centroids
        reg_centroid = np.asarray(cloud_reg.get_center())
        cad_centroid = np.asarray(cloud_cad.get_center())
        
        # Calculate translation
        translation = cad_centroid - reg_centroid
        
        # Create transformation matrix
        T_result = T_init.copy()
        T_result[0:3, 3] += translation
        
        print(f"Translation: {translation}")
        
        if b_plot:
            print("Visualizing major axis translation...")
            # Transform registration cloud
            cloud_reg_transformed = cloud_reg.transform(T_result)
            
            # Color clouds differently
            cloud_reg.paint_uniform_color([1, 0, 0])  # Red
            cloud_cad.paint_uniform_color([0, 1, 0])  # Green
            cloud_reg_transformed.paint_uniform_color([0, 0, 1])  # Blue
            
            o3d.visualization.draw_geometries([cloud_reg, cloud_cad, cloud_reg_transformed], 
                                            window_name="Major Axis Translation",
                                            width=800, height=600)
        
        return T_result
    
    def plane_rotation_using_obb(self, 
                               cloud_reg: o3d.geometry.PointCloud,
                               cloud_cad: o3d.geometry.PointCloud,
                               T_init: np.ndarray,
                               b_plot: bool = False) -> np.ndarray:
        """
        PCL의 matchingPlaneRotationUsingOBB 구현
        OBB (Oriented Bounding Box)를 사용하여 평면 회전 정렬
        """
        print("Plane rotation using OBB...")
        
        if len(cloud_reg.points) == 0 or len(cloud_cad.points) == 0:
            print("Empty point clouds")
            return T_init
        
        # Get OBB for both clouds
        reg_obb = cloud_reg.get_oriented_bounding_box()
        cad_obb = cloud_cad.get_oriented_bounding_box()
        
        # Get rotation matrices from OBB
        reg_rotation = reg_obb.R
        cad_rotation = cad_obb.R
        
        # Calculate rotation difference
        rotation_diff = cad_rotation @ reg_rotation.T
        
        # Create transformation matrix
        T_result = T_init.copy()
        T_result[0:3, 0:3] = rotation_diff @ T_init[0:3, 0:3]
        
        print(f"OBB rotation applied")
        
        if b_plot:
            print("Visualizing OBB rotation...")
            # Transform registration cloud
            cloud_reg_transformed = cloud_reg.transform(T_result)
            
            # Color clouds differently
            cloud_reg.paint_uniform_color([1, 0, 0])  # Red
            cloud_cad.paint_uniform_color([0, 1, 0])  # Green
            cloud_reg_transformed.paint_uniform_color([0, 0, 1])  # Blue
            
            o3d.visualization.draw_geometries([cloud_reg, cloud_cad, cloud_reg_transformed], 
                                            window_name="OBB Rotation",
                                            width=800, height=600)
        
        return T_result
    
    def evaluate_registration(self, source: o3d.geometry.PointCloud,
                             target: o3d.geometry.PointCloud,
                             transformation: np.ndarray,
                             max_correspondence_distance: float = 0.01) -> Tuple[float, float]:
        """
        Evaluate registration quality.
        
        Args:
            source: Source point cloud
            target: Target point cloud
            transformation: Transformation to evaluate
            max_correspondence_distance: Distance threshold for correspondences
            
        Returns:
            Tuple of (fitness, inlier_rmse)
        """
        evaluation = o3d.pipelines.registration.evaluate_registration(
            source, target, max_correspondence_distance, transformation
        )
        
        return evaluation.fitness, evaluation.inlier_rmse
    
    def create_voxel_grid(self, pcd: o3d.geometry.PointCloud,
                         voxel_size: float = 0.001) -> o3d.geometry.VoxelGrid:
        """
        Create voxel grid from point cloud.
        Used for collision checking (approximation of OctoMap).
        
        Args:
            pcd: Input point cloud
            voxel_size: Voxel size (e.g., 0.001 for 1mm)
            
        Returns:
            Voxel grid
        """
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)
        
        print(f"Created voxel grid with {len(voxel_grid.get_voxels())} voxels")
        
        return voxel_grid
    
    def check_collision_with_voxel_grid(self, pcd: o3d.geometry.PointCloud,
                                       voxel_grid: o3d.geometry.VoxelGrid,
                                       threshold: int = 10) -> Tuple[bool, int]:
        """
        Check collision using voxel grid occupancy.
        
        Args:
            pcd: Point cloud to check (e.g., gripper)
            voxel_grid: Environment voxel grid
            threshold: Minimum number of occupied voxels to consider collision
            
        Returns:
            Tuple of (is_collision, collision_count)
        """
        points = np.asarray(pcd.points)
        
        collision_count = 0
        voxels = voxel_grid.get_voxels()
        
        # Check each point
        for point in points:
            # Check if point is inside any voxel
            voxel_idx = voxel_grid.get_voxel(point)
            
            # Simple occupancy check (could be improved)
            if voxel_idx is not None:
                collision_count += 1
        
        is_collision = collision_count >= threshold
        
        return is_collision, collision_count
    
    def extract_center_with_normal(self, pcd: o3d.geometry.PointCloud) -> Tuple[np.ndarray, np.ndarray]:
        """
        Extract centroid and its normal.
        
        Args:
            pcd: Input point cloud (must have normals)
            
        Returns:
            Tuple of (centroid, normal at centroid)
        """
        points = np.asarray(pcd.points)
        
        if len(points) == 0:
            return np.zeros(3), np.zeros(3)
        
        # Compute centroid
        centroid = np.mean(points, axis=0)
        
        # Find nearest point to centroid
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        [k, idx, _] = pcd_tree.search_knn_vector_3d(centroid, 1)
        
        nearest_idx = idx[0]
        
        # Get normal at nearest point
        if pcd.has_normals():
            normals = np.asarray(pcd.normals)
            normal = normals[nearest_idx]
        else:
            normal = np.zeros(3)
        
        return centroid, normal

