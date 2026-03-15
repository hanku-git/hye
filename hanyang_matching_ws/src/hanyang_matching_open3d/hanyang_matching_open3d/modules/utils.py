"""
Utility functions for coordinate transformations and conversions.
"""

import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from typing import Tuple, List


def ros_to_open3d(ros_cloud: PointCloud2) -> o3d.geometry.PointCloud:
    """
    Convert ROS PointCloud2 message to Open3D PointCloud.
    
    Args:
        ros_cloud: ROS PointCloud2 message
        
    Returns:
        Open3D PointCloud
    """
    # Get point cloud data
    points = []
    colors = []
    
    for point in pc2.read_points(ros_cloud, skip_nans=True):
        points.append([point[0], point[1], point[2]])
        
        # If RGB/RGBA data exists
        if len(point) > 3:
            # Assuming RGB or RGBA
            if len(point) >= 6:  # x, y, z, r, g, b
                r, g, b = point[3], point[4], point[5]
                colors.append([r/255.0, g/255.0, b/255.0])
    
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    
    if len(colors) > 0:
        pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
    
    return pcd


def open3d_to_ros(pcd: o3d.geometry.PointCloud, frame_id: str = "base_link") -> PointCloud2:
    """
    Convert Open3D PointCloud to ROS PointCloud2 message.
    
    Args:
        pcd: Open3D PointCloud
        frame_id: Frame ID for the ROS message
        
    Returns:
        ROS PointCloud2 message
    """
    import rclpy
    from builtin_interfaces.msg import Time
    
    points = np.asarray(pcd.points)
    
    # Check if colors exist
    has_colors = pcd.has_colors()
    
    if has_colors:
        colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)
        
        # Create structured array with XYZ and RGB
        dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32),
                 ('r', np.uint8), ('g', np.uint8), ('b', np.uint8)]
        
        cloud_data = np.zeros(len(points), dtype=dtype)
        cloud_data['x'] = points[:, 0]
        cloud_data['y'] = points[:, 1]
        cloud_data['z'] = points[:, 2]
        cloud_data['r'] = colors[:, 0]
        cloud_data['g'] = colors[:, 1]
        cloud_data['b'] = colors[:, 2]
        
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='r', offset=12, datatype=pc2.PointField.UINT8, count=1),
            pc2.PointField(name='g', offset=13, datatype=pc2.PointField.UINT8, count=1),
            pc2.PointField(name='b', offset=14, datatype=pc2.PointField.UINT8, count=1),
        ]
    else:
        # Only XYZ
        dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
        
        cloud_data = np.zeros(len(points), dtype=dtype)
        cloud_data['x'] = points[:, 0]
        cloud_data['y'] = points[:, 1]
        cloud_data['z'] = points[:, 2]
        
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
        ]
    
    # Create PointCloud2 message
    header = ros_cloud.header if hasattr(ros_cloud, 'header') else None
    if header is None:
        from std_msgs.msg import Header
        header = Header()
        header.frame_id = frame_id
    
    ros_msg = pc2.create_cloud(header, fields, cloud_data.tolist())
    
    return ros_msg


def pose_vec_to_matrix(pose: List[float]) -> np.ndarray:
    """
    Convert pose vector [x, y, z, rx, ry, rz] to 4x4 transformation matrix.
    Angles are in degrees.
    
    Args:
        pose: [x, y, z, rx, ry, rz] where angles are in degrees
        
    Returns:
        4x4 transformation matrix
    """
    x, y, z, rx, ry, rz = pose
    
    # Convert degrees to radians
    rx_rad = np.deg2rad(rx)
    ry_rad = np.deg2rad(ry)
    rz_rad = np.deg2rad(rz)
    
    # Create rotation matrix (ZYX Euler angles)
    cos_rx = np.cos(rx_rad)
    sin_rx = np.sin(rx_rad)
    cos_ry = np.cos(ry_rad)
    sin_ry = np.sin(ry_rad)
    cos_rz = np.cos(rz_rad)
    sin_rz = np.sin(rz_rad)
    
    # Rotation matrix
    R = np.array([
        [cos_ry*cos_rz, -cos_ry*sin_rz, sin_ry],
        [cos_rx*sin_rz + sin_rx*sin_ry*cos_rz, cos_rx*cos_rz - sin_rx*sin_ry*sin_rz, -sin_rx*cos_ry],
        [sin_rx*sin_rz - cos_rx*sin_ry*cos_rz, sin_rx*cos_rz + cos_rx*sin_ry*sin_rz, cos_rx*cos_ry]
    ])
    
    # Create 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    
    return T


def matrix_to_pose_vec(T: np.ndarray) -> List[float]:
    """
    Convert 4x4 transformation matrix to pose vector [x, y, z, rx, ry, rz].
    Angles are in degrees.
    
    Args:
        T: 4x4 transformation matrix
        
    Returns:
        [x, y, z, rx, ry, rz] where angles are in degrees
    """
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]
    
    # Extract rotation matrix
    R = T[:3, :3]
    
    # Convert to ZYX Euler angles
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    
    singular = sy < 1e-6
    
    if not singular:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0
    
    # Convert radians to degrees
    rx_deg = np.rad2deg(rx)
    ry_deg = np.rad2deg(ry)
    rz_deg = np.rad2deg(rz)
    
    return [x, y, z, rx_deg, ry_deg, rz_deg]


def remove_nan_points(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    """
    Remove NaN points from point cloud.
    
    Args:
        pcd: Input point cloud
        
    Returns:
        Point cloud without NaN points
    """
    points = np.asarray(pcd.points)
    
    # Find valid points (no NaN)
    valid_mask = ~np.isnan(points).any(axis=1)
    
    # Create new point cloud with valid points only
    pcd_clean = o3d.geometry.PointCloud()
    pcd_clean.points = o3d.utility.Vector3dVector(points[valid_mask])
    
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        pcd_clean.colors = o3d.utility.Vector3dVector(colors[valid_mask])
    
    if pcd.has_normals():
        normals = np.asarray(pcd.normals)
        pcd_clean.normals = o3d.utility.Vector3dVector(normals[valid_mask])
    
    return pcd_clean


def filter_by_view_frustum(pcd: o3d.geometry.PointCloud, 
                          view_frustum: List[float]) -> o3d.geometry.PointCloud:
    """
    Filter point cloud using axis-aligned bounding box (view frustum).
    
    Args:
        pcd: Input point cloud
        view_frustum: [min_x, max_x, min_y, max_y, min_z, max_z]
        
    Returns:
        Filtered point cloud
    """
    if len(view_frustum) != 6:
        raise ValueError("view_frustum must have 6 elements: [min_x, max_x, min_y, max_y, min_z, max_z]")
    
    min_bound = np.array([view_frustum[0], view_frustum[2], view_frustum[4]])
    max_bound = np.array([view_frustum[1], view_frustum[3], view_frustum[5]])
    
    # Create axis-aligned bounding box
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
    
    # Crop point cloud
    pcd_filtered = pcd.crop(bbox)
    
    return pcd_filtered


def cloud_scaling(pcd: o3d.geometry.PointCloud, scale_mode: int) -> o3d.geometry.PointCloud:
    """
    Scale point cloud coordinates.
    
    Args:
        pcd: Input point cloud
        scale_mode: 0 = mm to m (divide by 1000), 1 = m to mm (multiply by 1000)
        
    Returns:
        Scaled point cloud
    """
    points = np.asarray(pcd.points)
    
    if scale_mode == 0:
        # mm to m
        points = points / 1000.0
    elif scale_mode == 1:
        # m to mm
        points = points * 1000.0
    else:
        raise ValueError("scale_mode must be 0 (mm to m) or 1 (m to mm)")
    
    pcd_scaled = o3d.geometry.PointCloud()
    pcd_scaled.points = o3d.utility.Vector3dVector(points)
    
    if pcd.has_colors():
        pcd_scaled.colors = pcd.colors
    
    if pcd.has_normals():
        pcd_scaled.normals = pcd.normals
    
    return pcd_scaled


def compute_transform_from_dh_and_js(DH: np.ndarray, q: np.ndarray, 
                                     tcp: np.ndarray) -> np.ndarray:
    """
    Compute forward kinematics transformation from DH parameters and joint states.
    
    Args:
        DH: DH parameters (6x4): [theta_offset, d, a, alpha] for each joint
        q: Joint angles (6,) in radians
        tcp: TCP offset [x, y, z, rx, ry, rz] in [m] and [deg]
        
    Returns:
        4x4 transformation matrix from base to end-effector
    """
    T = np.eye(4)
    
    for i in range(6):
        theta = q[i] + DH[i, 0]
        d = DH[i, 1]
        a = DH[i, 2]
        alpha = DH[i, 3]
        
        # DH transformation
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        Ti = np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        
        T = T @ Ti
    
    # Apply TCP offset
    T_tcp = pose_vec_to_matrix(tcp)
    T = T @ T_tcp
    
    return T


def estimate_normals_with_sensor_origin(pcd: o3d.geometry.PointCloud,
                                       sensor_origin: np.ndarray = np.array([0, 0, 0]),
                                       radius: float = 0.01,
                                       max_nn: int = 30) -> o3d.geometry.PointCloud:
    """
    Estimate normals and orient them towards sensor origin.
    
    Args:
        pcd: Input point cloud
        sensor_origin: Sensor position [x, y, z]
        radius: Search radius for normal estimation
        max_nn: Maximum number of neighbors
        
    Returns:
        Point cloud with oriented normals
    """
    # Estimate normals
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
    )
    
    # Orient normals towards sensor origin
    pcd.orient_normals_towards_camera_location(camera_location=sensor_origin)
    
    return pcd


def evaluate_matching_accuracy(cloud_source: o3d.geometry.PointCloud,
                               cloud_target: o3d.geometry.PointCloud,
                               threshold: float = 0.005) -> float:
    """
    Evaluate matching accuracy using PCL-style OctoMap-based approach.
    
    This function mimics PCL's evaluateMatchingResults which uses OctoMap
    to check if matched points are within occupied voxels of the CAD model.
    
    Args:
        cloud_source: Source point cloud (matched CAD)
        cloud_target: Target point cloud (reference CAD model)
        threshold: Voxel size threshold (default 0.6mm like PCL)
        
    Returns:
        Matching accuracy (percentage of points in occupied voxels)
    """
    if len(cloud_source.points) == 0:
        return 0.0
    
    # Convert to numpy arrays
    points_source = np.asarray(cloud_source.points)
    points_target = np.asarray(cloud_target.points)
    
    if len(points_target) == 0:
        return 0.0
    
    # Create voxel grid from target (CAD model)
    # This simulates PCL's OctoMap approach
    voxel_size = threshold  # Use threshold as voxel size
    
    # Create voxel grid
    target_min = points_target.min(axis=0)
    target_max = points_target.max(axis=0)
    
    # Calculate voxel grid dimensions
    grid_size = np.ceil((target_max - target_min) / voxel_size).astype(int)
    
    # Create occupancy grid
    occupied_voxels = set()
    
    # Mark occupied voxels from target points
    for point in points_target:
        voxel_coord = tuple(np.floor((point - target_min) / voxel_size).astype(int))
        occupied_voxels.add(voxel_coord)
    
    # Count source points that fall in occupied voxels
    occupied_count = 0
    for point in points_source:
        voxel_coord = tuple(np.floor((point - target_min) / voxel_size).astype(int))
        if voxel_coord in occupied_voxels:
            occupied_count += 1
    
    # Calculate accuracy as percentage
    accuracy = (occupied_count / len(points_source)) * 100.0
    
    return accuracy


def evaluate_matching_accuracy_improved(cloud_source: o3d.geometry.PointCloud,
                                        cloud_target: o3d.geometry.PointCloud,
                                        voxel_size: float = 0.0006) -> float:
    """
    Improved matching accuracy evaluation using PCL-style approach.
    
    This function more closely mimics PCL's evaluateMatchingResults by:
    1. Using proper voxel size (0.6mm = 0.0006m)
    2. Checking point occupancy in voxel grid
    3. Using the same evaluation logic as PCL
    
    Args:
        cloud_source: Source point cloud (matched CAD)
        cloud_target: Target point cloud (reference CAD model)
        voxel_size: Voxel size in meters (default 0.6mm)
        
    Returns:
        Matching accuracy (percentage of points in occupied voxels)
    """
    if len(cloud_source.points) == 0:
        return 0.0
    
    # Convert to numpy arrays
    points_source = np.asarray(cloud_source.points)
    points_target = np.asarray(cloud_target.points)
    
    if len(points_target) == 0:
        return 0.0
    
    # Create voxel grid from target (CAD model)
    target_min = points_target.min(axis=0)
    target_max = points_target.max(axis=0)
    
    # Calculate voxel grid dimensions
    grid_size = np.ceil((target_max - target_min) / voxel_size).astype(int)
    
    # Create occupancy grid using dictionary for efficiency
    occupied_voxels = {}
    
    # Mark occupied voxels from target points
    for point in points_target:
        voxel_coord = tuple(np.floor((point - target_min) / voxel_size).astype(int))
        occupied_voxels[voxel_coord] = True
    
    # Count source points that fall in occupied voxels
    occupied_count = 0
    for point in points_source:
        voxel_coord = tuple(np.floor((point - target_min) / voxel_size).astype(int))
        if voxel_coord in occupied_voxels:
            occupied_count += 1
    
    # Calculate accuracy as percentage (same as PCL)
    accuracy = (occupied_count / len(points_source)) * 100.0
    
    return accuracy


def paint_uniform_color(pcd: o3d.geometry.PointCloud, 
                        color: List[float]) -> o3d.geometry.PointCloud:
    """
    Paint point cloud with uniform color.
    
    Args:
        pcd: Input point cloud
        color: RGB color [r, g, b] in range [0, 1]
        
    Returns:
        Colored point cloud
    """
    pcd.paint_uniform_color(color)
    return pcd


def merge_point_clouds(clouds: List[o3d.geometry.PointCloud]) -> o3d.geometry.PointCloud:
    """
    Merge multiple point clouds into one.
    
    Args:
        clouds: List of point clouds
        
    Returns:
        Merged point cloud
    """
    if len(clouds) == 0:
        return o3d.geometry.PointCloud()
    
    merged = o3d.geometry.PointCloud()
    
    for cloud in clouds:
        merged += cloud
    
    return merged


def compute_centroid(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    """
    Compute centroid of point cloud.
    
    Args:
        pcd: Input point cloud
        
    Returns:
        Centroid [x, y, z]
    """
    points = np.asarray(pcd.points)
    
    if len(points) == 0:
        return np.zeros(3)
    
    centroid = np.mean(points, axis=0)
    
    return centroid

