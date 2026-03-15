#!/usr/bin/env python3
"""
Launch file for Open3D matching node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description"""
    
    # Declare launch arguments
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug mode'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Get package share directory
    package_share = FindPackageShare('hanyang_matching_open3d')
    
    # Config file path
    config_file = PathJoinSubstitution([
        package_share,
        'config',
        'matching_params.yaml'
    ])
    
    # Matching node
    matching_node = Node(
        package='hanyang_matching_open3d',
        executable='matching_node',
        name='hanyang_matching_open3d_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'debug_mode': LaunchConfiguration('debug_mode')}
        ],
        remappings=[
            # Remap output topics to distinguish Open3D results
            ('/cad_matching_result', '/open3d/cad_matching_result'),
            ('/scanning_result', '/open3d/scanning_result'),
            ('/cloud_matching_results', '/open3d/cloud_matching_results'),
            ('/cloud_pre_matching_results', '/open3d/cloud_pre_matching_results'),
            ('/cloud_segmentation_results', '/open3d/cloud_segmentation_results'),
        ]
    )
    
    return LaunchDescription([
        debug_mode_arg,
        use_sim_time_arg,
        matching_node
    ])

