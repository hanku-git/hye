#!/usr/bin/env python3
"""
Full pipeline launch file for Open3D matching system.
Launches matching node with complete Zivid → SAM → Matching → Robot workflow.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for full pipeline"""
    
    # Declare launch arguments
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug mode with verbose logging'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    num_threads_arg = DeclareLaunchArgument(
        'num_threads',
        default_value='4',
        description='Number of threads for parallel mask processing'
    )
    
    matching_method_arg = DeclareLaunchArgument(
        'matching_method',
        default_value='1',
        description='Matching method: 1=ICP, 2=GICP'
    )
    
    matching_accuracy_limit_arg = DeclareLaunchArgument(
        'matching_accuracy_limit',
        default_value='50.0',
        description='Minimum matching accuracy threshold (%)'
    )
    
    # Get package share directory
    package_share = FindPackageShare('hanyang_matching_open3d')
    
    # Config file path
    config_file = PathJoinSubstitution([
        package_share,
        'config',
        'matching_params.yaml'
    ])
    
    # Matching node (full pipeline)
    matching_node = Node(
        package='hanyang_matching_open3d',
        executable='matching_node_full.py',
        name='hanyang_matching_open3d_node',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'debug_mode': LaunchConfiguration('debug_mode'),
                'num_threads': LaunchConfiguration('num_threads'),
                'matching_method': LaunchConfiguration('matching_method'),
                'matching_accuracy_limit': LaunchConfiguration('matching_accuracy_limit'),
            }
        ],
        remappings=[
            # Remap output topics to distinguish Open3D results
            ('/cad_matching_result', '/open3d/cad_matching_result'),
            ('/scanning_result', '/open3d/scanning_result'),
            ('/cloud_matching_results', '/open3d/cloud_matching_results'),
            ('/cloud_pre_matching_results', '/open3d/cloud_pre_matching_results'),
            ('/cloud_segmentation_results', '/open3d/cloud_segmentation_results'),
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        debug_mode_arg,
        use_sim_time_arg,
        num_threads_arg,
        matching_method_arg,
        matching_accuracy_limit_arg,
        LogInfo(msg=['Launching Open3D Matching Node (Full Pipeline)...']),
        matching_node,
        LogInfo(msg=['Node launched successfully'])
    ])


