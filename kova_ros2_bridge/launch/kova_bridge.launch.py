#!/usr/bin/env python3
"""
Launch file for Kova ROS2 Bridge
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('kova_ros2_bridge')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'kova_bridge.yaml'),
        description='Path to configuration file'
    )
    
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='robot_001',
        description='Robot ID for Kova network'
    )
    
    validation_threshold_arg = DeclareLaunchArgument(
        'validation_threshold',
        default_value='0.7',
        description='Data validation threshold'
    )
    
    auto_claim_rewards_arg = DeclareLaunchArgument(
        'auto_claim_rewards',
        default_value='true',
        description='Enable automatic reward claiming'
    )
    
    # Get launch configuration
    config_file = LaunchConfiguration('config_file')
    robot_id = LaunchConfiguration('robot_id')
    validation_threshold = LaunchConfiguration('validation_threshold')
    auto_claim_rewards = LaunchConfiguration('auto_claim_rewards')
    
    # Kova Bridge Node
    kova_bridge_node = Node(
        package='kova_ros2_bridge',
        executable='kova_bridge_node',
        name='kova_bridge',
        parameters=[config_file, {
            'robot_id': robot_id,
            'validation_threshold': validation_threshold,
            'auto_claim_rewards': auto_claim_rewards
        }],
        output='screen'
    )
    
    # Sensor Processor Node
    sensor_processor_node = Node(
        package='kova_ros2_bridge',
        executable='sensor_processor_node',
        name='sensor_processor',
        parameters=[config_file],
        output='screen'
    )
    
    # Reward Claimer Node
    reward_claimer_node = Node(
        package='kova_ros2_bridge',
        executable='reward_claimer_node',
        name='reward_claimer',
        parameters=[config_file, {
            'auto_claim': auto_claim_rewards
        }],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        robot_id_arg,
        validation_threshold_arg,
        auto_claim_rewards_arg,
        kova_bridge_node,
        sensor_processor_node,
        reward_claimer_node
    ])
