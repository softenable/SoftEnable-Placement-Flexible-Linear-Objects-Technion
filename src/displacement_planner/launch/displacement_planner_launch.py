#!/usr/bin/env python3
"""Launch file for displacement planner service."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate the launch description for displacement planner."""
    
    # Get package directory
    package_dir = get_package_share_directory('displacement_planner')
    
    # Configuration file path
    config_file = os.path.join(package_dir, 'config', 'displacement_planner.yaml')
    
    # Displacement planner node
    displacement_planner_node = Node(
        package='displacement_planner',
        executable='planner',
        name='displacement_planner',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        displacement_planner_node
    ])
