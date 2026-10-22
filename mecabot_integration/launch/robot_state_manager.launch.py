#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'mecabot_integration'
    pkg_share = get_package_share_directory(package_name)
    script_path = os.path.join(pkg_share, '../../lib', package_name, 'robot_state_manager.py')
    locations_config = os.path.join(
        pkg_share, 'config', 'robot_locations.yaml'
    )

    return LaunchDescription([
        Node(
            package='april_tabloo',
            executable='april_tabloo',
            name='april_tabloo',
            parameters=[locations_config],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'python3', script_path,
                '--ros-args', '--params-file', locations_config,
            ],
            output='screen'
        )
    ])
