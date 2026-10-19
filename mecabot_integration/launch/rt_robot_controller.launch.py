#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'mecabot_integration'
    package_share = get_package_share_directory(package_name)
    locations_config = os.path.join(
        package_share, 'config', 'robot_locations.yaml'
    )

    return LaunchDescription([
        Node(
            package=package_name,
            executable='rt_robot_controller',
            name='robot_controller',
            parameters=[locations_config],
            output='screen'
        )
    ])
