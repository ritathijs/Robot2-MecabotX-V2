#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    package_name = 'mecabot_integration'
    pkg_share = get_package_share_directory(package_name)
    script_path = os.path.join(pkg_share, '../../lib', package_name, 'quiz_socket_bridge.py') # Of je andere scriptnaam
    locations_config = os.path.join(
        pkg_share, 'config', 'robot_locations.yaml'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'python3', script_path,
                '--ros-args', '--params-file', locations_config,
            ],
            output='screen',
            additional_env={
                'QUIZ_SERVER_URL': 'http://10.0.0.11:80'
            }
        )
    ])
