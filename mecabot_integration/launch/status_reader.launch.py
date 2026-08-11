#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    package_name = 'mecabot_integration'
    pkg_share = get_package_share_directory(package_name)
    script_path = os.path.join(pkg_share, '../../lib', package_name, 'status_reader.py')

    current_python_path = os.environ.get('PYTHONPATH', '')
    workspace_python_path = "/home/wheeltec/wheeltec_ros2/install/wheeltec_robot_msg/local/lib/python3.10/dist-packages:" + current_python_path

    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', script_path],
            output='screen',
            additional_env={
                'PYTHONPATH': workspace_python_path
            }
        )
    ])

