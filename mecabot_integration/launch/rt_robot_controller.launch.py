#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecabot_integration',
            executable='rt_robot_controller',
            name='robot_controller',
            output='screen'
        )
    ])

