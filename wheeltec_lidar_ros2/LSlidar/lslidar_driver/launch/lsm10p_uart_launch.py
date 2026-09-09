#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os

def generate_launch_description():

    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lidar_uart_ros2','lsm10_p.yaml')
                     
    driver_node = LifecycleNode(package='lslidar_driver',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',		
                                output='screen',
                                emulate_tty=True,
                                namespace='',
                                parameters=[driver_dir],
                                respawn=True,      #RT toegevoegd op 7/9/2026 wegens verdwijnen van de lslidar_driver_node en robot is verdwaald als gevolg
                                respawn_delay=2.0,
                                )
    return LaunchDescription([
        driver_node,
    ])

