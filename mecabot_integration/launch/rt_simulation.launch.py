#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'mecabot_integration'
    pkg_share = get_package_share_directory(package_name)
    lib_dir = os.path.join(pkg_share, '../../lib', package_name)
    locations_config = os.path.join(
        pkg_share, 'config', 'robot_locations.yaml'
    )

    def script(name):
        return os.path.join(lib_dir, name)

    start_docked = LaunchConfiguration('start_docked')
    speed = LaunchConfiguration('speed')
    auto_activate = LaunchConfiguration('auto_activate_delay_sec')

    # De simulatie vervangt Nav2, AMCL, laadpaal, bumper en sensoren.
    # Quiz-bridge, camera, mux en AprilTag-node worden niet gestart.
    return LaunchDescription([
        DeclareLaunchArgument('start_docked', default_value='true'),
        DeclareLaunchArgument('speed', default_value='1.0'),
        DeclareLaunchArgument('auto_activate_delay_sec', default_value='0.0'),
        ExecuteProcess(
            cmd=[
                'python3', script('sim_robot.py'),
                '--ros-args',
                '-p', ['start_docked:=', start_docked],
                '-p', ['speed:=', speed],
                '-p', ['auto_activate_delay_sec:=', auto_activate],
            ],
            output='screen',
        ),
        ExecuteProcess(
            cmd=[
                'python3', script('robot_state_manager.py'),
                '--ros-args', '--params-file', locations_config,
            ],
            output='screen',
        ),
        Node(
            package=package_name,
            executable='rt_robot_controller',
            name='robot_controller',
            parameters=[locations_config],
            output='screen',
        ),
        Node(
            package=package_name,
            executable='rt_bt_node',
            name='behavior_tree_node',
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3', script('rt_drive_to_coord.py')],
            output='screen',
        ),
    ])
