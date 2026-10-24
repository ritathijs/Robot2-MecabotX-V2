#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
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

    use_bridge = LaunchConfiguration('use_bridge')
    start_docked = LaunchConfiguration('start_docked')
    speed = LaunchConfiguration('speed')
    auto_activate = LaunchConfiguration('auto_activate_delay_sec')
    scenario = LaunchConfiguration('scenario')
    visitor_at = LaunchConfiguration('visitor_at_sec')
    low_battery_at = LaunchConfiguration('low_battery_at_sec')
    reactivate = LaunchConfiguration('reactivate_after_charge_sec')

    # De simulatie vervangt Nav2, AMCL, laadpaal, bumper en sensoren.
    # Quiz-bridge, camera, mux en AprilTag-node worden niet gestart.
    return LaunchDescription([
        DeclareLaunchArgument('use_bridge', default_value='false'),
        DeclareLaunchArgument('start_docked', default_value='true'),
        DeclareLaunchArgument('speed', default_value='1.0'),
        DeclareLaunchArgument('auto_activate_delay_sec', default_value='0.0'),
        # scenario:=day speelt bezoeker, lage batterij en terug laden af.
        DeclareLaunchArgument('scenario', default_value='none'),
        DeclareLaunchArgument('visitor_at_sec', default_value='60.0'),
        DeclareLaunchArgument('low_battery_at_sec', default_value='150.0'),
        DeclareLaunchArgument('reactivate_after_charge_sec', default_value='0.0'),
        ExecuteProcess(
            cmd=[
                'python3', script('sim_robot.py'),
                '--ros-args',
                '-p', ['start_docked:=', start_docked],
                '-p', ['speed:=', speed],
                '-p', ['auto_activate_delay_sec:=', auto_activate],
                '-p', ['scenario:=', scenario],
                '-p', ['visitor_at_sec:=', visitor_at],
                '-p', ['low_battery_at_sec:=', low_battery_at],
                '-p', ['reactivate_after_charge_sec:=', reactivate],
            ],
            output='screen',
        ),
        # Zonder bridge draait alleen de statusmachine; met use_bridge:=true
        # start de bridge de statusmachine en verbindt met de Pi (QUIZ_SERVER_URL).
        ExecuteProcess(
            condition=UnlessCondition(use_bridge),
            cmd=[
                'python3', script('robot_state_manager.py'),
                '--ros-args', '--params-file', locations_config,
            ],
            output='screen',
        ),
        ExecuteProcess(
            condition=IfCondition(use_bridge),
            cmd=[
                'python3', script('quiz_socket_bridge.py'),
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
        # De BT publiceert node-wissels eenmalig op een volatile topic; start
        # hem pas als de overige nodes klaar zijn om niets te missen.
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package=package_name,
                    executable='rt_bt_node',
                    name='behavior_tree_node',
                    output='screen',
                ),
            ],
        ),
        ExecuteProcess(
            cmd=['python3', script('rt_drive_to_coord.py')],
            output='screen',
        ),
    ])
