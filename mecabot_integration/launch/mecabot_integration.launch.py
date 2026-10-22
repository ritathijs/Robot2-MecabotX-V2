#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    
    # Centrale package naam van jouw sandbox
    package_name = 'mecabot_integration'
    
    # Haal de dynamische share map op (werkt op elke robot via Git)
    pkg_share = get_package_share_directory(package_name)
    locations_config = os.path.join(
        pkg_share, 'config', 'robot_locations.yaml'
    )
    
    # Genereer de absolute paden naar de Python scripts in de installatiemap
    bridge_script = os.path.join(pkg_share, '../../lib', package_name, 'quiz_socket_bridge.py')
    rt_drive_to_coord_script= os.path.join(pkg_share, '../../lib', package_name, 'rt_drive_to_coord.py')

    # =========================================================================
    #  1. C++ NODES (Gebruiken de standaard 'Node' actie)
    # =========================================================================
    
    # A. Robot Controller (C++ Switch)
    robot_controller_node = Node(
        package=package_name,
        executable='rt_robot_controller',
        name='robot_controller',
        parameters=[locations_config],
        output='screen'
    )

    # B. Behavior Tree Node (C++ Brein)
    behavior_tree_node = Node(
        package=package_name,
        executable='rt_bt_node',
        name='behavior_tree_node',
        output='screen'
    )

    # =========================================================================
    #  2. PYTHON NODES (Gebruiken 'ExecuteProcess' om libexec-fouten te voorkomen)
    # =========================================================================

    # C. Quiz Socket Bridge
    quiz_socket_bridge_node = ExecuteProcess(
        cmd=[
            'python3', bridge_script,
            '--ros-args', '--params-file', locations_config,
        ],
        output='screen',
        additional_env={
            'QUIZ_SERVER_URL': 'http://10.0.0.11:80'
        }
    )

    # D. AprilTag-poseherstel. De state manager start de reset zodra de
    # camera- en batterijstatus beschikbaar zijn.
    apriltag_node = Node(
        package='april_tabloo',
        executable='april_tabloo',
        name='april_tabloo',
        parameters=[locations_config],
        output='screen'
    )

    # E. Drive_to_coord
    rt_drive_to_coord_node = ExecuteProcess(
        cmd=['python3', rt_drive_to_coord_script],
        output='screen'
    )

    # QuizSocketBridge start zelf de RobotStateManager.
    return LaunchDescription([
        robot_controller_node,
        behavior_tree_node,
        quiz_socket_bridge_node,
        apriltag_node,
        rt_drive_to_coord_node
    ])
