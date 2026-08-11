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
    
    # Genereer de absolute paden naar de 3 Python scripts in de installatiemap
    bridge_script = os.path.join(pkg_share, '../../lib', package_name, 'quiz_socket_bridge.py')
    manager_script = os.path.join(pkg_share, '../../lib', package_name, 'robot_state_manager.py')
    reader_script = os.path.join(pkg_share, '../../lib', package_name, 'status_reader.py')

    # Haal het actieve terminal-zoekpad op en voeg de map met custom services toe
    current_python_path = os.environ.get('PYTHONPATH', '')
    workspace_python_path = f"/home/wheeltec/wheeltec_ros2/install/wheeltec_robot_msg/local/lib/python3.10/dist-packages:{current_python_path}"

    # =========================================================================
    #  1. C++ NODES (Gebruiken de standaard 'Node' actie)
    # =========================================================================
    
    # A. Robot Controller (C++ Switch)
    robot_controller_node = Node(
        package=package_name,
        executable='rt_robot_controller',
        name='robot_controller',
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

    # C. Quiz Socket Bridge (Gecorrigeerd met Pi IP en PYTHONPATH)
    quiz_socket_bridge_node = ExecuteProcess(
        cmd=['python3', bridge_script],
        output='screen',
        additional_env={
            'QUIZ_SERVER_URL': 'http://10.0.0.11:80', # <--- VERANDER DIT naar het echte IP van de Pi!
            'PYTHONPATH': workspace_python_path
        }
    )

    # D. Robot State Manager (Gecorrigeerd met PYTHONPATH)
    robot_state_manager_node = ExecuteProcess(
        cmd=['python3', manager_script],
        output='screen',
        additional_env={
            'PYTHONPATH': workspace_python_path
        }
    )

    # E. Status Reader (Gecorrigeerd met PYTHONPATH)
    status_reader_node = ExecuteProcess(
        cmd=['python3', reader_script],
        output='screen',
        additional_env={
            'PYTHONPATH': workspace_python_path
        }
    )

    # Voeg alle 5 de nodes samen in de ROS 2 Launch Description
    return LaunchDescription([
        robot_controller_node,
        behavior_tree_node,
        quiz_socket_bridge_node,
        robot_state_manager_node,
        status_reader_node
    ])

