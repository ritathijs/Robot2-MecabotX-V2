import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file = '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/param/wheeltec_params/param_flagship_mec_dl.yaml'
    mask_yaml_file = '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC_KEEPOUT_SERVICE.yaml'

    return LaunchDescription([
        # Costmap Filter Info Server
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            namespace='',
            output='screen',
            parameters=[params_file]),

        # Map Server (voor het masker)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_map_server',
            namespace='',
            output='screen',
            parameters=[{'yaml_filename': mask_yaml_file}],
            remappings=[('/map', '/keepout_filter_mask')]),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['costmap_filter_info_server', 'keepout_map_server']}]),
        
        # Keepout_filter_throttle node   
	Node(
            package='keepout_filter_throttle',
            executable='keepout_filter_throttle',
            name='keepout_filter_throttle',
            output='screen')
                        
    ])
    
 #KULeuven2025-26
