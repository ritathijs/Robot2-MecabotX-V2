#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # 1 - Navigatie
    wheeltec_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wheeltec_nav2'),
                'launch',
                'wheeltec_nav2.launch.py'
            ])
        )
    )

    # 2 - Quiz_bt_node (socket naar roscommando's)
    quiz_bt_node = Node(
        package='quiz_bt_node',    
        executable='quiz_bt_node',
        name='quiz_bt_node',
        output='screen'
    )

    # 3 - Drive to coord
    drive_to_coord_node = Node(
        package='drive_to_coord',
        executable='drive_to_coord',
        output='screen'
    )

    # 4 - Bumper
    bumper_node = Node(
        package='bumper',
        executable='bumper',
        output='screen'
    )

    # 5 - Camera
    wheeltec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turn_on_wheeltec_robot'),
                'launch',
                'wheeltec_camera.launch.py'
            ])
        )
    )

    # 6 - Pointcloud to laserscan (zodat cameraview wordt opgenomen door nav)
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        output='screen',
        remappings=[
            ('cloud_in', '/camera/depth/points'),
            ('scan', '/camera_scan'),
        ],
        parameters=[{
            'target_frame': 'camera_link',
            'min_height': 0.1,
            'max_height': 1.5,
            'angle_min': -1.5708,
            'angle_max': 1.5708,
            'range_min': 0.5,
            'range_max': 5.0,
            'use_inf': True,
        }]
    )

    # 7 - Keepoutfilter (no go zones)
    keepout_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wheeltec_nav2'),
                'launch',
                'keepout_filter.launch.py'
            ])
        )
    )

    # 8 - Autocharge
    auto_recharge_ros2_node = Node(
        package='auto_recharge_ros2',
        executable='auto_recharge',
        output='screen'
    )

    # 9 - Visual tracker (people follower)
    visualtracker_node = Node(
        package='people_follower_ros2',
        executable='visualtracker',
        output='screen'
    )

    # 10 - Behavior Tree
    bt_node = Node(
        package='mecabot_bt',
        executable='bt_node',
        output='screen'
    )
    
    
    delay_1_nav2 = 0.0
    delay_2_quiz_bt = 5.0
    delay_3_drive_to_coord = 10.0
    delay_4_bumper = 13.0
    delay_5_camera = 14.0
    delay_6_pointcloud_to_laserscan = 16.0
    delay_7_keepout_filter = 18.0
    delay_8_auto_recharge = 20.0
    delay_9_visualtracker = 21.0
    delay_10_bt_node = 25.0
    
    # ==========================================
    # LAUNCH DESCRIPTION RETURN
    # ==========================================

    return LaunchDescription([
        TimerAction(period=delay_1_nav2, actions=[wheeltec_nav2_launch]),
        TimerAction(period=delay_2_quiz_bt, actions=[quiz_bt_node]),
        TimerAction(period=delay_3_drive_to_coord, actions=[drive_to_coord_node]),
        TimerAction(period=delay_4_bumper, actions=[bumper_node]),
        TimerAction(period=delay_5_camera, actions=[wheeltec_camera_launch]),
        TimerAction(period=delay_6_pointcloud_to_laserscan, actions=[pointcloud_to_laserscan_node]),
        TimerAction(period=delay_7_keepout_filter, actions=[keepout_filter_launch]),
        TimerAction(period=delay_8_auto_recharge, actions=[auto_recharge_ros2_node]),
        TimerAction(period=delay_9_visualtracker, actions=[visualtracker_node]),
        TimerAction(period=delay_10_bt_node, actions=[bt_node]),
	])
