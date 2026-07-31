#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # 1 - Static transform
    static_tf_odom_combined = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_odom_combined_tf',
        arguments=[
            '0', '0', '0',      # x y z
            '0', '0', '0',      # roll pitch yaw
            'odom',
            'odom_combined'
        ],
        output='screen'
    )

    # 2 - Navigatie
    wheeltec_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wheeltec_nav2'),
                'launch',
                'wheeltec_nav2.launch.py'
            ])
        )
    )

    # 3 - Drive Mux
    drive_mux_node = Node(
        package='drive_mux',
        executable='drive_mux',
        name='drive_mux',
    )

    # 4 - Quiz_bt_node (socket naar roscommando's)
    quiz_bt_node = Node(
        package='quiz_bt_node',
        executable='quiz_bt_node',
        name='quiz_bt_node',
        output='screen'
    )

    # 5 - Drive to coord
    drive_to_coord_node = Node(
        package='drive_to_coord',
        executable='drive_to_coord',
        output='log',
        arguments=['--ros-args', '--log-level', 'error']
    )

    # 6 - Bumper
    bumper_node = Node(
        package='bumper',
        executable='bumper',
        output='log',
        arguments=['--ros-args', '--log-level', 'error']
    )

    # 7 - Camera
    wheeltec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turn_on_wheeltec_robot'),
                'launch',
                'wheeltec_camera.launch.py'
            ])
        )
    )

    # 8 - Pointcloud to laserscan (zodat cameraview wordt opgenomen door nav)
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        output='log',
        arguments=['--ros-args', '--log-level', 'error'],
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

    # 9 - Keepoutfilter (no go zones)
    keepout_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wheeltec_nav2'),
                'launch',
                'keepout_filter.launch.py'
            ])
        )
    )

    # 10 - Autocharge
    auto_recharge_ros2_node = Node(
        package='auto_recharge_ros2',
        executable='auto_recharge',
        output='screen'
    )
    
    # 11 - Apriltag
    apriltag_node = Node(
        package='april_tabloo',
        executable='april_tabloo',
        name='april_tabloo',
        output='screen'
    )

    # 12 - Visual tracker (people follower)
    visualtracker_node = Node(
        package='people_follower_ros2',
        executable='visualtracker',
        output='log',
        arguments=['--ros-args', '--log-level', 'error']
    )

    # 13 - Robot position reset
    robot_position_reset_node = Node(
        package='robot_position_reset',
        executable='robot_position_reset',
        name='robot_position_reset',
        output='screen'
    )

    # 14 - Behavior Tree
    bt_node = Node(
        package='mecabot_bt',
        executable='bt_node',
        output='screen'
    )

    # 15 - Bat check
    batcheck_node = Node(
        package='autocharge_batcheck',
        executable='autocharge_batcheck',
        name='autocharge_batcheck',
        output='screen'
    )
    
    # ==========================================
    # DELAYS
    # ==========================================

    delay_1_tf = 0.0
    delay_2_nav2 = 20.0
    delay_3_drive_mux = 40.0
    delay_4_quiz_bt = 60.0
    delay_5_drive_to_coord = 80.0
    delay_6_bumper = 100.0
    delay_7_camera = 120.0
    delay_8_pointcloud_to_laserscan = 140.0
    delay_9_keepout_filter = 160.0
    delay_10_visualtracker = 180.0
    delay_11_auto_recharge = 200.0
    delay_12_apriltag = 220.0
    delay_13_robot_position_reset = 240.0
    delay_14_bt_node = 260.0
    delay_15_batcheck = 280.0

    # ==========================================
    # LAUNCH DESCRIPTION RETURN
    # ==========================================

    return LaunchDescription([
        TimerAction(period=delay_1_tf, actions=[static_tf_odom_combined]),
        TimerAction(period=delay_2_nav2, actions=[wheeltec_nav2_launch]),
        TimerAction(period=delay_3_drive_mux, actions=[drive_mux_node]),
        TimerAction(period=delay_4_quiz_bt, actions=[quiz_bt_node]),
        TimerAction(period=delay_5_drive_to_coord, actions=[drive_to_coord_node]),
        TimerAction(period=delay_6_bumper, actions=[bumper_node]),
        TimerAction(period=delay_7_camera, actions=[wheeltec_camera_launch]),
        TimerAction(period=delay_8_pointcloud_to_laserscan, actions=[pointcloud_to_laserscan_node]),
        TimerAction(period=delay_9_keepout_filter, actions=[keepout_filter_launch]),
        TimerAction(period=delay_10_visualtracker, actions=[visualtracker_node]),
        TimerAction(period=delay_11_auto_recharge, actions=[auto_recharge_ros2_node]),
        TimerAction(period=delay_12_apriltag, actions=[apriltag_node]),
        TimerAction(period=delay_13_robot_position_reset, actions=[robot_position_reset_node]),
        TimerAction(period=delay_14_bt_node, actions=[bt_node]),
        TimerAction(period=delay_15_batcheck, actions=[batcheck_node]),
    ])
