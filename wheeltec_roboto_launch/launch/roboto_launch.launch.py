#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- Launch includes (ros2 launch ...) ----
    wheeltec_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wheeltec_nav2'),
                'launch',
                'wheeltec_nav2.launch.py'
            ])
        )
    )

    wheeltec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turn_on_wheeltec_robot'),
                'launch',
                'wheeltec_camera.launch.py'
            ])
        )
    )

    # apriltag_sync_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare('apriltag_sync'),
    #             'launch',
    #             'apriltag_sync.launch.py'
    #         ])
    #     )
    # )

    keepout_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wheeltec_nav2'),
                'launch',
                'keepout_filter.launch.py'
            ])
        )
    )

    # ---- Nodes (ros2 run ...) ----
    drive_to_coord_node = Node(
        package='drive_to_coord',
        executable='drive_to_coord',
        output='screen'
    )

    bumper_node = Node(
        package='bumper',
        executable='bumper',
        output='screen'
    )

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

    visualtracker_node = Node(
        package='people_follower_ros2',
        executable='visualtracker',
        output='screen'
    )

    bt_node = Node(
        package='mecabot_bt',
        executable='bt_node',
        output='screen'
    )
    
    auto_recharge_ros2_node = Node(
	package='auto_recharge_ros2',
	executable='auto_recharge',
	output='screen'
    )
    
    amcl_pose_saver_node = Node(
    	package='pose_persistence',
    	executable='amcl_pose_saver',
    	output='screen'
    )
    
    initial_pose_restorer_node = Node(
    	package='pose_persistence',
    	executable='initial_pose_restorer',
    	output='screen'
    )

    # ---- Sequencing ----
    # 1) ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py
    # 2) ros2 run drive_to_coord drive_to_coord
    # 3) ros2 run bumper bumper
    # 4) ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py
    # 5) ros2 run pointcloud_to_laserscan ...
    # 6) ros2 launch apriltag_sync apriltag_sync.launch.py
    # 7) ros2 launch wheeltec_nav2 keepout_filter.launch.py
    # 8) ros2 run people_follower_ros2 visualtracker
    # 10) ros2 run auto_recharge_ros2 auto_recharge
    # 12) ros2 run pose_persistence initial_pose_restorer
    # 9) ros2 run mecabot_bt bt_node
    # 11) ros2 run pose_persistence amcl_pose_saver


   
    t1 = 0.0
    t2 = 10.0
    t3 = 13.0
    t4 = 14.0
    t5 = 16.0
    t6 = 17.0
    t7 = 18.0
    t8 = 19.0
    t10 =20.0
    t12 = 21.0
    t9 = 23.0
    t11 = 25.0

    return LaunchDescription([
        TimerAction(period=t1, actions=[wheeltec_nav2_launch]),
        TimerAction(period=t2, actions=[drive_to_coord_node]),
        TimerAction(period=t3, actions=[bumper_node]),
        TimerAction(period=t4, actions=[wheeltec_camera_launch]),
        TimerAction(period=t5, actions=[pointcloud_to_laserscan_node]),
        # TimerAction(period=t6, actions=[apriltag_sync_launch]),
        TimerAction(period=t7, actions=[keepout_filter_launch]),
        TimerAction(period=t8, actions=[visualtracker_node]),
        TimerAction(period=t10, actions=[auto_recharge_ros2_node]),
        TimerAction(period=t12, actions=[initial_pose_restorer_node]),
        TimerAction(period=t9, actions=[bt_node]),
        TimerAction(period=t11, actions=[amcl_pose_saver_node]),
    ])
