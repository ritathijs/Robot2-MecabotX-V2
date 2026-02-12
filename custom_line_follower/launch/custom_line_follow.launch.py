import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paden naar de Wheeltec robot mappen
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # 1. Start de Robot Basis (motoren, odometrie, etc.)
    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py'))
    )

    # 2. Start de Camera
    wheeltec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_camera.launch.py'))
    )

    # 3. Start JOUW nieuwe custom node
    my_line_follower = Node(
        package='custom_line_follower',
        executable='line_follow_node',
        name='custom_line_follower',
        output='screen'
    )

    return LaunchDescription([
        wheeltec_robot,
        wheeltec_camera,
        my_line_follower
    ])
