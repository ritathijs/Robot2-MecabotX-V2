import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

	pkg_share = get_package_share_directory('apriltag_sync')
	tags_config = os.path.join(pkg_share, 'config', 'tags.yaml')
	locations_config_path = os.path.join(pkg_share, 'config', 'tag_locations.yaml')

	ld = LaunchDescription()

	# -------------------------------------------------
	# 1. AprilTag detector
	# -------------------------------------------------
	apriltag_node = Node(
		package='apriltag_ros',
		executable='apriltag_node',
		name='apriltag_node',
		parameters=[tags_config],
		remappings=[
			('/image_rect', '/camera/color/image_raw'),
			('/camera_info', '/camera/camera_info')
		]
	)
	ld.add_action(apriltag_node)

	# -------------------------------------------------
	# 2. Static tag frames in map
	# -------------------------------------------------
	with open(locations_config_path, 'r') as f:
		content = yaml.safe_load(f)
		locations = content.get('tag_locations', {})

	for tag_id, pos in locations.items():

		static_frame = f"static_tag36h11:{tag_id}"
		safe_node_name = f"static_tf_tag_{tag_id}"

		static_tf_node = Node(
			package='tf2_ros',
			executable='static_transform_publisher',
			name=safe_node_name,
			arguments=[
				str(pos['x']),
				str(pos['y']),
				str(pos['z']),
				str(pos['yaw']), '0', '0',
				'map',
				static_frame
			]
	)

	ld.add_action(static_tf_node)

	# -------------------------------------------------
	# 3. AMCL Calibrator
	# -------------------------------------------------
	calibrator_node = Node(
		package='apriltag_sync',
		executable='apriltag_amcl_calibrator',
		name='apriltag_amcl_calibrator',
		output='screen'
	)

	ld.add_action(calibrator_node)

	return ld

