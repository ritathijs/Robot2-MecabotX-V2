import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.time import Time


class DriveToCoord(Node):
	def __init__(self):
		super().__init__('drive_to_goal')

		self.lastcoord = None
		self._goal_handle = None

		# Publishers
		self.status_pub = self.create_publisher(String, '/drive_to_coord_status', 10)
		self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

		# Action client
		self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

		# Subscribers
		self.BTnode_sub = self.create_subscription(
			String,
			'/BehaviorTreeNode',
			self.handle_BTnode_callback,
			10
		)

		self.coord_sub = self.create_subscription(
			PoseStamped,
			'/btDriveCoord',
			self.incoming_goal_callback,
			10
		)

		# Status
		self.publish_status(10, "gestart")
		self.get_logger().info('DriveToGoal node gestart.')

	def publish_status(self, status, text: str):
		msg = String()
		
		if self.lastcoord == None:
			stamp = "0000000000000000000"
		else:
			stamp = f"{self.lastcoord.header.stamp.sec:010d}.{self.lastcoord.header.stamp.nanosec:09d}"
			
		msg.data = f"{status:02d}" +"-"+ stamp +"-"+ text
		
		self.status_pub.publish(msg)
		self.get_logger().info(f"[STATUS] {text}")

	def incoming_goal_callback(self, msg):
		self.lastcoord = msg
		self.publish_status(11, "coördinaat opgeslagen")
		self.get_logger().info('Nieuw coördinaat ontvangen.')

	def handle_BTnode_callback(self, msg: String):
		self.get_logger().info("Nieuw topic ontvangen van BehaviorTree.")
		data = msg.data.strip()
		if data in ["IsRobotAtQuiz", "drive"]:
			self.send_goal()
		else:
			self.get_logger().warn("Niet relevant voor drive_to_goal.")

	def send_goal(self):
		if self.lastcoord is None:
			self.get_logger().warn("Geen coördinaat ontvangen — goal niet gestuurd!")
			self.publish_status(12, "geen coördinaat")
			return

		self.get_logger().info("Goal wordt verzonden...")

		self._action_client.wait_for_server()
		self.get_logger().info("Nav2 server beschikbaar.")

		goal_pose = PoseStamped()
		goal_pose.header.frame_id = 'map'
		goal_pose.header.stamp = Time().to_msg()
		goal_pose.pose = self.lastcoord.pose

		goal = NavigateToPose.Goal()
		goal.pose = goal_pose

		self.publish_status(13, "goal verzonden")

		self._send_goal_future = self._action_client.send_goal_async(goal)
		self._send_goal_future.add_done_callback(self.goal_response_callback)

	def goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.publish_status(10, "goal afgewezen")
			self.get_logger().warn("Goal NIET geaccepteerd door Nav2!")
			return

		self.get_logger().info("Goal geaccepteerd ✅")
		self._goal_handle = goal_handle
		self.publish_status(15, "goal geaccepteerd")

		self._get_result_future = goal_handle.get_result_async()
		self._get_result_future.add_done_callback(self.result_callback)

	def result_callback(self, future):
		status = future.result().status
		self.publish_status(status, "NAV") #NAV statussen
		self.get_logger().info(f"Goal afgerond, Nav2-status: {status02d}")


def main(args=None):
	rclpy.init(args=args)
	node = DriveToCoord()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.get_logger().info('Afgebroken door gebruiker.')
	finally:
		rclpy.shutdown()


if __name__ == '__main__':
	main()

