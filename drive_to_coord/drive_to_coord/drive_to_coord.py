import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class DriveToCoord(Node):
	def __init__(self):
		super().__init__('drive_to_goal')
		self.get_logger().info('DriveToGoal init')

		# Beschermt gedeelde variabelen tegen gelijktijdige toegang door verschillende threads
		self.lock = threading.Lock()

		self.last_BehaviorTreeNode = None
		self.last_btDriveCoord = None
		self.last_peoplesearchcoord = None
		
		self.currentgoal = None
		self._goal_handle = None
		self.needs_action = False

		# --- CALLBACK GROUPS ---
		# Twee aparte groepen: één voor inkomende data, één voor de verwerkings-loop
		self.sub_cb_group = MutuallyExclusiveCallbackGroup()
		self.timer_cb_group = MutuallyExclusiveCallbackGroup()
		self.nav_cb_group = MutuallyExclusiveCallbackGroup()

		# --- ACTION CLIENT ---
		self._action_client = ActionClient(
			self, NavigateToPose, 'navigate_to_pose', 
			callback_group=self.nav_cb_group
		)

		# --- PUBLISHERS ---
		self.status_pub = self.create_publisher(String, '/drive_to_coord_status', 10)
		self.estop_cmd_vel_pub = self.create_publisher(Twist, '/estop_cmd_vel', 1)
		self.keepout_filter_pub = self.create_publisher(Bool, '/toggle_keepout', 1)


		# --- SUBSCRIBERS ---
		self.BehaviorTreeNode_sub = self.create_subscription(
			String, '/BehaviorTreeNode', self.BehaviorTreeNode_callback, 1, 
			callback_group=self.sub_cb_group
		)
		self.btDriveCoord_sub = self.create_subscription(
			PoseStamped, '/btDriveCoord', self.btDriveCoord_callback, 1, 
			callback_group=self.sub_cb_group
		)
		self.BTnode_sub = self.create_subscription(
			PoseStamped, '/peoplesearchcoord', self.peoplesearchcoord_callback, 1, 
			callback_group=self.sub_cb_group
		)

		# --- ROS TIMER (Vervangt de handmatige thread + sleep) ---
		# Draait exact 4 keer per seconde (0.25s interval)
		self.timer = self.create_timer(
			0.25, self.control_loop, 
			callback_group=self.timer_cb_group
		)

		# Wachten op nav server & Status versturen
		self._action_client.wait_for_server()
		self.get_logger().info('DriveToGoal node gestart.')
		self.publish_status(10, "gestart")

	# --- CALLBACKS (Sub_cb_group) ---

	def BehaviorTreeNode_callback(self, msg):
		with self.lock:
			self.last_BehaviorTreeNode = msg.data.strip()
			self.needs_action = True
		self.get_logger().info("Nieuw topic ontvangen van BehaviorTree.")

	def btDriveCoord_callback(self, msg):
		with self.lock:
			self.last_btDriveCoord = msg
		self.publish_status(11, "coördinaat opgeslagen")
		self.get_logger().info('Nieuw coördinaat ontvangen.')

	def peoplesearchcoord_callback(self, msg):
		with self.lock:
			self.last_peoplesearchcoord = msg
			self.needs_action = True

	# --- 4 HZ CONTROL LOOP (Timer_cb_group) ---

	def control_loop(self):
		"""Wordt door de ROS timer 4 keer per seconde aangeroepen."""
		with self.lock:
			if self.needs_action:
				self.actiondistribute()
				self.needs_action = False

	# --- LOGICA & ACTIES ---

	def actiondistribute(self):
		# Aangeroepen vanuit de control_loop, de lock is al actief.
		
		if self.last_BehaviorTreeNode in ["DriveToChargingStation","StatusDriveToChargingDock","IsRobotCharging","IsBatteryFull","BatteryCharged", "RobotWaitInChargingStation", "StopRobotCharging", "ManualDriving", "MDForceCharging" ]:
			self.keepout_filter(False)
		else:
			self.keepout_filter(True)

		if self.last_BehaviorTreeNode in ["IsRobotAtQuiz", "IsRobotAtWorkArea"]:
			self.send_goal("behaviortree", self.last_btDriveCoord)

		elif self.last_BehaviorTreeNode in ["CheckingNearbyVisitors", "DriveWorkArea", "DriveQuizLocation"]:
			self.send_goal("peoplesearch", self.last_peoplesearchcoord)

		elif self.last_BehaviorTreeNode in ["DriveToChargingStation","StatusDriveToChargingDock","IsRobotCharging","IsBatteryFull","BatteryCharged", "RobotWaitInChargingStation", "StopRobotCharging", "ManualDriving", "MDForceCharging"]:
			return
			
		else:
			self.emergencystop()

	def keepout_filter(self, state):
		msg = Bool()
		msg.data = state
		self.keepout_filter_pub.publish(msg)
	
	def emergencystop(self):
		# Robot laten stoppen door 0 te sturen op /estop_cmd_vel
		self.get_logger().info("Noodstop procedure: 0-velocity sturen.")
		stop_msg = Twist()
		self.estop_cmd_vel_pub.publish(stop_msg)
	
		# Goal proberen te cancelen
		if self._goal_handle is not None:
			self.get_logger().info("Bezig met annuleren van huidige goal...")
			try:
				self._goal_handle.cancel_goal_async()
				self._goal_handle = None
			except Exception as e:
				self.get_logger().warn(f"Fout bij annuleren goal: {e}")

	def send_goal(self, source, coordinate=None):
		if coordinate is None:
			msg = f"Geen coördinaat ontvangen van {source} — goal niet gestuurd!"
			self.get_logger().warn(msg)
			self.publish_status(12, msg)
			return
		
		self.currentgoal = NavigateToPose.Goal()
		self.currentgoal.pose = coordinate

		self.publish_status(13, "goal verzonden")

		self._send_goal_future = self._action_client.send_goal_async(self.currentgoal)
		self._send_goal_future.add_done_callback(self.goal_response_callback)
		

	# --- CALLBACKS (Nav_cb_group) ---
	def goal_response_callback(self, future):
		with self.lock:
			self._goal_handle = future.result()
			
			if not self._goal_handle.accepted:
				self.publish_status(10, "goal afgewezen")
				self.get_logger().warn("Goal NIET geaccepteerd door Nav2!")
				self._goal_handle = None
				return

			self.get_logger().info("Goal geaccepteerd ✅")
			self.publish_status(15, "goal geaccepteerd")

			self._get_result_future = self._goal_handle.get_result_async()
			self._get_result_future.add_done_callback(self.result_callback)

	def result_callback(self, future):
		with self.lock:
			status = future.result().status
			self.publish_status(status, "NAV")
			self.get_logger().info(f"Goal afgerond, Nav2-status: {status:02d}")
			self._goal_handle = None

	# --- status publisher ---
	def publish_status(self, status, text: str):
		msg = String()

		if self.last_btDriveCoord is None:
			stamp = "0000000000000000000"
		else:
			stamp = f"{self.last_btDriveCoord.header.stamp.sec:010d}.{self.last_btDriveCoord.header.stamp.nanosec:09d}"

		msg.data = f"{status:02d}-{stamp}-{text}"

		self.status_pub.publish(msg)
		self.get_logger().info(f"[STATUS] {text}")


def main(args=None):
	rclpy.init(args=args)
	node = DriveToCoord()
	
	executor = MultiThreadedExecutor(num_threads=3)
	executor.add_node(node)
	
	try:
		executor.spin()
	except KeyboardInterrupt:
		node.get_logger().info('Afgebroken door gebruiker.')
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()

