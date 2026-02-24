import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.time import Time


class DriveToCoord(Node):
	def __init__(self):
		super().__init__('drive_to_goal')
		self.get_logger().info('DriveToGoal init')

		self.last_BehaviorTreeNode = None
		self.last_btDriveCoord = None
		self.last_peoplesearchcoord = None
		
		self.currentgoal = None
		self._goal_handle = None

		# Runtime / flow-control (latest-only processing, stamps, and priority)
		self._last_btDriveCoord_stamp = 0.0
		self._last_peoplesearchcoord_stamp = 0.0
		self._last_goal_stamp_sent = 0.0
		self._coords_dirty = False
		self._behavior_dirty = False
		self._actiondistribute_rate_hz = 5.0  # process incoming state at up to 5 Hz
		self._action_lock = threading.Lock()
		# timer to process only the latest snapshot (prevents backlog / old-data processing)
		self._actiondistribute_timer = self.create_timer(1.0 / self._actiondistribute_rate_hz, self._periodic_actiondistribute)

		# Action client
		self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

		# Publishers
		self.status_pub = self.create_publisher(String, '/drive_to_coord_status', 10)
		self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
		self.extendedzone_pub = self.create_publisher(String, '/allow_extended_zone', 1)

		# Subscribers
		self.BehaviorTreeNode_sub = self.create_subscription(String, '/BehaviorTreeNode', self.BehaviorTreeNode_callback, 1)
		self.btDriveCoord_sub = self.create_subscription(PoseStamped, '/btDriveCoord', self.btDriveCoord_callback, 1)
		self.BTnode_sub = self.create_subscription(PoseStamped, '/peoplesearchcoord', self.peoplesearchcoord_callback, 1)

		# Wachten op nav server & Status versturen
		self._action_client.wait_for_server()
		self.get_logger().info('DriveToGoal node gestart.')
		self.publish_status(10, "gestart")


	def BehaviorTreeNode_callback(self, msg):
		self.get_logger().info("Nieuw topic ontvangen van BehaviorTree.")
		self.last_BehaviorTreeNode = msg.data.strip()
		# mark behavior as dirty and process with priority immediately
		self._behavior_dirty = True
		# try to acquire lock briefly to preempt periodic processing
		acquired = self._action_lock.acquire(timeout=0.1)
		if acquired:
			try:
				self.actiondistribute()
			finally:
				self._action_lock.release()
		else:
			self.get_logger().debug("BehaviorTreeNode priority: verwerking uitgesteld naar timer.")

	def btDriveCoord_callback(self, msg):
		# only accept newer coordinates (drop old/stale messages)
		msg_stamp = self._stamp_to_seconds(msg.header.stamp)
		if msg_stamp <= self._last_btDriveCoord_stamp:
			self.get_logger().debug("btDriveCoord - oud bericht genegeerd.")
			return
		self.last_btDriveCoord = msg
		self._last_btDriveCoord_stamp = msg_stamp
		self._coords_dirty = True
		self.publish_status(11, "coördinaat opgeslagen")
		self.get_logger().info('Nieuw coördinaat ontvangen.')

	def peoplesearchcoord_callback(self, msg):
		# accept only newer messages and mark for periodic processing
		msg_stamp = self._stamp_to_seconds(msg.header.stamp)
		if msg_stamp <= self._last_peoplesearchcoord_stamp:
			self.get_logger().debug("peoplesearchcoord - oud bericht genegeerd.")
			return
		#msg.header.frame_id = "base_link"
		self.last_peoplesearchcoord = msg
		self._last_peoplesearchcoord_stamp = msg_stamp
		self._coords_dirty = True

	def actiondistribute(self):
		# Non-blocking guard: laat maar één instance tegelijk verwerken
		acquired = self._action_lock.acquire(blocking=False)
		if not acquired:
			self.get_logger().debug("actiondistribute is busy; overslaan.")
			return
		try:
			# Kies of je de extended zone naar het laadstation nodig hebt of niet
			if self.last_BehaviorTreeNode in ["DriveToChargingStation","StatusDriveToChargingDock","IsRobotCharging","IsBatteryFull","BatteryCharged", "DriveWorkArea"]:
				msg = String()
				msg.data = "true"
				self.extendedzone_pub.publish(msg)
			else:
				msg = String()
				msg.data = "false"
				self.extendedzone_pub.publish(msg)
			
			# Beslissingsboom: BehaviorTreeNode heeft prioriteit boven coordinate-updates
			if self.last_BehaviorTreeNode in ["IsRobotAtQuiz", "RobotAtWorkArea"]:
				if self.last_btDriveCoord is None:
					self.get_logger().warn("Geen coördinaat ontvangen van btDriveCoord — goal niet gestuurd!")
					self.publish_status(12, "geen coördinaat ontvangen van btDriveCoord")
					return
				coord_stamp = self._stamp_to_seconds(self.last_btDriveCoord.header.stamp)
				# drop stale/duplicate goals
				if coord_stamp <= self._last_goal_stamp_sent:
					self.get_logger().info("btDriveCoord is niet nieuwer dan laatst verzonden goal — genegeerd.")
					return
				if self._goal_handle is not None and self.currentgoal is not None:
					active_stamp = self._stamp_to_seconds(self.currentgoal.pose.header.stamp)
					if coord_stamp == active_stamp:
						self.get_logger().info("Zelfde goal is al actief — overslaan.")
						return
				self.send_goal(self.last_btDriveCoord)

			elif self.last_BehaviorTreeNode in ["CheckingNearbyVisitors", "DriveWorkArea", "DriveQuizLocation"]:
				if self.last_peoplesearchcoord is None:
					self.get_logger().warn("Geen coördinaat ontvangen van peoplesearch — goal niet gestuurd!")
					self.publish_status(12, "geen coördinaat ontvangen van peoplesearch")
					return
				coord_stamp = self._stamp_to_seconds(self.last_peoplesearchcoord.header.stamp)
				if coord_stamp <= self._last_goal_stamp_sent:
					self.get_logger().info("peoplesearchcoord is niet nieuwer dan laatst verzonden goal — genegeerd.")
					return
				if self._goal_handle is not None and self.currentgoal is not None:
					active_stamp = self._stamp_to_seconds(self.currentgoal.pose.header.stamp)
					if coord_stamp == active_stamp:
						self.get_logger().info("Zelfde peoplesearch goal is al actief — overslaan.")
						return
				self.send_goal(self.last_peoplesearchcoord)

			elif self.last_BehaviorTreeNode in ["DriveToChargingStation","StatusDriveToChargingDock","IsRobotCharging","IsBatteryFull","BatteryCharged"]:
				# Navigatie laten rusten bij batterijtopics
				return
			
			else:
				# als BehaviorTreeNode geen bekende status heeft, emergency stop
				self.emergencystop()
				self.get_logger().warn(f"Status '{self.last_BehaviorTreeNode}' niet relevant voor drive_to_goal. Stop geactiveerd.")
		finally:
			self._action_lock.release()

	def emergencystop(self):
		# <--- CHECK: Alleen annuleren als er een goal handle bestaat
		if self._goal_handle is not None:
			self.get_logger().info("Bezig met annuleren van huidige goal...")
			try:
				self._goal_handle.cancel_goal_async()
			except Exception as e:
				self.get_logger().warn(f"Fout bij annuleren goal: {e}")
			
			self._goal_handle = None # Reset handle zodat we niet opnieuw proberen te annuleren
		
		# Stuur stop commando's
		self.get_logger().info("Noodstop procedure: 0-velocity sturen.")
		stop_msg = Twist()
		for i in range(20):
			self.cmd_vel_pub.publish(stop_msg)

	def send_goal(self, coordinate):
		# guard against stale goals
		coord_stamp = self._stamp_to_seconds(coordinate.header.stamp)
		if coord_stamp <= self._last_goal_stamp_sent:
			self.get_logger().info("send_goal: goal is niet nieuwer dan laatst verzonden — genegeerd.")
			return

		# if the same goal is already active, skip
		if self._goal_handle is not None and self.currentgoal is not None:
			active_stamp = self._stamp_to_seconds(self.currentgoal.pose.header.stamp)
			if coord_stamp == active_stamp:
				self.get_logger().info("send_goal: identieke goal al actief — overslaan.")
				return

		self.currentgoal = NavigateToPose.Goal()
		self.currentgoal.pose = coordinate

		# remember stamp we sent (prevents race with incoming older messages)
		self._last_goal_stamp_sent = coord_stamp

		self.publish_status(13, "goal verzonden")

		self._send_goal_future = self._action_client.send_goal_async(self.currentgoal)
		self._send_goal_future.add_done_callback(self.goal_response_callback)

	def goal_response_callback(self, future):
		# <--- GEWIJZIGD: Opslaan in self._goal_handle, niet in een lokale variabele
		self._goal_handle = future.result()
		
		if not self._goal_handle.accepted:
			self.publish_status(10, "goal afgewezen")
			self.get_logger().warn("Goal NIET geaccepteerd door Nav2!")
			self._goal_handle = None # Reset als geweigerd
			return

		self.get_logger().info("Goal geaccepteerd ✅")
		self.publish_status(15, "goal geaccepteerd")

		self._get_result_future = self._goal_handle.get_result_async()
		self._get_result_future.add_done_callback(self.result_callback)

	def result_callback(self, future):
		status = future.result().status
		self.publish_status(status, "NAV")  # NAV statussen
		self.get_logger().info(f"Goal afgerond, Nav2-status: {status:02d}")
		
		# Goal is klaar, dus we resetten de handle zodat emergencystop niet crasht
		self._goal_handle = None 

	def publish_status(self, status, text: str):
		msg = String()

		if self.last_btDriveCoord == None:
			stamp = "0000000000000000000"
		else:
			# Let op: msg.sec en msg.nanosec zijn integers, format ze correct
			stamp = f"{self.last_btDriveCoord.header.stamp.sec:010d}.{self.last_btDriveCoord.header.stamp.nanosec:09d}"

		msg.data = f"{status:02d}" + "-" + stamp + "-" + text

		self.status_pub.publish(msg)
		self.get_logger().info(f"[STATUS] {text}")

	def _stamp_to_seconds(self, stamp):
		"""Convert a ROS2 header.stamp to float seconds (handles zero stamps)."""
		return float(stamp.sec) + float(stamp.nanosec) * 1e-9

	def _periodic_actiondistribute(self):
		"""
		Timer-driven processing that only acts on the latest data.
		This prevents rclpy's executor from falling behind when incoming
		topics are faster than we can process.
		"""
		# Only run actiondistribute when there's new state to act on
		if not (self._behavior_dirty or self._coords_dirty):
			return
		# clear dirty flags and process the latest snapshot
		self._behavior_dirty = False
		self._coords_dirty = False
		self.actiondistribute()


def main(args=None):
	rclpy.init(args=args)
	node = DriveToCoord()
	executor = MultiThreadedExecutor()
	executor.add_node(node)
	try:
		executor.spin()
	except KeyboardInterrupt:
		node.get_logger().info('Afgebroken door gebruiker.')
	finally:
		executor.shutdown()
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()

