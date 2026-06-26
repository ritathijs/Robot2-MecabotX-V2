import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from dataclasses import dataclass

@dataclass
class GoalRecord:
	source: str				  # De bron van het doel (BehaviorTree of PeopleSearch)
	goal: NavigateToPose.Goal	# De PoseStamped (coördinaten)
	handle: any = None		   # De ClientGoalHandle (wordt later gevuld)
	result: any = None		   # Het uiteindelijke Resultaat
	accepted: bool = False	   # Is het doel geaccepteerd?
	status: int = 0			  # De statuscode (1=Pending, 4=Canceled, etc.)

behavior_tree_nodes = {
	# keepoutfilter_on: "True" / "False" / "none"
	# drive_action: "estop" / "release" / "behaviortree" / "peoplesearch"
	# message_frequency: "always" / "once"

	# --- Oplaad statussen ---
	"DriveToChargingStation":	  {"keepoutfilter_on": "False", "drive_action": "release", "message_frequency": "once"},
	"StatusDriveToChargingDock":	{"keepoutfilter_on": "False", "drive_action": "release", "message_frequency": "once"},
	"IsRobotCharging":			{"keepoutfilter_on": "False", "drive_action": "release", "message_frequency": "once"},
	"IsBatteryFull":				{"keepoutfilter_on": "False", "drive_action": "release", "message_frequency": "once"},
	"BatteryCharged":			  {"keepoutfilter_on": "False", "drive_action": "release", "message_frequency": "once"},
	"RobotWaitInChargingStation":   {"keepoutfilter_on": "False", "drive_action": "release", "message_frequency": "once"},
	"StopRobotCharging":			{"keepoutfilter_on": "False", "drive_action": "release", "message_frequency": "once"},
	"ManualDriving":				{"keepoutfilter_on": "False", "drive_action": "release", "message_frequency": "once"},
	"MDForceCharging":			{"keepoutfilter_on": "False", "drive_action": "release", "message_frequency": "once"},


	"RobotDriveToChargingStation":	{"keepoutfilter_on": "False",  "drive_action": "behaviortree", "message_frequency": "always"},
	"RobotIsRobotAtChargingStation":   {"keepoutfilter_on": "False",  "drive_action": "behaviortree", "message_frequency": "once"},


	"RobotRotationFollowMe":	{"keepoutfilter_on": "False", "drive_action": "release", "message_frequency": "once"},

	# --- Behavior Tree doelen ---
	"IsRobotAtQuiz":				{"keepoutfilter_on": "True",  "drive_action": "behaviortree", "message_frequency": "once"},
	"IsRobotAtWorkArea":			{"keepoutfilter_on": "True",  "drive_action": "behaviortree", "message_frequency": "once"},
	"DriveQuizLocation":			{"keepoutfilter_on": "True",  "drive_action": "estop", "message_frequency": "once"},

	# --- People Search doelen ---
	"CheckingNearbyVisitors":	  {"keepoutfilter_on": "True",  "drive_action": "peoplesearch", "message_frequency": "always"},
	"DriveWorkArea":				{"keepoutfilter_on": "True",  "drive_action": "behaviortree", "message_frequency": "always"},
	"DriveQuizLocation":			{"keepoutfilter_on": "True",  "drive_action": "behaviortree", "message_frequency": "always"},
	
	# --- Default ---
	"Default":					{"keepoutfilter_on": "none",  "drive_action": "estop", "message_frequency": "always"},
}

class DriveToCoord(Node):
	def __init__(self):
		super().__init__('drive_to_goal')
		self.get_logger().info('DriveToGoal init')

		# Beschermt gedeelde variabelen tegen gelijktijdige toegang door verschillende threads
		self.lock = threading.Lock()

		self.last_BehaviorTreeNode = None
		self.last_btDriveCoord = None
		self.last_peoplesearchcoord = None
		
		self.current_goals = []
		self.needs_action = False
		
		self.keepoutfilter_state = None

		# --- CALLBACK GROUPS ---
		# drie aparte groepen: één voor inkomende data, één voor de verwerkings-loop, en één voor de callbacks van de nav
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
		self.peoplesearchCoord_sub = self.create_subscription(
			PoseStamped, '/peoplesearchcoord', self.peoplesearchcoord_callback, 1, 
			callback_group=self.sub_cb_group
		)

		# --- ROS TIMER (Vervangt de handmatige thread + sleep) ---
		# Draait exact 4 keer per seconde (0.25s interval)
		self.timer = self.create_timer(
			0.5, self.control_loop, 
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

	# --- 4 HZ CONTROL LOOP (Timer_cb_group) ---

	def control_loop(self):
		"""Wordt door de ROS timer 4 keer per seconde aangeroepen."""
		with self.lock:
			if self.needs_action:
				self.actiondistribute()

	# --- LOGICA & ACTIES ---

	def actiondistribute(self):
		# Aangeroepen vanuit de control_loop, de lock is al actief.
		
		currentnode = behavior_tree_nodes.get(self.last_BehaviorTreeNode)

		if currentnode is None:
			currentnode = behavior_tree_nodes["Default"]
			self.get_logger().error(
				f"Onbekende BehaviorTreeNode ontvangen: '{self.last_BehaviorTreeNode}' → ESTOP geactiveerd!"
			)
			print(f"[ESTOP TRIGGER] Onbekende node: {self.last_BehaviorTreeNode}")
		
		if currentnode["keepoutfilter_on"] == "True": 
			self.keepout_filter(True)
		elif currentnode["keepoutfilter_on"] == "False": 
			self.keepout_filter(False)
			
		if currentnode["drive_action"] == "estop":
			self.emergencystop()
			self.cancel_all_goals()
		elif currentnode["drive_action"] == "release":
			self.cancel_all_goals()
		elif currentnode["drive_action"] == "behaviortree":
			self.send_goal("BehaviorTree", self.last_btDriveCoord)
		elif currentnode["drive_action"] == "peoplesearch":
			self.send_goal("PeopleSearch", self.last_peoplesearchcoord)
			
		if currentnode["message_frequency"] == "always":
			self.needs_action = True
		elif currentnode["message_frequency"] == "once":
			self.needs_action = False

	def keepout_filter(self, state):
		if self.keepoutfilter_state != state:
			self.keepoutfilter_state = state
			msg = Bool()
			msg.data = state
			self.keepout_filter_pub.publish(msg)
	
	def emergencystop(self):
		# Robot laten stoppen door 0 te sturen op /estop_cmd_vel
		self.get_logger().info("Noodstop procedure: 0-velocity sturen.")
		stop_msg = Twist()
		self.estop_cmd_vel_pub.publish(stop_msg)
	
	def cancel_all_goals(self):
		if not self.current_goals:
			return

		# Doorloop de records in omgekeerde volgorde
		for record in reversed(self.current_goals):
			# We kunnen alleen annuleren als er een handle is opgeslagen
			if record.handle is not None and record.accepted:
				self.get_logger().info("Bezig met annuleren van huidige goal...")
				try:
					record.handle.cancel_goal_async()
				except Exception as e:
					self.get_logger().warn(f"Fout bij annuleren goal: {e}")
		
		# Maak de lijst leeg nadat alles is geannuleerd
		self.current_goals.clear()
		self.get_logger().info("Alle lokale goal-records zijn gewist.")

	def send_goal(self, source, coordinate=None):
		if coordinate is None:
			msg = f"Geen coördinaat ontvangen van {source} — goal niet gestuurd!"
			self.get_logger().warn(msg)
			self.publish_status(12, msg)
			return
		
		# Voorkom IndexError door eerst te checken of de lijst niet leeg is
		if self.current_goals:
			# Let op: we moeten .goal.pose aanspreken omdat .pose in het Goal() bericht zit
			if self.current_goals[-1].goal.pose.header.stamp == coordinate.header.stamp:
				self.get_logger().info("Zelfde coördinaat en timestamp als huidige goal, geen nieuwe goal verzonden.")
				return

		goal_msg = NavigateToPose.Goal()
		goal_msg.pose = coordinate

		new_record = GoalRecord(
			source=source,
			goal=goal_msg, 
			handle=None,
			result=None,
			accepted=False,
			status=0  # Nav2 statussen zijn integers (0 = onbekend/verzonden)
		)
		
		self.current_goals.append(new_record)
	
		self.publish_status(13, "goal verzonden")

		self.send_goal_future = self._action_client.send_goal_async(new_record.goal)
		
		# Geef het new_record mee aan de callback via een lambda
		self.send_goal_future.add_done_callback(
			lambda future: self.goal_response_callback(future, new_record)
		)
		

	# --- CALLBACKS (Nav_cb_group) ---
	def goal_response_callback(self, future, record):
		with self.lock:
			handle = future.result()
			
			# Sla de handle en de accepted-status op in ons record
			record.handle = handle
			record.accepted = handle.accepted
			
			if not record.accepted:
				self.publish_status(10, "goal afgewezen")
				self.get_logger().warn("Goal NIET geaccepteerd door Nav2!")
				record.status = 5 # 5 staat vaak voor Aborted/Rejected
				return

			self.get_logger().info("Goal geaccepteerd ✅")
			self.publish_status(15, "goal geaccepteerd")
			record.status = 2 # 2 staat voor Executing

			self._get_result_future = handle.get_result_async()
			
			# Geef het record opnieuw mee aan de result callback
			self._get_result_future.add_done_callback(
				lambda future: self.result_callback(future, record)
			)

	def result_callback(self, future, record):
		with self.lock:
			result_handle = future.result()
			
			# Vul de laatste informatie in het record in
			record.result = result_handle.result
			record.status = result_handle.status
			
			self.publish_status(record.status, "NAV")
			self.get_logger().info(f"Goal afgerond, Nav2-status: {record.status:02d}")

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





