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

	# These names are published by robot_controller.cpp. Keepout changes only
	# when crossing between the work area and the charging station.
	"DriveWorkArea": {"keepoutfilter_on": "False", "drive_action": "behaviortree", "message_frequency": "always"},
	"robot-go-charge": {"keepoutfilter_on": "False", "drive_action": "behaviortree", "message_frequency": "always"},
	"robot-docking": {"keepoutfilter_on": "False", "drive_action": "release", "message_frequency": "once"},

	"follow-robot-screen": {"keepoutfilter_on": "True", "drive_action": "behaviortree", "message_frequency": "always"},
	"robot-arrived-at-quiz-location": {"keepoutfilter_on": "True", "drive_action": "release", "message_frequency": "once"},
	"RobotRotationFollowMe": {"keepoutfilter_on": "True", "drive_action": "peoplesearch", "message_frequency": "always"},
	"robot-arrived-at-visitors": {"keepoutfilter_on": "True", "drive_action": "release", "message_frequency": "once"},
	"explore-location-c": {"keepoutfilter_on": "True", "drive_action": "behaviortree", "message_frequency": "always"},
	"explore-location-d": {"keepoutfilter_on": "True", "drive_action": "behaviortree", "message_frequency": "always"},
	"explore-location-e": {"keepoutfilter_on": "True", "drive_action": "behaviortree", "message_frequency": "always"},
	"robot-error-drive": {"keepoutfilter_on": "none", "drive_action": "estop", "message_frequency": "always"},

	# Neutrale toestand (o.a. na herstel van de startlocatie): blijf veilig stilstaan.
	"None": {"keepoutfilter_on": "none", "drive_action": "estop", "message_frequency": "always"},

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
		self.last_goal_end_time = None
		self.last_finished_pose = None
		self.RESEND_COOLDOWN_S = 5.0
		
		self.needs_action = False
		
		self.keepoutfilter_state = None
		self.robot_docked = False
		self.robot_active = True

		# IR-docking bezig: dan mogen er geen Nav2-goals naar het laadstation meer lopen
		self.docking_active = False
		self.docking_start_time = None
		self.DOCKING_ACTIVE_MAX_S = 210.0

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

		self.docked_sub = self.create_subscription(
			Bool, '/IsRobotDocked', self.docked_callback, 1,
			callback_group=self.sub_cb_group
		)

		self.active_sub = self.create_subscription(
			Bool, '/IsRobotActive', self.active_callback, 1,
			callback_group=self.sub_cb_group
		)

		self.force_charge_sub = self.create_subscription(
			String, '/force_charge', self.force_charge_callback, 10,
			callback_group=self.sub_cb_group
		)

		self.charge_event_sub = self.create_subscription(
			String, '/auto_recharge_event', self.charge_event_callback, 10,
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
			node_name = msg.data.strip()
			self.last_BehaviorTreeNode = node_name
			self.docking_active = False
			self.needs_action = True
		self.get_logger().info("Nieuw topic ontvangen van BehaviorTree.")

	def docked_callback(self, msg):
		with self.lock:
			self.robot_docked = msg.data

	def force_charge_callback(self, msg):
		command = msg.data.strip().upper()
		with self.lock:
			if command.endswith("START"):
				self.docking_active = True
				self.docking_start_time = self.get_clock().now()
				self.needs_action = True
			elif command.endswith("STOP"):
				self.docking_active = False

	def charge_event_callback(self, msg):
		event = msg.data.strip().upper()
		if event.endswith((
			"DOCKING-FAILED", "ROBOT-UNDOCKED",
			"ROBOT-CHARGING", "ROBOT-CHARGING-STOPPED",
		)):
			with self.lock:
				self.docking_active = False

	def docking_in_progress(self):
		if not self.docking_active:
			return False
		elapsed = (self.get_clock().now() - self.docking_start_time).nanoseconds / 1e9
		if elapsed > self.DOCKING_ACTIVE_MAX_S:
			self.docking_active = False
			return False
		return True

	def active_callback(self, msg):
		with self.lock:
			self.robot_active = msg.data

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
		elif (
			currentnode["drive_action"] == "behaviortree"
			and self.last_BehaviorTreeNode == "robot-go-charge"
			and (self.robot_docked or self.docking_in_progress())
		):
			# Gedockt of IR-docking bezig: het laadstation-doel niet blijven
			# herhalen en lopende goals stoppen zodat Nav2 niet tegenwerkt.
			self.cancel_all_goals()
		elif (
			currentnode["drive_action"] == "behaviortree"
			and not self.robot_active
			and self.last_BehaviorTreeNode != "robot-go-charge"
		):
			# Inactief: geen nieuwe goals sturen (alleen naar het laadstation mag).
			pass
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
			mode = "aan" if state else "uit"
			self.get_logger().info(f"Keepoutfilter omgeschakeld: {mode}")
	
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

	@staticmethod
	def same_pose(a, b, pos_tol=0.05, dot_tol=1e-3):
		"""Inhoudelijke vergelijking van twee PoseStamped (stamp wordt genegeerd)."""
		pa, pb = a.pose.position, b.pose.position
		qa, qb = a.pose.orientation, b.pose.orientation
		if a.header.frame_id != b.header.frame_id:
			return False
		if abs(pa.x - pb.x) > pos_tol or abs(pa.y - pb.y) > pos_tol or abs(pa.z - pb.z) > pos_tol:
			return False
		# q en -q beschrijven dezelfde oriëntatie
		dot = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w
		return abs(dot) >= 1.0 - dot_tol

	def active_record(self):
		"""Het goal dat nu verstuurd/actief is (nog geen resultaat of afwijzing)."""
		return self.current_goals[-1] if self.current_goals else None

	def finish_record(self, record):
		if record in self.current_goals:
			self.current_goals.remove(record)
		self.last_goal_end_time = self.get_clock().now()
		self.last_finished_pose = record.goal.pose

	def send_goal(self, source, coordinate=None):
		if coordinate is None:
			msg = f"Geen coördinaat ontvangen van {source} — goal niet gestuurd!"
			self.get_logger().warn(msg)
			self.publish_status(12, msg)
			return

		now = self.get_clock().now()
		active = self.active_record()

		if active is not None:
			# Zelfde bestemming nog actief: behouden, niet opnieuw sturen.
			if self.same_pose(active.goal.pose, coordinate):
				return
			# Echt nieuwe bestemming: vervangt het bestaande goal (Nav2 preempt).
			self.get_logger().info(f"Nieuwe bestemming van {source}: bestaand goal wordt vervangen.")
			self.current_goals.clear()
		elif (
			self.last_goal_end_time is not None
			and self.last_finished_pose is not None
			and self.same_pose(self.last_finished_pose, coordinate)
		):
			# Zelfde bestemming als het net afgeronde goal: korte pauze tegen een zend-storm.
			# Een andere bestemming wordt meteen verstuurd.
			elapsed = (now - self.last_goal_end_time).nanoseconds / 1e9
			if elapsed < self.RESEND_COOLDOWN_S:
				return

		goal_msg = NavigateToPose.Goal()
		goal_msg.pose = coordinate

		new_record = GoalRecord(source=source, goal=goal_msg)
		self.current_goals.append(new_record)

		self.publish_status(13, "goal verzonden")

		future = self._action_client.send_goal_async(new_record.goal)
		future.add_done_callback(
			lambda f: self.goal_response_callback(f, new_record)
		)

	# --- CALLBACKS (Nav_cb_group) ---
	def goal_response_callback(self, future, record):
		with self.lock:
			try:
				handle = future.result()
			except Exception as e:
				self.get_logger().error(f"Goal versturen mislukt: {e}")
				self.finish_record(record)
				return

			record.handle = handle
			record.accepted = handle.accepted

			if not record.accepted:
				self.publish_status(10, "goal afgewezen")
				self.get_logger().warn("Goal NIET geaccepteerd door Nav2! active goal vrijgegeven.")
				record.status = 5
				self.finish_record(record)
				return

			# Intussen vervangen of geannuleerd: meteen alsnog annuleren.
			if record not in self.current_goals:
				self.get_logger().info("Geaccepteerd goal is al vervangen/geannuleerd: annuleren.")
				try:
					handle.cancel_goal_async()
				except Exception as e:
					self.get_logger().warn(f"Fout bij annuleren goal: {e}")
				return

			self.get_logger().info("Goal geaccepteerd ✅")
			self.publish_status(15, "goal geaccepteerd")
			record.status = 2

			result_future = handle.get_result_async()
			result_future.add_done_callback(
				lambda f: self.result_callback(f, record)
			)

	def result_callback(self, future, record):
		with self.lock:
			try:
				result_handle = future.result()
			except Exception as e:
				self.get_logger().error(f"Resultaat ophalen mislukt: {e}")
				self.finish_record(record)
				return

			record.result = result_handle.result
			record.status = result_handle.status

			# Vervangen of geannuleerd goal (niet meer in de lijst): negeren.
			if record not in self.current_goals:
				self.get_logger().info(
					f"Resultaat van oud doel genegeerd (status {record.status:02d})."
				)
				return

			self.publish_status(record.status, "NAV")
			self.get_logger().info(f"Goal afgerond, Nav2-status: {record.status:02d}")
			# Afgehandeld record verwijderen: lijst blijft kort, nieuw goal mag.
			self.finish_record(record)

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
