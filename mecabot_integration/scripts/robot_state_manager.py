#!/usr/bin/env python3
from dataclasses import dataclass
from threading import Lock
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
from wheeltec_robot_msg.srv import GetRobotState

@dataclass
class RobotState:
    robot_active: bool = True
    quiz_active: bool = False
    quiz_finished: bool = False
    admin_open: bool = False
    admin_login_open: bool = False
    network_ok: bool = True
    battery_ok: bool = True
    docked: bool = False
    charging: bool = False
    visitors_nearby: bool = False
    in_working_zone: bool = False
    manual_drive: bool = False
    estop_active: bool = False
    last_screen: str | None = None

class RobotStateManager(Node):
    def __init__(self):
        super().__init__("robot_state_manager")
        self.state = RobotState()
        self.lock = Lock()
        self.last_drive_time = None
        self.drive_timeout_sec = 0.4 # 400 ms → beveiliging voor manual drive

        # Publishers
        self.screen_pub = self.create_publisher(String, "/screen_command", 10)
        self.cmd_pub = self.create_publisher(String, "/robot_command", 10)
        self.drive_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriptions
        self.create_subscription(Float32, "/BatteryAverageVoltage", self.on_battery, 10)
        self.create_subscription(String, "/bump_status", self.on_bumper, 10)
        self.create_subscription(Bool, "/visitors_nearby", self.on_visitors, 10)
        self.create_subscription(Bool, "/in_working_zone", self.on_working_zone, 10)
        self.create_subscription(Bool, "/dock_state", self.on_docked, 10)
        self.create_subscription(String, "/BehaviorTreeNode", self.on_behavior_node_changed, 10)

        # Service
        self.create_service(GetRobotState, "/get_robot_state", self.handle_state_request)

        # ESTOP timer
        self.estop_timer = None
        self.estop_end_time = None

        # Manual drive timeout timer
        self.drive_timeout_timer = self.create_timer(0.1, self._manual_drive_timeout_check)
        self.get_logger().info("RobotStateManager gestart (Gecorrigeerd).")

    # ---------------------------
    # STATE UPDATE CALLBACKS
    # ---------------------------
    def on_battery(self, msg: Float32):
        with self.lock:
            self.state.battery_ok = msg.data >= 22.0
    def on_behavior_node_changed(self, msg: String):
        screen = msg.data
        if not screen or screen == "None":
            return

        with self.lock:
            robot_active = self.state.robot_active
            admin_open = self.state.admin_open

        # Als de admin open is, mag het scherm niet zomaar veranderen
        if admin_open:
            return

        # Sla op en stuur het scherm dwingend naar de Pi!
        with self.lock:
            self.state.last_screen = screen
            
        screen_msg = String()
        screen_msg.data = screen
        self.screen_pub.publish(screen_msg)
        self.get_logger().info(f"Onafhankelijke schermwissel doorgezet naar Pi: {screen}")

    def on_bumper(self, msg: String):
        self.activate_estop("bumper")

    def on_visitors(self, msg: Bool):
        with self.lock:
            self.state.visitors_nearby = msg.data

    def on_working_zone(self, msg: Bool):
        with self.lock:
            self.state.in_working_zone = msg.data

    def on_docked(self, msg: Bool):
        with self.lock:
            self.state.docked = msg.data

    # ---------------------------
    # SOCKET EVENTS (GECORRIGEERD: Lock verwijderd rondom routing)
    # ---------------------------
    def handle_socket_event(self, event: str, data=None):
        if event == "quiz_finished":
            with self.lock:
                self.state.quiz_active = False
                self.state.quiz_finished = True
        elif event == "quiz_inactive":
            with self.lock:
                self.state.quiz_active = False
        elif event == "drive_to_quiz_location":
            self.send_robot_command("DRIVE_QUIZ_LOCATION")
        elif event == "screen_request":
            self.handle_screen_request(data)
        elif event == "ask_is_active":
            # data bevat hier de bridge-referentie (gegeven vanuit de timer)
            bridge = data
            with self.lock:
                current_active = self.state.robot_active
            
            # VUUR DE STATUS AF: De Orin Nano geeft direct antwoord!
            if bridge:
                bridge.send_active_status_to_pi(current_active)

        elif event == "active_button_toggled":
            # data bevat hier een tuple: (echte_data, bridge-referentie)
            if data is not None and isinstance(data, tuple):
                socket_data, bridge = data
                is_active = bool(socket_data.get("active", True))
                
                with self.lock:
                    self.state.robot_active = is_active
                self.get_logger().info(f"🔄 Robot actief status handmatig omgezet naar: {is_active}")
                
                # Bevestig direct terug aan de Pi zodat de knop live van kleur verandert!
                if bridge:
                    bridge.send_active_status_to_pi(is_active)
        elif event == "admin_login_open":
            self._handle_admin_login_open()
        elif event == "admin_login_closed":
            with self.lock:
                self.state.admin_login_open = False
                self.state.admin_login_closed = True
        elif event == "admin_open":
            self._handle_admin_panel_open()
        elif event == "admin_closed":
            with self.lock:
                self.state.admin_open = False
                self.state.admin_login_open = False
        elif event == "battery_request":
            self.send_battery_status()
        elif event == "manual_drive":
            self.handle_manual_drive(data)
        elif event == "time_updated":
            pass
        elif event == "connect":
            with self.lock:
                self.state.network_ok = True
        elif event == "disconnect":
            with self.lock:
                self.state.network_ok = False

    # ---------------------------
    # LOGIN / ADMIN FLOW
    # ---------------------------
    def _handle_admin_login_open(self):
        with self.lock:
            self.state.admin_login_open = True
            self.state.admin_login_closed = False
        self.send_robot_command("STOP")
        self._publish_zero_twist()
        
    def _handle_admin_panel_open(self):
        with self.lock:
            self.state.admin_login_open = False
            self.state.admin_open = True
        self.send_robot_command("STOP")
        self._publish_zero_twist()

    # ---------------------------
    # SCHERMLOGICA (GECORRIGEERD: Risicovolle blokkade verwijderd)
    # ---------------------------
    def handle_screen_request(self, data):
        if data is None:
            return
        screen = data.get("screen")
        if screen is None:
            return

        with self.lock:
            robot_active = self.state.robot_active
            admin_open = self.state.admin_open

        msg = String()
        
        # Als de robot inactief is of de admin is open, tonen we het wachtscherm
        if not robot_active or admin_open:
            msg.data = "QUIZ_WAIT"
            self.screen_pub.publish(msg)
            return

        # Sla het nieuwe scherm op als het laatst getoonde scherm
        with self.lock:
            self.state.last_screen = screen
            
        # Stuur het scherm direct door naar de Pi
        msg.data = screen
        self.screen_pub.publish(msg)

    # ---------------------------
    # MANUAL DRIVE
    # ---------------------------
    def handle_manual_drive(self, data):
        with self.lock:
            admin_open = self.state.admin_open
            estop = self.state.estop_active
            docked = self.state.docked
            robot_active = self.state.robot_active

        if not admin_open or estop or docked or not robot_active:
            return

        self._execute_manual_drive(data)

    def _execute_manual_drive(self, data):
        if data is None:
            return
        self.last_drive_time = time.time()
        twist = Twist()
        try:
            twist.linear.x = float(data.get("x", 0.0))
            twist.linear.y = float(data.get("y", 0.0))
            twist.angular.z = float(data.get("rotation", 0.0))
            self.drive_pub.publish(twist)
        except Exception as e:
            self.get_logger().error(f"Fout bij publiceren manual drive: {e}")

    def _manual_drive_timeout_check(self):
        if self.last_drive_time is None:
            return
        elapsed = time.time() - self.last_drive_time
        if elapsed > self.drive_timeout_sec:
            self._publish_zero_twist()
            self.send_robot_command("STOP")
            self.last_drive_time = None

    # ---------------------------
    # ESTOP
    # ---------------------------
    def activate_estop(self, reason: str):
        self.get_logger().warn(f"ESTOP geactiveerd: {reason}")
        with self.lock:
            self.state.estop_active = True
        self.start_estop_period(30000)

    def start_estop_period(self, ms: int):
        sec = ms / 1000.0
        self.estop_end_time = time.time() + sec
        if self.estop_timer is not None:
            try:
                self.destroy_timer(self.estop_timer)
            except Exception:
                pass
        self.estop_timer = self.create_timer(0.05, self.estop_tick)

    def estop_tick(self):
        if time.time() >= self.estop_end_time:
            if self.estop_timer is not None:
                self.destroy_timer(self.estop_timer) # GECORRIGEERD: Geen .cancel() crash meer
                self.estop_timer = None
            with self.lock:
                self.state.estop_active = False
            return
        
        self._publish_zero_twist()
        self.send_robot_command("STOP")

    # ---------------------------
    # BATTERY
    # ---------------------------
    def send_battery_status(self):
        with self.lock:
            bat_ok = self.state.battery_ok
        msg = String()
        msg.data = f"BATTERY:{bat_ok}"
        self.screen_pub.publish(msg)

    # ---------------------------
    # SERVICE VOOR BT
    # ---------------------------
    def handle_state_request(self, request, response):
        with self.lock:
            response.robot_active = self.state.robot_active
            response.quiz_active = self.state.quiz_active
            response.quiz_finished = self.state.quiz_finished
            response.admin_open = self.state.admin_open
            response.network_ok = self.state.network_ok
            response.battery_ok = self.state.battery_ok
            response.docked = self.state.docked
            response.charging = self.state.charging
            response.visitors_nearby = self.state.visitors_nearby
            response.in_working_zone = self.state.in_working_zone
        return response

    # ---------------------------
    # HELPERS
    # ---------------------------
    def send_robot_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

    def _publish_zero_twist(self):
        twist = Twist()
        self.drive_pub.publish(twist)
