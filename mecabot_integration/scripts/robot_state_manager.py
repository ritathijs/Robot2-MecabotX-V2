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
    admin_login_closed: bool = False
    network_ok: bool = True
    battery_ok: bool = True
    docked: bool = False
    charging: bool = False
    visitors_nearby: bool = False
    in_working_zone: bool = False
    manual_drive: bool = False
    estop_active: bool = False
    last_screen: str | None = None
    # BEST PRACTICE TOEVOEGING:
    battery_voltage: float = 24.0  # Startwaarde/fallback spanning

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
        self.get_logger().info("RobotStateManager gestart (Gecorrigeerd & Validatie Actief).")

    # ---------------------------
    # STATE UPDATE CALLBACKS
    # ---------------------------
    def on_battery(self, msg: Float32):
        with self.lock:
            # Sla de spanning live op in de centrale toestand
            self.state.battery_voltage = float(msg.data)
            # Grenswaarde bijvoorbeeld 22.0V
            self.state.battery_ok = msg.data >= 22.0

    def on_behavior_node_changed(self, msg: String):
        screen = msg.data
        if not screen or screen == "None":
            return
        with self.lock:
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
        status_text = msg.data.strip()
        
        # Als de driver meldt dat er GEEN botsing is, doen we niks en verlaten we de functie
        if status_text == "De bumper is niet ingedrukt":
            return
            
        # In alle andere gevallen is er een bumper ECHT ingedrukt!
        # De ESTOP wordt geactiveerd en de robot stopt direct.
        self.activate_estop(f"bumper_trigger ({status_text})")

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
    # SOCKET EVENTS (Gecorrigeerd voor naamconflict crashes)
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
            bridge = data
            with self.lock:
                current_active = self.state.robot_active
            if bridge:
                bridge.send_active_status_to_pi(current_active)
        elif event == "active_button_toggled":
            # data bevat een tuple: (socket_data, bridge-referentie)
            if data is not None and isinstance(data, tuple):
                _, bridge = data  # We negeren de socket_data van de Pi volledig!
                
                with self.lock:
                    # Best Practice: Draai de huidige status blindelings om (not)
                    self.state.robot_active = not self.state.robot_active
                    new_status = self.state.robot_active
                
                self.get_logger().info(f"🔄 Knop ingedrukt op Pi! Status omgekeerd naar: {new_status}")
                
                # We sturen de nieuwe status direct terug zodat de knop live van kleur verandert!
                if bridge:
                    bridge.send_active_status_to_pi(new_status)
        elif event == "admin_login_open":
            self._handle_admin_login_open()
        elif event == "admin_login_closed":
            # GEWIJZIGD: We overschrijven hier geen methodenamen meer!
            with self.lock:
                self.state.admin_login_open = False
        elif event == "admin_open":
            self._handle_admin_panel_open()
        elif event == "admin_closed" or event == "admin_login_closed":

            with self.lock:
                self.state.admin_open = False
                self.state.admin_login_open = False
                # Haal direct en in alle situaties het laatst bekende scherm op
                saved_last_screen = self.state.last_screen

            self.get_logger().info("🔒 Admin panel gesloten. Vorige schermtoestand blindelings herstellen...")
            
            msg = String()
            
            # Controleer of er een vorig scherm is opgeslagen
            if saved_last_screen:
                msg.data = saved_last_screen
                self.get_logger().info(f"🔄 Scherm hersteld naar: {saved_last_screen}")
            else:
                # Alleen als er echt geen vorig scherm bekend is (None), sturen we de startup
                # RT TO DO: Hier is het misschien best om de status van de robot te checken en dan te beslissen welk scherm we tonen.
                msg.data = "robot-startup"
                self.get_logger().info("🤖 Geen vorig scherm bekend -> Fallback naar robot-startup")
                
            self.screen_pub.publish(msg)
            # ==========================================

        elif event == "battery_request":
            self.get_logger().info("📬 Batterij-aanvraag ontvangen! Live pulse berekenen...")
            with self.lock:
                bat_ok = self.state.battery_ok
                v = self.state.battery_voltage  # Dit is de live float, bijv. 23.74
                self.get_logger().info(f"🔋 [STATE] Live accu-info verzenden -> Status OK: {bat_ok} | Spanning: {v:.2f}V")

            # We sturen de pure status en de ruwe float-waarde mee in de string, gescheiden door een dubbelepunt
            msg = String()
            msg.data = f"BATTERY_PULSE:{bat_ok}:{v}"
            self.screen_pub.publish(msg)
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
    # LOGIN / ADMIN FLOW (GEFIXT!)
    # ---------------------------
    def _handle_admin_login_open(self):
        with self.lock:
            self.state.admin_login_open = True
        self.send_robot_command("STOP")
        self._publish_zero_twist()

    def _handle_admin_panel_open(self):
        with self.lock:
            self.state.admin_login_open = False
            self.state.admin_open = True
        self.send_robot_command("STOP")
        self._publish_zero_twist()

    # ---------------------------
    # SCHERMLOGICA (GECORRIGEERD: Geen dubbele locks & Actieve Robot-Validatie)
    # ---------------------------
    def handle_screen_request(self, data):
        if data is None:
            return
            
        # Veiligheid: Pak de schermnaam, of de data nu een dict of een platte string is
        if isinstance(data, dict):
            screen = data.get("screen")
        else:
            screen = str(data)
            
        if not screen or screen == "None":
            return

        # Haal alle benodigde variabelen in één keer veilig op uit de lock (voorkomt deadlocks)
        with self.lock:
            admin_open = self.state.admin_open
            admin_login_open = self.state.admin_login_open

        # BLOKKADE-STAP: Als de admin openstaat of men is bezig met inloggen -> Geen schermwissel!
        if admin_open or admin_login_open:
            self.get_logger().info(f"⏸️ Schermverzoek '{screen}' gepauzeerd: Admin of Login is geopend.")
            return

        # ALLES IN ORDE ROUTE: In alle andere gevallen accepteren we het scherm blindelings,
        # of de robot nu slaapt, rijdt of laadt (de Behavior Tree regelt de juiste pagina).
        with self.lock:
            self.state.last_screen = screen

        msg = String()
        msg.data = screen
        self.screen_pub.publish(msg)
        self.get_logger().info(f"✅ Schermwissel goedgekeurd door RobotStateManager: {screen}")
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

    # ============================================================
    # ESTOP (GECORRIGEERD: Voorkomt rclpy InvalidHandle crashes)
    # ============================================================
    def activate_estop(self, reason: str):
        with self.lock:
            # Als de noodstop nog niet actief was, loggen we het één keer
            if not self.state.estop_active:
                self.get_logger().warn(f"ESTOP geactiveerd wegens: {reason}")
            self.state.estop_active = True
            
        # Verschuif de eindtijd met 30 seconden in de toekomst (veilig zonder destroy_timer)
        self.estop_end_time = time.time() + 30.0
        
        # Start de timer alleen de allereerste keer op als hij nog niet bestaat
        if self.estop_timer is None:
            self.estop_timer = self.create_timer(0.05, self.estop_tick)

    def estop_tick(self):
        # Controleer of de stoptijd is verstreken
        if time.time() >= self.estop_end_time:
            with self.lock:
                self.state.estop_active = False
            self.get_logger().info("ESTOP periode verstreken. Robot vrijgegeven.")
            
            # We annuleren de timer niet hard, we stoppen gewoon de sturing
            if self.estop_timer is not None:
                self.destroy_timer(self.estop_timer)
                self.estop_timer = None
            return

        # Zolang de ESTOP actief is, dwingen we stilstand af
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
        
    def send_battery_percentage(self):
        """Meldt aan de bridge dat het batterijpercentage (voltage) verstuurd moet worden."""
        msg = String()
        msg.data = "robot-update-battery-percentage"
        
        self.get_logger().info("🔋 RobotStateManager: Aanvraag voor robot-update-battery-percentage doorgezet naar bridge.")
        self.screen_pub.publish(msg)
        self.get_logger().info(f"🔋 Ruwe batterijspanning doorgegeven naar bridge: {v:.2f}V")

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
