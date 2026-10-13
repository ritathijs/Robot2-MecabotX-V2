#!/usr/bin/env python3

from dataclasses import dataclass
from threading import Lock
import math
import time
from typing import Any

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
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
    battery_voltage: float = 24.0

    docked: bool = False
    at_station: bool = False
    charging: bool = False

    visitors_nearby: bool = False
    in_working_zone: bool = False
    manual_drive: bool = False
    estop_active: bool = False

    localization_valid: bool = False
    received_amcl: bool = False

    at_quiz: bool = False
    at_work_area: bool = False
    active_destination: str | None = None

    last_screen: str | None = None


class RobotStateManager(Node):
    BATTERY_OK_THRESHOLD = 22.0
    DRIVE_TIMEOUT_SEC = 0.4
    ESTOP_DURATION_SEC = 30.0

    # Koppel de namen die door de controller/BT gebruikt worden aan een
    # bestemming. De naam "follow-robot-screen" wordt door de huidige
    # controller voor beide quizlocaties gebruikt.
    DESTINATION_BY_NODE = {
        "DriveQuizLocation": "quiz",
        "FallbackDriveQuizLocation": "quiz",
        "follow-robot-screen": "quiz",
        "robot-arrived-at-quiz-location": "quiz",

        "DriveWorkArea": "work_area",
        "FallbackDriveToWorkArea": "work_area",
        "robot-startup": "work_area",

        "RobotDriveToChargingStation": "charging_station",
        "FallbackDriveToChargingStation": "charging_station",
        "robot-go-charge": "charging_station",
    }

    def __init__(self):
        super().__init__("robot_state_manager")

        self.state = RobotState()
        self.lock = Lock()

        self.last_drive_time: float | None = None
        self.estop_end_time: float | None = None
        self.estop_timer = None

        # Uitgaande topics
        self.screen_pub = self.create_publisher(
            String, "/screen_command", 10
        )
        self.cmd_pub = self.create_publisher(
            String, "/robot_command", 10
        )
        self.drive_pub = self.create_publisher(
            Twist, "/cmd_vel", 10
        )

        # Statusoutputs voor de Behavior Tree
        self.localization_pub = self.create_publisher(
            Bool, "/IsLocalizationValid", 10
        )
        self.quiz_pub = self.create_publisher(
            Bool, "/IsRobotAtQuiz", 10
        )
        self.work_pub = self.create_publisher(
            Bool, "/IsRobotAtWorkArea", 10
        )
        self.charging_station_pub = self.create_publisher(
            Bool, "/IsRobotNearChargingStation", 10
        )
        # Compatibiliteit met de oude topicnaam uit status_reader.py.
        self.charging_station_legacy_pub = self.create_publisher(
            Bool, "/IsRobotAtChargingStation", 10
        )
        self.charging_pub = self.create_publisher(
            Bool, "/IsRobotCharging", 10
        )
        self.robot_active_pub = self.create_publisher(
            Bool, "/IsRobotActive", 10
        )
        self.battery_ok_pub = self.create_publisher(
            Bool, "/IsBatteryOK", 10
        )
        self.battery_low_pub = self.create_publisher(
            Bool, "/IsBatteryLow", 10
        )

        # Sensor-, status- en feedbacktopics
        self.create_subscription(
            Float32,
            "/BatteryAverageVoltage",
            self.on_battery,
            10,
        )
        self.create_subscription(
            String,
            "/bump_status",
            self.on_bumper,
            10,
        )
        self.create_subscription(
            Bool,
            "/visitors_nearby",
            self.on_visitors,
            10,
        )
        self.create_subscription(
            Bool,
            "/in_working_zone",
            self.on_working_zone,
            10,
        )
        self.create_subscription(
            Bool,
            "/dock_state",
            self.on_docked,
            10,
        )
        self.create_subscription(
            String,
            "/BehaviorTreeNode",
            self.on_behavior_node_changed,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.on_amcl,
            10,
        )
        self.create_subscription(
            String,
            "/reset_done",
            self.on_localization_reset,
            10,
        )
        self.create_subscription(
            String,
            "/auto_recharge_event",
            self.on_recharge_event,
            10,
        )
        self.create_subscription(
            Bool,
            "/RobotActive",
            self.on_robot_active,
            10,
        )
        self.create_subscription(
            String,
            "/drive_to_coord_status",
            self.on_navigation_status,
            10,
        )

        # Service voor het opvragen van de actuele robotstatus
        self.create_service(
            GetRobotState,
            "/get_robot_state",
            self.handle_state_request,
        )

        self.drive_timeout_timer = self.create_timer(
            0.1,
            self._manual_drive_timeout_check,
        )

        self._publish_status_snapshot()
        self.get_logger().info("RobotStateManager gestart.")

    # ------------------------------------------------------------------
    # Algemene helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _bool_message(value: bool) -> Bool:
        msg = Bool()
        msg.data = bool(value)
        return msg

    def _publish_status_snapshot(self) -> None:
        with self.lock:
            localization_valid = self.state.localization_valid
            at_quiz = self.state.at_quiz
            at_work_area = self.state.at_work_area
            at_station = self.state.at_station
            charging = self.state.charging
            robot_active = self.state.robot_active
            battery_ok = self.state.battery_ok

        self.localization_pub.publish(
            self._bool_message(localization_valid)
        )
        self.quiz_pub.publish(self._bool_message(at_quiz))
        self.work_pub.publish(self._bool_message(at_work_area))

        station_msg = self._bool_message(at_station)
        self.charging_station_pub.publish(station_msg)
        self.charging_station_legacy_pub.publish(station_msg)

        self.charging_pub.publish(self._bool_message(charging))
        self.robot_active_pub.publish(self._bool_message(robot_active))
        self.battery_ok_pub.publish(self._bool_message(battery_ok))
        self.battery_low_pub.publish(self._bool_message(not battery_ok))

    def send_robot_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)

    def _publish_zero_twist(self) -> None:
        self.drive_pub.publish(Twist())

    # ------------------------------------------------------------------
    # Sensor- en statuscallbacks
    # ------------------------------------------------------------------

    def on_battery(self, msg: Float32) -> None:
        voltage = float(msg.data)

        with self.lock:
            self.state.battery_voltage = voltage
            self.state.battery_ok = voltage >= self.BATTERY_OK_THRESHOLD

        self._publish_status_snapshot()

    def on_visitors(self, msg: Bool) -> None:
        with self.lock:
            self.state.visitors_nearby = msg.data

    def on_working_zone(self, msg: Bool) -> None:
        with self.lock:
            self.state.in_working_zone = msg.data

    def on_docked(self, msg: Bool) -> None:
        with self.lock:
            self.state.docked = msg.data

    def on_robot_active(self, msg: Bool) -> None:
        with self.lock:
            self.state.robot_active = msg.data

        self._publish_status_snapshot()

    def on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        covariance = msg.pose.covariance
        cov_x = covariance[0]
        cov_y = covariance[7]
        cov_yaw = covariance[35]

        amcl_good = (
            cov_x < 0.1
            and cov_y < 0.1
            and cov_yaw < 0.1
        )

        with self.lock:
            self.state.received_amcl = True
            self.state.localization_valid = (
                amcl_good
                or self.state.at_station
                or self.state.charging
            )
            localization_valid = self.state.localization_valid

        self._publish_status_snapshot()
        self.get_logger().info(
            "[Localization] AMCL covariance "
            f"({cov_x:.3f}, {cov_y:.3f}, {cov_yaw:.3f}); "
            f"valid={localization_valid}"
        )

    def on_localization_reset(self, msg: String) -> None:
        if msg.data.strip().lower() != "done":
            return

        with self.lock:
            self.state.localization_valid = True

        self._publish_status_snapshot()
        self.get_logger().info(
            "[Localization] Reset bevestigd; lokalisatie geldig."
        )

    def on_recharge_event(self, msg: String) -> None:
        event = msg.data
        self.get_logger().info(f"[Docking] Event ontvangen: {event}")

        with self.lock:
            if "DRIVE-TO-DOCK-SUCCESS" in event:
                self.state.at_station = True
                self.state.localization_valid = True

            elif "ROBOT-CHARGING" in event:
                self.state.charging = True
                self.state.at_station = True
                self.state.docked = True
                self.state.robot_active = False
                self.state.localization_valid = True

            elif "DOCK-FAILED" in event:
                self.state.at_station = False
                self.state.docked = False
                self.state.charging = False

        self._publish_status_snapshot()

    def on_behavior_node_changed(self, msg: String) -> None:
        node_name = msg.data.strip()
        if not node_name or node_name == "None":
            return

        destination = self.DESTINATION_BY_NODE.get(node_name)

        with self.lock:
            admin_open = self.state.admin_open
            self.state.active_destination = destination

            if destination == "quiz":
                self.state.at_quiz = False
            elif destination == "work_area":
                self.state.at_work_area = False
            elif destination == "charging_station":
                self.state.at_station = False

        # In de huidige architectuur wordt dit topic ook gebruikt om
        # schermwissels naar de Pi door te geven. Dit behoudt dat gedrag.
        if not admin_open:
            with self.lock:
                self.state.last_screen = node_name

            screen_msg = String()
            screen_msg.data = node_name
            self.screen_pub.publish(screen_msg)

        self._publish_status_snapshot()
        self.get_logger().info(
            f"[Robot] Node ontvangen: {node_name}; "
            f"bestemming={destination}"
        )

    def on_navigation_status(self, msg: String) -> None:
        raw = msg.data.strip()
        status_text, separator, _rest = raw.partition("-")

        if not separator:
            self.get_logger().error(
                f"[Nav2] Ongeldig statusbericht ontvangen: {raw!r}"
            )
            return

        try:
            status_code = int(status_text)
        except ValueError:
            self.get_logger().error(
                f"[Nav2] Statuscode niet te lezen uit: {raw!r}"
            )
            return

        # Nav2 GoalStatus: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED.
        if status_code not in (4, 5, 6):
            return

        succeeded = status_code == 4

        with self.lock:
            destination = self.state.active_destination

            if destination == "quiz":
                self.state.at_quiz = succeeded
            elif destination == "work_area":
                self.state.at_work_area = succeeded
            elif destination == "charging_station":
                self.state.at_station = succeeded
            else:
                self.get_logger().warning(
                    "[Nav2] Resultaat ontvangen zonder bekende "
                    "actieve bestemming; status niet aan een locatie gekoppeld."
                )
                return

        self._publish_status_snapshot()
        self.get_logger().info(
            f"[Nav2] Goal afgerond: status={status_code}, "
            f"bestemming={destination}, geslaagd={succeeded}"
        )

    # ------------------------------------------------------------------
    # Bumper en noodstop
    # ------------------------------------------------------------------

    def on_bumper(self, msg: String) -> None:
        status_text = msg.data.strip()

        if status_text == "De bumper is niet ingedrukt":
            return

        self.activate_estop(f"bumper_trigger ({status_text})")

    def activate_estop(self, reason: str) -> None:
        with self.lock:
            if not self.state.estop_active:
                self.get_logger().warning(
                    f"ESTOP geactiveerd: {reason}"
                )
            self.state.estop_active = True

        self.estop_end_time = (
            time.monotonic() + self.ESTOP_DURATION_SEC
        )

        if self.estop_timer is None:
            self.estop_timer = self.create_timer(
                0.05,
                self._estop_tick,
            )

    def _estop_tick(self) -> None:
        if self.estop_end_time is not None:
            if time.monotonic() >= self.estop_end_time:
                with self.lock:
                    self.state.estop_active = False

                self.get_logger().info(
                    "ESTOP-timer verlopen; softwarematige ESTOP vrijgegeven."
                )

                if self.estop_timer is not None:
                    self.destroy_timer(self.estop_timer)
                    self.estop_timer = None

                self.estop_end_time = None
                return

        self._publish_zero_twist()
        self.send_robot_command("STOP")

    # ------------------------------------------------------------------
    # Socket-events vanuit QuizSocketBridge
    # ------------------------------------------------------------------

    def handle_socket_event(self, event: str, data: Any = None) -> None:
        if event == "quiz_finished":
            with self.lock:
                self.state.quiz_active = False
                self.state.quiz_finished = True

        elif event == "quiz_inactive":
            with self.lock:
                self.state.quiz_active = False

        elif event == "drive_to_quiz_location":
            # De huidige robot_controller herkent DRIVE_QUIZ_LOCATION_A/B.
            # Kies hier niet willekeurig een locatie; pas dit event of de
            # controller aan om een expliciete A- of B-keuze te gebruiken.
            self.get_logger().warning(
                "Event drive_to_quiz_location ontvangen, maar er is "
                "geen expliciete quizlocatie A/B opgegeven."
            )

        elif event == "screen_request":
            self.handle_screen_request(data)

        elif event == "ask_is_active":
            bridge = data
            with self.lock:
                current_active = self.state.robot_active

            if bridge is not None:
                bridge.send_active_status_to_pi(current_active)

        elif event == "active_button_toggled":
            if not isinstance(data, tuple) or len(data) != 2:
                self.get_logger().warning(
                    "Ongeldige payload voor active_button_toggled."
                )
                return

            _socket_data, bridge = data

            with self.lock:
                self.state.robot_active = not self.state.robot_active
                new_status = self.state.robot_active

            self._publish_status_snapshot()
            self.get_logger().info(
                f"Robot actief-status gewijzigd naar {new_status}."
            )

            if bridge is not None:
                bridge.send_active_status_to_pi(new_status)

        elif event == "admin_login_open":
            self._handle_admin_login_open()

        elif event == "admin_open":
            self._handle_admin_panel_open()

        elif event in ("admin_closed", "admin_login_closed"):
            self._handle_admin_closed()

        elif event == "battery_request":
            self._publish_battery_pulse()

        elif event == "manual_drive":
            self.handle_manual_drive(data)

        elif event == "connect":
            with self.lock:
                self.state.network_ok = True

        elif event == "disconnect":
            with self.lock:
                self.state.network_ok = False

        elif event == "time_updated":
            return

        else:
            self.get_logger().debug(
                f"Socket-event genegeerd: {event}"
            )

    def _handle_admin_login_open(self) -> None:
        with self.lock:
            self.state.admin_login_open = True

        self.send_robot_command("STOP")
        self._publish_zero_twist()

    def _handle_admin_panel_open(self) -> None:
        with self.lock:
            self.state.admin_login_open = False
            self.state.admin_open = True

        self.send_robot_command("STOP")
        self._publish_zero_twist()

    def _handle_admin_closed(self) -> None:
        with self.lock:
            self.state.admin_open = False
            self.state.admin_login_open = False
            saved_screen = self.state.last_screen

        screen_msg = String()
        screen_msg.data = saved_screen or "robot-startup"
        self.screen_pub.publish(screen_msg)

    def handle_screen_request(self, data: Any) -> None:
        if data is None:
            return

        if isinstance(data, dict):
            screen = data.get("screen")
        else:
            screen = str(data)

        if not isinstance(screen, str):
            self.get_logger().warning(
                "Schermverzoek bevat geen geldige schermnaam."
            )
            return

        screen = screen.strip()
        if not screen or screen == "None":
            return

        with self.lock:
            if self.state.admin_open or self.state.admin_login_open:
                self.get_logger().info(
                    f"Schermverzoek {screen!r} genegeerd: admin/login is open."
                )
                return

            self.state.last_screen = screen

        screen_msg = String()
        screen_msg.data = screen
        self.screen_pub.publish(screen_msg)

    # ------------------------------------------------------------------
    # Handmatige besturing
    # ------------------------------------------------------------------

    def handle_manual_drive(self, data: Any) -> None:
        with self.lock:
            allowed = (
                self.state.admin_open
                and not self.state.estop_active
                and not self.state.docked
                and self.state.robot_active
            )

        if not allowed:
            self.get_logger().warning(
                "Handmatige besturing geweigerd door robotstatus."
            )
            return

        self._execute_manual_drive(data)

    def _execute_manual_drive(self, data: Any) -> None:
        if not isinstance(data, dict):
            self.get_logger().warning(
                "Handmatig rijcommando moet een object/dictionary zijn."
            )
            return

        try:
            linear_x = float(data.get("x", 0.0))
            linear_y = float(data.get("y", 0.0))
            angular_z = float(data.get("rotation", 0.0))
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                f"Ongeldige snelheid in handmatig rijcommando: {exc}"
            )
            self._publish_zero_twist()
            return

        values = (linear_x, linear_y, angular_z)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error(
                "Handmatig rijcommando bevat een niet-eindige snelheid."
            )
            self._publish_zero_twist()
            return

        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z

        self.drive_pub.publish(twist)
        self.last_drive_time = time.monotonic()

    def _manual_drive_timeout_check(self) -> None:
        if self.last_drive_time is None:
            return

        if time.monotonic() - self.last_drive_time > self.DRIVE_TIMEOUT_SEC:
            self.get_logger().info(
                "Handmatige besturing gestopt wegens timeout."
            )
            self._publish_zero_twist()
            self.send_robot_command("STOP")
            self.last_drive_time = None

    # ------------------------------------------------------------------
    # Batterijcommunicatie met QuizSocketBridge
    # ------------------------------------------------------------------

    def _publish_battery_pulse(self) -> None:
        with self.lock:
            battery_ok = self.state.battery_ok
            voltage = self.state.battery_voltage

        msg = String()
        msg.data = f"BATTERY_PULSE:{battery_ok}:{voltage}"
        self.screen_pub.publish(msg)

        self.get_logger().info(
            f"Batterijstatus verzonden: ok={battery_ok}, "
            f"spanning={voltage:.2f} V"
        )

    def send_battery_status(self) -> None:
        with self.lock:
            battery_ok = self.state.battery_ok

        msg = String()
        msg.data = f"BATTERY:{battery_ok}"
        self.screen_pub.publish(msg)

    def send_battery_percentage(self) -> None:
        # De bridge verwacht voor batterijdata een BATTERY_PULSE-payload.
        self._publish_battery_pulse()

    # ------------------------------------------------------------------
    # Service en afsluiting
    # ------------------------------------------------------------------

    def handle_state_request(self, _request, response):
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

    def destroy_node(self):
        if self.estop_timer is not None:
            self.destroy_timer(self.estop_timer)
            self.estop_timer = None

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("RobotStateManager gestopt.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
