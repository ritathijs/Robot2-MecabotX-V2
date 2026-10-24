#!/usr/bin/env python3

from dataclasses import dataclass
from threading import Lock
import math
import time
from typing import Any

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32, String, UInt8
from wheeltec_robot_msg.srv import GetRobotState


@dataclass
class RobotState:
    robot_active: bool = False
    quiz_active: bool = False
    quiz_finished: bool = False

    admin_open: bool = False
    admin_login_open: bool = False
    robot_reboot_lock: bool = True
    network_ok: bool = True

    battery_ok: bool = False
    battery_voltage: float = 24.0
    battery_status_received: bool = False

    docked: bool = False
    charging_current: float = 0.0
    charging_current_received: bool = False
    infrared_detected: bool = False
    infrared_status_received: bool = False
    at_station: bool = False
    charging: bool = False

    visitors_nearby: bool = False
    in_working_zone: bool = False
    manual_drive: bool = False
    estop_active: bool = False
    bumper_pressed: bool = False
    quiz_options_pause: bool = False
    visitor_tracking_paused: bool = False

    localization_valid: bool = False
    received_amcl: bool = False
    startup_location_resolved: bool = False
    localization_recovery_requested: bool = False
    localization_recovery_started_at: float | None = None
    startup_pose_x: float | None = None
    startup_pose_y: float | None = None
    startup_reset_done_received: bool = False
    startup_location_failed: bool = False
    lost_screen_sent: bool = False

    at_quiz: bool = False
    at_work_area: bool = False
    at_location_c: bool = False
    at_location_d: bool = False
    at_location_e: bool = False
    active_destination: str | None = None

    last_screen: str | None = None


class RobotStateManager(Node):
    BATTERY_OK_THRESHOLD = 22.0
    DOCKED_AVERAGE_VOLTAGE_THRESHOLD = 24.5
    CHARGING_CURRENT_THRESHOLD = 0.1
    DRIVE_TIMEOUT_SEC = 0.4
    ESTOP_DURATION_SEC = 30.0
    STARTUP_SENSOR_TIMEOUT_SEC = 30.0
    LOCALIZATION_RECOVERY_TIMEOUT_SEC = 8.0
    APRILTAG_LOCATION_RADIUS = 3.0

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

        "RobotDriveToChargingStation": "charging_station",
        "FallbackDriveToChargingStation": "charging_station",
        "robot-go-charge": "charging_station",
        "DriveLocationC": "location_c",
        "DriveLocationD": "location_d",
        "DriveLocationE": "location_e",
        "explore-location-c": "location_c",
        "explore-location-d": "location_d",
        "explore-location-e": "location_e",
    }

    def __init__(self):
        super().__init__("robot_state_manager")

        self.state = RobotState()
        self.lock = Lock()

        self.last_drive_time: float | None = None
        self.estop_end_time: float | None = None
        self.estop_timer = None
        self.admin_pause_timer = None
        self.startup_started_at = time.monotonic()
        self.apriltag_location_anchors = {
            "work_area": self._declare_apriltag_pose(
                "apriltag_work_area_pose"
            ),
            "charging_area": self._declare_apriltag_pose(
                "apriltag_charging_station_pose"
            ),
        }

        # Uitgaande topics
        self.screen_pub = self.create_publisher(
            String, "/screen_command", 10
        )
        self.cmd_pub = self.create_publisher(
            String, "/robot_command", 10
        )
        self.localization_reset_pub = self.create_publisher(
            String, "/reset_active", 10
        )
        self.drive_pub = self.create_publisher(
            Twist, "/cmd_vel", 10
        )
        self.tracking_enable_pub = self.create_publisher(
            Bool, "/tracking_enable", 10
        )
        self.admin_pause_pub = self.create_publisher(
            Twist, "/estop_cmd_vel", 10
        )
        self._start_admin_pause()
        self.want_to_start_quiz_pub = self.create_publisher(
            Bool, "/WantToStartQuiz", 10
        )
        self.quiz_finished_pub = self.create_publisher(
            Bool, "/quiz_finished", 10
        )
        self.visitor_no_response_pub = self.create_publisher(
            Bool, "/visitor_no_response", 10
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
        # Compatibility with the legacy charging-station topic name.
        self.charging_station_legacy_pub = self.create_publisher(
            Bool, "/IsRobotAtChargingStation", 10
        )
        self.charging_pub = self.create_publisher(
            Bool, "/IsRobotCharging", 10
        )
        self.docked_pub = self.create_publisher(
            Bool, "/IsRobotDocked", 10
        )
        readiness_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.robot_state_ready_pub = self.create_publisher(
            Bool, "/IsRobotStateReady", readiness_qos
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
        self.location_c_pub = self.create_publisher(
            Bool, "/IsRobotAtLocationC", 10
        )
        self.location_d_pub = self.create_publisher(
            Bool, "/IsRobotAtLocationD", 10
        )
        self.location_e_pub = self.create_publisher(
            Bool, "/IsRobotAtLocationE", 10
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
            "/visitor_tracking_paused",
            self.on_visitor_tracking_paused,
            10,
        )
        self.create_subscription(
            Bool,
            "/reset_patrol_locations",
            self.on_patrol_cycle_complete,
            10,
        )
        self.create_subscription(
            Bool,
            "/in_working_zone",
            self.on_working_zone,
            10,
        )
        self.create_subscription(
            Float32,
            "/robot_charging_current",
            self.on_charging_current,
            10,
        )
        self.create_subscription(
            UInt8,
            "/robot_red_flag",
            self.on_infrared_status,
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
            PoseWithCovarianceStamped,
            "/initialpose",
            self.on_startup_initial_pose,
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
        self.create_subscription(
            String,
            "/robot_command",
            self.on_robot_command,
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
        self.startup_recovery_timer = self.create_timer(
            0.1,
            self._startup_location_check,
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

    def _declare_apriltag_pose(
        self,
        name: str,
    ) -> tuple[float, float]:
        pose = self.declare_parameter(
            name,
            Parameter.Type.DOUBLE_ARRAY,
        ).value
        if (
            not isinstance(pose, (list, tuple))
            or len(pose) != 3
        ):
            raise ValueError(
                f"Parameter {name!r} must contain [x, y, yaw]."
            )

        try:
            x, y, yaw = (float(value) for value in pose)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"Parameter {name!r} must contain numeric [x, y, yaw]."
            ) from exc

        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError(
                f"Parameter {name!r} must contain finite [x, y, yaw]."
            )

        return x, y

    def _publish_status_snapshot(self) -> None:
        with self.lock:
            localization_valid = self.state.localization_valid
            at_quiz = self.state.at_quiz
            at_work_area = self.state.at_work_area
            at_station = self.state.at_station
            charging = self.state.charging
            docked = self.state.docked
            robot_active = self.state.robot_active
            battery_ok = self.state.battery_ok
            tracking_enabled = (
                robot_active
                and battery_ok
                and at_work_area
                and localization_valid
                and not docked
                and not self.state.estop_active
                and not self.state.quiz_options_pause
                and not self.state.admin_open
                and not self.state.admin_login_open
                and not self.state.robot_reboot_lock
                and not self.state.visitor_tracking_paused
            )
            at_location_c = self.state.at_location_c
            at_location_d = self.state.at_location_d
            at_location_e = self.state.at_location_e
            startup_sensors_ready = self._startup_sensors_ready_locked()
            robot_state_ready = (
                startup_sensors_ready
                and self.state.startup_location_resolved
            )

        self.localization_pub.publish(
            self._bool_message(localization_valid)
        )
        self.quiz_pub.publish(self._bool_message(at_quiz))
        self.work_pub.publish(self._bool_message(at_work_area))
        self.location_c_pub.publish(self._bool_message(at_location_c))
        self.location_d_pub.publish(self._bool_message(at_location_d))
        self.location_e_pub.publish(self._bool_message(at_location_e))

        station_msg = self._bool_message(at_station)
        self.charging_station_pub.publish(station_msg)
        self.charging_station_legacy_pub.publish(station_msg)

        self.charging_pub.publish(self._bool_message(charging))
        self.docked_pub.publish(self._bool_message(docked))
        self.robot_state_ready_pub.publish(
            self._bool_message(robot_state_ready)
        )
        self.robot_active_pub.publish(self._bool_message(robot_active))
        self.battery_ok_pub.publish(self._bool_message(battery_ok))
        self.battery_low_pub.publish(self._bool_message(not battery_ok))
        self.tracking_enable_pub.publish(
            self._bool_message(tracking_enabled)
        )

    def send_robot_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)

    def _publish_zero_twist(self) -> None:
        self.drive_pub.publish(Twist())

    def _publish_admin_pause(self) -> None:
        self.admin_pause_pub.publish(Twist())

    def _start_admin_pause(self) -> None:
        self._publish_admin_pause()
        if self.admin_pause_timer is None:
            self.admin_pause_timer = self.create_timer(
                0.05,
                self._publish_admin_pause,
            )

    def _stop_admin_pause(self, force: bool = False) -> None:
        with self.lock:
            pause_required = (
                self.state.admin_open
                or self.state.admin_login_open
                or self.state.robot_reboot_lock
            )

        if pause_required and not force:
            self._publish_admin_pause()
            return

        if self.admin_pause_timer is not None:
            self.destroy_timer(self.admin_pause_timer)
            self.admin_pause_timer = None

    # ------------------------------------------------------------------
    # Sensor- en statuscallbacks
    # ------------------------------------------------------------------

    def on_battery(self, msg: Float32) -> None:
        voltage = float(msg.data)

        with self.lock:
            self.state.battery_voltage = voltage
            self.state.battery_ok = voltage >= self.BATTERY_OK_THRESHOLD
            self.state.battery_status_received = True
            self._update_dock_status_locked()

        self._publish_status_snapshot()

    def on_visitors(self, msg: Bool) -> None:
        with self.lock:
            self.state.visitors_nearby = msg.data

    def on_visitor_tracking_paused(self, msg: Bool) -> None:
        with self.lock:
            self.state.visitor_tracking_paused = msg.data

        self._publish_status_snapshot()

    def on_patrol_cycle_complete(self, msg: Bool) -> None:
        if not msg.data:
            return

        with self.lock:
            self.state.at_location_d = False
            self.state.at_location_e = False
            self.state.at_location_c = False
            if self.state.active_destination in (None, "location_e"):
                self.state.active_destination = None

        self._publish_status_snapshot()

    def on_working_zone(self, msg: Bool) -> None:
        with self.lock:
            self.state.in_working_zone = msg.data

    def on_charging_current(self, msg: Float32) -> None:
        with self.lock:
            self.state.charging_current = float(msg.data)
            self.state.charging_current_received = True
            self._update_dock_status_locked()
        self._publish_status_snapshot()

    def on_infrared_status(self, msg: UInt8) -> None:
        with self.lock:
            self.state.infrared_detected = msg.data > 0
            self.state.infrared_status_received = True
            self._update_dock_status_locked()
        self._publish_status_snapshot()

    def _update_dock_status_locked(self) -> None:
        self.state.charging = (
            self.state.charging_current > self.CHARGING_CURRENT_THRESHOLD
        )
        self.state.docked = self.state.charging or (
            self.state.battery_status_received
            and self.state.battery_voltage
            >= self.DOCKED_AVERAGE_VOLTAGE_THRESHOLD
            and self.state.infrared_detected
        )

        if self.state.docked:
            self.state.localization_valid = True
            self.state.at_station = True

    def _startup_sensors_ready_locked(self) -> bool:
        return (
            self.state.battery_status_received
            and self.state.charging_current_received
            and (
                self.state.infrared_status_received
                or self.state.charging_current
                > self.CHARGING_CURRENT_THRESHOLD
            )
        )

    def _startup_location_check(self) -> None:
        now = time.monotonic()
        publish_snapshot = False
        request_reset = False
        show_lost_screen = False

        with self.lock:
            if self.state.startup_location_resolved:
                if (
                    self.state.startup_location_failed
                    and not self.state.lost_screen_sent
                    and self.cmd_pub.get_subscription_count() > 0
                ):
                    self.state.lost_screen_sent = True
                    show_lost_screen = True
                else:
                    return
            elif self.state.startup_location_failed:
                if (
                    not self.state.lost_screen_sent
                    and self.cmd_pub.get_subscription_count() > 0
                ):
                    self.state.lost_screen_sent = True
                    show_lost_screen = True
                else:
                    return
            else:
                sensors_ready = self._startup_sensors_ready_locked()

                if sensors_ready:
                    self._update_dock_status_locked()

                    if self.state.docked:
                        self.state.at_station = True
                        self.state.at_work_area = False
                        self.state.at_quiz = False
                        self.state.at_location_c = False
                        self.state.at_location_d = False
                        self.state.at_location_e = False
                        self.state.localization_valid = True
                        self.state.startup_location_resolved = True
                        self.state.robot_active = False
                        publish_snapshot = True
                    elif not self.state.localization_recovery_requested:
                        if (
                            self.localization_reset_pub.get_subscription_count()
                            > 0
                        ):
                            msg = String()
                            msg.data = "active"
                            self.localization_reset_pub.publish(msg)
                            self.state.localization_recovery_requested = True
                            self.state.localization_recovery_started_at = now
                            request_reset = True
                        elif (
                            now - self.startup_started_at
                            >= self.STARTUP_SENSOR_TIMEOUT_SEC
                        ):
                            self.state.startup_location_failed = True
                            self.state.robot_active = False
                            publish_snapshot = True
                    elif (
                        self.state.localization_recovery_started_at is not None
                        and now - self.state.localization_recovery_started_at
                        >= self.LOCALIZATION_RECOVERY_TIMEOUT_SEC
                    ):
                        self.state.startup_location_failed = True
                        self.state.robot_active = False
                        publish_snapshot = True
                elif (
                    now - self.startup_started_at
                    >= self.STARTUP_SENSOR_TIMEOUT_SEC
                ):
                    self.state.startup_location_failed = True
                    self.state.robot_active = False
                    publish_snapshot = True

                if (
                    self.state.startup_location_failed
                    and not self.state.lost_screen_sent
                    and self.cmd_pub.get_subscription_count() > 0
                ):
                    self.state.lost_screen_sent = True
                    show_lost_screen = True

        if request_reset:
            self.get_logger().info(
                "Geen dockstatus bij opstart; AprilTag-lokalisatie aangevraagd."
            )
        if publish_snapshot:
            self._publish_status_snapshot()
        if show_lost_screen:
            self.send_robot_command("SHOW_LOST_SCREEN")
            self.get_logger().error(
                "Startlocatie kon niet worden vastgesteld; "
                "de Behavior Tree blijft geblokkeerd."
            )

    def on_robot_active(self, msg: Bool) -> None:
        with self.lock:
            changed = self.state.robot_active != msg.data
            self.state.robot_active = msg.data

        if changed:
            self.get_logger().info(
                f"[Robot] /RobotActive ontvangen: {msg.data}"
            )

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
                (amcl_good and self.state.startup_location_resolved)
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
            startup_reset_pending = (
                self.state.localization_recovery_requested
                and not self.state.startup_location_resolved
                and not self.state.docked
                and not self.state.startup_location_failed
            )

            if startup_reset_pending:
                self.state.startup_reset_done_received = True
                location = self._resolve_startup_location_locked()
            else:
                self.state.localization_valid = True
                location = None

        self._publish_status_snapshot()
        if not startup_reset_pending:
            self.get_logger().info(
                "[Localization] AprilTag-posecorrectie tijdens normaal "
                "gebruik bevestigd."
            )
            return

        if location is None:
            self.get_logger().info(
                "[Localization] AprilTag-reset bevestigd; "
                "wachten op de berekende robotpose."
            )
        elif location == "unknown":
            self.get_logger().error(
                "[Localization] AprilTag-pose kan niet eenduidig aan een "
                "werk- of laadzone worden gekoppeld; robot blijft geblokkeerd."
            )
        else:
            self.get_logger().info(
                f"[Localization] AprilTag-reset bevestigd; "
                f"startlocatie vastgesteld als {location}."
            )

    def on_startup_initial_pose(
        self,
        msg: PoseWithCovarianceStamped,
    ) -> None:
        with self.lock:
            if (
                not self.state.localization_recovery_requested
                or self.state.startup_location_resolved
                or self.state.startup_location_failed
            ):
                return

            self.state.startup_pose_x = float(msg.pose.pose.position.x)
            self.state.startup_pose_y = float(msg.pose.pose.position.y)
            if not self.state.startup_reset_done_received:
                return

            location = self._resolve_startup_location_locked()

        self._publish_status_snapshot()
        if location == "unknown":
            self.get_logger().error(
                "[Localization] AprilTag-pose kan niet eenduidig aan een "
                "werk- of laadzone worden gekoppeld; robot blijft geblokkeerd."
            )
        elif location is not None:
            self.get_logger().info(
                f"[Localization] AprilTag-pose vastgesteld als {location}."
            )

    def _resolve_startup_location_locked(self) -> str | None:
        if (
            not self.state.startup_reset_done_received
            or self.state.startup_pose_x is None
            or self.state.startup_pose_y is None
        ):
            return None

        distances = {
            location: math.hypot(
                self.state.startup_pose_x - anchor_x,
                self.state.startup_pose_y - anchor_y,
            )
            for location, (anchor_x, anchor_y)
            in self.apriltag_location_anchors.items()
        }
        if abs(distances["work_area"] - distances["charging_area"]) <= 1e-6:
            self.state.startup_location_failed = True
            self.state.robot_active = False
            return "unknown"

        location = min(distances, key=distances.get)
        if distances[location] > self.APRILTAG_LOCATION_RADIUS:
            self.state.startup_location_failed = True
            self.state.robot_active = False
            return "unknown"

        self.state.localization_valid = True
        self.state.startup_location_resolved = True
        self.state.at_station = location == "charging_area"
        self.state.at_work_area = location == "work_area"
        if self.state.at_work_area:
            self.state.robot_active = True
        self.state.at_quiz = False
        self.state.at_location_c = False
        self.state.at_location_d = False
        self.state.at_location_e = False
        self.state.active_destination = None
        return location

    def on_recharge_event(self, msg: String) -> None:
        event = msg.data
        self.get_logger().info(f"[Docking] Event ontvangen: {event}")

        with self.lock:
            if event.endswith("DRIVE-TO-DOCK-SUCCESS"):
                self.state.at_station = True
                self.state.localization_valid = True

            elif event.endswith("ROBOT-CHARGING"):
                self.state.charging = True
                self.state.at_station = True
                self.state.docked = True
                self.state.robot_active = False
                self.state.localization_valid = True
                self.state.startup_location_resolved = True

            elif event.endswith("ROBOT-CHARGING-STOPPED"):
                self.state.charging = False

            elif event.endswith("ROBOT-UNDOCKED"):
                self.state.charging = False
                self.state.docked = False
                self.state.at_station = False

            elif event.endswith("DOCKING-FAILED") or event.endswith("DOCK-FAILED"):
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

            if destination is not None:
                # Vertrekt de robot naar een andere bestemming, dan staat hij
                # niet meer bij het laadstation (tenzij hij nog gedockt is).
                if destination != "charging_station" and not self.state.docked:
                    self.state.at_station = False

                if destination == "quiz":
                    self.state.at_location_c = False
                    self.state.at_location_d = False
                    self.state.at_location_e = False
                    self.state.at_quiz = False
                elif destination == "work_area":
                    self.state.at_location_c = False
                    self.state.at_location_d = False
                    self.state.at_location_e = False
                    self.state.at_work_area = False
                elif destination == "charging_station":
                    self.state.at_location_c = False
                    self.state.at_location_d = False
                    self.state.at_location_e = False
                    self.state.at_station = False
                    self.state.at_work_area = False
                elif destination == "location_c":
                    self.state.at_location_c = False
                elif destination == "location_d":
                    self.state.at_location_d = False
                elif destination == "location_e":
                    self.state.at_location_e = False

        # Patrol navigation is part of explore mode; keep the visitor start
        # screen visible instead of switching back to the startup screen.
        patrol_destination = destination in {
            "location_c",
            "location_d",
            "location_e",
        }
        if patrol_destination:
            with self.lock:
                screen_changed = self.state.last_screen != "robot-explore"
                self.state.last_screen = "robot-explore"

            if screen_changed and not admin_open:
                screen_msg = String()
                screen_msg.data = "robot-explore"
                self.screen_pub.publish(screen_msg)
        elif not admin_open and node_name not in (
            "RobotRotationFollowMe",
            "robot-arrived-at-visitors",
        ):
            screen_name = (
                "robot-startup"
                if node_name == "DriveWorkArea"
                else node_name
            )
            with self.lock:
                self.state.last_screen = screen_name

            screen_msg = String()
            screen_msg.data = screen_name
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
            elif destination == "location_c":
                self.state.at_location_c = succeeded
            elif destination == "location_d":
                self.state.at_location_d = succeeded
            elif destination == "location_e":
                self.state.at_location_e = succeeded
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
            with self.lock:
                self.state.bumper_pressed = False
            return

        with self.lock:
            newly_pressed = not self.state.bumper_pressed
            self.state.bumper_pressed = True

        if not newly_pressed:
            return

        self.get_logger().warning(
            f"Bumper gedetecteerd: {status_text}; "
            "de bumpercontroller stuurt de stop-/uitwijkbeweging."
        )

    def activate_estop(
        self,
        reason: str,
        duration_sec: float = ESTOP_DURATION_SEC,
    ) -> None:
        with self.lock:
            if not self.state.estop_active:
                self.get_logger().warning(
                    f"ESTOP geactiveerd: {reason}"
                )
            self.state.estop_active = True
        self._publish_status_snapshot()

        self.estop_end_time = (
            time.monotonic() + duration_sec
        )

        if self.estop_timer is None:
            self.estop_timer = self.create_timer(
                0.05,
                self._estop_tick,
            )

    def _estop_tick(self) -> None:
        status_changed = False
        if self.estop_end_time is not None:
            if time.monotonic() >= self.estop_end_time:
                with self.lock:
                    self.state.estop_active = False
                self.estop_end_time = None
                status_changed = True

                self.get_logger().info(
                    "ESTOP-timer verlopen; softwarematige ESTOP vrijgegeven."
                )

        with self.lock:
            stop_required = (
                self.state.estop_active or self.state.quiz_options_pause
            )
        if not stop_required:
            if self.estop_timer is not None:
                self.destroy_timer(self.estop_timer)
                self.estop_timer = None
            if status_changed:
                self._publish_status_snapshot()
            return

        self._publish_zero_twist()
        self.send_robot_command("STOP")
        if status_changed:
            self._publish_status_snapshot()

    # ------------------------------------------------------------------
    # Socket-events vanuit QuizSocketBridge
    # ------------------------------------------------------------------

    def handle_socket_event(self, event: str, data: Any = None) -> None:
        if event == "quiz_finished":
            with self.lock:
                self.state.quiz_active = False
                self.state.quiz_finished = True

            self.quiz_finished_pub.publish(self._bool_message(True))
            self.want_to_start_quiz_pub.publish(self._bool_message(False))

        elif event == "quiz_inactive":
            with self.lock:
                self.state.quiz_active = False

        elif event == "drive_to_quiz_location":
            self.want_to_start_quiz_pub.publish(self._bool_message(True))

        elif event == "screen_request":
            self.handle_screen_request(data)

        elif event == "ask_is_active":
            bridge = data
            with self.lock:
                current_active = self.state.robot_active

            if bridge is not None:
                bridge.send_active_status_to_pi(current_active)

        # Laat hierna de overige bestaande elif-takken staan.

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

        elif event == "temporary_stop":
            with self.lock:
                admin_pause_active = (
                    self.state.admin_open or self.state.admin_login_open
                )

            if admin_pause_active:
                self.get_logger().info(
                    "Tijdelijke stop genegeerd: admin/login-pauze is actief."
                )
                return

            if not isinstance(data, dict):
                self.get_logger().warning(
                    "robot-stop-for-x-time bevat geen geldige payload."
                )
                return

            try:
                duration_sec = float(data.get("time")) / 1000.0
            except (TypeError, ValueError):
                self.get_logger().warning(
                    "robot-stop-for-x-time bevat geen geldige tijdsduur."
                )
                return

            if not math.isfinite(duration_sec) or duration_sec <= 0:
                self.get_logger().warning(
                    "robot-stop-for-x-time moet een positieve, eindige tijd bevatten."
                )
                return

            self.activate_estop(
                "tijdelijke stop aangevraagd via Pi",
                duration_sec,
            )

        elif event == "quiz_options_pause":
            if not isinstance(data, bool):
                self.get_logger().warning(
                    "robot-quiz-options-pause moet een boolean bevatten."
                )
                return

            with self.lock:
                self.state.quiz_options_pause = data
                estop_active = self.state.estop_active

            if data and self.estop_timer is None:
                self.estop_timer = self.create_timer(
                    0.05,
                    self._estop_tick,
                )
            elif not data and not estop_active:
                if self.estop_timer is not None:
                    self.destroy_timer(self.estop_timer)
                    self.estop_timer = None
                self.estop_end_time = None

            self._publish_status_snapshot()
            self.get_logger().info(
                "Robotpauze voor quizkeuzes "
                + ("geactiveerd." if data else "vrijgegeven.")
            )

        elif event == "visitor_no_response":
            if data is not True:
                self.get_logger().warning(
                    "robot-visitor-no-response moet true bevatten."
                )
                return

            self.visitor_no_response_pub.publish(self._bool_message(True))
            self.get_logger().info(
                "Geen reactie op quizkeuzes; de huidige bezoeker wordt losgelaten."
            )

        elif event == "robot_login_open":
            self._handle_robot_login_open()

        elif event == "robot_login_closed":
            self._handle_robot_login_closed()

        elif event == "robot_screen_state":
            self._handle_robot_screen_state(data)

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

        self._start_admin_pause()
        self.send_robot_command("STOP")
        self._publish_zero_twist()
        self._publish_status_snapshot()

    def _handle_robot_login_open(self) -> None:
        with self.lock:
            if self.state.admin_open:
                return
            self.state.admin_login_open = True

        self._start_admin_pause()
        self.send_robot_command("STOP")
        self._publish_zero_twist()

    def _handle_robot_login_closed(self) -> None:
        with self.lock:
            was_login_open = self.state.admin_login_open
            admin_open = self.state.admin_open
            self.state.admin_login_open = False
            saved_screen = self.state.last_screen

        if not admin_open:
            self._stop_admin_pause()
            self._publish_status_snapshot()
            if was_login_open:
                screen_msg = String()
                screen_msg.data = saved_screen or "robot-startup"
                self.screen_pub.publish(screen_msg)

    def _handle_robot_screen_state(self, data: Any) -> None:
        if (
            not isinstance(data, dict)
            or not isinstance(data.get("adminOpen"), bool)
            or not isinstance(data.get("adminLoginOpen"), bool)
            or not isinstance(data.get("robotRebootLock"), bool)
        ):
            self.get_logger().warning(
                "Ongeldige robot-screen-state ontvangen; status niet aangepast."
            )
            return

        admin_open = data["adminOpen"]
        admin_login_open = data["adminLoginOpen"] and not admin_open
        robot_reboot_lock = data["robotRebootLock"]

        with self.lock:
            previously_open = (
                self.state.admin_open
                or self.state.admin_login_open
                or self.state.robot_reboot_lock
            )
            self.state.admin_open = admin_open
            self.state.admin_login_open = admin_login_open
            self.state.robot_reboot_lock = robot_reboot_lock
            saved_screen = self.state.last_screen

        if admin_open or admin_login_open or robot_reboot_lock:
            self._start_admin_pause()
            self.send_robot_command("STOP")
            self._publish_zero_twist()
        else:
            self._stop_admin_pause()
            if previously_open:
                screen_msg = String()
                screen_msg.data = saved_screen or "robot-startup"
                self.screen_pub.publish(screen_msg)
        self._publish_status_snapshot()

    def on_robot_command(self, msg: String) -> None:
        if msg.data.strip() != "START_QUIZ":
            return

        with self.lock:
            self.state.quiz_active = True
            self.state.quiz_finished = False

        self.quiz_finished_pub.publish(self._bool_message(False))

    def _handle_admin_panel_open(self) -> None:
        with self.lock:
            self.state.admin_login_open = False
            self.state.admin_open = True

        self._start_admin_pause()
        self.send_robot_command("STOP")
        self._publish_zero_twist()
        self._publish_status_snapshot()

    def _handle_admin_closed(self) -> None:
        with self.lock:
            self.state.admin_open = False
            self.state.admin_login_open = False
            saved_screen = self.state.last_screen

        self._stop_admin_pause()

        screen_msg = String()
        screen_msg.data = saved_screen or "robot-startup"
        self.screen_pub.publish(screen_msg)
        self._publish_status_snapshot()

    def handle_screen_request(self, data: Any) -> None:
        if data is None:
            # De Pi vraagt zonder schermnaam om het huidige scherm (bv. na het
            # herladen of sluiten van login/admin): stuur het laatste scherm opnieuw.
            with self.lock:
                if self.state.admin_open or self.state.admin_login_open:
                    return
                current_screen = self.state.last_screen
            if current_screen:
                screen_msg = String()
                screen_msg.data = current_screen
                self.screen_pub.publish(screen_msg)
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
                and not self.state.robot_reboot_lock
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

        self._stop_admin_pause(force=True)

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
