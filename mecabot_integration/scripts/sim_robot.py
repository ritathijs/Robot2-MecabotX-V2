#!/usr/bin/env python3
"""Simulatie van Nav2 en robotsensoren voor het testen van de RT-flow.

Vervangt tijdelijk Nav2, AMCL, de laadpaal en de sensoren, zodat de echte
RT-nodes (BT, robot_controller, RobotStateManager, rt_drive_to_coord)
ongewijzigd kunnen draaien zonder dat de robot rijdt.
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, UInt8


def yaw_to_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class SimRobot(Node):
    def __init__(self):
        super().__init__("sim_robot")
        self.cb_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        self.speed = self.declare_parameter("speed", 1.0).value
        self.start_docked = self.declare_parameter("start_docked", True).value
        start_pose = self.declare_parameter(
            "start_pose", [0.0, 0.0, 0.0]
        ).value
        self.voltage_docked = self.declare_parameter(
            "battery_voltage_docked", 25.5
        ).value
        self.voltage_undocked = self.declare_parameter(
            "battery_voltage_undocked", 24.0
        ).value
        self.dock_delay = self.declare_parameter("dock_delay_sec", 3.0).value
        self.undock_delay = self.declare_parameter(
            "undock_delay_sec", 2.0
        ).value
        self.auto_activate_delay = self.declare_parameter(
            "auto_activate_delay_sec", 0.0
        ).value
        # Scenario "day": bezoeker en lage batterij op vaste tijdstippen.
        self.scenario = self.declare_parameter("scenario", "none").value
        self.visitor_at = self.declare_parameter("visitor_at_sec", 60.0).value
        self.visitor_duration = self.declare_parameter(
            "visitor_duration_sec", 40.0
        ).value
        self.visitor_distance = self.declare_parameter(
            "visitor_distance", 3.0
        ).value
        self.low_battery_at = self.declare_parameter(
            "low_battery_at_sec", 150.0
        ).value
        self.low_battery_voltage = self.declare_parameter(
            "low_battery_voltage", 20.0
        ).value
        self.reactivate_after_charge = self.declare_parameter(
            "reactivate_after_charge_sec", 0.0
        ).value

        self.x, self.y, self.yaw = (float(v) for v in start_pose)
        self.docked = bool(self.start_docked)
        self.active_goal = None
        self.battery_override = None
        self.visitor_distance_now = 0.0
        self.visitor_stamp = None
        self.bump_text = None

        self.amcl_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/amcl_pose", 10
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.battery_pub = self.create_publisher(
            Float32, "/BatteryAverageVoltage", 10
        )
        self.current_pub = self.create_publisher(
            Float32, "/robot_charging_current", 10
        )
        self.infrared_pub = self.create_publisher(UInt8, "/robot_red_flag", 10)
        self.bump_pub = self.create_publisher(String, "/bump_status", 10)
        self.event_pub = self.create_publisher(
            String, "/auto_recharge_event", 10
        )
        self.reset_done_pub = self.create_publisher(String, "/reset_done", 10)
        self.active_pub = self.create_publisher(Bool, "/RobotActive", 10)
        self.target_distance_pub = self.create_publisher(
            Float32, "/target_distance", 10
        )
        self.visitors_pub = self.create_publisher(
            Bool, "/visitors_nearby", 10
        )
        self.people_coord_pub = self.create_publisher(
            PoseStamped, "/peoplesearchcoord", 10
        )

        self.create_subscription(
            String, "/DockingCommand", self.on_docking_command, 10,
            callback_group=self.cb_group,
        )
        # Handmatige bediening tijdens de simulatie.
        self.create_subscription(
            Float32, "/sim/battery", self.on_sim_battery, 10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            Float32, "/sim/visitor_distance", self.on_sim_visitor, 10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            String, "/sim/bump", self.on_sim_bump, 10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            String, "/force_charge", self.on_force_charge, 10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            String, "/charge_XSTOP", self.on_undock_command, 10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            String, "/reset_active", self.on_reset_active, 10,
            callback_group=self.cb_group,
        )

        self.action_server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self.execute_goal,
            goal_callback=self.on_goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self.cb_group,
        )

        self.create_timer(0.1, self.publish_pose, callback_group=self.cb_group)
        self.create_timer(
            0.5, self.publish_sensors, callback_group=self.cb_group
        )

        if self.auto_activate_delay > 0.0:
            self.activate_timer = self.create_timer(
                self.auto_activate_delay,
                self.activate_robot,
                callback_group=self.cb_group,
            )

        self.start_scenario()
        self.get_logger().info(
            f"Simulatie gestart op ({self.x:.2f}, {self.y:.2f}), "
            f"gedockt={self.docked}, snelheid={self.speed} m/s."
        )

    # --- Sensoren en pose -------------------------------------------------

    def publish_pose(self):
        with self.lock:
            x, y, yaw = self.x, self.y, self.yaw
        self.amcl_pub.publish(self._pose_msg(x, y, yaw))

    def _pose_msg(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = (
            yaw_to_quaternion(yaw)
        )
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = covariance[35] = 0.01
        msg.pose.covariance = covariance
        return msg

    def publish_sensors(self):
        with self.lock:
            docked = self.docked

        battery = Float32()
        if docked:
            battery.data = float(self.voltage_docked)
        elif self.battery_override is not None:
            battery.data = float(self.battery_override)
        else:
            battery.data = float(self.voltage_undocked)
        self.battery_pub.publish(battery)

        current = Float32()
        current.data = 1.0 if docked else 0.0
        self.current_pub.publish(current)

        infrared = UInt8()
        infrared.data = 1 if docked else 0
        self.infrared_pub.publish(infrared)

        bump = String()
        bump.data = self.bump_text or "De bumper is niet ingedrukt"
        self.bump_pub.publish(bump)

        self.publish_visitor()

    def publish_visitor(self):
        with self.lock:
            distance = self.visitor_distance_now
            stamp = self.visitor_stamp
            x, y, yaw = self.x, self.y, self.yaw

        target = Float32()
        target.data = float(distance)
        self.target_distance_pub.publish(target)

        nearby = Bool()
        nearby.data = distance > 0.0
        self.visitors_pub.publish(nearby)

        if distance > 0.0 and stamp is not None:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = stamp
            pose.pose.position.x = x + distance * math.cos(yaw)
            pose.pose.position.y = y + distance * math.sin(yaw)
            pose.pose.orientation.z, pose.pose.orientation.w = (
                yaw_to_quaternion(yaw + math.pi)
            )
            self.people_coord_pub.publish(pose)

    # --- Handmatige bediening en scenario ---------------------------------

    def on_sim_battery(self, msg):
        self.battery_override = float(msg.data) if msg.data > 0.0 else None
        self.get_logger().info(
            f"Batterijspanning (niet gedockt): {self.battery_override}"
        )

    def on_sim_visitor(self, msg):
        with self.lock:
            appeared = self.visitor_distance_now <= 0.0 < msg.data
            self.visitor_distance_now = max(0.0, float(msg.data))
            if appeared:
                self.visitor_stamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            f"Bezoeker op {msg.data:.1f} m"
            if msg.data > 0.0 else "Bezoeker vertrokken"
        )

    def on_sim_bump(self, msg):
        text = msg.data.strip()
        self.bump_text = text or None
        self.get_logger().info(f"Bumper: {text or 'losgelaten'}")

    def start_scenario(self):
        if self.scenario != "day":
            return
        self.get_logger().info(
            "Scenario 'day': bezoeker na "
            f"{self.visitor_at:.0f} s, lage batterij na "
            f"{self.low_battery_at:.0f} s."
        )
        self._after(
            self.visitor_at,
            lambda: self.on_sim_visitor(
                Float32(data=float(self.visitor_distance))
            ),
        )
        self._after(
            self.visitor_at + self.visitor_duration,
            lambda: self.on_sim_visitor(Float32(data=0.0)),
        )
        self._after(
            self.low_battery_at,
            lambda: self.on_sim_battery(
                Float32(data=float(self.low_battery_voltage))
            ),
        )

    def activate_again(self):
        msg = Bool()
        msg.data = True
        self.active_pub.publish(msg)
        self.get_logger().info("Robot opnieuw actief gezet na laden.")

    def activate_robot(self):
        self.activate_timer.cancel()
        msg = Bool()
        msg.data = True
        self.active_pub.publish(msg)
        self.get_logger().info("Robot automatisch actief gezet (/RobotActive).")

    # --- Laadstation ------------------------------------------------------

    def _publish_event(self, text):
        msg = String()
        msg.data = text
        self.event_pub.publish(msg)

    def _after(self, delay, action):
        timer_holder = {}

        def fire():
            timer_holder["timer"].cancel()
            action()

        timer_holder["timer"] = self.create_timer(
            delay, fire, callback_group=self.cb_group
        )

    def on_docking_command(self, msg):
        if msg.data.strip().upper() != "START":
            return
        self._start_docking("Docking")

    def on_force_charge(self, msg):
        if msg.data.strip().upper() != "0START":
            return
        self._start_docking("IR-docking")

    def _start_docking(self, label):
        self.get_logger().info(f"{label} gestart (gesimuleerd).")

        def finish():
            with self.lock:
                self.docked = True
            self.battery_override = None
            self._publish_event("SIM-DRIVE-TO-DOCK-SUCCESS")
            self._publish_event("SIM-ROBOT-CHARGING")
            if self.reactivate_after_charge > 0.0:
                self._after(self.reactivate_after_charge, self.activate_again)

        self._after(self.dock_delay, finish)

    def on_undock_command(self, msg):
        if msg.data.strip().upper() != "XSTOPS":
            return
        self.get_logger().info("Undocking gestart (gesimuleerd).")

        def finish():
            with self.lock:
                self.docked = False
            self._publish_event("SIM-ROBOT-UNDOCKED")

        self._after(self.undock_delay, finish)

    def on_reset_active(self, msg):
        if msg.data.strip().lower() != "active":
            return
        self.get_logger().info("AprilTag-reset aangevraagd; bevestiging volgt.")

        def finish():
            with self.lock:
                x, y, yaw = self.x, self.y, self.yaw
            self.initial_pose_pub.publish(self._pose_msg(x, y, yaw))
            done = String()
            done.data = "done"
            self.reset_done_pub.publish(done)

        self._after(1.0, finish)

    # --- Nav2-actie -------------------------------------------------------

    def on_goal(self, _goal_request):
        return GoalResponse.ACCEPT

    def execute_goal(self, goal_handle):
        with self.lock:
            self.active_goal = goal_handle

        target = goal_handle.request.pose.pose
        target_x = target.position.x
        target_y = target.position.y
        target_yaw = quaternion_to_yaw(target.orientation)
        self.get_logger().info(
            f"Nav-doel ontvangen: ({target_x:.2f}, {target_y:.2f})."
        )

        period = 0.1
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Nav-doel geannuleerd.")
                return NavigateToPose.Result()

            with self.lock:
                if self.active_goal is not goal_handle:
                    goal_handle.abort()
                    return NavigateToPose.Result()
                dx = target_x - self.x
                dy = target_y - self.y
                distance = math.hypot(dx, dy)
                step = self.speed * period

                if distance <= step:
                    self.x, self.y, self.yaw = target_x, target_y, target_yaw
                    break

                self.x += dx / distance * step
                self.y += dy / distance * step
                self.yaw = math.atan2(dy, dx)

            time.sleep(period)

        with self.lock:
            self.active_goal = None
        goal_handle.succeed()
        self.get_logger().info("Nav-doel bereikt (gesimuleerd).")
        return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)
    node = SimRobot()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
