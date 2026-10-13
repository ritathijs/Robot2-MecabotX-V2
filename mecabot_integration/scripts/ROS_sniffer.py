#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import (
    String,
    Bool,
    Float32,
    Int8,
    UInt8
)

from geometry_msgs.msg import (
    Twist,
    PoseStamped,
    PoseWithCovarianceStamped
)

from sensor_msgs.msg import (
    LaserScan,
    PointCloud2
)

from nav_msgs.msg import (
    Odometry
)

from action_msgs.msg import (
    GoalStatusArray
)

from rclpy.qos import qos_profile_sensor_data

from datetime import datetime

LOGFILE = "ros.log"

TYPE_MAP = {
    "std_msgs/msg/String": String,
    "std_msgs/msg/Bool": Bool,
    "std_msgs/msg/Float32": Float32,
    "std_msgs/msg/Int8": Int8,
    "std_msgs/msg/UInt8": UInt8,
    "geometry_msgs/msg/Twist": Twist,
    "geometry_msgs/msg/PoseStamped": PoseStamped,
    "geometry_msgs/msg/PoseWithCovarianceStamped": PoseWithCovarianceStamped,
    "sensor_msgs/msg/LaserScan": LaserScan,
    "sensor_msgs/msg/PointCloud2": PointCloud2,
    "nav_msgs/msg/Odometry": Odometry,
    "action_msgs/msg/GoalStatusArray": GoalStatusArray,
}

TOPICS = [

    "/robot_command",
    "/screen_command",
    "/BehaviorTreeNode",

    "/btDriveCoord",
    "/goal_pose",
    "/drive_to_coord_status",

    "/DockingCommand",
    "/auto_recharge_event",
    "/dock_state",

    "/RobotActive",
    "/manual_drive_active",
    "/WantToStartQuiz",

    "/IsLocalizationValid",
    "/IsRobotAtQuiz",
    "/IsRobotAtWorkArea",
    "/IsRobotAtChargingStation",
    "/IsRobotCharging",
    "/IsBatteryLow",
    "/IsBatteryOK",
    "/IsRobotActive",
    "/IsRobotNearChargingStation",

    "/battery_percentage",
    "/BatteryAverageVoltage",
    "/PowerVoltage",
    "/robot_charging_current",
    "/robot_charging_flag",

    "/amcl_pose",

    "/odom",
    "/odom_combined",

    "/cmd_vel",
    "/robot_cmd_vel",
    "/gui_cmd_vel",

    "/camera_scan",

    "/visitors_nearby",
    "/in_working_zone",

    "/navigate_to_pose/_action/status",

    "/reset_done",
    "/resetPositionChargeStation",
]


class RosSniffer(Node):

    def __init__(self):

        super().__init__("ros_sniffer")

        self.subscribers = {}
        self.last_values = {}
        self.seen_topics = set()

        self.get_logger().info("🔍 ROS Sniffer gestart")

        self.scan_topics()

        self.scan_timer = self.create_timer(
            10.0,
            self.scan_topics
        )

        self.snapshot_timer = self.create_timer(
            30.0,
            self.robot_snapshot
        )

        self.publisher_timer = self.create_timer(
            60.0,
            self.show_publishers
        )

    #################################################################
    # Logging
    #################################################################

    def write_log(self, txt):

        now = datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S"
        )

        line = f"[{now}] {txt}"

        print(line)

        try:
            with open(LOGFILE, "a") as f:
                f.write(line + "\n")
        except Exception:
            pass

    #################################################################
    # Topic scanning
    #################################################################

    def scan_topics(self):

        topic_types = dict(
            self.get_topic_names_and_types()
        )

        for topic in sorted(topic_types):

            if topic in self.seen_topics:
                continue

            self.seen_topics.add(topic)

            pubs = self.count_publishers(topic)
            subs = self.count_subscribers(topic)

            self.write_log(
                f"NEW TOPIC: {topic} "
                f"{topic_types[topic]} "
                f"(pub={pubs}, sub={subs})"
            )

        for topic in TOPICS:

            if topic in self.subscribers:
                continue

            if topic not in topic_types:
                continue

            type_name = topic_types[topic][0]

            if type_name not in TYPE_MAP:

                self.write_log(
                    f"UNSUPPORTED TYPE: "
                    f"{topic} -> {type_name}"
                )

                continue

            msg_type = TYPE_MAP[type_name]

            qos = (
                qos_profile_sensor_data
                if type_name == "sensor_msgs/msg/LaserScan"
                else 10
            )

            self.subscribers[topic] = (
                self.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, t=topic:
                        self.cb(msg, t),
                    qos
                )
            )

            self.write_log(
                f"SUBSCRIBED: {topic}"
            )

    #################################################################
    # Message formatting
    #################################################################

    def format_msg(self, msg):

        if isinstance(msg, String):
            return msg.data

        if isinstance(msg, Bool):
            return str(msg.data)

        if isinstance(msg, Float32):
            return f"{msg.data:.2f}"

        if isinstance(msg, Int8):
            return str(msg.data)

        if isinstance(msg, UInt8):
            return str(msg.data)

        if isinstance(msg, Twist):
            return (
                f"lin={msg.linear.x:.2f}, "
                f"ang={msg.angular.z:.2f}"
            )

        if isinstance(msg, PoseStamped):
            return (
                f"x={msg.pose.position.x:.2f}, "
                f"y={msg.pose.position.y:.2f}"
            )

        if isinstance(msg, PoseWithCovarianceStamped):

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            cov_x = msg.pose.covariance[0]
            cov_y = msg.pose.covariance[7]
            cov_yaw = msg.pose.covariance[35]

            return (
                f"x={x:.2f}, "
                f"y={y:.2f}, "
                f"cov=({cov_x:.3f}, "
                f"{cov_y:.3f}, "
                f"{cov_yaw:.3f})"
            )

        if isinstance(msg, LaserScan):
            return f"ranges={len(msg.ranges)}"

        if isinstance(msg, PointCloud2):
            return (
                f"width={msg.width}, "
                f"height={msg.height}"
            )

        if isinstance(msg, Odometry):
            return (
                f"x={msg.pose.pose.position.x:.2f}, "
                f"y={msg.pose.pose.position.y:.2f}"
            )

        if isinstance(msg, GoalStatusArray):

            if len(msg.status_list) == 0:
                return "EMPTY"

            status = msg.status_list[-1].status

            meanings = {
                0: "UNKNOWN",
                1: "ACCEPTED",
                2: "EXECUTING",
                3: "CANCELING",
                4: "SUCCEEDED",
                5: "CANCELED",
                6: "ABORTED",
            }

            return (
                f"{status} "
                f"{meanings.get(status, '?')}"
            )

        return str(msg)

    #################################################################
    # Callback
    #################################################################

    def cb(self, msg, topic):

        value = self.format_msg(msg)

        old_value = self.last_values.get(topic)

        if value == old_value:
            return

        self.last_values[topic] = value

        self.write_log(
            f"{topic}: {value}"
        )

        if topic == "/BehaviorTreeNode":

            self.write_log(
                f"BT EVENT -> {value}"
            )

            self.robot_snapshot()

        elif topic == "/drive_to_coord_status":

            self.write_log(
                f"NAV EVENT -> {value}"
            )

        elif topic == "/DockingCommand":

            self.write_log(
                f"DOCK EVENT -> {value}"
            )

    #################################################################
    # Snapshot
    #################################################################

    def robot_snapshot(self):

        self.write_log("")
        self.write_log(
            "========== MECABOT SNAPSHOT =========="
        )

        sections = {

            "ROBOT": [
                "/RobotActive",
                "/manual_drive_active",
                "/BehaviorTreeNode"
            ],

            "QUIZ": [
                "/WantToStartQuiz",
                "/IsRobotAtQuiz"
            ],

            "LOCALIZATION": [
                "/IsLocalizationValid",
                "/amcl_pose"
            ],

            "NAVIGATION": [
                "/drive_to_coord_status",
                "/btDriveCoord",
                "/goal_pose",
                "/navigate_to_pose/_action/status",
                "/odom"
            ],

            "WORK AREA": [
                "/IsRobotAtWorkArea",
                "/in_working_zone",
                "/visitors_nearby"
            ],

            "DOCKING": [
                "/DockingCommand",
                "/dock_state",
                "/auto_recharge_event",
                "/IsRobotCharging",
                "/IsRobotAtChargingStation"
            ],

            "BATTERY": [
                "/battery_percentage",
                "/BatteryAverageVoltage",
                "/PowerVoltage",
                "/robot_charging_current",
                "/robot_charging_flag",
                "/IsBatteryLow",
                "/IsBatteryOK"
            ]
        }

        for section, topics in sections.items():

            self.write_log("")
            self.write_log(f"[{section}]")

            for topic in topics:

                value = self.last_values.get(
                    topic,
                    "UNKNOWN"
                )

                self.write_log(
                    f"{topic}: {value}"
                )

        self.write_log("")
        self.write_log(
            "======================================"
        )

    #################################################################
    # Important topics
    #################################################################

    def show_publishers(self):

        self.write_log("")
        self.write_log(
            "===== IMPORTANT TOPICS ====="
        )

        important = [

            "/BehaviorTreeNode",
            "/robot_command",
            "/drive_to_coord_status",
            "/DockingCommand",

            "/WantToStartQuiz",

            "/IsRobotAtQuiz",
            "/IsRobotAtWorkArea",
            "/IsRobotCharging",

            "/battery_percentage",
            "/PowerVoltage"
        ]

        for topic in important:

            pubs = self.count_publishers(topic)
            subs = self.count_subscribers(topic)

            self.write_log(
                f"{topic}: "
                f"pub={pubs} "
                f"sub={subs}"
            )

    #################################################################
    # MAIN
    #################################################################


def main(args=None):

    rclpy.init(args=args)

    node = RosSniffer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
