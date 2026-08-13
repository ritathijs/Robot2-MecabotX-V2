#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
from datetime import datetime

LOGFILE = "ros.log"

TYPE_MAP = {
    "std_msgs/msg/String": String,
    "std_msgs/msg/Bool": Bool,
    "std_msgs/msg/Float32": Float32,
    "geometry_msgs/msg/Twist": Twist,
}

TOPICS = [
    "/robot_command",
    "/screen_command",
    "/cmd_vel",
    "/gui_cmd_vel",
    "/manual_drive_active",
    "/BehaviorTreeNode",
    "/btDriveCoord",
    "/drive_to_coord_status",
    "/auto_recharge_event",
    "/RobotActive",
    "/BatteryAverageVoltage",
    "/bump_status",
    "/visitors_nearby",
    "/in_working_zone",
    "/dock_state",
]

def log(topic, msg):
    timestamp = datetime.now().isoformat()
    print(f"[{timestamp}] {topic}: {msg}")

class RosSniffer(Node):
    def __init__(self):
        super().__init__("ros_sniffer")
        self.get_logger().info("🔍 ROS Sniffer gestart…")

        # ROS2 geeft een lijst van tuples → maak er een dict van
        topic_types_list = self.get_topic_names_and_types()
        topic_types = dict(topic_types_list)

        for topic in TOPICS:
            if topic not in topic_types:
                self.get_logger().warn(f"⚠ Topic {topic} bestaat niet")
                continue

            # topic_types[topic] is een lijst van types → neem de eerste
            type_name = topic_types[topic][0]

            if type_name not in TYPE_MAP:
                self.get_logger().warn(f"⚠ Topic {topic} heeft onbekend type {type_name}")
                continue

            msg_type = TYPE_MAP[type_name]

            # Gebruik lambda om topicnaam mee te geven
            self.create_subscription(msg_type, topic,
                                     lambda msg, t=topic: self.cb(msg, t),
                                     10)

            self.get_logger().info(f"📡 Subscribed op {topic} ({type_name})")

    def cb(self, msg, topic):
        log(topic, msg)

def main(args=None):
        rclpy.init(args=args)
        node = RosSniffer()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

