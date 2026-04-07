import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import json
import math
import time  # nodig voor sleep

class PublishBackPose(Node):
    def __init__(self):
        super().__init__('publish_back_pose')

        # QoS voor XSTOP met reliable
        qos_reliable = QoSProfile(depth=1)
        qos_reliable.reliability = ReliabilityPolicy.RELIABLE

        # Publisher voor PoseWithCovarianceStamped (standaard QoS, gaat 10x sturen)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # Publisher voor XSTOP met reliable QoS
        self.xstop_pub = self.create_publisher(
            String,
            '/charge_XSTOP',
            qos_reliable
        )

        # Subscriber naar resetPositie
        self.reset_sub = self.create_subscription(
            String,
            '/resetPositionChargeStation',
            self.reset_callback,
            1
        )

        # Subscriber voor MANUAL_DRIVE_CONTROL
        self.manual_drive_sub = self.create_subscription(
            String,
            '/ManualDriveControleLocation',
            self.manual_drive_callback,
            1
        )

        # JSON bestand pad
        json_file = "/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/Charger_Position.json"
        with open(json_file, "r") as f:
            data = json.load(f)

        self.p_x = data["p_x"]
        self.p_y = data["p_y"]
        self.orien_z = data["orien_z"]
        self.orien_w = data["orien_w"]

        # Bereken yaw (2D)
        self.yaw = 2 * math.atan2(self.orien_z, self.orien_w)

        # Bereken achterliggende positie
        self.new_x = self.p_x - 1.2 * math.cos(self.yaw)
        self.new_y = self.p_y - 1.2 * math.sin(self.yaw)

        self.start_timer = self.create_timer(2.0, self.startup_callback)
        self.start_timer.cancelled = False  # zorg dat timer nog niet geannuleerd is


    def startup_callback(self):
        # Annuleer timer zodat dit maar 1 keer gebeurt
        self.start_timer.cancel()

        # Stuur XSTOP
        self.send_xstop()

        # Publiceer pose 10x
        self.publish_pose_multiple()


    def send_xstop(self):
        while self.xstop_pub.get_subscription_count() == 0:
            self.get_logger().info("Waiting for subscribers on /charge_XSTOP...")
            time.sleep(0.1)
        msg = String()
        msg.data = "XSTOP"
        self.xstop_pub.publish(msg)
        self.get_logger().info("Sent XSTOP to /force_charge (reliable)")

    def publish_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        pose_msg.pose.pose.position.x = self.new_x
        pose_msg.pose.pose.position.y = self.new_y
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = self.orien_z
        pose_msg.pose.pose.orientation.w = self.orien_w

        # Covariance instellen
        pose_msg.pose.covariance = [0.0]*36
        pose_msg.pose.covariance[0] = 0.01
        pose_msg.pose.covariance[7] = 0.01
        pose_msg.pose.covariance[35] = 0.01

        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f'Pose published: x={self.new_x:.3f}, y={self.new_y:.3f}')

    def publish_pose_multiple(self):
        # Publish 10 keer met kleine delay
        for i in range(10):
            self.publish_pose()
            self.get_logger().info(f'Publish {i+1}/10 to /initialpose')
            time.sleep(0.3)  # kleine delay zodat AMCL het zeker ontvangt

    def reset_callback(self, msg):
        if msg.data == "RESET":
            self.get_logger().info("RESET received, republishing pose")
            self.publish_pose_multiple()

    def manual_drive_callback(self, msg):
        if msg.data == "MANUAL_DRIVE_CONTROL":
            self.get_logger().info("MANUAL_DRIVE_CONTROL received, sending XSTOP")
            self.send_xstop()

def main(args=None):
    rclpy.init(args=args)
    node = PublishBackPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
