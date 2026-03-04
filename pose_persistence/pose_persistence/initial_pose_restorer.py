#!/usr/bin/env python3
import json, os, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

FILE_PATH = os.path.expanduser("~/.ros/last_amcl_pose.json")

class InitialPoseRestorer(Node):
    def __init__(self):
        super().__init__("initial_pose_restorer")
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        self.data = None
        if os.path.exists(FILE_PATH):
            try:
                with open(FILE_PATH, "r") as f:
                    self.data = json.load(f)
            except Exception as e:
                self.get_logger().error(f"Kan pose-file niet lezen: {e}")

        self.sent = 0
        self.max_sends = 8          # publish meerdere keren
        self.period = 1.0
        self.timer = self.create_timer(self.period, self.tick)

    def tick(self):
        if not self.data:
            self.get_logger().warn("Geen opgeslagen pose gevonden; skip /initialpose.")
            self.timer.cancel()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.data.get("frame_id", "map")
        msg.header.stamp = self.get_clock().now().to_msg()

        pose = self.data["pose"]
        p = pose["position"]
        o = pose["orientation"]

        msg.pose.pose.position.x = float(p["x"])
        msg.pose.pose.position.y = float(p["y"])
        msg.pose.pose.position.z = float(p.get("z", 0.0))
        msg.pose.pose.orientation.x = float(o["x"])
        msg.pose.pose.orientation.y = float(o["y"])
        msg.pose.pose.orientation.z = float(o["z"])
        msg.pose.pose.orientation.w = float(o["w"])

        # Covariance: niet té zeker, AMCL moet nog kunnen "locken"
        cov = [0.0] * 36
        cov[0]  = 0.25  # x var  (std ~0.5m)
        cov[7]  = 0.25  # y var
        cov[35] = 0.5   # yaw var (ruimer)
        msg.pose.covariance = cov

        self.pub.publish(msg)
        self.sent += 1
        self.get_logger().info(f"/initialpose gepubliceerd ({self.sent}/{self.max_sends})")

        if self.sent >= self.max_sends:
            self.timer.cancel()

def main():
    rclpy.init()
    node = InitialPoseRestorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
