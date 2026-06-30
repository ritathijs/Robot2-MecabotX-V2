import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped

from cv_bridge import CvBridge
from dt_apriltags import Detector

import cv2
import numpy as np
import json
import math


def quat_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class AprilTagAbsolutePose(Node):

    def __init__(self):
        super().__init__('apriltag_absolute_pose')

        self.declare_parameter("target_id", 0)
        self.declare_parameter("tag_size", 0.08)

        self.target_id = int(self.get_parameter("target_id").value)
        self.tag_size = float(self.get_parameter("tag_size").value)

        self.bridge = CvBridge()
        self.latest_frame = None

        self.camera_matrix = None
        self.dist_coeffs = None

        self.last_publish_time = self.get_clock().now()

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            refine_edges=1
        )

        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        with open("/home/wheeltec/wheeltec_ros2/src/april_tabloo/tags.json") as f:
            data = json.load(f)

        self.tag_x = float(data["p_x"])
        self.tag_y = float(data["p_y"])

        self.tag_yaw = 2 * math.atan2(data["orien_z"], data["orien_w"])

        self.arrow_dir = np.array([
            math.cos(self.tag_yaw),
            math.sin(self.tag_yaw)
        ])

        # rechtervector (wereldframe)
        self.right_dir = np.array([
            -math.sin(self.tag_yaw),
            math.cos(self.tag_yaw)
        ])

        self.get_logger().info("Node started: corrected left/right + forward pose")

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera intrinsics loaded")

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def publish_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = quat_from_yaw(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.05

        self.pose_pub.publish(msg)

        self.get_logger().info(
            f"POSE UPDATED: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )

    def timer_callback(self):
        if self.latest_frame is None or self.camera_matrix is None:
            return

        frame = self.latest_frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tags = self.detector.detect(gray)

        for tag in tags:
            if tag.tag_id != self.target_id:
                continue

            half = self.tag_size / 2.0

            obj = np.array([
                [-half, -half, 0],
                [half, -half, 0],
                [half, half, 0],
                [-half, half, 0]
            ], dtype=np.float64)

            img = tag.corners.astype(np.float64)

            ok, rvec, tvec = cv2.solvePnP(
                obj,
                img,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if not ok:
                continue

            tvec = tvec.reshape(3)

            depth = float(tvec[2])

            # ✅ FIX: spiegeling links/rechts corrigeren
            side = -float(tvec[0])

            if depth <= 0.0:
                continue

            x = self.tag_x + self.arrow_dir[0] * depth + self.right_dir[0] * side
            y = self.tag_y + self.arrow_dir[1] * depth + self.right_dir[1] * side

            yaw = math.atan2(self.arrow_dir[1], self.arrow_dir[0]) + math.pi

            now = self.get_clock().now()
            time_diff = (now - self.last_publish_time).nanoseconds / 1e9

            if time_diff >= 10.0:
                self.publish_pose(x, y, yaw)
                self.last_publish_time = now
            else:
                self.get_logger().info(
                    f"Detected. Waiting {10.0 - time_diff:.1f}s",
                    throttle_duration_sec=2.0
                )

            cv2.polylines(frame, [tag.corners.astype(int)], True, (0, 255, 0), 2)

        cv2.imshow("debug", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = AprilTagAbsolutePose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
