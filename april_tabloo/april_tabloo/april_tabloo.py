import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

from cv_bridge import CvBridge
from dt_apriltags import Detector

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
import json
import math
import time
import os


def quat_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class AprilTagAbsolutePose(Node):

    def __init__(self):
        super().__init__('apriltag_absolute_pose')

        self.bridge = CvBridge()

        self.latest_frame = None
        self.camera_matrix = None
        self.dist_coeffs = None

        # Maximale tijd (s) die we blokkerend willen wachten op subscribers
        # voordat we het opgeven i.p.v. voor altijd te blijven hangen.
        self.subscriber_wait_timeout = 10.0

        # Maximale tijd (s) die een reset-poging in totaal mag duren,
        # ongeacht of er al camera-frames binnenkomen. Dit voorkomt dat
        # de node oneindig blijft hangen als de camera bij startup nog
        # geen beeld levert (attempts telt dan anders nooit mee).
        self.reset_timeout = 5.0
        self.reset_start_time = None

        ##############################################
        # Status file
        ##############################################

        self.status_file = "/home/wheeltec/wheeltec_ros2/src/april_tabloo/status.txt"

        # eerste opdracht nog niet uitgevoerd
        self.first_reset_done = False

        # Bij startup altijd NOK schrijven
        self.write_status("NOK")

        self.done_sent = False

        ##############################################
        # Reset state
        ##############################################

        self.reset_active = False
        self.max_attempts = 5
        self.attempts = 0

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            String,
            "reset_active",
            self.reset_callback,
            qos
        )

        try:
            self.detector = Detector(
                families='tag36h11',
                nthreads=1,
                quad_decimate=1.0,
                refine_edges=1
            )
        except Exception as e:
            self.get_logger().error(f"Kon AprilTag Detector niet initialiseren: {e}")
            raise

        self.screen_pub = self.create_publisher(
            String,
            "/rpitopic",
            10
        )

        self.done_pub = self.create_publisher(
            String,
            "reset_done",
            qos
        )

        self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            10
        )

        self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self.camera_info_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        ##############################################
        # Tags laden (meerdere tags mogelijk)
        ##############################################
        # self.tags_data wordt een dict: { tag_id (int): {...} }
        # zodat we in timer_callback per gedetecteerde tag kunnen
        # opzoeken of het een gekende tag is, en zo ja met welke
        # locatie/oriëntatie/grootte die overeenkomt.

        tags_file = "/home/wheeltec/wheeltec_ros2/src/mecabot_bt/trees/tags.json"
        self.tags_data = {}

        try:
            with open(tags_file) as f:
                data = json.load(f)

            for entry in data["tags"]:

                tag_id = int(entry["id"])

                tag_x = float(entry["p_x"])
                tag_y = float(entry["p_y"])

                tag_yaw = 2 * math.atan2(
                    entry["orien_z"],
                    entry["orien_w"]
                )

                tag_size = float(entry["tag_size"])

                arrow_dir = np.array([
                    math.cos(tag_yaw),
                    math.sin(tag_yaw)
                ])

                right_dir = np.array([
                    -math.sin(tag_yaw),
                    math.cos(tag_yaw)
                ])

                self.tags_data[tag_id] = {
                    "x": tag_x,
                    "y": tag_y,
                    "yaw": tag_yaw,
                    "size": tag_size,
                    "arrow_dir": arrow_dir,
                    "right_dir": right_dir,
                }

            if not self.tags_data:
                raise ValueError("Geen tags gevonden in tags.json")

        except Exception as e:
            self.get_logger().error(
                f"Kon tags.json niet laden/parsen ({e}). "
                f"Val terug op tag id 0 met x=0, y=0, yaw=0, size=0.08."
            )
            self.tags_data = {
                0: {
                    "x": 0.0,
                    "y": 0.0,
                    "yaw": 0.0,
                    "size": 0.08,
                    "arrow_dir": np.array([1.0, 0.0]),
                    "right_dir": np.array([0.0, 1.0]),
                }
            }

        self.get_logger().info(
            f"Node started, gekende tag-ids: {list(self.tags_data.keys())}"
        )

    ##########################################################
    # Status schrijven
    ##########################################################

    def write_status(self, status):

        try:

            with open(self.status_file, "w") as f:
                f.write(status)

            self.get_logger().info(
                f"Apriltag status geschreven: {status}"
            )

        except Exception as e:

            self.get_logger().error(
                f"Kon statusfile niet schrijven: {e}"
            )

    ##########################################################

    def send_screen(self, text):
        try:
            msg = String()
            msg.data = text

            self.screen_pub.publish(msg)

            self.get_logger().info(
                f"Scherm gestuurd: {text}"
            )
        except Exception as e:
            self.get_logger().error(f"send_screen mislukt: {e}")

    ##########################################################

    def reset_callback(self, msg):
        try:
            if msg.data.lower() == "active":

                self.reset_active = True
                self.attempts = 0
                self.reset_start_time = time.time()

                self.get_logger().info(
                    "Reset ACTIVE ontvangen."
                )

            else:

                self.reset_active = False

                self.get_logger().info(
                    "Reset NON-ACTIVE."
                )
        except Exception as e:
            self.get_logger().error(f"reset_callback fout: {e}")

    ##########################################################

    def camera_info_callback(self, msg):
        try:
            if self.camera_matrix is None:

                self.camera_matrix = np.array(
                    msg.k
                ).reshape(3, 3)

                self.dist_coeffs = np.array(
                    msg.d
                )

                self.get_logger().info(
                    "Camera intrinsics loaded"
                )
        except Exception as e:
            self.get_logger().error(f"camera_info_callback fout: {e}")

    ##########################################################

    def image_callback(self, msg):
        if not self.reset_active:
            return

        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(
                msg,
                "bgr8"
            )
        except Exception as e:
            self.get_logger().error(f"image_callback: kon frame niet converteren: {e}")

    ##########################################################

    def publish_done(self):
        try:
            if self.done_sent:
                self.get_logger().info(
                    "DONE al verstuurd, overslaan."
                )
                return

            start = time.time()
            while self.done_pub.get_subscription_count() == 0:
                if time.time() - start >= self.subscriber_wait_timeout:
                    self.get_logger().error(
                        f"Timeout: nog steeds geen subscribers op reset_done na "
                        f"{self.subscriber_wait_timeout:.0f}s. Publiceer toch."
                    )
                    break

                self.get_logger().info(
                    "Waiting for subscribers on /reset_done..."
                )

                time.sleep(0.1)

            msg = String()
            msg.data = "done"

            self.done_pub.publish(msg)

            self.done_sent = True

            self.get_logger().info(
                "Sent first DONE on /reset_done"
            )
        except Exception as e:
            self.get_logger().error(f"publish_done mislukt: {e}")

    ##########################################################

    def publish_pose(self, x, y, yaw):
        try:
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

            for i in range(5):
                msg.header.stamp = self.get_clock().now().to_msg()

                self.pose_pub.publish(msg)

                time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"publish_pose mislukt: {e}")

    ##########################################################

    def timer_callback(self):

        if not self.reset_active:
            return

        try:
            # Harde deadline voor de volledige resetpoging, ongeacht of er
            # al camera-frames binnenkomen. Voorkomt dat de node oneindig
            # blijft hangen als de camera bij startup nog geen beeld geeft.
            if self.reset_start_time is not None and \
                    (time.time() - self.reset_start_time) >= self.reset_timeout:

                self.get_logger().warn(
                    f"Reset-poging timeout na {self.reset_timeout:.0f}s "
                    f"(geen tag/frame gevonden)."
                )

                self.write_status("NOK")
                self.first_reset_done = True

                self.reset_active = False
                self.attempts = 0
                self.reset_start_time = None
                return

            if self.latest_frame is None:
                return

            if self.camera_matrix is None:
                return

            frame = self.latest_frame.copy()

            gray = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2GRAY
            )

            tags = self.detector.detect(gray)

            self.attempts += 1

            self.get_logger().info(
                f"Detectiepoging {self.attempts}/{self.max_attempts}"
            )

            for tag in tags:

                # Kijk of dit een gekende tag is (id 0, id 2, ...)
                # in plaats van te vergelijken met één vaste target_id.
                tag_info = self.tags_data.get(tag.tag_id)

                if tag_info is None:
                    continue

                tag_size = tag_info["size"]
                half = tag_size / 2.0

                obj = np.array([

                    [-half, -half, 0],
                    [half, -half, 0],
                    [half, half, 0],
                    [-half, half, 0]

                ], dtype=np.float64)

                img = tag.corners.astype(
                    np.float64
                )

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

                side = -float(tvec[0])

                if depth <= 0.0:
                    continue

                ####################################################
                # Positiebepaling
                ####################################################

                tag_x = tag_info["x"]
                tag_y = tag_info["y"]
                arrow_dir = tag_info["arrow_dir"]
                right_dir = tag_info["right_dir"]

                x = (
                    tag_x
                    + arrow_dir[0] * depth
                    + right_dir[0] * side
                )

                y = (
                    tag_y
                    + arrow_dir[1] * depth
                    + right_dir[1] * side
                )

                yaw = math.atan2(
                    arrow_dir[1],
                    arrow_dir[0]
                ) + math.pi

                ####################################################
                # Tag gevonden
                ####################################################

                self.publish_pose(
                    x,
                    y,
                    yaw
                )

                # Eerste reset:
                # altijd NOK schrijven na afloop
                if self.first_reset_done is False:

                    self.write_status("OK")
                    self.send_screen("RobotStartup")

                    time.sleep(0.1)

                    self.write_status("NOK")

                    self.first_reset_done = True

                else:

                    self.write_status("OK")

                self.get_logger().info(
                    f"Tag {tag.tag_id} gevonden. Reset beëindigd."
                )

                self.publish_done()

                self.reset_active = False
                self.attempts = 0
                self.reset_start_time = None

                return

            ########################################################
            # Geen tag gevonden na alle pogingen
            ########################################################

            if self.attempts >= self.max_attempts:

                self.get_logger().warn(
                    "Geen AprilTag gevonden na 5 pogingen."
                )

                # eerste opdracht of volgende opdrachten:
                # altijd NOK bij mislukking

                self.write_status("NOK")

                self.first_reset_done = True

                self.reset_active = False
                self.attempts = 0
                self.reset_start_time = None

        except Exception as e:
            self.get_logger().error(f"timer_callback fout: {e}")
            # Sluit de lopende resetpoging netjes af i.p.v. oneindig te blijven hangen
            self.write_status("NOK")
            self.reset_active = False
            self.attempts = 0
            self.reset_start_time = None


##############################################################


def main():

    rclpy.init()

    node = AprilTagAbsolutePose()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()
