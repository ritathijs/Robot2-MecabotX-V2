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

        self.declare_parameter("target_id", 0)
        self.declare_parameter("tag_size", 0.08)

        self.target_id = int(self.get_parameter("target_id").value)
        self.tag_size = float(self.get_parameter("tag_size").value)

        self.bridge = CvBridge()

        self.latest_frame = None
        self.camera_matrix = None
        self.dist_coeffs = None


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


        self.detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            refine_edges=1
        )


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


        with open(
            "/home/wheeltec/wheeltec_ros2/src/mecabot_bt/trees/tags.json"
        ) as f:

            data = json.load(f)


        self.tag_x = float(data["p_x"])
        self.tag_y = float(data["p_y"])


        self.tag_yaw = 2 * math.atan2(
            data["orien_z"],
            data["orien_w"]
        )


        self.arrow_dir = np.array([
            math.cos(self.tag_yaw),
            math.sin(self.tag_yaw)
        ])


        self.right_dir = np.array([
            -math.sin(self.tag_yaw),
            math.cos(self.tag_yaw)
        ])


        self.get_logger().info("Node started")


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

        ##########################################################

    def send_screen(self, text):

        msg = String()
        msg.data = text

        self.screen_pub.publish(msg)

        self.get_logger().info(
            f"Scherm gestuurd: {text}"
        )

    ##########################################################


    def reset_callback(self, msg):

        if msg.data.lower() == "active":

            self.reset_active = True
            self.attempts = 0

            self.get_logger().info(
                "Reset ACTIVE ontvangen."
            )

        else:

            self.reset_active = False

            self.get_logger().info(
                "Reset NON-ACTIVE."
            )


    ##########################################################

    def camera_info_callback(self, msg):

        if self.camera_matrix is None:

            self.camera_matrix = np.array(
                msg.k
            ).reshape(3,3)

            self.dist_coeffs = np.array(
                msg.d
            )

            self.get_logger().info(
                "Camera intrinsics loaded"
            )


    ##########################################################

    def image_callback(self,msg):

        if not self.reset_active:
            return

        self.latest_frame = self.bridge.imgmsg_to_cv2(
            msg,
            "bgr8"
        )


    ##########################################################
    
    def publish_done(self):

        if self.done_sent:
            self.get_logger().info(
                "DONE al verstuurd, overslaan."
            )
            return


        while self.done_pub.get_subscription_count() == 0:

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

    ##########################################################

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


        for i in range(5):

            msg.header.stamp = self.get_clock().now().to_msg()

            self.pose_pub.publish(msg)

            time.sleep(0.1)



    ##########################################################

    def timer_callback(self):

        if not self.reset_active:
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


            if tag.tag_id != self.target_id:
                continue


            half = self.tag_size / 2.0


            obj = np.array([

                [-half, -half, 0],
                [ half, -half, 0],
                [ half,  half, 0],
                [-half,  half, 0]

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

            x = (
                self.tag_x
                + self.arrow_dir[0] * depth
                + self.right_dir[0] * side
            )


            y = (
                self.tag_y
                + self.arrow_dir[1] * depth
                + self.right_dir[1] * side
            )


            yaw = math.atan2(
                self.arrow_dir[1],
                self.arrow_dir[0]
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
                "Tag gevonden. Reset beëindigd."
            )


            self.publish_done()


            self.reset_active = False
            self.attempts = 0


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
