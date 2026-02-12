#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import cv_bridge

# HSV bereik voor zwart
LOWER_BLACK = np.array([0, 0, 0])
UPPER_BLACK = np.array([180, 255, 50])

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = cv_bridge.CvBridge()

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.active_sub = self.create_subscription(
            Bool, '/follower_active', self.active_callback, 10)

        self.is_active = False
        self.state = "IDLE"

        self.twist = Twist()
        self.start_time = None

        # -------- Snelheden --------
        self.forward_speed = 0.06
        self.backward_speed = -0.08
        self.steering_gain = 0.0015

        # -------- Tijden --------
        self.start_forward_time = 3.0
        self.on_line_forward_time = 2.0      # ⭐ verder vooruit op de lijn
        self.wait_before_backward_time = 3.0

        # -------- Strengere centrering --------
        self.center_tolerance = 5             # pixels
        self.center_required = 10             # opeenvolgende frames
        self.center_count = 0

    def active_callback(self, msg):
        self.is_active = msg.data

        if not self.is_active:
            self.stop_robot()
            self.state = "IDLE"
            self.get_logger().info("Robot gestopt.")
        else:
            self.state = "START_FORWARD"
            self.start_time = time.time()
            self.center_count = 0
            self.get_logger().info("Geactiveerd: eerst vooruit rijden...")

    def stop_robot(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def image_callback(self, msg):
        if not self.is_active:
            return

        # -------- START: eerst vooruit --------
        if self.state == "START_FORWARD":
            self.twist.linear.x = self.forward_speed
            self.twist.angular.z = 0.0

            if time.time() - self.start_time > self.start_forward_time:
                self.state = "FORWARD"
                self.get_logger().info("Startpositie bereikt, lijn volgen...")
            self.cmd_vel_pub.publish(self.twist)
            return

        # -------- Beeldverwerking --------
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, LOWER_BLACK, UPPER_BLACK)

        h, w, _ = image.shape
        mask[0:h-40, :] = 0

        M = cv2.moments(mask)

        if M['m00'] == 0:
            self.stop_robot()
            self.center_count = 0
            return

        cx = int(M['m10'] / M['m00'])
        erro = cx - w / 2 - 60

        factor = 1

        # -------- STATE MACHINE --------
        if self.state == "FORWARD":
            self.twist.linear.x = self.forward_speed
            factor = 1

            if abs(erro) < self.center_tolerance:
                self.center_count += 1
            else:
                self.center_count = 0

            if self.center_count >= self.center_required:
                self.state = "ON_LINE_FORWARD"
                self.start_time = time.time()
                self.center_count = 0
                self.get_logger().info("Stabiel gecentreerd, extra vooruit rijden...")

        elif self.state == "ON_LINE_FORWARD":
            self.twist.linear.x = self.forward_speed
            factor = 1

            # Als hij te veel afwijkt → terug naar FORWARD
            if abs(erro) > self.center_tolerance:
                self.state = "FORWARD"
                self.center_count = 0
                self.get_logger().info("Afwijking gedetecteerd, opnieuw centreren...")
                return

            if time.time() - self.start_time > self.on_line_forward_time:
                self.state = "WAIT_BEFORE_BACKWARD"
                self.start_time = time.time()
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.get_logger().info("Extra vooruit klaar, wachten...")

        elif self.state == "WAIT_BEFORE_BACKWARD":
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

            if time.time() - self.start_time > self.wait_before_backward_time:
                self.state = "BACKWARD"
                self.get_logger().info("Nu achteruit rijden...")
            self.cmd_vel_pub.publish(self.twist)
            return

        elif self.state == "BACKWARD":
            self.twist.linear.x = self.backward_speed
            factor = -1

        # -------- STURING --------
        self.twist.angular.z = -(float(erro) * self.steering_gain) * factor
        self.cmd_vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    follower = LineFollower()
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

