#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class FreezeAMCL(Node):

    def __init__(self):
        super().__init__('freeze_amcl')

        self.freeze = False

        # QoS zoals de meeste lasers gebruiken
        qos = rclpy.qos.QoSProfile(depth=10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos
        )

        self.freeze_sub = self.create_subscription(
            Bool,
            '/freeze_localization',
            self.freeze_callback,
            10
        )

        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan_amcl',
            qos
        )

        self.get_logger().info("Freeze AMCL node gestart.")

    def freeze_callback(self, msg):
        if msg.data != self.freeze:
            self.freeze = msg.data

            if self.freeze:
                self.get_logger().warn("AMCL localization FROZEN")
            else:
                self.get_logger().info("AMCL localization ACTIVE")

    def scan_callback(self, msg):
        if not self.freeze:
            self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = FreezeAMCL()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
