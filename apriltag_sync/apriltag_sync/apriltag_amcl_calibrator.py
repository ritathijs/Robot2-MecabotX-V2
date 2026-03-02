import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_geometry_msgs
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf2_ros import TransformException, Buffer, TransformListener


class AprilTagAmclCalibrator(Node):

    def __init__(self):
        super().__init__('apriltag_amcl_calibrator')

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ID mapping (hard gekoppeld aan jouw TF boom)
        self.tag_ids = [0, 1]

        self.cooldown_seconds = 5.0
        self.last_calibrated = self.get_clock().now()

        self.timer = self.create_timer(0.5, self.check_for_tags)

        self.get_logger().info("AprilTag AMCL Calibrator gestart.")

    def check_for_tags(self):

        elapsed = (
            self.get_clock().now() - self.last_calibrated
        ).nanoseconds / 1e9

        if elapsed < self.cooldown_seconds:
            return

        for tag_id in self.tag_ids:

            detected_frame = f"tag36h11:{tag_id}"
            static_frame = f"static_tag36h11:{tag_id}"

            if self.try_calibration(detected_frame, static_frame):
                self.last_calibrated = self.get_clock().now()
                break

    def try_calibration(self, detected_tag, static_tag):

        try:
            now = Time()

            if not self.tf_buffer.can_transform(
                detected_tag,
                'base_footprint',
                now
            ):
                return False

            if not self.tf_buffer.can_transform(
                'map',
                static_tag,
                now
            ):
                return False

            pose_in_base = PoseStamped()
            pose_in_base.header.frame_id = 'base_footprint'
            pose_in_base.header.stamp = now.to_msg()
            pose_in_base.pose.orientation.w = 1.0

            pose_in_detected_tag = self.tf_buffer.transform(
                pose_in_base,
                detected_tag
            )

            pose_in_detected_tag.header.frame_id = static_tag

            pose_in_map = self.tf_buffer.transform(
                pose_in_detected_tag,
                'map'
            )

            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = now.to_msg()
            msg.pose.pose = pose_in_map.pose

            msg.pose.covariance[0] = 0.05
            msg.pose.covariance[7] = 0.05
            msg.pose.covariance[35] = 0.05

            self.pose_pub.publish(msg)

            self.get_logger().info(
                f"Kalibratie via {detected_tag}"
            )

            return True

        except TransformException:
            return False


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagAmclCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
