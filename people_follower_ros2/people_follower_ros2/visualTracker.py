#!/usr/bin/env python3

# DIT WERKTTTTTT


from __future__ import division
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
import message_filters
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
from sensor_msgs.msg import Image
from turn_on_wheeltec_robot.msg import Position as PositionMsg


class PeopleFollowerNode(Node):
    def __init__(self):
        super().__init__('people_follower')

        # -----------------------------
        # Parameters (configurable)
        # -----------------------------
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')

        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('use_cuda', True)

        # Half FOVs (radians)
        self.declare_parameter('horizontal_half_fov', 0.5235987755982988)
        self.declare_parameter('vertical_half_fov',   0.43196898986859655)

        # Depth handling
        self.declare_parameter('depth_scale', 1.0)         # multiply raw depth to get meters
        self.declare_parameter('min_valid_distance', 0.5)  # m
        self.declare_parameter('max_valid_distance', 10.0)  # m
        self.declare_parameter('patch_radius_px', 6)       # sampling patch radius

        # OUTPUT conventions (to match your controller)
        self.declare_parameter('invert_angle_x', False)    # flip left/right sign
        self.declare_parameter('invert_angle_y', False)    # flip up/down sign
        self.declare_parameter('publish_distance_mm', True)# publish mm instead of meters (old code likely used mm)

        # Camera center pixel offsets (if camera is not perfectly centered)
        self.declare_parameter('center_offset_x_px', 0)
        self.declare_parameter('center_offset_y_px', 0)

        # Topics
        self.declare_parameter('annotated_topic', '/detected_image')
        self.declare_parameter('position_topic',  '/object_tracker/current_position')

        # Read params
        self.rgb_topic   = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.yolo_model  = self.get_parameter('yolo_model').value
        self.conf        = float(self.get_parameter('conf').value)
        self.use_cuda    = bool(self.get_parameter('use_cuda').value)

        self.h_fov = float(self.get_parameter('horizontal_half_fov').value)
        self.v_fov = float(self.get_parameter('vertical_half_fov').value)
        self.tan_h = math.tan(self.h_fov)
        self.tan_v = math.tan(self.v_fov)

        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.min_dist_m  = float(self.get_parameter('min_valid_distance').value)
        self.max_dist_m  = float(self.get_parameter('max_valid_distance').value)
        self.patch_r     = int(self.get_parameter('patch_radius_px').value)

        self.invert_x    = bool(self.get_parameter('invert_angle_x').value)
        self.invert_y    = bool(self.get_parameter('invert_angle_y').value)
        self.dist_mm_out = bool(self.get_parameter('publish_distance_mm').value)

        self.cx_off_px   = int(self.get_parameter('center_offset_x_px').value)
        self.cy_off_px   = int(self.get_parameter('center_offset_y_px').value)

        self.annotated_topic = self.get_parameter('annotated_topic').value
        self.position_topic  = self.get_parameter('position_topic').value

        # Model
        self.model = YOLO(self.yolo_model)
        if self.use_cuda:
            try:
                self.model.to('cuda')
                self.get_logger().info('Using CUDA for YOLO inference')
            except Exception as e:
                self.get_logger().warn(f'CUDA unavailable, falling back to CPU: {e}')

        # ROS I/O
        self.bridge = CvBridge()
        rgb_sub   = message_filters.Subscriber(self, Image, self.rgb_topic,   qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data)
        self.ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=30, slop=0.05)
        self.ts.registerCallback(self.on_synced)

        self.pub_annot = self.create_publisher(Image, self.annotated_topic, QoSProfile(depth=10))
        self.pub_pos   = self.create_publisher(PositionMsg, self.position_topic, QoSProfile(depth=10))
        self.pos_msg   = PositionMsg()

        self.get_logger().info(f'RGB: {self.rgb_topic}, Depth: {self.depth_topic}')
        self.get_logger().info(f'Annotated: {self.annotated_topic}, Position: {self.position_topic}')
        self.get_logger().info(f'publish_distance_mm={self.dist_mm_out}, invert_x={self.invert_x}, invert_y={self.invert_y}')

        self.img_h = None
        self.img_w = None

    def on_synced(self, rgb_msg: Image, depth_msg: Image):
        # RGB -> BGR for OpenCV/YOLO
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        if self.img_h is None or self.img_w is None:
            self.img_h, self.img_w = frame.shape[:2]

        # Depth to float meters
        depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        depth = self._normalize_depth(depth_raw)

        # YOLO people detection
        results = self.model.predict(source=frame, classes=[0], conf=self.conf, verbose=False)
        y = results[0]

        if y.boxes is None or len(y.boxes) == 0:
            self._publish_annotated(rgb_msg.header, y.plot())
            return

        # Pick closest person with valid depth
        closest = None  # (cx, cy, dist_m, idx)
        for i, box in enumerate(y.boxes):
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx = int(round((x1 + x2) / 2.0)) + self.cx_off_px
            cy = int(round((y1 + y2) / 2.0)) + self.cy_off_px
            cx = max(0, min(cx, self.img_w - 1))
            cy = max(0, min(cy, self.img_h - 1))

            dist_m = self._depth_at(depth, cx, cy, self.patch_r)
            if dist_m is None:
                continue
            if not (self.min_dist_m <= dist_m <= self.max_dist_m):
                continue
            if closest is None or dist_m < closest[2]:
                closest = (cx, cy, dist_m, i)

        annotated = y.plot()

        if closest is None:
            self._publish_annotated(rgb_msg.header, annotated)
            return

        cx, cy, dist_m, _ = closest

        # Angle convention (same as your old node but configurable flips)
        angle_x = self._pixel_to_angle_x(cx)
        angle_y = self._pixel_to_angle_y(cy)
        if self.invert_x:
            angle_x = -angle_x
        if self.invert_y:
            angle_y = -angle_y

        # Distance output units
        distance_out = dist_m * 1000.0 if self.dist_mm_out else dist_m

        # Publish
        self.pos_msg.angle_x = float(angle_x)
        self.pos_msg.angle_y = float(angle_y)
        self.pos_msg.distance = float(distance_out)
        self.pub_pos.publish(self.pos_msg)

        # Quick debug (one line, easy to watch)
        self.get_logger().info(f'cx={cx} cy={cy} angle_x={angle_x:.3f} angle_y={angle_y:.3f} dist={"{:.0f}mm".format(distance_out) if self.dist_mm_out else f"{distance_out:.2f}m"}')

        self._publish_annotated(rgb_msg.header, annotated)

    # ---------- helpers ----------
    def _normalize_depth(self, depth_raw):
        if depth_raw.dtype == np.uint16:
            depth = depth_raw.astype(np.float32)
            depth[depth == 0] = np.nan
            depth = (depth / 1000.0) * self.depth_scale  # -> meters
        elif depth_raw.dtype in (np.float32, np.float64):
            depth = depth_raw.astype(np.float32) * self.depth_scale  # meters
        else:
            depth = depth_raw.astype(np.float32) * self.depth_scale  # best effort
        return depth

    def _depth_at(self, depth_m: np.ndarray, cx: int, cy: int, r: int):
        h, w = depth_m.shape[:2]
        x1 = max(cx - r, 0); y1 = max(cy - r, 0)
        x2 = min(cx + r, w - 1); y2 = min(cy + r, h - 1)
        patch = depth_m[y1:y2+1, x1:x2+1]
        if patch.size == 0: return None
        vals = patch[~np.isnan(patch)]
        if vals.size == 0: return None
        return float(np.median(vals))

    def _pixel_to_angle_x(self, x_px: int):
        disp = 2.0 * (x_px / max(self.img_w, 1.0)) - 1.0
        return -math.atan(disp * self.tan_h)

    def _pixel_to_angle_y(self, y_px: int):
        disp = 2.0 * (y_px / max(self.img_h, 1.0)) - 1.0
        return -math.atan(disp * self.tan_v)

    def _publish_annotated(self, header, annotated_bgr: np.ndarray):
        msg = self.bridge.cv2_to_imgmsg(annotated_bgr, encoding='bgr8')
        msg.header = header
        self.pub_annot.publish(msg)


def main():
    rclpy.init()
    node = PeopleFollowerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

