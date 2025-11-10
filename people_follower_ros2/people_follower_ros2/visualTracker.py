#!/usr/bin/env python3

# -----------------------------------------------------------------------------
# PeopleFollowerNode — detects people in RGB, samples depth at bbox center,
# converts to angles and (optionally) relative coords, and publishes.
# Includes simple "lock-on" so it sticks to the first target until it's lost.
#
# TUNABLES (quick):
# - Camera topics:                 rgb_topic, depth_topic
# - YOLO speed/accuracy:           yolo_model, conf, use_cuda
# - Camera FOV (radians):          horizontal_half_fov, vertical_half_fov
# - Depth handling:                depth_scale, min_valid_distance, max_valid_distance, patch_radius_px
# - Output signs/units:            invert_angle_x, invert_angle_y, publish_distance_mm
# - Camera principal-point tweak:  center_offset_x_px, center_offset_y_px
# - Target sticking (lock-on):     lock_on_enabled, lock_max_center_dist_px, lock_lost_timeout_frames,
#                                  initial_target_policy ('first'|'closest'|'central')
# - Synchronizer timing:           queue_size (in code), slop (in code, seconds)
# -----------------------------------------------------------------------------

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
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32


class PeopleFollowerNode(Node):
    def __init__(self):
        super().__init__('people_follower')

        # -----------------------------
        # Parameters (configurable)
        # -----------------------------
        # Topic names — set these to match your camera drivers. RealSense defaults often match.
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')

        # YOLO settings
        # - yolo_model: swap 'yolov8n.pt' (fastest) with 'yolov8s/m/l.pt' for more accuracy.
        # - conf: detection confidence threshold (typical 0.25–0.7). Higher → fewer false positives.
        # - use_cuda: True to use GPU if available; False forces CPU.
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('use_cuda', True)

        # Camera half-FOVs (radians)
        # If your cam has HFOV=60°, VFOV=50°:
        #   horizontal_half_fov = (60 * pi/180) / 2 ≈ 0.5236
        #   vertical_half_fov   = (50 * pi/180) / 2 ≈ 0.4363
        # Incorrect FOVs distort angles and derived relative coords.
        self.declare_parameter('horizontal_half_fov', 0.5235987755982988)  # ≈ 30°
        self.declare_parameter('vertical_half_fov',   0.43196898986859655) # ≈ 24.76°

        # Depth handling
        # - depth_scale: multiply raw depth (after conversion) to meters. Keep 1.0 for meters.
        #   For uint16 depth in millimeters we already divide by 1000 inside _normalize_depth().
        # - min/max_valid_distance (m): clamp target range to avoid background grabs.
        # - patch_radius_px: median depth over a (2r+1)^2 patch; increase if depth is noisy (8–12), but watch speed.
        self.declare_parameter('depth_scale', 1.0)
        self.declare_parameter('min_valid_distance', 0.5)
        self.declare_parameter('max_valid_distance', 10.0)
        self.declare_parameter('patch_radius_px', 6)

        # Output conventions
        # - invert_angle_x / invert_angle_y: flip signs to match your controller expectations.
        # - publish_distance_mm: true → PositionMsg.distance is in millimeters (legacy compatibility).
        self.declare_parameter('invert_angle_x', False)
        self.declare_parameter('invert_angle_y', False)
        self.declare_parameter('publish_distance_mm', True)

        # Camera center pixel offsets (principal point tweak)
        # If optical center is not the image center, nudge pixel center used for angles/depth.
        # Positive x shifts to the right; positive y shifts downward.
        self.declare_parameter('center_offset_x_px', 0)
        self.declare_parameter('center_offset_y_px', 0)

        # Output topics — change if your pipeline expects different names.
        self.declare_parameter('annotated_topic', '/detected_image')
        self.declare_parameter('position_topic',  '/object_tracker/current_position')

        # ---------------- Lock-on behavior parameters ----------------
        # Stick to the FIRST valid person, ignore newcomers unless the target is lost.
        # - lock_max_center_dist_px: max pixel jump allowed between frames for same target.
        #   If the person moves fast / camera shakes, increase (150–220). If identity swaps occur, reduce.
        # - lock_lost_timeout_frames: consecutive frames without a close match before giving up.
        #   At 30 FPS, 30 frames ≈ 1s. Raise (60–90) to be more forgiving to occlusions.
        # - initial_target_policy: when no lock: 'first' (detector order), 'closest' (by depth), 'central' (closest to image center).
        self.declare_parameter('lock_on_enabled', True)
        self.declare_parameter('lock_max_center_dist_px', 120)
        self.declare_parameter('lock_lost_timeout_frames', 30)
        self.declare_parameter('initial_target_policy', 'first')

        # ---------- Read params (resolved values) ----------
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

        # Lock params (resolved)
        self.lock_on_enabled = bool(self.get_parameter('lock_on_enabled').value)
        self.lock_max_center_dist_px = int(self.get_parameter('lock_max_center_dist_px').value)
        self.lock_lost_timeout_frames = int(self.get_parameter('lock_lost_timeout_frames').value)
        self.initial_target_policy = str(self.get_parameter('initial_target_policy').value)

        # ---------- Model ----------
        self.model = YOLO(self.yolo_model)
        if self.use_cuda:
            try:
                self.model.to('cuda')
                self.get_logger().info('Using CUDA for YOLO inference')
            except Exception as e:
                self.get_logger().warn(f'CUDA unavailable, falling back to CPU: {e}')

        # ---------- ROS I/O ----------
        self.bridge = CvBridge()
        # Synchronizer tuning: queue_size (latency vs. robustness), slop (seconds) tolerance.
        rgb_sub   = message_filters.Subscriber(self, Image, self.rgb_topic,   qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data)
        self.ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=30, slop=0.05)
        self.ts.registerCallback(self.on_synced)

        # Publishers — external API unchanged (angles + distance in PositionMsg)
        self.pub_annot = self.create_publisher(Image, self.annotated_topic, QoSProfile(depth=10))
        self.pub_pos   = self.create_publisher(PositionMsg, self.position_topic, QoSProfile(depth=10))
        self.pos_msg   = PositionMsg()

        # ------------------------------------------------------------------
        # Relative coordinates (x,y,z) in camera frame as PointStamped
        # Distance-only topic as Float32 (meters)
        # self.pub_rel_point = self.create_publisher(PointStamped, '/target_relative', QoSProfile(depth=10))
        # self.pub_distance  = self.create_publisher(Float32,      '/target_distance',  QoSProfile(depth=10))
        # ------------------------------------------------------------------

        self.get_logger().info(f'RGB: {self.rgb_topic}, Depth: {self.depth_topic}')
        self.get_logger().info(f'Annotated: {self.annotated_topic}, Position: {self.position_topic}')
        self.get_logger().info(f'publish_distance_mm={self.dist_mm_out}, invert_x={self.invert_x}, invert_y={self.invert_y}')

        self.img_h = None
        self.img_w = None

        # Lock state — do not modify directly; tune via lock_* params.
        self.lock_active = False
        self.lock_cx = None
        self.lock_cy = None
        self.lock_miss_count = 0

    def on_synced(self, rgb_msg: Image, depth_msg: Image):
        # Convert RGB and cache image size once.
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        if self.img_h is None or self.img_w is None:
            self.img_h, self.img_w = frame.shape[:2]

        # Depth normalization to meters.
        depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        depth = self._normalize_depth(depth_raw)

        # Run YOLO on the RGB frame (classes=[0] restricts to 'person').
        results = self.model.predict(source=frame, classes=[0], conf=self.conf, verbose=False)
        y = results[0]

        annotated = y.plot()  # Visualization image (detections drawn)

        # Build candidate list: (cx, cy, dist_m, idx) — keep only candidates with valid depth in range.
        candidates = []
        if y.boxes is not None and len(y.boxes) > 0:
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
                candidates.append((cx, cy, dist_m, i))

        # -------- Lock-on selection logic --------
        chosen = None
        if self.lock_on_enabled and self.lock_active and len(candidates) > 0:
            # Prefer the candidate nearest to the previous lock center; reject large jumps.
            best = None
            best_d2 = None
            for (cx, cy, dist_m, i) in candidates:
                d2 = (cx - self.lock_cx)**2 + (cy - self.lock_cy)**2
                if best is None or d2 < best_d2:
                    best = (cx, cy, dist_m, i)
                    best_d2 = d2
            if best_d2 is not None and best_d2 <= (self.lock_max_center_dist_px ** 2):
                chosen = best
                self.lock_miss_count = 0
            else:
                # No nearby candidate — increment miss counter and possibly drop lock.
                self.lock_miss_count += 1
                if self.lock_miss_count > self.lock_lost_timeout_frames:
                    self.lock_active = False
                    self.lock_cx = None
                    self.lock_cy = None

        if chosen is None and len(candidates) > 0:
            # Acquire (or re-acquire) when no active lock.
            if not self.lock_on_enabled or not self.lock_active:
                policy = self.initial_target_policy.lower()
                if policy == 'closest':
                    chosen = min(candidates, key=lambda t: t[2])
                elif policy == 'central':
                    cx0, cy0 = self.img_w // 2, self.img_h // 2
                    chosen = min(candidates, key=lambda t: (t[0]-cx0)**2 + (t[1]-cy0)**2)
                else:  # 'first' (default): detector order this frame
                    chosen = candidates[0]
                if self.lock_on_enabled:
                    self.lock_active = True
                    self.lock_cx, self.lock_cy = chosen[0], chosen[1]
                    self.lock_miss_count = 0

        if chosen is None:
            # Nothing valid to publish this frame (or lock temporarily lost).
            self._publish_annotated(rgb_msg.header, annotated)
            return

        cx, cy, dist_m, _ = chosen

        # Update lock center (follows the target). Circle for visual debug.
        if self.lock_on_enabled and self.lock_active:
            self.lock_cx, self.lock_cy = cx, cy
            try:
                cv2.circle(annotated, (int(cx), int(cy)), 6, (0, 255, 0), 2)
            except Exception:
                pass

        # Compute angles from pixel center.
        angle_x = self._pixel_to_angle_x(cx)
        angle_y = self._pixel_to_angle_y(cy)
        if self.invert_x:
            angle_x = -angle_x
        if self.invert_y:
            angle_y = -angle_y

        # Optional relative coordinates in camera frame (meters).
        rel_x = dist_m * math.tan(angle_x)  # +X right
        rel_y = dist_m * math.tan(angle_y)  # +Y down
        rel_z = dist_m                      # +Z forward

        # Position message distance units (legacy: mm if publish_distance_mm).
        distance_out = dist_m * 1000.0 if self.dist_mm_out else dist_m

        # Publish angles + distance to PositionMsg (unchanged external API).
        self.pos_msg.angle_x = float(angle_x)
        self.pos_msg.angle_y = float(angle_y)
        self.pos_msg.distance = float(distance_out)
        self.pub_pos.publish(self.pos_msg)

        # ------------------------------------------------------------------
        # publishes for relative coordinates + distance-only
        
        # pt = PointStamped()
        # pt.header = rgb_msg.header
        # pt.point.x = float(rel_x)   # meters, +X right
        # pt.point.y = float(rel_y)   # meters, +Y down
        # pt.point.z = float(rel_z)   # meters, +Z forward
        # self.pub_rel_point.publish(pt)
        #
        # self.pub_distance.publish(Float32(data=float(dist_m)))  # meters only
        # ------------------------------------------------------------------

        # Log line for quick tuning. Shows lock status and miss counter.
        lock_info = f" lock={'ON' if (self.lock_on_enabled and self.lock_active) else 'OFF'} miss={self.lock_miss_count}"
        self.get_logger().info(
            f'cx={cx} cy={cy} angle_x={angle_x:.3f} angle_y={angle_y:.3f} '
            f'dist={"{:.0f}mm".format(distance_out) if self.dist_mm_out else f"{distance_out:.2f}m"} '
            f'rel=({rel_x:.2f},{rel_y:.2f},{rel_z:.2f})m' + lock_info
        )

        self._publish_annotated(rgb_msg.header, annotated)

    # ---------- helpers ----------
    def _normalize_depth(self, depth_raw):
        """Convert incoming depth image to meters.
        - uint16 (e.g., RealSense): 0→NaN, divide by 1000 to get meters, scale by depth_scale.
        - float32/64: multiply by depth_scale directly.
        """
        if depth_raw.dtype == np.uint16:
            depth = depth_raw.astype(np.float32)
            depth[depth == 0] = np.nan
            depth = (depth / 1000.0) * self.depth_scale
        elif depth_raw.dtype in (np.float32, np.float64):
            depth = depth_raw.astype(np.float32) * self.depth_scale
        else:
            depth = depth_raw.astype(np.float32) * self.depth_scale  # best-effort fallback
        return depth

    def _depth_at(self, depth_m: np.ndarray, cx: int, cy: int, r: int):
        """Median depth (m) in a (2r+1)x(2r+1) patch around (cx,cy).
        Increase r if you have speckle noise; decrease for sharper but noisier readings.
        """
        h, w = depth_m.shape[:2]
        x1 = max(cx - r, 0); y1 = max(cy - r, 0)
        x2 = min(cx + r, w - 1); y2 = min(cy + r, h - 1)
        patch = depth_m[y1:y2+1, x1:x2+1]
        if patch.size == 0: return None
        vals = patch[~np.isnan(patch)]
        if vals.size == 0: return None
        return float(np.median(vals))

    def _pixel_to_angle_x(self, x_px: int):
        # Map pixel x to angle (rad) using half-FOV; negative sign matches chosen convention.
        disp = 2.0 * (x_px / max(self.img_w, 1.0)) - 1.0
        return -math.atan(disp * self.tan_h)

    def _pixel_to_angle_y(self, y_px: int):
        # Map pixel y to angle (rad) using half-FOV; negative sign matches chosen convention.
        disp = 2.0 * (y_px / max(self.img_h, 1.0)) - 1.0
        return -math.atan(disp * self.tan_v)

    def _publish_annotated(self, header, annotated_bgr: np.ndarray):
        # Publish the annotated BGR image.
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

