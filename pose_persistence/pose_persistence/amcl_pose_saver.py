#!/usr/bin/env python3
import json, os, math, tempfile, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

FILE_PATH = os.path.expanduser("~/.ros/last_amcl_pose.json")

def yaw_from_quat(q):
    # yaw (Z) from quaternion
    siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def ang_diff(a, b):
    d = (a - b + math.pi) % (2*math.pi) - math.pi
    return abs(d)

class AmclPoseSaver(Node):
    def __init__(self):
        super().__init__("amcl_pose_saver")
        os.makedirs(os.path.dirname(FILE_PATH), exist_ok=True)

        self.last_saved = None
        self.last_write_time = 0.0

        # thresholds
        self.min_dist = 0.05          # 5 cm
        self.min_yaw = math.radians(3) # 3 deg
        self.min_period = 1.0         # schrijf max 1 Hz

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.cb, 10
        )

    def atomic_write_json(self, path, data):
        d = os.path.dirname(path)
        fd, tmp = tempfile.mkstemp(prefix=".tmp_pose_", dir=d)
        try:
            with os.fdopen(fd, "w") as f:
                json.dump(data, f, indent=2)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, path)  # atomic
        finally:
            if os.path.exists(tmp):
                try: os.remove(tmp)
                except: pass

    def cb(self, msg: PoseWithCovarianceStamped):
        now = time.time()
        if now - self.last_write_time < self.min_period:
            return

        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = yaw_from_quat(o)

        if self.last_saved is not None:
            dx = p.x - self.last_saved["x"]
            dy = p.y - self.last_saved["y"]
            dist = math.hypot(dx, dy)
            dyaw = ang_diff(yaw, self.last_saved["yaw"])
            if dist < self.min_dist and dyaw < self.min_yaw:
                return

        data = {
            "stamp_unix": now,
            "frame_id": msg.header.frame_id or "map",
            "pose": {
                "position": {"x": p.x, "y": p.y, "z": p.z},
                "orientation": {"x": o.x, "y": o.y, "z": o.z, "w": o.w},
                "yaw_rad": yaw,
            },
            "covariance": list(msg.pose.covariance),
        }

        self.atomic_write_json(FILE_PATH, data)
        self.last_saved = {"x": p.x, "y": p.y, "yaw": yaw}
        self.last_write_time = now

def main():
    rclpy.init()
    node = AmclPoseSaver()
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
