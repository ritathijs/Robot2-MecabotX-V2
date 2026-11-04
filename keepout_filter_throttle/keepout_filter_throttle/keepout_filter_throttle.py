import math
import os
import time
from functools import partial

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from keepout_filter_throttle import keepout_zone_tools as tools

DEFAULT_MAP_DIR = '/home/wheeltec/wheeltec_ros2_rt/src/wheeltec_robot_nav2/map'


class KeepoutToggler(Node):
    """Wisselt het keepout-masker en bewaakt dat de robot niet in de zone staat.

    * Zone AAN gebeurt pas als geen enkele zone-cel binnen ``guard_radius``
      van de robot ligt (anders staat de robot meteen in lethal terrein).
    * Bij voorkeur wordt het zachte masker geladen; ontbreekt of faalt dat,
      dan het oude binaire WORKING-masker.
    * Diagnose: logt de eigenschappen van het actieve masker en meldt wanneer
      de voetafdruk van de robot in lethale costmap-cellen staat.
    """

    def __init__(self):
        super().__init__('keepout_filter_throttle')

        self.declare_parameter('map_dir', DEFAULT_MAP_DIR)
        self.declare_parameter('guard_enabled', True)
        self.declare_parameter('guard_radius', 0.49)
        self.declare_parameter('escape_enabled', True)
        self.declare_parameter('escape_delay', 3.0)
        self.declare_parameter('diagnostics_enabled', True)
        self.declare_parameter('diagnostics_interval', 2.0)
        self.declare_parameter('diagnostics_log_period', 10.0)
        self.declare_parameter('footprint_half_x', 0.488)
        self.declare_parameter('footprint_half_y', 0.503)
        self.declare_parameter('mask_topic', '/keepout_filter_mask')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')

        map_dir = self.get_parameter('map_dir').value
        self.mask_empty = os.path.join(map_dir, 'WHEELTEC_KEEPOUT_SERVICE.yaml')
        mask_soft = os.path.join(map_dir, 'WHEELTEC_KEEPOUT_WORKING_SOFT.yaml')
        mask_legacy = os.path.join(map_dir, 'WHEELTEC_KEEPOUT_WORKING.yaml')
        self.mask_full_candidates = (
            [mask_soft, mask_legacy] if os.path.exists(mask_soft) else [mask_legacy])
        self.full_index = 0
        self.get_logger().info(
            f"Zone-masker: {self.mask_full_candidates[0]}"
            f"{' (met terugval op het binaire masker)' if len(self.mask_full_candidates) > 1 else ''}")

        # Launch laadt het SERVICE-masker (zone uit) bij het opstarten.
        self.desired = False
        self.applied = False
        self.escape = False
        self.in_zone_since = None
        self.request_in_flight = False
        self.pose = None
        self.zones = None
        self.diag_state = None
        self.last_diag_time = 0.0
        self.last_diag_log = 0.0

        self.subscription = self.create_subscription(Bool, '/toggle_keepout', self.toggle_callback, 10)
        self.client = self.create_client(LoadMap, '/keepout_map_server/load_map')

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        if not tools.TOOLS_AVAILABLE:
            self.get_logger().warn(
                "numpy/opencv/yaml ontbreken: guard en diagnose uitgeschakeld.")
        else:
            self._setup_zone_map()
            self.create_subscription(
                PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, latched)
            if self.get_parameter('diagnostics_enabled').value:
                self.create_subscription(
                    OccupancyGrid, self.get_parameter('mask_topic').value,
                    self.on_mask, latched)
                self.create_subscription(
                    OccupancyGrid, self.get_parameter('costmap_topic').value,
                    self.on_costmap, 1)

        self.create_timer(1.0, self._reconcile)

    def _setup_zone_map(self):
        try:
            self.zones = tools.ZoneMap(self.mask_full_candidates[0], self.mask_empty)
            self.get_logger().info(
                f"Zone-kaart geladen: {int(self.zones.zone.sum())} zone-cellen.")
        except Exception as exc:  # noqa: BLE001 - guard is optioneel
            self.get_logger().warn(
                f"Zone-kaart niet geladen ({exc}); guard en zone-diagnose uit.")

    def on_pose(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def toggle_callback(self, msg):
        self.desired = bool(msg.data)
        self.get_logger().info(
            f"Verzoek ontvangen: {'Zone AAN' if self.desired else 'Zone UIT'}")
        self._reconcile()

    def _zone_blocks_activation(self):
        if (self.zones is None or self.pose is None
                or not self.get_parameter('guard_enabled').value):
            return False
        x, y, _ = self.pose
        return self.zones.zone_within(
            x, y, self.get_parameter('guard_radius').value)

    def _update_escape(self):
        """Zone tijdelijk uit als de robot blijvend in lethaal zone-terrein staat.

        Eindigt automatisch zodra de robot buiten guard_radius van de zone is;
        daarna past _reconcile de gewenste zone-staat weer toe.
        """
        if (self.zones is None or self.pose is None
                or not self.get_parameter('escape_enabled').value):
            return
        x, y, _ = self.pose
        if self.escape:
            if not self._zone_blocks_activation():
                self.escape = False
                self.in_zone_since = None
                self.get_logger().info(
                    "Escape klaar: robot is vrij van de zone, zone-masker wordt hersteld.")
            return

        if not (self.applied and self.zones.zone_at(x, y)):
            self.in_zone_since = None
            return
        now = time.monotonic()
        if self.in_zone_since is None:
            self.in_zone_since = now
        elif now - self.in_zone_since >= self.get_parameter('escape_delay').value:
            self.escape = True
            self.get_logger().warn(
                f"Escape: robot staat al {now - self.in_zone_since:.0f} s in lethale zone "
                f"(x={x:.2f}, y={y:.2f}); zone tijdelijk UIT tot hij vrij is.")

    def _reconcile(self):
        self._update_escape()
        wanted = self.desired and not self.escape
        if self.request_in_flight or wanted == self.applied:
            return

        if wanted and self._zone_blocks_activation():
            self.get_logger().warn(
                "Zone AAN uitgesteld: de robot staat in of vlak bij de verboden zone "
                f"(pose x={self.pose[0]:.2f}, y={self.pose[1]:.2f}). "
                "Wordt toegepast zodra hij vrij is.",
                throttle_duration_sec=5.0)
            return

        if not self.client.service_is_ready():
            self.get_logger().warn(
                "keepout_map_server/load_map niet beschikbaar; opnieuw proberen.",
                throttle_duration_sec=5.0)
            return

        candidates = self.mask_full_candidates if wanted else [self.mask_empty]
        index = self.full_index if wanted else 0
        request = LoadMap.Request()
        request.map_url = candidates[index]

        self.request_in_flight = True
        self.get_logger().info(
            f"Masker laden: {'Zone AAN' if wanted else 'Zone UIT'} ({request.map_url})")
        future = self.client.call_async(request)
        future.add_done_callback(partial(self._on_loaded, wanted))

    def _on_loaded(self, state, future):
        self.request_in_flight = False
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Masker laden mislukt: {exc}")
            return

        if result.result == LoadMap.Response.RESULT_SUCCESS:
            self.applied = state
            return

        self.get_logger().error(
            f"Masker laden gaf foutcode {result.result}.")
        if state and len(self.mask_full_candidates) > 1:
            self.full_index = (self.full_index + 1) % len(self.mask_full_candidates)
            self.get_logger().warn(
                f"Volgende poging met {self.mask_full_candidates[self.full_index]}")

    def on_mask(self, msg):
        stats = tools.mask_statistics(msg.data)
        kind = 'ZACHT' if stats['zacht'] > 0 else 'BINAIR (geen tussenwaarden)'
        self.get_logger().info(
            f"[Keepout-diagnose] Masker ontvangen door de filter ({msg.info.width}x"
            f"{msg.info.height}, {msg.info.resolution} m): {kind}; "
            f"vrij={stats['vrij']} zacht={stats['zacht']} lethal={stats['lethal']} "
            f"onbekend={stats['onbekend']} waarden={stats['verschillende_waarden']}")

    def on_costmap(self, msg):
        if self.pose is None:
            return
        now = time.monotonic()
        if now - self.last_diag_time < self.get_parameter('diagnostics_interval').value:
            return
        self.last_diag_time = now

        try:
            report = tools.footprint_report(
                msg, self.pose,
                self.get_parameter('footprint_half_x').value,
                self.get_parameter('footprint_half_y').value,
                self.zones)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"Voetafdruk-diagnose mislukt: {exc}", throttle_duration_sec=30.0)
            return

        state = report['toestand']
        changed = state != self.diag_state
        period = self.get_parameter('diagnostics_log_period').value
        periodic = state != 'OK' and now - self.last_diag_log >= period
        self.diag_state = state
        if not (changed or periodic):
            return

        self.last_diag_log = now
        x, y, yaw = self.pose
        text = (
            f"[Keepout-diagnose] {state}: pose=({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} graden) "
            f"midden_kost={report['midden']} midden_in_zone={report['midden_in_zone']} "
            f"lethal_cellen_voetafdruk={report['lethal']} "
            f"waarvan_in_zone={report['lethal_in_zone']} "
            f"zone_actief={self.applied} gewenst={self.desired}")
        if state == 'OK':
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)

def main(args=None):
    rclpy.init(args=args)
    node = KeepoutToggler()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
