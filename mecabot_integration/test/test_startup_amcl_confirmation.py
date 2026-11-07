import ast
from copy import deepcopy
from dataclasses import dataclass
import math
from pathlib import Path
from threading import Lock
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


SOURCE = Path(__file__).resolve().parents[1] / "scripts" / "robot_state_manager.py"


def load_state_manager():
    parsed = ast.parse(SOURCE.read_text())
    classes = [
        item for item in parsed.body
        if isinstance(item, ast.ClassDef)
        and item.name in ("RobotState", "RobotStateManager")
    ]
    module = ast.Module(
        body=[
            ast.ImportFrom(
                module="__future__",
                names=[ast.alias(name="annotations")],
                level=0,
            ),
            *classes,
        ],
        type_ignores=[],
    )
    namespace = {
        "Node": object,
        "dataclass": dataclass,
        "deepcopy": deepcopy,
        "math": math,
        "time": time,
        "String": lambda: SimpleNamespace(data=""),
        "PoseWithCovarianceStamped": lambda: pose(),
    }
    exec(compile(ast.fix_missing_locations(module), str(SOURCE), "exec"), namespace)
    return namespace["RobotState"], namespace["RobotStateManager"]


RobotState, RobotStateManager = load_state_manager()


def pose(x=3.0, y=4.0, yaw=0.0, stamp=0):
    q = SimpleNamespace(
        x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)
    )
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = covariance[35] = 0.01
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id="map", stamp=SimpleNamespace(sec=stamp, nanosec=0)
        ),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y), orientation=q
            ),
            covariance=covariance,
        ),
    )


class StartupAmclConfirmationTest(unittest.TestCase):
    def setUp(self):
        self.node = RobotStateManager.__new__(RobotStateManager)
        self.node.state = RobotState(
            battery_status_received=True,
            charging_current_received=True,
            infrared_status_received=True,
            camera_info_received=True,
            localization_recovery_requested=True,
            localization_recovery_started_at=100.0,
        )
        self.node.lock = Lock()
        self.node.apriltag_location_anchors = {
            "work_area": (3.0, 4.0), "charging_area": (12.0, -16.0)
        }
        self.node.startup_started_at = 100.0
        self.node._startup_failed_since = None
        self.node._startup_ready_since = None
        self.node.startup_dock_fallback = False
        self.node.dock_pose = (12.0, -16.0, 0.0, 1.0)
        self.node.initial_pose_pub = Mock()
        self.node.cmd_pub = Mock()
        self.node.screen_pub = Mock()
        self.node.cmd_pub.get_subscription_count.return_value = 0
        self.node.get_logger = Mock(return_value=Mock())
        self.node._publish_status_snapshot = Mock()
        self.node.get_clock = Mock(
            return_value=SimpleNamespace(
                now=lambda: SimpleNamespace(
                    to_msg=lambda: SimpleNamespace(sec=200, nanosec=0)
                )
            )
        )

    def start_confirmation(self, done_first=False):
        with patch.object(time, "monotonic", return_value=101.0):
            if done_first:
                self.node.on_localization_reset(SimpleNamespace(data="done"))
            self.node.on_startup_initial_pose(pose())
            if not done_first:
                self.node.on_localization_reset(SimpleNamespace(data="done"))

    def tick(self, now):
        with patch.object(time, "monotonic", return_value=now):
            self.node._startup_location_check()

    def test_requires_fresh_matching_amcl_after_republishing(self):
        for done_first in (False, True):
            with self.subTest(done_first=done_first):
                self.setUp()
                self.start_confirmation(done_first)
                self.node.on_amcl(pose(stamp=201))
                self.assertFalse(self.node.state.startup_location_resolved)
                self.tick(102.0)
                self.assertEqual(self.node.initial_pose_pub.publish.call_count, 1)
                for sample in (
                    pose(stamp=200),
                    pose(x=0.0, y=0.0, stamp=201),
                    pose(yaw=math.pi, stamp=201),
                ):
                    self.node.on_amcl(sample)
                    self.assertFalse(self.node.state.startup_location_resolved)
                invalid_covariance = pose(stamp=201)
                invalid_covariance.pose.covariance[0] = float("nan")
                self.node.on_amcl(invalid_covariance)
                self.assertFalse(self.node.state.startup_location_resolved)
                wrong_frame = pose(stamp=201)
                wrong_frame.header.frame_id = "odom"
                self.node.on_amcl(wrong_frame)
                self.assertFalse(self.node.state.startup_location_resolved)
                self.tick(103.1)
                self.assertEqual(self.node.initial_pose_pub.publish.call_count, 2)
                self.node.on_amcl(pose(stamp=201))
                self.assertTrue(self.node.state.startup_location_resolved)
                self.assertTrue(self.node.state.at_work_area)
                self.assertTrue(self.node.state.robot_active)

    def test_timeout_blocks_start_and_rejects_late_confirmation(self):
        self.start_confirmation()
        self.tick(102.0)
        self.tick(132.0)
        self.assertTrue(self.node.state.startup_location_failed)
        self.assertFalse(self.node.state.startup_location_resolved)
        self.assertFalse(self.node.state.robot_active)
        self.node.on_amcl(pose(stamp=201))
        self.assertFalse(self.node.state.startup_location_resolved)
        self.node.on_localization_reset(SimpleNamespace(data="done"))
        self.assertFalse(self.node.state.localization_valid)

    def test_invalid_initial_pose_does_not_start_confirmation(self):
        msg = pose()
        msg.header.frame_id = "odom"
        self.node.on_startup_initial_pose(msg)
        self.node.on_localization_reset(SimpleNamespace(data="done"))
        self.assertTrue(self.node.state.startup_location_failed)
        self.assertFalse(self.node.state.robot_active)

    def test_external_activation_is_ignored_before_confirmation(self):
        self.node.on_robot_active(SimpleNamespace(data=True))
        self.assertFalse(self.node.state.robot_active)
        self.node.handle_socket_event("active_button_toggled", (None, None))
        self.assertFalse(self.node.state.robot_active)

    def test_duplicate_done_does_not_restart_confirmation_timeout(self):
        self.start_confirmation()
        with patch.object(time, "monotonic", return_value=125.0):
            self.node.on_localization_reset(SimpleNamespace(data="done"))
        self.assertEqual(self.node.state.startup_pose_confirmation_started_at, 101.0)
        self.tick(132.0)
        self.assertTrue(self.node.state.startup_location_failed)

    def test_position_and_yaw_tolerances(self):
        for x, yaw, accepted in (
            (3.49, 0.49, True),
            (3.51, 0.0, False),
            (3.0, 0.51, False),
        ):
            with self.subTest(x=x, yaw=yaw):
                self.setUp()
                self.start_confirmation()
                self.tick(102.0)
                self.node.on_amcl(pose(x=x, yaw=yaw, stamp=201))
                self.assertEqual(self.node.state.startup_location_resolved, accepted)

    def dock_sensors(self):
        self.node.state.charging_current = 1.0
        self.node.state.battery_voltage = 25.0
        self.node.state.infrared_detected = True

    def test_dock_start_waits_for_matching_amcl_and_stays_inactive(self):
        self.dock_sensors()
        self.tick(101.0)
        self.assertEqual(self.node.state.startup_pose_source, "dock")
        self.assertFalse(self.node.state.startup_location_resolved)
        self.node.on_robot_active(SimpleNamespace(data=True))
        self.assertFalse(self.node.state.robot_active)
        self.tick(102.0)
        self.tick(103.0)
        self.assertEqual(self.node.initial_pose_pub.publish.call_count, 2)
        self.node.on_amcl(pose(x=12.0, y=-16.0, stamp=200))
        self.assertFalse(self.node.state.startup_location_resolved)
        self.node.on_amcl(pose(x=0.0, y=0.0, stamp=201))
        self.assertFalse(self.node.state.startup_location_resolved)
        self.node.on_amcl(pose(x=12.0, y=-16.0, stamp=201))
        self.assertTrue(self.node.state.startup_location_resolved)
        self.assertTrue(self.node.state.at_station)
        self.assertFalse(self.node.state.robot_active)
        self.node.on_robot_active(SimpleNamespace(data=True))
        self.assertTrue(self.node.state.robot_active)

    def test_dock_timeout_and_retry_never_unlock(self):
        self.dock_sensors()
        self.tick(101.0)
        self.tick(102.0)
        self.tick(132.0)
        self.assertTrue(self.node.state.startup_location_failed)
        self.assertFalse(self.node.state.startup_location_resolved)
        self.node.on_amcl(pose(x=12.0, y=-16.0, stamp=201))
        self.assertFalse(self.node.state.startup_location_resolved)
        self.tick(133.0)
        self.tick(164.0)
        self.tick(165.0)
        self.assertEqual(self.node.state.startup_pose_source, "dock")
        self.assertFalse(self.node.state.startup_location_resolved)

    def test_missing_dock_pose_and_lost_dock_remain_blocked(self):
        self.dock_sensors()
        self.node.dock_pose = None
        self.tick(101.0)
        self.assertTrue(self.node.state.startup_location_failed)
        self.assertFalse(self.node.state.startup_location_resolved)
        self.setUp()
        self.dock_sensors()
        self.tick(101.0)
        self.tick(102.0)
        self.node.state.charging_current = 0.0
        self.node.state.infrared_detected = False
        self.tick(103.0)
        self.assertTrue(self.node.state.startup_location_failed)
        self.node.on_amcl(pose(x=12.0, y=-16.0, stamp=201))
        self.assertFalse(self.node.state.startup_location_resolved)

    def test_new_docking_starts_new_confirmation(self):
        self.node.state.startup_location_resolved = True
        self.node.state.robot_active = True
        with patch.object(time, "monotonic", return_value=101.0):
            self.node.on_recharge_event(SimpleNamespace(data="ROBOT-CHARGING"))
        self.assertFalse(self.node.state.startup_location_resolved)
        self.assertFalse(self.node.state.robot_active)
        self.assertEqual(self.node.state.startup_pose_source, "dock")

    def test_confirmed_dock_remains_inactive_when_late_events_arrive(self):
        self.dock_sensors()
        self.tick(101.0)
        self.tick(102.0)
        self.node.on_recharge_event(SimpleNamespace(data="DRIVE-TO-DOCK-SUCCESS"))
        self.assertFalse(self.node.state.localization_valid)
        self.node.on_amcl(pose(x=12.0, y=-16.0, stamp=201))
        self.assertTrue(self.node.state.startup_location_resolved)
        self.assertFalse(self.node.state.robot_active)
        self.assertTrue(self.node.state.localization_valid)

    def test_simulator_applies_map_initial_pose(self):
        source = SOURCE.parent / "sim_robot.py"
        tree = ast.parse(source.read_text())
        sim = next(
            item for item in tree.body
            if isinstance(item, ast.ClassDef) and item.name == "SimRobot"
        )
        method = next(
            item for item in sim.body
            if isinstance(item, ast.FunctionDef) and item.name == "on_initial_pose"
        )
        module = ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[]))
        namespace = {"quaternion_to_yaw": lambda q: 2 * math.atan2(q.z, q.w)}
        exec(compile(module, str(source), "exec"), namespace)
        fake = SimpleNamespace(
            lock=Lock(), x=0.0, y=0.0, yaw=0.0, get_logger=Mock(return_value=Mock())
        )
        namespace["on_initial_pose"](fake, pose(x=12.0, y=-16.0, yaw=0.5))
        self.assertEqual((fake.x, fake.y), (12.0, -16.0))
        self.assertAlmostEqual(fake.yaw, 0.5)
        invalid = pose(x=0.0, y=0.0)
        invalid.header.frame_id = "odom"
        namespace["on_initial_pose"](fake, invalid)
        self.assertEqual((fake.x, fake.y), (12.0, -16.0))


if __name__ == "__main__":
    unittest.main()
