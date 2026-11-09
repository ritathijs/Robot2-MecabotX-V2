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
        clock = patch.object(time, "monotonic", return_value=103.0)
        clock.start()
        self.addCleanup(clock.stop)
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
        self.node.localization_reset_pub = Mock()
        self.node.localization_reset_pub.get_subscription_count.return_value = 1
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

    def test_dock_start_wake_waits_for_matching_amcl(self):
        self.dock_sensors()
        self.tick(101.0)
        self.assertEqual(self.node.state.startup_pose_source, "dock")
        self.assertFalse(self.node.state.startup_location_resolved)
        self.node.on_robot_active(SimpleNamespace(data=True))
        self.assertFalse(self.node.state.robot_active)
        self.assertTrue(self.node.state.dock_wake_pending)
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
        self.assertTrue(self.node.state.robot_active)

    def prepare_sleeping_dock(self):
        self.dock_sensors()
        self.node._update_dock_status_locked()
        self.node.state.startup_location_resolved = True
        self.node.state.localization_valid = True
        self.node.state.robot_active = False

    def request_wake(self, source):
        with patch.object(time, "monotonic", return_value=101.0):
            if source == "topic":
                self.node.on_robot_active(SimpleNamespace(data=True))
            else:
                self.node.handle_socket_event(
                    "active_button_toggled", (None, None)
                )

    def test_wake_requires_new_dock_pose_and_matching_fresh_amcl(self):
        for source in ("topic", "socket"):
            with self.subTest(source=source):
                self.setUp()
                self.prepare_sleeping_dock()
                self.request_wake(source)
                self.assertTrue(self.node.state.dock_wake_pending)
                self.assertFalse(self.node.state.robot_active)
                self.assertFalse(self.node.state.startup_location_resolved)
                self.node.on_amcl(pose(x=12.0, y=-16.0, stamp=201))
                self.assertFalse(self.node.state.robot_active)
                self.tick(102.0)
                published = self.node.initial_pose_pub.publish.call_args.args[0]
                self.assertEqual(published.header.frame_id, "map")
                self.assertEqual(published.pose.pose.position.x, 12.0)
                self.assertEqual(published.pose.pose.position.y, -16.0)
                for sample in (
                    pose(x=12.0, y=-16.0, stamp=200),
                    pose(x=13.0, y=-16.0, stamp=201),
                    pose(x=12.0, y=-16.0, yaw=1.0, stamp=201),
                ):
                    self.node.on_amcl(sample)
                    self.assertFalse(self.node.state.robot_active)
                self.node.on_amcl(pose(x=12.0, y=-16.0, stamp=201))
                self.assertTrue(self.node.state.robot_active)
                self.assertTrue(self.node.state.startup_location_resolved)
                self.assertFalse(self.node.state.dock_wake_pending)

    def test_pending_wake_can_be_cancelled(self):
        for cancel_source in ("topic", "socket"):
            with self.subTest(source=cancel_source):
                self.setUp()
                self.prepare_sleeping_dock()
                self.request_wake("topic")
                if cancel_source == "topic":
                    self.node.on_robot_active(SimpleNamespace(data=False))
                else:
                    self.node.handle_socket_event(
                        "active_button_toggled", (None, None)
                    )
                self.tick(102.0)
                self.node.on_amcl(pose(x=12.0, y=-16.0, stamp=201))
                self.assertTrue(self.node.state.startup_location_resolved)
                self.assertFalse(self.node.state.dock_wake_pending)
                self.assertFalse(self.node.state.robot_active)

    def test_pending_wake_is_cancelled_by_timeout_or_lost_dock(self):
        for failure in ("timeout", "lost_dock"):
            with self.subTest(failure=failure):
                self.setUp()
                self.prepare_sleeping_dock()
                self.request_wake("topic")
                self.tick(102.0)
                if failure == "timeout":
                    self.tick(132.0)
                else:
                    self.node.on_charging_current(SimpleNamespace(data=0.0))
                    self.node.on_infrared_status(SimpleNamespace(data=0))
                    self.tick(103.0)
                self.assertTrue(self.node.state.startup_location_failed)
                self.assertFalse(self.node.state.dock_wake_pending)
                self.node.on_amcl(pose(x=12.0, y=-16.0, stamp=201))
                self.assertFalse(self.node.state.robot_active)
                self.assertFalse(self.node.state.startup_location_resolved)

    def test_duplicate_wake_topic_does_not_restart_confirmation(self):
        self.prepare_sleeping_dock()
        self.request_wake("topic")
        self.tick(102.0)
        self.request_wake("topic")
        self.assertEqual(self.node.state.startup_pose_confirmation_started_at, 101.0)
        self.assertEqual(self.node.state.startup_pose_first_sent_ns, 200_000_000_000)
        self.node.on_amcl(pose(x=12.0, y=-16.0, stamp=201))
        self.assertTrue(self.node.state.robot_active)

    def test_late_amcl_cannot_wake_even_before_timeout_timer_runs(self):
        self.prepare_sleeping_dock()
        self.request_wake("topic")
        self.tick(102.0)
        with patch.object(time, "monotonic", return_value=132.0):
            self.node.on_amcl(pose(x=12.0, y=-16.0, stamp=201))
        self.assertTrue(self.node.state.startup_location_failed)
        self.assertFalse(self.node.state.dock_wake_pending)
        self.assertFalse(self.node.state.robot_active)

    def test_retry_after_wake_timeout_does_not_wake_without_new_request(self):
        self.prepare_sleeping_dock()
        self.request_wake("topic")
        self.tick(102.0)
        self.tick(132.0)
        self.tick(133.0)
        self.tick(164.0)
        self.tick(165.0)
        self.tick(166.0)
        # Na de mislukte laadpaalpose volgt AprilTag-herstel, geen nieuwe wake.
        self.assertTrue(self.node.state.dock_pose_rejected)
        self.assertTrue(self.node.state.localization_recovery_requested)
        self.node.on_amcl(pose(x=12.0, y=-16.0, stamp=201))
        self.assertFalse(self.node.state.startup_location_resolved)
        self.assertFalse(self.node.state.robot_active)
        self.assertFalse(self.node.state.dock_wake_pending)

    def test_wake_with_missing_dock_pose_stays_inactive(self):
        self.prepare_sleeping_dock()
        self.node.dock_pose = None
        self.request_wake("topic")
        self.assertTrue(self.node.state.startup_location_failed)
        self.assertFalse(self.node.state.dock_wake_pending)
        self.assertFalse(self.node.state.robot_active)

    def test_activation_away_from_dock_remains_immediate(self):
        for source in ("topic", "socket"):
            with self.subTest(source=source):
                self.setUp()
                self.node.state.startup_location_resolved = True
                self.request_wake(source)
                self.assertTrue(self.node.state.robot_active)
                self.assertFalse(self.node.state.dock_wake_pending)
                self.node.initial_pose_pub.publish.assert_not_called()

    def test_activation_with_ir_only_dock_requires_amcl_confirmation(self):
        for source in ("topic", "socket"):
            for rejected in (False, True):
                with self.subTest(source=source, rejected=rejected):
                    self.setUp()
                    self.node.state.battery_voltage = 25.0
                    self.node.state.infrared_detected = True
                    self.node.state.startup_location_resolved = True
                    self.node.state.dock_pose_rejected = rejected
                    self.node._update_dock_status_locked()
                    self.assertTrue(self.node.state.docked)
                    self.assertFalse(self.node.state.charging)
                    self.request_wake(source)
                    self.assertFalse(self.node.state.robot_active)
                    self.assertTrue(self.node.state.dock_wake_pending)
                    self.assertFalse(self.node.state.startup_location_resolved)
                    self.tick(102.0)
                    published = self.node.initial_pose_pub.publish.call_args.args[0]
                    self.assertEqual(published.pose.pose.position.x, 12.0)
                    self.assertEqual(published.pose.pose.position.y, -16.0)
                    self.node.on_amcl(pose(x=12.0, y=-16.0, stamp=201))
                    self.assertTrue(self.node.state.robot_active)
                    self.assertTrue(self.node.state.startup_location_resolved)

    def test_ir_without_dock_does_not_request_dock_pose(self):
        for source in ("topic", "socket"):
            for resolved in (False, True):
                with self.subTest(source=source, resolved=resolved):
                    self.setUp()
                    self.node.state.battery_voltage = 23.7
                    self.node.state.infrared_detected = True
                    self.node.state.startup_location_resolved = resolved
                    self.node.state.startup_location_failed = not resolved
                    self.node._update_dock_status_locked()
                    self.assertFalse(self.node.state.docked)
                    self.request_wake(source)
                    self.assertFalse(self.node.state.dock_wake_pending)
                    self.assertEqual(self.node.state.robot_active, resolved)
                    self.assertFalse(self.node.state.docked)
                    self.assertFalse(self.node.state.charging)
                    self.assertIsNone(self.node.state.startup_pose_source)
                    self.node.initial_pose_pub.publish.assert_not_called()

    def test_charging_event_preserves_pending_dock_wake(self):
        self.prepare_sleeping_dock()
        self.request_wake("topic")
        self.tick(102.0)
        self.node.on_recharge_event(SimpleNamespace(data="0ROBOT-CHARGING"))
        self.node.on_recharge_event(SimpleNamespace(data="0ROBOT-CHARGING"))
        self.assertTrue(self.node.state.dock_wake_pending)
        self.assertEqual(self.node.state.startup_pose_confirmation_started_at, 101.0)
        self.assertEqual(self.node.state.startup_pose_first_sent_ns, 200_000_000_000)
        self.node.on_amcl(pose(x=12.0, y=-16.0, stamp=201))
        self.assertTrue(self.node.state.robot_active)

    def test_pushed_onto_dock_shows_charging_screen_and_recovers(self):
        self.node.state.startup_location_resolved = True
        self.node.state.robot_active = False
        self.node.state.last_screen = "robot-docking"
        self.node.state.charging_current = 0.0
        self.node.on_charging_current(SimpleNamespace(data=1.2))
        self.assertEqual(self.node.state.last_screen, "robot-charging")
        self.node.screen_pub.publish.assert_called()

    def test_active_docking_keeps_docking_screen(self):
        self.node.state.robot_active = True
        self.node.state.last_screen = "robot-docking"
        self.node.on_charging_current(SimpleNamespace(data=1.2))
        self.assertEqual(self.node.state.last_screen, "robot-docking")

    def test_rejected_ir_only_dock_pose_is_not_retried(self):
        self.node.state.battery_voltage = 25.0
        self.node.state.infrared_detected = True
        self.node.state.charging_current = 0.0
        self.tick(101.0)
        self.assertEqual(self.node.state.startup_pose_source, "dock")
        self.tick(132.0)
        self.assertTrue(self.node.state.startup_location_failed)
        self.assertTrue(self.node.state.dock_pose_rejected)
        self.tick(133.0)
        self.tick(164.0)
        self.assertFalse(self.node.state.startup_location_failed)
        self.assertIsNone(self.node.state.startup_pose_source)
        self.tick(165.0)
        self.assertNotEqual(self.node.state.startup_pose_source, "dock")
        self.node.localization_reset_pub.publish.assert_called_once()

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
        self.assertNotEqual(self.node.state.startup_pose_source, "dock")
        self.assertTrue(self.node.state.dock_pose_rejected)
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

    def test_inactive_while_charging_restores_sleep_screen(self):
        for activation_source in ("socket", "topic"):
            for admin_open in (False, True):
                with self.subTest(source=activation_source, admin=admin_open):
                    self.setUp()
                    self.node.state.startup_location_resolved = True
                    self.node.state.robot_active = True
                    self.node.state.charging = True
                    self.node.state.last_screen = "robot-startup"
                    self.node.state.admin_open = admin_open
                    if activation_source == "socket":
                        self.node.handle_socket_event(
                            "active_button_toggled", (None, None)
                        )
                    else:
                        self.node.on_robot_active(SimpleNamespace(data=False))
                    self.assertFalse(self.node.state.robot_active)
                    self.assertEqual(self.node.state.last_screen, "robot-charging")
                    if admin_open:
                        self.node.screen_pub.publish.assert_not_called()
                    else:
                        self.assertEqual(
                            self.node.screen_pub.publish.call_args.args[0].data,
                            "robot-charging",
                        )

    def test_charging_measurement_recovers_stale_startup_screen(self):
        self.node.state.last_screen = "robot-startup"
        self.node.on_charging_current(SimpleNamespace(data=1.26))
        self.assertEqual(self.node.state.last_screen, "robot-charging")
        self.assertEqual(
            self.node.screen_pub.publish.call_args.args[0].data, "robot-charging"
        )
        self.node.on_charging_current(SimpleNamespace(data=1.26))
        self.assertEqual(self.node.screen_pub.publish.call_count, 1)

    def test_late_work_area_node_does_not_replace_sleep_screen(self):
        self.node.state.charging = True
        self.node.state.last_screen = "robot-charging"
        self.node.on_behavior_node_changed(SimpleNamespace(data="DriveWorkArea"))
        self.assertEqual(self.node.state.last_screen, "robot-charging")
        self.assertEqual(
            self.node.screen_pub.publish.call_args.args[0].data, "robot-charging"
        )

    def test_sleep_screen_is_restored_after_admin_closes(self):
        for closing_method in ("_handle_admin_closed", "_handle_robot_login_closed"):
            with self.subTest(method=closing_method):
                self.setUp()
                self.node.state.admin_open = closing_method == "_handle_admin_closed"
                self.node.state.admin_login_open = not self.node.state.admin_open
                self.node.state.last_screen = "robot-startup"
                self.node._stop_admin_pause = Mock()
                self.node.on_charging_current(SimpleNamespace(data=1.26))
                self.node.screen_pub.publish.assert_not_called()
                getattr(self.node, closing_method)()
                self.assertEqual(
                    self.node.screen_pub.publish.call_args.args[0].data,
                    "robot-charging",
                )

    def test_active_work_area_still_shows_startup(self):
        self.node.state.robot_active = True
        self.node.state.charging = True
        self.node.on_behavior_node_changed(SimpleNamespace(data="DriveWorkArea"))
        self.assertEqual(self.node.state.last_screen, "robot-startup")
        self.assertEqual(
            self.node.screen_pub.publish.call_args.args[0].data, "robot-startup"
        )

    def test_charging_recovery_preserves_other_states_and_screens(self):
        for active, screen in (
            (True, "robot-startup"),
            (False, "robot-error-charge"),
            (False, "robot-go-charge"),
        ):
            with self.subTest(active=active, screen=screen):
                self.setUp()
                self.node.state.robot_active = active
                self.node.state.last_screen = screen
                self.node.on_charging_current(SimpleNamespace(data=1.26))
                self.assertEqual(self.node.state.last_screen, screen)
                self.node.screen_pub.publish.assert_not_called()

        self.setUp()
        self.node.state.last_screen = "robot-startup"
        self.node.on_charging_current(SimpleNamespace(data=0.0))
        self.assertEqual(self.node.state.last_screen, "robot-startup")
        self.node.screen_pub.publish.assert_not_called()

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
