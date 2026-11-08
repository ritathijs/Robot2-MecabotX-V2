import ast
import math
from pathlib import Path
from threading import Lock
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def twist():
    return SimpleNamespace(
        linear=SimpleNamespace(x=0.0, y=0.0),
        angular=SimpleNamespace(z=0.0),
    )


def load_class(filename, name, namespace):
    source = SCRIPTS / filename
    tree = ast.parse(source.read_text())
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == name)
    module = ast.Module(
        body=[
            ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0),
            cls,
        ],
        type_ignores=[],
    )
    exec(compile(ast.fix_missing_locations(module), str(source), "exec"), namespace)
    return namespace[name]


RobotStateManager = load_class(
    "robot_state_manager.py",
    "RobotStateManager",
    {"Node": object, "time": time, "math": math, "Twist": twist,
     "String": lambda: SimpleNamespace(data="")},
)
QuizSocketBridge = load_class(
    "quiz_socket_bridge.py",
    "QuizSocketBridge",
    {"Node": object, "time": time, "Twist": twist},
)


class ManualDriveRelocalizationTest(unittest.TestCase):
    def setUp(self):
        self.manager = RobotStateManager.__new__(RobotStateManager)
        self.manager.state = SimpleNamespace(
            admin_open=True, estop_active=False, docked=False,
            startup_location_resolved=True,
        )
        self.manager.lock = Lock()
        self.manager.manual_drive_lock = Lock()
        self.manager.last_drive_time = None
        self.manager.manual_drive_reset_due = None
        self.manager.localization_reset_pub = Mock()
        self.manager.localization_reset_pub.get_subscription_count.return_value = 1
        self.manager._publish_zero_twist = Mock()
        self.manager.send_robot_command = Mock()
        self.manager.get_logger = Mock(return_value=Mock())
        self.bridge = QuizSocketBridge.__new__(QuizSocketBridge)
        self.bridge.admin_panel_open = True
        self.bridge.is_admin_login_open = False
        self.bridge.gui_cmd_vel_pub = Mock()
        self.bridge.state_manager_callback = self.manager.handle_socket_event
        self.bridge.get_logger = Mock(return_value=Mock())

    def test_arrow_stop_requests_one_reset_after_settling(self):
        with patch.object(time, "monotonic", return_value=10.0):
            self.bridge._process_drive_event({"direction": "forward", "speed": 1})
        self.assertEqual(self.bridge.gui_cmd_vel_pub.publish.call_args.args[0].linear.x, 0.15)
        with patch.object(time, "monotonic", return_value=10.5):
            self.manager._manual_drive_timeout_check()
        self.manager.localization_reset_pub.publish.assert_not_called()
        with patch.object(time, "monotonic", return_value=11.6):
            self.manager._manual_drive_timeout_check()
            self.manager._manual_drive_timeout_check()
        self.manager.localization_reset_pub.publish.assert_called_once()
        self.assertEqual(self.manager.localization_reset_pub.publish.call_args.args[0].data, "active")

    def test_stop_or_rejected_drive_does_not_schedule_reset(self):
        with patch.object(time, "monotonic", return_value=10.0):
            self.bridge._process_drive_event({"direction": "stop", "speed": 0})
            self.bridge.admin_panel_open = False
            self.bridge._process_drive_event({"direction": "forward", "speed": 1})
        self.assertIsNone(self.manager.last_drive_time)
        self.assertIsNone(self.manager.manual_drive_reset_due)

    def test_new_motion_delays_pending_reset(self):
        with patch.object(time, "monotonic", return_value=10.0):
            self.bridge._process_drive_event({"direction": "forward", "speed": 1})
        with patch.object(time, "monotonic", return_value=10.5):
            self.manager._manual_drive_timeout_check()
        with patch.object(time, "monotonic", return_value=11.4):
            self.bridge._process_drive_event({"direction": "left", "speed": 1})
        with patch.object(time, "monotonic", return_value=11.6):
            self.manager._manual_drive_timeout_check()
        self.manager.localization_reset_pub.publish.assert_not_called()
        with patch.object(time, "monotonic", return_value=12.0):
            self.manager._manual_drive_timeout_check()
        with patch.object(time, "monotonic", return_value=13.1):
            self.manager._manual_drive_timeout_check()
        self.manager.localization_reset_pub.publish.assert_called_once()

    def test_reset_is_not_requested_during_estop_or_when_docked(self):
        for field in ("estop_active", "docked"):
            with self.subTest(field=field):
                self.setUp()
                setattr(self.manager.state, field, True)
                with patch.object(time, "monotonic", return_value=10.0):
                    self.bridge._process_drive_event({"direction": "forward", "speed": 1})
                with patch.object(time, "monotonic", return_value=10.5):
                    self.manager._manual_drive_timeout_check()
                with patch.object(time, "monotonic", return_value=11.6):
                    self.manager._manual_drive_timeout_check()
                self.manager.localization_reset_pub.publish.assert_not_called()

    def test_failed_publish_does_not_schedule_reset(self):
        self.bridge.gui_cmd_vel_pub.publish.side_effect = RuntimeError("publish failed")
        self.bridge._process_drive_event({"direction": "forward", "speed": 1})
        self.assertIsNone(self.manager.last_drive_time)
        self.bridge.get_logger().error.assert_called_once()


if __name__ == "__main__":
    unittest.main()
