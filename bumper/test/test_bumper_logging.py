import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import MagicMock, patch


class BumperLoggingTest(unittest.TestCase):
    def setUp(self):
        gpio = types.ModuleType("Jetson.GPIO")
        gpio.BOARD = "BOARD"
        gpio.IN = "IN"
        gpio.setmode = MagicMock()
        gpio.setup = MagicMock()
        self.states = {7: 0, 15: 0, 31: 0, 32: 0}
        gpio.input = lambda pin: self.states[pin]

        class FakeNode:
            def __init__(self, name):
                self.create_publisher = MagicMock(side_effect=lambda *args: MagicMock())
                self.create_timer = MagicMock(side_effect=lambda *args: MagicMock())
                self.get_logger = MagicMock(return_value=MagicMock())

        node_module = types.ModuleType("rclpy.node")
        node_module.Node = FakeNode
        modules = {
            "rclpy": types.ModuleType("rclpy"),
            "rclpy.node": node_module,
            "Jetson": types.ModuleType("Jetson"),
            "Jetson.GPIO": gpio,
            "geometry_msgs": types.ModuleType("geometry_msgs"),
            "geometry_msgs.msg": types.ModuleType("geometry_msgs.msg"),
            "std_msgs": types.ModuleType("std_msgs"),
            "std_msgs.msg": types.ModuleType("std_msgs.msg"),
        }
        modules["geometry_msgs.msg"].Twist = MagicMock
        modules["std_msgs.msg"].String = MagicMock
        spec = importlib.util.spec_from_file_location(
            "bumper_under_test", Path(__file__).parents[1] / "bumper" / "bumper.py"
        )
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(self.module)

        self.node = self.module.GPIOReaderNode()
        self.logger = self.node.get_logger.return_value
        self.logger.reset_mock()
        self.node.bumper_cmd_vel_pub.reset_mock()

    def test_unchanged_status_is_logged_once(self):
        self.node.detect()
        self.node.detect()

        self.assertEqual(self.logger.info.call_count, 1)
        self.assertIn("status: Free", self.logger.info.call_args.args[0])
        self.node.bumper_cmd_vel_pub.publish.assert_not_called()

    def test_pin_and_state_changes_are_logged_without_changing_stop_behavior(self):
        self.node.detect()
        self.states[7] = 1
        self.node.detect()
        self.assertEqual(self.node.action, "Touched")
        self.assertEqual(self.node.bumper_cmd_vel_pub.publish.call_count, 1)
        self.assertIn("status: Touched", self.logger.info.call_args.args[0])
        self.assertIn("7: 1", self.logger.info.call_args.args[0])

        self.node.detect()
        self.assertEqual(self.logger.info.call_count, 2)
        self.assertEqual(self.node.bumper_cmd_vel_pub.publish.call_count, 2)

        self.states[15] = 1
        self.node.detect()
        self.assertEqual(self.logger.info.call_count, 3)
        self.assertIn("15: 1", self.logger.info.call_args.args[0])

        self.node.setAvoid()
        self.node.detect()
        self.assertEqual(self.logger.info.call_count, 4)
        self.assertIn("status: Avoid", self.logger.info.call_args.args[0])
        self.node.setStop()
        self.node.detect()
        self.assertEqual(self.logger.info.call_count, 5)
        self.assertIn("status: Stop", self.logger.info.call_args.args[0])
        self.states[7] = 0
        self.states[15] = 0
        self.node.detect()
        self.assertEqual(self.node.action, "Free")
        self.assertEqual(self.logger.info.call_count, 6)
        self.assertIn("status: Free", self.logger.info.call_args.args[0])


if __name__ == "__main__":
    unittest.main()
