import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


SOURCE = Path(__file__).resolve().parents[1] / "scripts" / "rt_auto_recharger.py"


def twist():
    return SimpleNamespace(linear=SimpleNamespace(x=0.0))


def load_stop_charge(clock):
    tree = ast.parse(SOURCE.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "AutoRecharger")
    method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "Stop_Charge")
    module = ast.Module(body=[method], type_ignores=[])
    namespace = {
        "Twist": twist,
        "time": clock,
        "print_and_fixRetract": lambda message: None,
        "YELLOW": "",
        "CYAN": "",
        "RESET": "",
    }
    exec(compile(ast.fix_missing_locations(module), str(SOURCE), "exec"), namespace)
    return namespace["Stop_Charge"]


class UndockDriveTest(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.commands = []
        clock = SimpleNamespace(monotonic=lambda: self.now, sleep=self.advance)
        self.stop_charge = load_stop_charge(clock)
        self.node = SimpleNamespace(
            undock_drive_duration_sec=4.5,
            Gui_Cmd_vel_pub=SimpleNamespace(publish=self.publish),
            Pub_Recharger_Flag=Mock(),
            start_turn=0,
            find_redsignal=0,
            robot={"RED": 0},
        )

    def advance(self, duration):
        self.now += duration

    def publish(self, message):
        self.commands.append((self.now, message.linear.x))

    def test_publishes_forward_throughout_duration_then_stops(self):
        self.stop_charge(self.node)
        self.assertEqual(self.commands[0], (0.0, 0.2))
        self.assertGreater(self.commands[-2][0], 4.3)
        self.assertTrue(all(speed == 0.2 for _, speed in self.commands[:-1]))
        self.assertEqual(self.commands[-1], (4.5, 0.0))
        self.assertTrue(self.node.must_reset)

    def test_can_stop_charging_without_driving(self):
        self.stop_charge(self.node, drive_forward=False)
        self.assertEqual(self.commands, [])
        self.assertTrue(self.node.must_reset)


if __name__ == "__main__":
    unittest.main()