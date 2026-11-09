import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


ROOT = Path(__file__).resolve().parents[2]


class Duration:
    def __init__(self, seconds=0):
        self.seconds = seconds

    def __le__(self, other):
        return self.seconds <= other.seconds

    def __gt__(self, other):
        return self.seconds > other.seconds


class Instant:
    def __init__(self, seconds):
        self.seconds = seconds

    def __sub__(self, other):
        return Duration(self.seconds - other.seconds)


def twist(x=0.0):
    return SimpleNamespace(
        linear=SimpleNamespace(x=x, y=0.0),
        angular=SimpleNamespace(z=0.0),
    )


def load_mux():
    source = ROOT / "drive_mux" / "drive_mux" / "drive_mux.py"
    tree = ast.parse(source.read_text())
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef))
    module = ast.Module(body=[cls], type_ignores=[])
    namespace = {"Node": object, "Duration": Duration, "Twist": twist}
    exec(compile(ast.fix_missing_locations(module), str(source), "exec"), namespace)
    return namespace["driveMux"]


def load_drive_to_coord():
    source = ROOT / "mecabot_integration" / "scripts" / "rt_drive_to_coord.py"
    tree = ast.parse(source.read_text())
    nodes = next(
        item for item in tree.body
        if isinstance(item, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "behavior_tree_nodes"
                for target in item.targets)
    )
    cls = next(
        item for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "DriveToCoord"
    )
    module = ast.Module(body=[nodes, cls], type_ignores=[])
    namespace = {
        "Node": object,
        "Bool": lambda: SimpleNamespace(data=False),
        "Twist": twist,
    }
    exec(compile(ast.fix_missing_locations(module), str(source), "exec"), namespace)
    return namespace["DriveToCoord"]


DriveMux = load_mux()
DriveToCoord = load_drive_to_coord()


class LostManualDriveMuxTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.mux = DriveMux.__new__(DriveMux)
        self.mux.get_clock = lambda: SimpleNamespace(now=lambda: Instant(self.now))
        for source in ("chargehp", "gui", "admin_pause", "estop", "turn",
                       "charge", "bump", "search", "nav", "inhibit"):
            setattr(self.mux, "lastmessage_" + source, Instant(0))
        self.mux.autonomy_inhibited = True
        self.mux.bumper_ignoren = lambda: False
        self.mux.pub = Mock()

    def inhibit(self, value):
        self.mux.inhibit_callback(SimpleNamespace(data=value))

    def test_lost_allows_gui_but_blocks_all_autonomous_sources(self):
        self.inhibit(True)
        self.mux.gui_callback(twist(0.15))
        self.assertEqual(self.mux.active_source, "GUI")
        for callback in (
            self.mux.chargehp_callback, self.mux.turn_callback,
            self.mux.charge_callback, self.mux.search_callback,
            self.mux.nav_callback,
        ):
            callback(twist(0.3))
            self.assertEqual(self.mux.active_source, "GUI")
            self.assertEqual(self.mux.last_cmd.linear.x, 0.15)

    def test_gui_timeout_stops_without_waiting_for_other_sources(self):
        self.inhibit(True)
        self.mux.gui_callback(twist(0.15))
        self.now += 0.41
        self.mux.inhibit_watchdog()
        self.assertEqual(self.mux.active_source, "AUTONOMY_STOP")
        self.assertEqual(self.mux.last_cmd.linear.x, 0.0)

    def test_estop_and_effective_bumper_still_override_manual(self):
        for callback, source in (
            ("estop_callback", "ESTOP"), ("bump_callback", "BUMP"),
        ):
            with self.subTest(source=source):
                self.setUp()
                self.inhibit(True)
                getattr(self.mux, callback)(twist())
                self.mux.gui_callback(twist(0.15))
                self.mux.inhibit_watchdog()
                self.assertEqual(self.mux.active_source, source)
                self.assertEqual(self.mux.last_cmd.linear.x, 0.0)

    def test_missing_heartbeat_blocks_and_stops_then_recovers(self):
        self.inhibit(False)
        self.mux.chargehp_callback(twist(0.2))
        self.assertEqual(self.mux.active_source, "CHARGEHP")
        self.now += 1.51
        self.mux.inhibit_watchdog()
        self.assertEqual(self.mux.active_source, "AUTONOMY_STOP")
        self.mux.nav_callback(twist(0.2))
        self.assertEqual(self.mux.active_source, "AUTONOMY_STOP")
        self.inhibit(False)
        self.mux.nav_callback(twist(0.2))
        self.assertEqual(self.mux.active_source, "NAV")

    def test_entering_lost_stops_existing_autonomous_command(self):
        self.inhibit(False)
        self.mux.chargehp_callback(twist(0.2))
        self.inhibit(True)
        self.assertEqual(self.mux.active_source, "AUTONOMY_STOP")
        self.assertEqual(self.mux.last_cmd.linear.x, 0.0)
        self.mux.gui_callback(twist(0.15))
        self.assertEqual(self.mux.active_source, "GUI")

    def test_initially_blocks_and_stops_without_inhibit_publisher(self):
        self.mux.nav_callback(twist(0.2))
        self.mux.inhibit_watchdog()
        self.assertEqual(self.mux.active_source, "AUTONOMY_STOP")
        self.mux.gui_callback(twist(0.15))
        self.assertEqual(self.mux.active_source, "GUI")

    def test_lost_and_neutral_cancel_goals_without_estop(self):
        node = DriveToCoord.__new__(DriveToCoord)
        node.cancel_all_goals = Mock()
        node.emergencystop = Mock()
        node.autonomy_inhibit_pub = Mock()
        node.keepout_filter = Mock()
        for state in ("robot-error-drive", "None"):
            node.last_BehaviorTreeNode = state
            node.actiondistribute()
            node.publish_autonomy_inhibit()
            self.assertTrue(node.autonomy_inhibit_pub.publish.call_args.args[0].data)
        self.assertEqual(node.cancel_all_goals.call_count, 2)
        node.emergencystop.assert_not_called()
        node.last_BehaviorTreeNode = "DriveWorkArea"
        node.publish_autonomy_inhibit()
        self.assertFalse(node.autonomy_inhibit_pub.publish.call_args.args[0].data)
        node.last_BehaviorTreeNode = "onbekend"
        node.publish_autonomy_inhibit()
        self.assertTrue(node.autonomy_inhibit_pub.publish.call_args.args[0].data)


if __name__ == "__main__":
    unittest.main()
