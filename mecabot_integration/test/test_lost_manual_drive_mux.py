import ast
from pathlib import Path
import threading
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

    def test_manual_drive_remains_available_when_heartbeat_expires(self):
        self.inhibit(False)
        self.now += 1.51
        self.mux.gui_callback(twist(0.15))
        self.mux.inhibit_watchdog()
        self.assertEqual(self.mux.active_source, "GUI")
        self.assertEqual(self.mux.last_cmd.linear.x, 0.15)
        self.mux.nav_callback(twist(0.3))
        self.assertEqual(self.mux.active_source, "GUI")

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


class BehaviorCommandTest(unittest.TestCase):
    def setUp(self):
        self.node = DriveToCoord.__new__(DriveToCoord)
        self.node.lock = threading.Lock()
        self.node.get_logger = Mock(return_value=Mock())
        self.node.publish_status = Mock()
        self.node.autonomy_inhibit_pub = Mock()
        self.node.send_goal = Mock()
        self.node.cancel_all_goals = Mock()
        self.node.robot_docked = False
        self.node.robot_active = True
        self.node.docking_active = False
        self.node.last_finished_pose = None
        self.node.last_btDriveCoord = None
        self.node.last_BehaviorTreeNode = None
        self.node.needs_action = False

    def command(self, state, pose=None, sequence=1):
        self.node.behavior_command_callback(SimpleNamespace(
            behavior_node=state, sequence_id=sequence,
            has_pose=pose is not None, pose=pose,
        ))

    def test_legacy_status_cannot_combine_with_previous_pose(self):
        old_pose, new_pose = object(), object()
        self.command("DriveWorkArea", old_pose)
        self.node.BehaviorTreeNode_callback(SimpleNamespace(data="explore-location-c"))
        self.node.actiondistribute()
        self.node.send_goal.assert_called_once_with("BehaviorTree", old_pose)
        self.command("explore-location-c", new_pose, sequence=2)
        self.node.actiondistribute()
        self.node.send_goal.assert_called_with("BehaviorTree", new_pose)

    def test_same_status_with_new_pose_is_applied_together(self):
        for pose in (object(), object()):
            self.command("follow-robot-screen", pose)
            self.node.actiondistribute()
            self.node.send_goal.assert_called_with("BehaviorTree", pose)

    def test_all_controller_navigation_states_use_their_own_pose(self):
        for state in ("follow-robot-screen", "explore-location-c",
                      "explore-location-d", "explore-location-e",
                      "DriveWorkArea", "robot-go-charge"):
            with self.subTest(state=state):
                pose = object()
                self.command(state, pose)
                self.node.actiondistribute()
                self.node.send_goal.assert_called_with("BehaviorTree", pose)

    def test_status_only_states_cancel_and_cannot_reuse_old_pose(self):
        for state in ("robot-docking", "robot-arrived-at-quiz-location",
                      "robot-error-drive", "None"):
            with self.subTest(state=state):
                self.command("DriveWorkArea", object())
                self.command(state)
                self.node.send_goal.reset_mock()
                self.node.cancel_all_goals.reset_mock()
                self.node.actiondistribute()
                self.node.cancel_all_goals.assert_called_once()
                self.node.send_goal.assert_not_called()

    def test_invalid_pose_presence_does_not_change_state(self):
        self.command("DriveWorkArea", object())
        previous_pose = self.node.last_btDriveCoord
        self.command("explore-location-c")
        self.command("robot-docking", object())
        self.assertEqual(self.node.last_BehaviorTreeNode, "DriveWorkArea")
        self.assertIs(self.node.last_btDriveCoord, previous_pose)
        self.assertEqual(self.node.get_logger().error.call_count, 2)

    def test_visitor_and_unknown_legacy_states_are_still_handled(self):
        for state in ("RobotRotationFollowMe", "robot-arrived-at-visitors",
                      "Default", "unexpected-state"):
            with self.subTest(state=state):
                self.node.BehaviorTreeNode_callback(SimpleNamespace(data=state))
                self.assertEqual(self.node.last_BehaviorTreeNode, state)


if __name__ == "__main__":
    unittest.main()
