import ast
from pathlib import Path
from types import SimpleNamespace
import threading
import time
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "mecabot_integration"


class PatrolWithoutVisitorsTest(unittest.TestCase):
    def test_mission_priorities_and_patrol_are_preserved(self):
        root = ET.parse(PACKAGE / "config" / "behavior_tree.xml").getroot()
        mission = root.find(".//ReactiveFallback[@name='MissionControl']")
        self.assertIsNotNone(mission)
        self.assertEqual(
            [child.get("name") for child in mission],
            ["QuizPreparationAndExecution", "WervingsPatrouille"],
        )
        for node in (
            "SendDriveLocationC", "SendDriveLocationD", "SendDriveLocationE",
            "CompletePatrolCycle", "StartInfraroodDocking",
        ):
            self.assertIsNotNone(root.find(f".//BehaviorTree//{node}"))
        for node in (
            "IsPersonDetected", "StartDrivingToPeople", "CheckingNearbyVisitors",
            "RobotRotationFollowMe", "VisitorNoResponseRecovery",
        ):
            self.assertIsNone(root.find(f".//BehaviorTree//{node}"))

    def test_rt_startup_does_not_require_or_start_tracker(self):
        startup = (ROOT / "systemdservicesAndBash" / "startup_mecabot_rt.sh").read_text()
        self.assertNotIn("people_follower_ros2", startup)
        self.assertNotIn("visualtracker", startup)
        self.assertIn("wheeltec_camera.launch.py", startup)
        self.assertIn("pointcloud_to_laserscan_node", startup)

    def test_tracking_is_disabled_for_active_and_inactive_states(self):
        source = (PACKAGE / "scripts" / "robot_state_manager.py").read_text()
        tree = ast.parse(source)
        manager = next(
            node for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == "RobotStateManager"
        )
        method = next(
            node for node in manager.body
            if isinstance(node, ast.FunctionDef) and node.name == "_publish_status_snapshot"
        )
        module = ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[]))
        namespace = {"time": time}
        exec(compile(module, str(PACKAGE / "scripts" / "robot_state_manager.py"), "exec"), namespace)

        state = SimpleNamespace(
            localization_valid=True, at_quiz=False, quiz_requested=False,
            at_work_area=True, at_station=False, charging=False, docked=False,
            docking_attempts=0, robot_active=True, battery_ok=True,
            at_location_c=False, at_location_d=False, at_location_e=False,
            battery_status_received=True, startup_location_resolved=True,
        )
        node = SimpleNamespace(
            state=state, lock=threading.Lock(), MAX_DOCKING_ATTEMPTS=3,
            _last_snapshot=None, _last_snapshot_time=0, SNAPSHOT_REPEAT_SEC=0,
            _bool_message=lambda value: value,
        )
        for name in (
            "localization_pub", "quiz_pub", "want_to_start_quiz_pub", "work_pub",
            "location_c_pub", "location_d_pub", "location_e_pub",
            "charging_station_pub", "charging_station_legacy_pub", "charging_pub",
            "docked_pub", "docking_retry_available_pub", "robot_state_ready_pub",
            "robot_active_pub", "battery_ok_pub", "battery_low_pub",
            "tracking_enable_pub",
        ):
            setattr(node, name, Mock())

        publish = namespace["_publish_status_snapshot"]
        publish(node)
        state.robot_active = False
        publish(node)
        self.assertEqual(
            [call.args[0] for call in node.tracking_enable_pub.publish.call_args_list],
            [False, False],
        )


if __name__ == "__main__":
    unittest.main()
