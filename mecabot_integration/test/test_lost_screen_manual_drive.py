import ast
import unittest
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[1] / "scripts" / "rt_drive_to_coord.py"


def load_nodes():
    tree = ast.parse(SOURCE.read_text())
    for item in tree.body:
        if isinstance(item, ast.Assign) and item.targets[0].id == "behavior_tree_nodes":
            return ast.literal_eval(item.value)


class LostScreenManualDriveTest(unittest.TestCase):
    def test_lost_screen_does_not_send_estop(self):
        nodes = load_nodes()
        self.assertEqual(nodes["robot-error-drive"]["drive_action"], "release")
        self.assertEqual(nodes["None"]["drive_action"], "release")

    def test_unknown_node_still_estops(self):
        self.assertEqual(load_nodes()["Default"]["drive_action"], "estop")


if __name__ == "__main__":
    unittest.main()
