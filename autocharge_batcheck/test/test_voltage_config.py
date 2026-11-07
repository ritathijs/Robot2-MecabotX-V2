import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE_ROOT / "autocharge_batcheck" / "autocharge_batcheck.py"


def load_module():
    modules = {
        name: ModuleType(name)
        for name in (
            "rclpy", "rclpy.node", "rclpy.qos",
            "ament_index_python", "ament_index_python.packages",
            "std_msgs", "std_msgs.msg",
        )
    }
    modules["rclpy.node"].Node = object
    modules["rclpy.qos"].DurabilityPolicy = object
    modules["rclpy.qos"].QoSProfile = object
    modules["ament_index_python.packages"].get_package_share_directory = Mock()
    modules["std_msgs.msg"].Float32 = object
    spec = importlib.util.spec_from_file_location("batcheck_under_test", SOURCE)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class VoltageConfigTest(unittest.TestCase):
    def test_installed_config_is_loaded_without_legacy_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run(
                [
                    sys.executable, "setup.py", "install_data",
                    "--install-dir", directory,
                ],
                cwd=PACKAGE_ROOT,
                check=True,
                text=True,
            )
            prefix = Path(directory)
            self.assertTrue(
                (prefix / "share/ament_index/resource_index/packages/autocharge_batcheck").is_file()
            )
            share = prefix / "share/autocharge_batcheck"
            config = share / "config/spanningsniveaus.json"
            self.assertEqual(
                json.loads(config.read_text()),
                {"battery_low_voltage": 21.0, "battery_ok_voltage": 23.0},
            )
            module = load_module()
            module.get_package_share_directory.return_value = str(share)
            node = SimpleNamespace(get_logger=Mock(return_value=Mock()))
            module.AutochargeBatcheck.load_voltage_config(node)
            module.get_package_share_directory.assert_called_once_with("autocharge_batcheck")
            self.assertEqual((node.low_voltage, node.ok_voltage), (21.0, 23.0))

    def test_custom_values_are_loaded(self):
        with tempfile.TemporaryDirectory() as directory:
            config_dir = Path(directory) / "config"
            config_dir.mkdir()
            (config_dir / "spanningsniveaus.json").write_text(
                '{"battery_low_voltage": 20.5, "battery_ok_voltage": 22.5}'
            )
            module = load_module()
            module.get_package_share_directory.return_value = directory
            node = SimpleNamespace(get_logger=Mock(return_value=Mock()))
            module.AutochargeBatcheck.load_voltage_config(node)
            self.assertEqual((node.low_voltage, node.ok_voltage), (20.5, 22.5))

    def test_missing_config_is_logged_and_raises(self):
        with tempfile.TemporaryDirectory() as directory:
            module = load_module()
            module.get_package_share_directory.return_value = directory
            logger = Mock()
            node = SimpleNamespace(get_logger=Mock(return_value=logger))
            with self.assertRaises(FileNotFoundError):
                module.AutochargeBatcheck.load_voltage_config(node)
            logger.error.assert_called_once()


if __name__ == "__main__":
    unittest.main()
