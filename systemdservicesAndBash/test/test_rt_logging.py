import datetime
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPTS = Path(__file__).resolve().parents[1]
WRITER = SCRIPTS / "rt_log_writer.py"
FILTER = SCRIPTS / "filter_rt_log.py"
spec = importlib.util.spec_from_file_location("rt_log_writer", WRITER)
writer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(writer)


class LoggingTests(unittest.TestCase):
    def test_rollover_and_cleanup_only_managed_daily_logs(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            for name in (
                "camera.2026-11-06.log", "camera.2026-11-07.log",
                "nav2.2026-11-06.log", "camera.log", "andere.2026-11-06.log",
            ):
                (directory / name).write_text("oud")
            log = writer.DailyLog(directory / "camera.log")
            try:
                log.update_date(datetime.datetime(2026, 11, 14, 23, 59))
                self.assertFalse((directory / "camera.2026-11-06.log").exists())
                self.assertFalse((directory / "nav2.2026-11-06.log").exists())
                self.assertTrue((directory / "camera.2026-11-07.log").exists())
                self.assertTrue((directory / "camera.log").exists())
                self.assertTrue((directory / "andere.2026-11-06.log").exists())
                log.update_date(datetime.datetime(2026, 11, 15))
                self.assertFalse((directory / "camera.2026-11-07.log").exists())
                self.assertTrue((directory / "camera.2026-11-15.log").exists())
            finally:
                log.close()

    def test_writer_copies_terminal_bytes_and_timestamps_every_line(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp) / "nav2.log"
            result = subprocess.run(
                [sys.executable, str(WRITER), str(base)],
                input=b"eerste\nlaatste zonder newline", capture_output=True, check=True,
            )
            self.assertEqual(result.stdout, b"eerste\nlaatste zonder newline")
            log_path = base.with_name(f"nav2.{datetime.date.today().isoformat()}.log")
            lines = log_path.read_text().splitlines()
            self.assertEqual(len(lines), 2)
            self.assertRegex(lines[0], r"^\[\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d{3}\] eerste$")
            self.assertTrue(lines[1].endswith(" laatste zonder newline"))

    def test_filter_date_range_keywords_and_output(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            base = directory / "nav2.log"
            daily = directory / "nav2.2026-11-14.log"
            daily.write_text(
                "[2026-11-14 09:59:59.999] voor\n"
                "[2026-11-14 10:00:00.000] Camera\n"
                "[2026-11-14 10:30:59.999] fout\n"
                "[2026-11-14 10:31:00.000] na\n"
                "oude regel\n"
            )
            args = [sys.executable, str(FILTER), str(base), "--datum", "2026-11-14",
                    "--van", "10:00", "--tot=10:30"]
            result = subprocess.run(args, text=True, capture_output=True, check=True)
            self.assertEqual(result.stdout.count("\n"), 2)
            self.assertIn("Camera", result.stdout)
            self.assertIn("fout", result.stdout)
            self.assertEqual(result.stderr, "")
            seconds = subprocess.run(
                [sys.executable, str(FILTER), str(daily), "--datum", "2026-11-14",
                 "--van", "10:30:59", "--tot", "10:30:59"],
                text=True, capture_output=True, check=True,
            )
            self.assertIn("fout", seconds.stdout)
            self.assertNotIn("Camera", seconds.stdout)
            output = directory / "selectie.log"
            result = subprocess.run(
                args + ["--zoek", "CAMERA", "--zoek", "andere", "--uitvoer", str(output)],
                text=True, capture_output=True, check=True,
            )
            self.assertEqual(result.stdout, "")
            self.assertIn("Camera", output.read_text())
            self.assertNotIn("fout", output.read_text())
            existing = subprocess.run(
                args + ["--uitvoer", str(output)], text=True, capture_output=True,
            )
            self.assertNotEqual(existing.returncode, 0)
            self.assertIn("Camera", output.read_text())

    def test_filter_rejects_bad_range_and_source_as_output(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / "nav2.2026-11-14.log"
            source.write_text("[2026-11-14 10:00:00.000] test\n")
            args = [sys.executable, str(FILTER), str(source), "--datum", "2026-11-14"]
            bad_range = subprocess.run(args + ["--van", "11:00", "--tot", "10:00"],
                                       text=True, capture_output=True)
            self.assertNotEqual(bad_range.returncode, 0)
            same_file = subprocess.run(
                args + ["--van", "10:00", "--tot", "11:00", "--uitvoer", str(source)],
                text=True, capture_output=True,
            )
            self.assertNotEqual(same_file.returncode, 0)
            self.assertIn("test", source.read_text())

    def test_filter_legacy_ros_timestamps_uses_local_time_and_preserves_input(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / "mecabot_integration.log"
            original = (
                "[python3-3] [INFO] [1794573922.153623067] [robot_state_manager]: Goal afgerond\n"
                "[python3-5] [INFO] [1794573922.153863298] [drive_to_goal]: ander\n"
                "[python3-3] [WARN] [1794673685.740620564] [robot_state_manager]: ESTOP\n"
                "regel zonder tijdstempel\n"
            )
            source.write_text(original)
            (Path(temp) / "mecabot_integration.2026-11-13.log").write_text(
                "[2026-11-13 13:45:22.000] nieuw bestand\n"
            )
            output = Path(temp) / "selectie.log"
            result = subprocess.run(
                [sys.executable, str(FILTER), str(source), "--oud", "--datum", "2026-11-13",
                 "--van", "13:45:22", "--tot", "13:45:22", "--zoek", "goal afgerond",
                 "--uitvoer", str(output)],
                text=True, capture_output=True, check=True,
                env={**os.environ, "TZ": "Europe/Brussels"},
            )
            self.assertEqual(source.read_text(), original)
            self.assertEqual(
                output.read_text(),
                "[2026-11-13 13:45:22.153] "
                "[python3-3] [INFO] [1794573922.153623067] [robot_state_manager]: Goal afgerond\n",
            )
            self.assertEqual(result.stderr, "")

    def test_untimed_lines_use_either_adjacent_timestamp_and_keyword_matches_group(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / "nav2.log"
            source.write_text(
                "begin zonder tijd\n"
                "[2026-11-14 09:59:59.000] voor de periode\n"
                "[container] bericht voor eerste geselecteerde regel\n"
                "[2026-11-14 10:00:00.000] start\n"
                "[container] eerste vervolgregel\n"
                "[container] zoekwoord in vervolg\n"
                "[2026-11-14 10:30:59.000] laatste geselecteerde regel\n"
                "[container] vervolg na laatste geselecteerde regel\n"
                "[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node\n"
                "[2026-11-14 10:31:00.000] na de periode\n"
                "[container] buiten de periode\n"
            )
            args = [sys.executable, str(FILTER), str(source), "--oud",
                    "--datum", "2026-11-14", "--van", "10:00", "--tot", "10:30"]
            result = subprocess.run(args, text=True, capture_output=True, check=True)
            self.assertNotIn("begin zonder tijd", result.stdout)
            self.assertNotIn("voor de periode", result.stdout)
            self.assertIn("bericht voor eerste geselecteerde regel", result.stdout)
            self.assertIn("vervolg na laatste geselecteerde regel", result.stdout)
            self.assertIn("[INFO] [launch_ros.actions.load_composable_nodes]", result.stdout)
            self.assertNotIn("na de periode", result.stdout)
            self.assertNotIn("buiten de periode", result.stdout)
            self.assertEqual(result.stderr, "")
            searched = subprocess.run(args + ["--zoek", "zoekwoord"],
                                      text=True, capture_output=True, check=True)
            self.assertIn("] start", searched.stdout)
            self.assertIn("eerste vervolgregel", searched.stdout)
            self.assertIn("zoekwoord in vervolg", searched.stdout)
            self.assertNotIn("laatste geselecteerde regel", searched.stdout)

    def test_leading_and_trailing_untimed_lines_are_kept(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / "oud.log"
            source.write_text(
                "begin zonder tijd\n"
                "[2026-11-14 10:00:00.000] bericht\n"
                "einde zonder tijd\n"
            )
            result = subprocess.run(
                [sys.executable, str(FILTER), str(source), "--oud",
                 "--datum", "2026-11-14", "--van", "10:00", "--tot", "10:00"],
                text=True, capture_output=True, check=True,
            )
            self.assertEqual(result.stdout, source.read_text())

    def test_untimed_only_file_is_kept_with_warning(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / "oud.log"
            source.write_text("eerste regel\ntweede regel\n")
            result = subprocess.run(
                [sys.executable, str(FILTER), str(source), "--oud",
                 "--datum", "2026-11-14", "--van", "10:00", "--tot", "11:00"],
                text=True, capture_output=True, check=True,
            )
            self.assertEqual(result.stdout, source.read_text())
            self.assertIn("tijdselectie kon niet worden toegepast", result.stderr)


if __name__ == "__main__":
    unittest.main()
