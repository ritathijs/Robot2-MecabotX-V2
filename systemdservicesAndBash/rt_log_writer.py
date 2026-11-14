#!/usr/bin/env python3
"""Kopieer terminaluitvoer naar dagelijkse RT-logbestanden."""

import datetime
import os
from pathlib import Path
import re
import select
import sys


KEEP_DAYS = 8
LOG_NAMES = (
    "static_tf", "nav2", "mux", "bumper", "camera", "pointcloud", "keepout", "batcheck",
    "auto_recharger", "mecabot_integration",
)
LOG_PATTERN = re.compile(
    rf"^(?:{'|'.join(LOG_NAMES)})\.(\d{{4}}-\d{{2}}-\d{{2}})\.log$"
)


def remove_old_logs(directory, today):
    cutoff = today - datetime.timedelta(days=KEEP_DAYS - 1)
    for path in directory.iterdir():
        match = LOG_PATTERN.fullmatch(path.name)
        if not match:
            continue
        try:
            file_date = datetime.date.fromisoformat(match.group(1))
        except ValueError:
            continue
        if file_date < cutoff and path.is_file() and not path.is_symlink():
            try:
                path.unlink()
            except FileNotFoundError:
                # Another RT logger may have removed the same file.
                pass


class DailyLog:
    def __init__(self, base_path):
        self.base_path = Path(base_path)
        self.file = None
        self.date = None

    def update_date(self, now):
        today = now.date()
        if today == self.date:
            return
        if self.file is not None:
            self.file.close()
        self.file = None
        remove_old_logs(self.base_path.parent, today)
        path = self.base_path.with_name(
            f"{self.base_path.stem}.{today.isoformat()}{self.base_path.suffix}"
        )
        self.file = path.open("ab", buffering=0)
        self.date = today

    def write(self, line):
        now = datetime.datetime.now()
        self.update_date(now)
        timestamp = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.file.write(f"[{timestamp}] ".encode("ascii") + line + b"\n")

    def close(self):
        if self.file is not None:
            self.file.close()


def main():
    if len(sys.argv) == 3 and sys.argv[1] == "--cleanup":
        try:
            remove_old_logs(Path(sys.argv[2]), datetime.date.today())
        except (OSError, ValueError) as exc:
            print(f"RT-logs opruimen mislukt: {exc}", file=sys.stderr)
            return 1
        return 0
    if len(sys.argv) != 2:
        print("Gebruik: rt_log_writer.py LOGBESTAND", file=sys.stderr)
        return 2
    log = DailyLog(sys.argv[1])
    pending = b""
    try:
        log.update_date(datetime.datetime.now())
        while True:
            ready, _, _ = select.select([sys.stdin.buffer], [], [], 60)
            if not ready:
                log.update_date(datetime.datetime.now())
                continue
            data = os.read(sys.stdin.fileno(), 65536)
            if not data:
                if pending:
                    log.write(pending)
                break
            sys.stdout.buffer.write(data)
            sys.stdout.buffer.flush()
            lines = (pending + data).split(b"\n")
            pending = lines.pop()
            for line in lines:
                log.write(line)
    except (OSError, ValueError) as exc:
        print(f"RT-logging mislukt: {exc}", file=sys.stderr)
        return 1
    finally:
        log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
