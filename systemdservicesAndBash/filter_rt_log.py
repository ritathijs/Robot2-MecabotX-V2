#!/usr/bin/env python3
"""Filter dagelijkse RT-logs op datum, tijd en optionele zoekwoorden."""

import argparse
import datetime
from pathlib import Path
import re
import sys


TIMESTAMP = re.compile(r"^\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\] ")
ROS_TIMESTAMP = re.compile(
    r"^(?:\[[^\]\r\n]+\]\s*)?\[(?:DEBUG|INFO|WARN|ERROR|FATAL)\]\s*"
    r"\[(\d{1,12})(?:\.(\d{1,9}))?\]"
)


def read_timestamp(line):
    match = TIMESTAMP.match(line)
    if match:
        try:
            return datetime.datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S.%f"), False
        except ValueError:
            return None, False
    match = ROS_TIMESTAMP.match(line)
    if match:
        try:
            timestamp = datetime.datetime.fromtimestamp(int(match.group(1)))
            microseconds = int((match.group(2) or "").ljust(6, "0")[:6])
            return timestamp.replace(microsecond=microseconds), True
        except (OSError, OverflowError, ValueError):
            return None, False
    return None, False


def parse_date(value):
    try:
        return datetime.date.fromisoformat(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Gebruik een datum als JJJJ-MM-DD.") from exc


def parse_time(value):
    try:
        if not re.fullmatch(r"\d{2}:\d{2}(:\d{2})?", value):
            raise ValueError
        return datetime.time.fromisoformat(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Gebruik een tijd als UU:MM of UU:MM:SS.") from exc


def parse_end_time(value):
    return parse_time(value), (
        datetime.timedelta(minutes=1) if len(value) == 5 else datetime.timedelta(seconds=1)
    )


def records(input_file):
    timestamp = None
    header = None
    is_ros = False
    before = []
    after = []
    for line in input_file:
        next_timestamp, next_is_ros = read_timestamp(line)
        if next_timestamp is None:
            (after if header is not None else before).append(line)
            continue
        if header is not None:
            yield timestamp, header, is_ros, before, after, next_timestamp
            before, after = [], []
        timestamp, header, is_ros = next_timestamp, line, next_is_ros
    if header is not None:
        yield timestamp, header, is_ros, before, after, None
    elif before:
        yield None, None, False, before, [], None


def write_record(record, output, start, end, keywords):
    timestamp, header, is_ros, before, after, next_timestamp = record
    lines = before + ([header] if header is not None else []) + after
    if keywords and not any(word in line.casefold() for word in keywords for line in lines):
        return
    selected = timestamp is not None and start <= timestamp < end
    next_selected = next_timestamp is not None and start <= next_timestamp < end
    if selected or timestamp is None:
        output.writelines(before)
    if selected:
        if is_ros:
            prefix = timestamp.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            output.write(f"[{prefix}] ")
        output.write(header)
    if selected or next_selected:
        output.writelines(after)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logbestand", type=Path, help="Basisbestand of gedateerd logbestand")
    parser.add_argument("--datum", type=parse_date, required=True, help="Datum: JJJJ-MM-DD")
    parser.add_argument("--van", type=parse_time, required=True, help="Begintijd, inclusief")
    parser.add_argument("--tot", type=parse_end_time, required=True, help="Eindtijd, inclusief")
    parser.add_argument("--zoek", action="append", default=[], metavar="WOORD",
                        help="Zoekwoord (herhaalbaar; minstens één moet in het bericht voorkomen)")
    parser.add_argument("--uitvoer", type=Path, help="Nieuw uitvoerbestand in plaats van scherm")
    parser.add_argument("--oud", action="store_true",
                        help="Lees het opgegeven oude bestand, ook als een dagbestand bestaat")
    args = parser.parse_args()
    end_time, end_resolution = args.tot
    if args.van > end_time:
        parser.error("--tot moet op dezelfde dag liggen en niet vóór --van.")

    source = args.logbestand
    dated = source.with_name(f"{source.stem}.{args.datum.isoformat()}{source.suffix}")
    if not args.oud and dated.exists():
        source = dated
    elif not source.exists():
        parser.error(f"Logbestand niet gevonden: {source if args.oud else dated}")
    if args.uitvoer and args.uitvoer.resolve() == source.resolve():
        parser.error("Het uitvoerbestand mag niet hetzelfde zijn als het invoerbestand.")

    start = datetime.datetime.combine(args.datum, args.van)
    # Een opgegeven minuut of seconde is volledig inbegrepen.
    end = datetime.datetime.combine(args.datum, end_time) + end_resolution
    keywords = [word.casefold() for word in args.zoek]
    saw_timestamp = False
    saw_untimed = False
    try:
        with source.open("r", encoding="utf-8", errors="replace") as input_file:
            output_file = args.uitvoer.open("x", encoding="utf-8") if args.uitvoer else None
            try:
                output = output_file if output_file is not None else sys.stdout
                for record in records(input_file):
                    saw_timestamp |= record[0] is not None
                    saw_untimed |= record[0] is None
                    write_record(record, output, start, end, keywords)
            finally:
                if output_file is not None:
                    output_file.close()
    except OSError as exc:
        print(f"Logfilter mislukt: {exc}", file=sys.stderr)
        return 1
    if saw_untimed and not saw_timestamp:
        print("Geen tijdstempels gevonden; de tijdselectie kon niet worden toegepast.",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
