#!/usr/bin/env python3
import csv
import pathlib
import sys


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: summarize_task_bridge.py <task_bridge.csv> <summary.txt>")
        return 2
    rows = list(csv.DictReader(pathlib.Path(sys.argv[1]).open("r", encoding="utf-8")))
    counts = {}
    for row in rows:
        counts[row["task_directive"]] = counts.get(row["task_directive"], 0) + 1
    lines = [f"{key}={counts[key]}" for key in sorted(counts)] or ["no_directives"]
    pathlib.Path(sys.argv[2]).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
