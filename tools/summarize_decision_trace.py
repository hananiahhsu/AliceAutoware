#!/usr/bin/env python3
import csv
import sys
from collections import Counter
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: summarize_decision_trace.py <decision_trace_csv>")
        return 2
    path = Path(sys.argv[1])
    rows = list(csv.DictReader(path.open("r", encoding="utf-8")))
    directives = Counter(row["bt_directive"] for row in rows)
    max_risk = max((float(row["current_lane_risk"]) for row in rows), default=0.0)
    min_speed_cap = min((float(row["global_speed_cap"]) for row in rows), default=0.0)
    print(f"[MAD] decision trace file: {path}")
    for key in sorted(directives):
        print(f"[MAD] {key}: {directives[key]}")
    print(f"[MAD] max current_lane_risk: {max_risk:.3f}")
    print(f"[MAD] min speed cap: {min_speed_cap:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
