#!/usr/bin/env python3
import csv
import sys
from collections import Counter
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: summarize_events.py <event_csv>")
        return 2
    path = Path(sys.argv[1])
    rows = list(csv.DictReader(path.open("r", encoding="utf-8")))
    counter = Counter(row["event_type"] for row in rows)
    print(f"[MAD] event file: {path}")
    for key in sorted(counter):
        print(f"[MAD] {key}: {counter[key]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
