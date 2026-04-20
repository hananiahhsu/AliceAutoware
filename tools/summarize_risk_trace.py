#!/usr/bin/env python3
import csv
import pathlib
import sys


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: summarize_risk_trace.py <risk_trace.csv> <summary.txt>")
        return 2
    rows = list(csv.DictReader(pathlib.Path(sys.argv[1]).open("r", encoding="utf-8")))
    if not rows:
        pathlib.Path(sys.argv[2]).write_text("no rows\n", encoding="utf-8")
        return 0
    nonempty = [r for r in rows if r["top_risks"]]
    summary = [
        f"frames={len(rows)}",
        f"frames_with_risks={len(nonempty)}",
        f"last_top_risks={nonempty[-1]['top_risks'] if nonempty else 'none'}",
    ]
    pathlib.Path(sys.argv[2]).write_text("\n".join(summary) + "\n", encoding="utf-8")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
