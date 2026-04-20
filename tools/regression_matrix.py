#!/usr/bin/env python3
import csv
import pathlib
import sys


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: regression_matrix.py <scenario_summary.csv> <output_md>")
        return 2
    with open(sys.argv[1], encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    out_path = pathlib.Path(sys.argv[2])
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as out:
        out.write("# Regression Matrix\n\n")
        out.write("| Scenario | Collision | Min TTC | Gap Reject | RSS Reject | Max Joint | Mission |\n")
        out.write("|---|---|---:|---:|---:|---:|---|\n")
        for row in rows:
            out.write(
                f"| {row['scenario']} | {row['collided']} | {row['min_ttc']} | {row['gap_rejections']} | {row['rss_lane_change_rejections']} | {row['max_interaction_conflicts']} | {row['mission_result']} |\n"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
