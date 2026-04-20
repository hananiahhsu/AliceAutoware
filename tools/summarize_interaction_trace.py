#!/usr/bin/env python3
import csv
import pathlib
import sys


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: summarize_interaction_trace.py <input_csv> <output_txt>")
        return 2
    input_path = pathlib.Path(sys.argv[1])
    output_path = pathlib.Path(sys.argv[2])
    rows = list(csv.DictReader(input_path.open(encoding="utf-8")))
    peak = max(rows, key=lambda row: float(row["severity"])) if rows else None
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as out:
        out.write("interaction_trace_summary\n")
        out.write(f"rows={len(rows)}\n")
        if peak is not None:
            out.write(f"peak_primary_actor={peak['primary_actor']}\n")
            out.write(f"peak_secondary_actor={peak['secondary_actor']}\n")
            out.write(f"peak_target_lane={peak['target_lane']}\n")
            out.write(f"peak_time_to_overlap={peak['time_to_overlap']}\n")
            out.write(f"peak_severity={peak['severity']}\n")
            out.write(f"peak_label={peak['label']}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
