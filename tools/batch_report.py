#!/usr/bin/env python3
import csv
import pathlib
import sys


def main() -> int:
    if len(sys.argv) != 2:
        print('Usage: batch_report.py <summary.csv>')
        return 1

    path = pathlib.Path(sys.argv[1])
    rows = list(csv.DictReader(path.open('r', encoding='utf-8')))
    if not rows:
        print('[MAD] no rows found')
        return 2

    print('[MAD] batch summary')
    print(f'  scenarios        : {len(rows)}')
    print(f'  avg speed        : {sum(float(r["avg_speed"]) for r in rows)/len(rows):.3f} m/s')
    print(f'  worst min TTC    : {min(float(r["min_ttc"]) for r in rows):.3f} s')
    print(f'  collisions       : {[r["scenario"] for r in rows if r["collided"] == "1"]}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
