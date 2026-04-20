#!/usr/bin/env python3
import csv
import pathlib
import statistics
import sys


def main() -> int:
    if len(sys.argv) != 2:
        print('Usage: evaluate_run.py <input.csv>')
        return 1

    input_path = pathlib.Path(sys.argv[1])
    rows = list(csv.DictReader(input_path.open('r', encoding='utf-8')))
    if not rows:
        print('[MAD] no rows found')
        return 2

    speeds = [float(row['ego_speed']) for row in rows]
    lane_errors = [float(row.get('lane_error', '0') or 0.0) for row in rows]
    ttcs = [float(row.get('min_ttc', '1000000000') or 1e9) for row in rows if float(row.get('min_ttc', '1000000000') or 1e9) < 1e8]
    collisions = any(int(row['collided']) for row in rows)
    decisions = {row['decision'] for row in rows}
    final_row = rows[-1]

    print('[MAD] evaluation summary')
    print(f'  samples          : {len(rows)}')
    print(f'  avg ego speed    : {statistics.fmean(speeds):.3f} m/s')
    print(f'  max ego speed    : {max(speeds):.3f} m/s')
    print(f'  min TTC          : {(min(ttcs) if ttcs else float("inf")):.3f} s')
    print(f'  avg lane error   : {statistics.fmean(lane_errors):.3f} m')
    print(f'  final pose       : ({final_row["ego_x"]}, {final_row["ego_y"]})')
    print(f'  collided         : {collisions}')
    print(f'  decisions        : {sorted(decisions)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
