#!/usr/bin/env python3
import csv, pathlib, statistics, sys

def load(path: pathlib.Path):
    with path.open('r', encoding='utf-8') as fp:
        return list(csv.DictReader(fp))

def main():
    if len(sys.argv) != 3:
        print('Usage: analyze_bridge.py <state_csv> <command_csv>')
        return 1
    state_rows = load(pathlib.Path(sys.argv[1]))
    command_rows = load(pathlib.Path(sys.argv[2]))
    if not state_rows or not command_rows:
        print('[MAD] empty bridge files')
        return 2
    speeds = [float(r['ego_speed']) for r in state_rows]
    accels = [abs(float(r['acceleration'])) for r in command_rows]
    steers = [abs(float(r['steering_angle'])) for r in command_rows]
    print('[MAD] bridge summary')
    print(f'  frames           : {len(state_rows)}')
    print(f'  avg ego speed    : {statistics.fmean(speeds):.3f} m/s')
    print(f'  max |accel cmd|  : {max(accels):.3f} m/s^2')
    print(f'  max |steer cmd|  : {max(steers):.3f} rad')
    return 0
if __name__ == '__main__':
    raise SystemExit(main())
