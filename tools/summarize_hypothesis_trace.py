#!/usr/bin/env python3
import csv
import pathlib
import sys
from collections import Counter


def main() -> int:
    if len(sys.argv) != 3:
        print('usage: summarize_hypothesis_trace.py <hypothesis_trace.csv> <summary.txt>')
        return 2

    rows = list(csv.DictReader(pathlib.Path(sys.argv[1]).open('r', encoding='utf-8')))
    if not rows:
        pathlib.Path(sys.argv[2]).write_text('no_rows\n', encoding='utf-8')
        return 0

    label_counter = Counter(row['label'] for row in rows)
    min_conflict = min(float(row['earliest_conflict_time']) for row in rows)
    max_prob = max(float(row['probability']) for row in rows)

    lines = [
        f'rows={len(rows)}',
        f'min_earliest_conflict_time={min_conflict:.3f}',
        f'max_probability={max_prob:.3f}',
    ]
    for key in sorted(label_counter):
        lines.append(f'label_{key}={label_counter[key]}')

    pathlib.Path(sys.argv[2]).write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
