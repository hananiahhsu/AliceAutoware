#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"${ROOT_DIR}/out/build/linux/mad_batch_eval"
python3 "${ROOT_DIR}/tools/batch_report.py" "${ROOT_DIR}/out/batch/scenario_summary.csv"
