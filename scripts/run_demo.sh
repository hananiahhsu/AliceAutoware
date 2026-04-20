#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO="${1:-highway_lane_change}"
BIN="${ROOT_DIR}/out/build/linux/mad_demo"
if [[ ! -x "${BIN}" ]]; then
  echo "[MAD] demo binary not found. Run scripts/build_linux.sh first."
  exit 2
fi
"${BIN}" "${SCENARIO}"
