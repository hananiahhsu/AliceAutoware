#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
BUILD_DIR="${ROOT_DIR}/out/build/linux"
SCENARIO="${1:-highway_lane_change}"
"${BUILD_DIR}/mad_visualizer" "${SCENARIO}"
