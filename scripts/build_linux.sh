#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/out/build_v8"
INSTALL_DIR="${ROOT_DIR}/out/install_v8"

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" -DMAD_BUILD_TESTS=ON
cmake --build "${BUILD_DIR}" --parallel
ctest --test-dir "${BUILD_DIR}" --output-on-failure
cmake --install "${BUILD_DIR}"
python3 "${ROOT_DIR}/tools/check_architecture_sync.py"

echo "[MAD] v8 build complete"
