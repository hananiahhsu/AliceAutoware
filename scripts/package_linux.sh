#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIST_DIR="${ROOT_DIR}/out/dist_v8"
PACKAGE_NAME="momenta_autodrive_stack_linux"

bash "${ROOT_DIR}/scripts/build_linux.sh"
mkdir -p "${DIST_DIR}"
rm -rf "${DIST_DIR}/${PACKAGE_NAME}"
mkdir -p "${DIST_DIR}/${PACKAGE_NAME}/bin"
cp -r "${ROOT_DIR}/out/install_v8/." "${DIST_DIR}/${PACKAGE_NAME}/"
cp -f "${ROOT_DIR}/out/build_v8/mad_demo" "${DIST_DIR}/${PACKAGE_NAME}/bin/" 2>/dev/null || true
cp -f "${ROOT_DIR}/out/build_v8/mad_batch_eval" "${DIST_DIR}/${PACKAGE_NAME}/bin/" 2>/dev/null || true
(cd "${DIST_DIR}" && tar -czf "${PACKAGE_NAME}.tar.gz" "${PACKAGE_NAME}")
echo "[MAD] package created: ${DIST_DIR}/${PACKAGE_NAME}.tar.gz"
