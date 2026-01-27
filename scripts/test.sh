#!/usr/bin/env bash
set -euo pipefail

OUT_DIR="${1:-out}"
IMG="$(ls -1t "${OUT_DIR}"/*.img.xz 2>/dev/null | head -n 1 || true)"

if [[ -z "${IMG}" ]]; then
  echo "No *.img.xz found in ${OUT_DIR}"
  exit 1
fi

echo "Latest: ${IMG}"
xz -t "${IMG}"
xz -l "${IMG}"
echo "OK"
