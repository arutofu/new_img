#!/usr/bin/env bash
set -euo pipefail

ENV_FILE="${1:?env}"
CACHE_DIR="${2:?cache}"
BUILD_DIR="${3:?build}"
OUT_DIR="${4:?out}"

source "$ENV_FILE"
mkdir -p "$BUILD_DIR" "$OUT_DIR"

BASE_IMG="$BUILD_DIR/base.img"
MIDDLE_IMG="$BUILD_DIR/middle.img"

if [[ ! -f "$BASE_IMG" ]]; then
  echo "Missing $BASE_IMG. Run: make fetch && make unpack"
  exit 1
fi

# Rebuild middle if missing, base newer, or FORCE_MIDDLE=true
if [[ ! -f "$MIDDLE_IMG" || "$BASE_IMG" -nt "$MIDDLE_IMG" || "${FORCE_MIDDLE:-false}" == "true" ]]; then
  sudo bash "$(dirname "$0")/build_middle.sh" "$ENV_FILE" "$CACHE_DIR" "$BUILD_DIR"
else
  echo "Using cached middle: $MIDDLE_IMG"
fi

sudo bash "$(dirname "$0")/build_final.sh" "$ENV_FILE" "$CACHE_DIR" "$BUILD_DIR" "$OUT_DIR"
