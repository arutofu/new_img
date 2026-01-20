#!/usr/bin/env bash
set -euo pipefail

ENV_FILE="${1:?env}"
CACHE_DIR="${2:?cache}"
BUILD_DIR="${3:?build}"

source "$ENV_FILE"

mkdir -p "$BUILD_DIR"

OUT_IMG="$BUILD_DIR/base.img"

FNAME="$(basename "$BASE_URL")"
SRC="$CACHE_DIR/$FNAME"

if [[ ! -f "$SRC" ]]; then
  echo "ERROR: base archive not found: $SRC"
  echo "Run: make fetch"
  exit 1
fi

echo "Unpacking to: $OUT_IMG"
rm -f "$OUT_IMG"

case "$SRC" in
  *.img)
    cp -f "$SRC" "$OUT_IMG"
    ;;
  *.img.xz|*.xz)
    xz -dc "$SRC" > "$OUT_IMG"
    ;;
  *.zip)
    TMP="$(mktemp -d)"
    trap 'rm -rf "$TMP"' EXIT

    unzip -q "$SRC" -d "$TMP"

    INNER_IMG="$(find "$TMP" -maxdepth 3 -type f -name '*.img' | head -n 1 || true)"
    INNER_XZ="$(find "$TMP" -maxdepth 3 -type f \( -name '*.img.xz' -o -name '*.xz' \) | head -n 1 || true)"

    if [[ -n "$INNER_IMG" && -f "$INNER_IMG" ]]; then
      cp -f "$INNER_IMG" "$OUT_IMG"
    elif [[ -n "$INNER_XZ" && -f "$INNER_XZ" ]]; then
      xz -dc "$INNER_XZ" > "$OUT_IMG"
    else
      echo "ERROR: zip does not contain .img or .img.xz"
      echo "Contents:"
      find "$TMP" -maxdepth 3 -type f | sed -n '1,200p'
      exit 2
    fi
    ;;
  *)
    echo "ERROR: unsupported base format: $SRC"
    echo "Supported: .img, .xz/.img.xz, .zip"
    exit 2
    ;;
esac

echo "Done: $OUT_IMG"