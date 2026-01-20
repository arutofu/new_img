#!/usr/bin/env bash
set -euo pipefail

ENV_FILE="${1:?env file}"
CACHE_DIR="${2:?cache dir}"

source "$ENV_FILE"
mkdir -p "$CACHE_DIR"

FNAME="$(basename "$BASE_URL")"
DEST="$CACHE_DIR/$FNAME"

if [[ -f "$DEST" ]]; then
  echo "Already downloaded: $DEST"
  exit 0
fi

echo "Downloading: $BASE_URL"
wget --progress=dot:giga -O "$DEST" "$BASE_URL"
echo "Done: $DEST"
