#!/usr/bin/env bash
set -euo pipefail

run_dir() {
  local dir="$1"
  [[ -d "$dir" ]] || die "Directory not found: $dir"

  local f
  shopt -s nullglob
  for f in "$dir"/*.sh; do
    info "[module] $(basename "$dir")/$(basename "$f")"
    # shellcheck disable=SC1090
    source "$f"
  done
  shopt -u nullglob
}
