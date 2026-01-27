#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

STAGE="${1:-final}"
CHROOT_ROOT="/root/chroot"

if [[ ! -d "$CHROOT_ROOT" ]]; then
  echo "Missing $CHROOT_ROOT inside image. Did you copy scripts/chroot -> /root ?" >&2
  exit 2
fi

# shellcheck disable=SC1091
source "$CHROOT_ROOT/lib/tasks.sh"
# shellcheck disable=SC1091
source "$CHROOT_ROOT/lib/runner.sh"

require_root
os_info
fs_sanity

case "$STAGE" in
  middle)
    # shellcheck disable=SC1091
    source "$CHROOT_ROOT/stages/middle.sh"
    ;;
  final)
    # shellcheck disable=SC1091
    source "$CHROOT_ROOT/stages/final.sh"
    ;;
  *)
    die "Unknown stage: $STAGE"
    ;;
esac
