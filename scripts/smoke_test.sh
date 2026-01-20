#!/usr/bin/env bash
set -euo pipefail

OUT_DIR="${1:?out dir}"
IMG="$(ls -t "$OUT_DIR"/*.img 2>/dev/null | head -n1 || true)"
[[ -n "$IMG" ]] || { echo "No .img in $OUT_DIR"; exit 1; }

echo "Testing: $IMG"

MNT="$(mktemp -d --suffix=.smoke)"
BOOT="$MNT/boot"
ROOT="$MNT/root"
mkdir -p "$BOOT" "$ROOT"

cleanup() {
  set +e
  umount "$BOOT" >/dev/null 2>&1 || true
  umount "$ROOT" >/dev/null 2>&1 || true
  if [[ -n "${LOOP:-}" ]]; then
    kpartx -d "$LOOP" >/dev/null 2>&1 || true
    losetup -d "$LOOP" >/dev/null 2>&1 || true
  fi
  rm -rf "$MNT"
}
trap cleanup EXIT

LOOP="$(losetup -Pf --show "$IMG")"
sleep 0.5

mount "${LOOP}p2" "$ROOT"
mount "${LOOP}p1" "$BOOT" || true

test -d "$ROOT/etc" && echo "OK: /etc"
test -d "$ROOT/home" && echo "OK: /home"
test -f "$BOOT/config.txt" && echo "OK: /boot/config.txt" || echo "WARN: /boot not mounted/absent"
if [[ -f "$BOOT/ssh" ]]; then echo "OK: ssh marker exists"; fi

echo "Smoke test done."
