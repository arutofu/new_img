#!/usr/bin/env bash
set -euo pipefail

IMG="${1:?img}"
ACTION="${2:?exec|copy}"
A="${3:-}"
B="${4:-}"

[[ -f "$IMG" ]] || { echo "No image: $IMG"; exit 1; }

MNT="$(mktemp -d --suffix=.imgmnt)"
cleanup() {
  set +e
  umount -R "$MNT" >/dev/null 2>&1 || true
  if [[ -n "${LOOP:-}" ]]; then
    losetup -d "$LOOP" >/dev/null 2>&1 || true
  fi
  rm -rf "$MNT"
}
trap cleanup EXIT

LOOP="$(losetup -Pf --show "$IMG")"
sleep 0.5

mount "${LOOP}p2" "$MNT"
mkdir -p "$MNT/boot"
mount "${LOOP}p1" "$MNT/boot" || true

# qemu для armhf
if [[ "$(uname -m)" != "armv7l" && "$(uname -m)" != "aarch64" ]]; then
  install -m 0755 /usr/bin/qemu-arm-static "$MNT/usr/bin/qemu-arm-static"
fi

mount -t proc proc "$MNT/proc"
mount -t sysfs sysfs "$MNT/sys"
mount --bind /dev "$MNT/dev"
mount --bind /dev/pts "$MNT/dev/pts"
cp -L /etc/resolv.conf "$MNT/etc/resolv.conf"

case "$ACTION" in
  copy)
    SRC="${A:?src}"
    DST="${B:?dst in image, absolute path}"
    mkdir -p "$(dirname "$MNT$DST")"
    cp -a "$SRC" "$MNT$DST"
    ;;
  exec)
    SCRIPT="${A:?script}"
    shift 3 || true
    NAME="$(basename "$SCRIPT").$RANDOM"
    cp -a "$SCRIPT" "$MNT/root/$NAME"
    chmod +x "$MNT/root/$NAME"

    # Явный запуск ARM bash через qemu (без binfmt_misc)
    chroot "$MNT" /usr/bin/qemu-arm-static /bin/bash -lc "/root/$NAME ${*:-}"

    rm -f "$MNT/root/$NAME"
    ;;
  *)
    echo "Usage: $0 <img> copy <src> <dst> | exec <script> [args...]"
    exit 2
    ;;
esac
