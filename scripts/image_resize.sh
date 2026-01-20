#!/usr/bin/env bash
set -euo pipefail

IMG="${1:?image path}"
MODE="${2:?max|min}"
ARG="${3:-}"

[[ -f "$IMG" ]] || { echo "No image: $IMG"; exit 1; }

resize_max() {
  local size="${1:?size like 7G}"
  echo "[resize] truncate to $size"
  truncate -s"$size" "$IMG"

  local loop
  loop="$(losetup -Pf --show "$IMG")"
  sleep 0.5

  echo "[resize] expand partition #2"
  echo ", +" | sfdisk -N 2 "$loop" >/dev/null
  losetup -d "$loop"
  sleep 0.5

  loop="$(losetup -Pf --show "$IMG")"
  sleep 0.5

  echo "[resize] e2fsck + resize2fs"
  e2fsck -fvy "${loop}p2"
  resize2fs "${loop}p2"

  losetup -d "$loop"
}

shrink_min() {
  local free_mib="${1:-200}"

  echo "[shrink] leave free MiB: $free_mib"
  local partinfo partstart partnumber
  partinfo="$(parted -m "$IMG" unit B print)"
  partnumber="$(echo "$partinfo" | grep ext4 | awk -F: '{print $1}')"
  partstart="$(echo "$partinfo" | grep ext4 | awk -F: '{print substr($2,0,length($2)-1)}')"

  local loop
  loop="$(losetup -f --show -o "$partstart" "$IMG")"

  set +e
  e2fsck -fvy "$loop"
  set -e

  local min_blocks
  min_blocks="$(resize2fs -P "$loop" | awk -F': ' '{print $2}')"

  local free_blocks=$(( free_mib * 1024 * 1024 / 4096 ))
  local target_blocks=$(( min_blocks + free_blocks ))

  resize2fs -p "$loop" "$target_blocks"
  sleep 1
  losetup -d "$loop"

  local partnewsize=$(( target_blocks * 4096 ))
  local newend=$(( partstart + partnewsize ))

  parted "$IMG" rm "$partnumber" >/dev/null
  parted "$IMG" unit B mkpart primary "$partstart" "$newend" >/dev/null

  local endresult
  endresult="$(parted -m "$IMG" unit B print free | tail -1 | awk -F: '{print substr($2,0,length($2)-1)}')"
  truncate -s "$endresult" "$IMG"
}

case "$MODE" in
  max) resize_max "${ARG:?need size, e.g. 7G}" ;;
  min) shrink_min "${ARG:-200}" ;;
  *) echo "Usage: $0 <img> max <7G> | min <FREE_MIB>"; exit 2 ;;
esac
