#!/usr/bin/env bash
set -euo pipefail

BUILD_START_EPOCH="$(date +%s)"

ENV_FILE="${1:?env}"
CACHE_DIR="${2:?cache}"
BUILD_DIR="${3:?build}"
OUT_DIR="${4:?out}"

source "$ENV_FILE"
mkdir -p "$BUILD_DIR" "$OUT_DIR"

MIDDLE_IMG="$BUILD_DIR/middle.img"
WORK_IMG="$BUILD_DIR/work.img"

if [[ ! -f "$MIDDLE_IMG" ]]; then
  echo "Missing $MIDDLE_IMG. Run: make middle (or make build)"
  exit 1
fi

cp -f "$MIDDLE_IMG" "$WORK_IMG"
sync

bash "$(dirname "$0")/image_resize.sh" "$WORK_IMG" max "$EXPAND_TO"

# Copy drone workspace into final image if requested
if [[ "${INSTALL_DRONE:-false}" == "true" ]]; then
  if [[ -z "${DRONE_WS_SRC:-}" ]]; then
    echo "INSTALL_DRONE=true but DRONE_WS_SRC is empty"
    exit 2
  fi
  if [[ ! -d "$DRONE_WS_SRC" ]]; then
    echo "DRONE_WS_SRC not found: $DRONE_WS_SRC"
    exit 2
  fi
  bash "$(dirname "$0")/image_chroot.sh" "$WORK_IMG" copy "$DRONE_WS_SRC" "/home/${PI_USER:-pi}/catkin_ws"
fi

# Copy chroot runner + entrypoint into image
bash "$(dirname "$0")/image_chroot.sh" "$WORK_IMG" copy "$(dirname "$0")/chroot" "/root"
bash "$(dirname "$0")/image_chroot.sh" "$WORK_IMG" copy "$(dirname "$0")/chroot.sh" "/root/chroot.sh"

STAMP="$(date +%Y%m%d_%H%M%S)"
BUILD_VERSION="$STAMP"

ENABLE_SSH="$ENABLE_SSH" \
PI_USER="$PI_USER" \
PI_PASSWORD="$PI_PASSWORD" \
EXTRA_APT_PACKAGES="$EXTRA_APT_PACKAGES" \
INSTALL_ROS="false" \
ROS_DISTRO="${ROS_DISTRO:-noetic}" \
INSTALL_DRONE="${INSTALL_DRONE:-false}" \
BUILD_VERSION="$BUILD_VERSION" \
bash "$(dirname "$0")/image_chroot.sh" "$WORK_IMG" exec "$(dirname "$0")/chroot.sh" final

bash "$(dirname "$0")/image_resize.sh" "$WORK_IMG" min "$FINAL_FREE_MIB"

OUT_IMG="$OUT_DIR/${OUT_NAME_PREFIX}_${STAMP}.img"
OUT_XZ="${OUT_IMG}.xz"

mv -f "$WORK_IMG" "$OUT_IMG"
sync

echo "Compressing image to xz..."
xz -T0 -6 --verbose "$OUT_IMG"
rm -f "$OUT_IMG"

BUILD_END_EPOCH="$(date +%s)"
BUILD_SECS="$((BUILD_END_EPOCH - BUILD_START_EPOCH))"
printf "FINAL BUILD TIME: %02d:%02d:%02d\n" $((BUILD_SECS/3600)) $(((BUILD_SECS%3600)/60)) $((BUILD_SECS%60))

echo "DONE: $OUT_XZ"
ls -lh "$OUT_XZ"
