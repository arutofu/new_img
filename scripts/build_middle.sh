#!/usr/bin/env bash
set -euo pipefail

BUILD_START_EPOCH="$(date +%s)"

ENV_FILE="${1:?env}"
CACHE_DIR="${2:?cache}"
BUILD_DIR="${3:?build}"

source "$ENV_FILE"
mkdir -p "$BUILD_DIR"

BASE_IMG="$BUILD_DIR/base.img"
MIDDLE_IMG="$BUILD_DIR/middle.img"
WORK_IMG="$BUILD_DIR/middle_work.img"

if [[ ! -f "$BASE_IMG" ]]; then
  echo "Missing $BASE_IMG. Run: make fetch && make unpack"
  exit 1
fi

MIDDLE_EXPAND_TO="${MIDDLE_EXPAND_TO:-$EXPAND_TO}"
MIDDLE_FREE_MIB="${MIDDLE_FREE_MIB:-2000}"

cp -f "$BASE_IMG" "$WORK_IMG"
sync

bash "$(dirname "$0")/image_resize.sh" "$WORK_IMG" max "$MIDDLE_EXPAND_TO"

# Copy chroot runner + entrypoint into image
bash "$(dirname "$0")/image_chroot.sh" "$WORK_IMG" copy "$(dirname "$0")/chroot" "/root"
bash "$(dirname "$0")/image_chroot.sh" "$WORK_IMG" copy "$(dirname "$0")/chroot.sh" "/root/chroot.sh"

STAMP="$(date +%Y%m%d_%H%M%S)"
BUILD_VERSION="middle_${STAMP}"

ENABLE_SSH="$ENABLE_SSH" \
PI_USER="$PI_USER" \
PI_PASSWORD="$PI_PASSWORD" \
EXTRA_APT_PACKAGES="$EXTRA_APT_PACKAGES" \
INSTALL_ROS="${INSTALL_ROS:-false}" \
ROS_DISTRO="${ROS_DISTRO:-noetic}" \
INSTALL_DRONE="false" \
BUILD_VERSION="$BUILD_VERSION" \
bash "$(dirname "$0")/image_chroot.sh" "$WORK_IMG" exec "$(dirname "$0")/chroot.sh" middle

bash "$(dirname "$0")/image_resize.sh" "$WORK_IMG" min "$MIDDLE_FREE_MIB"

mv -f "$WORK_IMG" "$MIDDLE_IMG"
sync

BUILD_END_EPOCH="$(date +%s)"
BUILD_SECS="$((BUILD_END_EPOCH - BUILD_START_EPOCH))"
printf "MIDDLE BUILD TIME: %02d:%02d:%02d\n" $((BUILD_SECS/3600)) $(((BUILD_SECS%3600)/60)) $((BUILD_SECS%60))

echo "DONE: $MIDDLE_IMG"
ls -lh "$MIDDLE_IMG"
