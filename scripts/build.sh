#!/usr/bin/env bash
set -euo pipefail

BUILD_START_EPOCH="$(date +%s)"

ENV_FILE="${1:?env}"
CACHE_DIR="${2:?cache}"
BUILD_DIR="${3:?build}"
OUT_DIR="${4:?out}"

source "$ENV_FILE"
mkdir -p "$BUILD_DIR" "$OUT_DIR"

BASE_IMG="$BUILD_DIR/base.img"
WORK_IMG="$BUILD_DIR/work.img"

if [[ ! -f "$BASE_IMG" ]]; then
  echo "Missing $BASE_IMG. Run: make fetch && make unpack"
  exit 1
fi

cp -f "$BASE_IMG" "$WORK_IMG"
sync

bash "$(dirname "$0")/image_resize.sh" "$WORK_IMG" max "$EXPAND_TO"

# Если нужно — копируем drone workspace в образ ДО provision
if [[ "${INSTALL_DRONE:-false}" == "true" ]]; then
  if [[ -z "${DRONE_WS_SRC:-}" ]]; then
    echo "INSTALL_DRONE=true but DRONE_WS_SRC is empty"
    exit 2
  fi
  if [[ ! -d "$DRONE_WS_SRC" ]]; then
    echo "DRONE_WS_SRC not found: $DRONE_WS_SRC"
    exit 2
  fi

  # Копируем в /home/pi/catkin_ws (пользователь создастся в provision, но файлы уже будут)
  bash "$(dirname "$0")/image_chroot.sh" "$WORK_IMG" copy "$DRONE_WS_SRC" "/home/${PI_USER:-pi}/catkin_ws"
fi

TMP_PROV="$BUILD_DIR/provision.sh"
cp -a "$(dirname "$0")/provision.sh" "$TMP_PROV"

# Build version for /etc/drone_version (from final image name stamp)
STAMP="$(date +%Y%m%d_%H%M%S)"
BUILD_VERSION="$STAMP"

ENABLE_SSH="$ENABLE_SSH" \
PI_USER="$PI_USER" \
PI_PASSWORD="$PI_PASSWORD" \
EXTRA_APT_PACKAGES="$EXTRA_APT_PACKAGES" \
INSTALL_ROS="${INSTALL_ROS:-false}" \
ROS_DISTRO="${ROS_DISTRO:-noetic}" \
INSTALL_DRONE="${INSTALL_DRONE:-false}" \
BUILD_VERSION="$BUILD_VERSION" \
bash "$(dirname "$0")/image_chroot.sh" "$WORK_IMG" exec "$TMP_PROV"

bash "$(dirname "$0")/image_resize.sh" "$WORK_IMG" min "$FINAL_FREE_MIB"

# --- финальное имя образа ---
OUT_IMG="$OUT_DIR/${OUT_NAME_PREFIX}_${STAMP}.img"
OUT_XZ="${OUT_IMG}.xz"

# переносим work.img в out под финальным именем
mv -f "$WORK_IMG" "$OUT_IMG"
sync

echo "Compressing image to xz..."
# -T0  → все ядра
# -6   → баланс скорость/размер
xz -T0 -6 --verbose "$OUT_IMG"

# Удаляем исходный .img, оставляем только .xz
rm -f "$OUT_IMG"

# Время сборки (до DONE, как ты просил)
BUILD_END_EPOCH="$(date +%s)"
BUILD_SECS="$((BUILD_END_EPOCH - BUILD_START_EPOCH))"

HOURS=$((BUILD_SECS / 3600))
MINUTES=$(((BUILD_SECS % 3600) / 60))
SECONDS=$((BUILD_SECS % 60))

printf "BUILD TIME: %02d:%02d:%02d\n" "$HOURS" "$MINUTES" "$SECONDS"

echo "DONE: $OUT_XZ"
ls -lh "$OUT_XZ"
