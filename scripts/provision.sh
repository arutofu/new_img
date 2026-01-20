#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

# ============================================================
# provision.sh (Clover-like)
# - fixes apt order + buster archived repos
# - pinned cmake 3.13.4-1 (buster)
# - adds systemd services: drone(roslaunch), butterfly(57575)
# - installs ros web tools: rosbridge-server + web_video_server
# - installs MAVROS + pymavlink for selfcheck
# - fixes Wi-Fi AP: rfkill unblock before hostapd + dnsmasq waits for wlan0
# - adds WPA2 password for Wi-Fi AP (default: dronewifi)
# - fixes roswww_static autogen (drone-www.service) vs symlinked ~/.ros/www/*
# - auto-sources ROS env on login (ssh/tty) so rosrun/roslaunch work
# - installs udev rules from catkin_ws/src/drone/udev (if present)
# - adds package entrypoint so `rosrun drone selfcheck` works
# ============================================================

# -----------------------------
# Settings (env overrides)
# -----------------------------
PI_USER="${PI_USER:-pi}"
PI_PASSWORD="${PI_PASSWORD:-raspberry}"
HOST_NAME="${HOST_NAME:-raspberry}"

INSTALL_ROS="${INSTALL_ROS:-false}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"

INSTALL_DRONE="${INSTALL_DRONE:-false}"
CATKIN_WS="${CATKIN_WS:-/home/${PI_USER}/catkin_ws}"
DRONE_SRC_DIR="${DRONE_SRC_DIR:-${CATKIN_WS}/src}"

# Networking / services
ENABLE_WIFI_AP="${ENABLE_WIFI_AP:-true}"
WIFI_COUNTRY="${WIFI_COUNTRY:-DE}"
WIFI_CHANNEL="${WIFI_CHANNEL:-7}"
WIFI_AP_IP="${WIFI_AP_IP:-192.168.11.1}"
WIFI_AP_CIDR="${WIFI_AP_CIDR:-24}"
WIFI_DHCP_START="${WIFI_DHCP_START:-192.168.11.20}"
WIFI_DHCP_END="${WIFI_DHCP_END:-192.168.11.200}"
WIFI_DHCP_LEASE="${WIFI_DHCP_LEASE:-12h}"

# Wi-Fi security (WPA2-PSK). Default password per your request.
WIFI_AP_PSK="${WIFI_AP_PSK:-dronewifi}"

ENABLE_MDNS="${ENABLE_MDNS:-true}"

ENABLE_WEB="${ENABLE_WEB:-true}"
WEB_ROOT_OVERRIDE="${WEB_ROOT_OVERRIDE:-}" # if empty -> use /home/pi/.ros/www

ENABLE_FILEBROWSER="${ENABLE_FILEBROWSER:-true}"
FILEBROWSER_PORT="${FILEBROWSER_PORT:-8090}"

AUTOLOGIN_TTY1="${AUTOLOGIN_TTY1:-true}"

EXTRA_APT_PACKAGES="${EXTRA_APT_PACKAGES:-}"

# Optional services / web tooling
ENABLE_ROS_AUTOSTART="${ENABLE_ROS_AUTOSTART:-true}"

# NOTE: In Clover-like setup rosbridge is started from drone.launch.
# Standalone rosbridge service is OFF by default to avoid duplicate masters/run_id mismatch.
ENABLE_ROSBRIDGE_SERVICE="${ENABLE_ROSBRIDGE_SERVICE:-false}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"

ENABLE_BUTTERFLY="${ENABLE_BUTTERFLY:-true}"
BUTTERFLY_PORT="${BUTTERFLY_PORT:-57575}"

# Install web_video_server package (used by drone.launch). Standalone systemd service is OFF by default.
ENABLE_WEB_VIDEO_SERVER="${ENABLE_WEB_VIDEO_SERVER:-true}"
ENABLE_WEB_VIDEO_SERVER_SERVICE="${ENABLE_WEB_VIDEO_SERVER_SERVICE:-false}"
WEB_VIDEO_PORT="${WEB_VIDEO_PORT:-8080}"

# Clover pinned CMake (buster native)
CMAKE_VER="${CMAKE_VER:-3.13.4-1}"
CMAKE_DATA_VER="${CMAKE_DATA_VER:-3.13.4-1}"

# -----------------------------
# Logging
# -----------------------------
echo_stamp() {
  local text type
  text="$(date '+[%Y-%m-%d %H:%M:%S]') $1"
  type="${2:-INFO}"
  text="\e[1m${text}\e[0m"
  case "$type" in
    SUCCESS) text="\e[32m${text}\e[0m" ;;
    ERROR)   text="\e[31m${text}\e[0m" ;;
    *)       text="\e[34m${text}\e[0m" ;;
  esac
  echo -e "${text}"
}
info() { echo_stamp "$*" "INFO"; }
ok()   { echo_stamp "$*" "SUCCESS"; }
warn() { echo_stamp "$*" "ERROR"; }
die()  { warn "$*"; exit 1; }

have_cmd() { command -v "$1" >/dev/null 2>&1; }

require_root() {
  [[ "$(id -u)" == "0" ]] || die "Must be run as root (inside chroot). uid=$(id -u)"
}

print_kv() { printf "  - %-24s: %s\n" "$1" "$2"; }

# -----------------------------
# Retry helper (Clover-style)
# -----------------------------
my_travis_retry() {
  local result=0 count=1
  while [ "$count" -le 3 ]; do
    [ "$result" -ne 0 ] && {
      echo -e "\nThe command \"$*\" failed. Retrying, $count of 3.\n" >&2
    }
    ! { "$@"; result=$?; }
    [ "$result" -eq 0 ] && break
    count=$((count + 1))
    sleep 1
  done
  [ "$count" -gt 3 ] && echo -e "\nThe command \"$*\" failed 3 times.\n" >&2
  return "$result"
}

# -----------------------------
# Diagnostics
# -----------------------------
os_info() {
  info "OS / arch info"
  if [[ -f /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    print_kv "PRETTY_NAME" "${PRETTY_NAME:-unknown}"
    print_kv "VERSION_CODENAME" "${VERSION_CODENAME:-unknown}"
    print_kv "ID" "${ID:-unknown}"
  fi
  print_kv "ARCH" "$(dpkg --print-architecture 2>/dev/null || echo unknown)"
  print_kv "KERNEL" "$(uname -a 2>/dev/null || echo unknown)"
  print_kv "WHOAMI" "$(whoami)"
  print_kv "LANG" "${LANG:-<unset>}"
}

fs_sanity() {
  info "Filesystem sanity checks"
  [[ -d /etc ]] || die "/etc missing (broken rootfs)"
  [[ -d /var/lib/dpkg ]] || die "/var/lib/dpkg missing (dpkg DB missing)"
  [[ -e /dev/null ]] || die "/dev/null missing (is /dev bind-mounted?)"
  [[ -d /proc ]] || die "/proc missing"
  [[ -d /sys ]] || die "/sys missing"
}

# -----------------------------
# APT config + sources (do BEFORE any apt-get update)
# -----------------------------
apt_configure_archived_repos() {
  info "Configure apt retries + archive-friendly options"
  cat >/etc/apt/apt.conf.d/80-retries <<'EOF'
APT::Acquire::Retries "3";
Acquire::Retries "3";
Acquire::Check-Valid-Until "false";
Acquire::AllowReleaseInfoChange "true";
Acquire::AllowReleaseInfoChange::Suite "true";
Acquire::AllowReleaseInfoChange::Codename "true";
Acquire::AllowReleaseInfoChange::Version "true";
EOF
}

apt_fix_sources_buster() {
  info "Fix APT sources for Raspbian Buster (avoid dead raspbian.raspberrypi.org)"
  rm -f /etc/apt/sources.list.d/*.list 2>/dev/null || true

  cat >/etc/apt/sources.list <<'EOF'
deb http://legacy.raspbian.org/raspbian buster main contrib non-free rpi
deb http://archive.raspberrypi.org/debian buster main
EOF

  [[ -s /etc/apt/sources.list ]] || die "sources.list empty after rewrite"
  info "Current /etc/apt/sources.list:"
  sed -n '1,120p' /etc/apt/sources.list || true
}

apt_update() {
  info "apt-get update"
  LC_ALL=C my_travis_retry apt-get update -y --allow-releaseinfo-change
}

apt_install() {
  info "apt-get install: $*"
  LC_ALL=C my_travis_retry apt-get install -y --no-install-recommends "$@"
}

apt_remove() {
  info "apt-get remove: $*"
  LC_ALL=C my_travis_retry apt-get remove -y "$@" || true
}

apt_policy_dump() {
  info "APT sources + cmake policy (diagnostics)"
  echo "----- /etc/apt/sources.list -----"
  [[ -f /etc/apt/sources.list ]] && sed -n '1,200p' /etc/apt/sources.list || true
  echo "----- /etc/apt/sources.list.d/*.list -----"
  ls -la /etc/apt/sources.list.d 2>/dev/null || true
  for f in /etc/apt/sources.list.d/*.list; do
    [[ -f "$f" ]] || continue
    echo "----- $f -----"
    sed -n '1,200p' "$f" || true
  done
  echo "----- apt-cache policy cmake -----"
  (apt-cache policy cmake || true) | sed -n '1,160p'
  echo "----- dpkg -l | grep cmake -----"
  (dpkg -l | grep -E '^(ii|rc)\s+cmake(\s|$)|^(ii|rc)\s+cmake-data(\s|$)' || true)
}

# -----------------------------
# Locale fix
# -----------------------------
ensure_locales() {
  info "Ensure locales (ru_RU.UTF-8) to avoid apt/perl warnings"
  apt_install locales || true

  if [[ -f /etc/locale.gen ]]; then
    sed -i 's/^\s*#\s*\(ru_RU.UTF-8 UTF-8\)/\1/' /etc/locale.gen 2>/dev/null || true
    sed -i 's/^\s*#\s*\(en_US.UTF-8 UTF-8\)/\1/' /etc/locale.gen 2>/dev/null || true
  fi

  if have_cmd locale-gen; then
    LC_ALL=C locale-gen ru_RU.UTF-8 || true
    LC_ALL=C locale-gen en_US.UTF-8 || true
  fi

  if have_cmd update-locale; then
    LC_ALL=C update-locale LANG=ru_RU.UTF-8 LC_ALL=ru_RU.UTF-8 || true
  fi

  cat >/etc/default/locale <<'EOF'
LANG=ru_RU.UTF-8
LC_ALL=ru_RU.UTF-8
EOF

  export LANG=ru_RU.UTF-8
  info "Locale check:"
  (locale || true) | sed -n '1,80p'
}

# -----------------------------
# Add repos/keys like Clover (ROS + Coex)
# -----------------------------
add_ros_and_coex_repos_like_clover() {
  info "Install apt tools for keys/repos"
  apt_install ca-certificates wget curl gnupg dirmngr lsb-release apt-transport-https || true

  if ! apt-key list 2>/dev/null | grep -q "F42ED6FBAB17C654"; then
    info "Add ROS apt key (best-effort)"
    my_travis_retry apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
      --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 || true
  fi

  if [[ ! -f /etc/apt/sources.list.d/ros-latest.list ]]; then
    echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-latest.list
  fi

  if ! apt-key list 2>/dev/null | grep -qi "packages.coex.tech"; then
    info "Add Coex apt key (best-effort)"
    my_travis_retry wget -qO- 'http://packages.coex.tech/key.asc' | apt-key add - || true
  fi

  if ! grep -q "packages.coex.tech" /etc/apt/sources.list 2>/dev/null; then
    echo 'deb http://packages.coex.tech buster main' >> /etc/apt/sources.list
  fi
}

# -----------------------------
# CMake sanity
# -----------------------------
cmake_sanity_test() {
  have_cmd cmake || return 1
  have_cmd cc || return 1

  local tmpd
  tmpd="$(mktemp -d)"

  cat >"${tmpd}/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.0)
project(cmake_sanity C)
add_executable(hello main.c)
EOF
  cat >"${tmpd}/main.c" <<'EOF'
#include <stdio.h>
int main(){ puts("ok"); return 0; }
EOF

  if (cd "$tmpd" && cmake -G "Unix Makefiles" . >/dev/null 2>&1); then
    rm -rf "$tmpd"
    return 0
  fi

  echo "----- cmake sanity FAILED; logs -----" >&2
  [[ -f "${tmpd}/CMakeFiles/CMakeError.log" ]] && tail -n 120 "${tmpd}/CMakeFiles/CMakeError.log" >&2 || true
  rm -rf "$tmpd"
  return 1
}

install_cmake_like_clover() {
  info "Install CMake pinned like Clover: cmake-data=${CMAKE_DATA_VER} cmake=${CMAKE_VER}"

  apt_remove cmake cmake-data || true

  if ! apt_install "cmake-data=${CMAKE_DATA_VER}" "cmake=${CMAKE_VER}"; then
    apt_policy_dump
    die "Failed to install pinned CMake versions (cmake=${CMAKE_VER}, cmake-data=${CMAKE_DATA_VER})."
  fi

  apt-mark hold cmake cmake-data >/dev/null 2>&1 || true

  info "CMake version now: $(cmake --version | head -n 1 || true)"
  if ! cmake_sanity_test; then
    apt_policy_dump
    die "Pinned CMake installed, but sanity test still fails. Check /dev bind mounts and repo mix."
  fi
  ok "CMake sanity OK"
}

# -----------------------------
# Base deps + user/ssh
# -----------------------------
ensure_user_and_hostname() {
  info "Ensure user ${PI_USER} and hostname ${HOST_NAME}"
  apt_install sudo openssh-server || true

  if id "${PI_USER}" >/dev/null 2>&1; then
    info "User exists: ${PI_USER}"
  else
    useradd -m -s /bin/bash "${PI_USER}"
  fi

  if [[ -n "${PI_PASSWORD}" ]]; then
    echo "${PI_USER}:${PI_PASSWORD}" | chpasswd
  else
    passwd -d "${PI_USER}" >/dev/null 2>&1 || true
  fi

  cat >/etc/sudoers.d/010-"${PI_USER}"-nopasswd <<EOF
${PI_USER} ALL=(ALL) NOPASSWD:ALL
EOF
  chmod 0440 /etc/sudoers.d/010-"${PI_USER}"-nopasswd

  echo "${HOST_NAME}" > /etc/hostname
  hostname "${HOST_NAME}" 2>/dev/null || true
  sed -i "s/127.0.1.1.*/127.0.1.1\t${HOST_NAME}/g" /etc/hosts 2>/dev/null || true

  systemctl enable ssh 2>/dev/null || true

  if [[ -z "${PI_PASSWORD}" ]]; then
    if [[ -f /etc/ssh/sshd_config ]]; then
      sed -i 's/^#\?PasswordAuthentication\s\+.*/PasswordAuthentication yes/' /etc/ssh/sshd_config || true
      sed -i 's/^#\?PermitEmptyPasswords\s\+.*/PermitEmptyPasswords yes/' /etc/ssh/sshd_config || true
      grep -q '^PasswordAuthentication' /etc/ssh/sshd_config || echo 'PasswordAuthentication yes' >> /etc/ssh/sshd_config
      grep -q '^PermitEmptyPasswords' /etc/ssh/sshd_config || echo 'PermitEmptyPasswords yes' >> /etc/ssh/sshd_config
    fi
  fi
}

install_additional_software(){
  info "Install additional software"
  apt-get update
  apt-get install --no-install-recommends -y \
  unzip \
  zip \
  ipython \
  ipython3 \
  screen \
  byobu  \
  nmap \
  lsof \
  git \
  dnsmasq  \
  tmux \
  tree \
  vim \
  libjpeg8 \
  tcpdump \
  libpoco-dev \
  libzbar0 \
  python3-rosdep \
  python3-rosinstall-generator \
  python3-wstool \
  python3-rosinstall \
  build-essential \
  libffi-dev \
  monkey \
  pigpio python-pigpio python3-pigpio \
  i2c-tools \
  espeak espeak-data python-espeak python3-espeak \
  ntpdate \
  python-dev \
  python3-dev \
  python-systemd \
  mjpg-streamer \
  python3-opencv
}

install_extra_packages() {
  if [[ -n "${EXTRA_APT_PACKAGES}" ]]; then
    info "Install EXTRA_APT_PACKAGES: ${EXTRA_APT_PACKAGES}"
    # shellcheck disable=SC2086
    LC_ALL=C my_travis_retry apt-get install -y --no-install-recommends ${EXTRA_APT_PACKAGES} || true
  fi
}

# -----------------------------
# ROS install (optional)
# -----------------------------
install_ros_if_needed() {
  if [[ "${INSTALL_ROS}" != "true" ]]; then
    info "INSTALL_ROS=false -> skip ROS install"
    return 0
  fi

  info "Install ROS: ros-${ROS_DISTRO}-ros-base + catkin"
  apt_update

  apt_install \
    "ros-${ROS_DISTRO}-ros-base" \
    "ros-${ROS_DISTRO}-catkin" \
    python3-rosdep || {
      apt_policy_dump
      die "ROS install failed (check repos/keys)."
    }

  [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || die "ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash"
  ok "ROS installed: ${ROS_DISTRO}"
}

install_geographiclib_geoids_like_clover() {
  info "Ensure GeographicLib geoids are installed (needed by MAVROS)"

  if dpkg -l 2>/dev/null | grep -qE '^ii\s+geographiclib-geoids(\s|$)'; then
    ok "geographiclib-geoids package installed"
    return 0
  fi

  if have_cmd geographiclib-get-geoids; then
    info "Running geographiclib-get-geoids egm96-5 (best-effort)"
    my_travis_retry geographiclib-get-geoids egm96-5 || true
  fi

  local mavros_script="/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh"
  if [[ -f "${mavros_script}" ]]; then
    info "Running MAVROS dataset installer (best-effort)"
    /bin/bash "${mavros_script}" || true
  fi

  if [[ -d /usr/share/GeographicLib/geoids ]] || [[ -d /usr/share/geographiclib/geoids ]]; then
    ok "GeographicLib geoids directory present"
    return 0
  fi

  warn "GeographicLib geoids not detected (may still work depending on your use-case)."
  return 0
}

install_ros_web_tools() {
  if [[ "${INSTALL_ROS}" != "true" ]]; then
    info "INSTALL_ROS=false -> skip ROS web tools install"
    return 0
  fi

  info "Install ROS web tools + MAVROS (rosbridge-server + web-video-server + mavros)"
  apt_update

  local pkgs=(
    "ros-${ROS_DISTRO}-rosbridge-server"
  )

  if [[ "${ENABLE_WEB_VIDEO_SERVER}" == "true" ]]; then
    pkgs+=("ros-${ROS_DISTRO}-web-video-server")
  fi

  pkgs+=(
    "ros-${ROS_DISTRO}-mavros"
    "ros-${ROS_DISTRO}-mavros-extras"
    geographiclib-tools
  )

  local optional_pkgs=(
    geographiclib-doc
    python3-rosinstall
    python3-rosinstall-generator
    python3-wstool
  )

  if ! apt_install "${pkgs[@]}"; then
    apt_policy_dump
    die "Failed to install ROS web tools/MAVROS"
  fi

  for p in "${optional_pkgs[@]}"; do
    apt_install "$p" || true
  done

  # pymavlink for python selfcheck (best-effort)
  if apt-cache show python3-pymavlink >/dev/null 2>&1; then
    apt_install python3-pymavlink || true
  else
    info "python3-pymavlink not available via apt -> use apt deps + pip --no-deps to avoid lxml build on buster"
    # avoid building lxml from source (buster pip/pytoml breaks on modern pyproject.toml)
    apt_install python3-lxml python3-future python3-numpy || true
    my_travis_retry pip3 install --no-cache-dir --no-deps "pymavlink==2.4.39" || true
  fi

  install_geographiclib_geoids_like_clover || true

  ok "ROS web tools/MAVROS installed"
}

install_base_build_deps() {
  info "Install base build dependencies"
  apt_install \
    bash coreutils findutils grep sed gawk \
    build-essential make \
    git \
    python3 python3-pip python3-dev \
    pkg-config \
    ca-certificates curl wget || true
  have_cmd cc || die "C compiler not found after build-essential"

  info "Installing OpenCV 4.2-compatible ROS packages"
  apt install -y --no-install-recommends \
  ros-${ROS_DISTRO}-compressed-image-transport=1.14.0-0buster \
  ros-${ROS_DISTRO}-cv-bridge=1.15.0-0buster \
  ros-${ROS_DISTRO}-cv-camera=0.5.1-0buster \
  ros-${ROS_DISTRO}-image-publisher=1.15.3-0buster \
  ros-${ROS_DISTRO}-web-video-server=0.2.1-0buster
  apt-mark hold \
  ros-${ROS_DISTRO}-compressed-image-transport \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-cv-camera \
  ros-${ROS_DISTRO}-image-publisher \
  ros-${ROS_DISTRO}-web-video-server
}

install_aruco_pose_like_clover() {
  if [[ "${INSTALL_ROS}" != "true" ]]; then
    info "INSTALL_ROS=false -> skip aruco_pose"
    return 0
  fi

  info "Install/ensure aruco_pose (local repo, Clover-like behavior)"

  local ws_src="${CATKIN_WS}/src"
  local target="${ws_src}/aruco_pose"

  # Where your repository is located inside the target filesystem
  # Example from you: /home/artem/work/new_img/drone/catkin_ws/src/aruco_pose
  local repo_root="${CATKIN_WS}/src/aruco_pose"
  local src_local="${repo_root}/drone/catkin_ws/src/aruco_pose"

  mkdir -p "${ws_src}"

  # If already present in workspace - OK
  if [[ -d "${target}" ]]; then
    ok "aruco_pose already present in workspace: ${target}"
  else
    # Prefer local copy (no network)
    if [[ -d "${src_local}" ]]; then
      info "Copy aruco_pose from local repo: ${src_local} -> ${target}"
      cp -a "${src_local}" "${target}"
      ok "Copied aruco_pose into workspace: ${target}"
    else
      warn "Local aruco_pose not found at ${src_local}"
      warn "Set NEW_IMG_SRC_ROOT to the repo path inside image, or add aruco_pose via other method."
      return 0
    fi
  fi

  # Build it so it becomes runnable via rosrun/roslaunch
  if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    info "Rebuild catkin_ws to compile aruco_pose"
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash" || true
    set -u
    (cd "${CATKIN_WS}" && catkin_make 2>&1 | tee /var/log/drone_catkin_make_aruco.log) || {
      warn "catkin_make for aruco_pose failed (see /var/log/drone_catkin_make_aruco.log)"
      return 0
    }
    ok "aruco_pose build OK"
  else
    warn "ROS not found at /opt/ros/${ROS_DISTRO}/setup.bash -> cannot build aruco_pose"
  fi
}

# -----------------------------
# Drone build (optional)
# -----------------------------
verify_catkin_workspace() {
  info "Verify catkin workspace layout"
  print_kv "CATKIN_WS" "${CATKIN_WS}"
  print_kv "DRONE_SRC_DIR" "${DRONE_SRC_DIR}"

  [[ -d "${CATKIN_WS}" ]] || die "CATKIN_WS missing: ${CATKIN_WS}"
  [[ -d "${DRONE_SRC_DIR}" ]] || die "src dir missing: ${DRONE_SRC_DIR}"

  local cnt
  cnt="$(find "${DRONE_SRC_DIR}" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l | tr -d ' ')"
  [[ "${cnt}" -gt 0 ]] || die "src dir looks empty: ${DRONE_SRC_DIR}"
}

build_drone_if_needed() {
  if [[ "${INSTALL_DRONE}" != "true" ]]; then
    info "INSTALL_DRONE=false -> skip drone build"
    return 0
  fi

  info "Build drone/catkin_ws requested"
  [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || die "ROS required for catkin build, missing /opt/ros/${ROS_DISTRO}/setup.bash"

  verify_catkin_workspace

  apt_install \
    "ros-${ROS_DISTRO}-nodelet" \
    "ros-${ROS_DISTRO}-pluginlib" \
    "ros-${ROS_DISTRO}-roscpp" \
    "ros-${ROS_DISTRO}-rospy" \
    "ros-${ROS_DISTRO}-std-msgs" \
    "ros-${ROS_DISTRO}-message-generation" \
    "ros-${ROS_DISTRO}-message-runtime" \
    "ros-${ROS_DISTRO}-geometry-msgs" \
    "ros-${ROS_DISTRO}-sensor-msgs" \
    "ros-${ROS_DISTRO}-geographic-msgs" \
    "ros-${ROS_DISTRO}-tf" \
    "ros-${ROS_DISTRO}-tf2" \
    "ros-${ROS_DISTRO}-tf2-geometry-msgs" \
    "ros-${ROS_DISTRO}-tf2-ros" \
    "ros-${ROS_DISTRO}-image-transport" \
    "ros-${ROS_DISTRO}-cv-bridge" \
    "ros-${ROS_DISTRO}-dynamic-reconfigure" \
    "ros-${ROS_DISTRO}-mavros-msgs" \
    libgeographic-dev \
    libopencv-dev || {
      apt_policy_dump
      die "Failed to install required deps for drone build"
    }

  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u

  have_cmd catkin_make || die "catkin_make not found after sourcing ROS"

  rm -rf "${CATKIN_WS}/build" "${CATKIN_WS}/devel" "${CATKIN_WS}/install"
  info "catkin_make"
  (cd "${CATKIN_WS}" && catkin_make 2>&1 | tee /var/log/drone_catkin_make.log) || {
    warn "catkin_make failed. Tail of /var/log/drone_catkin_make.log:"
    tail -n 200 /var/log/drone_catkin_make.log || true
    die "Drone build failed."
  }

  ok "Drone build OK"
}

additional_ROS_packages(){
  info "Installing additional ROS packages"
  apt-get update
  apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-rosserial \
    ros-${ROS_DISTRO}-usb-cam \
    ros-${ROS_DISTRO}-vl53l1x \
    ros-${ROS_DISTRO}-ws281x \
    ros-${ROS_DISTRO}-rosshow \
    ros-${ROS_DISTRO}-cmake-modules \
    ros-${ROS_DISTRO}-image-view \
    ros-${ROS_DISTRO}-nodelet-topic-tools \
    ros-${ROS_DISTRO}-stereo-msgs \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-angles
}

# -----------------------------
# Console autologin
# -----------------------------
enable_console_autologin() {
  [[ "${AUTOLOGIN_TTY1}" == "true" ]] || { info "AUTOLOGIN_TTY1=false -> skip"; return 0; }
  info "Enable console autologin for ${PI_USER} on tty1"
  mkdir -p /etc/systemd/system/getty@tty1.service.d
  cat >/etc/systemd/system/getty@tty1.service.d/autologin.conf <<EOF
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin ${PI_USER} --noclear %I \$TERM
EOF
}

# -----------------------------
# mDNS (raspberry.local)
# -----------------------------
setup_mdns_avahi() {
  [[ "${ENABLE_MDNS}" == "true" ]] || { info "ENABLE_MDNS=false -> skip mDNS"; return 0; }
  info "Install/enable avahi-daemon for .local name (raspberry.local)"
  apt_install avahi-daemon avahi-utils || true
  systemctl enable avahi-daemon 2>/dev/null || true
  systemctl enable avahi-daemon.service 2>/dev/null || true
}

# -----------------------------
# Wi-Fi AP fixes
# -----------------------------
setup_wifi_rfkill_unblock_service() {
  [[ "${ENABLE_WIFI_AP}" == "true" ]] || return 0
  info "Add rfkill-unblock service (before hostapd)"
  cat >/etc/systemd/system/drone-wifi-unblock.service <<'EOF'
[Unit]
Description=Unblock Wi-Fi (rfkill) before AP services
Before=hostapd.service dnsmasq.service
Wants=sys-subsystem-rfkill-devices-phy0.device
After=sys-subsystem-rfkill-devices-phy0.device

[Service]
Type=oneshot
ExecStart=/usr/sbin/rfkill unblock all

[Install]
WantedBy=multi-user.target
EOF
  systemctl enable drone-wifi-unblock.service 2>/dev/null || true
}

setup_wifi_ap() {
  [[ "${ENABLE_WIFI_AP}" == "true" ]] || { info "ENABLE_WIFI_AP=false -> skip Wi-Fi AP"; return 0; }

  info "Setup Wi-Fi Access Point (hostapd/dnsmasq)"
  apt_install hostapd dnsmasq iptables iptables-persistent rfkill || true

  setup_wifi_rfkill_unblock_service

  systemctl disable hostapd 2>/dev/null || true
  systemctl disable dnsmasq 2>/dev/null || true
  systemctl stop hostapd 2>/dev/null || true
  systemctl stop dnsmasq 2>/dev/null || true

  if [[ -f /etc/dhcpcd.conf ]]; then
    if ! grep -q "^interface wlan0" /etc/dhcpcd.conf; then
      cat >>/etc/dhcpcd.conf <<EOF

interface wlan0
  static ip_address=${WIFI_AP_IP}/${WIFI_AP_CIDR}
  nohook wpa_supplicant
EOF
    fi
  fi

  mkdir -p /etc/dnsmasq.d
  cat >/etc/dnsmasq.d/drone-ap.conf <<EOF
interface=wlan0
bind-dynamic
dhcp-range=${WIFI_DHCP_START},${WIFI_DHCP_END},255.255.255.0,${WIFI_DHCP_LEASE}
domain-needed
bogus-priv
EOF

  mkdir -p /etc/hostapd

  cat >/etc/hostapd/hostapd.conf <<EOF
country_code=${WIFI_COUNTRY}
interface=wlan0
driver=nl80211
ssid=drone-0000
hw_mode=g
channel=${WIFI_CHANNEL}
ieee80211n=1
wmm_enabled=1
auth_algs=1

wpa=2
wpa_passphrase=${WIFI_AP_PSK}
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
EOF

  if [[ -f /etc/default/hostapd ]]; then
    sed -i 's/^#\?DAEMON_CONF=.*/DAEMON_CONF="\/etc\/hostapd\/hostapd.conf"/' /etc/default/hostapd || true
    grep -q '^DAEMON_CONF=' /etc/default/hostapd || echo 'DAEMON_CONF="/etc/hostapd/hostapd.conf"' >> /etc/default/hostapd
  else
    cat >/etc/default/hostapd <<'EOF'
DAEMON_CONF="/etc/hostapd/hostapd.conf"
EOF
  fi

  mkdir -p /etc/systemd/system/dnsmasq.service.d
  cat >/etc/systemd/system/dnsmasq.service.d/override.conf <<'EOF'
[Unit]
After=sys-subsystem-net-devices-wlan0.device
Wants=sys-subsystem-net-devices-wlan0.device
EOF

  cat >/etc/sysctl.d/99-drone-ipforward.conf <<'EOF'
net.ipv4.ip_forward=1
EOF

  mkdir -p /etc/iptables
  cat >/etc/iptables/rules.v4 <<'EOF'
*filter
:INPUT ACCEPT [0:0]
:FORWARD ACCEPT [0:0]
:OUTPUT ACCEPT [0:0]
-A FORWARD -i wlan0 -o eth0 -j ACCEPT
-A FORWARD -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
COMMIT
*nat
:PREROUTING ACCEPT [0:0]
:INPUT ACCEPT [0:0]
:OUTPUT ACCEPT [0:0]
:POSTROUTING ACCEPT [0:0]
-A POSTROUTING -o eth0 -j MASQUERADE
COMMIT
EOF

  cat >/usr/local/sbin/drone-generate-ssid.sh <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
CONF=/etc/hostapd/hostapd.conf
STAMP=/var/lib/drone/ssid_generated
mkdir -p /var/lib/drone

[[ -f "$STAMP" ]] && exit 0

rand=$(shuf -i 0-9999 -n 1 2>/dev/null || echo $((RANDOM%10000)))
ssid=$(printf "drone-%04d" "$rand")

if [[ -f "$CONF" ]]; then
  if grep -q '^ssid=' "$CONF"; then
    sed -i "s/^ssid=.*/ssid=${ssid}/" "$CONF"
  else
    echo "ssid=${ssid}" >> "$CONF"
  fi
fi

echo "$ssid" > /var/lib/drone/wifi_ssid
touch "$STAMP"
EOF
  chmod +x /usr/local/sbin/drone-generate-ssid.sh

  cat >/etc/systemd/system/drone-generate-ssid.service <<'EOF'
[Unit]
Description=Generate Wi-Fi AP SSID (drone-XXXX) on first boot
Before=hostapd.service

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/drone-generate-ssid.sh

[Install]
WantedBy=multi-user.target
EOF

  systemctl enable drone-generate-ssid.service 2>/dev/null || true
  systemctl enable hostapd 2>/dev/null || true
  systemctl enable dnsmasq 2>/dev/null || true
}

# -----------------------------
# Filebrowser
# -----------------------------
install_filebrowser_noauth() {
  [[ "${ENABLE_FILEBROWSER}" == "true" ]] || { info "ENABLE_FILEBROWSER=false -> skip filebrowser"; return 0; }

  info "Install Filebrowser (no-auth)"
  local arch asset url tmpd
  arch="$(dpkg --print-architecture 2>/dev/null || echo armhf)"
  case "$arch" in
    armhf) asset="linux-armv7-filebrowser.tar.gz" ;;
    arm64) asset="linux-arm64-filebrowser.tar.gz" ;;
    amd64) asset="linux-amd64-filebrowser.tar.gz" ;;
    *)     asset="linux-armv7-filebrowser.tar.gz" ;;
  esac

  url="https://github.com/filebrowser/filebrowser/releases/latest/download/${asset}"
  tmpd="$(mktemp -d)"

  apt_install ca-certificates curl || true
  my_travis_retry curl -fsSL -o "${tmpd}/fb.tgz" "$url" || {
    rm -rf "$tmpd"
    die "Failed to download filebrowser from GitHub releases (${asset})"
  }
  tar -xzf "${tmpd}/fb.tgz" -C "$tmpd"
  install -m 0755 "${tmpd}/filebrowser" /usr/local/bin/filebrowser
  rm -rf "$tmpd"

  mkdir -p /var/lib/filebrowser
  chown -R "${PI_USER}:${PI_USER}" /var/lib/filebrowser || true

  cat >/etc/systemd/system/filebrowser.service <<EOF
[Unit]
Description=File Browser
After=network-online.target
Wants=network-online.target

[Service]
User=${PI_USER}
Group=${PI_USER}
WorkingDirectory=/
ExecStart=/usr/local/bin/filebrowser \
  --noauth \
  --root=/ \
  --address=0.0.0.0 \
  --port=${FILEBROWSER_PORT} \
  --database=/var/lib/filebrowser/filebrowser.db
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

  systemctl enable filebrowser.service 2>/dev/null || true
}

# -----------------------------
# Nginx static web
# -----------------------------
setup_static_web_like_clover() {
  [[ "${ENABLE_WEB}" == "true" ]] || { info "ENABLE_WEB=false -> skip web"; return 0; }

  info "Setup static web hosting on :80 (nginx)"
  apt_install nginx || true
  systemctl enable nginx 2>/dev/null || true

  local www_root
  if [[ -n "${WEB_ROOT_OVERRIDE}" ]]; then
    www_root="${WEB_ROOT_OVERRIDE}"
  else
    www_root="/home/${PI_USER}/.ros/www"
  fi

  mkdir -p "$www_root"
  chown -R "${PI_USER}:${PI_USER}" "/home/${PI_USER}/.ros" || true

  for pkg in drone drone_blocks; do
    if [[ -L "${www_root}/${pkg}" ]]; then
      rm -f "${www_root}/${pkg}" || true
    fi
  done

  cat >/etc/nginx/sites-available/drone <<EOF
server {
    listen 80 default_server;
    listen [::]:80 default_server;
    server_name ${HOST_NAME}.local ${HOST_NAME} _;
    root ${www_root};
    index index.html;

    location / {
        try_files \$uri \$uri/ =404;
    }
}
EOF
  ln -sf /etc/nginx/sites-available/drone /etc/nginx/sites-enabled/drone
  rm -f /etc/nginx/sites-enabled/default 2>/dev/null || true
}

# -----------------------------
# roswww_static update service
# -----------------------------
setup_roswww_static_update_service() {
  [[ "${ENABLE_WEB}" == "true" ]] || return 0
  info "Setup roswww_static update service (drone/www -> ~/.ros/www)"

  local srcdir
  srcdir="${CATKIN_WS}/src/drone/www"
  [[ -d "$srcdir" ]] || warn "Expected web source dir not found: ${srcdir} (still configuring services)"

  cat >/etc/default/drone-www <<EOF
ROS_DISTRO=${ROS_DISTRO}
ROSWWW_DEFAULT=drone
ROSWWW_STATIC_PATH=${srcdir}
EOF

cat >/usr/local/sbin/drone-www-update.sh <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

if [[ -f /etc/default/drone-www ]]; then
  # shellcheck disable=SC1091
  . /etc/default/drone-www
fi

export ROSWWW_DEFAULT="${ROSWWW_DEFAULT:-drone}"
export ROSWWW_STATIC_PATH="${ROSWWW_STATIC_PATH:-/home/pi/catkin_ws/src/drone/www}"

WWW_DIR="/home/pi/.ros/www"
mkdir -p "${WWW_DIR}"

# IMPORTANT:
# roswww_static expects real directories under ~/.ros/www.
rm -rf "${WWW_DIR}/drone" "${WWW_DIR}/drone_blocks" 2>/dev/null || true

setup_ros="/opt/ros/${ROS_DISTRO:-noetic}/setup.bash"
setup_ws="/home/pi/catkin_ws/devel/setup.bash"

set +u
[[ -f "$setup_ros" ]] && source "$setup_ros"
[[ -f "$setup_ws"  ]] && source "$setup_ws"
set -u

if [[ -x "/home/pi/catkin_ws/devel/lib/roswww_static/update" ]]; then
  exec "/home/pi/catkin_ws/devel/lib/roswww_static/update"
fi

exec rosrun roswww_static update
EOF
  chmod +x /usr/local/sbin/drone-www-update.sh
  chown "${PI_USER}:${PI_USER}" /usr/local/sbin/drone-www-update.sh || true

  cat >/etc/systemd/system/drone-www.service <<EOF
[Unit]
Description=Generate Drone web pages (roswww_static)
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
User=${PI_USER}
Group=${PI_USER}
ExecStart=/usr/local/sbin/drone-www-update.sh

[Install]
WantedBy=multi-user.target
EOF

  systemctl enable drone-www.service 2>/dev/null || true
}

# -----------------------------
# ROS systemd services
# -----------------------------
write_ros_env_file() {
  cat >/etc/default/ros <<EOF
ROS_DISTRO=${ROS_DISTRO}
ROS_MASTER_URI=http://localhost:11311
EOF
}

setup_rosbridge_service() {
  [[ "${ENABLE_ROS_AUTOSTART}" == "true" ]] || { info "ENABLE_ROS_AUTOSTART=false -> skip ros services"; return 0; }
  [[ "${ENABLE_ROSBRIDGE_SERVICE}" == "true" ]] || { info "ENABLE_ROSBRIDGE_SERVICE=false -> skip standalone rosbridge (drone.launch starts it)"; return 0; }

  info "Create rosbridge-websocket systemd service (ws://0.0.0.0:${ROSBRIDGE_PORT})"
  write_ros_env_file

  cat >/etc/systemd/system/rosbridge-websocket.service <<EOF
[Unit]
Description=ROS Bridge WebSocket (rosbridge_server)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
EnvironmentFile=/etc/default/ros
User=${PI_USER}
Group=${PI_USER}
WorkingDirectory=/home/${PI_USER}
ExecStart=/bin/bash -lc 'source /opt/ros/\${ROS_DISTRO}/setup.bash && roslaunch rosbridge_server rosbridge_websocket.launch address:=0.0.0.0 port:=${ROSBRIDGE_PORT}'
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

  systemctl enable rosbridge-websocket.service 2>/dev/null || true
}

setup_drone_launch_service() {
  [[ "${ENABLE_ROS_AUTOSTART}" == "true" ]] || return 0

  info "Create drone systemd service (roslaunch drone drone.launch)"
  write_ros_env_file

  cat >/etc/systemd/system/drone.service <<EOF
[Unit]
Description=Drone main ROS launch
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
EnvironmentFile=/etc/default/ros
User=${PI_USER}
Group=${PI_USER}
WorkingDirectory=${CATKIN_WS}
ExecStart=/bin/bash -lc 'source /opt/ros/\${ROS_DISTRO}/setup.bash && [[ -f ${CATKIN_WS}/devel/setup.bash ]] && source ${CATKIN_WS}/devel/setup.bash; roslaunch drone drone.launch'
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

  systemctl enable drone.service 2>/dev/null || true
}

setup_web_video_server() {
  [[ "${ENABLE_WEB_VIDEO_SERVER_SERVICE}" == "true" ]] || { info "ENABLE_WEB_VIDEO_SERVER_SERVICE=false -> skip standalone web-video-server (drone.launch starts it)"; return 0; }
  [[ "${INSTALL_ROS}" == "true" ]] || { info "INSTALL_ROS=false -> skip web-video-server service"; return 0; }

  info "Create standalone web-video-server systemd service (http://0.0.0.0:${WEB_VIDEO_PORT})"
  write_ros_env_file

  cat >/etc/systemd/system/web-video-server.service <<EOF
[Unit]
Description=ROS web_video_server
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
EnvironmentFile=/etc/default/ros
User=${PI_USER}
Group=${PI_USER}
WorkingDirectory=/home/${PI_USER}
ExecStart=/bin/bash -lc 'source /opt/ros/\${ROS_DISTRO}/setup.bash && exec rosrun web_video_server web_video_server _port:=${WEB_VIDEO_PORT} _address:=0.0.0.0'
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

  systemctl enable web-video-server.service 2>/dev/null || true
}

setup_butterfly_service() {
  [[ "${ENABLE_BUTTERFLY}" == "true" ]] || { info "ENABLE_BUTTERFLY=false -> skip"; return 0; }
  info "Install butterfly web terminal + systemd service on :${BUTTERFLY_PORT}"

  apt_install python3-pip || true
  my_travis_retry pip3 install --no-cache-dir butterfly || true

  local server="/usr/local/bin/butterfly.server.py"
  if [[ ! -x "${server}" ]]; then
    local alt
    alt="$(command -v butterfly.server.py 2>/dev/null || true)"
    if [[ -n "${alt}" ]]; then
      server="${alt}"
    else
      warn "butterfly.server.py not found in PATH; using default ${server} anyway"
    fi
  fi

  cat >/etc/systemd/system/butterfly.service <<EOF
[Unit]
Description=Butterfly Web Terminal
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${PI_USER}
Group=${PI_USER}
WorkingDirectory=/home/${PI_USER}
ExecStart=/bin/bash -lc 'exec ${server} --unsecure --i-hereby-declare-i-dont-want-any-security-whatsoever --host=0.0.0.0 --port=${BUTTERFLY_PORT}'
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

  systemctl enable butterfly.service 2>/dev/null || true
}

# -----------------------------
# udev rules (FCU / sensors)
# -----------------------------
setup_drone_udev_rules() {
  local udev_dir="${CATKIN_WS}/src/drone/udev"
  if [[ -d "${udev_dir}" ]]; then
    info "Install udev rules from ${udev_dir}"
    mkdir -p /etc/udev/rules.d
    cp -f "${udev_dir}"/*.rules /etc/udev/rules.d/ 2>/dev/null || true
    chmod 0644 /etc/udev/rules.d/*.rules 2>/dev/null || true
    usermod -a -G dialout,video,i2c,spi,gpio "${PI_USER}" 2>/dev/null || true
    udevadm control --reload-rules 2>/dev/null || true
    udevadm trigger 2>/dev/null || true
  else
    info "No udev rules dir found at ${udev_dir} -> skip"
  fi
}

# -----------------------------
# Make `rosrun drone selfcheck` work (package-level entrypoint)
# -----------------------------
setup_drone_entrypoints() {
  local pkg_dir="${CATKIN_WS}/src/drone"
  local src_py="${pkg_dir}/src/selfcheck.py"

  if [[ ! -d "${pkg_dir}" ]]; then
    info "drone package dir not found at ${pkg_dir} -> skip entrypoints"
    return 0
  fi

  if [[ -f "${src_py}" ]]; then
    info "Create entrypoint: ${pkg_dir}/selfcheck (so rosrun finds it)"
    cat >"${pkg_dir}/selfcheck" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "${DIR}/src/selfcheck.py" "$@"
EOF
    chmod +x "${pkg_dir}/selfcheck" || true

    # Optional: convenient symlink
    ln -sf "src/selfcheck.py" "${pkg_dir}/selfcheck.py" || true
    chmod +x "${pkg_dir}/selfcheck.py" 2>/dev/null || true
  else
    warn "selfcheck.py not found at ${src_py} -> entrypoint not created"
  fi
}

# -----------------------------
# Auto-source ROS env for interactive shells (ssh/tty)
# -----------------------------
setup_ros_shell_autosource() {
  info "Setup ROS environment autosource for interactive shells"
  cat >/etc/profile.d/ros-drone.sh <<EOF
# Auto-source ROS + catkin_ws for interactive shells
case "\$-" in
  *i*) ;;
  *) return 0 ;;
esac

# ROS
if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
  . /opt/ros/${ROS_DISTRO}/setup.bash
fi

# Workspace (prefer current user)
if [ -n "\$HOME" ] && [ -f "\$HOME/catkin_ws/devel/setup.bash" ]; then
  . "\$HOME/catkin_ws/devel/setup.bash"
elif [ -f /home/pi/catkin_ws/devel/setup.bash ]; then
  . /home/pi/catkin_ws/devel/setup.bash
fi
EOF
  chmod 0644 /etc/profile.d/ros-drone.sh
}

# -----------------------------
# Writes version in /etc/drone_version file
# -----------------------------
write_image_build_version() {
  local v="${BUILD_VERSION:-}"
  if [[ -z "$v" ]]; then
    v="$(date +%Y%m%d_%H%M%S)"
  fi
  info "Write /etc/drone_version = ${v}"
  echo "$v" > /etc/drone_version
  chmod 0644 /etc/drone_version || true
}

# -----------------------------
# Cleanup
# -----------------------------
cleanup() {
  info "Cleanup apt cache"
  apt-get clean || true
  rm -rf /var/lib/apt/lists/* || true
}

# ============================================================
# MAIN
# ============================================================
require_root
os_info
fs_sanity

apt_configure_archived_repos
apt_fix_sources_buster

apt_update
ensure_locales

add_ros_and_coex_repos_like_clover
apt_update

install_cmake_like_clover

ensure_user_and_hostname
install_extra_packages
install_additional_software

install_ros_if_needed
install_ros_web_tools

install_base_build_deps

install_aruco_pose_like_clover
build_drone_if_needed
additional_ROS_packages

# Make ROS tools available immediately after login (ssh/tty)
setup_ros_shell_autosource

# Install udev rules from the drone package (if present)
setup_drone_udev_rules

# Make rosrun drone selfcheck work (entrypoint inside package root)
setup_drone_entrypoints

enable_console_autologin
setup_mdns_avahi

setup_wifi_ap
install_filebrowser_noauth

setup_static_web_like_clover
setup_roswww_static_update_service

setup_rosbridge_service
setup_drone_launch_service
setup_web_video_server
setup_butterfly_service

# Reload systemd units (we wrote a bunch of them)
systemctl daemon-reload 2>/dev/null || true

info "Generate web pages now (best-effort)"
su - "${PI_USER}" -c "/usr/local/sbin/drone-www-update.sh" || true
systemctl restart nginx 2>/dev/null || true

write_image_build_version

sudo apt update

cleanup
ok "DONE"
