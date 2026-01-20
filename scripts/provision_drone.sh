#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

echo "[drone] start"
echo "[drone] os-release:"
cat /etc/os-release | sed -n '1,6p'

# Проверка что workspace на месте
if [[ ! -d /home/pi/catkin_ws/src ]]; then
  echo "ERROR: /home/pi/catkin_ws/src not found (workspace not copied)"
  exit 2
fi

# Проверка ROS (на bookworm почти наверняка нет /opt/ros/noetic)
if [[ ! -d /opt/ros/noetic ]]; then
  echo "ERROR: /opt/ros/noetic not found."
  echo "You are building on bookworm. Clover-like stack (ROS1 Noetic + blocks/web) won't install via apt here."
  echo "Recommended: switch base image to raspios buster armhf (like Clover) or accept source-build path."
  exit 3
fi

echo "[drone] install deps"
apt-get update
apt-get install -y --no-install-recommends \
  python3-rosdep python3-pip build-essential cmake \
  ros-noetic-rosbridge-suite \
  ros-noetic-tf2-web-republisher \
  ros-noetic-web-video-server \
  ros-noetic-mavros ros-noetic-mavros-extras

echo "[drone] rosdep init/update"
rosdep init 2>/dev/null || true
rosdep update || true

echo "[drone] build catkin_ws"
sudo -u pi bash -lc '
set -e
source /opt/ros/noetic/setup.bash
cd /home/pi/catkin_ws
rosdep install -y --from-paths src --ignore-src --rosdistro noetic || true
catkin_make -DCMAKE_BUILD_TYPE=Release
'

echo "[drone] done"
