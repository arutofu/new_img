#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

apt-get update --allow-releaseinfo-change -o Acquire::Check-Valid-Until=false

# На buster lite обычно нет gnupg2 — ставим gnupg + dirmngr
apt-get install -y --no-install-recommends curl ca-certificates gnupg dirmngr

# Ключ ROS -> keyring
install -d -m 0755 /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg.tmp
mv -f /usr/share/keyrings/ros-archive-keyring.gpg.tmp /usr/share/keyrings/ros-archive-keyring.gpg

# Репо ROS (для buster) с signed-by
cat >/etc/apt/sources.list.d/ros1.list <<'SRC'
deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu buster main
SRC

apt-get update --allow-releaseinfo-change -o Acquire::Check-Valid-Until=false

echo "[rosrepo] added"