#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

apt-get update --allow-releaseinfo-change -o Acquire::Check-Valid-Until=false >/dev/null

echo "ARCH: $(dpkg --print-architecture)"
echo "OS: $(. /etc/os-release; echo "$PRETTY_NAME")"
echo

echo "=== does apt see any ros-melodic packages? (top 20) ==="
apt-cache search '^ros-melodic-' | head -n 20 || true
echo

echo "=== policy ros-melodic-ros-base ==="
apt-cache policy ros-melodic-ros-base || true
echo

echo "=== show ros-melodic-ros-base ==="
apt-cache show ros-melodic-ros-base 2>/dev/null | sed -n '1,80p' || true
echo

echo "=== list files for ROS Packages indices (debug) ==="
ls -1 /var/lib/apt/lists | grep -E 'packages\.ros\.org|ros' | head -n 30 || true