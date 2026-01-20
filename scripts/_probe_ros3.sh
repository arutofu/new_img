#!/usr/bin/env bash
set -euo pipefail

P="$(ls -1 /var/lib/apt/lists/packages.ros.org_ros_ubuntu_dists_buster_main_binary-armhf_Packages* | head -n 1 || true)"
echo "Packages file: $P"
if [[ -z "$P" ]]; then
  echo "ERROR: Packages index not found"
  exit 1
fi

echo "First 40 lines:"
sed -n '1,40p' "$P" || true
echo

echo "Count of ROS packages (Package: ros-...):"
grep -c '^Package: ros-' "$P" || true
echo

echo "First 20 ros- packages:"
grep '^Package: ros-' "$P" | head -n 20 || true