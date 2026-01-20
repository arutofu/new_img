#!/usr/bin/env bash
set -e
. /etc/os-release
echo "OS: $PRETTY_NAME"
apt-get update
apt-cache policy ros-noetic-ros-base | sed -n '1,40p' || true
