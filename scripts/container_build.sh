#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/humble/install/setup.bash ]; then
  source /opt/ros/humble/install/setup.bash
else
  echo "ROS2 Humble setup.bash not found" >&2
  exit 1
fi

if ! command -v colcon >/dev/null 2>&1; then
  apt-get update
  apt-get install -y python3-colcon-common-extensions python3-pip
fi

python3 -m pip install fastapi uvicorn requests pyyaml
colcon build --symlink-install

