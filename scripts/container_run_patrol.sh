#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [ -f .env ]; then
  set -a
  # shellcheck disable=SC1091
  source .env
  set +a
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/humble/install/setup.bash ]; then
  source /opt/ros/humble/install/setup.bash
else
  echo "ROS2 Humble setup.bash not found" >&2
  exit 1
fi

if [ ! -f install/setup.bash ]; then
  ./scripts/container_build.sh
fi

source install/setup.bash
exec ros2 launch ccai_jetbot_patrol patrol.launch.py

