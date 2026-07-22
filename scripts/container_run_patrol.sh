#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [ -f .env ]; then
  set -a
  # shellcheck disable=SC1091
  source .env
  set +a
fi

export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
elif [ -f /opt/ros/humble/install/setup.bash ]; then
  set +u
  source /opt/ros/humble/install/setup.bash
  set -u
else
  echo "ROS2 Humble setup.bash not found" >&2
  exit 1
fi

FORCE_BUILD_ON_RUN="${FORCE_BUILD_ON_RUN:-1}"

if [ "${FORCE_BUILD_ON_RUN}" = "1" ] || [ ! -f install/setup.bash ]; then
  ./scripts/container_build.sh
fi

set +u
source install/setup.bash
set -u
exec ros2 launch ccai_jetbot_patrol patrol.launch.py
