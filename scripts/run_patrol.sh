#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
if [ -f .env ]; then
  set -a
  # shellcheck disable=SC1091
  source .env
  set +a
fi
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi
ros2 launch ccai_jetbot_patrol patrol.launch.py
