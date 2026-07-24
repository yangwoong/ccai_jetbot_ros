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

# D435i/librealsense2 support (see scripts/install_realsense_d435i.sh) - no-op
# if that script was never run. Needed for realsense2_camera_node to find
# librealsense2.so at runtime, since it's installed to a bind-mounted prefix
# rather than a system path (so it survives container recreation).
if [ -f deps/librealsense/librealsense_env.sh ]; then
  set +u
  source deps/librealsense/librealsense_env.sh
  set -u
fi

exec ros2 launch ccai_jetbot_patrol patrol.launch.py
