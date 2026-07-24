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
# See ccai_jetbot_patrol/config/cyclonedds.xml for background - this alone
# turned out NOT to be enough (same "Failed to find a free participant
# index for domain 0" recurred with it in place, so either it isn't being
# picked up or domain 0's state is broken in some other way). Belt and
# suspenders: also move off domain 0 entirely onto a domain that has none of
# whatever stale/exhausted state accumulated there - each ROS_DOMAIN_ID maps
# to its own port range, so this is a genuinely clean slate regardless of
# what's actually wrong with domain 0.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$(pwd)/ccai_jetbot_patrol/config/cyclonedds.xml}"
echo "[ccai] ROS_DOMAIN_ID=${ROS_DOMAIN_ID} CYCLONEDDS_URI=${CYCLONEDDS_URI}"
if [ ! -f "${CYCLONEDDS_URI#file://}" ]; then
  echo "[ccai] WARNING: CYCLONEDDS_URI points at a file that doesn't exist: ${CYCLONEDDS_URI#file://}" >&2
fi
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
if [ -f install/setup.bash ]; then
  set +u
  source install/setup.bash
  set -u
fi
# D435i/librealsense2 support (see scripts/install_realsense_d435i.sh) - no-op
# if that script was never run. Needed for realsense2_camera_node to find
# librealsense2.so at runtime, since it's installed to a bind-mounted prefix
# rather than a system path (so it survives container recreation).
if [ -f deps/librealsense/librealsense_env.sh ]; then
  set +u
  source deps/librealsense/librealsense_env.sh
  set -u
fi
ros2 launch ccai_jetbot_patrol patrol.launch.py
