#!/usr/bin/env bash
set -euo pipefail

# Installs RTAB-Map (RGB-D SLAM: visual odometry + occupancy-grid mapping
# from the D435i alone - no wheel encoders needed) and Nav2 (costmap +
# planner + controller, for actually navigating to a point on that map).
#
# This is a genuinely large, experimental addition on top of the reactive
# depth_nav_node patrol that is this project's stable default - see
# docs/navigation_roadmap.md for how the two relate. Run this INSIDE the
# ccai-jetbot container (same place install_realsense_d435i.sh runs):
#   docker exec -it ccai-jetbot bash
#   scripts/install_slam_nav2.sh
#
# PACKAGE AVAILABILITY IS UNVERIFIED FROM THIS ENVIRONMENT. This image
# backports ROS2 Humble onto bionic (Ubuntu 18.04) via a custom apt repo
# that has already turned out to be missing ordinary packages
# (ros-humble-xacro, ros-humble-diagnostic-updater - see
# scripts/install_realsense_d435i.sh). rtabmap_ros and Nav2 are each dozens
# of packages with their own dependency trees; unlike the small gaps found
# so far, source-building either stack blind is not realistic to script.
# This script tries apt first and reports exactly what's missing - if
# packages ARE missing, that list is the next thing to act on (case by
# case), not something this script attempts to paper over automatically.

cd "$(dirname "$0")/.."

LOG_FILE="${LOG_FILE:-$(pwd)/install_slam_nav2.log}"
exec > >(tee -a "${LOG_FILE}") 2>&1
echo "[ccai] logging to ${LOG_FILE}"

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

./scripts/container_fix_ros_apt_key.sh

PACKAGES=(
  ros-humble-rtabmap-ros
  ros-humble-navigation2
  ros-humble-nav2-bringup
  ros-humble-robot-localization
)

echo "[ccai] Attempting apt install of: ${PACKAGES[*]}"
MISSING=()
for pkg in "${PACKAGES[@]}"; do
  if apt-get install -y "${pkg}" 2>&1 | tee /tmp/ccai_apt_pkg.log; then
    :
  fi
  if ! dpkg -s "${pkg}" >/dev/null 2>&1; then
    MISSING+=("${pkg}")
  fi
done

if [ "${#MISSING[@]}" -gt 0 ]; then
  cat <<EOF

[ccai] NOT installed via apt: ${MISSING[*]}
This image's ROS apt repo doesn't carry these (same situation as
xacro/diagnostic_updater earlier). Source-building rtabmap_ros/Nav2 blind is
not something this script attempts - the dependency trees are too large to
guess correctly upfront. Send this log back and each missing package will be
resolved the same way xacro/diagnostic_updater were (clone the specific repo,
skip unrelated sibling packages that don't compile against this image's
older rclcpp, etc.) - just one at a time based on the actual error, not
speculatively.
EOF
  exit 1
fi

echo "[ccai] All packages installed via apt successfully."
echo "[ccai] Note: this project's navigation stack has since moved to a custom"
echo "[ccai] lightweight controller (visual_odom_node + patrol_node's own"
echo "[ccai] point-to-point/coverage logic) instead of rtabmap/Nav2 - see"
echo "[ccai] docs/navigation_roadmap.md. These packages being installed here"
echo "[ccai] doesn't wire anything up on its own."
