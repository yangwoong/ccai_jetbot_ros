#!/usr/bin/env bash
set -euo pipefail

# Builds (once) and runs the rtabmap_ros sidecar container - see
# docs/navigation_roadmap.md's 2026-07-28 "두 번째 컨테이너" entry for the
# full reasoning: ros-humble-rtabmap-ros is a normal apt package on ROS2
# Humble's official target OS (Ubuntu 22.04/jammy), just not on this
# project's main container (a custom Ubuntu 18.04/bionic backport that
# doesn't carry it - see install_slam_nav2.sh's repeated from-source-build
# OOM failures). Runs as a SEPARATE container alongside the main
# ccai-jetbot one, sharing --network host and ROS_DOMAIN_ID so it can
# subscribe to the D435i topics the main container already publishes - no
# source build here at all, apt only.
#
# UNVERIFIED (2026-07-28): never run on real hardware. Run this on the
# Jetson HOST (same place you'd run host_docker_run.sh), not inside
# "docker exec" into the main container.
#
# Requires the MAIN container (ccai-jetbot) already running with the D435i
# active AND CCAI_ENABLE_VISUAL_ODOM=1 (so align_depth is enabled and
# /camera/camera/aligned_depth_to_color/image_raw exists - see
# ccai_jetbot_patrol/launch/rtabmap_sidecar.launch.py's docstring).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."

CONTAINER_NAME="${RTABMAP_CONTAINER_NAME:-ccai-rtabmap}"
IMAGE_TAG="${RTABMAP_IMAGE_TAG:-ccai-rtabmap:humble-jammy}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
REBUILD_IMAGE="${REBUILD_IMAGE:-0}"

if [ "${REBUILD_IMAGE}" = "1" ] || ! docker image inspect "${IMAGE_TAG}" >/dev/null 2>&1; then
  echo "[ccai] building ${IMAGE_TAG} (apt-get install only, no source compile - should be fast)"
  docker build -t "${IMAGE_TAG}" -f docker/rtabmap_sidecar/Dockerfile .
else
  echo "[ccai] reusing existing image ${IMAGE_TAG} (set REBUILD_IMAGE=1 to rebuild)"
fi

docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

docker run -d \
  --name "${CONTAINER_NAME}" \
  --restart unless-stopped \
  --network host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
  -v "$(pwd)/ccai_jetbot_patrol/launch:/ccai_launch:ro" \
  "${IMAGE_TAG}" \
  bash -c "source /opt/ros/humble/setup.bash && ros2 launch /ccai_launch/rtabmap_sidecar.launch.py"

echo "[ccai] started ${CONTAINER_NAME} (image ${IMAGE_TAG}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
echo "[ccai] logs: docker logs -f ${CONTAINER_NAME}"
echo "[ccai] make sure the main ccai-jetbot container is running with CCAI_ENABLE_VISUAL_ODOM=1"
echo "[ccai] (needed so /camera/camera/aligned_depth_to_color/image_raw actually exists)"
