#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-ccai-jetbot}"
HOST_WS="${HOST_WS:-/home/roboat/work/ros2_ws}"
REPO_DIR="${REPO_DIR:-ccai_jetbot_ros}"
IMAGE="${IMAGE:-dustynv/ros:humble-desktop-l4t-r32.7.1}"
WORKDIR="/home/workspace/${REPO_DIR}"
DOCKER_ARGS=()

if [ -e /tmp/argus_socket ]; then
  DOCKER_ARGS+=(-v /tmp/argus_socket:/tmp/argus_socket)
fi

docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

docker run -d \
  --name "${CONTAINER_NAME}" \
  --network host \
  --privileged \
  -v "${HOST_WS}:/home/workspace" \
  "${DOCKER_ARGS[@]}" \
  -w "${WORKDIR}" \
  "${IMAGE}" \
  bash -c "./scripts/container_run_patrol.sh"

echo "started ${CONTAINER_NAME}"
echo "logs: docker logs -f ${CONTAINER_NAME}"
echo "web:  http://JETSON_IP:8080"
