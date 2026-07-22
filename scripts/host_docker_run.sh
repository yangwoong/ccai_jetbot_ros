#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-ccai-jetbot}"
HOST_WS="${HOST_WS:-/home/roboat/work/ros2_ws}"
REPO_DIR="${REPO_DIR:-ccai_jetbot_ros}"
IMAGE="${IMAGE:-dustynv/ros:humble-desktop-l4t-r32.7.1}"
WORKDIR="/home/workspace/${REPO_DIR}"
CCAI_SAFE_START="${CCAI_SAFE_START:-1}"
DOCKER_ARGS=()

if [ "${CCAI_SAFE_START}" = "1" ]; then
  CCAI_ENABLE_HARDWARE="${CCAI_ENABLE_HARDWARE:-0}"
  CCAI_ENABLE_CAMERA="${CCAI_ENABLE_CAMERA:-0}"
  CCAI_ENABLE_VISION="${CCAI_ENABLE_VISION:-0}"
  CCAI_ENABLE_VLM="${CCAI_ENABLE_VLM:-0}"
else
  CCAI_ENABLE_HARDWARE="${CCAI_ENABLE_HARDWARE:-1}"
  CCAI_ENABLE_CAMERA="${CCAI_ENABLE_CAMERA:-1}"
  CCAI_ENABLE_VISION="${CCAI_ENABLE_VISION:-1}"
  CCAI_ENABLE_VLM="${CCAI_ENABLE_VLM:-1}"
fi

CCAI_ENABLE_PATROL="${CCAI_ENABLE_PATROL:-1}"
CCAI_ENABLE_LLM="${CCAI_ENABLE_LLM:-1}"
CCAI_ENABLE_WEB="${CCAI_ENABLE_WEB:-1}"
CCAI_ENABLE_TELEGRAM="${CCAI_ENABLE_TELEGRAM:-1}"
CCAI_ENABLE_OTA="${CCAI_ENABLE_OTA:-1}"
if [ -z "${CCAI_CAMERA_MODE:-}" ]; then
  if [ "${CCAI_SAFE_START}" = "1" ]; then
    CCAI_CAMERA_MODE="disabled"
  else
    CCAI_CAMERA_MODE="usb"
  fi
fi
DOCKER_PRIVILEGED="${DOCKER_PRIVILEGED:-0}"
DOCKER_RUNTIME_NVIDIA="${DOCKER_RUNTIME_NVIDIA:-}"

if [ "${DOCKER_PRIVILEGED}" = "1" ]; then
  DOCKER_ARGS+=(--privileged)
fi

if [ "${CCAI_ENABLE_CAMERA}" = "1" ] && [ "${CCAI_CAMERA_MODE}" = "csi" ]; then
  DOCKER_RUNTIME_NVIDIA="${DOCKER_RUNTIME_NVIDIA:-1}"
fi
DOCKER_RUNTIME_NVIDIA="${DOCKER_RUNTIME_NVIDIA:-0}"

if [ "${DOCKER_RUNTIME_NVIDIA}" = "1" ]; then
  DOCKER_ARGS+=(--runtime nvidia)
fi

if [ "${CCAI_ENABLE_CAMERA}" = "1" ] && [ "${CCAI_CAMERA_MODE}" = "csi" ] && [ -e /tmp/argus_socket ]; then
  DOCKER_ARGS+=(-v /tmp/argus_socket:/tmp/argus_socket)
fi

if [ "${CCAI_ENABLE_CAMERA}" = "1" ] && [ "${CCAI_CAMERA_MODE}" != "csi" ] && [ -e /dev/video0 ]; then
  DOCKER_ARGS+=(--device /dev/video0)
fi

if [ "${CCAI_ENABLE_HARDWARE}" = "1" ]; then
  for dev in /dev/i2c-*; do
    [ -e "${dev}" ] || continue
    DOCKER_ARGS+=(--device "${dev}")
  done
fi

docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

docker run -d \
  --name "${CONTAINER_NAME}" \
  --network host \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}" \
  -e CCAI_ENABLE_HARDWARE="${CCAI_ENABLE_HARDWARE}" \
  -e CCAI_ENABLE_CAMERA="${CCAI_ENABLE_CAMERA}" \
  -e CCAI_ENABLE_VISION="${CCAI_ENABLE_VISION}" \
  -e CCAI_ENABLE_PATROL="${CCAI_ENABLE_PATROL}" \
  -e CCAI_ENABLE_VLM="${CCAI_ENABLE_VLM}" \
  -e CCAI_ENABLE_LLM="${CCAI_ENABLE_LLM}" \
  -e CCAI_ENABLE_WEB="${CCAI_ENABLE_WEB}" \
  -e CCAI_ENABLE_TELEGRAM="${CCAI_ENABLE_TELEGRAM}" \
  -e CCAI_ENABLE_OTA="${CCAI_ENABLE_OTA}" \
  -e CCAI_CAMERA_MODE="${CCAI_CAMERA_MODE}" \
  -v "${HOST_WS}:/home/workspace" \
  "${DOCKER_ARGS[@]}" \
  -w "${WORKDIR}" \
  "${IMAGE}" \
  bash -c "./scripts/container_run_patrol.sh"

echo "started ${CONTAINER_NAME}"
echo "safe_start=${CCAI_SAFE_START} hardware=${CCAI_ENABLE_HARDWARE} camera=${CCAI_ENABLE_CAMERA} camera_mode=${CCAI_CAMERA_MODE} vision=${CCAI_ENABLE_VISION} vlm=${CCAI_ENABLE_VLM} privileged=${DOCKER_PRIVILEGED} runtime_nvidia=${DOCKER_RUNTIME_NVIDIA}"
echo "logs: docker logs -f ${CONTAINER_NAME}"
echo "web:  http://JETSON_IP:8080"
