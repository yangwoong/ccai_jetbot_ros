#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-ccai-jetbot}"
HOST_WS="${HOST_WS:-/home/roboat/work/ros2_ws}"
REPO_DIR="${REPO_DIR:-ccai_jetbot_ros}"
HOST_REPO="${HOST_WS}/${REPO_DIR}"
WORKDIR="/home/workspace/${REPO_DIR}"

if [ ! -d "${HOST_REPO}/.git" ]; then
  echo "git repository not found: ${HOST_REPO}" >&2
  exit 1
fi

echo "[host] updating git repository"
git -C "${HOST_REPO}" fetch origin
git -C "${HOST_REPO}" pull --ff-only

if ! docker ps -a --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "container ${CONTAINER_NAME} not found. Run scripts/host_docker_run.sh first." >&2
  exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  docker start "${CONTAINER_NAME}" >/dev/null
fi

echo "[container] rebuilding workspace"
docker exec "${CONTAINER_NAME}" bash -c "cd '${WORKDIR}' && ./scripts/container_build.sh"

echo "[host] restarting container"
docker restart "${CONTAINER_NAME}" >/dev/null

echo "updated and restarted ${CONTAINER_NAME}"
echo "logs: docker logs -f ${CONTAINER_NAME}"
