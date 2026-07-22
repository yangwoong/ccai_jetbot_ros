#!/usr/bin/env bash
set -euo pipefail

HOST="${CCAI_CSI_MJPEG_HOST:-127.0.0.1}"
PORT="${CCAI_CSI_MJPEG_PORT:-8090}"
PID_FILE="${CCAI_CSI_MJPEG_PID_FILE:-/tmp/ccai_csi_mjpeg.pid}"
LOG_FILE="${CCAI_CSI_MJPEG_LOG_FILE:-/tmp/ccai_csi_mjpeg.log}"

JETBOT_REPO_PATH="${JETBOT_REPO_PATH:-}"
JETBOT_PYTHONPATHS=()
for path in \
  "${JETBOT_REPO_PATH}" \
  "${HOME:-}/jetbot" \
  "/home/roboat/jetbot" \
  "/home/roboat/work/jetbot" \
  "/home/roboat/work/ros2_ws/jetbot"; do
  [ -n "${path}" ] || continue
  [ -d "${path}/jetbot" ] || continue
  JETBOT_PYTHONPATHS+=("${path}")
done

if [ "${#JETBOT_PYTHONPATHS[@]}" -gt 0 ]; then
  JETBOT_PATH_JOINED="$(IFS=:; echo "${JETBOT_PYTHONPATHS[*]}")"
  export PYTHONPATH="${JETBOT_PATH_JOINED}${PYTHONPATH:+:${PYTHONPATH}}"
else
  JETBOT_PATH_JOINED=""
fi

if [ -f "${PID_FILE}" ] && kill -0 "$(cat "${PID_FILE}")" >/dev/null 2>&1; then
  echo "CSI MJPEG server already running, pid=$(cat "${PID_FILE}")"
  echo "url=http://${HOST}:${PORT}/stream.mjpg"
  exit 0
fi

nohup ./scripts/host_csi_mjpeg_server.py \
  --host "${HOST}" \
  --port "${PORT}" \
  --sensor-mode "${CCAI_CSI_SENSOR_MODE:-3}" \
  --capture-width "${CCAI_CSI_CAPTURE_WIDTH:-816}" \
  --capture-height "${CCAI_CSI_CAPTURE_HEIGHT:-616}" \
  --fps "${CCAI_CSI_FPS:-30}" \
  --width "${CCAI_CAMERA_WIDTH:-224}" \
  --height "${CCAI_CAMERA_HEIGHT:-224}" \
  --flip-method "${CCAI_CSI_FLIP_METHOD:-0}" \
  --jpeg-quality "${CCAI_CAMERA_JPEG_QUALITY:-45}" \
  --backend "${CCAI_CSI_HOST_BACKEND:-auto}" \
  >"${LOG_FILE}" 2>&1 &

echo "$!" >"${PID_FILE}"
echo "started CSI MJPEG server, pid=$(cat "${PID_FILE}")"
echo "url=http://${HOST}:${PORT}/stream.mjpg"
echo "log=${LOG_FILE}"
if [ -n "${JETBOT_PATH_JOINED}" ]; then
  echo "jetbot_pythonpath=${JETBOT_PATH_JOINED}"
else
  echo "jetbot_pythonpath=not found; set JETBOT_REPO_PATH=/path/to/jetbot if using CCAI_CSI_HOST_BACKEND=jetbot"
fi
