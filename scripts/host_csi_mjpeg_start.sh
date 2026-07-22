#!/usr/bin/env bash
set -euo pipefail

HOST="${CCAI_CSI_MJPEG_HOST:-127.0.0.1}"
PORT="${CCAI_CSI_MJPEG_PORT:-8090}"
PID_FILE="${CCAI_CSI_MJPEG_PID_FILE:-/tmp/ccai_csi_mjpeg.pid}"
LOG_FILE="${CCAI_CSI_MJPEG_LOG_FILE:-/tmp/ccai_csi_mjpeg.log}"

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
  --width "${CCAI_CAMERA_WIDTH:-320}" \
  --height "${CCAI_CAMERA_HEIGHT:-240}" \
  --flip-method "${CCAI_CSI_FLIP_METHOD:-0}" \
  --jpeg-quality "${CCAI_CAMERA_JPEG_QUALITY:-45}" \
  --backend "${CCAI_CSI_HOST_BACKEND:-auto}" \
  >"${LOG_FILE}" 2>&1 &

echo "$!" >"${PID_FILE}"
echo "started CSI MJPEG server, pid=$(cat "${PID_FILE}")"
echo "url=http://${HOST}:${PORT}/stream.mjpg"
echo "log=${LOG_FILE}"
