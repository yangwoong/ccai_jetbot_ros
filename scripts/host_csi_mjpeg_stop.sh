#!/usr/bin/env bash
set -euo pipefail

PID_FILE="${CCAI_CSI_MJPEG_PID_FILE:-/tmp/ccai_csi_mjpeg.pid}"

if [ ! -f "${PID_FILE}" ]; then
  echo "CSI MJPEG server is not running"
  exit 0
fi

pid="$(cat "${PID_FILE}")"
if kill -0 "${pid}" >/dev/null 2>&1; then
  kill "${pid}"
  echo "stopped CSI MJPEG server, pid=${pid}"
else
  echo "CSI MJPEG server pid is stale, pid=${pid}"
fi
rm -f "${PID_FILE}"
