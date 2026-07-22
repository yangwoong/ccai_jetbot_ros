#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-ccai-jetbot}"
CSI_SENSOR_ID="${CCAI_CSI_SENSOR_ID:-0}"
CSI_SENSOR_MODE="${CCAI_CSI_SENSOR_MODE:-3}"
CSI_CAPTURE_WIDTH="${CCAI_CSI_CAPTURE_WIDTH:-816}"
CSI_CAPTURE_HEIGHT="${CCAI_CSI_CAPTURE_HEIGHT:-616}"
CSI_FPS="${CCAI_CSI_FPS:-30}"
CSI_OUTPUT_WIDTH="${CCAI_CAMERA_WIDTH:-320}"
CSI_OUTPUT_HEIGHT="${CCAI_CAMERA_HEIGHT:-240}"
CSI_FLIP_METHOD="${CCAI_CSI_FLIP_METHOD:-0}"

PIPELINE="nvarguscamerasrc sensor-mode=${CSI_SENSOR_MODE} ! video/x-raw(memory:NVMM), width=${CSI_CAPTURE_WIDTH}, height=${CSI_CAPTURE_HEIGHT}, format=(string)NV12, framerate=(fraction)${CSI_FPS}/1 ! nvvidconv flip-method=${CSI_FLIP_METHOD} ! video/x-raw, width=(int)${CSI_OUTPUT_WIDTH}, height=(int)${CSI_OUTPUT_HEIGHT}, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"
PIPELINE_WITH_ID="nvarguscamerasrc sensor-id=${CSI_SENSOR_ID} ! video/x-raw(memory:NVMM), width=${CSI_CAPTURE_WIDTH}, height=${CSI_CAPTURE_HEIGHT}, format=(string)NV12, framerate=(fraction)${CSI_FPS}/1 ! nvvidconv flip-method=${CSI_FLIP_METHOD} ! video/x-raw, width=(int)${CSI_OUTPUT_WIDTH}, height=(int)${CSI_OUTPUT_HEIGHT}, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"

echo "[host] argus and video devices"
systemctl is-active nvargus-daemon 2>/dev/null | sed 's/^/nvargus-daemon: /' || true
ls -ld /tmp/argus_socket 2>/dev/null || true
ls -l /dev/video* 2>/dev/null || true

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "container ${CONTAINER_NAME} is not running" >&2
  echo "start probe container with:"
  echo "  CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 CCAI_CAMERA_MODE=disabled DOCKER_RUNTIME_NVIDIA=1 ./scripts/host_docker_run.sh" >&2
  exit 1
fi

if ! docker exec "${CONTAINER_NAME}" test -S /tmp/argus_socket; then
  echo "container ${CONTAINER_NAME} cannot see /tmp/argus_socket" >&2
  echo "restart it for CSI probing with:"
  echo "  CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 CCAI_CAMERA_MODE=disabled DOCKER_RUNTIME_NVIDIA=1 ./scripts/host_docker_run.sh" >&2
  exit 1
fi

echo
echo "[container] OpenCV CSI probe: JetBot pipeline"
docker exec -i \
  -e PIPELINE="${PIPELINE}" \
  -e PIPELINE_WITH_ID="${PIPELINE_WITH_ID}" \
  "${CONTAINER_NAME}" \
  python3 - <<'PY'
import os
import time

import cv2

for name in ["PIPELINE", "PIPELINE_WITH_ID"]:
    pipeline = os.environ[name]
    print("pipeline_name=" + name)
    print(pipeline)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("opened=false")
        continue
    for _ in range(20):
        ok, frame = cap.read()
        if ok and frame is not None:
            print("opened=true")
            print("frame_shape={0}".format(frame.shape))
            print("frame_stddev={0:.2f}".format(float(frame.std())))
            cap.release()
            raise SystemExit(0)
        time.sleep(0.1)
    cap.release()
    print("opened=true")
    print("read=false")
raise SystemExit(2)
PY
