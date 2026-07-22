#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-ccai-jetbot}"
CAMERA_DEVICE="${CCAI_CAMERA_DEVICE:-/dev/video0}"

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "container ${CONTAINER_NAME} is not running" >&2
  exit 1
fi

echo "[host] video devices"
ls -l /dev/video* 2>/dev/null || true

echo
echo "[container] OpenCV probe: ${CAMERA_DEVICE}"
docker exec -i \
  -e CAMERA_DEVICE="${CAMERA_DEVICE}" \
  "${CONTAINER_NAME}" \
  python3 - <<'PY'
import os
import time

import cv2

device = os.environ.get("CAMERA_DEVICE", "/dev/video0")
print("device=" + device)
cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
if not cap.isOpened():
    print("opened=false")
    raise SystemExit(2)

for key, value in [
    (cv2.CAP_PROP_FRAME_WIDTH, 640),
    (cv2.CAP_PROP_FRAME_HEIGHT, 480),
    (cv2.CAP_PROP_FPS, 5),
    (cv2.CAP_PROP_BUFFERSIZE, 1),
]:
    cap.set(key, value)

for index in range(10):
    ok, frame = cap.read()
    if ok and frame is not None:
        print("opened=true")
        print("frame_shape={0}".format(frame.shape))
        print("frame_stddev={0:.2f}".format(float(frame.std())))
        raise SystemExit(0)
    time.sleep(0.1)

print("opened=true")
print("read=false")
raise SystemExit(3)
PY
