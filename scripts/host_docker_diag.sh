#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-ccai-jetbot}"

echo "[host] container"
docker ps --filter "name=${CONTAINER_NAME}" --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'

echo
echo "[container] environment and devices"
docker exec "${CONTAINER_NAME}" bash -c '
set +e
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}"
echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
echo "Python: $(python3 --version 2>&1)"
echo "I2C devices:"
ls -l /dev/i2c* 2>/dev/null || true
echo "Video devices:"
ls -l /dev/video* 2>/dev/null || true
'

echo
echo "[container] python modules"
docker exec "${CONTAINER_NAME}" bash -c '
set +e
python3 - <<'"'"'PY'"'"'
modules = ["smbus", "PIL", "cv2", "jetbot", "Adafruit_SSD1306"]
for module in modules:
    try:
        __import__(module)
        print(module + ": ok")
    except Exception as exc:
        print(module + ": missing/unavailable: " + str(exc))
PY
'

echo
echo "[container] i2c scan"
docker exec "${CONTAINER_NAME}" bash -c '
set +e
if command -v i2cdetect >/dev/null 2>&1; then
  for bus in 0 1; do
    if [ -e "/dev/i2c-${bus}" ]; then
      echo "i2cdetect -y ${bus}"
      i2cdetect -y "${bus}" || true
    fi
  done
else
  echo "i2cdetect not installed"
fi
'

echo
echo "[host] recent relevant logs"
docker logs --since 3m "${CONTAINER_NAME}" 2>&1 | grep -E "pca9685|jetbot|OLED|raw OLED|camera|ddsi|unavailable|failed|ROS_LOCALHOST" || true

echo
echo "[host] web checks"
if command -v curl >/dev/null 2>&1; then
  curl -fsS http://127.0.0.1:8080/api/status >/tmp/ccai_jetbot_status.json && echo "web status: ok" || echo "web status: failed"
  curl -fsS http://127.0.0.1:8080/api/camera.jpg --output /tmp/ccai_jetbot_camera.jpg && echo "camera jpg: /tmp/ccai_jetbot_camera.jpg" || echo "camera jpg: failed"
else
  echo "curl not installed on host"
fi

