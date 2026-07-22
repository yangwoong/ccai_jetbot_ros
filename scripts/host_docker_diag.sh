#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-ccai-jetbot}"

echo "[host] container"
docker ps --filter "name=${CONTAINER_NAME}" --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
CONTAINER_RUNNING=0
if docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  CONTAINER_RUNNING=1
fi

echo
echo "[host] camera services and previous boot hints"
if command -v systemctl >/dev/null 2>&1; then
  systemctl is-active nvargus-daemon 2>/dev/null | sed 's/^/nvargus-daemon: /' || true
fi
if command -v journalctl >/dev/null 2>&1; then
  echo "previous boot kernel/camera faults:"
  journalctl -k -b -1 --no-pager -n 120 2>/dev/null | grep -Ei "panic|oops|watchdog|brown|thermal|voltage|argus|nvargus|tegra|vi|csi|imx|camera" || true
  echo "current boot nvargus logs:"
  journalctl -u nvargus-daemon -b --no-pager -n 80 2>/dev/null || true
fi
echo "host video devices:"
ls -l /dev/video* 2>/dev/null || true
if command -v v4l2-ctl >/dev/null 2>&1; then
  for dev in /dev/video*; do
    [ -e "${dev}" ] || continue
    echo "host v4l2 formats: ${dev}"
    v4l2-ctl --device="${dev}" --list-formats-ext 2>/dev/null || true
  done
fi

if [ "${CONTAINER_RUNNING}" != "1" ]; then
  echo
  echo "[container] skipped because ${CONTAINER_NAME} is not running"
  exit 0
fi

echo
echo "[container] environment and devices"
docker exec "${CONTAINER_NAME}" bash -c '
set +e
echo "ROS_LOCALHOST_ONLY(exec env)=${ROS_LOCALHOST_ONLY:-unset}"
echo "CCAI_ENABLE_HARDWARE=${CCAI_ENABLE_HARDWARE:-unset}"
echo "CCAI_ENABLE_CAMERA=${CCAI_ENABLE_CAMERA:-unset}"
echo "CCAI_CAMERA_MODE=${CCAI_CAMERA_MODE:-unset}"
echo "CCAI_CAMERA_DEVICE=${CCAI_CAMERA_DEVICE:-unset}"
echo "CCAI_CAMERA_URL=${CCAI_CAMERA_URL:-unset}"
echo "CCAI_CAMERA_RETRY_LIMIT=${CCAI_CAMERA_RETRY_LIMIT:-unset}"
echo "CCAI_CSI_SENSOR_ID=${CCAI_CSI_SENSOR_ID:-unset}"
echo "CCAI_CSI_SENSOR_MODE=${CCAI_CSI_SENSOR_MODE:-unset}"
echo "CCAI_CSI_CAPTURE_WIDTH=${CCAI_CSI_CAPTURE_WIDTH:-unset}"
echo "CCAI_CSI_CAPTURE_HEIGHT=${CCAI_CSI_CAPTURE_HEIGHT:-unset}"
echo "CCAI_CSI_FPS=${CCAI_CSI_FPS:-unset}"
echo "CCAI_CSI_FLIP_METHOD=${CCAI_CSI_FLIP_METHOD:-unset}"
echo "CCAI_ENABLE_VISION=${CCAI_ENABLE_VISION:-unset}"
echo "CCAI_ENABLE_VLM=${CCAI_ENABLE_VLM:-unset}"
echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
echo "Python: $(python3 --version 2>&1)"
echo "I2C devices:"
ls -l /dev/i2c* 2>/dev/null || true
echo "Video devices:"
ls -l /dev/video* 2>/dev/null || true
echo "Argus socket:"
ls -ld /tmp/argus_socket 2>/dev/null || echo "/tmp/argus_socket not mounted"
echo "Last invalid camera frame:"
ls -l /tmp/ccai_camera_last_invalid.jpg 2>/dev/null || echo "no invalid-frame capture"
if command -v v4l2-ctl >/dev/null 2>&1; then
  echo "v4l2 formats:"
  for dev in /dev/video*; do
    [ -e "${dev}" ] || continue
    echo "${dev}"
    v4l2-ctl --device="${dev}" --list-formats-ext 2>/dev/null || true
  done
else
  echo "v4l2-ctl not installed"
fi
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
  for dev in /dev/i2c-*; do
    [ -e "${dev}" ] || continue
    bus="${dev##*-}"
    echo "i2cdetect -y ${bus}"
    i2cdetect -y "${bus}" || true
  done
else
  echo "i2cdetect not installed"
fi
'

echo
echo "[host] recent relevant logs"
docker logs --since 3m "${CONTAINER_NAME}" 2>&1 | grep -E "pca9685|jetbot|OLED|raw OLED|camera|vision|ddsi|unavailable|failed|invalid|ROS_LOCALHOST" || true

echo
echo "[host] web checks"
if command -v curl >/dev/null 2>&1; then
  curl -fsS http://127.0.0.1:8080/api/status >/tmp/ccai_jetbot_status.json && echo "web status: ok" || echo "web status: failed"
  curl -fsS http://127.0.0.1:8080/api/camera.jpg --output /tmp/ccai_jetbot_camera.jpg && echo "camera jpg: /tmp/ccai_jetbot_camera.jpg" || echo "camera jpg: failed"
else
  echo "curl not installed on host"
fi
