#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

MODEL_PATH="${CCAI_YOLO_MODEL_PATH:-data/models/yolov8n.onnx}"
ENGINE_PATH="${CCAI_YOLO_ENGINE_PATH:-data/models/yolov8n.engine}"
CONTAINER_NAME="${CONTAINER_NAME:-ccai-jetbot}"

if [ ! -f "${MODEL_PATH}" ]; then
  echo "onnx model not found: ${MODEL_PATH}" >&2
  echo "run ./scripts/download_yolo_model.sh (on a Python 3.8+ machine) and copy it here first" >&2
  exit 1
fi

# JetPack installs trtexec here but usually doesn't put it on PATH.
FALLBACK_TRTEXEC="/usr/src/tensorrt/bin/trtexec"

find_trtexec() {
  if command -v trtexec >/dev/null 2>&1; then
    command -v trtexec
  elif [ -x "${FALLBACK_TRTEXEC}" ]; then
    echo "${FALLBACK_TRTEXEC}"
  fi
}

echo "building ${ENGINE_PATH} from ${MODEL_PATH} with trtexec (vision_nav_node loads this engine directly if present)"

HOST_TRTEXEC="$(find_trtexec || true)"

if [ -n "${HOST_TRTEXEC}" ]; then
  "${HOST_TRTEXEC}" \
    --onnx="${MODEL_PATH}" \
    --saveEngine="${ENGINE_PATH}" \
    --fp16 \
    --skipInference=false
elif docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "trtexec not found on host PATH; running inside container ${CONTAINER_NAME}"
  docker exec -w "/home/workspace/$(basename "$(pwd)")" "${CONTAINER_NAME}" bash -c "
    set -e
    TRTEXEC=\$(command -v trtexec || echo '${FALLBACK_TRTEXEC}')
    \"\${TRTEXEC}\" --onnx='${MODEL_PATH}' --saveEngine='${ENGINE_PATH}' --fp16 --skipInference=false
  "
else
  echo "trtexec not found on host (checked PATH and ${FALLBACK_TRTEXEC}) and container ${CONTAINER_NAME} is not running" >&2
  echo "trtexec ships with the L4T/JetPack TensorRT install (dpkg -l | grep tensorrt to confirm it's installed)" >&2
  exit 1
fi

echo
echo "engine written to: ${ENGINE_PATH}"
echo "restart the container (docker restart ccai-jetbot) so vision_nav_node picks it up."
echo "engines are hardware/TensorRT-version specific - do not copy this file to a different device, and do not commit it to git."
