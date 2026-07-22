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

run_trtexec() {
  trtexec \
    --onnx="${MODEL_PATH}" \
    --saveEngine="${ENGINE_PATH}" \
    --fp16 \
    --skipInference=false
}

echo "building ${ENGINE_PATH} from ${MODEL_PATH} with trtexec (vision_nav_node loads this engine directly if present)"

if command -v trtexec >/dev/null 2>&1; then
  run_trtexec
elif docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "trtexec not found on host; running inside container ${CONTAINER_NAME}"
  docker exec -w "/home/workspace/$(basename "$(pwd)")" "${CONTAINER_NAME}" bash -c "
    set -e
    trtexec --onnx='${MODEL_PATH}' --saveEngine='${ENGINE_PATH}' --fp16 --skipInference=false
  "
else
  echo "trtexec not found on host and container ${CONTAINER_NAME} is not running" >&2
  echo "trtexec ships with the L4T/JetPack TensorRT install; it should be on PATH on the Jetson host or inside the dustynv ROS container" >&2
  exit 1
fi

echo
echo "engine written to: ${ENGINE_PATH}"
echo "restart the container (docker restart ccai-jetbot) so vision_nav_node picks it up."
echo "engines are hardware/TensorRT-version specific - do not copy this file to a different device, and do not commit it to git."
