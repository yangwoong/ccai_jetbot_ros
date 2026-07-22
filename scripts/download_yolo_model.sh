#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

MODEL_URL="${CCAI_YOLO_MODEL_URL:-https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.onnx}"
MODEL_PATH="${CCAI_YOLO_MODEL_PATH:-data/models/yolov8n.onnx}"

mkdir -p "$(dirname "${MODEL_PATH}")"

if [ -f "${MODEL_PATH}" ]; then
  echo "already present: ${MODEL_PATH}"
  exit 0
fi

echo "downloading ${MODEL_URL} -> ${MODEL_PATH}"
curl -fL --output "${MODEL_PATH}.tmp" "${MODEL_URL}"
mv "${MODEL_PATH}.tmp" "${MODEL_PATH}"
echo "done: ${MODEL_PATH} ($(du -h "${MODEL_PATH}" | cut -f1))"
echo "vision_nav_node picks this up automatically (yolo_model_path parameter); restart the container to load it."
