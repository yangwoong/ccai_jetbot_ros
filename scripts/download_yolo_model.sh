#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

MODEL_NAME="${CCAI_YOLO_MODEL_NAME:-yolov8n}"
MODEL_PATH="${CCAI_YOLO_MODEL_PATH:-data/models/${MODEL_NAME}.onnx}"
IMG_SIZE="${CCAI_YOLO_IMG_SIZE:-320}"

mkdir -p "$(dirname "${MODEL_PATH}")"

if [ -f "${MODEL_PATH}" ]; then
  echo "already present: ${MODEL_PATH}"
  exit 0
fi

# A pre-exported .onnx isn't hosted by Ultralytics; only .pt weights are. If the
# caller already has their own onnx mirror, honor that first.
if [ -n "${CCAI_YOLO_MODEL_URL:-}" ]; then
  echo "downloading ${CCAI_YOLO_MODEL_URL} -> ${MODEL_PATH}"
  curl -fL --output "${MODEL_PATH}.tmp" "${CCAI_YOLO_MODEL_URL}"
  mv "${MODEL_PATH}.tmp" "${MODEL_PATH}"
  echo "done: ${MODEL_PATH} ($(du -h "${MODEL_PATH}" | cut -f1))"
  exit 0
fi

PYTHON_BIN="${PYTHON_BIN:-python3}"
PY_OK=0
if command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
  PY_MAJOR_MINOR="$("${PYTHON_BIN}" -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null || echo "0.0")"
  case "${PY_MAJOR_MINOR}" in
    3.8|3.9|3.1*) PY_OK=1 ;;
  esac
fi

if [ "${PY_OK}" != "1" ]; then
  cat >&2 <<EOF
This machine's ${PYTHON_BIN} ($(${PYTHON_BIN} --version 2>&1 || echo "not found")) is too old for the
'ultralytics' package (needs Python >= 3.8) used to export the ONNX model.

Run this same script on a normal PC (your Mac, a Linux box, or the H200 server)
where 'pip install ultralytics' works, then copy the resulting file over:

  ./scripts/download_yolo_model.sh
  scp ${MODEL_PATH} roboat@JETSON_IP:$(pwd)/${MODEL_PATH}

Or, if you already have an .onnx hosted somewhere, point directly at it:

  CCAI_YOLO_MODEL_URL=https://your-mirror/${MODEL_NAME}.onnx ./scripts/download_yolo_model.sh
EOF
  exit 1
fi

if ! "${PYTHON_BIN}" -c "import ultralytics" >/dev/null 2>&1; then
  echo "installing ultralytics (one-time, needs internet)"
  "${PYTHON_BIN}" -m pip install --user ultralytics
fi

WORKDIR="$(mktemp -d)"
trap 'rm -rf "${WORKDIR}"' EXIT

echo "exporting ${MODEL_NAME}.onnx (imgsz=${IMG_SIZE}) via ultralytics"
(
  cd "${WORKDIR}"
  "${PYTHON_BIN}" - "${MODEL_NAME}" "${IMG_SIZE}" <<'PY'
import sys
from ultralytics import YOLO

model_name, img_size = sys.argv[1], int(sys.argv[2])
YOLO(f"{model_name}.pt").export(format="onnx", imgsz=img_size)
PY
)
mv "${WORKDIR}/${MODEL_NAME}.onnx" "${MODEL_PATH}"
echo "done: ${MODEL_PATH} ($(du -h "${MODEL_PATH}" | cut -f1))"
echo "vision_nav_node picks this up automatically (yolo_model_path parameter); restart the container to load it."
