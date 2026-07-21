#!/usr/bin/env bash
set -euo pipefail

MODEL="${1:-Qwen/Qwen3-VL-32B-Instruct}"
PORT="${PORT:-8000}"

python3 -m vllm.entrypoints.openai.api_server \
  --host 0.0.0.0 \
  --port "${PORT}" \
  --model "${MODEL}" \
  --tensor-parallel-size "${TENSOR_PARALLEL_SIZE:-1}" \
  --trust-remote-code \
  --limit-mm-per-prompt image=1

