#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-ccai-jetbot}"
docker logs -f "${CONTAINER_NAME}"

