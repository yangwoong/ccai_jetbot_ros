#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
elif [ -f /opt/ros/humble/install/setup.bash ]; then
  set +u
  source /opt/ros/humble/install/setup.bash
  set -u
else
  echo "ROS2 Humble setup.bash not found" >&2
  exit 1
fi

INSTALL_OS_DEPS="${INSTALL_OS_DEPS:-0}"

if ! command -v colcon >/dev/null 2>&1; then
  INSTALL_OS_DEPS=1
fi

if [ "${INSTALL_OS_DEPS}" = "1" ]; then
  ./scripts/container_fix_ros_apt_key.sh
  apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-opencv \
    python3-pil \
    python3-smbus \
    v4l-utils \
    i2c-tools \
    ros-humble-cv-bridge
fi

# The container filesystem is recreated from scratch on every host_docker_run.sh
# run (docker rm -f + docker run), so anything pip-installed inside it is wiped
# each time. Point pip's cache at the bind-mounted repo directory so slow builds
# (pycuda in particular, which compiles against nvcc) only pay that cost once -
# later runs reuse the cached wheel instead of recompiling from scratch.
export PIP_CACHE_DIR="${PIP_CACHE_DIR:-$(pwd)/.pip-cache}"
mkdir -p "${PIP_CACHE_DIR}"

python3 - <<'PY' || python3 -m pip install requests pyyaml
import requests
import yaml
PY

# pycuda backs the optional TensorRT YOLO path (ccai_jetbot_patrol/tensorrt_yolo.py).
# Best-effort: vision_nav_node already falls back to OpenCV DNN/HOG if this isn't
# available, so a failed/slow build here should never block the rest of the stack.
python3 - <<'PY' || python3 -m pip install pycuda || echo "pycuda install failed; TensorRT YOLO path will stay disabled" >&2
import pycuda.driver  # noqa: F401
PY

# A failure in ANY discovered package (e.g. an optional/experimental
# dependency like realsense-ros, or one of its own dependencies) used to kill
# this whole script outright via `set -e` above - and since
# container_run_patrol.sh runs this as the container's foreground startup
# step, that took the entire container down with it (camera, patrol, LLM, web
# chat, everything), not just the broken package. Never let a colcon build
# failure be fatal here: log it clearly and continue, so the rest of the
# stack still starts. If ccai_jetbot_patrol itself is what failed to build,
# the launch step right after this script returns will fail loudly and
# specifically on that, which is the actually-actionable signal.
if ! colcon build --symlink-install; then
  echo "[ccai] colcon build had failures (see output above) - continuing anyway so the rest of the stack can still start. A package that actually failed to build will not have picked up its latest code." >&2
fi
