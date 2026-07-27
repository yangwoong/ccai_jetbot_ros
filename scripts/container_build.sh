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

# D435i/librealsense2 support (see scripts/install_realsense_d435i.sh) - no-op
# if that script was never run. Needed here too (not just at launch time) so
# find_package(realsense2) succeeds if colcon ever needs to (re)configure
# realsense2_camera - it's installed to a bind-mounted prefix rather than a
# system path, so it won't be on the default search paths.
if [ -f deps/librealsense/librealsense_env.sh ]; then
  set +u
  source deps/librealsense/librealsense_env.sh
  set -u
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
# available. In practice a failing build here was NOT harmless: no prebuilt
# wheel exists for this platform, so pip falls back to a from-source build
# that pulls in numpy as a build dependency, which failed here with
# "xlocale.h: No such file or directory" (older numpy vs. newer glibc) - pip
# then retried several numpy versions in a row before giving up, taking many
# minutes, and since nothing here remembered the failure, this repeated in
# full on *every single container start* (FORCE_BUILD_ON_RUN=1 by default),
# delaying the whole stack - including web_chat_node - by that much every
# time. Now: capped with `timeout` so one attempt can't run indefinitely, and
# a failure is remembered in a marker file on the bind-mounted side so it's
# not retried on every future start. Delete the marker (or fix the
# xlocale.h issue - a symlink to locale.h is the usual workaround) to retry.
# v2 (2026-07-27): "TensorRT engine load failed (No module named 'pycuda')"
# kept recurring on real hardware even after the xlocale.h workaround above
# was added - almost certainly because a v1 marker from BEFORE that fix
# shipped was still sitting in the bind-mounted pip cache, permanently
# skipping every future attempt regardless of whether the underlying build
# would now succeed. Renamed the marker to `_v2` so this fix (and the apt-first
# attempt below) actually get one fresh try on real hardware instead of being
# silently blocked by a stale marker from an earlier, since-fixed failure.
# Also: try the platform's prebuilt apt package first - it's a real binary,
# so it sidesteps the whole from-source-numpy-build class of failure
# entirely - and only fall back to the slow pip source build if apt doesn't
# have it.
PYCUDA_FAIL_MARKER="${PIP_CACHE_DIR}/pycuda_build_failed_v2"
if python3 - <<'PY' 2>/dev/null
import pycuda.driver  # noqa: F401
PY
then
  : # already importable, nothing to do
elif [ -f "${PYCUDA_FAIL_MARKER}" ]; then
  echo "[ccai] skipping pycuda build - a previous attempt failed (marker: ${PYCUDA_FAIL_MARKER}, reason below). TensorRT YOLO path stays disabled; OpenCV DNN/HOG fallback is used instead. Delete that file to retry." >&2
  cat "${PYCUDA_FAIL_MARKER}" >&2 2>/dev/null || true
elif apt-get install -y python3-pycuda > "${PIP_CACHE_DIR}/pycuda_apt.log" 2>&1; then
  echo "[ccai] pycuda installed via apt (python3-pycuda)"
else
  echo "[ccai] python3-pycuda not available via apt (see ${PIP_CACHE_DIR}/pycuda_apt.log) - trying pip source build"
  # The actual cause of the earlier xlocale.h failure: newer glibc merged
  # xlocale.h into locale.h and dropped the old header entirely, but the
  # numpy version pycuda pulls in (needed to build for this old Python 3.6)
  # still #includes it unconditionally. This symlink is the standard,
  # widely-documented workaround - restores the header numpy expects without
  # patching numpy itself.
  if [ ! -e /usr/include/xlocale.h ] && [ -e /usr/include/locale.h ]; then
    echo "[ccai] symlinking /usr/include/xlocale.h -> locale.h (numpy build compat)"
    ln -s /usr/include/locale.h /usr/include/xlocale.h
  fi
  echo "[ccai] attempting pycuda pip install (capped at ${PYCUDA_BUILD_TIMEOUT_SECONDS:-600}s)"
  PYCUDA_BUILD_LOG="${PIP_CACHE_DIR}/pycuda_pip_build.log"
  if timeout "${PYCUDA_BUILD_TIMEOUT_SECONDS:-600}" python3 -m pip install pycuda 2>&1 | tee "${PYCUDA_BUILD_LOG}"; then
    echo "[ccai] pycuda installed"
  else
    echo "[ccai] pycuda install failed or timed out; TensorRT YOLO path will stay disabled (OpenCV DNN/HOG fallback used instead). Won't retry on future starts - delete ${PYCUDA_FAIL_MARKER} to retry." >&2
    {
      echo "pycuda pip build failed at $(date -u +%FT%TZ)"
      tail -n 40 "${PYCUDA_BUILD_LOG}" 2>/dev/null
    } > "${PYCUDA_FAIL_MARKER}"
  fi
fi

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
