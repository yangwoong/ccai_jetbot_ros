#!/usr/bin/env bash
set -euo pipefail

# Installs what's needed to run an Intel RealSense D435i under ROS2 Humble on
# this Jetson (JetPack 4.6.x / L4T r32.7.1, dustynv/ros:humble-desktop image).
#
# Intel does not publish arm64 apt packages for librealsense2, so the
# community-recommended path for Jetson is to build librealsense from source
# using its userspace/libusb backend (-DFORCE_RSUSB_BACKEND=true), which
# avoids needing a patched kernel UVC driver entirely. The realsense-ros ROS2
# wrapper is also built from source here rather than via apt, because its
# .deb depends on an apt librealsense2 package that doesn't exist for arm64 -
# installing it via apt would just fail dependency resolution.
#
# Run this INSIDE the ccai-jetbot container (same place container_build.sh
# runs) so it shares the same ROS2 workspace/colcon toolchain:
#   docker exec -it ccai-jetbot bash
#   scripts/install_realsense_d435i.sh
#
# NOTE ON VERSION PINNING: REALSENSE_TAG/REALSENSE_ROS_BRANCH/XACRO_BRANCH/
# DIAGNOSTICS_BRANCH below are best-effort defaults, not guaranteed-current
# tag/branch names - none of this was verified against the live repos from
# this environment. If a clone fails, check the repo on GitHub for the
# current release/branch and rerun with the matching env var set
# (IntelRealSense/librealsense, IntelRealSense/realsense-ros, ros/xacro,
# ros/diagnostics).
#
# RESUMING AFTER AN INTERRUPTION: safe to just rerun as-is. The clone is
# skipped if deps/librealsense already exists, and `cmake --build` (make
# underneath) only recompiles what's missing/changed, so a rerun resumes
# rather than starts over - UNLESS the interruption was a hard kill mid-write
# (e.g. OOM) that could leave a partial object file; if a rerun immediately
# fails again in the same spot, wipe just the build directory (keeps the
# cloned source, avoiding a re-clone) and rebuild clean:
#   rm -rf deps/librealsense/build
#   scripts/install_realsense_d435i.sh
# A dropped `docker exec -it` session (SSH disconnect, terminal closed) also
# kills the build since it's the session's foreground process - run it
# detached so it survives that:
#   docker exec -d ccai-jetbot bash -c \
#     "cd /home/workspace/ccai_jetbot_ros && ./scripts/install_realsense_d435i.sh"
# The script logs to install_realsense_d435i.log in the repo root itself (see
# LOG_FILE below) - that's bind-mounted, so check progress straight from the
# HOST, no docker exec needed, and it survives even if the container crashes
# or gets recreated:
#   tail -f install_realsense_d435i.log

cd "$(dirname "$0")/.."

# Self-log to a file inside the bind-mounted repo (not /tmp - /tmp lives only
# inside the container's writable layer, so it's gone the moment the
# container is recreated, e.g. via host_docker_run.sh - which is exactly what
# ate the previous run's log). This path is visible from the HOST too, so it
# can be checked with a plain `cat`/`tail`, no docker exec needed, even after
# a crash that takes the container down with it.
LOG_FILE="${LOG_FILE:-$(pwd)/install_realsense_d435i.log}"
exec > >(tee -a "${LOG_FILE}") 2>&1
echo "[ccai] logging to ${LOG_FILE} (host path: $(pwd)/install_realsense_d435i.log under the bind-mounted repo)"

REALSENSE_TAG="${REALSENSE_TAG:-master}"
REALSENSE_ROS_BRANCH="${REALSENSE_ROS_BRANCH:-ros2-master}"
DEPS_DIR="$(pwd)/deps"
mkdir -p "${DEPS_DIR}"

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

echo "[ccai] Installing librealsense2 build dependencies"
./scripts/container_fix_ros_apt_key.sh
apt-get install -y \
  git cmake build-essential pkg-config \
  libssl-dev libusb-1.0-0-dev libudev-dev \
  libgtk-3-dev

if pkg-config --exists realsense2 2>/dev/null; then
  echo "[ccai] librealsense2 already installed ($(pkg-config --modversion realsense2)), skipping build"
  # Still needed even when skipping the build below - see the explanation at
  # the other touch of this file a few lines down.
  [ -d "${DEPS_DIR}/librealsense" ] && touch "${DEPS_DIR}/librealsense/COLCON_IGNORE"
else
  if [ ! -d "${DEPS_DIR}/librealsense" ]; then
    echo "[ccai] Cloning librealsense (${REALSENSE_TAG})"
    git clone --depth 1 --branch "${REALSENSE_TAG}" https://github.com/IntelRealSense/librealsense.git "${DEPS_DIR}/librealsense"
  fi
  # librealsense's own repo has a top-level CMakeLists.txt, which makes colcon
  # (run later for realsense-ros, and by container_build.sh on every rebuild)
  # mistake it for a colcon package and try to build it *again* itself with
  # colcon's own default cmake options (examples included, none of our
  # -DFORCE_RSUSB_BACKEND/-DBUILD_EXAMPLES=false flags below) - wasted time at
  # best, and it can fail outright (e.g. "file not recognized: File
  # truncated" building the bundled example apps we don't need). We already
  # build+install librealsense2 ourselves right below via plain cmake, so
  # colcon has no reason to touch this directory at all.
  touch "${DEPS_DIR}/librealsense/COLCON_IGNORE"
  echo "[ccai] Building librealsense2 with the RSUSB backend (no kernel patch needed) - this is slow on Jetson, expect 30-60+ minutes"
  cmake -S "${DEPS_DIR}/librealsense" -B "${DEPS_DIR}/librealsense/build" \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_EXAMPLES=false \
    -DBUILD_GRAPHICAL_EXAMPLES=false \
    -DBUILD_PYTHON_BINDINGS=false \
    -DCMAKE_BUILD_TYPE=Release
  # Jetson Nano has only 4GB RAM; librealsense's larger C++ translation units
  # (each easily 500MB-1GB+ while compiling) can OOM-kill the build under
  # -j$(nproc) (4 parallel jobs). Defaulting to 2 trades some speed for not
  # getting silently killed partway through. Override with
  # REALSENSE_BUILD_JOBS=N if this device has more headroom (e.g. swap).
  cmake --build "${DEPS_DIR}/librealsense/build" -- -j"${REALSENSE_BUILD_JOBS:-2}"
  cmake --install "${DEPS_DIR}/librealsense/build"
  ldconfig

  echo "[ccai] Installing udev rules for USB permissions"
  if [ -f "${DEPS_DIR}/librealsense/config/99-realsense-libusb.rules" ]; then
    cp "${DEPS_DIR}/librealsense/config/99-realsense-libusb.rules" /etc/udev/rules.d/
    udevadm control --reload-rules 2>/dev/null || true
    udevadm trigger 2>/dev/null || true
  else
    echo "[ccai] udev rules file not found at expected path - USB permissions may need manual setup" >&2
  fi
fi

echo "[ccai] Fetching realsense-ros (ROS2 wrapper) source into deps/"
if [ ! -d "${DEPS_DIR}/realsense-ros" ]; then
  git clone --depth 1 --branch "${REALSENSE_ROS_BRANCH}" https://github.com/IntelRealSense/realsense-ros.git "${DEPS_DIR}/realsense-ros"
fi

if command -v rosdep >/dev/null 2>&1; then
  echo "[ccai] Installing realsense-ros's ROS package dependencies via rosdep"
  rosdep update || true
  rosdep install --from-paths "${DEPS_DIR}/realsense-ros" --ignore-src -y --skip-keys="librealsense2" || true
fi

# Confirmed (not just a rosdep OS-mapping gap, an earlier assumption here
# that turned out wrong): this image's ROS apt repo has no ros-humble-xacro
# or ros-humble-diagnostic-updater package at all ("Unable to locate
# package"). This image backports ROS2 Humble onto bionic (Ubuntu 18.04) for
# L4T r32.7.1 compatibility, and these two apparently weren't built for that
# combination. Build them from source instead, in the same workspace as
# realsense-ros so colcon picks them up automatically alongside it.
XACRO_BRANCH="${XACRO_BRANCH:-ros2}"
DIAGNOSTICS_BRANCH="${DIAGNOSTICS_BRANCH:-ros2}"
if [ ! -d "${DEPS_DIR}/xacro" ]; then
  echo "[ccai] Cloning ros/xacro (${XACRO_BRANCH}) - no apt package for it on this image"
  git clone --depth 1 --branch "${XACRO_BRANCH}" https://github.com/ros/xacro.git "${DEPS_DIR}/xacro"
fi
if [ ! -d "${DEPS_DIR}/diagnostics" ]; then
  echo "[ccai] Cloning ros/diagnostics (${DIAGNOSTICS_BRANCH}) for diagnostic_updater/diagnostic_msgs - no apt package for it on this image"
  git clone --depth 1 --branch "${DIAGNOSTICS_BRANCH}" https://github.com/ros/diagnostics.git "${DEPS_DIR}/diagnostics"
fi
if command -v rosdep >/dev/null 2>&1; then
  echo "[ccai] Installing xacro/diagnostics' own package dependencies via rosdep"
  rosdep install --from-paths "${DEPS_DIR}/xacro" "${DEPS_DIR}/diagnostics" --ignore-src -y || true
fi
# NOTE: as with librealsense/realsense-ros above, if THIS clone also hits a
# missing-apt-package wall for one of its own transitive dependencies, the
# same pattern applies - clone that dependency's repo into deps/ too. Check
# the colcon build output below for which package failed and why.

echo "[ccai] Building realsense-ros with colcon (this also happens automatically next time container_build.sh runs)"
colcon build --symlink-install

cat <<'EOF'

[ccai] Done. Next steps:
  1. Connect the D435i via USB (forward-facing mount - the CSI camera stays ceiling-mounted for object recognition).
  2. Sanity-check the realsense driver on its own first, before trusting the integrated launch:
       source install/setup.bash
       ros2 launch realsense2_camera rs_launch.py enable_depth:=true enable_color:=false
     In another shell: ros2 topic echo /camera/camera/depth/image_rect_raw --once
     (confirm it publishes - if the topic name differs, e.g. no double "camera/camera" namespace on
      some realsense-ros versions, update depth_nav_node.depth_image_topic in config/robot.yaml)
  3. config/robot.yaml already defaults to depth_nav_node.enabled: true and
     vision_nav_node.drive_enabled: false now that D435i is the primary nav sensor.
  4. Recreate the container so it picks up CCAI_ENABLE_DEPTH_NAV and the USB device mount
     (a plain `docker restart` does NOT pick up new `docker run` flags/env):
       ./scripts/host_docker_run.sh
     This launches depth_nav_node AND the realsense2_camera driver together (patrol.launch.py
     includes rs_launch.py when CCAI_ENABLE_DEPTH_NAV=1) - no separate manual realsense launch
     needed for normal operation once step 2 confirms it works.
EOF
