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
# NOTE ON VERSION PINNING: REALSENSE_TAG/REALSENSE_ROS_BRANCH below are best-
# effort defaults, not guaranteed-current tag/branch names - IntelRealSense's
# repos rename/retag over time and this was not verified against the live
# repos from this environment. If a clone fails, check
# https://github.com/IntelRealSense/librealsense/releases and
# https://github.com/IntelRealSense/realsense-ros for the current release,
# and rerun with e.g. REALSENSE_TAG=v2.xx.x REALSENSE_ROS_BRANCH=<branch> set.

cd "$(dirname "$0")/.."

REALSENSE_TAG="${REALSENSE_TAG:-master}"
REALSENSE_ROS_BRANCH="${REALSENSE_ROS_BRANCH:-ros2-master}"
DEPS_DIR="$(pwd)/deps"
mkdir -p "${DEPS_DIR}"

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "ROS2 Humble setup.bash not found" >&2
  exit 1
fi

echo "[ccai] Installing librealsense2 build dependencies"
apt-get update
apt-get install -y \
  git cmake build-essential pkg-config \
  libssl-dev libusb-1.0-0-dev libudev-dev \
  libgtk-3-dev

if pkg-config --exists realsense2 2>/dev/null; then
  echo "[ccai] librealsense2 already installed ($(pkg-config --modversion realsense2)), skipping build"
else
  if [ ! -d "${DEPS_DIR}/librealsense" ]; then
    echo "[ccai] Cloning librealsense (${REALSENSE_TAG})"
    git clone --depth 1 --branch "${REALSENSE_TAG}" https://github.com/IntelRealSense/librealsense.git "${DEPS_DIR}/librealsense"
  fi
  echo "[ccai] Building librealsense2 with the RSUSB backend (no kernel patch needed) - this is slow on Jetson, expect 30-60+ minutes"
  cmake -S "${DEPS_DIR}/librealsense" -B "${DEPS_DIR}/librealsense/build" \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_EXAMPLES=false \
    -DBUILD_GRAPHICAL_EXAMPLES=false \
    -DBUILD_PYTHON_BINDINGS=false \
    -DCMAKE_BUILD_TYPE=Release
  cmake --build "${DEPS_DIR}/librealsense/build" -- -j"$(nproc)"
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

echo "[ccai] Building realsense-ros with colcon (this also happens automatically next time container_build.sh runs)"
colcon build --symlink-install

cat <<'EOF'

[ccai] Done. Next steps:
  1. Connect the D435i via USB and confirm it's a forward-facing mount (not the CSI camera, which stays ceiling-mounted for object recognition).
  2. source install/setup.bash
  3. ros2 launch realsense2_camera rs_launch.py enable_depth:=true enable_color:=true
  4. In another shell: ros2 topic echo /camera/camera/depth/image_rect_raw --once
     (confirm it publishes - adjust config/robot.yaml depth_nav_node.depth_image_topic if the actual topic name differs, e.g. no double "camera/camera" namespace on some realsense-ros versions)
  5. Once confirmed, in config/robot.yaml set:
       depth_nav_node.enabled: true
       vision_nav_node.drive_enabled: false
  6. Launch the full stack with CCAI_ENABLE_DEPTH_NAV=1 in the environment.
EOF
