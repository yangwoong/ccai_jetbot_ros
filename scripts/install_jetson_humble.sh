#!/usr/bin/env bash
set -euo pipefail

echo "[ccai] Installing ROS2 Humble runtime dependencies"
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-opencv \
  python3-pil \
  python3-smbus \
  i2c-tools \
  v4l-utils \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs \
  ros-humble-teleop-twist-keyboard

python3 -m pip install --user fastapi uvicorn requests pyyaml

if ! rosdep --version >/dev/null 2>&1; then
  echo "[ccai] rosdep not available"
else
  sudo rosdep init 2>/dev/null || true
  rosdep update
fi

echo "[ccai] Done. Build with: colcon build --symlink-install"
