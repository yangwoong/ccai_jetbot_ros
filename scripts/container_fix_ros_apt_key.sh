#!/usr/bin/env bash
set -euo pipefail

# Open Robotics rotated the ROS2 apt repo's signing key in 2025; containers
# built from images baked before that rotation (this dustynv image included)
# have the old, now-expired key and every `apt-get update` fails with
# "EXPKEYSIG ... Open Robotics <info@osrfoundation.org>" - which then blocks
# any apt-get install too, in this container and any other apt work.
#
# Idempotent and safe to call before any apt-get update in this repo's
# scripts: does nothing if apt-get update already succeeds.

if apt-get update >/tmp/ccai_apt_update.log 2>&1; then
  exit 0
fi

if ! grep -qE "EXPKEYSIG|NO_PUBKEY|KEYEXPIRED" /tmp/ccai_apt_update.log; then
  echo "[ccai] apt-get update failed for a reason other than an expired/missing key:" >&2
  cat /tmp/ccai_apt_update.log >&2
  exit 1
fi

echo "[ccai] ROS2 apt signing key expired/missing - fetching the current key"
KEYRING_DIR="/usr/share/keyrings"
mkdir -p "${KEYRING_DIR}"
if curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros.key; then
  if command -v gpg >/dev/null 2>&1; then
    gpg --dearmor < /tmp/ros.key > "${KEYRING_DIR}/ros-archive-keyring.gpg"
  fi
  # Also register it the legacy apt-key way, since older /etc/apt/sources.list
  # entries (rather than a signed-by= .sources file) are what this bionic-
  # based image ships with, and apt-key is what those actually check.
  apt-key add /tmp/ros.key 2>/dev/null || true
else
  echo "[ccai] failed to fetch the new ROS apt key from GitHub" >&2
  exit 1
fi

echo "[ccai] retrying apt-get update"
apt-get update
