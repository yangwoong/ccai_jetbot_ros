#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [ -f .env ]; then
  set -a
  # shellcheck disable=SC1091
  source .env
  set +a
fi

export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$(pwd)/ccai_jetbot_patrol/config/cyclonedds.xml}"

# Both of the above (cyclonedds.xml's ParticipantIndex=none, and moving off
# domain 0) turned out NOT to fix "Failed to find a free participant index" -
# the exact same failure recurred verbatim on domain 42 too, which rules out
# stale per-domain state as the cause. The log showed why: this container's
# loopback interface isn't multicast-capable ("selected interface 'lo' is not
# multicast-capable: disabling multicast"), so CycloneDDS falls back to
# unicast-only discovery - and something about that fallback path with 11
# nodes starting at once here can't allocate participants reliably. Rather
# than keep chasing CycloneDDS internals, switch RMW implementations
# entirely to sidestep this specific component: FastDDS doesn't use this
# participant-index allocation scheme at all.
if find /opt/ros/humble -maxdepth 4 -name 'librmw_fastrtps_cpp.so' 2>/dev/null | grep -q .; then
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
else
  echo "[ccai] WARNING: rmw_fastrtps_cpp not found under /opt/ros/humble - staying on default RMW (cyclonedds), which is known-broken here for many-node startup" >&2
fi
echo "[ccai] ROS_DOMAIN_ID=${ROS_DOMAIN_ID} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<default>} CYCLONEDDS_URI=${CYCLONEDDS_URI}"
if [ ! -f "${CYCLONEDDS_URI#file://}" ]; then
  echo "[ccai] WARNING: CYCLONEDDS_URI points at a file that doesn't exist: ${CYCLONEDDS_URI#file://}" >&2
fi

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

FORCE_BUILD_ON_RUN="${FORCE_BUILD_ON_RUN:-1}"

if [ "${FORCE_BUILD_ON_RUN}" = "1" ] || [ ! -f install/setup.bash ]; then
  ./scripts/container_build.sh
fi

set +u
source install/setup.bash
set -u

# D435i/librealsense2 support (see scripts/install_realsense_d435i.sh) - no-op
# if that script was never run. Needed for realsense2_camera_node to find
# librealsense2.so at runtime, since it's installed to a bind-mounted prefix
# rather than a system path (so it survives container recreation).
if [ -f deps/librealsense/librealsense_env.sh ]; then
  set +u
  source deps/librealsense/librealsense_env.sh
  set -u
fi

exec ros2 launch ccai_jetbot_patrol patrol.launch.py
