#!/usr/bin/env bash
set -euo pipefail

# Installs RTAB-Map (RGB-D SLAM: visual odometry + occupancy-grid mapping
# from the D435i alone - no wheel encoders needed) and Nav2 (costmap +
# planner + controller, for actually navigating to a point on that map).
#
# This is a genuinely large, experimental addition on top of the reactive
# depth_nav_node patrol that is this project's stable default - see
# docs/navigation_roadmap.md for how the two relate. Run this INSIDE the
# ccai-jetbot container (same place install_realsense_d435i.sh runs):
#   docker exec -it ccai-jetbot bash
#   scripts/install_slam_nav2.sh
#
# PACKAGE AVAILABILITY IS UNVERIFIED FROM THIS ENVIRONMENT. This image
# backports ROS2 Humble onto bionic (Ubuntu 18.04) via a custom apt repo
# that has already turned out to be missing ordinary packages
# (ros-humble-xacro, ros-humble-diagnostic-updater - see
# scripts/install_realsense_d435i.sh). rtabmap_ros and Nav2 are each dozens
# of packages with their own dependency trees; unlike the small gaps found
# so far, source-building either stack blind is not realistic to script.
# This script tries apt first and reports exactly what's missing - if
# packages ARE missing, that list is the next thing to act on (case by
# case), not something this script attempts to paper over automatically.
#
# 2026-07-28: confirmed (again) that all four packages below are absent
# from this image's apt repo - see docs/navigation_roadmap.md's 2026-07-24
# entry, this hasn't changed. Per explicit user request to try the source
# build path in parallel with continuing to improve the existing
# LLM/room-scan explorer (not instead of it), this script now ALSO attempts
# a source build of rtabmap's CORE library + rtabmap_ros ONLY (not the
# rest of Nav2 - that's an even larger stack, and this project's own
# point-to-point controller already covers "go to a taught location"
# without it) when the apt path comes up empty. This is a genuinely
# large, UNVERIFIED, likely-to-partially-fail attempt on a 4GB-RAM Jetson
# Nano - see build_rtabmap_from_source() below for the specific
# safeguards (swap check, capped parallelism, timeout, full logging) and
# read the log this produces before assuming anything succeeded.

cd "$(dirname "$0")/.."

LOG_FILE="${LOG_FILE:-$(pwd)/install_slam_nav2.log}"
exec > >(tee -a "${LOG_FILE}") 2>&1
echo "[ccai] logging to ${LOG_FILE}"

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

./scripts/container_fix_ros_apt_key.sh

ensure_swap() {
  # Building rtabmap's core (against PCL/g2o/OpenCV) is real C++ compilation
  # work - on this 4GB-RAM Jetson Nano, past attempts at similarly large
  # native builds (pycuda) showed real memory pressure. A swapfile gives a
  # slow-but-survivable path instead of the OOM killer taking out the build
  # (or worse, some other container process) partway through.
  local total_swap_kb
  total_swap_kb=$(awk '/SwapTotal/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)
  if [ "${total_swap_kb:-0}" -ge 2097152 ]; then
    echo "[ccai] swap already present ($((total_swap_kb / 1024))MB), skipping swapfile creation"
    return 0
  fi
  local swap_path="/swapfile_ccai_rtabmap"
  local swap_size_mb="${RTABMAP_SWAP_SIZE_MB:-4096}"
  if [ -f "${swap_path}" ]; then
    echo "[ccai] ${swap_path} already exists, enabling it"
    swapon "${swap_path}" 2>/dev/null || true
    return 0
  fi
  local free_disk_mb
  free_disk_mb=$(df -Pm / | awk 'NR==2 {print $4}')
  if [ "${free_disk_mb:-0}" -lt $((swap_size_mb + 1024)) ]; then
    echo "[ccai] not enough free disk (${free_disk_mb}MB) to safely add a ${swap_size_mb}MB swapfile - skipping, build may OOM" >&2
    return 0
  fi
  echo "[ccai] creating ${swap_size_mb}MB swapfile at ${swap_path} (no existing swap found)"
  if fallocate -l "${swap_size_mb}M" "${swap_path}" 2>/dev/null || dd if=/dev/zero of="${swap_path}" bs=1M count="${swap_size_mb}" 2>/dev/null; then
    chmod 600 "${swap_path}"
    mkswap "${swap_path}" && swapon "${swap_path}" && echo "[ccai] swap enabled"
  else
    echo "[ccai] swapfile creation failed - continuing without it, build may OOM" >&2
  fi
}

build_rtabmap_from_source() {
  echo "[ccai] ===== attempting rtabmap source build (unverified, may fail - see docs/navigation_roadmap.md) ====="
  local build_log
  build_log="$(pwd)/rtabmap_source_build.log"
  : > "${build_log}"

  ensure_swap 2>&1 | tee -a "${build_log}"

  echo "[ccai] installing rtabmap's system build dependencies via apt (best-effort)" | tee -a "${build_log}"
  local sys_deps=(libpcl-dev libsqlite3-dev libqt5svg5-dev libopencv-dev libeigen3-dev)
  local missing_sys=()
  for dep in "${sys_deps[@]}"; do
    apt-get install -y "${dep}" >> "${build_log}" 2>&1 || true
    if ! dpkg -s "${dep}" >/dev/null 2>&1; then
      missing_sys+=("${dep}")
    fi
  done
  if [ "${#missing_sys[@]}" -gt 0 ]; then
    echo "[ccai] missing system build deps: ${missing_sys[*]} - the build below will likely fail on these, see ${build_log}" | tee -a "${build_log}" >&2
  fi

  local ws="deps/rtabmap_ws"
  mkdir -p "${ws}/src"
  if [ ! -d "${ws}/src/rtabmap" ]; then
    (git clone --branch humble-devel --depth 1 https://github.com/introlab/rtabmap.git "${ws}/src/rtabmap" ||
     git clone --depth 1 https://github.com/introlab/rtabmap.git "${ws}/src/rtabmap") >> "${build_log}" 2>&1
  fi
  if [ ! -d "${ws}/src/rtabmap_ros" ]; then
    (git clone --branch humble-devel --depth 1 https://github.com/introlab/rtabmap_ros.git "${ws}/src/rtabmap_ros" ||
     git clone --depth 1 https://github.com/introlab/rtabmap_ros.git "${ws}/src/rtabmap_ros") >> "${build_log}" 2>&1
  fi

  local core_timeout="${RTABMAP_CORE_BUILD_TIMEOUT_SECONDS:-2400}"
  echo "[ccai] building rtabmap core library (cmake, capped at ${core_timeout}s, 1 job to limit memory use)" | tee -a "${build_log}"
  if ! timeout "${core_timeout}" bash -c "
      set -e
      cd '${ws}/src/rtabmap'
      mkdir -p build
      cd build
      cmake ..
      make -j1
      make install
    " >> "${build_log}" 2>&1
  then
    echo "[ccai] rtabmap core build failed or timed out - see ${build_log}. Not attempting rtabmap_ros (it depends on this)." | tee -a "${build_log}" >&2
    return 1
  fi
  ldconfig 2>/dev/null || true

  local ros_timeout="${RTABMAP_ROS_BUILD_TIMEOUT_SECONDS:-2400}"
  echo "[ccai] building rtabmap_ros (colcon, capped at ${ros_timeout}s, 1 parallel worker)" | tee -a "${build_log}"
  if ! timeout "${ros_timeout}" bash -c "
      set -e
      cd '${ws}'
      colcon build --packages-select rtabmap_ros --parallel-workers 1
    " >> "${build_log}" 2>&1
  then
    echo "[ccai] rtabmap_ros build failed or timed out - see ${build_log}" | tee -a "${build_log}" >&2
    return 1
  fi

  cat <<EOF | tee -a "${build_log}"
[ccai] rtabmap_ros built successfully at ${ws}.
[ccai] To use it: 'source ${ws}/install/setup.bash' (in addition to the main
[ccai] workspace's setup.bash) before launching. This build does NOT wire
[ccai] anything into patrol_node/depth_nav_node automatically - integrating
[ccai] it (launch files, remapping the D435i topics it needs, deciding how
[ccai] it coexists with or replaces room-scan) is a separate next step.
EOF
}

PACKAGES=(
  ros-humble-rtabmap-ros
  ros-humble-navigation2
  ros-humble-nav2-bringup
  ros-humble-robot-localization
)

echo "[ccai] Attempting apt install of: ${PACKAGES[*]}"
MISSING=()
for pkg in "${PACKAGES[@]}"; do
  if apt-get install -y "${pkg}" 2>&1 | tee /tmp/ccai_apt_pkg.log; then
    :
  fi
  if ! dpkg -s "${pkg}" >/dev/null 2>&1; then
    MISSING+=("${pkg}")
  fi
done

if [ "${#MISSING[@]}" -gt 0 ]; then
  echo
  echo "[ccai] NOT installed via apt: ${MISSING[*]}"
  echo "[ccai] This image's ROS apt repo doesn't carry these (same situation as xacro/diagnostic_updater earlier)."

  RTABMAP_MISSING=0
  for pkg in "${MISSING[@]}"; do
    [ "${pkg}" = "ros-humble-rtabmap-ros" ] && RTABMAP_MISSING=1
  done
  if [ "${RTABMAP_MISSING}" -eq 1 ]; then
    if ! build_rtabmap_from_source; then
      echo "[ccai] rtabmap source build did not complete - see rtabmap_source_build.log and send it back; each failure gets resolved one at a time from the actual error, same as xacro/diagnostic_updater/pycuda before it, not guessed upfront." >&2
    fi
  fi

  echo "[ccai] Not attempting navigation2/nav2-bringup/robot-localization from source in this script -"
  echo "[ccai] that stack is even larger, and this project's own point-to-point controller already covers"
  echo "[ccai] 'go to a taught location' without it. Revisit separately once/if rtabmap's own localization"
  echo "[ccai] and occupancy grid are confirmed working."
  exit 0
fi

echo "[ccai] All packages installed via apt successfully."
echo "[ccai] Note: this project's navigation stack has since moved to a custom"
echo "[ccai] lightweight controller (visual_odom_node + patrol_node's own"
echo "[ccai] point-to-point/coverage logic) instead of rtabmap/Nav2 - see"
echo "[ccai] docs/navigation_roadmap.md. These packages being installed here"
echo "[ccai] doesn't wire anything up on its own."
