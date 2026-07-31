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
# 2026-07-28: a run was diagnosed as "the fix didn't work" when it was
# actually running a stale checkout (no `git pull` before rerunning) -
# the container bind-mounts the host repo, so a git pull on either side
# is enough, but there was no way to tell from the log alone that this
# had (or hadn't) happened. Print it every run so that's never ambiguous.
echo "[ccai] repo commit: $(git rev-parse --short HEAD 2>/dev/null || echo unknown) ($(git log -1 --format=%cd --date=iso 2>/dev/null || echo unknown))"

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
  #
  # 2026-07-28: confirmed on real hardware that `swapon` fails with
  # "Operation not permitted" from INSIDE this container - ordinary Docker
  # containers don't have CAP_SYS_ADMIN, which swapon requires, regardless
  # of the container's own memory limits. Swap is a host-kernel resource
  # anyway: once enabled on the HOST, every container using that kernel can
  # use it with no extra privileges needed - trying to swapon from inside
  # the container was the wrong layer for this from the start. This
  # function now only ever probes/reports; if there isn't enough swap
  # already active on the host, it prints the exact commands to run there
  # (outside docker) instead of silently attempting (and failing) to
  # create it here.
  local total_swap_kb
  total_swap_kb=$(awk '/SwapTotal/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)
  # Confirmed on real hardware that 4GB RAM + ~4GB pre-existing swap still
  # OOM-killed the compiler on more than one corelib file even with the
  # -O1 fix below - 6GB is a floor, not a guarantee; if it still fails,
  # go bigger.
  if [ "${total_swap_kb:-0}" -ge 6291456 ]; then
    echo "[ccai] swap already present ($((total_swap_kb / 1024))MB) - OK"
    return 0
  fi
  cat <<EOF
[ccai] Only $((total_swap_kb / 1024))MB of swap is active, and this container
[ccai] can't create more itself (swapon needs a host-level privilege Docker
[ccai] containers don't have). Run this on the JETSON HOST (not inside
[ccai] "docker exec"), then re-run this script inside the container:

  sudo fallocate -l 8G /swapfile_ccai_rtabmap || sudo dd if=/dev/zero of=/swapfile_ccai_rtabmap bs=1M count=8192
  sudo chmod 600 /swapfile_ccai_rtabmap
  sudo mkswap /swapfile_ccai_rtabmap
  sudo swapon /swapfile_ccai_rtabmap

[ccai] Continuing anyway with whatever swap is already active - this build
[ccai] may OOM again without it.
EOF
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
  # 2026-07-28: confirmed on real hardware that the default Release build
  # (-O3) OOM-kills the compiler partway through corelib/src/Parameters.cpp
  # even at -j1 with 4GB RAM + 4GB swap - Parameters.cpp's giant macro-based
  # parameter table is a well-known single-file memory hog for RTAB-Map on
  # constrained ARM boards (make's "wait: No child processes" is the
  # kernel OOM killer reaping cc1plus, not a real compile error). -O1
  # instead of -O3 is the standard community workaround, cutting per-file
  # compiler memory substantially. Also drop BUILD_APP/BUILD_TOOLS/
  # BUILD_EXAMPLES (RTAB-Map's own Qt5/VTK-based GUI/tools/examples) -
  # this project only needs the core library + rtabmap_ros, and those Qt/VTK
  # targets are themselves heavy to compile.
  if ! timeout "${core_timeout}" bash -c "
      set -e
      cd '${ws}/src/rtabmap'
      # Wipe any previous build dir - a prior attempt (like the one that
      # just OOM-killed mid-compile) can leave a CMake cache configured
      # without today's flags, or half-written object files; safer to
      # reconfigure from scratch than assume incremental state is sound.
      rm -rf build
      mkdir -p build
      cd build
      # 2026-07-28: -O1 alone got further (past Parameters.cpp) but still
      # OOM-killed on a later file (Odometry.cpp) - stacking GCC's own
      # memory-reduction params on top: lowering ggc-min-expand/
      # ggc-min-heapsize makes the compiler's garbage collector run more
      # often during compilation instead of letting garbage accumulate,
      # trading some compile time for meaningfully lower peak memory per
      # translation unit. See ensure_swap's comment for the other half of
      # this mitigation (more swap - has to be done on the host, not here).
      cmake -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_CXX_FLAGS_RELEASE='-O1 --param ggc-min-expand=10 --param ggc-min-heapsize=32768' \
            -DBUILD_APP=OFF -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF ..
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
    cat <<EOF
[ccai] ros-humble-rtabmap-ros is missing here (expected - see above), and
[ccai] this script no longer attempts to source-build it in THIS
[ccai] container (2026-07-28: three attempts here each OOM-killed the
[ccai] compiler on a different corelib file, even after -O1/dropping the
[ccai] GUI build and adding swap - see docs/navigation_roadmap.md's
[ccai] "rtabmap 코어 빌드" entries for the history if picking this back up
[ccai] is ever worth it).
[ccai]
[ccai] Instead: ros-humble-rtabmap-ros is a normal apt binary package on
[ccai] ROS2 Humble's OFFICIAL target OS (Ubuntu 22.04/jammy) - just not on
[ccai] THIS container's custom Ubuntu 18.04/bionic backport. Run this on
[ccai] the Jetson HOST instead (not inside this container):
[ccai]
[ccai]   scripts/run_rtabmap_container.sh
[ccai]
[ccai] which builds a separate, standard jammy-based container (apt
[ccai] install only, no compilation, no OOM risk) and runs it alongside
[ccai] this one, sharing --network host + ROS_DOMAIN_ID so it can
[ccai] subscribe to the D435i topics this container already publishes.
[ccai] build_rtabmap_from_source() is kept in this script only as a
[ccai] reference for what was already tried, in case the source-build
[ccai] path needs revisiting for some other reason later.
EOF
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
