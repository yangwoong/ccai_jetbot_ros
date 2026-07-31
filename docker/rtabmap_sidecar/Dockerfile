# rtabmap_ros sidecar - see docs/navigation_roadmap.md's 2026-07-28 "두 번째
# 컨테이너" entry for the full reasoning. Short version: ros-humble-rtabmap-ros
# is a normal apt binary package on ROS2 Humble's OFFICIAL target OS
# (Ubuntu 22.04/jammy) - it's only missing from this project's MAIN
# container (dustynv/ros:humble-desktop-l4t-r32.7.1, a custom Ubuntu
# 18.04/bionic backport of Humble - see scripts/install_slam_nav2.sh's
# repeated from-source-build OOM failures on that image). Base this on the
# official arm64 ros:humble-ros-base image instead and just apt-get
# install it - no compilation at all, so no OOM risk. Runs as a SEPARATE
# container (scripts/run_rtabmap_container.sh) alongside the main
# ccai-jetbot one, sharing --network host + ROS_DOMAIN_ID to subscribe to
# the D435i topics the main container already publishes.
#
# UNVERIFIED (2026-07-28): never built or run on real hardware. If
# ros-humble-rtabmap-ros pulls in a dependency this Jetson's aarch64 jammy
# apt mirror doesn't have, that will show up as an apt error here - send
# the build log back the same way as every other package gap this session.
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-rtabmap-ros \
      ros-humble-image-transport \
      ros-humble-image-transport-plugins \
      ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ccai_launch
