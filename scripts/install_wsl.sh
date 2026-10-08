#!/usr/bin/env bash
set -euo pipefail
[[ $(id -u) -eq 0 ]] || { echo 'Run this installer as root.' >&2; exit 1; }
source /etc/os-release
[[ ${VERSION_CODENAME:-} == noble ]] || { echo 'This setup targets Ubuntu 24.04 Noble.' >&2; exit 1; }
export DEBIAN_FRONTEND=noninteractive
apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 update
apt-get install -y ca-certificates curl locales git python3
if ! dpkg-query -W ros2-apt-source >/dev/null 2>&1; then
  release=$(curl -fsSL --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
  curl -fL --retry 3 -o /tmp/warehouse-ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${release}/ros2-apt-source_${release}.noble_all.deb"
  sha256sum /tmp/warehouse-ros2-apt-source.deb
  dpkg -i /tmp/warehouse-ros2-apt-source.deb
fi
locale-gen en_US.UTF-8
apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 update
apt-get install -y ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-minimal-tb3-sim ros-jazzy-ros-gz ros-dev-tools gh mesa-utils x11-utils xdotool imagemagick ffmpeg
echo 'ROS2_WAREHOUSE_INSTALL_COMPLETE'
