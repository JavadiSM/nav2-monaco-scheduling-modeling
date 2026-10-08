#!/usr/bin/env bash
# Source from the project wrappers. Keep changes local to the calling process.
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy is missing. Run: sudo bash scripts/install_wsl.sh' >&2
  return 1
fi
# ROS setup files are not compatible with nounset in every installation.
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export LANG=${LANG:-en_US.UTF-8}
if [[ -S /tmp/.X11-unix/X0 ]]; then
  export DISPLAY=${DISPLAY:-:0}
fi
if [[ -S /mnt/wslg/runtime-dir/wayland-0 ]]; then
  export WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}
  export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}
fi
if [[ -S /mnt/wslg/PulseServer ]]; then
  export PULSE_SERVER=${PULSE_SERVER:-unix:/mnt/wslg/PulseServer}
fi
# X11 is supported by WSLg and enables window-specific evidence capture.
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-xcb}
export QT_X11_NO_MITSHM=1

