#!/usr/bin/env bash
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$project_dir/scripts/environment.sh"
mkdir -p "$project_dir/artifacts"
if [[ -f "$project_dir/artifacts/demo.pid" ]]; then
  previous_pid=$(cat "$project_dir/artifacts/demo.pid")
  if [[ "$previous_pid" =~ ^[0-9]+$ ]] && [[ -r /proc/$previous_pid/cmdline ]] && tr '\0' ' ' < "/proc/$previous_pid/cmdline" | grep -q 'ros2 launch'; then
    echo 'This project demo is already running. Stop it with: bash scripts/stop_demo.sh' >&2
    exit 1
  fi
fi
printf '%s\n' "$$" > "$project_dir/artifacts/demo.pid"
python3 "$project_dir/scripts/make_rviz_config.py" "$project_dir/artifacts/demo.rviz"
exec ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False use_rviz:=True use_sim_time:=True "rviz_config_file:=$project_dir/artifacts/demo.rviz" "$@"
