#!/usr/bin/env bash
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
pid_file="$project_dir/artifacts/demo.pid"
[[ -f "$pid_file" ]] || { echo 'No project demo PID is recorded.'; exit 0; }
demo_pid=$(cat "$pid_file")
[[ "$demo_pid" =~ ^[0-9]+$ ]] || { echo 'Invalid demo PID file.' >&2; exit 1; }
[[ -r /proc/$demo_pid/cmdline ]] || { echo 'The recorded demo has already exited.'; exit 0; }
demo_command=$(tr '\0' ' ' < "/proc/$demo_pid/cmdline")
if [[ "$demo_command" != *'ros2 launch nav2_bringup tb3_simulation_launch.py'* && "$demo_command" != *"ros2 launch $project_dir/launch/monaco.launch.py"* ]]; then
  echo 'The recorded PID belongs to another process; refusing to stop it.' >&2
  exit 1
fi
kill -INT "$demo_pid"
echo 'Requested graceful shutdown of the project demo.'

