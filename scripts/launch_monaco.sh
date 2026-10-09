#!/usr/bin/env bash
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$project_dir/scripts/environment.sh"
mkdir -p "$project_dir/artifacts"
if [[ -f "$project_dir/artifacts/demo.pid" ]]; then
  previous_pid=$(cat "$project_dir/artifacts/demo.pid")
  if [[ "$previous_pid" =~ ^[0-9]+$ ]] && [[ -r /proc/$previous_pid/cmdline ]] && tr '\0' ' ' < "/proc/$previous_pid/cmdline" | grep -q 'ros2 launch'; then
    echo 'A project demo is already running. Stop it with: bash scripts/stop_demo.sh' >&2
    exit 1
  fi
fi
python3 "$project_dir/scripts/verify_frozen_scene.py"
python3 "$project_dir/scripts/make_rviz_config.py" "$project_dir/artifacts/monaco.rviz"
python3 - "$project_dir/artifacts/monaco.rviz" <<'PY'
import sys, yaml
from pathlib import Path
p = Path(sys.argv[1])
c = yaml.safe_load(p.read_text())
c['Visualization Manager']['Views']['Current'].update({'X': 0.0, 'Y': 0.0, 'Angle': 0.0, 'Scale': 16.0})
c['Visualization Manager']['Displays'].append({'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Circuit checkpoints and edge cars', 'Enabled': True, 'Value': True, 'Marker Topic': {'Depth': 1, 'Durability Policy': 'Transient Local', 'History Policy': 'Keep Last', 'Reliability Policy': 'Reliable', 'Value': '/monaco/markers'}, 'Queue Size': 100})
p.write_text(yaml.safe_dump(c, sort_keys=False))
PY
printf '%s\n' "$$" > "$project_dir/artifacts/demo.pid"
exec ros2 launch "$project_dir/launch/monaco.launch.py" "$@"

