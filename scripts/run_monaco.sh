#!/usr/bin/env bash
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$project_dir/scripts/environment.sh"
exec python3 "$project_dir/scripts/drive_monaco.py" "$@"
