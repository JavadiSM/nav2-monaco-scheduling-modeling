#!/usr/bin/env bash
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$project_dir/scripts/environment.sh"
mkdir -p "$project_dir/artifacts"
exec python3 "$project_dir/scripts/verify_navigation.py" --output "$project_dir/artifacts/verification.json" "$@"

