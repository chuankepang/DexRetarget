#!/usr/bin/env bash
# Install Python dependencies only in the project's dexteleop Conda environment.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime_root="${REALSENSE_HOME:-/data/Software/realsense}"
python_bin="${REALSENSE_PYTHON:-/data/miniconda3/envs/dexteleop/bin/python}"
if [[ ! -x "$python_bin" ]]; then
    echo "Python not found: $python_bin; set REALSENSE_PYTHON to a supported Python executable." >&2
    exit 1
fi
if [[ ! -d "$(dirname -- "$python_bin")/../conda-meta" ]]; then
    echo "Expected a Conda environment Python: $python_bin" >&2
    exit 1
fi
"$python_bin" -m pip install -r "$project_root/realsense/requirements.txt"
mkdir -p "$runtime_root"
"$python_bin" -m pip freeze > "$runtime_root/installed-requirements.txt"
echo "Installed RealSense Python SDK in Conda: $python_bin"
echo "Inventory: $python_bin $project_root/realsense/scripts/check_cameras.py --inventory"
