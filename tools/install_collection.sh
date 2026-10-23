#!/usr/bin/env bash
# One environment for teleop, capture, processing and dataset export.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${DEXTELEOP_PYTHON:-/data/miniconda3/envs/dexteleop/bin/python}"
if [[ ! -x "$python_bin" || ! -d "$(dirname -- "$python_bin")/../conda-meta" ]]; then
    echo "Expected dexteleop Conda Python; set DEXTELEOP_PYTHON if it was relocated." >&2
    exit 1
fi
"$python_bin" -m pip install -r "$project_root/requirements/collection.txt"
if [[ "${1:-}" == "--export" ]]; then
    "$python_bin" -m pip install torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cu128
    "$python_bin" -m pip install --no-deps 'https://github.com/huggingface/lerobot/archive/0cf864870cf29f4738d3ade893e6fd13fbd7cdb5.zip'
    "$python_bin" -m pip install -r "$project_root/requirements/export.txt"
fi
echo "Ready: $python_bin"
