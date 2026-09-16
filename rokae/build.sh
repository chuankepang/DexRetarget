#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python_bin="${ROKAE_PYTHON:-/SSD-512G/conda_envs/anydex/bin/python}"
build_dir="${ROKAE_BUILD_DIR:-$script_dir/build}"

cmake -S "$script_dir" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$python_bin"
cmake --build "$build_dir" --parallel

echo "Built module under: $script_dir/python/"
