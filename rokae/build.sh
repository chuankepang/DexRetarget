#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python_bin="${ROKAE_PYTHON:-/data/miniconda3/envs/dexteleop/bin/python}"
build_arch="$(uname -m)"
build_dir="${ROKAE_BUILD_DIR:-$script_dir/build/$build_arch}"

# Conda compiler activation configures the sysroot and linker consistently.
# Activate dexteleop before calling this script when using its build tools.

cmake -S "$script_dir" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$python_bin"
cmake --build "$build_dir" --parallel

echo "Built module under: $script_dir/python/"
