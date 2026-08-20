#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sdk_root="${ROKAE_SDK_ROOT:-/SSD-512G/Project/rokae-cpp/xCoreSDK-CPP}"

cmake -S "$script_dir" -B "$script_dir/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DROKAE_SDK_ROOT="$sdk_root"
cmake --build "$script_dir/build" --parallel

echo "Built: $script_dir/build/libanydex_rokae_bridge.so"
