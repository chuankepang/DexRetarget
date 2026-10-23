#!/usr/bin/env bash
# Private copies of the three Ubuntu GUI libraries missing on this host.
set -euo pipefail
runtime_root="${REALSENSE_HOME:-/data/Software/realsense}"
mkdir -p "$runtime_root/downloads/ubuntu"
cd "$runtime_root/downloads/ubuntu"
apt-get download libglfw3=3.4-4 libglu1-mesa=9.0.2-1.1build2 libopengl0=1.7.0-3
for archive in ./libglfw3_*.deb ./libglu1-mesa_*.deb ./libopengl0_*.deb; do
    dpkg-deb -x "$archive" "$runtime_root/sdk"
done
