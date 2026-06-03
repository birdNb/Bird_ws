#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi
if [ -f "${HOME}/sim2real/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME}/sim2real/devel/setup.bash"
elif [ -f "${HOME}/sim2real/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME}/sim2real/install/setup.bash"
fi

rm -rf build
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"
echo "Build OK: $(pwd)/vision_controller"
