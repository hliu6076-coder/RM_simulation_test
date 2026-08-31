#!/usr/bin/env bash
# ROS 2 Humble setup scripts are not compatible with Bash nounset (`set -u`).
set -eo pipefail

# This computer must build with concurrency 1 to avoid freezes.
workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
livox_setup="${LIVOX_WS_SETUP:-/home/wangxiaotao/ws_livox/install/setup.bash}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Missing ROS 2 Humble setup: /opt/ros/humble/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${livox_setup}" ]]; then
  echo "Missing Livox workspace setup: ${livox_setup}" >&2
  exit 1
fi

cd "${workspace_root}"
source /opt/ros/humble/setup.bash
source "${livox_setup}"

# Limit both the package executor and the underlying Make/CMake build.
export CMAKE_POLICY_VERSION_MINIMUM=3.5
export CMAKE_BUILD_PARALLEL_LEVEL=1
export CTEST_PARALLEL_LEVEL=1
export MAKEFLAGS="-j1 -l1"

echo "Safe build enabled: colcon=1 package, make/cmake=1 job"

colcon build \
  --symlink-install \
  --executor sequential \
  --parallel-workers 1 \
  --packages-skip livox_ros_driver2 point_lio \
  --allow-overriding imu_complementary_filter \
  --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
