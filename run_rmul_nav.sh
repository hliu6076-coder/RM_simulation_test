#!/usr/bin/env bash
# ROS 2 Humble setup scripts are not compatible with Bash nounset (`set -u`).
set -eo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
livox_setup="${LIVOX_WS_SETUP:-/home/wangxiaotao/ws_livox/install/setup.bash}"

for setup_file in \
  /opt/ros/humble/setup.bash \
  "${livox_setup}" \
  "${workspace_root}/install/setup.bash"; do
  if [[ ! -f "${setup_file}" ]]; then
    echo "Missing environment setup: ${setup_file}" >&2
    exit 1
  fi
done

cd "${workspace_root}"
source /opt/ros/humble/setup.bash
source "${livox_setup}"
source "${workspace_root}/install/setup.bash"

exec ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL \
  mode:=nav \
  lio:=fastlio \
  localization:=slam_toolbox \
  lio_rviz:=False \
  nav_rviz:=True \
  "$@"
