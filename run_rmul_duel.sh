#!/usr/bin/env bash
# Lightweight two-robot RMUL manual duel. ROS setup scripts require nounset off.
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

exec ros2 launch rm_combat_gazebo rmul_duel.launch.py \
  gui:=true \
  autostart:=false \
  contact_damage:=true \
  lan_gateway:=true \
  lan_bind_host:=0.0.0.0 \
  lan_port:=8765 \
  lan_red_token:=red-test-2026 \
  lan_blue_token:=blue-test-2026 \
  lan_referee_token:=referee-test-2026 \
  "$@"
