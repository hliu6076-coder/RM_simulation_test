#!/usr/bin/env bash
# Lightweight two-robot RMUL manual duel. ROS setup scripts require nounset off.
set -eo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
livox_setup="${LIVOX_WS_SETUP:-/home/wangxiaotao/ws_livox/install/setup.bash}"

for setup_file in \
  /opt/ros/humble/setup.bash \
  "${workspace_root}/install/setup.bash"; do
  if [[ ! -f "${setup_file}" ]]; then
    echo "Missing environment setup: ${setup_file}" >&2
    exit 1
  fi
done

cd "${workspace_root}"
source /opt/ros/humble/setup.bash
if [[ -f "${livox_setup}" ]]; then
  source "${livox_setup}"
fi
source "${workspace_root}/install/setup.bash"

# Keep the official match deterministic even when the login shell contains
# stale ROS discovery settings. Use RM_DUEL_* for an intentional override.
export ROS_DOMAIN_ID="${RM_DUEL_ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${RM_DUEL_LOCALHOST_ONLY:-1}"

duel_bind_host="${RM_DUEL_BIND_HOST:-0.0.0.0}"
duel_port="${RM_DUEL_PORT:-8765}"
duel_red_token="${RM_DUEL_RED_TOKEN:-red-test-2026}"
duel_blue_token="${RM_DUEL_BLUE_TOKEN:-blue-test-2026}"
duel_referee_token="${RM_DUEL_REFEREE_TOKEN:-referee-test-2026}"

echo "RMUL duel: ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, referee=${duel_bind_host}:${duel_port}"

exec ros2 launch rm_combat_gazebo rmul_duel.launch.py \
  gui:=true \
  autostart:=false \
  contact_damage:=true \
  lan_gateway:=true \
  lan_bind_host:="${duel_bind_host}" \
  lan_port:="${duel_port}" \
  lan_red_token:="${duel_red_token}" \
  lan_blue_token:="${duel_blue_token}" \
  lan_referee_token:="${duel_referee_token}" \
  "$@"
