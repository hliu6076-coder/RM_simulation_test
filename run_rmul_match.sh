#!/usr/bin/env bash
# Single-host RMUL match entry. ROS 2 Humble setup scripts require nounset off.
set -eo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

match_bind_host="${RM_MATCH_BIND_HOST:-0.0.0.0}"
match_port="${RM_MATCH_PORT:-8765}"
match_red_token="${RM_MATCH_RED_TOKEN:-red-test-2026}"
match_blue_token="${RM_MATCH_BLUE_TOKEN:-blue-test-2026}"
match_referee_token="${RM_MATCH_REFEREE_TOKEN:-referee-test-2026}"

exec "${workspace_root}/run_rmul_nav.sh" \
  combat:=True \
  combat_autostart:=False \
  combat_lan:=True \
  combat_lan_bind_host:="${match_bind_host}" \
  combat_lan_port:="${match_port}" \
  combat_red_token:="${match_red_token}" \
  combat_blue_token:="${match_blue_token}" \
  combat_referee_token:="${match_referee_token}" \
  "$@"
