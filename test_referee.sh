#!/usr/bin/env bash
# Unit tests plus an isolated LAN referee smoke test.
set -eo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
test_domain="${RM_REFEREE_TEST_DOMAIN_ID:-143}"
test_port="${RM_REFEREE_TEST_PORT:-18765}"
test_red_token="${RM_REFEREE_TEST_RED_TOKEN:-red-test-2026}"
test_blue_token="${RM_REFEREE_TEST_BLUE_TOKEN:-blue-test-2026}"
test_referee_token="${RM_REFEREE_TEST_REFEREE_TOKEN:-referee-test-2026}"
test_tmp="$(mktemp -d -t rm-referee-test.XXXXXX)"
launch_pid=""

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
  if [[ ${exit_code} -ne 0 ]]; then
    echo "Referee launch log: ${test_tmp}/launch.log" >&2
    tail -n 80 "${test_tmp}/launch.log" >&2 || true
  else
    rm -r "${test_tmp}"
  fi
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

for setup_file in /opt/ros/humble/setup.bash "${workspace_root}/install/setup.bash"; do
  if [[ ! -f "${setup_file}" ]]; then
    echo "Missing environment setup: ${setup_file}" >&2
    echo "Build the combat packages before running this test." >&2
    exit 1
  fi
done

cd "${workspace_root}"
source /opt/ros/humble/setup.bash
source "${workspace_root}/install/setup.bash"
export ROS_DOMAIN_ID="${test_domain}"
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="${test_tmp}/ros-log"

echo "[1/3] Running referee unit tests"
python3 -m pytest -q src/rm_combat/rm_referee/test

echo "[2/3] Starting isolated LAN referee on 127.0.0.1:${test_port}"
setsid ros2 launch rm_referee lan_referee_demo.launch.py \
  bind_host:=127.0.0.1 \
  port:="${test_port}" \
  red_token:="${test_red_token}" \
  blue_token:="${test_blue_token}" \
  referee_token:="${test_referee_token}" \
  state_broadcast_hz:=10.0 >"${test_tmp}/launch.log" 2>&1 &
launch_pid=$!

client="${workspace_root}/install/rm_referee/lib/rm_referee/referee_lan_client"
ready=0
for _ in $(seq 1 50); do
  if "${client}" --host 127.0.0.1 --port "${test_port}" \
      --role red --name smoke-red --token "${test_red_token}" \
      --watch-seconds 0 >"${test_tmp}/ready.log" 2>&1; then
    ready=1
    break
  fi
  sleep 0.1
done
if [[ ${ready} -ne 1 ]]; then
  echo "LAN referee did not become ready" >&2
  exit 1
fi

if "${client}" --host 127.0.0.1 --port "${test_port}" \
    --role referee --name rejected-referee --token wrong-token \
    --command start >"${test_tmp}/bad-token.log" 2>&1; then
  echo "Invalid referee token was unexpectedly accepted" >&2
  exit 1
fi

echo "[3/3] Checking authenticated start/pause/resume/reset commands"
for command in start pause resume reset; do
  "${client}" --host 127.0.0.1 --port "${test_port}" \
    --role referee --name smoke-referee --token "${test_referee_token}" \
    --command "${command}" >"${test_tmp}/${command}.log"
  rg -q '"success": true' "${test_tmp}/${command}.log"
  echo "  ${command}: OK"
done

echo "Referee tests passed (domain=${test_domain}, port=${test_port})."
