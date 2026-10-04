#!/usr/bin/env bash
set -Eeo pipefail
set +u

REPO="${REPO:-$HOME/Escritorio/TESIS_2_IMPLEMENTATION}"
REPORT="${REPORT:-$HOME/Descargas/VALIDACION_PUENTE_KINOVA_MOCK.txt}"
LAUNCH_LOG="${LAUNCH_LOG:-$HOME/Descargas/PUENTE_KINOVA_MOCK_LAUNCH.log}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

cd "$REPO"
source /opt/ros/humble/setup.bash
source install/setup.bash

existing_nodes="$(ros2 node list 2>/dev/null || true)"
if grep -Eq '(^|/)(jaco_hardware_adapter|safety_supervisor_node)$' \
    <<<"$existing_nodes"; then
  echo "ERROR=YA_EXISTE_UN_STACK_JACO_EN_ROS_DOMAIN_ID_$ROS_DOMAIN_ID"
  exit 3
fi

setsid ros2 launch thesis_hardware jaco_physical_system.launch.py \
  mock_hardware:=true \
  hardware_output_enabled:=true \
  use_demo_obstacle:=true \
  start_gui:=false \
  start_rviz:=false \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  timeout 3 ros2 service call \
    /thesis/hardware/set_armed \
    std_srvs/srv/SetBool \
    "{data: false}" \
    >/dev/null 2>&1 || true
  kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
  for _ in $(seq 1 50); do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      wait "$LAUNCH_PID" 2>/dev/null || true
      return
    fi
    sleep 0.1
  done
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  sleep 0.5
  kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 80); do
  if ros2 service type /thesis/hardware/set_armed \
      2>/dev/null | grep -q 'std_srvs/srv/SetBool'; then
    break
  fi
  sleep 0.1
done

{
  echo "FECHA=$(date --iso-8601=seconds)"
  echo "HEAD=$(git rev-parse HEAD 2>/dev/null || echo SIN_COMMIT)"
  timeout 35 python3 \
    tools/validation/validate_hardware_bridge_mock.py
  echo
  echo "=== NODOS ==="
  ros2 node list
  echo
  echo "=== FRECUENCIA JOINT_STATES ==="
  timeout 4 ros2 topic hz /joint_states || true
} | tee "$REPORT"

echo "REPORT=$REPORT"
echo "LAUNCH_LOG=$LAUNCH_LOG"
