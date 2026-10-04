#!/usr/bin/env bash
set -Eeo pipefail
set +u

REPO="${REPO:-$HOME/Escritorio/TESIS_2_IMPLEMENTATION}"
DEFAULT_KINOVA_ROOT="$HOME/Escritorio/TESIS_2_DEPENDENCIES/kinova-ros"
DEFAULT_KINOVA_ROOT="$DEFAULT_KINOVA_ROOT/kinova_driver"
KINOVA_ROOT="${KINOVA_ROOT:-$DEFAULT_KINOVA_ROOT}"
REPORT="${REPORT:-$HOME/Descargas/RESULTADO_MICROMOVIMIENTO_KINOVA.txt}"
LAUNCH_LOG="${LAUNCH_LOG:-$HOME/Descargas/MICROMOVIMIENTO_KINOVA_LAUNCH.log}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-82}"

echo "============================================================"
echo "PRUEBA FÍSICA SUPERVISADA: J6 +2° Y RETORNO"
echo "Usa el pipeline completo y un obstáculo sintético lejano."
echo "Mantén libre el área y accesible la parada física del JACO2."
echo "============================================================"
read -r -p "Escribe MOVER J6 para continuar: " confirmation </dev/tty
if [[ "$confirmation" != "MOVER J6" ]]; then
  echo "PRUEBA_CANCELADA=SI"
  exit 2
fi

cd "$REPO"
source /opt/ros/humble/setup.bash
source install/local_setup.bash

export KINOVA_ROOT
export LD_LIBRARY_PATH="$KINOVA_ROOT/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"

mkdir -p "$(dirname "$REPORT")" "$(dirname "$LAUNCH_LOG")"
exec > >(tee "$REPORT") 2>&1

echo "FECHA=$(date --iso-8601=seconds)"
echo "REPO=$REPO"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "MOVIMIENTO_MAXIMO_SOLICITADO=2_GRADOS_J6"

existing_nodes="$(ros2 node list 2>/dev/null || true)"
if grep -Eq '(^|/)(jaco_hardware_adapter|safety_supervisor_node)$' \
    <<<"$existing_nodes"; then
  echo "ERROR=YA_EXISTE_UN_STACK_JACO_EN_ROS_DOMAIN_ID_$ROS_DOMAIN_ID"
  exit 3
fi

setsid ros2 launch thesis_hardware jaco_physical_system.launch.py \
  mock_hardware:=false \
  hardware_output_enabled:=true \
  use_demo_obstacle:=true \
  demo_obstacle_x:=2.0 \
  demo_obstacle_y:=0.0 \
  demo_obstacle_z:=0.65 \
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

echo "=== ESPERANDO EL STACK FÍSICO ==="
for _ in $(seq 1 150); do
  if ros2 service type /thesis/hardware/set_armed 2>/dev/null \
      | grep -q 'std_srvs/srv/SetBool'; then
    break
  fi
  sleep 0.1
done

ros2 service type /thesis/hardware/set_armed \
  | grep -q 'std_srvs/srv/SetBool'

echo "=== EJECUCIÓN DEL MICROMOVIMIENTO ==="
timeout 55 python3 \
  tools/validation/validate_kinova_physical_micro_motion.py

echo
echo "=== DIAGNÓSTICO FINAL ==="
timeout 3 ros2 topic echo --once /thesis/hardware/diagnostics || true
echo
echo "=== ESTADO FINAL DE ARMADO ==="
timeout 3 ros2 topic echo --once /thesis/hardware/armed || true
echo
echo "RESULTADO=PRUEBA_COMPLETADA"
echo "REPORTE=$REPORT"
echo "LAUNCH_LOG=$LAUNCH_LOG"
