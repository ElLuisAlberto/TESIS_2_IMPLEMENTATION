#!/usr/bin/env bash
# Validate fail-closed behavior when arm_controller rejects a trajectory.

set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT" || exit 2

source /opt/ros/humble/setup.bash
source install/setup.bash

EVID="${HOME}/Descargas/TESIS_AVANCE2_EVIDENCIAS"
STAMP="$(date +%Y%m%d_%H%M%S)"
DIR="${EVID}/pruebas/e15/lote_${STAMP}"
CSV="${DIR}/matriz_e15_lote.csv"
MANIFEST="${DIR}/manifest.csv"

mkdir -p "$DIR"

printf '%s\n' \
  'repetition,recorder_ready,deactivate_rc,stimulus_rc,reactivate_rc,recorder_rc' \
  > "$MANIFEST"

REC_PID=""

restore_controller() {
  # Idempotent best-effort recovery for interruptions and early failures.
  timeout 15 ros2 control switch_controllers \
    --activate arm_controller \
    --strict \
    >/dev/null 2>&1 || true
}

cleanup() {
  if [ -n "$REC_PID" ] && kill -0 "$REC_PID" 2>/dev/null; then
    kill "$REC_PID" 2>/dev/null || true
    wait "$REC_PID" 2>/dev/null || true
  fi

  restore_controller
}

trap cleanup EXIT INT TERM

printf '\n===== PRECONDICIONES =====\n'

for node in \
  /controller_manager \
  /proximity_monitor \
  /safety_supervisor_node \
  /simulation_command_adapter
do
  if ros2 node list | grep -qx "$node"; then
    printf 'NODO_OK=%s\n' "$node"
  else
    printf 'ERROR_NODO_AUSENTE=%s\n' "$node"
    exit 2
  fi
done

timeout 15 ros2 control list_controllers
CONTROLLER_SERVICE_RC=$?

printf 'CONTROLLER_MANAGER_RC=%s\n' "$CONTROLLER_SERVICE_RC"

if [ "$CONTROLLER_SERVICE_RC" -ne 0 ]; then
  printf 'ERROR: controller_manager no respondió.\n'
  exit 2
fi

for repetition in $(seq 1 5); do
  printf '\n===== E15 REPETICIÓN %02d =====\n' "$repetition"

  REC_LOG="${DIR}/recorder_r${repetition}.log"
  STIM_LOG="${DIR}/stimulus_r${repetition}.log"
  INJECT_LOG="${DIR}/controller_r${repetition}.log"

  ros2 run thesis_validation scenario_recorder \
    E15 "$repetition" \
    --duration 8 \
    --output "$CSV" \
    >"$REC_LOG" 2>&1 &

  REC_PID=$!

  READY=0
  for _ in $(seq 1 48); do
    COUNT="$(
      grep -c 'Suscripción activa:' "$REC_LOG" 2>/dev/null ||
      true
    )"

    if [ "$COUNT" -ge 7 ]; then
      READY=1
      break
    fi

    sleep 0.25
  done

  DEACTIVATE_RC=2
  STIMULUS_RC=2
  REACTIVATE_RC=2

  if [ "$READY" -eq 1 ]; then
    (
      timeout 15 ros2 control switch_controllers \
        --deactivate arm_controller \
        --strict

      DEACTIVATE_RC=$?
      printf 'DESACTIVAR_RC=%s\n' "$DEACTIVATE_RC"
      exit "$DEACTIVATE_RC"
    ) >"$INJECT_LOG" 2>&1

    DEACTIVATE_RC=$?

    if [ "$DEACTIVATE_RC" -eq 0 ]; then
      python3 tools/validation/candidate_safe_stimulus.py \
        --scenario E15 \
        --repetition "$repetition" \
        --repeats 1 \
        --rate-hz 10.0 \
        >"$STIM_LOG" 2>&1

      STIMULUS_RC=$?

      # Allow the adapter rejection and supervisor STOP to propagate.
      sleep 2
    else
      printf 'ERROR: no se pudo desactivar arm_controller.\n' \
        >"$STIM_LOG"
    fi

    timeout 15 ros2 control switch_controllers \
      --activate arm_controller \
      --strict \
      >>"$INJECT_LOG" 2>&1

    REACTIVATE_RC=$?

    printf 'REACTIVAR_RC=%s\n' "$REACTIVATE_RC" \
      >>"$INJECT_LOG"
  else
    printf 'ERROR: registrador no preparado.\n' >"$STIM_LOG"
    printf 'ERROR: inyección omitida.\n' >"$INJECT_LOG"
  fi

  wait "$REC_PID"
  RECORDER_RC=$?
  REC_PID=""

  printf '%s,%s,%s,%s,%s,%s\n' \
    "$repetition" \
    "$READY" \
    "$DEACTIVATE_RC" \
    "$STIMULUS_RC" \
    "$REACTIVATE_RC" \
    "$RECORDER_RC" \
    >>"$MANIFEST"

  grep -E \
    'TEMAS_SUSCRITOS|REGISTRO=|RESULTADO_ESCENARIO' \
    "$REC_LOG" |
  tail -n 3

  sleep 0.5
done

printf '\n===== RESUMEN CSV =====\n'

python3 - "$CSV" "$MANIFEST" <<'PY_SUMMARY'
import csv
import sys
from collections import Counter

csv_path = sys.argv[1]
manifest_path = sys.argv[2]

with open(csv_path, newline="", encoding="utf-8") as stream:
    rows = list(csv.DictReader(stream))

with open(manifest_path, newline="", encoding="utf-8") as stream:
    manifest = list(csv.DictReader(stream))

verdicts = Counter(row["verdict"] for row in rows)

print(f"FILAS_E15={len(rows)}")
print("VEREDICTOS=" + str(dict(verdicts)))

for row in rows:
    print(
        f'R{row["repetition"]}: '
        f'observed={row["observed"]}; '
        f'scale={row["speed_scale"]}; '
        f'reason={row["reason_code"]}; '
        f'controller={row["controller_status"]}; '
        f'latency_ms={row["latency_end_to_end_ms"]}; '
        f'verdict={row["verdict"]}'
    )

print("EJECUCION_PROCESOS=" + str(manifest))

processes_ok = all(
    row["recorder_ready"] == "1"
    and row["deactivate_rc"] == "0"
    and row["stimulus_rc"] == "0"
    and row["reactivate_rc"] == "0"
    and row["recorder_rc"] == "0"
    for row in manifest
)

evidence_ok = (
    len(rows) == 5
    and all(row["scenario_id"] == "E15" for row in rows)
    and all(row["verdict"] == "PASS" for row in rows)
    and all(row["observed"] == "STOP" for row in rows)
    and all(float(row["speed_scale"]) == 0.0 for row in rows)
    and all(
        row["controller_status"] in {"FAILED", "REJECTED"}
        for row in rows
    )
)

if not processes_ok:
    print("LOTE_E15=FAIL:PROCESOS")
    raise SystemExit(1)

if not evidence_ok:
    print("LOTE_E15=FAIL:EVIDENCIA")
    raise SystemExit(1)

print("LOTE_E15=PASS")
PY_SUMMARY

SUMMARY_RC=$?

printf '\nCSV=%s\n' "$CSV"
printf 'MANIFEST=%s\n' "$MANIFEST"
printf 'DIRECTORIO=%s\n' "$DIR"
printf '\n===== CONTROLADORES FINALES =====\n'
timeout 15 ros2 control list_controllers
FINAL_CONTROLLER_RC=$?

if [ "$SUMMARY_RC" -ne 0 ]; then
  exit "$SUMMARY_RC"
fi

exit "$FINAL_CONTROLLER_RC"
