#!/usr/bin/env bash
# Verify READY -> WAITING -> READY during the E09/E10 fail-safe tests.

set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT" || exit 2
source /opt/ros/humble/setup.bash
source install/setup.bash

STAMP="$(date +%Y%m%d_%H%M%S)"
EVID="${HOME}/Descargas/TESIS_AVANCE2_EVIDENCIAS/pruebas/readiness/lote_${STAMP}"
mkdir -p "$EVID"
RECORDER_CSV="${EVID}/readiness_transitions.csv"
RECORDER_LOG="${EVID}/readiness_recorder.log"
BATCH_LOG="${EVID}/e09_e10_batch.log"
RECORDER_PID=""

cleanup() {
    if [ -n "$RECORDER_PID" ] && kill -0 "$RECORDER_PID" 2>/dev/null; then
        kill -INT "$RECORDER_PID" 2>/dev/null || true
        wait "$RECORDER_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

printf 'DIRECTORIO_EVIDENCIA=%s\n' "$EVID"
printf '\n===== ESPERANDO AL STACK =====\n'
# Refresh only the ROS CLI discovery daemon; this does not restart ROS nodes.
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

REQUIRED_NODES=(
    /system_readiness
    /safety_supervisor_node
    /proximity_monitor
    /simulation_command_adapter
    /controller_manager
)
START_WAIT=$SECONDS
READY_STATUS=""
NODES=""
STACK_READY=0

while [ $((SECONDS - START_WAIT)) -lt 60 ]; do
    NODES="$(ros2 node list 2>/dev/null || true)"
    ALL_NODES_PRESENT=1
    for node in "${REQUIRED_NODES[@]}"; do
        if ! printf '%s\n' "$NODES" | grep -qx "$node"; then
            ALL_NODES_PRESENT=0
            break
        fi
    done

    if [ "$ALL_NODES_PRESENT" -eq 1 ]; then
        READY_STATUS="$(timeout 2 ros2 topic echo --once \
            /thesis/system_readiness 2>/dev/null || true)"
        if printf '%s\n' "$READY_STATUS" | grep -q 'data: READY'; then
            STACK_READY=1
            break
        fi
    fi
    sleep 1
done

if [ "$STACK_READY" -ne 1 ]; then
    printf 'ERROR: el stack no llegó a READY en 60 s.\n'
    printf 'NODOS_DETECTADOS:\n%s\n' "$NODES"
    printf 'ULTIMO_ESTADO_READINESS:\n%s\n' "$READY_STATUS"
    printf 'No se inyectaron fallos. Deja el launch activo y revisa su terminal.\n'
    exit 2
fi

for node in "${REQUIRED_NODES[@]}"; do
    printf 'NODO_OK=%s\n' "$node"
done
printf 'READINESS_INICIAL=READY\n'
printf 'STACK_READY_EN=%ss\n' "$((SECONDS - START_WAIT))"

python3 tools/validation/readiness_transition_recorder.py \
    --duration-sec 150 \
    --output "$RECORDER_CSV" \
    >"$RECORDER_LOG" 2>&1 &
RECORDER_PID=$!

RECORDER_READY=0
for _ in $(seq 1 40); do
    if grep -q 'READINESS_RECORDER_READY=1' "$RECORDER_LOG"; then
        RECORDER_READY=1
        break
    fi
    if ! kill -0 "$RECORDER_PID" 2>/dev/null; then
        break
    fi
    sleep 0.25
done
if [ "$RECORDER_READY" -ne 1 ]; then
    printf 'ERROR: no inició el registrador de readiness. Revisa %s\n' "$RECORDER_LOG"
    exit 2
fi

printf '\n===== EJECUTANDO PRUEBAS E09/E10 =====\n'
SCENARIOS="${SCENARIOS:-E09 E10}" \
    REPETITIONS="${REPETITIONS:-5}" \
    bash tools/validation/run_e09_e10_failsafe_batch.sh >"$BATCH_LOG" 2>&1
BATCH_RC=$?
cat "$BATCH_LOG"

cleanup
RECORDER_PID=""

CSV_PATH="$(sed -n 's/^CSV=//p' "$BATCH_LOG" | tail -n 1)"
MANIFEST_PATH="$(sed -n 's/^MANIFEST=//p' "$BATCH_LOG" | tail -n 1)"
if [ -z "$CSV_PATH" ] || [ ! -f "$CSV_PATH" ]; then
    printf 'ERROR: no se encontró la matriz CSV de E09/E10.\n'
    exit 2
fi

printf '\n===== VERIFICACIÓN COMBINADA =====\n'
python3 - "$CSV_PATH" "$MANIFEST_PATH" "$RECORDER_CSV" <<'PY'
import csv
import os
import sys
from collections import Counter
from pathlib import Path

batch_csv, manifest_csv, readiness_csv = sys.argv[1:]
with open(batch_csv, newline='', encoding='utf-8') as stream:
    rows = list(csv.DictReader(stream))
with open(manifest_csv, newline='', encoding='utf-8') as stream:
    manifest = list(csv.DictReader(stream))
with open(readiness_csv, newline='', encoding='utf-8') as stream:
    transitions = list(csv.DictReader(stream))

scenarios = {row['scenario'] for row in manifest}
waiting_e09 = [
    row for row in transitions
    if row['readiness_status'].startswith('WAITING')
    and 'estado articular' in row['readiness_status']
]
waiting_e10 = [
    row for row in transitions
    if row['readiness_status'].startswith('WAITING')
    and 'cálculo de proximidad' in row['readiness_status']
]
ready_count = sum(row['readiness_status'] == 'READY' for row in transitions)
print(f'FILAS_FALLSAFE={len(rows)}')
print('FALLOS_FALLSAFE=' + str(dict(Counter(r['verdict'] for r in rows))))
print(f'ESPERAS_POR_JOINT_STATES={len(waiting_e09)}')
print(f'ESPERAS_POR_PROXIMIDAD={len(waiting_e10)}')
print(f'RETORNOS_A_READY={ready_count}')
print(f'FILAS_MANIFEST={len(manifest)}')
for row in rows:
    print(
        'E{scenario_id} R{repetition}: observed={observed}; '
        'reason={reason_code}; latency_ms={latency_end_to_end_ms}; '
        'verdict={verdict}'.format(**row)
    )

repetitions = int(os.environ.get('REPETITIONS', '5'))
expected_count = repetitions * len(scenarios)
expected_rows = len(manifest) == expected_count and len(rows) == expected_count
manifest_ok = all(
    row['recorder_ready'] == '1'
    and row['injection_rc'] == '0'
    and row['readiness_wait_rc'] == '0'
    and row['stimulus_rc'] == '0'
    and row['restore_rc'] == '0'
    and row['recorder_rc'] == '0'
    for row in manifest
)
fail_safe_ok = all(
    row['verdict'] == 'PASS'
    and row['observed'] == 'STOP'
    and float(row['speed_scale']) == 0.0
    for row in rows
)
readiness_ok = (
    bool(scenarios)
    and ('E09' not in scenarios or len(waiting_e09) >= repetitions)
    and ('E10' not in scenarios or len(waiting_e10) >= repetitions)
    and ready_count >= expected_count + 1
)

if not expected_rows or not manifest_ok or not fail_safe_ok or not readiness_ok:
    print('RESULTADO_READINESS=FAIL')
    raise SystemExit(1)
print('RESULTADO_READINESS=PASS')
PY
VERIFY_RC=$?

printf '\nARCHIVO_READINESS=%s\n' "$RECORDER_CSV"
printf 'LOG_READINESS=%s\n' "$RECORDER_LOG"
printf 'LOG_E09_E10=%s\n' "$BATCH_LOG"
printf 'CSV_E09_E10=%s\n' "$CSV_PATH"
printf 'MANIFEST_E09_E10=%s\n' "$MANIFEST_PATH"

if [ "$BATCH_RC" -ne 0 ]; then
    exit "$BATCH_RC"
fi
exit "$VERIFY_RC"
