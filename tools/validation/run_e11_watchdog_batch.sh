#!/usr/bin/env bash
# Run ten reproducible E11 watchdog repetitions and preserve every outcome.

set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT" || exit 2

source /opt/ros/humble/setup.bash
source install/setup.bash

EVID="${HOME}/Descargas/TESIS_AVANCE2_EVIDENCIAS"
STAMP="$(date +%Y%m%d_%H%M%S)"
BATCH_DIR="${EVID}/pruebas/e11/lote_${STAMP}"
CSV="${BATCH_DIR}/matriz_e11_lote.csv"
MANIFEST="${BATCH_DIR}/manifest.csv"
TRACE="${BATCH_DIR}/pipeline_timing.yaml"

mkdir -p "$BATCH_DIR"
printf 'repetition,stimulus_rc,recorder_rc\n' > "$MANIFEST"

if ! ros2 node list | grep -qx '/safety_supervisor_node'; then
    printf '%s\n' 'ERROR: falta /safety_supervisor_node; no se inició el lote.'
    exit 2
fi
if ! ros2 node list | grep -qx '/simulation_command_adapter'; then
    printf '%s\n' 'ERROR: falta /simulation_command_adapter; no se inició el lote.'
    exit 2
fi

printf 'DIRECTORIO_EVIDENCIA=%s\n' "$BATCH_DIR"
timeout 110 ros2 topic echo /thesis/pipeline_timing > "$TRACE" 2>&1 &
TRACE_PID=$!

for repetition in $(seq 1 10); do
    REC_LOG="${BATCH_DIR}/recorder_r${repetition}.log"
    STIM_LOG="${BATCH_DIR}/stimulus_r${repetition}.log"

    printf '\n===== E11 REPETICION %02d =====\n' "$repetition"
    ros2 run thesis_validation scenario_recorder \
        E11 "$repetition" \
        --duration 8 \
        --output "$CSV" \
        > "$REC_LOG" 2>&1 &
    REC_PID=$!

    sleep 3

    python3 tools/validation/e11_watchdog_stimulus.py \
        --repetition "$repetition" \
        --joint-index 0 \
        --delta-rad 0.03 \
        --stream-sec 1.0 \
        --rate-hz 20.0 \
        > "$STIM_LOG" 2>&1
    STIM_RC=$?

    wait "$REC_PID"
    REC_RC=$?
    printf '%s,%s,%s\n' "$repetition" "$STIM_RC" "$REC_RC" >> "$MANIFEST"

    tail -n 4 "$STIM_LOG"
    grep -E 'RESULTADO_ESCENARIO|REGISTRO=' "$REC_LOG" | tail -n 2
    sleep 0.5
done

if kill -0 "$TRACE_PID" 2>/dev/null; then
    kill "$TRACE_PID"
fi
wait "$TRACE_PID" 2>/dev/null

printf '\n===== RESUMEN CSV =====\n'
python3 - "$CSV" "$MANIFEST" <<'PY'
import csv
import sys
from collections import Counter

csv_path, manifest_path = sys.argv[1:]
with open(csv_path, newline='', encoding='utf-8') as stream:
    rows = list(csv.DictReader(stream))
with open(manifest_path, newline='', encoding='utf-8') as stream:
    manifest = list(csv.DictReader(stream))

print(f'FILAS_E11={len(rows)}')
print('VEREDICTOS=' + str(dict(Counter(row['verdict'] for row in rows))))
for row in rows:
    print(
        'R{repetition}: verdict={verdict}; hold_ms={jog_hold_age_ms}; '
        'latency_ms={latency_end_to_end_ms}; command_id={command_id}'.format(
            **row
        )
    )
print('EJECUCION_PROCESOS=' + str(manifest))
PY

printf '\nCSV=%s\nMANIFEST=%s\nTRAZA=%s\n' \
    "$CSV" "$MANIFEST" "$TRACE"
