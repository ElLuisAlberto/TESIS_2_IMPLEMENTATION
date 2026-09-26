#!/usr/bin/env bash
# Run five E14 repetitions with explicit recorder and supervisor discovery.

set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT" || exit 2
source /opt/ros/humble/setup.bash
source install/setup.bash

EVID="${HOME}/Descargas/TESIS_AVANCE2_EVIDENCIAS"
STAMP="$(date +%Y%m%d_%H%M%S)"
DIR="${EVID}/pruebas/e14/lote_reliable_${STAMP}"
CSV="${DIR}/matriz_e14_lote.csv"
MANIFEST="${DIR}/manifest.csv"
mkdir -p "$DIR"
printf 'repetition,recorder_ready,stimulus_rc,recorder_rc\n' > "$MANIFEST"

if ! ros2 node list | grep -qx '/safety_supervisor_node'; then
    printf '%s\n' 'ERROR: falta /safety_supervisor_node; no se inició el lote.'
    exit 2
fi

for repetition in $(seq 1 5); do
    recorder_log="${DIR}/recorder_r${repetition}.log"
    stimulus_log="${DIR}/stimulus_r${repetition}.log"
    printf '\n===== E14 REPETICION %02d =====\n' "$repetition"
    ros2 run thesis_validation scenario_recorder E14 "$repetition" \
        --duration 8 --output "$CSV" > "$recorder_log" 2>&1 &
    recorder_pid=$!

    ready=0
    for _ in $(seq 1 48); do
        count="$(grep -c 'Suscripción activa:' "$recorder_log" 2>/dev/null || true)"
        if [ "$count" -ge 7 ]; then ready=1; break; fi
        sleep 0.25
    done
    if [ "$ready" -eq 1 ]; then
        python3 tools/validation/e14_candidate_rejection_stimulus.py \
            --repetition "$repetition" > "$stimulus_log" 2>&1
        stimulus_rc=$?
    else
        printf '%s\n' 'ERROR: registrador no quedó listo; estímulo no publicado.' > "$stimulus_log"
        stimulus_rc=2
    fi
    wait "$recorder_pid"; recorder_rc=$?
    printf '%s,%s,%s,%s\n' "$repetition" "$ready" "$stimulus_rc" "$recorder_rc" >> "$MANIFEST"
    grep -E 'RESULTADO_ESCENARIO|REGISTRO=' "$recorder_log" | tail -n 2
    sleep 0.5
done

printf '\n===== RESUMEN CSV =====\n'
python3 - "$CSV" "$MANIFEST" <<'PY'
import csv
import sys
from collections import Counter
with open(sys.argv[1], newline='', encoding='utf-8') as f:
    rows = list(csv.DictReader(f))
with open(sys.argv[2], newline='', encoding='utf-8') as f:
    manifest = list(csv.DictReader(f))
print(f'FILAS_E14={len(rows)}')
print('VEREDICTOS=' + str(dict(Counter(r['verdict'] for r in rows))))
for r in rows:
    print('R{repetition}: observed={observed}; reason={reason_code}; controller={controller_status}; latency_ms={latency_end_to_end_ms}; verdict={verdict}'.format(**r))
print('EJECUCION_PROCESOS=' + str(manifest))
PY
printf '\nCSV=%s\nMANIFEST=%s\nDIRECTORIO=%s\n' "$CSV" "$MANIFEST" "$DIR"
