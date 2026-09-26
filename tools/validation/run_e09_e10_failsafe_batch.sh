#!/usr/bin/env bash
# Run the E09/E10 fail-safe matrix and restore each temporary fault injection.

set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT" || exit 2
source /opt/ros/humble/setup.bash
source install/setup.bash

EVID="${HOME}/Descargas/TESIS_AVANCE2_EVIDENCIAS"
STAMP="$(date +%Y%m%d_%H%M%S)"
DIR="${EVID}/pruebas/e09_e10/lote_${STAMP}"
CSV="${DIR}/matriz_e09_e10_lote.csv"
MANIFEST="${DIR}/manifest.csv"
mkdir -p "$DIR"
printf 'scenario,repetition,recorder_ready,injection_rc,stimulus_rc,restore_rc,recorder_rc\n' > "$MANIFEST"

if ! ros2 node list | grep -qx '/safety_supervisor_node'; then
    printf '%s\n' 'ERROR: falta /safety_supervisor_node; no se inició el lote.'
    exit 2
fi
if ! ros2 node list | grep -qx '/proximity_monitor'; then
    printf '%s\n' 'ERROR: falta /proximity_monitor; no se inició el lote.'
    exit 2
fi

restore_state_broadcaster() {
    ros2 control switch_controllers --activate joint_state_broadcaster \
        --strict >/dev/null 2>&1
}
restore_proximity_publication() {
    ros2 param set /proximity_monitor status_publishing_enabled true \
        >/dev/null 2>&1
}

run_one() {
    local scenario="$1" repetition="$2"
    local recorder_log="${DIR}/recorder_${scenario}_r${repetition}.log"
    local stimulus_log="${DIR}/stimulus_${scenario}_r${repetition}.log"
    local ready=0 injection_rc=0 stimulus_rc=2 restore_rc=0 recorder_rc=0
    printf '\n===== %s REPETICION %02d =====\n' "$scenario" "$repetition"

    ros2 run thesis_validation scenario_recorder "$scenario" "$repetition" \
        --duration 10 --output "$CSV" > "$recorder_log" 2>&1 &
    local recorder_pid=$!
    for _ in $(seq 1 48); do
        local count
        count="$(grep -c 'Suscripción activa:' "$recorder_log" 2>/dev/null || true)"
        if [ "$count" -ge 7 ]; then ready=1; break; fi
        sleep 0.25
    done

    if [ "$ready" -eq 1 ] && [ "$scenario" = 'E09' ]; then
        ros2 control switch_controllers --deactivate joint_state_broadcaster \
            --strict > "${DIR}/inject_${scenario}_r${repetition}.log" 2>&1
        injection_rc=$?
        sleep 1.2
    elif [ "$ready" -eq 1 ]; then
        ros2 param set /proximity_monitor status_publishing_enabled false \
            > "${DIR}/inject_${scenario}_r${repetition}.log" 2>&1
        injection_rc=$?
        sleep 1.2
    else
        injection_rc=2
        printf '%s\n' 'ERROR: registrador no quedó listo.' > "${DIR}/inject_${scenario}_r${repetition}.log"
    fi

    if [ "$ready" -eq 1 ] && [ "$injection_rc" -eq 0 ]; then
        python3 tools/validation/candidate_safe_stimulus.py \
            --scenario "$scenario" --repetition "$repetition" \
            > "$stimulus_log" 2>&1
        stimulus_rc=$?
    else
        printf '%s\n' 'ERROR: no se publicó estímulo por preparación fallida.' > "$stimulus_log"
    fi

    if [ "$scenario" = 'E09' ]; then
        restore_state_broadcaster; restore_rc=$?
    else
        restore_proximity_publication; restore_rc=$?
    fi
    wait "$recorder_pid"; recorder_rc=$?
    printf '%s,%s,%s,%s,%s,%s,%s\n' \
        "$scenario" "$repetition" "$ready" "$injection_rc" "$stimulus_rc" \
        "$restore_rc" "$recorder_rc" >> "$MANIFEST"
    grep -E 'RESULTADO_ESCENARIO|REGISTRO=' "$recorder_log" | tail -n 2
    sleep 1
}

trap 'restore_state_broadcaster; restore_proximity_publication' EXIT INT TERM
for scenario in ${SCENARIOS:-E09 E10}; do
    for repetition in $(seq 1 5); do run_one "$scenario" "$repetition"; done
done
trap - EXIT INT TERM

printf '\n===== RESUMEN CSV =====\n'
python3 - "$CSV" "$MANIFEST" <<'PY'
import csv
import sys
from collections import Counter
with open(sys.argv[1], newline='', encoding='utf-8') as f:
    rows = list(csv.DictReader(f))
with open(sys.argv[2], newline='', encoding='utf-8') as f:
    manifest = list(csv.DictReader(f))
print(f'FILAS_E09_E10={len(rows)}')
print('VEREDICTOS=' + str(dict(Counter(r['verdict'] for r in rows))))
for row in rows:
    print('R{scenario_id}.{repetition}: observed={observed}; reason={reason_code}; scale={speed_scale}; latency_ms={latency_end_to_end_ms}; verdict={verdict}'.format(**row))
print('EJECUCION_PROCESOS=' + str(manifest))
PY
printf '\nCSV=%s\nMANIFEST=%s\nDIRECTORIO=%s\n' "$CSV" "$MANIFEST" "$DIR"
