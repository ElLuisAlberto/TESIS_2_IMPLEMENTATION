#!/usr/bin/env bash
# Run five correlated E13 saturation repetitions and preserve every artifact.

set -o pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT" || return 2 2>/dev/null || exit 2
source /opt/ros/humble/setup.bash
source install/setup.bash

EVID="${HOME}/Descargas/TESIS_AVANCE2_EVIDENCIAS"
STAMP="$(date +%Y%m%d_%H%M%S)"
DIR="${EVID}/pruebas/e13/lote_${STAMP}"
CSV="${DIR}/matriz_e13_lote.csv"
MANIFEST="${DIR}/manifest.csv"
mkdir -p "$DIR"
printf 'repetition,direction,recorder_ready,stimulus_rc,recorder_rc\n' > "$MANIFEST"

for repetition in $(seq 1 5); do
  if (( repetition % 2 == 1 )); then direction=1; else direction=-1; fi
  printf '\n===== E13 REPETICION %02d DIRECCION %+d =====\n' \
    "$repetition" "$direction"
  recorder_log="${DIR}/recorder_r${repetition}.log"
  stimulus_log="${DIR}/stimulus_r${repetition}.log"

  ros2 run thesis_validation scenario_recorder \
    E13 "$repetition" --duration 8 --output "$CSV" \
    > "$recorder_log" 2>&1 &
  recorder_pid=$!

  ready=0
  for _ in $(seq 1 48); do
    count="$(grep -c 'Suscripción activa:' "$recorder_log" 2>/dev/null || true)"
    if [ "$count" -ge 7 ]; then ready=1; break; fi
    sleep 0.25
  done

  stimulus_rc=2
  if [ "$ready" -eq 1 ]; then
    python3 tools/validation/e13_velocity_saturation_stimulus.py \
      --repetition "$repetition" --direction "$direction" \
      > "$stimulus_log" 2>&1
    stimulus_rc=$?
  else
    printf 'ERROR: registrador no preparado.\n' > "$stimulus_log"
  fi

  wait "$recorder_pid"
  recorder_rc=$?
  printf '%s,%s,%s,%s,%s\n' \
    "$repetition" "$direction" "$ready" \
    "$stimulus_rc" "$recorder_rc" >> "$MANIFEST"
  grep -E 'REGISTRO=|RESULTADO_ESCENARIO=' "$recorder_log" | tail -n 2
  sleep 0.5
done

printf '\n===== RESUMEN CSV =====\n'
python3 - "$CSV" "$MANIFEST" <<'PY'
import csv
import sys
from collections import Counter

with open(sys.argv[1], newline='', encoding='utf-8') as stream:
    rows = list(csv.DictReader(stream))
with open(sys.argv[2], newline='', encoding='utf-8') as stream:
    manifest = list(csv.DictReader(stream))
print(f'FILAS_E13={len(rows)}')
print('VEREDICTOS=' + str(dict(Counter(row['verdict'] for row in rows))))
for row in rows:
    print(
        f"R{row['repetition']}: observed={row['observed']}; "
        f"qdot={row['qdot_medida_max']}; "
        f"latency_ms={row['latency_end_to_end_ms']}; "
        f"saturation={row['saturation_confirmed']}; "
        f"velocity_checked={row['joint_velocity_checked']}; "
        f"velocity_violation={row['joint_velocity_violation']}; "
        f"verdict={row['verdict']}"
    )
print('EJECUCION_PROCESOS=' + str(manifest))
PY

printf '\nCSV=%s\nMANIFEST=%s\nDIRECTORIO=%s\n' "$CSV" "$MANIFEST" "$DIR"
