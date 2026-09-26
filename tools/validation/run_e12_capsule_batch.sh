#!/usr/bin/env bash
# Validate selected E12 capsules with one accepted candidate per placement.

set -o pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT" || exit 2
source /opt/ros/humble/setup.bash
source install/setup.bash

EVID="${HOME}/Descargas/TESIS_AVANCE2_EVIDENCIAS"
STAMP="$(date +%Y%m%d_%H%M%S)"
DIR="${EVID}/pruebas/e12/lote_v2_${STAMP}"
CSV="${DIR}/matriz_e12_lote.csv"
MANIFEST="${DIR}/manifest.csv"
mkdir -p "$DIR"
printf 'segment,repetition,pose_rc,recorder_ready,stimulus_rc,recorder_rc\n' > "$MANIFEST"

DEFAULT_SEGMENTS='base_to_shoulder shoulder_to_upper_arm upper_arm_to_forearm forearm_to_wrist_1 wrist_1_to_wrist_2 wrist_2_to_tool'
SEGMENT_LIST="${SEGMENTS:-$DEFAULT_SEGMENTS}"
restore_obstacle() {
  ign service --service /world/jaco_world/set_pose \
    --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 3000 \
    --req 'name: "safety_obstacle" position { x: 0.60 y: 0.0 z: 0.65 } orientation { w: 1.0 }' \
    >/dev/null 2>&1
}
trap restore_obstacle EXIT INT TERM

for segment in $SEGMENT_LIST; do
  for repetition in $(seq 1 5); do
    printf '\n===== E12 %s R%02d =====\n' "$segment" "$repetition"
    pose_log="${DIR}/pose_${segment}_r${repetition}.log"
    recorder_log="${DIR}/recorder_${segment}_r${repetition}.log"
    stimulus_log="${DIR}/stimulus_${segment}_r${repetition}.log"
    python3 tools/validation/e12_capsule_pose.py --segment "$segment" > "$pose_log" 2>&1
    pose_rc=$?; ready=0; stimulus_rc=2; recorder_rc=0
    if [ "$pose_rc" -eq 0 ]; then
      ros2 run thesis_validation scenario_recorder E12 "$repetition" \
        --expected-segment "$segment" --duration 10 --output "$CSV" > "$recorder_log" 2>&1 &
      recorder_pid=$!
      for _ in $(seq 1 48); do
        count="$(grep -c 'Suscripción activa:' "$recorder_log" 2>/dev/null || true)"
        if [ "$count" -ge 7 ]; then ready=1; break; fi
        sleep 0.25
      done
      if [ "$ready" -eq 1 ]; then
        python3 tools/validation/candidate_safe_stimulus.py \
          --scenario E12 --repetition "$repetition" --repeats 1 > "$stimulus_log" 2>&1
        stimulus_rc=$?
      else
        echo 'ERROR: registrador no quedó listo.' > "$stimulus_log"
      fi
      wait "$recorder_pid"; recorder_rc=$?
      # The accepted 8 s simulation goal must finish before the next case.
      sleep 3
    else
      echo 'ERROR: la pose de cápsula no se confirmó.' > "$recorder_log"
      echo 'ERROR: estímulo omitido.' > "$stimulus_log"
    fi
    printf '%s,%s,%s,%s,%s,%s\n' "$segment" "$repetition" "$pose_rc" "$ready" "$stimulus_rc" "$recorder_rc" >> "$MANIFEST"
    grep -E 'RESULTADO_ESCENARIO|REGISTRO=' "$recorder_log" 2>/dev/null | tail -n 2
  done
done

printf '\n===== RESUMEN CSV =====\n'
python3 - "$CSV" "$MANIFEST" <<'PY'
import csv, sys
from collections import Counter
with open(sys.argv[1], newline='', encoding='utf-8') as f: rows = list(csv.DictReader(f))
with open(sys.argv[2], newline='', encoding='utf-8') as f: manifest = list(csv.DictReader(f))
print(f'FILAS_E12={len(rows)}')
print('VEREDICTOS=' + str(dict(Counter(r['verdict'] for r in rows))))
for segment in sorted({r['expected_segment'] for r in rows}):
    group = [r for r in rows if r['expected_segment'] == segment]
    print(f'{segment}: filas={len(group)}; veredictos={dict(Counter(r["verdict"] for r in group))}')
print('EJECUCION_PROCESOS=' + str(manifest))
PY
printf '\nCSV=%s\nMANIFEST=%s\nDIRECTORIO=%s\n' "$CSV" "$MANIFEST" "$DIR"
