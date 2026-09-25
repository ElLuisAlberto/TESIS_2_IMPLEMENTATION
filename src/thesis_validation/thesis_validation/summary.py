"""Generate a deterministic summary from the experimental scenario CSV."""

import argparse
import csv
from collections import Counter
from pathlib import Path
from typing import Dict, List

from .contracts import SCENARIOS
from .evaluation import evaluate


SEGMENTS = frozenset({
    'base_to_shoulder', 'shoulder_to_upper_arm',
    'upper_arm_to_forearm', 'forearm_to_wrist_1',
    'wrist_1_to_wrist_2', 'wrist_2_to_tool',
})


def summarize(path: Path) -> int:
    """Print coverage and PASS/FAIL information for a matrix CSV."""
    with path.open(newline="", encoding="utf-8") as stream:
        rows: List[Dict[str, str]] = list(csv.DictReader(stream))
    counts = Counter(row.get("scenario_id", "") for row in rows)
    repetitions = {
        key: [row.get('repetition') for row in rows
              if row.get('scenario_id') == key]
        for key in SCENARIOS
    }
    failures = []
    for index, row in enumerate(rows, start=2):
        result = evaluate(row)
        if not result.passed:
            failures.append((
                index, row.get('scenario_id', ''), result.reasons,
            ))
        expected_verdict = (
            'PASS' if result.passed else 'FAIL:' + '|'.join(result.reasons)
        )
        if row.get('verdict') != expected_verdict:
            failures.append((
                index, row.get('scenario_id', ''),
                ('STORED_VERDICT_MISMATCH',),
            ))
    print(f"REGISTROS={len(rows)}")
    for scenario_id, spec in SCENARIOS.items():
        required = 10 if spec.requires_latency else spec.repetitions
        actual = counts[scenario_id]
        distinct = len(set(repetitions[scenario_id]))
        status = (
            'PASS' if actual >= required and distinct >= required else 'FAIL'
        )
        print(f"{scenario_id}: {actual}/{required} {status}")
    for line, scenario_id, reasons in failures:
        print(
            f"FALLO_LINEA={line} ESCENARIO={scenario_id} "
            f"RAZONES={','.join(reasons)}"
        )
    coverage_ok = all(
        counts[key] >= (10 if spec.requires_latency else spec.repetitions)
        and len(set(repetitions[key])) >= (
            10 if spec.requires_latency else spec.repetitions
        )
        for key, spec in SCENARIOS.items()
    )
    segment_counts = Counter(
        row.get('expected_segment') for row in rows
        if row.get('scenario_id') == 'E12' and evaluate(row).passed
    )
    for segment in sorted(SEGMENTS):
        print(f'E12_{segment}={segment_counts[segment]}/5')
    segments_ok = all(segment_counts[key] >= 5 for key in SEGMENTS)
    global_ok = coverage_ok and segments_ok and not failures
    print("RESULTADO_MATRIZ=" + ("PASS" if global_ok else "FAIL"))
    return 0 if global_ok else 1


def main() -> None:
    """Run the command-line summary utility."""
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    args = parser.parse_args()
    raise SystemExit(summarize(args.csv))
