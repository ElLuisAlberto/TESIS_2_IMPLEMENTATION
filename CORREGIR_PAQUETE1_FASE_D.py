#!/usr/bin/env python3
"""Apply the minimal corrections found by Package 1 Phase D tests."""

from __future__ import annotations

import argparse
import hashlib
import shutil
import sys
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict


EXPECTED_HASHES: Dict[str, str] = {
    'src/thesis_core/thesis_core/safety_supervisor_node.py':
        'ae838092ec6a6cff550b813ca40ee970ca01a69c9ca77b0e10410338280d4f74',
    'src/thesis_core/thesis_core/execution_reference.py':
        '7bfc3b1ba6bbd675d34b0554c0887775b9db2beb08cefea922a9a5cee5c9dde7',
    'src/thesis_core/test/test_execution_reference.py':
        'a582276455c18143a3c5af90034bbeadfb932d25c352367f2ef862c24d6e4382',
    'src/thesis_ui/thesis_ui/joint_control_gui.py':
        '2ae59c6f432cc3f36511dd80b111d02a3e898fe3b23820d0fc2babe6f8c0a120',
    'src/thesis_ui/test/test_gui_state.py':
        'b944b72a4107410c982e4d14d1fb5a8c41cc2741536ed0e965627143e64275bd',
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f'{label}: expected one match, found {count}')
    return text.replace(old, new, 1)


def transform_safety(text: str) -> str:
    replacements = (
        (
            "                f'REJECTED {msg.command_id}: STOP proximity, "
            "clearance={clearance:.3f} m, segment={segment}',\n",
            "                f'REJECTED {msg.command_id}: STOP proximity, '\n"
            "                f'clearance={clearance:.3f} m, '\n"
            "                f'segment={segment}',\n",
            'STOP diagnostic wrapping',
        ),
        (
            "                    f'REJECTED {msg.command_id}: REDUCTION "
            "requested but duration cannot be increased safely',\n",
            "                    f'REJECTED {msg.command_id}: REDUCTION '\n"
            "                    'requested but duration cannot be '\n"
            "                    'increased safely',\n",
            'REDUCTION diagnostic wrapping',
        ),
        (
            "                    f'REJECTED {msg.command_id}: {joint_name} "
            "requested velocity {requested_velocity:.4f} rad/s exceeds "
            "{allowed_velocity:.4f} rad/s',\n",
            "                    f'REJECTED {msg.command_id}: {joint_name} '\n"
            "                    f'requested velocity '\n"
            "                    f'{requested_velocity:.4f} rad/s exceeds '\n"
            "                    f'{allowed_velocity:.4f} rad/s',\n",
            'velocity diagnostic wrapping',
        ),
        (
            "                    f'REJECTED {msg.command_id}: {joint_name}="
            "{position:.4f} rad outside [{lower:.4f}, {upper:.4f}]',\n",
            "                    f'REJECTED {msg.command_id}: {joint_name}='\n"
            "                    f'{position:.4f} rad outside '\n"
            "                    f'[{lower:.4f}, {upper:.4f}]',\n",
            'position diagnostic wrapping',
        ),
        (
            "                f'REJECTED {msg.command_id}: duration must be "
            "between {MIN_DURATION_SEC:.1f} and {MAX_DURATION_SEC:.1f} "
            "seconds',\n",
            "                f'REJECTED {msg.command_id}: duration must be '\n"
            "                f'between {MIN_DURATION_SEC:.1f} and '\n"
            "                f'{MAX_DURATION_SEC:.1f} seconds',\n",
            'duration diagnostic wrapping',
        ),
    )
    for old, new, label in replacements:
        text = replace_once(text, old, new, label)
    return text


def transform_execution_reference(text: str) -> str:
    return replace_once(
        text,
        '    """Sample an absolute-time reference from the current execution '
        'point.\n\n',
        '    """\n'
        '    Sample an absolute-time reference from the current execution '
        'point.\n\n',
        'execution reference docstring',
    )


def transform_execution_test(text: str) -> str:
    text = replace_once(
        text,
        "        self.assertEqual(late[-1][1:], (2.0, 3.0, 0.0, 0.0, 0.0))\n"
        "\n\n    def test_continuous_joint_uses_shortest_turn",
        "        self.assertEqual(late[-1][1:], (2.0, 3.0, 0.0, 0.0, 0.0))\n"
        "\n    def test_continuous_joint_uses_shortest_turn",
        'first excess blank line',
    )
    return replace_once(
        text,
        "        self.assertAlmostEqual(normalized[0], math.radians(190.0))\n"
        "\n\n    def test_sample_reference_rejects_invalid_input",
        "        self.assertAlmostEqual(normalized[0], math.radians(190.0))\n"
        "\n    def test_sample_reference_rejects_invalid_input",
        'second excess blank line',
    )


def transform_gui(text: str) -> str:
    return replace_once(
        text,
        '            blocker = QSignalBlocker(self.jog_mode_button)\n'
        '            self.jog_mode_button.setChecked(False)\n'
        '            del blocker\n',
        '            blocker = QSignalBlocker(self.jog_mode_button)\n'
        '            self.jog_mode_button.setChecked(False)\n'
        '            blocker.unblock()\n',
        'QSignalBlocker use',
    )


def transform_gui_test(text: str) -> str:
    text = replace_once(
        text,
        'from PyQt5.QtWidgets import QApplication\n',
        'from PyQt5.QtWidgets import QApplication  # noqa: E402\n',
        'PyQt environment import',
    )
    text = replace_once(
        text,
        'from thesis_ui.joint_control_gui import (\n',
        'from thesis_ui.joint_control_gui import (  # noqa: E402\n',
        'GUI environment import',
    )
    return replace_once(
        text,
        '        self.current_positions = dict(zip(JOINT_NAMES, '
        '[0, 3.14, 3.14, 0, 0, 0]))\n'
        "        self.last_command_id = 'gui_test'\n",
        '        self.current_positions = dict(zip(JOINT_NAMES, '
        '[0, 3.14, 3.14, 0, 0, 0]))\n'
        '        self.current_velocities = {}\n'
        "        self.last_command_id = 'gui_test'\n",
        'FakeNode current velocities',
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path.cwd())
    parser.add_argument('--check', action='store_true')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    if not (root / 'src').is_dir():
        print(f'ERROR: no src directory under {root}', file=sys.stderr)
        return 2

    transforms: Dict[str, Callable[[str], str]] = {
        'src/thesis_core/thesis_core/safety_supervisor_node.py':
            transform_safety,
        'src/thesis_core/thesis_core/execution_reference.py':
            transform_execution_reference,
        'src/thesis_core/test/test_execution_reference.py':
            transform_execution_test,
        'src/thesis_ui/thesis_ui/joint_control_gui.py': transform_gui,
        'src/thesis_ui/test/test_gui_state.py': transform_gui_test,
    }

    for relative, expected in EXPECTED_HASHES.items():
        path = root / relative
        if not path.is_file():
            print(f'ERROR: missing {relative}', file=sys.stderr)
            return 3
        actual = sha256(path)
        if actual != expected:
            print(
                f'ERROR: hash mismatch for {relative}\n'
                f'expected={expected}\nactual={actual}',
                file=sys.stderr,
            )
            return 4

    changed: Dict[str, str] = {}
    try:
        for relative, transform in transforms.items():
            original = (root / relative).read_text(encoding='utf-8')
            result = transform(original)
            if result == original:
                raise RuntimeError(f'{relative}: transform made no change')
            changed[relative] = result
    except RuntimeError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 5

    if args.check:
        print('CHECK_CORRECCION_OK=1')
        for relative in changed:
            print(f'WOULD_MODIFY={relative}')
        return 0

    stamp = datetime.now().strftime('%Y%m%dT%H%M%S')
    backup = root.parent / f'RESPALDO_TESIS_PAQUETE1_FASE_D_{stamp}'
    for relative in changed:
        source = root / relative
        destination = backup / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)

    for relative, result in changed.items():
        (root / relative).write_text(result, encoding='utf-8')

    print('CORRECCION_APLICADA_OK=1')
    print(f'RESPALDO={backup}')
    for relative in changed:
        print(f'MODIFICADO={relative}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
