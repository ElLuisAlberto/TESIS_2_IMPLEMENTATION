#!/usr/bin/env python3
"""Corrige únicamente el f-string inválido del cierre de la Fase D."""

from __future__ import annotations

import argparse
import py_compile
import shutil
from datetime import datetime
from pathlib import Path


BAD_FRAGMENT = """                f'TTC_nom={event_time_or_invalid(
                    nominal_geometry.first_collision_time
                ):.3f} s'
"""

GOOD_FRAGMENT = """                f'TTC_nom={event_time_or_invalid(nominal_geometry.first_collision_time):.3f} s'
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path.home() / "Escritorio" / "TESIS_2_IMPLEMENTATION",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    target = (
        args.workspace
        / "src"
        / "thesis_core"
        / "thesis_core"
        / "safety_supervisor_node.py"
    )
    if not target.is_file():
        raise SystemExit(f"ERROR: no existe {target}")

    source = target.read_text(encoding="utf-8")
    if GOOD_FRAGMENT in source and BAD_FRAGMENT not in source:
        print("RESULTADO_CORRECCION=YA_CORREGIDO")
    else:
        count = source.count(BAD_FRAGMENT)
        if count != 1:
            raise SystemExit(
                "ERROR: se esperaba exactamente un f-string defectuoso; "
                f"se encontraron {count}. No se modificó el archivo."
            )
        backup_dir = (
            Path.home()
            / "RESPALDO_TESIS_PAQUETE5"
            / f"correccion_fase_d_{datetime.now():%Y%m%d_%H%M%S}"
        )
        backup_dir.mkdir(parents=True, exist_ok=False)
        backup = backup_dir / target.name
        shutil.copy2(target, backup)
        target.write_text(
            source.replace(BAD_FRAGMENT, GOOD_FRAGMENT, 1),
            encoding="utf-8",
        )
        print(f"RESPALDO={backup}")
        print("RESULTADO_CORRECCION=APLICADA")

    py_compile.compile(str(target), doraise=True)
    print("RESULTADO_SINTAXIS=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
