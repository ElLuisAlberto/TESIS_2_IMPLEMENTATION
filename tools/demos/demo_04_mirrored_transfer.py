#!/usr/bin/env python3
"""Compare two opposing multi-joint transfer motions."""

from demo_common import HOME_DEG, run_demo


def main():
    run_demo('traslados_espejados', [
        ('Traslado A hacia el costado', (55, 206, 194, -44, 47, 44)),
        ('Retorno a HOME', HOME_DEG),
        ('Traslado B hacia el lado opuesto', (43, 210, 208, -45, 43, -38)),
        ('Retorno a HOME', HOME_DEG),
    ])


if __name__ == '__main__':
    main()
