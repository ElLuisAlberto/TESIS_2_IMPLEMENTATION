#!/usr/bin/env python3
"""Contrast two elbow and shoulder configurations with wrist motion."""

from demo_common import HOME_DEG, run_demo


def main():
    run_demo('niveles_hombro_codo', [
        ('Configuración compacta A', (-52, 211, 150, 44, -35, -38)),
        ('Retorno a HOME', HOME_DEG),
        ('Configuración compacta B', (-54, 198, 205, 38, -47, -47)),
        ('Retorno a HOME', HOME_DEG),
    ])


if __name__ == '__main__':
    main()
