#!/usr/bin/env python3
"""Show coordinated changes in the wrist joints and arm configuration."""

from demo_common import HOME_DEG, run_demo


def main():
    run_demo('reorientacion_muneca', [
        ('Reorientación A con brazo recogido', (51, 209, 155, -38, -43, 45)),
        ('Retorno a HOME', HOME_DEG),
        ('Reorientación B con giro combinado', (55, 206, 194, -44, 47, 44)),
        ('Retorno a HOME', HOME_DEG),
    ])


if __name__ == '__main__':
    main()
