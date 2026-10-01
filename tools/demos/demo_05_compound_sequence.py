#!/usr/bin/env python3
"""Run a compound sequence with multiple joints changing per command."""

from demo_common import HOME_DEG, run_demo


def main():
    run_demo('secuencia_compuesta', [
        ('Fase 1: alcance y orientación', (53, 173, 152, -51, 50, -34)),
        ('Retorno a HOME', HOME_DEG),
        ('Fase 2: elevación y giro de muñeca', (50, 203, 207, -36, 50, -39)),
        ('Retorno final a HOME', HOME_DEG),
    ])


if __name__ == '__main__':
    main()
