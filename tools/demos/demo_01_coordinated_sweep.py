#!/usr/bin/env python3
"""Sweep several joints together while returning to the initial pose."""

from demo_common import HOME_DEG, run_demo


def main():
    run_demo('barrido_coordinado', [
        ('Barrido diagonal A', (54, 189, 210, -46, 48, -46)),
        ('Retorno a HOME', HOME_DEG),
        ('Barrido diagonal B', (-52, 211, 150, 44, -35, -38)),
        ('Retorno a HOME', HOME_DEG),
    ])


if __name__ == '__main__':
    main()
