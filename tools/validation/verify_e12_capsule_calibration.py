#!/usr/bin/env python3
"""Verify that every configured E12 capsule has a discriminating pose."""

import rclpy

from e12_capsule_pose import CapsulePose, NAMES, RADII


def main():
    rclpy.init()
    node = CapsulePose()
    failures = []
    try:
        node.wait_for_geometry(5.0)
        print('RADIOS=' + ','.join(f'{value:.3f}' for value in RADII))
        for name in NAMES:
            try:
                center, clearance = node.choose_pose(name, 0.10)
            except RuntimeError as error:
                failures.append(name)
                print(f'SEGMENTO={name}; VERIFICACION=FAIL; MOTIVO={error}')
                continue
            print(
                f'SEGMENTO={name}; VERIFICACION=OK; '
                f'POSE={center[0]:.6f},{center[1]:.6f},{center[2]:.6f}; '
                f'CLEARANCE={clearance:.6f}'
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if failures:
        raise SystemExit('CAPSULAS_NO_DISCRIMINABLES=' + ','.join(failures))
    print('CAPSULAS_DISCRIMINABLES=6/6')


if __name__ == '__main__':
    main()
