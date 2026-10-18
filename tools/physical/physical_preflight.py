#!/usr/bin/env python3
"""Read-only preflight for the physical JACO2 stack.

This observer never creates publishers, service clients, or action clients.
It may be run in static-only mode while the arm is disconnected.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time


JOINT_NAMES = tuple(f'j2n6s300_joint_{index}' for index in range(1, 7))
TERMINAL_STATUSES = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'DRY_RUN'}
KINOVA_USB_ID = '22cd:0000'
KINOVA_REQUIRED_SYMBOLS = {
    'InitAPI', 'CloseAPI', 'GetDevices', 'SetActiveDevice',
    'GetAngularPosition', 'StartControlAPI', 'StopControlAPI',
    'SendBasicTrajectory', 'EraseAllTrajectories', 'SetAngularControl',
}


def command_output(arguments):
    """Run one non-mutating host command and return its result."""
    return subprocess.run(
        arguments, check=False, text=True, capture_output=True, timeout=10
    )


def dependency_status(path):
    """Report whether a shared object exists and all ldd entries resolve."""
    candidate = Path(path).expanduser().resolve()
    result = {'path': str(candidate), 'exists': candidate.is_file()}
    if not result['exists']:
        result['resolved'] = False
        result['detail'] = 'library is absent'
        result['required_symbols_available'] = False
        result['missing_required_symbols'] = sorted(
            KINOVA_REQUIRED_SYMBOLS
        )
        return result
    completed = command_output(['ldd', str(candidate)])
    result['resolved'] = (
        completed.returncode == 0
        and 'not found' not in completed.stdout
    )
    result['detail'] = (completed.stdout + completed.stderr).strip()
    symbols = command_output(['nm', '-D', '--defined-only', str(candidate)])
    exported = {
        line.split()[-1] for line in symbols.stdout.splitlines()
        if line.split()
    }
    missing = sorted(KINOVA_REQUIRED_SYMBOLS - exported)
    result['required_symbols_available'] = (
        symbols.returncode == 0 and not missing
    )
    result['missing_required_symbols'] = missing
    return result


def usb_status():
    """Detect the known Kinova USB device and check current-user access."""
    completed = command_output(['lsusb'])
    lines = completed.stdout.splitlines() if completed.returncode == 0 else []
    matches = [line for line in lines if KINOVA_USB_ID in line.lower()]
    devices = []
    for line in matches:
        match = re.match(r'Bus (\d+) Device (\d+):', line)
        device = {'description': line, 'path': None, 'readable': False,
                  'writable': False}
        if match:
            path = Path('/dev/bus/usb') / match.group(1) / match.group(2)
            device.update({
                'path': str(path),
                'readable': os.access(path, os.R_OK),
                'writable': os.access(path, os.W_OK),
            })
        devices.append(device)
    return {
        'lsusb_available': completed.returncode == 0,
        'kinova_usb_id': KINOVA_USB_ID,
        'detected': bool(devices),
        'devices': devices,
    }


def static_checks(workspace, kinova_root):
    """Inspect source defaults, SDK files and current USB enumeration."""
    config = (
        workspace / 'src/thesis_hardware_bridge/config/jaco_hardware.yaml'
    )
    launch = (
        workspace
        / 'src/thesis_hardware/launch/jaco_physical_system.launch.py'
    )
    config_text = config.read_text(encoding='utf-8')
    launch_text = launch.read_text(encoding='utf-8')
    library = kinova_root / 'lib/x86_64-linux-gnu/USBCommandLayerUbuntu.so'
    header = kinova_root / 'include/kinova/Kinova.API.USBCommandLayerUbuntu.h'
    safe_defaults = {
        'mock_hardware_false': bool(re.search(
            r'^\s*mock_hardware:\s*false\s*$', config_text, re.MULTILINE
        )),
        'hardware_output_enabled_false': bool(re.search(
            r'^\s*hardware_output_enabled:\s*false\s*$',
            config_text, re.MULTILINE,
        )),
        'physical_launch_gui_disabled': (
            "'start_gui',\n            default_value='false'" in launch_text
        ),
        'physical_launch_gui_arm_disabled': (
            "'allow_gui_arm_control',\n            default_value='false'"
            in launch_text
        ),
    }
    return {
        'safe_defaults': safe_defaults,
        'kinova_header': {'path': str(header), 'exists': header.is_file()},
        'kinova_library': dependency_status(library),
        'usb': usb_status(),
    }


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def canonical_positions(message):
    values = dict(zip(message.name, message.position))
    if not all(name in values for name in JOINT_NAMES):
        return None
    result = tuple(float(values[name]) for name in JOINT_NAMES)
    return result if all(math.isfinite(value) for value in result) else None


def run_ros_observation(duration_sec, motion_tolerance_rad):
    """Observe the already-running physical stack without sending requests."""
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.node import Node
    from rclpy.parameter_client import AsyncParameterClient
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener
    from thesis_interfaces.msg import (
        ExecutionTrajectory,
        JointCommand,
        ProximityStatus,
    )

    class Observer(Node):
        def __init__(self):
            super().__init__('physical_preflight_read_only')
            self.connected = None
            self.armed = None
            self.ready = None
            self.readiness = None
            self.joints = []
            self.proximities = []
            self.diagnostics = {}
            self.command_counts = Counter()
            self.execution_events = []
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(
                Bool, '/thesis/hardware/connected',
                lambda msg: setattr(self, 'connected', bool(msg.data)), 10
            )
            self.create_subscription(
                Bool, '/thesis/hardware/armed',
                lambda msg: setattr(self, 'armed', bool(msg.data)), 10
            )
            self.create_subscription(
                Bool, '/thesis/system_ready',
                lambda msg: setattr(self, 'ready', bool(msg.data)), 10
            )
            self.create_subscription(
                String, '/thesis/system_readiness',
                lambda msg: setattr(self, 'readiness', msg.data), 10
            )
            self.create_subscription(
                JointState, '/joint_states', self.joint_callback, 50
            )
            self.create_subscription(
                ProximityStatus, '/thesis/proximity_status',
                self.proximity_callback, 20
            )
            self.create_subscription(
                DiagnosticArray, '/thesis/hardware/diagnostics',
                self.diagnostic_callback, 10
            )
            for topic in (
                '/thesis/candidate_command',
                '/thesis/supervised_command',
                '/thesis/candidate_jog_command',
                '/thesis/supervised_jog_command',
            ):
                self.create_subscription(
                    JointCommand, topic,
                    lambda _msg, name=topic: self.command_counts.update(
                        [name]
                    ),
                    10,
                )
            self.create_subscription(
                ExecutionTrajectory, '/thesis/execution_trajectory',
                self.execution_events.append, 20
            )

        def joint_callback(self, message):
            positions = canonical_positions(message)
            self.joints.append({
                'received': time.monotonic(),
                'stamp': stamp_seconds(message.header.stamp),
                'names': list(message.name),
                'positions': positions,
            })

        def proximity_callback(self, message):
            self.proximities.append({
                'received': time.monotonic(),
                'stamp': stamp_seconds(message.stamp),
                'frame': message.reference_frame,
                'state': message.state,
                'segment': message.limiting_segment,
            })

        def diagnostic_callback(self, message):
            for status in message.status:
                if status.name == 'thesis_hardware_bridge/JACO2':
                    self.diagnostics = {
                        item.key: item.value for item in status.values
                    }
                    self.diagnostics['message'] = status.message

    rclpy.init()
    node = Observer()
    started = time.monotonic()
    try:
        while time.monotonic() - started < duration_sec:
            rclpy.spin_once(node, timeout_sec=0.05)

        parameter_client = AsyncParameterClient(node, 'jaco_hardware_adapter')
        parameters = {}
        if parameter_client.wait_for_service(timeout_sec=1.0):
            future = parameter_client.get_parameters([
                'mock_hardware', 'hardware_output_enabled',
                'state_publish_rate_hz',
            ])
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if future.done() and future.result() is not None:
                values = future.result().values
                parameters = {
                    'mock_hardware': values[0].bool_value,
                    'hardware_output_enabled': values[1].bool_value,
                    'state_publish_rate_hz': values[2].double_value,
                }

        now_ros = node.get_clock().now().nanoseconds / 1e9
        valid_joints = [sample for sample in node.joints
                        if sample['positions'] is not None]
        rate = 0.0
        if len(valid_joints) >= 2:
            span = valid_joints[-1]['received'] - valid_joints[0]['received']
            rate = (len(valid_joints) - 1) / span if span > 0.0 else 0.0
        maximum_motion = math.inf
        if valid_joints:
            first = valid_joints[0]['positions']
            maximum_motion = max(
                abs(math.atan2(math.sin(value - first[index]),
                               math.cos(value - first[index])))
                for sample in valid_joints
                for index, value in enumerate(sample['positions'])
            )
        latest_joint_age = (
            now_ros - valid_joints[-1]['stamp'] if valid_joints else math.inf
        )
        latest_proximity = node.proximities[-1] if node.proximities else None
        proximity_age = (
            now_ros - latest_proximity['stamp']
            if latest_proximity else math.inf
        )
        tf_result = {'available': False, 'age_sec': None, 'error': None}
        if valid_joints:
            try:
                from rclpy.time import Time
                transform = node.tf_buffer.lookup_transform(
                    'world', 'j2n6s300_end_effector',
                    Time(
                        nanoseconds=int(
                            valid_joints[-1]['stamp'] * 1_000_000_000
                        )
                    ),
                )
                tf_result['available'] = True
                tf_result['age_sec'] = now_ros - stamp_seconds(
                    transform.header.stamp
                )
            except Exception as exception:  # tf2 exceptions vary by distro
                tf_result['error'] = str(exception)

        node_names = {name for name, _namespace in
                      node.get_node_names_and_namespaces()}
        terminal = [event for event in node.execution_events
                    if event.status in TERMINAL_STATUSES]
        return {
            'parameters': parameters,
            'connected': node.connected,
            'armed': node.armed,
            'system_ready': node.ready,
            'system_readiness': node.readiness,
            'joint_state': {
                'samples': len(node.joints),
                'valid_samples': len(valid_joints),
                'rate_hz': rate,
                'latest_source_age_sec': latest_joint_age,
                'maximum_observed_motion_rad': maximum_motion,
                'stationary': maximum_motion <= motion_tolerance_rad,
                'canonical_joints_present': bool(valid_joints),
            },
            'tf': tf_result,
            'proximity': {
                'samples': len(node.proximities),
                'latest': latest_proximity,
                'latest_source_age_sec': proximity_age,
            },
            'diagnostics': node.diagnostics,
            'supervisor_present': 'safety_supervisor_node' in node_names,
            'observed_command_messages': dict(node.command_counts),
            'execution_events': [
                {'command_id': event.command_id, 'status': event.status,
                 'detail': event.detail}
                for event in node.execution_events
            ],
            'terminal_execution_events': len(terminal),
            'controller_state': (
                'native Kinova USB backend; controller_manager not applicable'
            ),
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


def evaluate(static, runtime):
    """Return gate status and exact blocking reasons."""
    reasons = []
    if not all(static['safe_defaults'].values()):
        reasons.append('unsafe source defaults')
    if not static['kinova_header']['exists']:
        reasons.append('Kinova API header absent')
    if (
        not static['kinova_library']['resolved']
        or not static['kinova_library']['required_symbols_available']
    ):
        reasons.append('Kinova API library or dependency absent')
    if runtime is None:
        reasons.append('physical runtime was not observed')
        return 'STATIC_ONLY', reasons
    usb = static['usb']
    if not usb['detected']:
        reasons.append('Kinova USB device is not detected')
    elif not all(
            device['readable'] and device['writable']
            for device in usb['devices']):
        reasons.append('current user lacks read/write access to Kinova USB')
    parameters = runtime['parameters']
    if parameters.get('mock_hardware') is not False:
        reasons.append('effective mock_hardware is not false')
    if parameters.get('hardware_output_enabled') is not False:
        reasons.append('effective hardware_output_enabled is not false')
    if runtime['connected'] is not True:
        reasons.append('physical adapter is not connected')
    if runtime['armed'] is not False:
        reasons.append('adapter is not disarmed')
    joints = runtime['joint_state']
    if not joints['canonical_joints_present']:
        reasons.append('canonical JointState is absent or incomplete')
    if (
        not math.isfinite(joints['latest_source_age_sec'])
        or joints['latest_source_age_sec'] > 0.5
    ):
        reasons.append('JointState source stamp is absent or stale')
    if joints['rate_hz'] < 10.0:
        reasons.append('JointState frequency is below 10 Hz')
    if not joints['stationary']:
        reasons.append('motion was observed during read-only preflight')
    if not runtime['tf']['available']:
        reasons.append(
            'world to end-effector TF is unavailable at JointState stamp'
        )
    proximity = runtime['proximity']
    latest = proximity['latest']
    if latest is None or latest['frame'] != 'world':
        reasons.append('world-frame proximity is absent')
    if (
        not math.isfinite(proximity['latest_source_age_sec'])
        or proximity['latest_source_age_sec'] > 0.75
    ):
        reasons.append('proximity source stamp is absent or stale')
    if runtime['system_ready'] is not True:
        reasons.append('system readiness is not READY')
    if not runtime['supervisor_present']:
        reasons.append('safety supervisor node is absent')
    if runtime['observed_command_messages']:
        reasons.append('a physical command message was observed')
    diagnostics = runtime['diagnostics']
    if diagnostics.get('fault') != 'false':
        reasons.append('adapter reports a critical fault or no diagnostics')
    if diagnostics.get('api_open') != 'true':
        reasons.append('Kinova API is not initialized')
    if diagnostics.get('control_api_started') != 'false':
        reasons.append('Kinova control API is unexpectedly active')
    if diagnostics.get('active_goal') != 'false':
        reasons.append('a physical point goal is pending')
    if diagnostics.get('jog_active') != 'false':
        reasons.append('a physical JOG command is active')
    if diagnostics.get('mode') != 'hold':
        reasons.append('adapter diagnostic mode is not hold')
    if diagnostics.get('output_command_rate_hz_measured') not in (
            None, '0.000000'):
        reasons.append('adapter reports physical output commands')
    if runtime['execution_events']:
        reasons.append('an execution event was observed')
    return ('PASS' if not reasons else 'BLOCKED'), reasons


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--static-only', action='store_true')
    parser.add_argument('--duration-sec', type=float, default=8.0)
    parser.add_argument('--motion-tolerance-rad', type=float, default=0.003)
    parser.add_argument(
        '--workspace',
        type=Path,
        default=Path(__file__).resolve().parents[2],
    )
    parser.add_argument('--kinova-root', type=Path, default=Path(
        os.environ.get(
            'KINOVA_ROOT',
            '~/Escritorio/TESIS_2_DEPENDENCIES/kinova-ros/kinova_driver',
        )
    ).expanduser())
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    static = static_checks(
        args.workspace.resolve(), args.kinova_root.resolve()
    )
    runtime = None if args.static_only else run_ros_observation(
        args.duration_sec, args.motion_tolerance_rad
    )
    status, reasons = evaluate(static, runtime)
    report = {
        'timestamp_local': datetime.now().astimezone().isoformat(),
        'mode': 'static-only' if args.static_only else 'physical-read-only',
        'status': status,
        'reasons': reasons,
        'static': static,
        'runtime': runtime,
        'movement_commands_sent': False,
    }
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + '\n', encoding='utf-8')
    return 0 if status in {'PASS', 'STATIC_ONLY'} else 2


if __name__ == '__main__':
    sys.exit(main())
