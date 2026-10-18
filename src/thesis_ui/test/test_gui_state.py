"""Headless GUI regression checks; run in the sourced ROS workspace."""
import os
import math
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from builtin_interfaces.msg import Time as TimeMessage  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402
from thesis_interfaces.msg import (  # noqa: E402
    CartesianCommand,
    CartesianTrajectoryCommand,
)
from thesis_simulation import cartesian_command_adapter  # noqa: E402
from thesis_simulation import cartesian_jog_adapter  # noqa: E402
from thesis_simulation import cartesian_trajectory_adapter  # noqa: E402
from thesis_simulation.cartesian_command_adapter import (  # noqa: E402
    CartesianCommandAdapter,
)
from thesis_simulation.cartesian_jog_adapter import (  # noqa: E402
    CartesianJogAdapter,
)
from thesis_simulation.cartesian_trajectory_adapter import (  # noqa: E402
    CartesianTrajectoryAdapter,
)
from thesis_ui.joint_control_gui import (  # noqa: E402
    CARTESIAN_JOG_NOTCH_STEPS, JOINT_NAMES, JointControlWindow, JointGuiNode,
)
from tf2_ros import TransformException  # noqa: E402


class CapturingPublisher:

    def __init__(self):
        self.messages = []

    def get_subscription_count(self):
        return 1

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    execution_callback = JointGuiNode.execution_callback
    execution_is_active = JointGuiNode.execution_is_active

    def __init__(self):
        self.current_positions = dict(zip(JOINT_NAMES, [0, 3.14, 3.14, 0, 0, 0]))
        self.current_velocities = {}
        self.last_command_id = 'gui_test'
        self.last_allowed_id = None
        self.last_prediction = None
        self.last_decision = None
        self.last_execution = None
        self.last_execution_control = None
        self.last_cartesian_trajectory_status = None
        self.last_proximity = SimpleNamespace(minimum_clearance=0.32)
        self.proximity_received_at = time.monotonic()
        self.control_received_at = None
        self.execution_by_id = {}
        self.end_effector_pose = (0.485, 0.064, 1.040, 0, 0, 0)
        self.candidate_publisher = CapturingPublisher()
        self.cartesian_candidate_publisher = CapturingPublisher()
        self.cartesian_jog_publisher = CapturingPublisher()
        self.cartesian_trajectory_publisher = CapturingPublisher()
        self.cartesian_trajectory_cancel_publisher = CapturingPublisher()

    def state_is_ready(self):
        return True

    def update_end_effector_pose(self):
        pass

    def connection_status(self):
        return {'Control preventivo': True, '/joint_states': True}

    def cartesian_trajectory_is_active(self):
        status = self.last_cartesian_trajectory_status
        return status is not None and status.status in ('RUNNING', 'PAUSED')

    publish_cartesian_trajectory = JointGuiNode.publish_cartesian_trajectory
    publish_cartesian_candidate = JointGuiNode.publish_cartesian_candidate
    publish_cartesian_jog = JointGuiNode.publish_cartesian_jog
    cancel_cartesian_trajectory = JointGuiNode.cancel_cartesian_trajectory

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(
                nanoseconds=123456,
                to_msg=lambda: TimeMessage(sec=1, nanosec=123456),
            )
        )


def execution(command_id='gui_test', status='ACCEPTED'):
    return SimpleNamespace(
        command_id=command_id, status=status,
        duration_sec=6.0, detail='Resultado de prueba',
    )


class GuiStateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.node = FakeNode()
        self.window = JointControlWindow(self.node)
        for timer in (
            self.window.ros_timer,
            self.window.ui_timer,
            self.window.preview_timer,
            self.window.jog_timer,
        ):
            timer.stop()
        self.window.add_history_row('gui_test', 0, 6, 'PENDIENTE')

    def tearDown(self):
        self.window.deleteLater()

    def test_refresh_reaches_pose_and_velocity_after_connections(self):
        self.window.refresh_ui()
        self.assertIn('Control preventivo', self.window.connection_indicators)
        self.assertEqual(self.window.pose_labels[0].text(), '0.485')
        self.assertIsNotNone(self.window.velocity_table.item(0, 0))
        self.assertTrue(self.window.send_button.isEnabled())

    def test_execution_lifecycle_updates_history_and_send_lock(self):
        for status in ('PENDING', 'ACCEPTED', 'SUCCEEDED'):
            self.node.execution_callback(execution(status=status))
            self.window.refresh_ui()
            self.assertEqual(self.window.history_table.item(0, 4).text(), status)
            self.assertEqual(self.window.send_button.isEnabled(), status == 'SUCCEEDED')

    def test_unrelated_rejection_does_not_unlock_active_goal(self):
        self.node.execution_callback(execution())
        self.node.execution_callback(execution('another', 'REJECTED'))
        self.window.refresh_ui()
        self.assertFalse(self.window.send_button.isEnabled())
        self.assertEqual(self.node.last_execution.command_id, 'gui_test')

    def test_canceled_execution_keeps_final_state(self):
        self.node.execution_callback(execution(status='CANCELED'))
        self.window.refresh_ui()
        self.assertIn('CANCELED', self.window.status_label.text())
        self.assertTrue(self.window.send_button.isEnabled())

    def test_stale_projection_is_not_displayed_as_live(self):
        self.node.execution_callback(execution())
        self.node.last_execution_control = SimpleNamespace(
            command_id='gui_test', state='REDUCTION', speed_scale=0.5,
            minimum_clearance=0.15, minimum_time_from_now=0.65,
            minimum_sample_index=13, minimum_sample_count=21,
            time_to_collision=-1.0,
        )
        self.node.control_received_at = time.monotonic()
        self.window.refresh_ui()
        self.assertIn('+0.65 s', self.window.metric_labels['Instante del mínimo'].text())
        self.node.control_received_at -= 2.0
        self.window.refresh_ui()
        self.assertEqual(self.window.metric_labels['Mínimo proyectado'].text(),
                         'Sin evaluación activa')

    def test_volume_label_distinguishes_preview_from_safe_jog(self):
        self.window.preview_button.setChecked(True)
        self.window.refresh_volume_mode()
        self.assertIn('VOLUMEN NOMINAL', self.window.volume_mode_label.text())
        self.window.jog_mode_button.blockSignals(True)
        self.window.jog_mode_button.setChecked(True)
        self.window.jog_mode_button.blockSignals(False)
        self.window.refresh_volume_mode()
        self.assertIn(
            'VOLUMEN SUPERVISADO CONTINUO',
            self.window.volume_mode_label.text(),
        )

    def test_velocity_violation_disables_point_command(self):
        self.window.duration_input.setValue(0.1)
        self.window.target_inputs[0].setValue(180.0)
        self.window.refresh_velocity_preview()
        self.assertFalse(self.window.send_button.isEnabled())
        self.assertIn(
            'VELOCIDAD: NO ENVIABLE',
            self.window.velocity_summary_label.text(),
        )

    def test_simulation_has_cartesian_pose_controls(self):
        self.assertEqual(len(self.window.cartesian_target_inputs), 6)
        self.window.copy_current_cartesian_pose()
        self.assertTrue(self.window.cartesian_target_initialized)
        self.assertAlmostEqual(
            self.window.cartesian_target_inputs[0].value(), 0.485, places=3
        )
        self.window.adjust_cartesian_target(0, 0.01)
        self.assertAlmostEqual(
            self.window.cartesian_target_inputs[0].value(), 0.495, places=3
        )
        self.window.refresh_ui()
        self.assertTrue(self.window.cartesian_send_button.isEnabled())

    def test_long_cartesian_trajectory_has_speed_and_mode_controls(self):
        self.assertEqual(len(self.window.cartesian_trajectory_inputs), 6)
        self.assertEqual(
            self.window.cartesian_trajectory_mode.currentData(), 'RELATIVE'
        )
        self.assertAlmostEqual(
            self.window.cartesian_linear_speed_input.value(), 0.04
        )
        self.assertAlmostEqual(
            self.window.cartesian_angular_speed_input.value(), 10.0
        )
        self.window.cartesian_trajectory_inputs[0].setValue(-0.12)
        self.window.refresh_ui()
        self.assertTrue(
            self.window.cartesian_trajectory_send_button.isEnabled()
        )

    def test_manual_control_is_split_into_joint_and_cartesian_tabs(self):
        self.assertIsNotNone(self.window.manual_control_tabs)
        self.assertEqual(self.window.manual_control_tabs.count(), 2)
        self.assertEqual(self.window.manual_control_tabs.tabText(0), 'Joints')
        self.assertIn(
            'X/Y/Z', self.window.manual_control_tabs.tabText(1)
        )
        self.assertEqual(len(self.window.cartesian_jog_sliders), 6)
        self.assertEqual(len(self.window.cartesian_jog_speed_inputs), 6)

    def test_cartesian_wheel_publishes_bounded_world_velocity(self):
        self.window.cartesian_jog_mode_button.setChecked(True)
        self.assertTrue(self.window.cartesian_jog_mode_button.isChecked())
        self.assertFalse(self.window.jog_mode_button.isChecked())
        self.window.cartesian_wheel_intentions[0].add(
            time.monotonic(), 1.0
        )
        self.window.publish_cartesian_jog_intent()
        message = self.node.cartesian_jog_publisher.messages[-1]
        linear = (
            message.twist.linear.x,
            message.twist.linear.y,
            message.twist.linear.z,
        )
        angular = (
            message.twist.angular.x,
            message.twist.angular.y,
            message.twist.angular.z,
        )
        self.assertEqual(message.header.frame_id, 'world')
        self.assertGreater(linear[0], 0.0)
        self.assertLessEqual(
            linear[0], self.window.cartesian_jog_speed_inputs[0].value()
        )
        self.assertEqual(linear[1:], (0.0, 0.0))
        self.assertEqual(angular, (0.0, 0.0, 0.0))

    def test_cartesian_jog_all_axes_signs_units_and_watchdog(self):
        self.window.cartesian_jog_mode_button.setChecked(True)
        for axis in range(6):
            for sign in (-1.0, 1.0):
                for intention in self.window.cartesian_wheel_intentions:
                    intention.clear()
                self.window.cartesian_wheel_intentions[axis].add(
                    time.monotonic(), sign,
                )
                self.window.publish_cartesian_jog_intent()
                message = self.node.cartesian_jog_publisher.messages[-1]
                self.assertIsInstance(message, TwistStamped)
                self.assertEqual(message.header.frame_id, 'world')
                linear = (
                    message.twist.linear.x,
                    message.twist.linear.y,
                    message.twist.linear.z,
                )
                angular = (
                    message.twist.angular.x,
                    message.twist.angular.y,
                    message.twist.angular.z,
                )
                expected = (
                    sign * CARTESIAN_JOG_NOTCH_STEPS[axis] / 0.25
                )
                if axis < 3:
                    self.assertAlmostEqual(linear[axis], expected)
                    self.assertEqual(
                        linear[:axis] + linear[axis + 1:], (0.0, 0.0)
                    )
                    self.assertEqual(angular, (0.0, 0.0, 0.0))
                else:
                    self.assertAlmostEqual(
                        angular[axis - 3], math.radians(expected)
                    )
                    self.assertEqual(linear, (0.0, 0.0, 0.0))
                    self.assertEqual(
                        angular[:axis - 3] + angular[axis - 2:],
                        (0.0, 0.0),
                    )

        for intention in self.window.cartesian_wheel_intentions:
            intention.clear()
        self.window.cartesian_wheel_intentions[0].add(
            time.monotonic() - 0.251, 1.0,
        )
        self.window.publish_cartesian_jog_intent()
        message = self.node.cartesian_jog_publisher.messages[-1]
        self.assertEqual(
            (message.twist.linear.x, message.twist.linear.y,
             message.twist.linear.z),
            (0.0, 0.0, 0.0),
        )
        self.assertEqual(
            (message.twist.angular.x, message.twist.angular.y,
             message.twist.angular.z),
            (0.0, 0.0, 0.0),
        )

    def test_cartesian_point_gui_publishes_absolute_pose_all_axes(self):
        base = (0.485, 0.064, 1.040, 0.0, 0.0, 0.0)
        for axis in range(6):
            for sign in (-1.0, 1.0):
                self.window.pending_command_id = None
                self.window.copy_current_cartesian_pose()
                increment = sign * (0.01 if axis < 3 else 5.0)
                self.window.adjust_cartesian_target(axis, increment)
                self.window.send_cartesian_candidate()
                message = self.node.cartesian_candidate_publisher.messages[-1]
                self.assertIsInstance(message, CartesianCommand)
                self.assertEqual(message.frame_id, 'world')
                values = (
                    message.x, message.y, message.z,
                    message.roll, message.pitch, message.yaw,
                )
                expected = list(base)
                expected[axis] += (
                    increment if axis < 3 else math.radians(increment)
                )
                self.assertEqual(values, tuple(expected))

    def test_cartesian_trajectory_gui_publishes_modes_all_axes(self):
        for mode in ('RELATIVE', 'ABSOLUTE'):
            for axis in range(6):
                for sign in (-1.0, 1.0):
                    self.window.cartesian_trajectory_pending_id = None
                    self.window.cartesian_trajectory_mode.setCurrentIndex(
                        0 if mode == 'RELATIVE' else 1
                    )
                    value = sign * (0.10 if axis < 3 else 10.0)
                    self.window.cartesian_trajectory_inputs[axis].setValue(
                        value
                    )
                    self.window.send_cartesian_trajectory()
                    message = (
                        self.node.cartesian_trajectory_publisher.messages[-1]
                    )
                    self.assertIsInstance(
                        message, CartesianTrajectoryCommand
                    )
                    self.assertEqual(message.frame_id, 'world')
                    self.assertEqual(message.mode, mode)
                    values = (
                        message.x, message.y, message.z,
                        message.roll, message.pitch, message.yaw,
                    )
                    expected = value if axis < 3 else math.radians(value)
                    self.assertAlmostEqual(values[axis], expected)

    def test_long_cartesian_trajectory_locks_other_motion_modes(self):
        self.node.last_cartesian_trajectory_status = SimpleNamespace(
            command_id='cartesian_path_test',
            status='PAUSED',
            progress=0.4,
            speed_scale=0.0,
            remaining_distance_m=0.07,
            planned_duration_sec=3.0,
            position_error_m=0.07,
            orientation_error_rad=0.1,
            detail='STOP preventivo',
        )
        self.window.refresh_ui()
        self.assertFalse(self.window.send_button.isEnabled())
        self.assertFalse(self.window.cartesian_send_button.isEnabled())
        self.assertFalse(
            self.window.cartesian_trajectory_send_button.isEnabled()
        )
        self.assertTrue(
            self.window.cartesian_trajectory_cancel_button.isEnabled()
        )
        self.assertIn(
            'PAUSED', self.window.cartesian_trajectory_status_label.text()
        )

    def test_physical_mode_has_explicit_hardware_controls(self):
        self.window.deleteLater()
        self.node.operation_mode = 'hardware'
        self.window = JointControlWindow(self.node)
        for timer in (
            self.window.ros_timer,
            self.window.ui_timer,
            self.window.preview_timer,
            self.window.jog_timer,
        ):
            timer.stop()
        self.assertIn('BRAZO FÍSICO', self.window.windowTitle())
        self.assertIsNotNone(self.window.arm_hardware_button)
        self.assertIsNotNone(self.window.disarm_hardware_button)
        self.assertIsNone(self.window.cartesian_send_button)
        self.assertIsNone(self.window.cartesian_trajectory_send_button)
        self.assertIn('JACO2 USB conectado', self.window.connection_indicators)


class CartesianAdapterCallbackTests(unittest.TestCase):

    @staticmethod
    def _clock():
        return SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: TimeMessage(sec=2, nanosec=0),
            )
        )

    @staticmethod
    def _identity():
        return (
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )

    @staticmethod
    def _logger():
        return SimpleNamespace(info=lambda _message: None, warning=lambda _message: None)

    @staticmethod
    def _cartesian_command(frame='world'):
        message = CartesianCommand()
        message.stamp = TimeMessage(sec=2, nanosec=0)
        message.command_id = 'point-test'
        message.frame_id = frame
        message.duration.sec = 1
        return message

    @staticmethod
    def _trajectory_command(frame='world'):
        message = CartesianTrajectoryCommand()
        message.stamp = TimeMessage(sec=2, nanosec=0)
        message.command_id = 'trajectory-test'
        message.frame_id = frame
        message.mode = 'RELATIVE'
        message.x = 0.10
        message.linear_speed_mps = 0.04
        message.angular_speed_radps = math.radians(10.0)
        return message

    def test_point_adapter_emits_joint_command_or_rejects_without_motion(self):
        current = (0.0, math.pi, math.pi, 0.0, 0.0, 0.0)
        output = CapturingPublisher()
        rejections = []
        node = SimpleNamespace(
            current_joint_vector=lambda: current,
            current_world_transform=self._identity,
            command_publisher=output,
            reject=lambda *_args: rejections.append(_args),
            maximum_position_step_m=0.10,
            maximum_orientation_step_rad=math.radians(20.0),
            position_tolerance_m=0.003,
            orientation_tolerance_rad=math.radians(1.0),
            duration_seconds=CartesianCommandAdapter.duration_seconds,
            get_logger=self._logger,
        )
        solved = SimpleNamespace(
            success=True,
            positions=tuple(value + 0.01 for value in current),
            iterations=1,
            position_error_m=0.0,
            orientation_error_rad=0.0,
        )
        with patch.object(
                cartesian_command_adapter, 'pose_error',
                return_value=((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))), patch.object(
                cartesian_command_adapter, 'align_relative_target',
                return_value=self._identity()), patch.object(
                cartesian_command_adapter, 'solve_inverse_kinematics_transform',
                return_value=solved):
            CartesianCommandAdapter.receive_command(
                node, self._cartesian_command()
            )
        self.assertEqual(len(output.messages), 1)
        self.assertEqual(output.messages[0].joint_names, list(JOINT_NAMES))
        self.assertEqual(tuple(output.messages[0].positions), solved.positions)

        CartesianCommandAdapter.receive_command(
            node, self._cartesian_command('map')
        )
        self.assertEqual(len(output.messages), 1)
        self.assertEqual(rejections[-1][1], 'INVALID_CARTESIAN_FRAME')

        node.current_joint_vector = lambda: None
        CartesianCommandAdapter.receive_command(
            node, self._cartesian_command()
        )
        self.assertEqual(len(output.messages), 1)
        self.assertEqual(rejections[-1][1], 'CARTESIAN_STATE_UNAVAILABLE')

        node.current_joint_vector = lambda: current
        node.current_world_transform = lambda: (_ for _ in ()).throw(
            TransformException('missing TF')
        )
        CartesianCommandAdapter.receive_command(
            node, self._cartesian_command()
        )
        self.assertEqual(len(output.messages), 1)
        self.assertEqual(rejections[-1][1], 'CARTESIAN_TF_UNAVAILABLE')

        node.current_world_transform = self._identity
        with patch.object(
                cartesian_command_adapter, 'pose_error',
                return_value=((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))), patch.object(
                cartesian_command_adapter, 'align_relative_target',
                return_value=self._identity()), patch.object(
                cartesian_command_adapter, 'solve_inverse_kinematics_transform',
                return_value=SimpleNamespace(
                    success=False, position_error_m=1.0,
                    orientation_error_rad=1.0)):
            CartesianCommandAdapter.receive_command(
                node, self._cartesian_command()
            )
        self.assertEqual(len(output.messages), 1)
        self.assertEqual(rejections[-1][1], 'INVERSE_KINEMATICS_FAILED')

        with patch.object(
                cartesian_command_adapter, 'pose_error',
                return_value=((0.11, 0.0, 0.0), (0.0, 0.0, 0.0))):
            CartesianCommandAdapter.receive_command(
                node, self._cartesian_command()
            )
        self.assertEqual(len(output.messages), 1)
        self.assertEqual(rejections[-1][1], 'CARTESIAN_STEP_TOO_LARGE')

    def test_jog_adapter_emits_target_and_holds_on_tf_or_ik_failure(self):
        current = (0.0, math.pi, math.pi, 0.0, 0.0, 0.0)
        output = CapturingPublisher()
        warnings = []
        node = SimpleNamespace(
            current_joint_vector=lambda: current,
            current_world_transform=self._identity,
            command_publisher=output,
            horizon_sec=1.0,
            integration_step_sec=0.10,
            maximum_linear_ik_step_m=0.005,
            maximum_angular_ik_step_rad=math.radians(1.0),
            maximum_linear_speed_mps=0.15,
            maximum_angular_speed_radps=math.radians(30.0),
            position_tolerance_m=0.0002,
            orientation_tolerance_rad=math.radians(0.1),
            warn_limited=lambda detail: warnings.append(detail),
            get_logger=self._logger,
        )
        node.publish_target = lambda stamp, positions: (
            CartesianJogAdapter.publish_target(node, stamp, positions)
        )
        message = TwistStamped()
        message.header.stamp = TimeMessage(sec=2, nanosec=0)
        message.header.frame_id = 'world'
        message.twist.linear.x = 0.01
        solved = SimpleNamespace(
            success=True, positions=tuple(value + 0.001 for value in current)
        )
        with patch.object(
                cartesian_jog_adapter, 'solve_inverse_kinematics_transform',
                return_value=solved):
            CartesianJogAdapter.receive_twist(node, message)
        self.assertEqual(
            len(output.messages[-1].positions), len(current)
        )
        for actual, expected in zip(
                output.messages[-1].positions,
                tuple(value + 0.01 for value in current)):
            self.assertAlmostEqual(actual, expected)

        node.current_world_transform = lambda: (_ for _ in ()).throw(
            TransformException('missing TF')
        )
        CartesianJogAdapter.receive_twist(node, message)
        self.assertEqual(tuple(output.messages[-1].positions), current)

        node.current_world_transform = self._identity
        with patch.object(
                cartesian_jog_adapter, 'solve_inverse_kinematics_transform',
                return_value=SimpleNamespace(success=False, positions=current,
                                             position_error_m=1.0,
                                             orientation_error_rad=1.0)):
            CartesianJogAdapter.receive_twist(node, message)
        self.assertEqual(tuple(output.messages[-1].positions), current)

        count = len(output.messages)
        message.header.frame_id = 'map'
        CartesianJogAdapter.receive_twist(node, message)
        self.assertEqual(len(output.messages), count)
        self.assertTrue(warnings)

        message.header.frame_id = 'world'
        node.current_joint_vector = lambda: None
        CartesianJogAdapter.receive_twist(node, message)
        self.assertEqual(len(output.messages), count)

        node.current_joint_vector = lambda: current
        message.twist.linear.x = 1.0
        CartesianJogAdapter.receive_twist(node, message)
        self.assertEqual(tuple(output.messages[-1].positions), current)

    def test_trajectory_adapter_plans_joint_intent_or_rejects_safely(self):
        current = (0.0,) * len(JOINT_NAMES)
        output = CapturingPublisher()
        rejections = []
        node = SimpleNamespace(
            active=None,
            current_joint_vector=lambda: current,
            current_world_transform=self._identity,
            validate_speed=lambda _message: None,
            target_transform=lambda _message, _world: self._identity(),
            plan_joint_path=lambda *_args: (current, (0.01,) * 6),
            reject=lambda *_args: rejections.append(_args),
            publish_active_status=lambda _status: None,
            intent_publisher=output,
            horizon_sec=1.0,
            set_duration=lambda message, seconds: (
                CartesianTrajectoryAdapter.set_duration(message, seconds)
            ),
            maximum_displacement_m=0.60,
            maximum_rotation_rad=math.pi,
            maximum_nominal_duration_sec=120.0,
            joint_timing_margin=1.20,
            get_logger=self._logger,
            get_clock=self._clock,
        )
        with patch.object(
                cartesian_trajectory_adapter, 'cartesian_distance',
                return_value=(0.10, 0.0)), patch.object(
                cartesian_trajectory_adapter, 'synchronized_duration',
                return_value=1.0), patch.object(
                cartesian_trajectory_adapter, 'minimum_joint_path_duration',
                return_value=0.10):
            CartesianTrajectoryAdapter.command_callback(
                node, self._trajectory_command()
            )
        self.assertIsNotNone(node.active)
        CartesianTrajectoryAdapter.publish_intent(node, node.active)
        self.assertEqual(output.messages[-1].command_id, 'trajectory-test')
        self.assertEqual(tuple(output.messages[-1].positions), (0.01,) * 6)

        node.active = None
        CartesianTrajectoryAdapter.command_callback(
            node, self._trajectory_command('map')
        )
        self.assertIsNone(node.active)
        self.assertIn('frame_id debe ser world', rejections[-1][1])

        node.current_joint_vector = lambda: None
        CartesianTrajectoryAdapter.command_callback(
            node, self._trajectory_command()
        )
        self.assertIsNone(node.active)
        self.assertIn('estado articular completo y reciente', rejections[-1][1])

    def test_trajectory_adapter_tf_ik_and_limit_fail_without_intent(self):
        current = (0.0,) * len(JOINT_NAMES)
        output = CapturingPublisher()
        rejections = []
        node = SimpleNamespace(
            active=None,
            current_joint_vector=lambda: current,
            current_world_transform=lambda: (_ for _ in ()).throw(
                TransformException('missing TF')
            ),
            validate_speed=lambda _message: None,
            target_transform=lambda _message, _world: self._identity(),
            plan_joint_path=lambda *_args: (current, (0.01,) * 6),
            reject=lambda *_args: rejections.append(_args),
            publish_active_status=lambda _status: None,
            intent_publisher=output,
            horizon_sec=1.0,
            maximum_displacement_m=0.60,
            maximum_rotation_rad=math.pi,
            maximum_nominal_duration_sec=120.0,
            joint_timing_margin=1.20,
            get_logger=self._logger,
            get_clock=self._clock,
        )
        CartesianTrajectoryAdapter.command_callback(
            node, self._trajectory_command()
        )
        self.assertIsNone(node.active)
        self.assertIn('missing TF', rejections[-1][1])

        node.current_world_transform = self._identity
        node.plan_joint_path = lambda *_args: (_ for _ in ()).throw(
            ValueError('IK failed')
        )
        with patch.object(
                cartesian_trajectory_adapter, 'cartesian_distance',
                return_value=(0.10, 0.0)), patch.object(
                cartesian_trajectory_adapter, 'synchronized_duration',
                return_value=1.0):
            CartesianTrajectoryAdapter.command_callback(
                node, self._trajectory_command()
            )
        self.assertIsNone(node.active)
        self.assertIn('IK failed', rejections[-1][1])

        node.plan_joint_path = lambda *_args: (current, (0.01,) * 6)
        with patch.object(
                cartesian_trajectory_adapter, 'cartesian_distance',
                return_value=(0.61, 0.0)):
            CartesianTrajectoryAdapter.command_callback(
                node, self._trajectory_command()
            )
        self.assertIsNone(node.active)
        self.assertIn('máximo', rejections[-1][1])
        self.assertEqual(output.messages, [])

if __name__ == '__main__':
    unittest.main()
