"""Headless GUI regression checks; run in the sourced ROS workspace."""
import os
import time
import unittest
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5.QtWidgets import QApplication  # noqa: E402
from thesis_ui.joint_control_gui import (  # noqa: E402
    JOINT_NAMES, JointControlWindow, JointGuiNode,
)


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
        self.last_proximity = SimpleNamespace(minimum_clearance=0.32)
        self.proximity_received_at = time.monotonic()
        self.control_received_at = None
        self.execution_by_id = {}
        self.end_effector_pose = (0.485, 0.064, 1.040, 0, 0, 0)
        self.candidate_publisher = SimpleNamespace(get_subscription_count=lambda: 1)

    def state_is_ready(self):
        return True

    def update_end_effector_pose(self):
        pass

    def connection_status(self):
        return {'Control preventivo': True, '/joint_states': True}


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
        for timer in (self.window.ros_timer, self.window.ui_timer,
                      self.window.preview_timer):
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


if __name__ == '__main__':
    unittest.main()
