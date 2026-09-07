import math
import sys
import time

from PyQt5.QtCore import QSignalBlocker
from PyQt5.QtCore import Qt
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QHeaderView
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QDoubleSpinBox
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QGroupBox
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QScrollArea
from PyQt5.QtWidgets import QSlider
from PyQt5.QtWidgets import QTableWidget
from PyQt5.QtWidgets import QTableWidgetItem
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from thesis_interfaces.msg import JointCommand
import tf2_ros


JOINT_NAMES = [
    'j2n6s300_joint_1',
    'j2n6s300_joint_2',
    'j2n6s300_joint_3',
    'j2n6s300_joint_4',
    'j2n6s300_joint_5',
    'j2n6s300_joint_6',
]

JOINT_LABELS = [
    'J1 - Base',
    'J2 - Hombro',
    'J3 - Codo',
    'J4 - Muñeca 1',
    'J5 - Muñeca 2',
    'J6 - Muñeca 3',
]

JOINT_LIMITS_DEG = [
    (-360.0, 360.0),
    (47.0, 313.0),
    (19.0, 341.0),
    (-360.0, 360.0),
    (-360.0, 360.0),
    (-360.0, 360.0),
]

INITIAL_POSE_DEG = [0.0, 180.0, 180.0, 0.0, 0.0, 0.0]
TEST_POSE_DEG = [math.degrees(0.20), 180.0, 180.0, 0.0, 0.0, 0.0]

# These values match the current supervisor configuration:
# nominal J1-J3 = 36 deg/s, nominal J4-J6 = 48 deg/s,
# with velocity_scale = 0.5.
ALLOWED_SPEED_DEG = [18.0, 18.0, 18.0, 24.0, 24.0, 24.0]
CONTINUOUS_JOINT_INDEXES = {0, 3, 4, 5}


def quaternion_to_rpy(x, y, z, w):
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch = math.asin(sin_pitch)

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)

    return roll, pitch, yaw


class JointGuiNode(Node):

    def __init__(self):
        super().__init__('joint_control_gui_node')

        self.current_positions = {}
        self.last_state_time = None
        self.last_command_id = None
        self.last_allowed_id = None
        self.end_effector_pose = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
        )

        self.candidate_publisher = self.create_publisher(
            JointCommand,
            '/thesis/candidate_command',
            10,
        )

        self.state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.state_callback,
            10,
        )

        self.supervised_subscription = self.create_subscription(
            JointCommand,
            '/thesis/supervised_command',
            self.supervised_callback,
            10,
        )

    def state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in JOINT_NAMES and math.isfinite(position):
                self.current_positions[name] = float(position)

        if all(name in self.current_positions for name in JOINT_NAMES):
            self.last_state_time = time.monotonic()

    def supervised_callback(self, msg):
        if msg.command_id == self.last_command_id:
            self.last_allowed_id = msg.command_id

    def state_is_ready(self):
        if self.last_state_time is None:
            return False

        return time.monotonic() - self.last_state_time <= 1.0

    def update_end_effector_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'world',
                'j2n6s300_end_effector',
                Time(),
            )
        except tf2_ros.TransformException:
            self.end_effector_pose = None
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        roll, pitch, yaw = quaternion_to_rpy(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )

        self.end_effector_pose = (
            translation.x,
            translation.y,
            translation.z,
            roll,
            pitch,
            yaw,
        )

    def connection_status(self):
        return {
            'Gazebo /clock': self.count_publishers('/clock') > 0,
            '/joint_states': self.count_publishers('/joint_states') > 0,
            'Supervisor': (
                self.candidate_publisher.get_subscription_count() > 0
            ),
            'Salida supervisada': (
                self.count_publishers('/thesis/supervised_command') > 0
            ),
        }

    def publish_candidate(self, positions, duration_sec):
        msg = JointCommand()
        now = self.get_clock().now()

        msg.stamp = now.to_msg()
        msg.command_id = f'gui_{now.nanoseconds}'
        msg.joint_names = list(JOINT_NAMES)
        msg.positions = list(positions)

        seconds = int(duration_sec)
        nanoseconds = int((duration_sec - seconds) * 1e9)
        msg.duration.sec = seconds
        msg.duration.nanosec = nanoseconds

        self.last_command_id = msg.command_id
        self.last_allowed_id = None
        self.candidate_publisher.publish(msg)

        return (
            msg.command_id,
            self.candidate_publisher.get_subscription_count(),
        )


class JointControlWindow(QMainWindow):

    def __init__(self, ros_node):
        super().__init__()

        self.ros_node = ros_node
        self.target_inputs = []
        self.target_sliders = []
        self.current_degree_labels = []
        self.current_radian_labels = []
        self.target_radian_labels = []
        self.pending_command_id = None
        self.pending_since = None
        self.connection_indicators = {}
        self.pose_labels = []
        self.minimum_safe_duration = None

        self.setWindowTitle('Tesis 2 - Control articular JACO2')
        self.resize(1050, 760)
        self.setMinimumSize(760, 520)

        self.build_ui()
        self.apply_style()

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.refresh_ui)
        self.ui_timer.start(100)

    def build_ui(self):
        content_widget = QWidget()
        main_layout = QVBoxLayout(content_widget)

        title = QLabel('Control articular supervisado - JACO2')
        title.setObjectName('titleLabel')
        main_layout.addWidget(title)

        safety_banner = QLabel(
            'MODO SIMULACIÓN: los comandos se publican únicamente en '
            '/thesis/candidate_command y deben pasar por el supervisor.'
        )
        safety_banner.setObjectName('safetyBanner')
        safety_banner.setWordWrap(True)
        main_layout.addWidget(safety_banner)

        state_group = QGroupBox('Estado y objetivos articulares')
        state_layout = QGridLayout(state_group)

        headers = [
            'Articulación',
            'Actual (°)',
            'Actual (rad)',
            'Objetivo (°)',
            'Ajuste',
            'Objetivo (rad)',
        ]

        for column, header in enumerate(headers):
            label = QLabel(header)
            label.setObjectName('headerLabel')
            state_layout.addWidget(label, 0, column)

        for index, joint_label in enumerate(JOINT_LABELS):
            row = index + 1
            name_label = QLabel(joint_label)
            current_degrees = QLabel('--')
            current_radians = QLabel('--')
            target_radians = QLabel('0.0000')
            target_input = QDoubleSpinBox()
            target_slider = QSlider(Qt.Horizontal)
            adjustment_widget = QWidget()
            adjustment_layout = QHBoxLayout(adjustment_widget)
            decrease_button = QPushButton('−1°')
            increase_button = QPushButton('+1°')

            lower, upper = JOINT_LIMITS_DEG[index]
            target_input.setRange(lower, upper)
            target_input.setDecimals(2)
            target_input.setSingleStep(1.0)
            target_input.setSuffix(' °')
            target_input.setValue(INITIAL_POSE_DEG[index])
            target_input.valueChanged.connect(
                lambda value, item=index: self.target_spin_changed(
                    item,
                    value,
                )
            )

            target_slider.setRange(
                int(lower * 10.0),
                int(upper * 10.0),
            )
            target_slider.setSingleStep(10)
            target_slider.setPageStep(50)
            target_slider.setValue(
                int(round(INITIAL_POSE_DEG[index] * 10.0))
            )
            target_slider.valueChanged.connect(
                lambda value, item=index: self.target_slider_changed(
                    item,
                    value,
                )
            )

            decrease_button.clicked.connect(
                lambda checked=False, item=index: self.jog_joint(
                    item,
                    -1.0,
                )
            )
            increase_button.clicked.connect(
                lambda checked=False, item=index: self.jog_joint(
                    item,
                    1.0,
                )
            )

            adjustment_layout.setContentsMargins(0, 0, 0, 0)
            adjustment_layout.addWidget(decrease_button)
            adjustment_layout.addWidget(target_slider, 1)
            adjustment_layout.addWidget(increase_button)

            self.current_degree_labels.append(current_degrees)
            self.current_radian_labels.append(current_radians)
            self.target_inputs.append(target_input)
            self.target_sliders.append(target_slider)
            self.target_radian_labels.append(target_radians)

            state_layout.addWidget(name_label, row, 0)
            state_layout.addWidget(current_degrees, row, 1)
            state_layout.addWidget(current_radians, row, 2)
            state_layout.addWidget(target_input, row, 3)
            state_layout.addWidget(adjustment_widget, row, 4)
            state_layout.addWidget(target_radians, row, 5)

        main_layout.addWidget(state_group)

        connection_group = QGroupBox('Conexiones de la simulación')
        connection_layout = QHBoxLayout(connection_group)

        for name in [
            'Gazebo /clock',
            '/joint_states',
            'Supervisor',
            'Salida supervisada',
        ]:
            indicator = QLabel(f'● {name}')
            indicator.setStyleSheet('color: #b91c1c; font-weight: bold;')
            self.connection_indicators[name] = indicator
            connection_layout.addWidget(indicator)

        main_layout.addWidget(connection_group)

        pose_group = QGroupBox(
            'Pose actual del efector final respecto a world'
        )
        pose_layout = QGridLayout(pose_group)
        pose_names = ['X (m)', 'Y (m)', 'Z (m)', 'Roll', 'Pitch', 'Yaw']

        for index, name in enumerate(pose_names):
            name_label = QLabel(name)
            value_label = QLabel('--')
            value_label.setObjectName('poseValue')
            self.pose_labels.append(value_label)
            pose_layout.addWidget(name_label, 0, index)
            pose_layout.addWidget(value_label, 1, index)

        main_layout.addWidget(pose_group)

        command_group = QGroupBox('Configuración del comando')
        command_layout = QHBoxLayout(command_group)

        command_layout.addWidget(QLabel('Duración:'))
        self.duration_input = QDoubleSpinBox()
        self.duration_input.setRange(0.1, 30.0)
        self.duration_input.setDecimals(1)
        self.duration_input.setSingleStep(0.5)
        self.duration_input.setValue(3.0)
        self.duration_input.setSuffix(' s')
        command_layout.addWidget(self.duration_input)

        self.copy_button = QPushButton('Copiar postura actual')
        self.copy_button.clicked.connect(self.copy_current_pose)
        command_layout.addWidget(self.copy_button)

        initial_button = QPushButton('Objetivo inicial')
        initial_button.clicked.connect(
            lambda: self.load_pose(INITIAL_POSE_DEG)
        )
        command_layout.addWidget(initial_button)

        test_button = QPushButton('Objetivo de prueba')
        test_button.clicked.connect(
            lambda: self.load_pose(TEST_POSE_DEG)
        )
        command_layout.addWidget(test_button)

        main_layout.addWidget(command_group)

        preview_group = QGroupBox(
            'Previsualización cinemática de velocidad'
        )
        preview_layout = QVBoxLayout(preview_group)

        preview_note = QLabel(
            'Estimación informativa con velocity_scale = 0.5. '
            'El safety_supervisor conserva la decisión final.'
        )
        preview_note.setWordWrap(True)
        preview_layout.addWidget(preview_note)

        self.velocity_table = QTableWidget(6, 5)
        self.velocity_table.setHorizontalHeaderLabels([
            'Joint',
            'Desplazamiento',
            'Velocidad solicitada',
            'Límite permitido',
            'Previsión',
        ])
        self.velocity_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.velocity_table.verticalHeader().setVisible(False)
        self.velocity_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch
        )
        self.velocity_table.setMinimumHeight(220)
        preview_layout.addWidget(self.velocity_table)

        preview_footer = QHBoxLayout()
        self.velocity_summary_label = QLabel(
            'Esperando un estado articular reciente.'
        )
        self.velocity_summary_label.setObjectName('velocitySummary')
        self.adjust_duration_button = QPushButton(
            'Ajustar duración segura'
        )
        self.adjust_duration_button.setEnabled(False)
        self.adjust_duration_button.clicked.connect(
            self.adjust_to_safe_duration
        )
        preview_footer.addWidget(self.velocity_summary_label, 1)
        preview_footer.addWidget(self.adjust_duration_button)
        preview_layout.addLayout(preview_footer)

        main_layout.addWidget(preview_group)

        self.send_button = QPushButton('ENVIAR COMANDO CANDIDATO')
        self.send_button.setObjectName('sendButton')
        self.send_button.clicked.connect(self.send_candidate)
        self.send_button.setEnabled(False)
        main_layout.addWidget(self.send_button)

        self.connection_label = QLabel('Esperando /joint_states...')
        self.connection_label.setObjectName('connectionLabel')
        main_layout.addWidget(self.connection_label)

        self.status_label = QLabel(
            'Sin comandos enviados desde la interfaz.'
        )
        self.status_label.setObjectName('statusLabel')
        self.status_label.setWordWrap(True)
        main_layout.addWidget(self.status_label)

        history_group = QGroupBox('Historial de comandos de la GUI')
        history_layout = QVBoxLayout(history_group)
        self.history_table = QTableWidget(0, 5)
        self.history_table.setHorizontalHeaderLabels([
            'Hora',
            'ID',
            'Objetivo J1',
            'Duración',
            'Estado',
        ])
        self.history_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.history_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch
        )
        self.history_table.setMinimumHeight(150)
        history_layout.addWidget(self.history_table)
        main_layout.addWidget(history_group)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(content_widget)
        self.setCentralWidget(scroll_area)

        for index, value in enumerate(INITIAL_POSE_DEG):
            self.update_target_radians(index, value)

    def apply_style(self):
        self.setStyleSheet(
            'QMainWindow { background-color: #f4f6f8; }'
            'QGroupBox {'
            '  font-weight: bold; border: 1px solid #b8c2cc;'
            '  border-radius: 6px; margin-top: 10px; padding: 12px;'
            '}'
            'QGroupBox::title {'
            '  subcontrol-origin: margin; left: 12px; padding: 0 4px;'
            '}'
            '#titleLabel {'
            '  font-size: 20px; font-weight: bold; color: #17365d;'
            '}'
            '#safetyBanner {'
            '  background-color: #dbeafe; color: #17365d;'
            '  border: 1px solid #60a5fa; border-radius: 5px;'
            '  padding: 9px; font-weight: bold;'
            '}'
            '#headerLabel { font-weight: bold; color: #334155; }'
            '#poseValue {'
            '  font-size: 15px; font-weight: bold; color: #17365d;'
            '}'
            '#velocitySummary {'
            '  font-size: 14px; font-weight: bold; padding: 6px;'
            '}'
            '#sendButton {'
            '  background-color: #166534; color: white;'
            '  padding: 10px; font-weight: bold; border-radius: 5px;'
            '}'
            '#sendButton:disabled { background-color: #94a3b8; }'
            '#connectionLabel { font-weight: bold; color: #92400e; }'
            '#statusLabel {'
            '  background-color: white; border: 1px solid #cbd5e1;'
            '  border-radius: 5px; padding: 8px;'
            '}'
        )

    def spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.0)

    def refresh_ui(self):
        state_ready = self.ros_node.state_is_ready()
        self.send_button.setEnabled(state_ready)
        self.copy_button.setEnabled(state_ready)

        if state_ready:
            self.connection_label.setText(
                'Estado ROS: /joint_states recibido correctamente.'
            )
            self.connection_label.setStyleSheet('color: #166534;')
        else:
            self.connection_label.setText(
                'Estado ROS: esperando datos recientes de /joint_states.'
            )
            self.connection_label.setStyleSheet('color: #92400e;')

        for index, joint_name in enumerate(JOINT_NAMES):
            position = self.ros_node.current_positions.get(joint_name)

            if position is None:
                self.current_degree_labels[index].setText('--')
                self.current_radian_labels[index].setText('--')
                continue

            self.current_degree_labels[index].setText(
                f'{math.degrees(position):.2f}'
            )
            self.current_radian_labels[index].setText(
                f'{position:.4f}'
            )

        self.refresh_connections()
        self.refresh_end_effector_pose()
        self.refresh_velocity_preview()
        self.refresh_command_status()

    def shortest_delta_degrees(
        self,
        joint_index,
        target_degrees,
        current_degrees,
    ):
        delta = target_degrees - current_degrees

        if joint_index in CONTINUOUS_JOINT_INDEXES:
            delta = (delta + 180.0) % 360.0 - 180.0

        return delta

    def refresh_velocity_preview(self):
        if not self.ros_node.state_is_ready():
            self.minimum_safe_duration = None
            self.adjust_duration_button.setEnabled(False)
            self.velocity_summary_label.setText(
                'Esperando un estado articular reciente.'
            )
            self.velocity_summary_label.setStyleSheet(
                'color: #92400e;'
            )
            return

        duration_sec = self.duration_input.value()
        minimum_duration = 0.0
        limiting_joint = JOINT_LABELS[0]
        maximum_ratio = -1.0
        predicted_allow = True

        for index, joint_name in enumerate(JOINT_NAMES):
            current_radians = self.ros_node.current_positions[joint_name]
            current_degrees = math.degrees(current_radians)
            target_degrees = self.target_inputs[index].value()
            delta_degrees = self.shortest_delta_degrees(
                index,
                target_degrees,
                current_degrees,
            )
            requested_speed = abs(delta_degrees) / duration_sec
            allowed_speed = ALLOWED_SPEED_DEG[index]
            required_duration = abs(delta_degrees) / allowed_speed
            ratio = requested_speed / allowed_speed
            joint_allowed = requested_speed <= allowed_speed + 1e-9

            if required_duration > minimum_duration:
                minimum_duration = required_duration

            if ratio > maximum_ratio:
                maximum_ratio = ratio
                limiting_joint = JOINT_LABELS[index]

            if not joint_allowed:
                predicted_allow = False

            values = [
                JOINT_LABELS[index],
                f'{delta_degrees:+.2f}°',
                f'{requested_speed:.2f}°/s',
                f'{allowed_speed:.2f}°/s',
                'ALLOW' if joint_allowed else 'REJECT',
            ]

            for column, value in enumerate(values):
                item = QTableWidgetItem(value)

                if column == 4:
                    color = '#166534' if joint_allowed else '#b91c1c'
                    item.setForeground(QColor(color))

                self.velocity_table.setItem(index, column, item)

        self.minimum_safe_duration = minimum_duration
        self.adjust_duration_button.setEnabled(True)

        if predicted_allow:
            self.velocity_summary_label.setText(
                'PROBABLE ALLOW | '
                f'Joint limitante: {limiting_joint} | '
                f'Duración mínima estimada: {minimum_duration:.2f} s'
            )
            self.velocity_summary_label.setStyleSheet(
                'color: #166534;'
            )
        else:
            self.velocity_summary_label.setText(
                'PROBABLE REJECT | '
                f'Joint limitante: {limiting_joint} | '
                f'Duración mínima estimada: {minimum_duration:.2f} s'
            )
            self.velocity_summary_label.setStyleSheet(
                'color: #b91c1c;'
            )

    def adjust_to_safe_duration(self):
        if self.minimum_safe_duration is None:
            return

        duration_with_margin = (
            math.ceil((self.minimum_safe_duration + 0.1) * 10.0)
            / 10.0
        )
        duration_with_margin = max(0.1, duration_with_margin)
        duration_with_margin = min(30.0, duration_with_margin)
        self.duration_input.setValue(duration_with_margin)

        self.status_label.setText(
            'Duración ajustada con 0.1 s de margen. '
            'El supervisor volverá a verificar el comando al enviarlo.'
        )
        self.status_label.setStyleSheet('color: #334155;')

    def refresh_connections(self):
        for name, connected in self.ros_node.connection_status().items():
            indicator = self.connection_indicators[name]
            color = '#166534' if connected else '#b91c1c'
            text = '●' if connected else '○'
            indicator.setText(f'{text} {name}')
            indicator.setStyleSheet(
                f'color: {color}; font-weight: bold;'
            )

    def refresh_end_effector_pose(self):
        self.ros_node.update_end_effector_pose()
        pose = self.ros_node.end_effector_pose

        if pose is None:
            for label in self.pose_labels:
                label.setText('--')
            return

        values = [
            f'{pose[0]:.3f}',
            f'{pose[1]:.3f}',
            f'{pose[2]:.3f}',
            f'{math.degrees(pose[3]):.1f}°',
            f'{math.degrees(pose[4]):.1f}°',
            f'{math.degrees(pose[5]):.1f}°',
        ]

        for label, value in zip(self.pose_labels, values):
            label.setText(value)

    def refresh_command_status(self):
        if self.pending_command_id is None:
            return

        if self.ros_node.last_allowed_id == self.pending_command_id:
            self.status_label.setText(
                f'ALLOW: {self.pending_command_id} fue reenviado por '
                'el supervisor al adaptador de simulación.'
            )
            self.status_label.setStyleSheet('color: #166534;')
            self.update_history_state(
                self.pending_command_id,
                'ALLOW',
            )
            self.pending_command_id = None
            return

        if time.monotonic() - self.pending_since > 1.5:
            self.status_label.setText(
                f'SIN SALIDA SUPERVISADA: {self.pending_command_id}. '
                'El comando pudo ser rechazado; revise el mensaje del '
                'safety_supervisor para conocer la causa.'
            )
            self.status_label.setStyleSheet('color: #b91c1c;')
            self.update_history_state(
                self.pending_command_id,
                'SIN SALIDA',
            )
            self.pending_command_id = None

    def target_spin_changed(self, index, degrees):
        blocker = QSignalBlocker(self.target_sliders[index])
        self.target_sliders[index].setValue(
            int(round(degrees * 10.0))
        )
        del blocker
        self.update_target_radians(index, degrees)

    def target_slider_changed(self, index, slider_value):
        self.target_inputs[index].setValue(slider_value / 10.0)

    def jog_joint(self, index, increment_degrees):
        target_input = self.target_inputs[index]
        target_input.setValue(
            target_input.value() + increment_degrees
        )

    def update_target_radians(self, index, degrees):
        radians = math.radians(degrees)
        self.target_radian_labels[index].setText(f'{radians:.4f}')

    def add_history_row(
        self,
        command_id,
        target_j1,
        duration_sec,
        state,
    ):
        self.history_table.insertRow(0)
        values = [
            time.strftime('%H:%M:%S'),
            command_id,
            f'{target_j1:.2f}°',
            f'{duration_sec:.1f} s',
            state,
        ]

        for column, value in enumerate(values):
            self.history_table.setItem(
                0,
                column,
                QTableWidgetItem(value),
            )

        while self.history_table.rowCount() > 12:
            self.history_table.removeRow(
                self.history_table.rowCount() - 1
            )

    def update_history_state(self, command_id, state):
        for row in range(self.history_table.rowCount()):
            item = self.history_table.item(row, 1)

            if item is not None and item.text() == command_id:
                self.history_table.setItem(
                    row,
                    4,
                    QTableWidgetItem(state),
                )
                return

    def load_pose(self, pose_degrees):
        for target_input, value in zip(
            self.target_inputs,
            pose_degrees,
        ):
            target_input.setValue(value)

        self.status_label.setText(
            'Objetivo cargado. El brazo no se moverá hasta presionar '
            'ENVIAR COMANDO CANDIDATO.'
        )
        self.status_label.setStyleSheet('color: #334155;')

    def copy_current_pose(self):
        if not self.ros_node.state_is_ready():
            QMessageBox.warning(
                self,
                'Estado no disponible',
                'No existen datos articulares recientes.',
            )
            return

        positions = [
            self.ros_node.current_positions[name]
            for name in JOINT_NAMES
        ]

        for target_input, position in zip(
            self.target_inputs,
            positions,
        ):
            target_input.setValue(math.degrees(position))

        self.status_label.setText(
            'La postura actual se copió como objetivo.'
        )
        self.status_label.setStyleSheet('color: #334155;')

    def send_candidate(self):
        if not self.ros_node.state_is_ready():
            QMessageBox.warning(
                self,
                'Estado no disponible',
                'No se enviará el comando sin /joint_states reciente.',
            )
            return

        positions = [
            math.radians(target_input.value())
            for target_input in self.target_inputs
        ]

        command_id, subscriber_count = (
            self.ros_node.publish_candidate(
                positions,
                self.duration_input.value(),
            )
        )

        self.pending_command_id = command_id
        self.pending_since = time.monotonic()
        self.add_history_row(
            command_id,
            self.target_inputs[0].value(),
            self.duration_input.value(),
            'PENDIENTE',
        )

        if subscriber_count == 0:
            self.status_label.setText(
                f'ADVERTENCIA: {command_id} fue publicado, pero no hay '
                'suscriptores en /thesis/candidate_command. Inicie el '
                'pipeline de seguridad.'
            )
            self.status_label.setStyleSheet('color: #b91c1c;')
            return

        self.status_label.setText(
            f'PENDIENTE: {command_id} enviado al supervisor.'
        )
        self.status_label.setStyleSheet('color: #92400e;')

    def closeEvent(self, event):
        self.ros_timer.stop()
        self.ui_timer.stop()
        self.ros_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        event.accept()


def main(args=None):
    rclpy.init(args=args)
    ros_node = JointGuiNode()
    app = QApplication([sys.argv[0]])
    window = JointControlWindow(ros_node)
    window.show()
    return app.exec_()


if __name__ == '__main__':
    main()
