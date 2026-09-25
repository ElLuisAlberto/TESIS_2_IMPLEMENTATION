import math
import sys
import time
from collections import deque
from PyQt5.QtCore import QEvent

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
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from thesis_interfaces.msg import (
    CommandDecision,
    ExecutionControl,
    ExecutionTrajectory,
    JointCommand,
    ProximityStatus,
    TrajectoryPrediction,
)
import tf2_ros

from thesis_core.joint_model import (
    CONTINUOUS_JOINT_INDEXES,
    JOINT_NAMES,
    JOINT_POSITION_LIMITS,
    JOINT_VELOCITY_LIMITS,
    saturate_target_by_velocity,
)


JOINT_LABELS = [
    'J1 - Base',
    'J2 - Hombro',
    'J3 - Codo',
    'J4 - Muñeca 1',
    'J5 - Muñeca 2',
    'J6 - Muñeca 3',
]

JOINT_LIMITS_DEG = tuple(
    tuple(math.degrees(value) for value in JOINT_POSITION_LIMITS[name])
    for name in JOINT_NAMES
)

INITIAL_POSE_DEG = [0.0, 180.0, 180.0, 0.0, 0.0, 0.0]
TEST_POSE_DEG = [math.degrees(0.20), 180.0, 180.0, 0.0, 0.0, 0.0]

ALLOWED_SPEED_DEG = tuple(
    math.degrees(JOINT_VELOCITY_LIMITS[name]) for name in JOINT_NAMES
)
JOG_INPUT_TIMEOUT_SEC = 0.30


class WheelIntention:
    """Finite-window wheel rate. Excess input is never queued as a target."""

    def __init__(self, window=0.25, degrees_per_notch=1.0):
        self.window = window
        self.degrees_per_notch = degrees_per_notch
        self.events = deque()

    def clear(self):
        self.events.clear()

    def add(self, now, notches):
        if not math.isfinite(notches) or notches == 0.0:
            return
        if self.events and self.events[-1][1] * notches < 0:
            self.clear()
        self.events.append((now, notches))

    def velocity(self, now, maximum):
        while self.events and now - self.events[0][0] >= self.window:
            self.events.popleft()
        rate = sum(value for _, value in self.events)
        rate *= self.degrees_per_notch / self.window
        return max(-maximum, min(maximum, rate))


def limited_direct_step(
    current_degrees,
    requested_degrees,
    maximum_speed_deg_s,
    elapsed_sec,
    continuous=False,
):
    """Return the reachable reference and velocity for one mouse event."""
    elapsed_sec = min(max(float(elapsed_sec), 0.01), 0.25)
    delta = float(requested_degrees) - float(current_degrees)
    if continuous:
        delta = (delta + 180.0) % 360.0 - 180.0
    maximum_step = max(0.0, float(maximum_speed_deg_s)) * elapsed_sec
    applied = min(max(delta, -maximum_step), maximum_step)
    return (
        float(current_degrees) + applied,
        applied / elapsed_sec,
    )


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
        super().__init__(
            'joint_control_gui_node',
            parameter_overrides=[
                Parameter('use_sim_time', value=True),
            ],
        )

        self.current_positions = {}
        self.current_velocities = {}
        self.last_state_time = None
        self.last_command_id = None
        self.last_allowed_id = None
        self.last_supervised_duration = None
        self.last_prediction = None
        self.last_execution = None
        self.last_execution_control = None
        self.execution_by_id = {}
        self.last_proximity = None
        self.proximity_received_at = None
        self.control_received_at = None
        self.end_effector_pose = None
        self.intent_publisher = self.create_publisher(
            JointCommand, '/thesis/preview_intent', 10,
        )
        self.jog_intent_publisher = self.create_publisher(
            JointCommand, '/thesis/jog_intent', 10,
        )
        self.last_decision = None
        self.decision_subscription = self.create_subscription(
            CommandDecision,
            '/thesis/command_decision',
            self.decision_callback,
            10,
        )

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

        self.prediction_subscription = self.create_subscription(
            TrajectoryPrediction,
            '/thesis/trajectory_prediction',
            self.prediction_callback,
            10,
        )

        self.execution_subscription = self.create_subscription(
            ExecutionTrajectory,
            '/thesis/execution_trajectory',
            self.execution_callback,
            10,
        )
        self.execution_control_subscription = self.create_subscription(
            ExecutionControl,
            '/thesis/execution_control',
            self.execution_control_callback,
            10,
        )

        self.proximity_subscription = self.create_subscription(
            ProximityStatus, '/thesis/proximity_status',
            self.proximity_callback, 10,
        )

    def proximity_callback(self, msg):
        self.last_proximity = msg
        self.proximity_received_at = time.monotonic()

    def execution_is_active(self):
        return any(
            msg.status in ('PENDING', 'ACCEPTED')
            for msg in self.execution_by_id.values()
        )

    def state_callback(self, msg):
        self.current_velocities = {
            name: float(value) for name, value in zip(msg.name, msg.velocity)
            if name in JOINT_NAMES and math.isfinite(value)
        }
        for name, position in zip(msg.name, msg.position):
            if name in JOINT_NAMES and math.isfinite(position):
                self.current_positions[name] = float(position)

        if all(name in self.current_positions for name in JOINT_NAMES):
            self.last_state_time = time.monotonic()

    def supervised_callback(self, msg):
        if msg.command_id == self.last_command_id:
            self.last_allowed_id = msg.command_id
            self.last_supervised_duration = (
                float(msg.duration.sec)
                + float(msg.duration.nanosec) * 1e-9
            )

    def decision_callback(self, msg):
        if msg.command_id == self.last_command_id:
            self.last_decision = msg

    def prediction_callback(self, msg):
        if msg.command_id == self.last_command_id:
            self.last_prediction = msg

    def execution_callback(self, msg):
        self.execution_by_id[msg.command_id] = msg
        # Keep terminal history bounded, retaining every active execution.
        if len(self.execution_by_id) > 100:
            for command_id, execution in list(self.execution_by_id.items()):
                if execution.status not in ('PENDING', 'ACCEPTED'):
                    del self.execution_by_id[command_id]
                    break
        if (
            msg.command_id == self.last_command_id
            or self.last_command_id is None
        ):
            self.last_execution = msg

    def execution_control_callback(self, msg):
        self.last_execution_control = msg
        self.control_received_at = time.monotonic()

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
            'Control manual continuo': (
                self.jog_intent_publisher.get_subscription_count() > 0
            ),
            'Salida supervisada': (
                self.count_publishers('/thesis/supervised_command') > 0
            ),
            'Predicción geométrica': (
                self.count_publishers(
                    '/thesis/trajectory_prediction'
                ) > 0
            ),
            'Referencia de ejecución': (
                self.count_publishers(
                    '/thesis/execution_trajectory'
                ) > 0
            ),
            'Control preventivo': (
                self.count_publishers(
                    '/thesis/execution_control'
                ) > 0
            ),
        }

    def publish_candidate(self, positions, duration_sec):
        msg = JointCommand()
        now = self.get_clock().now()

        msg.stamp = now.to_msg()
        msg.command_id = f'gui_{now.nanoseconds}'
        msg.joint_names = list(JOINT_NAMES)
        current = tuple(
            self.current_positions[name] for name in JOINT_NAMES
        )
        saturation = saturate_target_by_velocity(
            current,
            positions,
            duration_sec,
        )
        msg.positions = list(saturation.positions)

        seconds = int(duration_sec)
        nanoseconds = int((duration_sec - seconds) * 1e9)
        msg.duration.sec = seconds
        msg.duration.nanosec = nanoseconds

        self.last_command_id = msg.command_id
        self.last_allowed_id = None
        self.last_supervised_duration = None
        self.last_prediction = None
        self.last_decision = None
        self.last_execution = None
        self.last_execution_control = None
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
        self.jog_speed_inputs = []
        self.wheel_intentions = [WheelIntention() for _ in JOINT_NAMES]
        self.decrease_buttons = []
        self.increase_buttons = []
        self.jog_velocity_labels = []
        self.jog_reference_deg = list(INITIAL_POSE_DEG)
        self.jog_velocity_deg_s = [0.0] * len(JOINT_NAMES)
        self.jog_last_input_time = [None] * len(JOINT_NAMES)
        self.jog_last_motion_time = [None] * len(JOINT_NAMES)
        self.pending_command_id = None
        self.pending_since = None
        self.connection_indicators = {}
        self.pose_labels = []
        self.minimum_safe_duration = None
        self.prediction_is_stale = False

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

        self.jog_timer = QTimer(self)
        self.jog_timer.timeout.connect(self.publish_jog_intent)
        self.jog_timer.start(50)

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
            target_slider.installEventFilter(self)
            target_slider.setToolTip(
                'Control manual: coloque el cursor aquí y gire la rueda. '
                'La barra muestra el ángulo medido.'
            )
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
            target_slider.sliderPressed.connect(
                lambda item=index: self.begin_direct_drag(item)
            )
            target_slider.sliderReleased.connect(
                lambda item=index: self.end_direct_drag(item)
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
            self.decrease_buttons.append(decrease_button)
            self.increase_buttons.append(increase_button)

            state_layout.addWidget(name_label, row, 0)
            state_layout.addWidget(current_degrees, row, 1)
            state_layout.addWidget(current_radians, row, 2)
            state_layout.addWidget(target_input, row, 3)
            state_layout.addWidget(adjustment_widget, row, 4)
            state_layout.addWidget(target_radians, row, 5)

        main_layout.addWidget(state_group)

        jog_group = QGroupBox('Control manual articular continuo')
        jog_layout = QGridLayout(jog_group)
        self.jog_mode_button = QPushButton(
            'ACTIVAR CONTROL MANUAL CONTINUO'
        )
        self.jog_mode_button.setCheckable(True)
        self.jog_mode_button.toggled.connect(self.toggle_jog_mode)
        jog_layout.addWidget(self.jog_mode_button, 0, 0, 1, 4)
        jog_note = QLabel(
            'Gire la rueda sobre la barra del joint. La barra muestra el '
            'ángulo medido. Más pulsos = mayor velocidad, limitada por Vmáx '
            'y supervisión. Sin pulsos durante 0.25 s: intención cero. '
            'Sensibilidad: 1° por paso estándar de rueda; ventana: 0.25 s.'
        )
        jog_note.setWordWrap(True)
        jog_layout.addWidget(jog_note, 1, 0, 1, 4)
        for index, joint_label in enumerate(JOINT_LABELS):
            row = index + 2
            jog_layout.addWidget(QLabel(joint_label), row, 0)
            jog_layout.addWidget(QLabel('Vmáx:'), row, 1)
            speed_input = QDoubleSpinBox()
            speed_input.setRange(1.0, ALLOWED_SPEED_DEG[index])
            speed_input.setDecimals(1)
            speed_input.setSingleStep(1.0)
            speed_input.setValue(ALLOWED_SPEED_DEG[index])
            speed_input.setSuffix(' °/s')
            self.jog_speed_inputs.append(speed_input)
            jog_layout.addWidget(speed_input, row, 2)
            velocity_label = QLabel('v mouse: 0.0 °/s')
            velocity_label.setStyleSheet('font-weight: bold; color: #166534;')
            self.jog_velocity_labels.append(velocity_label)
            jog_layout.addWidget(velocity_label, row, 3)
        main_layout.addWidget(jog_group)

        connection_group = QGroupBox('Conexiones de la simulación')
        connection_layout = QGridLayout(connection_group)

        for name in [
            'Gazebo /clock',
            '/joint_states',
            'Supervisor',
            'Control manual continuo',
            'Salida supervisada',
            'Predicción geométrica',
            'Referencia de ejecución',
            'Control preventivo',
        ]:
            indicator = QLabel(f'● {name}')
            indicator.setStyleSheet('color: #b91c1c; font-weight: bold;')
            self.connection_indicators[name] = indicator
            index = len(self.connection_indicators) - 1
            connection_layout.addWidget(indicator, index // 3, index % 3)

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
            'Estimación con límites operativos comunes de 18/24 °/s. '
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

        prediction_group = QGroupBox(
            'Evaluación geométrica preventiva del supervisor'
        )
        prediction_layout = QVBoxLayout(prediction_group)
        self.prediction_state_label = QLabel('SIN EVALUAR')
        self.prediction_state_label.setObjectName('predictionState')
        self.prediction_detail_label = QLabel(
            'Envíe un comando candidato para evaluar su trayectoria.'
        )
        self.prediction_detail_label.setWordWrap(True)
        prediction_layout.addWidget(self.prediction_state_label)
        prediction_layout.addWidget(self.prediction_detail_label)
        self.execution_status_label = QLabel(
            'Ejecución: sin referencia aceptada por Gazebo.'
        )
        self.execution_status_label.setWordWrap(True)
        prediction_layout.addWidget(self.execution_status_label)
        main_layout.addWidget(prediction_group)

        metrics_group = QGroupBox('Distancias y horizonte durante la ejecución')
        metrics_layout = QGridLayout(metrics_group)
        self.metric_labels = {}
        for index, name in enumerate([
            'Distancia actual', 'Mínimo proyectado', 'Instante del mínimo',
            'Decisión preventiva',
        ]):
            metrics_layout.addWidget(QLabel(name), 0, index)
            label = QLabel('--')
            label.setWordWrap(True)
            label.setStyleSheet('font-size: 16px; font-weight: bold;')
            self.metric_labels[name] = label
            metrics_layout.addWidget(label, 1, index)
        note = QLabel(
            'Distancias entre superficies del modelo geométrico. '
            't = 0: estado actual; t > 0: configuración futura. '
            'Las mediciones llegan por separado; no son una pareja sincronizada.'
        )
        note.setWordWrap(True)
        metrics_layout.addWidget(note, 2, 0, 1, 4)
        main_layout.insertWidget(2, metrics_group)

        self.preview_button = QPushButton(
            'ACTIVAR VOLUMEN NOMINAL CONTINUO'
        )
        self.preview_button.setCheckable(True)
        main_layout.addWidget(self.preview_button)
        self.preview_note = QLabel(
            'Volumen azul: intención nominal, horizonte 1 s y actualización '
            'solicitada a 10 Hz. Durante la ejecución, el volumen cambia de '
            'color y el supervisor puede reducir la velocidad o detener '
            'el goal activo de Gazebo.'
        )
        self.preview_note.setWordWrap(True)
        main_layout.addWidget(self.preview_note)
        self.preview_timer = QTimer(self)
        self.preview_timer.timeout.connect(self.publish_preview_intent)
        self.preview_timer.start(100)

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
            '#predictionState {'
            '  font-size: 18px; font-weight: bold; padding: 6px;'
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
        jog_active = self.jog_mode_button.isChecked()
        busy = (
            self.pending_command_id is not None
            or self.ros_node.execution_is_active()
        )
        connected = self.ros_node.candidate_publisher.get_subscription_count() > 0
        self.send_button.setEnabled(
            state_ready and connected and not busy and not jog_active
        )
        self.send_button.setText(
            'ESPERANDO RESULTADO DEL COMANDO' if busy
            else 'ENVIAR COMANDO CANDIDATO'
        )
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
            if jog_active:
                actual_degrees = math.degrees(position)
                self.jog_reference_deg[index] = actual_degrees
                self.set_joint_reference_display(index, actual_degrees)

            velocity = self.active_jog_velocity(index, time.monotonic())
            self.jog_velocity_labels[index].setText(
                f'Intención: {velocity:+.1f} °/s | medida: '
                + (
                    f'{math.degrees(self.ros_node.current_velocities[joint_name]):+.1f} °/s'
                    if state_ready and joint_name in self.ros_node.current_velocities
                    else 'sin dato'
                )
            )

        self.refresh_connections()
        self.refresh_end_effector_pose()
        self.refresh_velocity_preview()
        self.refresh_prediction_status()
        self.refresh_execution_status()
        self.refresh_command_status()
        self.refresh_runtime_metrics()

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
                'VELOCIDAD: PROBABLE ALLOW | '
                f'Joint limitante: {limiting_joint} | '
                f'Duración mínima estimada: {minimum_duration:.2f} s'
            )
            self.velocity_summary_label.setStyleSheet(
                'color: #166534;'
            )
        else:
            self.velocity_summary_label.setText(
                'VELOCIDAD: PROBABLE REJECT | '
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
            indicator = self.connection_indicators.get(name)
            if indicator is None:
                continue
            color = '#166534' if connected else '#b91c1c'
            text = '●' if connected else '○'
            indicator.setText(f'{text} {name}')
            indicator.setStyleSheet(
                f'color: {color}; font-weight: bold;'
            )

    def refresh_runtime_metrics(self):
        node = self.ros_node
        now = time.monotonic()
        proximity = node.last_proximity
        if proximity is not None and now - node.proximity_received_at <= 1.0:
            actual = f'{proximity.minimum_clearance:.3f} m'
        else:
            actual = 'Sin dato reciente'
        self.metric_labels['Distancia actual'].setText(actual)
        control = node.last_execution_control
        active = node.execution_by_id.get(
            control.command_id if control is not None else ''
        )
        fresh = (
            control is not None
            and node.control_received_at is not None
            and now - node.control_received_at <= 1.0
            and (
                (
                    active is not None
                    and active.status in ('PENDING', 'ACCEPTED')
                )
                or (
                    self.jog_mode_button.isChecked()
                    and control.command_id == 'jog_continuous'
                )
            )
        )
        values = ['Sin evaluación activa', '--', '--']
        color = '#475569'
        if fresh:
            values = [
                f'{control.minimum_clearance:.3f} m',
                (f'+{control.minimum_time_from_now:.2f} s | '
                 f'{control.minimum_sample_index + 1}/'
                 f'{control.minimum_sample_count}')
                if control.minimum_sample_count >= 2 else 'Sin muestra válida',
                f'{control.state} | escala {control.speed_scale:.2f}',
            ]
            color = {
                'ALLOW': '#166534', 'WARNING': '#a16207',
                'REDUCTION': '#c2410c', 'STOP': '#b91c1c',
            }.get(control.state, '#475569')
        for name, value in zip([
            'Mínimo proyectado', 'Instante del mínimo', 'Decisión preventiva',
        ], values):
            self.metric_labels[name].setText(value)
            self.metric_labels[name].setStyleSheet(
                f'color: {color}; font-size: 16px; font-weight: bold;'
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

    def refresh_prediction_status(self):
        if self.prediction_is_stale:
            self.prediction_state_label.setText(
                'GEOMETRÍA PROYECTADA: OBJETIVO SIN EVALUAR'
            )
            self.prediction_state_label.setStyleSheet(
                'color: #92400e;'
            )
            self.prediction_detail_label.setText(
                'El objetivo articular cambió. Envíe el comando candidato '
                'para calcular una nueva predicción geométrica.'
            )
            return

        decision = self.ros_node.last_decision
        if (
            decision is not None
            and decision.command_id == self.ros_node.last_command_id
            and not decision.accepted
        ):
            self.prediction_state_label.setText(
                'COMANDO RECHAZADO POR EL SUPERVISOR'
            )
            self.prediction_state_label.setStyleSheet('color: #b91c1c;')
            self.prediction_detail_label.setText(decision.reason)
            return

        prediction = self.ros_node.last_prediction

        if prediction is None:
            return

        colors = {
            'ALLOW': '#166534',
            'WARNING': '#a16207',
            'REDUCTION': '#c2410c',
            'STOP': '#b91c1c',
        }
        state = prediction.state
        color = colors.get(state, '#334155')
        sample_number = min(
            prediction.sample_index + 1,
            prediction.sample_count,
        )

        self.prediction_state_label.setText(
            f'GEOMETRÍA PROYECTADA: {state}'
        )
        self.prediction_state_label.setStyleSheet(f'color: {color};')
        self.prediction_detail_label.setText(
            f'Distancia mínima: {prediction.minimum_clearance:.3f} m | '
            f'Segmento: {prediction.limiting_segment} | '
            f'Muestra: {sample_number}/{prediction.sample_count} | '
            f'Avance: {prediction.trajectory_fraction:.0%}'
        )

    def refresh_execution_status(self):
        execution = self.ros_node.last_execution
        if execution is None:
            self.execution_status_label.setText(
                'Ejecución: sin referencia aceptada por Gazebo.'
            )
            self.execution_status_label.setStyleSheet('color: #92400e;')
            return

        colors = {
            'PENDING': '#92400e',
            'ACCEPTED': '#166534',
            'SUCCEEDED': '#166534',
            'FAILED': '#b91c1c',
            'REJECTED': '#b91c1c',
            'CANCELED': '#b91c1c',
            'DRY_RUN': '#92400e',
        }
        color = colors.get(execution.status, '#334155')
        duration = execution.duration_sec
        control = self.ros_node.last_execution_control
        control_text = ''
        if (
            control is not None
            and control.command_id == execution.command_id
        ):
            control_color = {
                'ALLOW': '#166534',
                'WARNING': '#a16207',
                'REDUCTION': '#c2410c',
                'STOP': '#b91c1c',
            }.get(control.state, '#334155')
            minimum_text = ''
            if control.minimum_sample_count >= 2:
                sample_number = min(
                    control.minimum_sample_index + 1,
                    control.minimum_sample_count,
                )
                minimum_text = (
                    f' Mínimo proyectado en t='
                    f'{control.minimum_time_from_now:.2f} s '
                    f'(muestra {sample_number}/'
                    f'{control.minimum_sample_count}).'
                )
            ttc = (
                f'{control.time_to_collision:.3f} s'
                if control.time_to_collision >= 0.0
                else 'sin solapamiento previsto'
            )
            control_text = (
                f' Control: {control.state} '
                f'(escala {control.speed_scale:.2f}, '
                f'd_min {control.minimum_clearance:.3f} m, '
                f'TTC: {ttc}).'
                f'{minimum_text}'
            )
            if execution.status in (
                'PENDING', 'ACCEPTED',
            ):
                color = control_color
        self.execution_status_label.setText(
            f'Ejecución {execution.status}: {execution.command_id}. '
            f'{execution.detail} Duración: {duration:.2f} s.'
            f'{control_text}'
        )
        self.execution_status_label.setStyleSheet(f'color: {color};')

    def refresh_command_status(self):
        for execution in self.ros_node.execution_by_id.values():
            self.update_history_state(execution.command_id, execution.status)
        execution = self.ros_node.execution_by_id.get(
            self.ros_node.last_command_id
        )
        if execution is not None:
            self.status_label.setText(
                f'{execution.status}: {execution.command_id}. {execution.detail}'
            )
            self.status_label.setStyleSheet(
                'color: #b91c1c;' if execution.status in (
                    'REJECTED', 'FAILED', 'CANCELED',
                ) else 'color: #334155;'
            )
            if execution.command_id == self.pending_command_id:
                self.pending_command_id = None
            return
        if self.pending_command_id is None:
            return

        decision = self.ros_node.last_decision
        if (
            decision is not None
            and decision.command_id == self.pending_command_id
            and not decision.accepted
        ):
            self.status_label.setText(
                f'RECHAZADO: {self.pending_command_id}. {decision.reason}'
            )
            self.status_label.setStyleSheet('color: #b91c1c;')
            self.update_history_state(
                self.pending_command_id,
                f'RECHAZADO: {decision.reason_code}',
            )
            self.pending_command_id = None
            return

        prediction = self.ros_node.last_prediction
        prediction_matches = (
            prediction is not None
            and prediction.command_id == self.pending_command_id
        )

        if prediction_matches and prediction.state == 'STOP':
            self.status_label.setText(
                f'STOP PREDICTIVO: {self.pending_command_id} rechazado. '
                f'Distancia mínima '
                f'{prediction.minimum_clearance:.3f} m en '
                f'{prediction.limiting_segment}.'
            )
            self.status_label.setStyleSheet('color: #b91c1c;')
            self.update_history_state(
                self.pending_command_id,
                'STOP PREDICTIVO',
            )
            self.pending_command_id = None
            return

        if self.ros_node.last_allowed_id == self.pending_command_id:
            state = prediction.state if prediction_matches else 'ALLOW'
            color = {
                'ALLOW': '#166534',
                'WARNING': '#a16207',
                'REDUCTION': '#c2410c',
            }.get(state, '#166534')
            duration = self.ros_node.last_supervised_duration
            duration_text = (
                f' Duración supervisada: {duration:.2f} s.'
                if duration is not None
                else ''
            )
            self.status_label.setText(
                f'{state}: {self.pending_command_id} fue reenviado por '
                f'el supervisor al adaptador de simulación.'
                f'{duration_text}'
            )
            self.status_label.setStyleSheet(f'color: {color};')
            self.update_history_state(
                self.pending_command_id,
                state,
            )
            return

        if time.monotonic() - self.pending_since > 5.0:
            self.status_label.setText(
                f'SIN SALIDA SUPERVISADA: {self.pending_command_id}. '
                'No hay confirmación; revise supervisor y adaptador antes '
                'de reiniciar la interfaz.'
            )
            self.status_label.setStyleSheet('color: #b91c1c;')
            self.update_history_state(
                self.pending_command_id,
                'SIN CONFIRMACIÓN',
            )

    def target_spin_changed(self, index, degrees):
        jog_button = getattr(self, 'jog_mode_button', None)
        if jog_button is not None and jog_button.isChecked():
            return
        self.prediction_is_stale = True
        blocker = QSignalBlocker(self.target_sliders[index])
        self.target_sliders[index].setValue(
            int(round(degrees * 10.0))
        )
        del blocker
        self.update_target_radians(index, degrees)

    def target_slider_changed(self, index, slider_value):
        jog_button = getattr(self, 'jog_mode_button', None)
        if jog_button is not None and jog_button.isChecked():
            position = self.ros_node.current_positions.get(JOINT_NAMES[index])
            if position is not None:
                self.set_joint_reference_display(index, math.degrees(position))
            return
        self.target_inputs[index].setValue(slider_value / 10.0)

    def begin_direct_drag(self, index):
        jog_button = getattr(self, 'jog_mode_button', None)
        if jog_button is None or not jog_button.isChecked():
            return
        position = self.ros_node.current_positions.get(JOINT_NAMES[index])
        if position is not None:
            self.jog_reference_deg[index] = math.degrees(position)
        self.jog_velocity_deg_s[index] = 0.0
        self.jog_last_input_time[index] = time.monotonic()
        self.jog_last_motion_time[index] = None

    def end_direct_drag(self, index):
        jog_button = getattr(self, 'jog_mode_button', None)
        if jog_button is None or not jog_button.isChecked():
            return
        self.jog_velocity_deg_s[index] = 0.0
        self.jog_last_input_time[index] = None
        self.jog_last_motion_time[index] = None
        position = self.ros_node.current_positions.get(JOINT_NAMES[index])
        if position is not None:
            degrees = math.degrees(position)
            self.jog_reference_deg[index] = degrees
            self.set_joint_reference_display(index, degrees)

    def jog_joint(self, index, increment_degrees):
        if self.jog_mode_button.isChecked():
            return
        target_input = self.target_inputs[index]
        target_input.setValue(
            target_input.value() + increment_degrees
        )

    def update_target_radians(self, index, degrees):
        radians = math.radians(degrees)
        self.target_radian_labels[index].setText(f'{radians:.4f}')

    def set_joint_reference_display(self, index, degrees):
        input_blocker = QSignalBlocker(self.target_inputs[index])
        slider_blocker = QSignalBlocker(self.target_sliders[index])
        self.target_inputs[index].setValue(degrees)
        self.target_sliders[index].setValue(
            int(round(degrees * 10.0))
        )
        del input_blocker
        del slider_blocker
        self.update_target_radians(index, degrees)

    def reset_direct_jog_state(self):
        now = time.monotonic()
        for index, name in enumerate(JOINT_NAMES):
            position = self.ros_node.current_positions.get(name)
            if position is None:
                continue
            degrees = math.degrees(position)
            self.jog_reference_deg[index] = degrees
            self.jog_velocity_deg_s[index] = 0.0
            self.jog_last_input_time[index] = now
            self.jog_last_motion_time[index] = None
            self.set_joint_reference_display(index, degrees)

    def active_jog_velocity(self, index, now=None):
        now = time.monotonic() if now is None else now
        if not self.jog_mode_button.isChecked() or not self.ros_node.state_is_ready():
            self.wheel_intentions[index].clear()
            return 0.0
        return self.wheel_intentions[index].velocity(
            now, self.jog_speed_inputs[index].value(),
        )

    def eventFilter(self, watched, event):
        button = getattr(self, 'jog_mode_button', None)
        if button is not None and button.isChecked() and watched in self.target_sliders:
            index = self.target_sliders.index(watched)
            if event.type() == QEvent.Wheel:
                if self.ros_node.state_is_ready():
                    # Qt angleDelta: 120 units per conventional wheel notch.
                    self.wheel_intentions[index].add(
                        time.monotonic(), event.angleDelta().y() / 120.0,
                    )
                event.accept()
                return True
            if event.type() in (
                QEvent.MouseButtonPress, QEvent.MouseButtonRelease,
                QEvent.MouseButtonDblClick, QEvent.MouseMove,
                QEvent.KeyPress, QEvent.KeyRelease,
            ):
                return True
            if event.type() in (QEvent.Leave, QEvent.FocusOut):
                self.wheel_intentions[index].clear()
        return super().eventFilter(watched, event)

    def changeEvent(self, event):
        if event.type() == QEvent.ActivationChange and not self.isActiveWindow():
            for intention in getattr(self, 'wheel_intentions', []):
                intention.clear()
        super().changeEvent(event)

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

    def toggle_jog_mode(self, checked):
        for intention in self.wheel_intentions:
            intention.clear()
        if checked and (
            not self.ros_node.state_is_ready()
            or self.ros_node.execution_is_active()
            or self.pending_command_id is not None
            or self.ros_node.jog_intent_publisher.get_subscription_count() == 0
        ):
            blocker = QSignalBlocker(self.jog_mode_button)
            self.jog_mode_button.setChecked(False)
            blocker.unblock()
            QMessageBox.warning(
                self,
                'Control manual no disponible',
                'Se requiere /joint_states reciente y ninguna trayectoria '
                'por objetivo en ejecución.',
            )
            return

        if checked:
            self.copy_current_pose()
            self.reset_direct_jog_state()
            self.preview_button.setChecked(False)
            self.preview_button.setEnabled(False)
            self.duration_input.setEnabled(False)
            for target_input in self.target_inputs:
                target_input.setEnabled(False)
            for button in self.decrease_buttons + self.increase_buttons:
                button.setEnabled(False)
            self.jog_mode_button.setText(
                'DESACTIVAR CONTROL MANUAL CONTINUO'
            )
            self.status_label.setText(
                'CONTROL POR RUEDA ACTIVO: gire la rueda sobre una barra. '
                'La barra muestra el ángulo medido; no deja destinos pendientes.'
            )
            self.status_label.setStyleSheet('color: #166534;')
        else:
            for index in range(len(JOINT_NAMES)):
                self.jog_velocity_deg_s[index] = 0.0
                self.jog_last_motion_time[index] = None
            self.publish_jog_intent(force_hold=True)
            self.preview_button.setEnabled(True)
            self.duration_input.setEnabled(True)
            for target_input in self.target_inputs:
                target_input.setEnabled(True)
            for button in self.decrease_buttons + self.increase_buttons:
                button.setEnabled(True)
            self.jog_mode_button.setText(
                'ACTIVAR CONTROL MANUAL CONTINUO'
            )
            self.status_label.setText(
                'Control manual desactivado; se solicitó mantener la '
                'postura actual.'
            )
            self.status_label.setStyleSheet('color: #334155;')

    def publish_jog_intent(self, force_hold=False):
        if not force_hold and not self.jog_mode_button.isChecked():
            return
        if not self.ros_node.state_is_ready():
            return
        if self.ros_node.jog_intent_publisher.get_subscription_count() == 0:
            return

        horizon = 1.0
        current = [
            self.ros_node.current_positions[name]
            for name in JOINT_NAMES
        ]
        horizon_target = []
        for index, current_value in enumerate(current):
            if force_hold:
                horizon_target.append(current_value)
                continue
            velocity_deg_s = self.active_jog_velocity(index)
            maximum_speed = self.jog_speed_inputs[index].value()
            velocity_deg_s = min(
                max(velocity_deg_s, -maximum_speed),
                maximum_speed,
            )
            lower, upper = map(math.radians, JOINT_LIMITS_DEG[index])
            delta = math.radians(velocity_deg_s) * horizon
            # Do not command a jump back into range if the measured joint is outside.
            if delta > 0:
                delta = min(delta, max(0.0, upper - current_value))
            elif delta < 0:
                delta = max(delta, min(0.0, lower - current_value))
            horizon_target.append(current_value + delta)

        msg = JointCommand()
        msg.stamp = self.ros_node.get_clock().now().to_msg()
        msg.command_id = 'jog_continuous'
        msg.joint_names = list(JOINT_NAMES)
        msg.positions = horizon_target
        msg.duration.sec = 1
        msg.duration.nanosec = 0
        self.ros_node.jog_intent_publisher.publish(msg)

    def publish_preview_intent(self):
        if self.jog_mode_button.isChecked():
            return
        if not self.preview_button.isChecked():
            return
        if not self.ros_node.state_is_ready():
            return
        msg = JointCommand()
        msg.stamp = self.ros_node.get_clock().now().to_msg()
        msg.command_id = 'continuous_preview'
        msg.joint_names = list(JOINT_NAMES)
        msg.positions = [
            math.radians(item.value()) for item in self.target_inputs
        ]
        duration = self.duration_input.value()
        msg.duration.sec = int(duration)
        msg.duration.nanosec = int((duration - int(duration)) * 1e9)
        self.ros_node.intent_publisher.publish(msg)

    def send_candidate(self):
        if self.jog_mode_button.isChecked():
            return
        if (
            self.pending_command_id is not None
            or self.ros_node.execution_is_active()
        ):
            return
        if self.ros_node.candidate_publisher.get_subscription_count() == 0:
            self.status_label.setText('Inicie el supervisor antes de enviar.')
            return
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
        self.prediction_is_stale = False
        self.prediction_state_label.setText(
            'GEOMETRÍA PROYECTADA: EVALUANDO...'
        )
        self.prediction_state_label.setStyleSheet('color: #92400e;')
        self.prediction_detail_label.setText(
            f'Esperando la predicción para {command_id}.'
        )
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
        if self.jog_mode_button.isChecked():
            self.publish_jog_intent(force_hold=True)
        self.jog_timer.stop()
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
