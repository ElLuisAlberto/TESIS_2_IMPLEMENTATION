"""Observe one live scenario and append a traceable validation record."""

import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path
import math
import re
import time
from typing import Any, Dict, Iterable, Optional, Set, Union

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import JointState
from thesis_core.joint_model import JOINT_NAMES, JOINT_VELOCITY_LIMITS

from .contracts import CRITICAL_FIELDS, SCENARIOS
from .evaluation import evaluate


def _walk(message: Any, names: Iterable[str]) -> Optional[Any]:
    for name in names:
        if hasattr(message, name):
            return getattr(message, name)
    return None


def _ros_time_seconds(message: Any) -> Optional[float]:
    if message is None:
        return None
    if not hasattr(message, "sec") or not hasattr(message, "nanosec"):
        return None
    return float(message.sec) + float(message.nanosec) * 1e-9


class ScenarioRecorder(Node):
    """Collect evidence without adding dependencies to production nodes."""

    _DYNAMIC_TOPICS = {
        "/thesis/execution_control",
        "/thesis/proximity_status",
        "/thesis/pipeline_timing",
        "/thesis/command_decision",
        "/thesis/execution_trajectory",
        "/thesis/jog_intent",
        "/arm_controller/joint_trajectory",
        "/thesis/joint_trajectory_prediction",
    }

    def __init__(self, scenario_id: str, repetition: int) -> None:
        super().__init__("advance2_scenario_recorder")
        self.scenario_id = scenario_id
        self.repetition = repetition
        self.latest: Dict[str, Any] = {}
        self.max_velocity = 0.0
        self.velocity_seen = False
        self.controls = []
        self.control_samples: list[Dict[str, Any]] = []
        self.command_id = "UNAVAILABLE"
        self.controller_status = "UNAVAILABLE"
        self.expected_segment = "UNAVAILABLE"
        self.last_jog_time: Optional[float] = None
        self.last_joint_positions: Optional[list] = None
        self.jog_hold_age_ms = -1.0
        self._pending_hold_age_ms = -1.0
        self.horizon_update_ms = -1.0
        self.saturation_confirmed = False
        self.joint_velocity_checked = False
        self.joint_velocity_violation = False
        self.last_jog_deltas: Optional[tuple] = None
        self.reversal: Optional[tuple] = None
        self._last_positions: Optional[list] = None
        self._last_joint_time: Optional[float] = None
        self._dynamic_subscription_handles = []
        self._subscribed_topics: Set[str] = set()
        self._execution_received = False
        self._start_monotonic = time.monotonic()
        self.create_subscription(
            JointState, "/joint_states", self._joint_cb, 20
        )
        self._discovery_timer = self.create_timer(
            0.25, self._connect_dynamic_topics
        )
        self._connect_dynamic_topics()

    def _connect_dynamic_topics(self) -> None:
        available = dict(self.get_topic_names_and_types())
        for topic in self._DYNAMIC_TOPICS - self._subscribed_topics:
            types = available.get(topic, [])
            if not types:
                continue
            try:
                message_type = get_message(types[0])
                qos: Union[int, QoSProfile] = 20
                if topic == "/arm_controller/joint_trajectory":
                    qos = QoSProfile(
                        history=HistoryPolicy.KEEP_LAST,
                        depth=20,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                    )
                subscription = self.create_subscription(
                    message_type,
                    topic,
                    lambda msg, source=topic: self._generic_cb(source, msg),
                    qos,
                )
            except (AttributeError, ImportError, RuntimeError) as error:
                self.get_logger().error(
                    f"No se pudo suscribir a {topic}: {error}"
                )
                continue
            self._dynamic_subscription_handles.append(subscription)
            self._subscribed_topics.add(topic)
            self.get_logger().info(f"Suscripción activa: {topic}")
        if self._subscribed_topics == self._DYNAMIC_TOPICS:
            self._discovery_timer.cancel()
        elif time.monotonic() - self._start_monotonic > 2.0:
            missing = sorted(
                self._DYNAMIC_TOPICS - self._subscribed_topics
            )
            self.get_logger().warning(
                "Temas aún no descubiertos: " + ", ".join(missing)
            )

    def _joint_cb(self, message: JointState) -> None:
        positions_by_name = dict(zip(message.name, message.position))
        if all(name in positions_by_name for name in JOINT_NAMES):
            self.last_joint_positions = [
                float(positions_by_name[name]) for name in JOINT_NAMES
            ]
        if message.velocity:
            velocity_by_name = dict(zip(message.name, message.velocity))
            if all(name in velocity_by_name for name in JOINT_NAMES):
                measured = max(
                    abs(velocity_by_name[name]) for name in JOINT_NAMES
                )
                self.max_velocity = max(self.max_velocity, measured)
                self.velocity_seen = True
                self.joint_velocity_checked = True
                self.joint_velocity_violation |= any(
                    not math.isfinite(velocity_by_name[name])
                    or abs(velocity_by_name[name]) > (
                        JOINT_VELOCITY_LIMITS[name] + 0.02
                    )
                    for name in JOINT_NAMES
                )
            return
        now = time.monotonic()
        positions = list(message.position)
        if (self._last_positions is not None
                and self._last_joint_time is not None):
            delta = now - self._last_joint_time
            if delta > 1e-6 and len(positions) == len(self._last_positions):
                measured = max(
                    abs(current - previous) / delta
                    for current, previous in zip(
                        positions, self._last_positions
                    )
                )
                self.max_velocity = max(self.max_velocity, measured)
                self.velocity_seen = True
        self._last_positions = positions
        self._last_joint_time = now

    def _execution_cb(self, message: Any) -> None:
        reason = str(_walk(message, ("reason",)) or "")
        match = re.search(
            r'joint_lim=(\w+); v_req=([-+\d.]+)rad/s; '
            r'v_lim=([-+\d.]+)rad/s', reason,
        )
        if match and match.group(1) in JOINT_VELOCITY_LIMITS:
            requested = float(match.group(2))
            limited = float(match.group(3))
            limit = JOINT_VELOCITY_LIMITS[match.group(1)]
            if (abs(requested) > abs(limited) + 0.0001
                    and abs(abs(limited) - limit) < 0.002):
                self.saturation_confirmed = True
        mappings = {
            "state": ("state",),
            "speed_scale": ("speed_scale",),
            "d_actual": ("current_clearance", "minimum_clearance"),
            "d_nominal": ("nominal_clearance", "minimum_clearance"),
            "d_supervisada": (
                "supervised_clearance",
                "minimum_clearance",
            ),
            "ttc_nominal": (
                "time_to_collision",
            ),
            "t_min": ("minimum_time_from_now",),
            "limiting_segment": ("limiting_segment",),
            "reason_code": ("reason_code",),
        }
        for key, names in mappings.items():
            value = _walk(message, names)
            if value is not None:
                self.latest[key] = value
        self.command_id = str(_walk(message, ("command_id",)) or "UNAVAILABLE")
        self.controls.append((
            str(self.latest.get("state", "UNKNOWN")),
            float(self.latest.get("speed_scale", -1.0)),
            float(self.latest.get("d_nominal", -1.0)),
        ))
        sample = {
            key: self.latest.get(key) for key in mappings
        }
        sample['command_id'] = message.command_id
        self.control_samples.append(sample)
        self._execution_received = True

    def _proximity_cb(self, message: Any) -> None:
        if self._execution_received:
            return
        state = _walk(message, ("state",))
        clearance = _walk(message, ("minimum_clearance",))
        segment = _walk(message, ("limiting_segment",))
        if state is not None:
            self.latest["state"] = state
        if clearance is not None:
            self.latest["d_actual"] = clearance
        if segment is not None:
            self.latest["limiting_segment"] = segment

    def _timing_cb(self, message: Any) -> None:
        stage = str(_walk(message, ("stage",)) or "").upper()
        stamp = _ros_time_seconds(_walk(message, ("stamp",)))
        intent_stamp = _ros_time_seconds(
            _walk(message, ("intent_stamp",))
        )
        if stage == "CONTROLLER_PUBLISH":
            if stamp is not None and intent_stamp is not None:
                latency_ms = (stamp - intent_stamp) * 1000.0
                if latency_ms >= 0.0:
                    self.latest["latency_end_to_end_ms"] = latency_ms
        if (stage == "STOP_DETECTED"
                and str(message.detail) == "JOG_WATCHDOG_EXPIRED"):
            self.latest["reason_code"] = "JOG_WATCHDOG_EXPIRED"
            if (self.scenario_id == "E11"
                    and self._pending_hold_age_ms >= 0.0
                    and self.jog_hold_age_ms < 0.0):
                self.jog_hold_age_ms = self._pending_hold_age_ms
                self.latest["state"] = "HOLD"
        self.latest["last_timing_stage"] = stage

    def _generic_cb(self, source: str, message: Any) -> None:
        if source == "/thesis/execution_control":
            self._execution_cb(message)
        elif source == "/thesis/proximity_status":
            self._proximity_cb(message)
        elif source == "/thesis/pipeline_timing":
            self._timing_cb(message)
        elif source == "/thesis/command_decision":
            self.command_id = str(message.command_id)
            if not message.accepted:
                self.controller_status = "REJECTED"
                self.latest["state"] = "REJECTED"
                self.latest["reason_code"] = message.reason_code
        elif source == "/thesis/execution_trajectory":
            self.controller_status = str(message.status)
        elif source == "/thesis/jog_intent":
            self.last_jog_time = time.monotonic()
            self._pending_hold_age_ms = -1.0
            self.command_id = str(message.command_id)
            if (self.last_joint_positions is not None
                    and tuple(message.joint_names) == JOINT_NAMES
                    and len(message.positions) == 6):
                deltas = tuple(
                    goal - current for goal, current in zip(
                        message.positions, self.last_joint_positions,
                    )
                )
                if self.last_jog_deltas is not None:
                    for index, (previous, current) in enumerate(zip(
                            self.last_jog_deltas, deltas)):
                        if previous * current < -1e-6:
                            self.reversal = (
                                time.monotonic(), str(message.command_id),
                                index, 1 if current > 0 else -1,
                            )
                            break
                self.last_jog_deltas = deltas
        elif source == "/thesis/joint_trajectory_prediction":
            if (self.reversal is not None
                    and message.source == "jog"
                    and message.command_id == self.reversal[1]
                    and message.sample_count >= 2
                    and len(message.positions) >= 12):
                index = self.reversal[2]
                delta = (
                    message.positions[6 + index] - message.positions[index]
                )
                if delta * self.reversal[3] > 0.0:
                    self.horizon_update_ms = (
                        time.monotonic() - self.reversal[0]
                    ) * 1000.0
                    self.reversal = None
        elif source == "/arm_controller/joint_trajectory":
            if (self.last_jog_time is not None
                    and self.last_joint_positions is not None
                    and len(message.points) == 1
                    and len(message.points[0].positions) == 6
                    and len(message.points[0].velocities) == 6
                    and max(
                        abs(v) for v in message.points[0].velocities
                    ) < 1e-9
                    and all(abs(a - b) < 0.01 for a, b in zip(
                        message.points[0].positions,
                        self.last_joint_positions,
                    ))):
                if self.scenario_id == "E11":
                    if self._pending_hold_age_ms < 0.0:
                        self._pending_hold_age_ms = (
                            time.monotonic() - self.last_jog_time
                        ) * 1000.0
                    if (self.jog_hold_age_ms < 0.0
                            and self.latest.get("reason_code")
                            == "JOG_WATCHDOG_EXPIRED"):
                        self.jog_hold_age_ms = self._pending_hold_age_ms
                        self.latest["state"] = "HOLD"

    def record(self) -> Dict[str, object]:
        """Build one normalized row after the observation interval."""
        selected = self.latest
        if self.scenario_id == 'E03':
            candidates = [
                sample for sample in self.control_samples
                if sample['ttc_nominal'] is not None
                and float(sample['ttc_nominal']) >= 0.0
            ]
            if candidates:
                selected = min(candidates, key=lambda sample: float(
                    sample['d_nominal']
                ))
        elif self.scenario_id in {'E04', 'E05'}:
            candidates = [
                sample for sample in self.control_samples
                if sample['state'] == 'REDUCTION'
                and sample['d_nominal'] is not None
                and float(sample['d_nominal']) < 0.02
                and sample['d_supervisada'] is not None
                and float(sample['d_supervisada']) >= 0.02 - 1e-6
            ]
            if candidates:
                selected = min(candidates, key=lambda sample: float(
                    sample['speed_scale']
                ))
        elif self.scenario_id == 'E06':
            candidates = [
                sample for sample in self.control_samples
                if sample['state'] == 'STOP'
                and sample['d_actual'] is not None
                and float(sample['d_actual']) <= 0.05
            ]
            if candidates:
                selected = candidates[-1]
        state = str(selected.get("state", "UNKNOWN"))
        if (self.scenario_id == "E11" and self.jog_hold_age_ms >= 0.0
                and self.latest.get("reason_code") == "JOG_WATCHDOG_EXPIRED"):
            state = "HOLD"
        row: Dict[str, object] = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "scenario_id": self.scenario_id,
            "repetition": self.repetition,
            "expected": "|".join(
                sorted(SCENARIOS[self.scenario_id].expected_states)
            ),
            "observed": state,
            "command_id": selected.get("command_id", self.command_id),
            "d_actual": selected.get("d_actual", -1.0),
            "d_nominal": selected.get("d_nominal", -1.0),
            "d_supervisada": selected.get("d_supervisada", -1.0),
            "ttc_nominal": selected.get("ttc_nominal", -1.0),
            "t_min": selected.get("t_min", -1.0),
            "speed_scale": selected.get("speed_scale", -1.0),
            "qdot_medida_max": (
                self.max_velocity if self.velocity_seen else -1.0
            ),
            "latency_end_to_end_ms": self.latest.get(
                "latency_end_to_end_ms", -1.0
            ),
            "limiting_segment": selected.get(
                "limiting_segment", "NONE"
            ),
            "reason_code": selected.get(
                "reason_code", "UNAVAILABLE"
            ),
            "observation_count": len(self.controls),
            "d_nominal_decreased": str(any(
                later[2] < earlier[2] - 1e-6
                for earlier, later in zip(self.controls, self.controls[1:])
            )).lower(),
            "scale_decreased": str(any(
                later[1] < earlier[1] - 1e-6
                for earlier, later in zip(self.controls, self.controls[1:])
            )).lower(),
            "scale_recovered": str(any(
                later[1] > earlier[1] + 1e-6
                for earlier, later in zip(self.controls, self.controls[1:])
            )).lower(),
            "controller_status": self.controller_status,
            "expected_segment": self.expected_segment,
            "jog_hold_age_ms": self.jog_hold_age_ms,
            "horizon_update_ms": self.horizon_update_ms,
            "saturation_confirmed": str(
                self.saturation_confirmed
            ).lower(),
            "joint_velocity_violation": str(
                self.joint_velocity_violation
            ).lower(),
            "joint_velocity_checked": str(
                self.joint_velocity_checked
            ).lower(),
            "verdict": "PENDING",
        }
        result = evaluate(row)
        row["verdict"] = (
            "PASS"
            if result.passed
            else "FAIL:" + "|".join(result.reasons)
        )
        return row


def _append(path: Path, row: Dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists() and path.stat().st_size > 0
    with path.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(CRITICAL_FIELDS)
        )
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def main() -> None:
    """Observe the running pipeline for a bounded interval."""
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario_id", choices=sorted(SCENARIOS))
    parser.add_argument("repetition", type=int)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--expected-segment", default="UNAVAILABLE")
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            Path.home()
            / "TESIS_AVANCE2_EVIDENCIAS"
            / "matriz_escenarios.csv"
        ),
    )
    parsed, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = ScenarioRecorder(parsed.scenario_id, parsed.repetition)
    node.expected_segment = parsed.expected_segment
    deadline = time.monotonic() + parsed.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    row = node.record()
    _append(parsed.output, row)
    print("TEMAS_SUSCRITOS=" + ",".join(sorted(node._subscribed_topics)))
    print("REGISTRO=" + str(row))
    print("RESULTADO_ESCENARIO=" + str(row["verdict"]))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
