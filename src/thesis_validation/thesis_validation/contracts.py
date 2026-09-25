"""Formal contracts for the Advance 2 experimental scenarios."""

from dataclasses import dataclass
from typing import Dict, FrozenSet


@dataclass(frozen=True)
class ScenarioSpec:
    """Describe one reproducible experimental scenario."""

    scenario_id: str
    name: str
    expected_states: FrozenSet[str]
    repetitions: int = 5
    requires_latency: bool = False


SCENARIOS: Dict[str, ScenarioSpec] = {
    "E01": ScenarioSpec("E01", "Objeto lejano", frozenset({"ALLOW"})),
    "E02": ScenarioSpec(
        "E02", "Objeto cercano fuera del volumen",
        frozenset({"ALLOW", "WARNING"}),
    ),
    "E03": ScenarioSpec(
        "E03",
        "Movimiento directo hacia el obstáculo",
        frozenset({"ALLOW", "WARNING", "REDUCTION"}),
    ),
    "E04": ScenarioSpec(
        "E04", "Entrada al margen predictivo", frozenset({"REDUCTION"}),
    ),
    "E05": ScenarioSpec(
        "E05", "Aproximación progresiva", frozenset({"REDUCTION"}),
    ),
    "E06": ScenarioSpec("E06", "Distancia crítica", frozenset({"STOP"})),
    "E07": ScenarioSpec(
        "E07", "Movimiento de retirada",
        frozenset({"STOP", "REDUCTION", "WARNING", "ALLOW"}),
    ),
    "E08": ScenarioSpec(
        "E08", "Cambio rápido de dirección",
        frozenset({"ALLOW", "WARNING", "REDUCTION"}),
        repetitions=10, requires_latency=True,
    ),
    "E09": ScenarioSpec(
        "E09", "Pérdida de joint_states", frozenset({"STOP"}),
    ),
    "E10": ScenarioSpec(
        "E10", "Pérdida de proximity_status", frozenset({"STOP"}),
    ),
    "E11": ScenarioSpec(
        "E11", "Caducidad JOG", frozenset({"HOLD"}),
        repetitions=10, requires_latency=True,
    ),
    "E12": ScenarioSpec(
        "E12",
        "Obstáculo frente a las seis cápsulas",
        frozenset({"ALLOW", "WARNING", "REDUCTION", "STOP"}),
        repetitions=30,
    ),
    "E13": ScenarioSpec(
        "E13", "Solicitud superior a Vmax",
        frozenset({"ALLOW", "WARNING", "REDUCTION"}),
    ),
    "E14": ScenarioSpec(
        "E14", "Posición fuera de límites", frozenset({"REJECTED"}),
    ),
    "E15": ScenarioSpec(
        "E15", "Fallo o rechazo del controlador", frozenset({"STOP"}),
    ),
}

CRITICAL_FIELDS = (
    "timestamp", "scenario_id", "repetition", "expected", "observed",
    "command_id", "d_actual", "d_nominal", "d_supervisada",
    "ttc_nominal", "t_min", "speed_scale", "qdot_medida_max",
    "latency_end_to_end_ms", "limiting_segment", "reason_code",
    "observation_count", "d_nominal_decreased", "scale_decreased",
    "scale_recovered", "controller_status", "expected_segment",
    "jog_hold_age_ms", "horizon_update_ms", "verdict",
    "saturation_confirmed", "joint_velocity_violation",
    "joint_velocity_checked",
)
