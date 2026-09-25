#!/usr/bin/env python3
"""Validate package A temporal stability without commanding the robot."""

from __future__ import annotations

import math
import subprocess
from typing import Callable, Iterable

from thesis_core.control_stability import ControlStabilityFilter, StabilityResult


RECOVERY_SAMPLES = 5
MAX_INCREMENT = 0.10
SAMPLE_PERIOD = 0.10
TOLERANCE = 1.0e-9


class Validation:
    """Collect named checks and print machine-readable PASS/FAIL evidence."""

    def __init__(self) -> None:
        self.failures: list[str] = []

    def check(self, condition: bool, name: str, detail: str = "") -> None:
        status = "PASS" if condition else "FAIL"
        suffix = f" | {detail}" if detail else ""
        print(f"{name}={status}{suffix}")
        if not condition:
            self.failures.append(name)


def make_filter() -> ControlStabilityFilter:
    """Build the frozen package-A stability filter."""
    return ControlStabilityFilter(
        recovery_required_samples=RECOVERY_SAMPLES,
        maximum_scale_increment=MAX_INCREMENT,
        recovery_sample_period_sec=SAMPLE_PERIOD,
    )


def describe(step: int, now: float, result: StabilityResult) -> None:
    """Print one decision sample in a compact reproducible format."""
    print(
        f"MUESTRA={step:02d} t={now:.2f} "
        f"solicitado={result.requested_state}/{result.requested_speed_scale:.3f} "
        f"filtrado={result.state}/{result.speed_scale:.3f} "
        f"recovery={result.recovery_count}/{result.recovery_required_count} "
        f"transition={int(result.transition)} "
        f"reason={result.transition_reason}"
    )


def scenario_oscillation(validation: Validation) -> None:
    """Alternate safe and unsafe samples around the predictive threshold."""
    print("\n=== E3.1 OSCILACIÓN ALREDEDOR DEL MARGEN ===")
    control = make_filter()
    result = control.update("jog-osc", "REDUCTION", 0.50, 0.0)
    describe(0, 0.0, result)
    outputs = [result]
    requested = [
        ("ALLOW", 1.0),
        ("REDUCTION", 0.50),
        ("ALLOW", 1.0),
        ("REDUCTION", 0.45),
        ("ALLOW", 1.0),
        ("REDUCTION", 0.45),
        ("ALLOW", 1.0),
        ("REDUCTION", 0.40),
    ]
    for index, (state, scale) in enumerate(requested, 1):
        now = index * SAMPLE_PERIOD
        result = control.update("jog-osc", state, scale, now)
        describe(index, now, result)
        outputs.append(result)

    unsafe_outputs = [outputs[index] for index in (2, 4, 6, 8)]
    validation.check(
        all(item.recovery_count == 0 for item in unsafe_outputs),
        "OSCILACION_REINICIA_RECUPERACION",
    )
    validation.check(
        max(item.speed_scale for item in outputs) <= 0.50 + TOLERANCE,
        "OSCILACION_NO_LIBERA_ESCALA",
        f"escala_max={max(item.speed_scale for item in outputs):.3f}",
    )
    validation.check(
        outputs[-1].state == "REDUCTION" and
        math.isclose(outputs[-1].speed_scale, 0.40, abs_tol=TOLERANCE),
        "RESTRICCION_EN_PRIMER_CICLO_INSEGURO",
    )


def scenario_stop_recovery(validation: Validation) -> None:
    """Verify immediate STOP and delayed, monotonic, rate-limited recovery."""
    print("\n=== E3.2 STOP SÚBITO Y RETIRADA PROGRESIVA ===")
    control = make_filter()
    control.update("trajectory-stop", "ALLOW", 1.0, 0.0)
    stop = control.update("trajectory-stop", "STOP", 0.0, 0.01)
    describe(0, 0.01, stop)
    validation.check(
        stop.state == "STOP" and stop.speed_scale == 0.0 and
        stop.transition_reason == "RESTRICTION_IMMEDIATE",
        "STOP_INMEDIATO",
    )

    outputs: list[StabilityResult] = []
    for index in range(1, 15):
        now = 0.01 + index * SAMPLE_PERIOD
        result = control.update("trajectory-stop", "ALLOW", 1.0, now)
        describe(index, now, result)
        outputs.append(result)

    validation.check(
        all(item.state == "STOP" and item.speed_scale == 0.0
            for item in outputs[:4]),
        "STOP_MANTIENE_CINCO_MUESTRAS",
    )
    validation.check(
        outputs[4].state == "REDUCTION" and
        math.isclose(outputs[4].speed_scale, 0.10, abs_tol=TOLERANCE),
        "RECUPERACION_INICIA_TRAS_CONFIRMACION",
    )
    scales = [item.speed_scale for item in outputs]
    increments = [second - first for first, second in zip(scales, scales[1:])]
    validation.check(
        scales == sorted(scales),
        "RECUPERACION_MONOTONICA",
        f"escalas={','.join(f'{value:.2f}' for value in scales)}",
    )
    validation.check(
        max(increments, default=0.0) <= MAX_INCREMENT + TOLERANCE,
        "INCREMENTO_ESCALA_LIMITADO",
        f"incremento_max={max(increments, default=0.0):.6f}",
    )
    validation.check(
        outputs[-1].state == "ALLOW" and
        math.isclose(outputs[-1].speed_scale, 1.0, abs_tol=TOLERANCE),
        "RECUPERACION_COMPLETA_ESTABLE",
    )


def scenario_reduction_recovery(validation: Validation) -> None:
    """Verify recovery following a predictive reduction."""
    print("\n=== E3.3 RECUPERACIÓN POSTERIOR A REDUCTION ===")
    control = make_filter()
    initial = control.update("trajectory-reduction", "REDUCTION", 0.50, 0.0)
    describe(0, 0.0, initial)
    outputs = []
    for index in range(1, 10):
        now = index * SAMPLE_PERIOD
        result = control.update("trajectory-reduction", "ALLOW", 1.0, now)
        describe(index, now, result)
        outputs.append(result)

    validation.check(
        all(math.isclose(item.speed_scale, 0.50, abs_tol=TOLERANCE)
            for item in outputs[:4]),
        "REDUCTION_ESPERA_CONFIRMACION",
    )
    expected = [0.60, 0.70, 0.80, 0.90, 1.00]
    observed = [item.speed_scale for item in outputs[4:9]]
    validation.check(
        all(math.isclose(a, b, abs_tol=TOLERANCE)
            for a, b in zip(observed, expected)),
        "REDUCTION_RAMPA_ESPERADA",
        f"observada={','.join(f'{value:.2f}' for value in observed)}",
    )


def scenario_command_change(validation: Validation) -> None:
    """Ensure a command identifier change discards prior recovery history."""
    print("\n=== E3.4 CAMBIO DE COMMAND_ID DURANTE RECUPERACIÓN ===")
    control = make_filter()
    control.update("command-old", "REDUCTION", 0.40, 0.0)
    for index in range(1, 4):
        result = control.update(
            "command-old", "ALLOW", 1.0, index * SAMPLE_PERIOD
        )
        describe(index, index * SAMPLE_PERIOD, result)
    changed = control.update("command-new", "WARNING", 1.0, 0.31)
    describe(4, 0.31, changed)
    validation.check(
        changed.state == "WARNING" and changed.speed_scale == 1.0 and
        changed.recovery_count == 0 and
        changed.transition_reason == "COMMAND_CHANGED_RESET",
        "COMMAND_ID_REINICIA_HISTERESIS",
    )


def scenario_same_semantics(validation: Validation) -> None:
    """Apply an identical decision stream to JOG and trajectory channels."""
    print("\n=== E3.5 MISMA SEMÁNTICA JOG/TRAYECTORIA ===")
    sequence = [
        (0.00, "ALLOW", 1.00),
        (0.01, "REDUCTION", 0.45),
        (0.11, "ALLOW", 1.00),
        (0.21, "ALLOW", 1.00),
        (0.22, "STOP", 0.00),
        (0.32, "ALLOW", 1.00),
    ]

    def evaluate(command_id: str) -> list[tuple[str, float, int, str]]:
        control = make_filter()
        values = []
        for index, (now, state, scale) in enumerate(sequence):
            result = control.update(command_id, state, scale, now)
            describe(index, now, result)
            values.append((
                result.state,
                result.speed_scale,
                result.recovery_count,
                result.transition_reason,
            ))
        return values

    print("CANAL=JOG")
    jog = evaluate("jog-semantic")
    print("CANAL=TRAJECTORY")
    trajectory = evaluate("trajectory-semantic")
    validation.check(jog == trajectory, "SEMANTICA_JOG_TRAYECTORIA_IDENTICA")


def command_output(args: Iterable[str], timeout: float = 5.0) -> tuple[int, str]:
    """Run one read-only ROS command and return merged output."""
    try:
        completed = subprocess.run(
            list(args),
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 124, str(exc)
    return completed.returncode, completed.stdout.strip()


def runtime_check(validation: Validation) -> None:
    """Check frozen parameters, interface fields and ROS publisher topology."""
    print("\n=== E3.6 INTEGRACIÓN DEL SUPERVISOR ACTIVO ===")
    node_code, node_list = command_output(["ros2", "node", "list"])
    supervisor_count = node_list.splitlines().count("/safety_supervisor_node")
    validation.check(
        node_code == 0 and supervisor_count == 1,
        "SUPERVISOR_UNICO_ACTIVO",
        f"cantidad={supervisor_count}",
    )

    expected_parameters: dict[str, Callable[[str], bool]] = {
        "runtime_recovery_required_samples": lambda value: value == "5",
        "runtime_max_scale_increment": lambda value: math.isclose(
            float(value), 0.10, abs_tol=TOLERANCE
        ),
        "runtime_recovery_sample_period_sec": lambda value: math.isclose(
            float(value), 0.10, abs_tol=TOLERANCE
        ),
    }
    for name, predicate in expected_parameters.items():
        code, output = command_output([
            "ros2", "param", "get", "/safety_supervisor_node", name
        ])
        value = output.rsplit(":", 1)[-1].strip() if ":" in output else output
        try:
            accepted = code == 0 and predicate(value)
        except ValueError:
            accepted = False
        validation.check(
            accepted,
            f"PARAMETRO_{name.upper()}",
            f"valor={value!r}",
        )

    interface_code, interface = command_output([
        "ros2", "interface", "show", "thesis_interfaces/msg/ExecutionControl"
    ])
    required_fields = {
        "string requested_state",
        "float64 requested_speed_scale",
        "string requested_reason_code",
        "uint32 recovery_count",
        "uint32 recovery_required_count",
        "bool transition",
        "string transition_reason",
    }
    missing_fields = sorted(field for field in required_fields if field not in interface)
    validation.check(
        interface_code == 0 and not missing_fields,
        "INTERFAZ_DIAGNOSTICO_TEMPORAL",
        f"faltantes={missing_fields}",
    )

    topic_code, topic_info = command_output([
        "ros2", "topic", "info", "/thesis/execution_control", "--verbose"
    ])
    publisher_ok = (
        topic_code == 0 and
        "Publisher count: 1" in topic_info and
        "Node name: safety_supervisor_node" in topic_info
    )
    validation.check(
        publisher_ok,
        "PUBLICADOR_CONTROL_TEMPORAL",
        "esperado=/safety_supervisor_node cantidad=1",
    )


def main() -> int:
    """Execute E3 and return a conventional process exit status."""
    print("============================================================")
    print("PAQUETE CONSOLIDADO A — VALIDACIÓN E3")
    print("Estabilidad temporal, histéresis y recuperación")
    print("============================================================")
    validation = Validation()
    scenario_oscillation(validation)
    scenario_stop_recovery(validation)
    scenario_reduction_recovery(validation)
    scenario_command_change(validation)
    scenario_same_semantics(validation)
    runtime_check(validation)

    print("\n=== RESULTADO FINAL ===")
    if validation.failures:
        print("RESULTADO_GLOBAL_E3=FAIL")
        print(f"FALLOS={','.join(validation.failures)}")
        return 1
    print("RESULTADO_GLOBAL_E3=PASS")
    print("FALLOS=NINGUNO")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
