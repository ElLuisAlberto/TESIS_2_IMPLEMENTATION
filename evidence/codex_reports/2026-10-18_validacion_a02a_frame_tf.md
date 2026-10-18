# Evidencia de validación — A-02a frame, STOP upstream y TF temporal

Fecha: 2026-10-18  
Estado: **DETENIDA POR ENTORNO DE PRUEBAS**

## Alcance aplicado

Se modificaron únicamente:

- `src/thesis_core/thesis_core/safety_supervisor_node.py`
- `src/thesis_core/thesis_core/proximity_monitor_node.py`
- `src/thesis_core/thesis_core/obstacle_input_bridge.py`
- `src/thesis_core/test/test_a02a_frame_tf_interlock.py`

No se modificaron C-01, A-01, A-03, IK, GUI, hardware, mensajes ROS, launch
ni `kinematic_tf_tolerance_m`.

## Comportamiento implementado

- El supervisor acepta `ProximityStatus` solo si `reference_frame == "world"`;
  un frame vacío o distinto limpia el estado de proximidad y registra el código
  estructurado `PROXIMITY_FRAME_INVALID`.
- Un `STOP` upstream se decide antes de ejecutar predicción y se registra como
  `UPSTREAM_PROXIMITY_STOP`; por tanto no puede convertirse en `ALLOW`,
  `WARNING` ni `REDUCTION`.
- El monitor pide cada TF con `Time.from_msg(source_stamp)`, no con el último
  TF disponible. Si falta, está fuera de la caché temporal o falla la
  transformación, publica `ProximityStatus` con `state=STOP` y
  `limiting_segment=TF_UNAVAILABLE`, conservando el sello de adquisición.
- El monitor y el puente de obstáculos restringen su frame de seguridad a
  `world`.

## Compilación

Entorno cargado:

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
```

Comando ejecutado:

```bash
colcon build --packages-select thesis_core
```

Resultado: **correcto**, `1 package finished` (`thesis_core`). Colcon emitió
solo la advertencia de que `thesis_core` existe también en el underlay
`install/thesis_core`.

## Pruebas solicitadas

Comando intentado:

```bash
pytest -q src/thesis_core/test/test_a02a_frame_tf_interlock.py \
  src/thesis_core/test/test_source_freshness.py \
  src/thesis_core/test/test_obstacle_contract.py
```

Resultado:

```text
/bin/bash: línea 5: pytest: orden no encontrada
```

No se ejecutó ninguna aserción. No se instaló nada ni se modificó PATH,
PYTHONPATH u otra variable persistente. Conforme a la condición de detención,
no se intentó un corredor alternativo ni se continuó la validación.

## Estado

- A-02a: **NO VALIDADA** por indisponibilidad del corredor de pruebas.
- A-02b: **BLOQUEADA POR CONTRATO**, sin cambios: la comparación completa
  TF--modelo sigue requiriendo estado articular sincronizado y una relación
  explícita modelo→`world`.
- No se inició ROS, Gazebo, GUI, cámara ni hardware y no se publicaron
  comandos.

