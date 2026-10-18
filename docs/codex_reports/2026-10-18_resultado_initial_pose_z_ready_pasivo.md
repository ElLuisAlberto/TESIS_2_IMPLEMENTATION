# Resultado de `initial_pose:=z_ready` en Gazebo headless

Fecha: 2026-10-18.

El preset quedó implementado y su inicialización pasiva fue validada.

## Implementación

- `initial_pose:=default` conserva exactamente la postura anterior
  `(0, π, π, 0, 0, 0)`.
- `initial_pose:=z_ready` usa `READY_JOINTS`:
  `(0°, 150°, 210°, 0°, 20°, 0°)`.
- El argumento admite únicamente `default|z_ready` y se reenvía desde
  `thesis_system.launch.py` a `jaco_gazebo.launch.py` y de allí al xacro.

## Validación

- Compilación de `thesis_simulation`: aprobada.
- Pila headless con salida simulada deshabilitada: aprobada.
- READY: primer intento, 4 s de un plazo global de 20 s.
- Las seis articulaciones coincidieron con `READY_JOINTS`; el máximo desvío
  observado fue aproximadamente `0.000102 rad` (`0.0058°`) y las velocidades
  fueron prácticamente cero.
- TF estable: traslación `(0.596, 0.070, 0.926) m`, coherente con la postura.
- No se publicaron comandos ni hubo movimiento ordenado.

## Restricción encontrada

El obstáculo demo predeterminado produce `STOP`, con clearance `−0.032 m` en
`wrist_2_to_tool`. READY aun así se publica porque actualmente solo requiere
una muestra reciente de proximidad. La inicialización es correcta, pero una
prueba posterior de movimiento Z no debe autorizarse con esta configuración
hasta resolver o justificar esa precondición de seguridad.

Evidencia completa:
[2026-10-18_initial_pose_z_ready_pasivo.txt](../../evidence/codex_reports/2026-10-18_initial_pose_z_ready_pasivo.txt).

