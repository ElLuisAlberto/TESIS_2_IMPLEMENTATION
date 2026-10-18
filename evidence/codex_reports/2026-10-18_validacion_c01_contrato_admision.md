# Evidencia — C-01: contrato de admisión puntual

Fecha: 2026-10-18

## Diagnóstico previo

Se ejecutaron solo los cinco casos fallidos con detalle:

```bash
python3 -m pytest -vv src/thesis_core/test/test_point_command_admission.py -k 'invalid_point_command_is_not_forwarded or reused_point_command_is_not_forwarded_twice'
```

Resultado: 5 fallas, 3 casos deseleccionados.

La implementación devuelve códigos estructurados desde
`admit_point_command()`, por ejemplo `POINT_COMMAND_STAMP_STALE`. El callback
los pasa como `reason_code` a:

```python
reject_point_command(msg, reason_code, reason)
```

El mock registra la tupla `(msg, reason_code, reason)`. Las aserciones
fallidas comparaban erróneamente `rejections[0][2]` (el texto completo, por
ejemplo `REJECTED point-stale: POINT_COMMAND_STAMP_STALE`) contra el código.
El contrato de `publish_decision()` y de las llamadas existentes conserva la
separación `reason_code` estructurado / `reason` diagnóstico legible.

## Corrección aplicada

Archivo modificado:

- `src/thesis_core/test/test_point_command_admission.py`

Las aserciones ahora verifican el código en `rejections[0][1]` y que el texto
en `rejections[0][2]` incluye dicho código. No se alteró la implementación,
política de frescura, IDs, STOP, mensajes ROS ni configuración.

## Validación

Entorno cargado de forma temporal:

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
```

Resultados:

| Comando | Resultado |
| --- | --- |
| `python3 -m pytest -q src/thesis_core/test/test_point_command_admission.py` | 8 aprobadas |
| `python3 -m pytest -q src/thesis_core/test/test_jog_message_age.py src/thesis_core/test/test_controller_failure_failsafe.py` | 6 aprobadas |
| `ctest --test-dir build/thesis_hardware_bridge --output-on-failure -R '^test_bridge_logic$'` | 1/1 aprobada |
| `colcon build --packages-select thesis_core thesis_hardware_bridge` | 2 paquetes compilados correctamente |

La prueba directa del puente cubre los sellos de punto/control cero, vencido y
futuro, ID vacío/reutilizado/ajeno y prioridad STOP. No se inició ROS,
simulación, GUI, hardware ni cámara.
