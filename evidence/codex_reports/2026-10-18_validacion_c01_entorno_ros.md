# Evidencia — validación C-01 con entorno ROS

Fecha: 2026-10-18

## Entorno usado

Desde la raíz del workspace se ejecutó, sin cambios persistentes:

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
```

Verificación:

```bash
python3 -c "import rclpy; print(rclpy.__file__)"
```

Resultado correcto:

```text
/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py
```

## Compilación

Comando:

```bash
colcon build --packages-select thesis_core thesis_hardware_bridge
```

Resultado: correcto. Paquetes terminados:

- `thesis_core` (1,51 s)
- `thesis_hardware_bridge` (13,8 s)

`colcon` informó únicamente la advertencia de paquetes ya presentes en el
underlay `install/`; no fue un fallo de compilación.

## Pruebas del supervisor C-01

Tras recargar el mismo entorno se ejecutó:

```bash
python3 -m pytest -q src/thesis_core/test/test_jog_message_age.py src/thesis_core/test/test_point_command_admission.py src/thesis_core/test/test_controller_failure_failsafe.py
```

Resultado: **9 aprobadas, 5 fallidas**.

Las cinco fallas están en `test_point_command_admission.py`: los casos de
sello cero, vencido, futuro, ID vacío e ID reutilizado verifican
`rejections[0][2]` como si fuese el código de rechazo, pero el callback pasa
en esa posición el texto completo, por ejemplo:

```text
REJECTED point-stale: POINT_COMMAND_STAMP_STALE
```

La expectativa de la prueba es únicamente:

```text
POINT_COMMAND_STAMP_STALE
```

No se ejecutó `test_bridge_logic` ni ninguna otra prueba tras este fallo,
conforme a la condición de detención. No se modificó código, YAML, launch,
entorno ni hardware durante esta validación.
