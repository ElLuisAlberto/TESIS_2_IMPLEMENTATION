# Evidencia — C-01: frescura y replay de punto/ExecutionControl

Fecha: 2026-10-18

## Cambio bajo prueba

- `thesis_core`: admisión previa a la publicación de comandos puntuales.
- `thesis_hardware_bridge`: admisión previa a crear un objetivo o aplicar un
  `ExecutionControl` no STOP.

## Comprobación sintáctica

Comando:

```bash
python3 -m py_compile src/thesis_core/thesis_core/safety_supervisor_node.py src/thesis_core/test/test_point_command_admission.py
```

Resultado: correcto (código de salida 0).

## Pruebas solicitadas

Comando inicial:

```bash
PYTHONPATH=src/thesis_core python3 -m pytest -q src/thesis_core/test/test_jog_message_age.py src/thesis_core/test/test_point_command_admission.py src/thesis_core/test/test_controller_failure_failsafe.py
```

Resultado: no se coleccionaron pruebas; `ModuleNotFoundError: No module named
'rclpy'` al importar `safety_supervisor_node.py` (3 errores de colección).

Reintento con ROS Humble cargado:

```bash
source /opt/ros/humble/setup.bash && PYTHONPATH=src/thesis_core python3 -m pytest -q src/thesis_core/test/test_jog_message_age.py src/thesis_core/test/test_point_command_admission.py src/thesis_core/test/test_controller_failure_failsafe.py
```

Resultado: mismo error de colección y mismo motivo. No se ejecutó ningún caso
de prueba. De acuerdo con la condición de detención solicitada, no se compiló
ni se ejecutó `thesis_hardware_bridge` ni se realizaron más pruebas.

## Estado

**BLOQUEADO POR ENTORNO DE PRUEBAS.** La corrección no queda validada hasta que
el intérprete ROS usado por las pruebas pueda importar `rclpy` y se ejecuten
las pruebas C-01, incluida la suite C++ del puente.
