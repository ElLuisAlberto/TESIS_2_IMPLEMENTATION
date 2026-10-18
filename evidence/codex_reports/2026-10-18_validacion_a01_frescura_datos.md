# Evidencia — A-01: frescura de datos

Fecha: 2026-10-18

## Corrección

- `Obstacle` rechaza explícitamente sellos de origen cero.
- `proximity_monitor` conserva el sello de adquisición del obstáculo y lo
  repite en `ProximityStatus`; valida de nuevo la edad en cada evaluación.
  Una entrada vencida produce indisponibilidad, no un nuevo `ALLOW`.
- El supervisor registra recepción y origen por separado. Sus comprobaciones
  de edad existentes se alimentan del sello de origen; `JointState` y
  `ProximityStatus` cero, vencidos o futuros limpian el estado y fallan
  cerrado.

## Compilación

```bash
colcon build --packages-select thesis_core
```

Resultado: correcto (`thesis_core`, 0.79 s).

## Pruebas

```bash
python3 -m pytest -q src/thesis_core/test/test_obstacle_contract.py src/thesis_core/test/test_source_freshness.py src/thesis_core/test/test_jog_message_age.py
```

Resultado: 16 aprobadas.

```bash
python3 -m pytest -q src/thesis_core/test/test_point_command_admission.py src/thesis_core/test/test_controller_failure_failsafe.py
```

Resultado: 11 aprobadas.

La primera colección previa a compilar falló porque `install/thesis_core`
contenía la versión anterior; tras compilar y recargar el overlay, todas las
pruebas seleccionadas pasaron. No se inició ROS, Gazebo, hardware ni cámara.
