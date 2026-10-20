# Evidencia de validación sin hardware

Fecha: 2026-10-19

## Comandos ejecutados

```text
colcon build --packages-select thesis_hardware_bridge thesis_ui --symlink-install
colcon test --packages-select thesis_hardware_bridge thesis_ui
ros2 topic echo --once /thesis/hardware/armed --no-daemon --spin-time 2
```

## Resultado

```text
thesis_hardware_bridge: 19 tests, 0 failures
thesis_ui:              25 tests, 0 failures, 1 skipped
/thesis/hardware/armed: data: false
```

Las comprobaciones no publicaron comandos de movimiento ni activaron la salida
física. No se realizó la prueba estática con armado: requiere autorización
nueva y separada del operador.
