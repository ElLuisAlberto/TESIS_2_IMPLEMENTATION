# Evidencia: corrección de desarmado automático en retirada

Fecha: 2026-10-20

## Pruebas sin movimiento

```text
colcon build --packages-select thesis_hardware_bridge --symlink-install
Summary: 1 package finished

colcon test --packages-select thesis_hardware_bridge
GTest: 24 tests passed
ament: 25 tests, 0 errors, 0 failures, 0 skipped
```

## Arranque físico estático posterior

```text
backend=kinova_usb
connected=true
api_open=true
control_api_started=false
output_permitted=true
armed=false
active_goal=false
jog_active=false
state_valid=true
system_ready=true
proximity_valid=true
safety_stop_active=false
protective_recovery_active=false
fault=false
feedback_rate_hz_measured=19.993689
output_command_rate_hz_measured=0.000000
jog_input_rate_hz_measured=0.000000
mode=hold
```

## Proximidad tras el reinicio

```text
state=REDUCTION
reference_frame=world
minimum_clearance=0.05129104366404154 m
limiting_segment=wrist_2_to_tool
obstacle_center=(0.59, 0.0, 0.65)
obstacle_radius_effective=0.12 m
```

El launch anterior se cerró con SIGINT sobre su grupo de procesos y se comprobó
que el bloqueo USB podía adquirirse antes del nuevo inicio. GUI y RViz quedaron
abiertos. No hubo armado, intención JOG, candidato ni salida física emitidos por
el agente.
