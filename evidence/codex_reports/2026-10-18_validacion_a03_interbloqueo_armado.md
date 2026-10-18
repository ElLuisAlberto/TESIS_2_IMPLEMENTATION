# Evidencia — validación A-03 del interbloqueo de armado

Fecha: 2026-10-18

## Corrección de compilación

Archivo corregido:

- `src/thesis_hardware_bridge/src/jaco_hardware_node.cpp`

Cambios mecánicos y limitados a los dos errores reportados:

1. `positive_values` pasó de `std::array<double, 10>` a
   `std::array<double, 11>` para incluir `safety_input_timeout_sec_`.
2. Se retiró `const` de `safety_supervisor_available()` y de
   `arm_interlock_inputs()`, porque en ROS Humble
   `get_node_graph_interface()` no es una operación `const`.

No se modificaron precondiciones, política de interbloqueo, watchdog,
`hardware_output_enabled` por defecto, YAML, launch ni mensajes ROS en esta
corrección.

## Entorno y compilación

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
colcon build --packages-select thesis_hardware thesis_hardware_bridge
```

Resultado:

```text
Finished <<< thesis_hardware_bridge [16.2s]
Finished <<< thesis_hardware [1.23s]
Summary: 2 packages finished [17.7s]
```

## Pruebas mock A-03

```bash
build/thesis_hardware_bridge/test_bridge_logic --gtest_filter='BridgeLogic.ArmInterlock*'
```

Resultado: 2/2 aprobadas.

- `ArmInterlockRequiresEveryPrecondition`: cubre ausencia individual de
  salida permitida, conexión, JointState fresco, `system_ready`/frescura,
  proximidad válida/fresca, supervisor y estado STOP/HOLD.
- `ArmInterlockDisarmsAfterSafetyLoss`: cubre pérdida posterior de proximidad
  fresca y exige desarmado cuando ya estaba armado.

No se inició ROS, Gazebo, hardware ni cámara.
