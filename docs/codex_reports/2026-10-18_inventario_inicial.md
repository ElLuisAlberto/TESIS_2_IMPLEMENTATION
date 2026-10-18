# Inventario inicial

Fecha de inventario: 2026-10-18

## Archivos modificados

- `src/thesis_core/thesis_core/cartesian_kinematics.py`
- `src/thesis_interfaces/CMakeLists.txt`
- `src/thesis_simulation/launch/jaco_gazebo.launch.py`
- `src/thesis_simulation/launch/thesis_system.launch.py`
- `src/thesis_simulation/setup.py`
- `src/thesis_simulation/thesis_simulation/system_readiness_node.py`
- `src/thesis_ui/package.xml`
- `src/thesis_ui/test/test_gui_state.py`
- `src/thesis_ui/thesis_ui/joint_control_gui.py`

## Archivos sin seguimiento

- `src/thesis_core/test/test_cartesian_path.py`
- `src/thesis_core/thesis_core/cartesian_path.py`
- `src/thesis_core/thesis_core/rigid_transform.py`
- `src/thesis_interfaces/msg/CartesianTrajectoryCommand.msg`
- `src/thesis_interfaces/msg/CartesianTrajectoryStatus.msg`
- `src/thesis_simulation/test/test_cartesian_jog_model.py`
- `src/thesis_simulation/test/test_cartesian_trajectory_model.py`
- `src/thesis_simulation/thesis_simulation/cartesian_jog_adapter.py`
- `src/thesis_simulation/thesis_simulation/cartesian_jog_model.py`
- `src/thesis_simulation/thesis_simulation/cartesian_trajectory_adapter.py`
- `src/thesis_simulation/thesis_simulation/cartesian_trajectory_model.py`
- `src/thesis_simulation/thesis_simulation/cartesian_trajectory_validator.py`

## Paquetes afectados

- `thesis_core`: cinemática, trayectoria cartesiana y transformaciones rígidas.
- `thesis_interfaces`: definiciones de mensajes de trayectoria cartesiana.
- `thesis_simulation`: adaptadores, modelo, validación y lanzamiento de control cartesiano en simulación.
- `thesis_ui`: interfaz y pruebas para control cartesiano.

## Comandos de compilación recomendados

Ejecutar desde la raíz del espacio de trabajo, manteniendo la salida de hardware desactivada:

```bash
colcon build --packages-select thesis_interfaces thesis_core thesis_simulation thesis_ui
source install/setup.bash
colcon test --packages-select thesis_interfaces thesis_core thesis_simulation thesis_ui
colcon test-result --verbose
```

## Riesgos detectados

- El árbol de trabajo ya contiene cambios locales y archivos sin seguimiento; deben preservarse antes de cualquier edición adicional.
- La funcionalidad en curso incorpora comandos, trayectorias y JOG cartesianos; debe limitarse a simulación.
- Los lanzadores de simulación y la interfaz podrían exponer rutas hacia hardware; `hardware_output` debe permanecer desactivado y no se deben enviar comandos al Kinova Jaco2 físico.
- Los mensajes nuevos requieren compilar `thesis_interfaces` antes de sus paquetes dependientes.
