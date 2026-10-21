# Evidencia de viabilidad cartesiana física JACO2

Fecha: 2026-10-20  
Resultado: **NO VIABLE AÚN**

## Restricciones cumplidas

- No se armó el JACO2.
- No se publicaron intenciones, candidatos ni comandos.
- No se ejecutó movimiento físico.
- No se cambió código, configuración, supervisor, umbrales ni launch.
- No se usó `sudo` ni se instalaron paquetes.

## Evidencia estática

1. `src/thesis_hardware/launch/jaco_physical_system.launch.py` no contiene
   `cartesian_command_adapter`, `cartesian_jog_adapter` ni
   `cartesian_trajectory_adapter`.
2. `src/thesis_simulation/launch/thesis_system.launch.py` inicia los tres
   adaptadores anteriores.
3. `src/thesis_ui/thesis_ui/joint_control_gui.py` crea la pestaña cartesiana
   solo en la rama que no es `hardware`; el modo físico añade únicamente el
   control manual articular.
4. `src/thesis_simulation/thesis_simulation/cartesian_jog_adapter.py` registra
   “simulation only” y publica el ID fijo `cartesian_jog_continuous`.
5. `src/thesis_hardware_bridge/src/jaco_hardware_node.cpp` llama a
   `jog_command_matches_arm_epoch()` antes de admitir un JOG.
6. `src/thesis_hardware_bridge/include/thesis_hardware_bridge/bridge_logic.hpp`
   exige el prefijo `jog:<arm_epoch>:`. La prueba C++ confirma el rechazo de
   `jog_continuous` y de épocas anteriores.
7. `src/thesis_core/thesis_core/cartesian_kinematics.py` implementa Jacobiano
   numérico y mínimos cuadrados amortiguados, pero no calcula valor singular
   mínimo, manipulabilidad o número de condición.
8. `src/thesis_core/thesis_core/safety_supervisor_node.py` declara y valida
   `kinematic_tf_tolerance_m`; no existe una comparación runtime FK–TF que use
   el parámetro.
9. Los informes A-02a existentes dejan A-02b bloqueada por falta de estado
   articular sincronizado y relación explícita modelo→`world`.

## Pruebas offline ejecutadas

Comando:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/thesis_core/test/test_cartesian_kinematics.py \
  src/thesis_core/test/test_cartesian_path.py \
  src/thesis_core/test/test_control_stability.py \
  src/thesis_core/test/test_safety_policy.py \
  src/thesis_simulation/test/test_cartesian_jog_model.py \
  src/thesis_simulation/test/test_cartesian_trajectory_model.py \
  src/thesis_ui/test/test_gui_state.py
```

Resultado:

```text
........................................................................ [ 82%]
...............                                                          [100%]
87 passed in 1.52s
```

Prueba focal del bridge:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select thesis_hardware_bridge \
  --ctest-args -R test_bridge_logic --output-on-failure
colcon test-result --test-result-base build/thesis_hardware_bridge
```

Resultado:

```text
Summary: 1 package finished [0.65s]
Summary: 25 tests, 0 errors, 0 failures, 0 skipped
```

No se ejecutó `colcon build` porque no hubo paquetes funcionales modificados.

## Observaciones ROS de solo lectura

La sesión encontrada no tenía `ROS_DOMAIN_ID` en el entorno, por lo que usaba
el dominio predeterminado y no el dominio físico solicitado 78.

`ros2 node list --no-daemon --spin-time 2` mostró la cadena física preventiva,
pero no un adaptador cartesiano:

```text
/capsule_visualizer
/hardware_readiness
/horizon_preview
/jaco_hardware_adapter
/obstacle_demo_source
/obstacle_input_bridge
/proximity_monitor
/robot_state_publisher
/safety_supervisor_node
/telemetry_logger
```

La lectura de armado devolvió:

```text
data: false
```

No llegó una muestra de `/joint_states` en la ventana de observación y
`tf2_echo world j2n6s300_end_effector` informó que los frames no formaban una
cadena conectada. No se intentó recuperar, reiniciar ni armar la sesión porque
esta tarea era diagnóstica.

## Conclusión de evidencia

La simulación dispone de una ruta cartesiana local probada, y el supervisor y
el bridge tienen defensas reutilizables. No existe todavía un contrato físico
coherente de TF–modelo, protección explícita de singularidad, integración con
la época de armado ni topología de launch física para el adaptador. Habilitar
la pestaña ahora produciría, como mínimo, comandos rechazados y, si se evitara
ese rechazo sin completar A-02b, eliminaría una defensa de seguridad.

