# Fase 6I: lectura física pura y perfil de cápsulas

## Resultado

`READY_FOR_ISOLATED_SHADOW_RETRY`

No se inició JACO2, D435i, RViz, TF, Gazebo, supervisor ni ninguna sesión física durante esta fase.

## Brechas resueltas

| Antes de Fase 6I | Contrato nuevo |
| --- | --- |
| `jaco_physical_system.launch.py` combinaba adaptador físico con la pila preventiva, mandos, telemetría y RViz. | `jaco_read_only_state.launch.py` contiene solo adaptador USB, `robot_state_publisher` y `hardware_readiness`. |
| El filtro recibía listas vacías de cápsulas y abortaba al declarar parámetros. | `jaco_self_filter_capsules.yaml` entrega seis cápsulas no vacías y verificadas antes de crear el nodo. |

El nuevo launch fija `hardware_output_enabled: False` en el adaptador y su único argumento de salida, `hardware_output`, vale `false` por defecto y rechaza explícitamente `true`. No declara ni incluye `safety_supervisor`, `obstacle_input_bridge`, monitores de proximidad, adaptadores cartesianos, GUI, trayectorias, RViz, telemetría, visualizadores de cápsulas ni publicadores de comandos.

El flujo permitido queda así:

```text
JACO USB (feedback) -> /joint_states -> robot_state_publisher -> TF del modelo
                                      -> hardware_readiness (diagnóstico)
D435i/preprocesador -> nube world -> jaco_self_filter_node -> tres nubes sombra
```

No existe conexión del launch de sombra hacia extractor, `/thesis/obstacle_input` o supervisor.

## Perfil de cápsulas

Archivo: `src/thesis_jaco_perception/config/jaco_self_filter_capsules.yaml`.

La fuente fue `src/thesis_core/config/jaco_capsules.yaml`, sin marcos inferidos: seis ejes consecutivos desde `j2n6s300_link_base` hasta `j2n6s300_end_effector`, con radios `[0.105, 0.105, 0.085, 0.075, 0.085, 0.090]` m. `jaco_rgbd_shadow.launch.py` valida que el YAML exista, tenga listas no vacías e iguales, marcos no vacíos, pares no degenerados ni duplicados y radios positivos, antes de iniciar el filtro. La sesión integrada pasa explícitamente el marco de cámara `d435i_depth_optical_frame` y exige `world`.

## Archivos modificados o creados

- `src/thesis_hardware/launch/jaco_read_only_state.launch.py`
- `src/thesis_hardware/test/test_read_only_launch_contract.py`
- `src/thesis_jaco_perception/config/jaco_self_filter_capsules.yaml`
- `src/thesis_jaco_perception/launch/jaco_rgbd_shadow.launch.py`
- `src/thesis_jaco_perception/launch/jaco_d435i_shadow_session.launch.py`
- `src/thesis_jaco_perception/test/test_shadow_session_contract.py`
- `src/thesis_jaco_perception/test/test_capsule_profile_contract.py`
- `src/thesis_jaco_perception/test/test_package_boundary.py`
- `src/thesis_jaco_perception/CMakeLists.txt`

## Verificación sin hardware

- `colcon build --packages-select thesis_hardware thesis_jaco_perception`: correcto.
- `colcon test --packages-select thesis_hardware thesis_jaco_perception`: correcto.
  - `thesis_hardware`: 15 pruebas, 0 fallos, 1 omitida preexistente.
  - `thesis_jaco_perception`: 50 pruebas, 0 fallos.
- `ros2 launch thesis_hardware jaco_read_only_state.launch.py --show-args`: correcto.
- `ros2 launch thesis_jaco_perception jaco_d435i_shadow_session.launch.py --show-args`: correcto.
- `git diff --check`: correcto.

La primera sesión temporal posterior aún requerirá autorización física, extrínseca candidata de sesión validada, JACO2 inmóvil y la comprobación en tiempo de ejecución de conexión, `/joint_states`, TF exacto por sello, nube y diagnósticos. Esta fase no inicia esa sesión.
