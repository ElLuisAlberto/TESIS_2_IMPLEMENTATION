# Fase 8E: nodo experimental mesh-shadow sin runtime

## Dictamen

`MESH_SHADOW_NODE_STATIC_READY`

Se creó `jaco_mesh_shadow_node` como comparación experimental aislada. No fue iniciado.

## Arquitectura

`/thesis/perception/points_filtered` (exige `frame_id=world`) → validación de edad y descripción collision → TF exacto al sello para `world -> d435i_depth_optical_frame` y `world -> link` → AABB world/BVH collision → tres salidas separadas. Sólo usa `robot_description` expandido recibido como parámetro; no ejecuta xacro ni acepta mallas visuales.

Parámetros: `input_topic` por defecto `/thesis/perception/points_filtered`; `camera_optical_frame` por defecto `d435i_depth_optical_frame`; `epsilon_mesh_total_m=0.07`, finito y no negativo; `robot_description`; `max_cloud_age_sec=0.5`. El epsilon es independiente del filtro de cápsulas.

Tópicos nuevos, `SensorDataQoS` (Best Effort/Volatile):

- `/thesis/jaco_perception/mesh_shadow/points_self_confirmed`
- `/thesis/jaco_perception/mesh_shadow/points_external_candidate`
- `/thesis/jaco_perception/mesh_shadow/points_occluded_unknown`
- `/thesis/jaco_perception/mesh_shadow/diagnostics`

Todas las nubes preservan header de entrada. Ante frame no-world, nube vencida, descripción/malla inválida, campos inválidos o TF ausente al sello exacto, no publica clasificación normal: publica sólo diagnóstico fail-closed. El diagnóstico incorpora `session_only=true`, `not_for_safety=true`, `geometry_kind=collision_mesh`.

## Aislamiento

El nodo no publica TF, no crea servicios de movimiento, no suscribe comandos/supervisor/hardware/bridge/extractor y no referencia `/thesis/obstacle_input`. Sus tópicos no coinciden con los del filtro de cápsulas. Rosout, servicios de parámetros y eventos de parámetros están desactivados para el nodo.

Los launches de sombra incluyen `enable_mesh_shadow_comparison=false` por defecto; el nodo queda bajo esa condición. Se propagó el argumento sólo por la ruta `jaco_d435i_shadow_session.launch.py -> jaco_rgbd_shadow.launch.py -> jaco_mesh_shadow_node`. El guard existente mantiene rechazo de `hardware_output=true`. No se modificó launch estándar de percepción, cápsulas, ROI, extrínseca ni control.

## Archivos

Se crearon `src/jaco_mesh_shadow_node.cpp` y `test/test_mesh_shadow_logic.cpp`. Se modificaron únicamente CMake y los dos launches de sombra autorizados. El test C++ cubre clasificación externa/propia/ocluida, NaN, epsilon inválido, provenance y primer link. El contrato Python existente continúa validando defaults seguros, aislamiento de sesión y rechazo de hardware output.

## Validación

Se ejecutaron únicamente:

```text
colcon build --packages-select thesis_jaco_perception --symlink-install
colcon test --packages-select thesis_jaco_perception --ctest-args -R "test_self_mesh_geometry|test_mesh_shadow_logic|test_shadow_session_contract"
colcon test-result --test-result-base build/thesis_jaco_perception --verbose
git diff --check
```

Resultado literal agregado: `104 tests, 0 errors, 0 failures, 0 skipped`. `git diff --check` sin salida.

No se inició ROS graph, nodo, launch, D435i, JACO2, RViz, TF, Gazebo ni sondas. `coverage_v1` permaneció activa e intacta. No hubo cámara, brazo, hardware ni promoción a seguridad.
