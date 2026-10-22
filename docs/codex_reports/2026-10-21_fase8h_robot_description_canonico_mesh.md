# Fase 8H: contrato canónico robot_description para mesh-shadow

## Dictamen

`MESH_SHADOW_CANONICAL_DESCRIPTION_READY`

## Fuente y causa

La fuente canónica del `robot_state_publisher` read-only es `thesis_description/urdf/j2n6s300_standalone.xacro`. `thesis_hardware/launch/jaco_read_only_state.launch.py` la expande con `Command(["xacro ", xacro_file])` y la entrega mediante `ParameterValue(..., value_type=str)`, sin argumentos Xacro adicionales.

Antes de esta fase, la ruta sesión → `jaco_rgbd_shadow` → nodo mesh llevaba un argumento `robot_description` con default vacío. Aunque 8G corrigió el tipo, no se garantizaba un modelo JACO2 no vacío.

## Corrección

`jaco_rgbd_shadow.launch.py` reutiliza el mismo Xacro canónico mediante `Command(["xacro ", xacro_file])` únicamente en `_create_mesh_shadow_node`. Si `enable_mesh_shadow_comparison=false`, retorna sin expandir, validar ni crear nodo. Si es true, valida XML no vacío, XML sintáctico, raíz `robot`, collision meshes y los links `j2n6s300_link_base`, `j2n6s300_link_6`, `j2n6s300_link_finger_1`; cualquier fallo aborta con `MESH_SHADOW_ROBOT_DESCRIPTION_INVALID` antes del nodo mesh.

El XML validado llega exclusivamente a `jaco_mesh_shadow_node` como `ParameterValue(xml, value_type=str)`. Se retiró el forwarding de la cadena vacía desde sesión; robot-state-publisher no cambió. Ruta resultante: sesión → sublaunch RGBD sombra → expansión canónica condicional → validación → nodo mesh. No llega a RealSense, supervisor, bridge, extractor, hardware, obstacle_input ni control.

## Pruebas y validación

El contrato comprueba modo false sin acción mesh, ParameterValue tipo str, fuente `j2n6s300_standalone.xacro`, XML con caracteres especiales como una sola cadena y rechazos fail-closed para vacío, no XML, raíz distinta y falta de collision/links. Se mantiene rechazo de hardware_output=true y contrato de cápsulas sin cambios.

Se ejecutaron exclusivamente `colcon build --packages-select thesis_jaco_perception --symlink-install`, `colcon test --packages-select thesis_jaco_perception --ctest-args -R "test_mesh_shadow_logic|test_shadow_session_contract"`, `colcon test-result --test-result-base build/thesis_jaco_perception --verbose` y `git diff --check`.

Resultado literal: `105 tests, 0 errors, 0 failures, 0 skipped`. Build correcto y `git diff --check` sin salida.

No se inició ROS graph, launch, cámara, JACO2, RViz, TF, Gazebo ni sondas. No se cerró ni modificó proceso existente; no hubo hardware ni reintento de 8F.
