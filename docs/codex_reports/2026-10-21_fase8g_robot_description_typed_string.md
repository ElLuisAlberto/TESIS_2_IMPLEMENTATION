# Fase 8G: robot_description tipado para mesh-shadow

## Dictamen

`MESH_SHADOW_ROBOT_DESCRIPTION_FIXED`

## Causa y corrección

Fase 8F falló con: `Unable to parse the value of parameter robot_description as yaml. If the parameter is meant to be a string, try wrapping it in launch_ros.parameter_descriptions.ParameterValue(value, value_type=str)`.

La causa demostrada estaba en `jaco_rgbd_shadow.launch.py`, línea 161 antes del cambio: el diccionario de parámetros mesh entregaba `"robot_description": LaunchConfiguration("robot_description")` sin tipo explícito, permitiendo inferencia YAML del XML expandido.

La entrada del único Node mesh ahora usa `"robot_description": ParameterValue(LaunchConfiguration("robot_description"), value_type=str)`. El XML no se convierte a YAML, no se serializa, recorta ni modifica. Un XML que contiene `<`, `>`, `:`, comillas y saltos de línea se verifica como una sola cadena en el contrato.

## Propagación y aislamiento

`jaco_d435i_shadow_session.launch.py` declara `robot_description` vacío por defecto y lo forwardea sólo al IncludeLaunchDescription de `jaco_rgbd_shadow.launch.py`. Éste lo tipa como str sólo para `jaco_mesh_shadow_node`; no llega a RealSense, supervisor, hardware, bridge, extractor ni `obstacle_input`. Robot-state-publisher no se alteró.

`enable_mesh_shadow_comparison` conserva default false y el nodo permanece condicionado por ese argumento. Los parámetros `input_topic`, `camera_optical_frame`, `epsilon_mesh_total_m` y `max_cloud_age_sec` no cambiaron. Una cadena vacía conserva diagnóstico fail-closed de carga de malla.

## Pruebas

Se extendió `test_shadow_session_contract.py` para comprobar condición false, ParameterValue tipado, forwarding mínimo, XML con caracteres especiales y ausencia de ruta a seguridad/control.

Se ejecutaron exclusivamente `colcon build --packages-select thesis_jaco_perception --symlink-install`, `colcon test --packages-select thesis_jaco_perception --ctest-args -R "test_mesh_shadow_logic|test_shadow_session_contract"`, `colcon test-result --test-result-base build/thesis_jaco_perception --verbose` y `git diff --check`.

Resultado literal: `104 tests, 0 errors, 0 failures, 0 skipped`. El build terminó correctamente y `git diff --check` no produjo salida.

No se inició ROS graph, launch, cámara, JACO2, RViz, TF, Gazebo ni sondas. No se cerró ni modificó proceso alguno. No hubo hardware ni reintento de Fase 8F.
