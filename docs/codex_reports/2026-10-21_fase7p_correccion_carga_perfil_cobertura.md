# Fase 7P: corrección estática de carga de `shadow_coverage_v1`

## Dictamen

`SHADOW_COVERAGE_PROFILE_LOAD_FIXED`

## Diagnóstico basado en evidencia

El único archivo del directorio de log del PID 82828 fue `/home/luis/.ros/log/2026-10-21-18-12-26-484252-miNave-82812/launch.log`; no hubo archivo stderr individual. `coredumpctl` no está instalado, por lo que no se inventó un mensaje de core dump ni se instaló ninguna herramienta.

El error literal completo disponible fue:

```text
[ERROR] [jaco_self_filter_node-8]: process has died [pid 82828, exit code -6,
cmd '/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/install/thesis_jaco_perception/lib/thesis_jaco_perception/jaco_self_filter_node --ros-args -r __node:=jaco_self_filter_node --params-file /home/luis/Escritorio/TESIS_2_IMPLEMENTATION/install/thesis_jaco_perception/share/thesis_jaco_perception/config/jaco_d435i_shadow.yaml --params-file /home/luis/Escritorio/TESIS_2_IMPLEMENTATION/src/thesis_jaco_perception/config/jaco_self_filter_shadow_coverage_v1.yaml --params-file /tmp/launch_params_ysj334l4'].
```

No quedó registrada la excepción stderr del parser ni una excepción de `JacoSelfFilterNode`; por eso no se atribuye un texto inexistente al abort. La causa estructural sí está demostrada por el comando anterior y el contenido del YAML: `jaco_rgbd_shadow.launch.py:139-149` añadía `LaunchConfiguration("capsule_profile_file")` a `Node(parameters=[...])`, por lo que launch generó el segundo `--params-file` temporal. Ese archivo tenía `shadow_coverage_metadata` en la raíz, fuera de `jaco_self_filter_node.ros__parameters`. La interfaz instalada `rcl_yaml_param_parser/parser.h:93-95` es el parser de archivos de parámetros, y los ejemplos instalados sitúan los parámetros bajo el mapa de nodo `ros__parameters`. Así, el perfil con metadata de validación no debía ser entregado como archivo de parámetros ROS completo.

## Corrección mínima aplicada

No se modificaron el self-filter geométrico, epsilon, ROI, extrínseca, perfiles base, supervisor, hardware bridge, extractor, obstacle bridge, `thesis_core`, CMake ni `package.xml`.

Antes, `jaco_rgbd_shadow.launch.py` construía el nodo con `parameters = [config_file, capsule_profile_file, argumentos explícitos]`. Al seleccionar el perfil temporal, `capsule_profile_file` era sustituido por el archivo completo temporal y launch lo convirtió en `--params-file`.

Después, `_create_self_filter_node` en `launch/jaco_rgbd_shadow.launch.py:86-114` mantiene el YAML base histórico como archivo normal y, sólo para un perfil temporal explícito y validado, añade a `parameters` el diccionario extraído. El archivo temporal nunca se agrega como ruta en `parameters` y por tanto nunca puede aparecer como segundo `--params-file`.

El extractor estricto `_validate_shadow_coverage_profile` en ambos launches conserva y comprueba la metadata raíz obligatoria:

```yaml
shadow_coverage_metadata:
  session_only: true
  not_for_safety: true
  profile_kind: shadow_coverage
```

Pero devuelve sólo este mapa para el nodo:

```text
capsule_start_frames
capsule_end_frames
capsule_inner_radii_m
```

El conjunto de claves debe ser exactamente esas tres listas. Cualquier clave adicional, incluidos `enabled`, `epsilon_total_m`, `input_topic`, salidas, `expected_frame`, `camera_optical_frame`, parámetros TF o edad de nube, es rechazada antes de crear procesos hijos. También se rechazan YAML malformado, metadata incorrecta, archivo inexistente, valores no finitos/no positivos, frames inválidos, pares duplicados y número de cápsulas distinto de seis.

El default vacío permanece en `config/jaco_self_filter_capsules.yaml`; no hay fallback silencioso para un perfil temporal inválido.

## Archivos modificados

- `src/thesis_jaco_perception/launch/jaco_d435i_shadow_session.launch.py`
- `src/thesis_jaco_perception/launch/jaco_rgbd_shadow.launch.py`
- `src/thesis_jaco_perception/test/test_shadow_session_contract.py`

No fue necesario modificar el YAML temporal v1, CMake ni `package.xml`.

## Validación estática

Se ejecutaron únicamente:

```text
colcon build --packages-select thesis_jaco_perception --symlink-install
colcon test --packages-select thesis_jaco_perception --ctest-args -R test_shadow_session_contract
colcon test-result --test-result-base build/thesis_jaco_perception --verbose
git diff --check
```

Resultados literales finales:

- build: `1 package finished`;
- `test_shadow_session_contract`: `37 passed in 0.15s`;
- CTest seleccionado: `1/1` passed;
- resumen: `81 tests, 0 errors, 0 failures, 0 skipped`;
- `git diff --check`: sin salida ni errores.

La prueba cubre el diccionario extraído de sólo tres listas, metadata raíz, ausencia de `SetLaunchConfiguration` y de ruta temporal como archivo de parámetros, clave adicional prohibida, YAML malformado, metadata/radios/frames inválidos, seis cápsulas inconsistentes, default histórico, `hardware_output:=true` rechazado y aislamiento de supervisor, bridge, extractor y obstacle bridge.

No se inició ROS, launch, D435i, JACO2, RViz, TF, Gazebo ni hardware durante esta fase. No se abrió cámara ni se ejecutó prueba runtime; el perfil mantiene su carácter temporal, aislado y no apto para seguridad.
