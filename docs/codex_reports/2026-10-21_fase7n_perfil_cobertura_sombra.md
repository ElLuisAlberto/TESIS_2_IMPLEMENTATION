# Fase 7N: perfil temporal de cobertura del brazo para self-filter en sombra

## Dictamen

`SHADOW_COVERAGE_PROFILE_READY`

## Alcance y aislamiento

Se creó un perfil opcional, experimental y exclusivamente de sesión para el self-filter de sombra. No se inició ROS, D435i, JACO2, RViz, TF, Gazebo ni ningún launch. No hubo comandos, trayectorias, jog, servicios físicos, instalación de paquetes ni uso de `sudo`.

El perfil no se conecta a supervisor, hardware bridge, extractor, obstacle bridge, `/thesis/obstacle_input`, control ni rutas de producción. No se modificaron `thesis_core`, la extrínseca final, ROI, epsilon, el self-filter, la geometría por defecto, CMake ni `package.xml`. Multi-pose no se utilizó.

## Archivos modificados

- `src/thesis_jaco_perception/config/jaco_self_filter_shadow_coverage_v1.yaml` (nuevo perfil temporal).
- `src/thesis_jaco_perception/launch/jaco_d435i_shadow_session.launch.py`.
- `src/thesis_jaco_perception/launch/jaco_rgbd_shadow.launch.py`.
- `src/thesis_jaco_perception/test/test_shadow_session_contract.py`.

## Mecanismo real de propagación

El nodo `jaco_self_filter_node` recibe la geometría mediante el parámetro YAML `capsule_profile_file`, cuya sección efectiva es `jaco_self_filter_node.ros__parameters` con los nombres exactos `capsule_start_frames`, `capsule_end_frames` y `capsule_inner_radii_m`.

El argumento nuevo de sesión es `shadow_coverage_profile_file`, con default literal vacío `""`. El launch de sesión valida un valor no vacío antes de crear acciones hijas y lo reenvía exclusivamente al sublaunch `jaco_rgbd_shadow.launch.py`. Allí `_resolve_capsule_profile` valida de nuevo el perfil temporal y usa `SetLaunchConfiguration` para seleccionar el `capsule_profile_file` del único `jaco_self_filter_node` de sombra.

Cuando `shadow_coverage_profile_file` permanece vacío, `_resolve_capsule_profile` conserva y valida el `capsule_profile_file` histórico por defecto: `config/jaco_self_filter_capsules.yaml`. Por tanto, el comportamiento sin argumento explícito conserva exactamente los radios históricos.

Todo perfil temporal no vacío debe existir y ser YAML válido antes de iniciar procesos, y debe contener literalmente:

```yaml
shadow_coverage_metadata:
  session_only: true
  not_for_safety: true
  profile_kind: shadow_coverage
```

También se rechazan antes de iniciar procesos: ausencia o error de metadata, radii no numéricos/no finitos/no positivos, marcos vacíos, pares degenerados o duplicados y cualquier número distinto de seis cápsulas consistentes.

## Perfil v1 y comparación de radios

`jaco_self_filter_shadow_coverage_v1.yaml` declara que es experimental, temporal y no apto para seguridad. Conserva exactamente los seis segmentos y frames existentes; no añade cápsulas. Los radios constituyen una hipótesis visual de cobertura, no una medición de calibración.

| Cápsula | Radio base (m) | Cambio (m) | Perfil v1 (m) |
|---:|---:|---:|---:|
| 0 | 0.105 | 0.000 | 0.105 |
| 1 | 0.105 | 0.000 | 0.105 |
| 2 | 0.085 | +0.015 | 0.100 |
| 3 | 0.075 | +0.015 | 0.090 |
| 4 | 0.085 | +0.015 | 0.100 |
| 5 | 0.090 | +0.015 | 0.105 |

```text
base: [0.105, 0.105, 0.085, 0.075, 0.085, 0.090]
v1:   [0.105, 0.105, 0.100, 0.090, 0.100, 0.105]
```

No se cambió `epsilon_total_m`, ROI ni extrínseca.

## Validación sin ROS

Comandos ejecutados:

```text
colcon build --packages-select thesis_jaco_perception --symlink-install --event-handlers console_direct+
colcon test --packages-select thesis_jaco_perception --ctest-args -R test_shadow_session_contract --event-handlers console_direct+
colcon test-result --test-result-base build/thesis_jaco_perception --verbose
git diff --check
```

Resultados finales literales:

- compilación: `1 package finished`, correcta;
- `test_shadow_session_contract`: `37 passed in 0.14s`;
- CTest seleccionado: `1/1` correcto;
- resumen de resultados: `81 tests, 0 errors, 0 failures, 0 skipped`;
- `git diff --check`: sin salida ni errores.

El contrato cubre: default vacío e histórico, aceptación del perfil v1, propagación exclusiva sesión→self-filter, archivo inexistente, metadata ausente o incorrecta, radii no positivos/no finitos, frames o tamaños inválidos, conservación del rechazo de `hardware_output:=true`, y ausencia de forwarding hacia supervisor, hardware bridge, extractor y obstacle bridge.

No se realizó una sesión de cámara o sombra en esta fase. El perfil queda disponible sólo para una futura evaluación explícitamente solicitada, sin promoción a seguridad ni configuración permanente.
