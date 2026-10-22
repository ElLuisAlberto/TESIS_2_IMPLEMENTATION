# Fase 7O: validación visual única de `shadow_coverage_v1`

## Dictamen

`BLOCKED_RUNTIME`

## Preparación verificada

No había una sesión previa de cámara/sombra identificable. Se verificó `/usr/bin/gnome-terminal`, las rutas de la extrínseca temporal v2, el perfil `jaco_self_filter_shadow_coverage_v1.yaml` y el perfil RViz existente. El YAML de cobertura contenía literalmente `session_only: true`, `not_for_safety: true` y `profile_kind: shadow_coverage`.

`ros2 launch thesis_jaco_perception jaco_d435i_shadow_session.launch.py --show-args` confirmó los argumentos usados, incluidos `shadow_coverage_profile_file`, `session_epsilon_total_m`, `enable_crop` y los seis límites XYZ.

## Único intento de sesión

Se abrió una única terminal nativa de launch y una segunda terminal nativa de RViz. El comando efectivo del launch fue:

```text
ros2 launch thesis_jaco_perception jaco_d435i_shadow_session.launch.py
start_jaco_read_only:=true start_camera:=true start_rviz:=false
enable_self_filter:=true publish_session_extrinsic:=true
candidate_extrinsic_file:=/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/evidence/codex_reports/2026-10-21_d435i_extrinsic_visual_hypothesis_v2.yaml
session_epsilon_total_m:=0.07 hardware_output:=false
enable_crop:=true min_x:=-0.55 max_x:=1.50 min_y:=-0.50 max_y:=0.50
min_z:=-0.25 max_z:=1.10
shadow_coverage_profile_file:=/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/src/thesis_jaco_perception/config/jaco_self_filter_shadow_coverage_v1.yaml
```

RViz se abrió con `src/thesis_jaco_perception/rviz/jaco_d435i_alignment.rviz`. No fue posible evaluar QoS ni recepción de salidas clasificadas porque el self-filter terminó durante el arranque; no se registró un error literal de RViz.

## Error literal y estado observado

El launch inició TF temporal v2, JACO2 de lectura, D435i, preprocesador y health node. El registro de launch informó literalmente:

```text
[ERROR] [jaco_self_filter_node-8]: process has died [pid 82828, exit code -6,
cmd '.../jaco_self_filter_node --ros-args ...
--params-file .../jaco_d435i_shadow.yaml
--params-file .../jaco_self_filter_shadow_coverage_v1.yaml ...'].
```

La consulta observacional de nodos confirmó cámara, TF temporal, adaptador JACO2 de lectura y preprocesador, pero no `/jaco_self_filter_node`. Por tanto no fue posible confirmar el parámetro efectivo `capsule_profile_file`, el epsilon runtime, nube de entrada del filtro, diagnóstico, frame de salida ni conteos `self_confirmed`/`external_confirmed`/`occluded_unknown`.

No se ejecutó una segunda sesión ni un reintento. No se colocó ni probó ningún objeto o caja. No se enviaron comandos, trayectorias, jog, acciones o servicios al JACO2; `hardware_output=false` se mantuvo en el único intento. No se inició supervisor, obstacle bridge, extractor, GUI física, Gazebo, telemetría de mando ni módulo multi-pose.

## Perfil temporal solicitado

| Cápsulas | Radios base (m) | Radios v1 (m) |
|---|---|---|
| 0..5 | `[0.105, 0.105, 0.085, 0.075, 0.085, 0.090]` | `[0.105, 0.105, 0.100, 0.090, 0.100, 0.105]` |

El perfil siguió siendo una hipótesis temporal, de sesión y no apta para seguridad; no se promovió a una configuración permanente ni se modificaron extrínseca v2, ROI temporal o epsilon.

## Cierre de la sesión fallida

Se verificaron los procesos propios antes del cierre: terminal/launch 82788 y 82812; TF 82814; robot state 82816; adaptador JACO2 82818; readiness 82820; D435i 82822; preprocesador 82824; health 82826; RViz 82996. Se envió SIGINT a los procesos propios de launch y RViz. Una comprobación posterior confirmó que ya no existían procesos propios de launch, D435i, self-filter, adaptador JACO2, preprocesador, TF temporal o RViz. No se usó `kill -9` ni se terminó ningún proceso ajeno.

## Evidencia Git

`git diff --check` terminó sin salida ni errores. `git status --short` mostró el árbol de trabajo preexistente con archivos modificados y no rastreados, preservado sin sobrescrituras. Esta fase no cambió código, CMake, YAML, launch, parámetros, TF, cápsulas, epsilon, crop, RViz ni extrínseca.
