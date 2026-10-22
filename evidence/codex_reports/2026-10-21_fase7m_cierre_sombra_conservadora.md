# Fase 7M: cierre conservador de self-filter RGB-D en sombra

## Dictamen

`SHADOW_SELF_FILTER_PARTIALLY_VALIDATED_CONSERVATIVE`

## Alcance cerrado

La validación queda cerrada como evidencia parcial y conservadora de modo sombra. No promueve ninguna configuración a seguridad, supervisor, control físico ni `/thesis/obstacle_input`. Durante toda la fase `hardware_output=false`; no hubo movimiento, comandos, trayectorias, jog ni servicios enviados al JACO2. El supervisor, bridge de obstáculos, extractor, GUI física, Gazebo y adaptadores de mando nunca se iniciaron.

La extrínseca temporal v2 y el ROI descritos a continuación fueron exclusivos de esta sesión: no son configuración final ni de seguridad. El módulo multi-pose permanece experimental y no validado.

## Configuración temporal usada

| Elemento | Valor de sesión |
|---|---|
| Extrínseca | YAML temporal v2 `evidence/codex_reports/2026-10-21_d435i_extrinsic_visual_hypothesis_v2.yaml`; `world -> d435i_link`: traslación `(0.030, -0.680, 0.300)` m, cuaternión `(0.061628, -0.061628, 0.704416, 0.704416)` |
| Epsilon de sesión/self-filter | `session_epsilon_total_m=0.07` m; runtime `epsilon_total_m: 0.07` |
| ROI temporal en `world` | `enable_crop=true`; `x=[-0.55, 1.50]`, `y=[-0.50, 0.50]`, `z=[-0.25, 1.10]` m |
| Hardware | JACO2 sólo lectura/desarmado; `hardware_output=false` |
| Visualización | RViz con perfil QoS Best Effort, sólo durante la sesión cerrada |

## Evidencia conservadora obtenida

La extrínseca v2 permitió una alineación visual suficiente entre nube y modelo para análisis en sombra. No implica precisión reproducible ni autorización de control.

El cluster externo lejano medido antes del ROI tenía 363 puntos, centroide `(-0.504004, 1.384317, 0.030113)` m y bounds Y `[1.358694, 1.435653]` m. Con el ROI temporal quedó fuera por `y > 0.50`. Después del ROI, el probe de clusters observó un único cluster externo de 243 puntos, centroide `(-0.086313, -0.062930, 0.643087)` m y bounds Y `[-0.113492, -0.016132]`; no apareció el cluster lejano previo.

El diagnóstico del self-filter observado durante la sesión indicó `SHADOW_CLASSIFICATION_COMPLETE`, `configuration_valid=true` y `tf_status=AVAILABLE`, con una nube de 1315 puntos: 794 `self_confirmed`, 244 `external_confirmed` y 277 `occluded_unknown`. Por tanto, una parte del brazo se clasificó como propia, mientras que los puntos dudosos se conservaron como externos u ocluidos, sin ser eliminados.

La sonda de residual de rayo procesó tres nubes (3923 puntos válidos): 2341 `SELF_CONFIRMED`, 745 `EXTERNAL_CONFIRMED` y 837 `OCCLUDED_UNKNOWN`. Los 745 externos no tuvieron intersección con el volumen de cápsulas a lo largo del rayo cámara→punto. Por ese motivo no son candidatos para eliminación mediante aumento ciego de epsilon o radios. Las estadísticas por cápsula y residual `r-s` están en `docs/codex_reports/2026-10-21_fase7l_residual_rayo_equivalente.md`.

## Limitaciones conocidas

- La extrínseca v2 se validó visualmente sólo en esta sesión y sigue siendo temporal.
- El ROI excluye el fondo lejano de la postura actual; no es un volumen final ni una barrera de seguridad.
- El residual de rayo reutiliza la geometría del filtro, pero no se pudo emparejar punto a punto con una nube de diagnóstico idéntica: el diagnóstico no publica el sello de su nube de entrada. Su dictamen conservador fue `PROBE_STILL_NOT_EQUIVALENT`.
- No se validó integración con supervisor, `/thesis/obstacle_input`, control físico ni seguridad. Esas integraciones permanecen expresamente fuera de alcance.
- Multi-pose permanece experimental/no validado.

## Cierre controlado de la sesión nativa

Antes del cierre se verificó la pertenencia por PID, PPID, PGID, SID y comando:

| PID | Proceso propio verificado |
|---:|---|
| 77255 | `ros2 launch ... jaco_d435i_shadow_session.launch.py`, líder del grupo 77255, con extrínseca v2, epsilon 0.07, ROI temporal y `hardware_output=false` |
| 77280 | `d435i_session_candidate_extrinsic` (TF temporal) |
| 77282 | `robot_state_publisher` |
| 77284 | `jaco_hardware_adapter` en la sesión de lectura |
| 77286 | `hardware_readiness` |
| 77288 | `realsense2_camera_node` D435i |
| 77290 | `pointcloud_preprocessor_node` |
| 77292 | `perception_health_node` |
| 77294 | `jaco_self_filter_node` |
| 77464 | `rviz2 -d src/thesis_jaco_perception/rviz/jaco_d435i_alignment.rviz` |

Se enviaron SIGINT únicamente a los PIDs propios 77255 y 77464. RViz cerró de inmediato. Una comprobación a los dos segundos aún vio los hijos del launch; se intentó SIGINT al PGID 77255 previamente verificado, pero el sistema ya no encontró ese grupo. La comprobación inmediata siguiente confirmó que 77255 y todos sus hijos habían terminado. No se usó `kill -9` ni se terminó ningún proceso ajeno.

La búsqueda final no encontró procesos propios de `jaco_d435i_shadow_session`, `realsense2_camera_node`, `pointcloud_preprocessor_node`, `jaco_self_filter_node`, `jaco_hardware_adapter`, TF temporal ni el RViz de alineación.

## Evidencia de árbol Git

`git diff --check` terminó correctamente, sin salida ni errores.

`git status --short` confirma un árbol ya modificado, preservado sin sobrescrituras en este cierre. Entradas modificadas rastreadas:

```text
M AGENTS.md
M thesis_perception/CMakeLists.txt
M thesis_perception/config/perception.yaml
M thesis_perception/launch/d435i_standalone.launch.py
M thesis_perception/launch/perception_processing.launch.py
M thesis_perception/package.xml
M thesis_perception/test/test_perception_safety_contract.py
```

También reportó los informes y artefactos previos no rastreados bajo `docs/codex_reports/`, `evidence/codex_reports/`, `src/thesis_hardware/`, `src/thesis_jaco_perception/`, `thesis_perception/include/`, `thesis_perception/src/pointcloud_cluster_probe.cpp` y `thesis_perception/test/test_pointcloud_cluster_metrics.cpp`. No se modificó, eliminó, añadió ni promovió ningún archivo operativo en esta fase de cierre.

## Siguiente prerrequisito

Antes de cualquier nueva promoción, el siguiente prerrequisito es disponer de un soporte fijo para la D435i y una calibración reproducible con marcador. Sólo después correspondería planificar una validación nueva, independiente y en sombra; este cierre no autoriza el uso en seguridad o control físico.
