# Fase 6Y: hipótesis temporal de extrínseca visual v2

## Estado

`ALIGNMENT_VIEW_READY_V2`

## YAML anterior y nuevo

El YAML anterior se preservó sin cambios: `evidence/codex_reports/2026-10-21_d435i_extrinsic_session_candidate.yaml`. Su hipótesis era `translation: [0.68, -0.03, 0.30]` y cuaternión `[0.0, -0.0871557427, 0.0, 0.9961946981]`.

Se creó la nueva copia reversible `evidence/codex_reports/2026-10-21_d435i_extrinsic_visual_hypothesis_v2.yaml`. Conserva `parent_frame: world`, `child_frame: d435i_link`, `session_only: true` y `not_for_safety: true`.

| Parámetro | Anterior | Hipótesis v2 |
|---|---:|---:|
| X (m) | 0.68 | 0.03 |
| Y (m) | -0.03 | -0.68 |
| Z (m) | 0.30 | 0.30 |
| RPY (grados) | no expresado como RPY | `[0.0, -10.0, 90.0]` |

La representación compatible con el launch de v2 usa `translation: [0.03, -0.68, 0.30]` y `quaternion_xyzw: [0.0616284167, -0.0616284167, 0.7044160264, 0.7044160264]`.

`x=0.03 m` es una hipótesis de desplazamiento frontal pequeño; `y=-0.68 m` representa la cámara a la derecha del JACO2; `z=0.30 m` es una altura aproximada; y `yaw=+90°` orienta la cámara hacia el brazo. Es solo una prueba visual reversible, no una calibración ni una entrada de seguridad.

## Sesión activa

Se cerró primero la sesión Fase 6X mediante SIGINT; sus procesos propios finalizaron limpiamente. Se inició una única sesión nueva con JACO2 de solo lectura, D435i, TF temporal v2, self-filter sombra, `session_epsilon_total_m:=0.05` y `hardware_output:=false`. RViz usa el perfil QoS Best Effort ya corregido.

El adaptador JACO2 informó `salida_fisica_permitida=false, armado=false`; `hardware_readiness` informó `READY`. La D435i serie `405622074532` inició profundidad y color a `640x480x30`.

## Nube, TF y RViz

- `/thesis/perception/points_filtered` entregó una muestra reciente con `frame_id=world` y 2145 puntos.
- RViz abrió con el perfil corregido y no emitió advertencias de incompatibilidad QoS.
- El publicador temporal confirmó `world -> d435i_link` con traslación `(0.030, -0.680, 0.300)` y cuaternión v2.
- `tf2_echo world d435i_depth_optical_frame` devolvió la transformación compuesta con esa traslación, confirmando la cadena hasta el frame óptico.

## Seguridad

No se enviaron comandos, trayectorias, jog ni servicios de movimiento. No se inició supervisor, obstacle bridge, extractor, GUI física, Gazebo ni adaptadores de comando. No se usó el módulo experimental multi-pose. No se alteró el YAML final, el TF durante la sesión ni la configuración de seguridad.

La sesión de sombra y RViz permanecen activos para la nueva captura visual del operador.
