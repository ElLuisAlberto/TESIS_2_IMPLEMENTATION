# Fase 7A: sombra epsilon temporal 0.07 m

## Estado

`BLOCKED_RUNTIME`

## Sesión única solicitada

Se cerró mediante SIGINT la sesión v2 previa (`session_epsilon_total_m:=0.06`),
incluidos sus procesos propios y RViz. Se inició una sola sesión de sombra con
el mismo YAML temporal v2, sin modificarlo:

`evidence/codex_reports/2026-10-21_d435i_extrinsic_visual_hypothesis_v2.yaml`

El lanzamiento cambió exclusivamente `session_epsilon_total_m:=0.06` por
`session_epsilon_total_m:=0.07`; conservó `hardware_output:=false`, JACO2 en
solo lectura, D435i, self-filter y RViz con el perfil QoS Best Effort existente.

## TF y arranque observado

TF v2 no cambió. El publicador temporal informó `world -> d435i_link` con
traslación `(0.030, -0.680, 0.300)` y cuaternión
`(0.061628, -0.061628, 0.704416, 0.704416)`.

Durante el arranque, el adaptador JACO2 informó
`salida_fisica_permitida=false, armado=false`; `hardware_readiness` informó
`READY`. La D435i serie `405622074532` inició profundidad y color a
`640x480x30`. RViz abrió con el perfil existente de Best Effort.

## Bloqueo de runtime

La terminal integrada finalizó los procesos de fondo de la sesión al concluir
su ejecución, antes de las consultas de observación. En consecuencia, no quedó
la sesión ni RViz activos para la captura y no fue posible obtener muestras de
las nubes en `world` ni los conteos disponibles de puntos:

| Salida | Conteo |
|---|---:|
| Verde: `/thesis/jaco_perception/points_self_confirmed` | no disponible |
| Rojo: `/thesis/jaco_perception/points_external_candidate` | no disponible |
| Amarillo: `/thesis/jaco_perception/points_occluded_unknown` | no disponible |

No se hizo reintento, conforme a la restricción de una sola sesión.

## Seguridad y alcance

No se modificó YAML, TF, cápsulas, código, launch, RViz ni CMake. No se usó el
módulo experimental multi-pose. No se iniciaron supervisor, obstacle bridge,
extractor, GUI física, Gazebo ni nodos de comando. No se enviaron comandos,
trayectorias, jog ni servicios de movimiento al JACO2.
