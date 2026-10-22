# Fase 6Z: sombra con epsilon temporal 0.06 m

## Estado

`EPSILON_VIEW_READY`

## Cambio único de sesión

Se cerró primero mediante SIGINT la sesión v2 previa y sus procesos propios finalizaron limpiamente. Se inició una sola sesión nueva con el mismo YAML temporal `evidence/codex_reports/2026-10-21_d435i_extrinsic_visual_hypothesis_v2.yaml` y el mismo perfil RViz QoS Best Effort.

La extrínseca v2 no cambió: `world -> d435i_link`, traslación `[0.03, -0.68, 0.30]`, RPY `[0.0, -10.0, 90.0]`. El cambio exclusivo fue `session_epsilon_total_m:=0.05` a `session_epsilon_total_m:=0.06`.

## Estado observado

- JACO2: solo lectura, `salida_fisica_permitida=false`, `armado=false`; `hardware_readiness` informó `READY`.
- D435i: activa; el preprocesador entregó una nube reciente en `world` de 2438 puntos.
- TF: `tf2_echo world d435i_link` devolvió traslación `(0.030, -0.680, 0.300)` y RPY `(0.000, -10.000, 90.000)`.
- RViz: activo con el perfil QoS Best Effort, sin advertencias de incompatibilidad; recibe la nube filtrada.
- Nodos: están presentes solo la cámara, TF temporal, estado JACO2 de solo lectura, preprocesado, health, self-filter, robot_state_publisher y RViz. No aparecen supervisor, obstacle bridge, extractor ni nodos de mando.

## Evidencia del self-filter

Una muestra por tópico, todas con `frame_id=world`, produjo los siguientes conteos:

| Salida | Puntos |
|---|---:|
| `/thesis/jaco_perception/points_self_confirmed` | 727 |
| `/thesis/jaco_perception/points_external_candidate` | 907 |
| `/thesis/jaco_perception/points_occluded_unknown` | 424 |

La separación se conserva: no se modificó TF, geometría, cápsulas ni ningún otro parámetro del filtro.

## Seguridad

La sesión se lanzó con `hardware_output=false`. No se enviaron comandos, trayectorias, jog ni servicios de movimiento al JACO2. No se inició supervisor, obstacle bridge, extractor, GUI física, Gazebo, adaptadores de comando ni el módulo experimental multi-pose. No se modificó YAML, código, CMake, launch, perfil RViz ni la extrínseca v2.

La sesión y RViz permanecen activos para la captura del operador.
