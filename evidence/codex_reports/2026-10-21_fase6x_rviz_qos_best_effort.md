# Fase 6X: RViz con QoS Best Effort para alineación visual

## Dictamen

`ALIGNMENT_VIEW_READY`

## Cambio exacto del perfil RViz

En cada bloque `Topic` de los cuatro displays PointCloud2 de `src/thesis_jaco_perception/rviz/jaco_d435i_alignment.rviz` se añadió `Depth: 5`, `Durability Policy: Volatile`, `History Policy: Keep Last` y `Reliability Policy: Best Effort`.

Se preservaron Fixed Frame `world`, RobotModel, TF y los colores: nube filtrada blanca, self-confirmed verde, external-candidate rojo y occluded-unknown amarillo.

## Sesión de sombra activa

Se inició una única sesión con ROS Humble, `install/setup.bash`, `ROS_DOMAIN_ID=78`, JACO2 de solo lectura, D435i, TF temporal candidato, self-filter sombra, `session_epsilon_total_m:=0.05` y `hardware_output:=false`.

RViz se abrió separadamente con el perfil corregido. No emitió la advertencia previa de incompatibilidad QoS.

## QoS y datos observados

Los cuatro publicadores fueron observados como `BEST_EFFORT` y `VOLATILE`. RViz quedó suscrito a `/thesis/perception/points_filtered` con `BEST_EFFORT` y `VOLATILE`; ese tópico tuvo tres suscriptores, incluyendo `rviz`.

| Display/tópico | Resultado observado |
|---|---|
| `/thesis/perception/points_filtered` | Recibido: `frame_id=world`, 1276 puntos; RViz suscrito con QoS compatible. |
| `/thesis/jaco_perception/points_self_confirmed` | Recibido: mensaje válido en `world`, 0 puntos. |
| `/thesis/jaco_perception/points_external_candidate` | Recibido: mensaje válido en `world`, 1289 puntos. |
| `/thesis/jaco_perception/points_occluded_unknown` | Recibido: mensaje válido en `world`, 0 puntos. |

Los mensajes vacíos son una salida válida del filtro para la alineación temporal actual; no son un fallo de QoS.

## Estado de hardware y TF

- JACO2: adaptador de solo lectura con `salida_fisica_permitida=false, armado=false`; `hardware_readiness` informó `READY`.
- D435i: RealSense D435i serie `405622074532`, profundidad y color activos a `640x480x30`.
- TF: `tf2_echo world d435i_depth_optical_frame` devolvió la transformación compuesta con traslación `(0.680, -0.030, 0.300)`; la cadena `world -> d435i_link -> ... -> d435i_depth_optical_frame` está disponible.
- El preprocesador publicó nube reciente en `world`.

## Seguridad y alcance

`hardware_output=false` se pasó explícitamente. No se enviaron comandos, trayectorias, jog ni servicios de movimiento; no se usó el módulo experimental multi-pose y no se alteró la transformación temporal ni ningún YAML de extrínseca.

No se iniciaron supervisor, obstacle bridge, extractor, GUI física, adaptadores de comando, Gazebo ni telemetría de mando. La sesión de sombra y RViz permanecen activos a la espera de la observación visual del operador.
