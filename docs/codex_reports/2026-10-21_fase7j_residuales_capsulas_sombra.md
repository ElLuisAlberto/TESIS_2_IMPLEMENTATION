# Fase 7J: residuales por cápsula en sombra

## Dictamen

`SELF_RESIDUAL_MEASUREMENT_READY`

## Alcance y garantías observacionales

Con la autorización posterior para añadir exclusivamente los targets nuevos,
se implementó `jaco_self_residual_probe` en `thesis_jaco_perception`. La
sonda tiene solamente las suscripciones a
`/thesis/jaco_perception/points_external_candidate`, `/joint_states` y las
suscripciones internas necesarias del listener TF. Deshabilita `rosout`, los
servicios de parámetros y los eventos de parámetros; no crea publicadores, TF,
servicios ni escribe configuración.

Exige `frame_id=world`, procesa como máximo tres muestras y termina sola antes
de cinco segundos. La ejecución usó tres muestras y terminó a los 2.4 s. No
reclasifica puntos, no altera radios y no propone perfiles de cápsulas: todos
los resultados se etiquetan explícitamente
`AMBIGUOUS_EXTERNAL_NEAR_CAPSULE`.

Se añadieron sólo los targets nuevos del ejecutable y de la prueba a
`src/thesis_jaco_perception/CMakeLists.txt`; no se editaron targets,
dependencias, opciones, pruebas ni lógica preexistentes. Las dependencias
empleadas (`geometry_msgs`, `rclcpp`, `sensor_msgs`, `tf2_ros` y la geometría
local del self-filter) ya estaban declaradas; no se modificó `package.xml`.

## Geometría de cápsulas usada

La sonda replica el perfil activo de seis cápsulas del self-filter:

| Cápsula | Segmento | Radio (m) |
|---:|---|---:|
| 0 | `base_to_shoulder`: `j2n6s300_link_base` → `j2n6s300_link_1` | 0.105 |
| 1 | `shoulder_to_upper_arm`: `j2n6s300_link_1` → `j2n6s300_link_2` | 0.105 |
| 2 | `upper_arm_to_forearm`: `j2n6s300_link_2` → `j2n6s300_link_3` | 0.085 |
| 3 | `forearm_to_wrist_1`: `j2n6s300_link_3` → `j2n6s300_link_4` | 0.075 |
| 4 | `wrist_1_to_wrist_2`: `j2n6s300_link_4` → `j2n6s300_link_5` | 0.085 |
| 5 | `wrist_2_to_tool`: `j2n6s300_link_5` → `j2n6s300_end_effector` | 0.090 |

Para cada punto rojo se seleccionó la cápsula con menor
`distancia_al_eje - radio` y se informó ese valor como `clearance`.

## Medición única en runtime

La única ejecución recibió tres nubes en `world`, con TF disponible para cada
muestra. Sellos TF usados: `1792622381.762194092`,
`1792622381.795550049` y `1792622382.329283447`; no hubo descartes por TF,
estado articular o nube inválida. Se evaluaron 722 puntos externos.

| Cápsula | Puntos | Mín. (m) | Mediana (m) | P90 (m) | Máx. (m) | (0,.01] | (.01,.02] | (.02,.04] | (.04,.08] | >.08 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 `base_to_shoulder` | 0 | — | — | — | — | 0 | 0 | 0 | 0 | 0 |
| 1 `shoulder_to_upper_arm` | 5 | .007208 | .007585 | .007585 | .007585 | 5 | 0 | 0 | 0 | 0 |
| 2 `upper_arm_to_forearm` | 166 | .006069 | .024016 | .038636 | .043331 | 3 | 47 | 104 | 12 | 0 |
| 3 `forearm_to_wrist_1` | 169 | .002092 | .014104 | .025510 | .038480 | 57 | 71 | 41 | 0 | 0 |
| 4 `wrist_1_to_wrist_2` | 189 | .004850 | .016175 | .029168 | .035249 | 54 | 67 | 68 | 0 | 0 |
| 5 `wrist_2_to_tool` | 193 | .000659 | .012024 | .026634 | .033428 | 80 | 60 | 53 | 0 | 0 |

La banda no positiva tuvo cero puntos para todas las cápsulas. Estos datos
indican proximidad geométrica a las cápsulas, pero permanecen ambiguos: no son
evidencia de que esos puntos sean brazo ni autorizan modificar un radio.

## Validación

- `colcon build --packages-select thesis_jaco_perception --symlink-install`:
  correcto.
- `test_self_residual_metrics`: 2/2 pruebas correctas. Cubre clearance
  sintético conocido, asociación a la cápsula más cercana, bandas y
  percentiles; no usa ROS ni hardware.
- `git diff --check`: correcto.

## Estado preservado

La sesión ROI nativa y RViz permanecieron activos durante toda la fase. No se
modificaron parámetros de runtime, crop, rango, epsilon, TF, YAML, cápsulas,
extrínseca, launches ni configuración operativa. `hardware_output=false` se
conservó. No se enviaron comandos, trayectorias, jog ni servicios al JACO2, y
no se inició supervisor, obstacle bridge, extractor, GUI física, Gazebo ni
nodos de mando.
