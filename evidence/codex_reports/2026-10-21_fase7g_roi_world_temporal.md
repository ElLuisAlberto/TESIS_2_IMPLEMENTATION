# Fase 7G: ROI temporal en `world`

## Dictamen

`BLOCKED_RUNTIME`

## Bloqueo del contrato de launch

No se cerró la sesión nativa actual ni se inició una nueva sesión, para no
interrumpir una sesión funcional cuando el ROI solicitado no puede ser aplicado
sin modificar archivos.

`ros2 launch thesis_jaco_perception jaco_d435i_shadow_session.launch.py
--show-args` confirma que el launch de sesión sólo declara:

- `start_jaco_read_only`
- `start_camera`
- `start_rviz`
- `enable_self_filter`
- `session_epsilon_total_m`
- `publish_session_extrinsic`
- `candidate_extrinsic_file`
- `hardware_output`

No declara `enable_crop`, `min_x`, `max_x`, `min_y`, `max_y`, `min_z` ni
`max_z`.

Además, `d435i_shadow_camera_pipeline.launch.py` crea un `GroupAction` con
`forwarding=False` y fija internamente:

| Argumento interno | Valor fijado |
|---|---:|
| `enable_crop` | `false` |
| `min_x`, `max_x` | `-1.5`, `1.5` |
| `min_y`, `max_y` | `-1.5`, `1.5` |
| `min_z`, `max_z` | `-1.0`, `2.5` |

Por tanto, los argumentos de ROI solicitados no alcanzan el
`pointcloud_preprocessor`. Cambiar estos valores exigiría modificar el
sublaunch o su contrato de argumentos, acción expresamente prohibida. El
ajuste de parámetros en runtime tampoco es una alternativa: el preprocesador
copia los límites a miembros privados al construir el nodo y no registra un
callback de parámetros para aplicar cambios posteriores.

## ROI solicitado, no aplicado

El ROI de prueba propuesto se revisó pero no se activó:

| Límite | Valor (m) |
|---|---:|
| `min_x`, `max_x` | `-0.55`, `1.50` |
| `min_y`, `max_y` | `-0.50`, `0.50` |
| `min_z`, `max_z` | `-0.25`, `1.10` |

El cluster lejano medido previamente, con centroide
`(-0.504004, 1.384317, 0.030113)` m y bounds Y `[1.358694, 1.435653]` m,
quedaría fuera por `y > 0.50`. La envolvente de cápsulas observada y los
escenarios autorizados documentados permanecen dentro de esos límites, pero
esa compatibilidad no sustituye el contrato de launch faltante.

## Estado preservado y seguridad

La sesión nativa actual sigue activa con el YAML temporal v2, epsilon `0.07`,
`hardware_output=false`, JACO2 de solo lectura, D435i, self-filter y RViz de
alineación. No se cerró, reinició ni modificó ningún proceso. No se ejecutó el
probe otra vez, no se activó crop y no se alteraron TF, cápsulas, YAML,
código, CMake, extrínseca ni configuración permanente.

No se enviaron comandos, trayectorias, jog ni servicios al JACO2. No se inició
supervisor, obstacle bridge, extractor, GUI física, Gazebo, adaptadores de
comando ni el módulo experimental multi-pose.
