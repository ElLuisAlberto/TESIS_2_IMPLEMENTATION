# Fase 7I: validación de sombra con ROI temporal en `world`

## Dictamen

`ROI_SHADOW_VIEW_READY`

## Cierre controlado de la sesión anterior

Se comprobó primero la pertenencia mediante PID, PPID, PGID, SID y comando.
El launch propio anterior era PID `71580`; sus hijos propios eran TF temporal
`71605`, adaptador JACO2 de solo lectura `71609`, D435i `71613` y self-filter
`71619`, todos con PPID/PGID/SID del launch. La ventana RViz propia de
alineación era PID `71777`.

Se enviaron SIGINT únicamente a `71580` y `71777`. Los hijos del launch
finalizaron limpiamente y se verificó que no permanecía ningún proceso de esa
sesión. No se usó SIGKILL ni se terminó ningún proceso ajeno.

## Nueva sesión nativa y argumentos reales

Se abrieron dos terminales nativas independientes: el launch ROI (PID `77255`)
y RViz de alineación (PID `77464`). Los procesos propios actuales incluyen TF
temporal `77280`, JACO2 read-only `77284`, D435i `77288` y self-filter `77294`.

El launch real usó el YAML temporal v2 sin cambios,
`session_epsilon_total_m:=0.07`, `hardware_output:=false`,
`start_jaco_read_only:=true`, `start_camera:=true`, self-filter habilitado y:

| Argumento ROI | Valor |
|---|---:|
| `enable_crop` | `true` |
| `min_x`, `max_x` | `-0.55`, `1.50` m |
| `min_y`, `max_y` | `-0.50`, `0.50` m |
| `min_z`, `max_z` | `-0.25`, `1.10` m |

El volcado de parámetros de `/pointcloud_preprocessor` confirmó los siete
valores, `target_frame=world`, `max_valid_range_m=1.5` m por default de este
launch nuevo y voxel de `0.01` m. No hubo cambio runtime de rango, epsilon o
TF.

## Verificación de nube, TF y seguridad

- `/thesis/hardware/armed` publicó `false`; JACO2 permanece conectado sólo en
  lectura y `hardware_output=false` está explícito en el launch.
- `/thesis/perception/points_filtered` publicó `frame_id=world` y 1321 puntos.
- `tf2_echo world d435i_link` confirmó v2: traslación `(0.030, -0.680, 0.300)`
  m y RPY `(0.000, -10.000, 90.000)` grados.
- RViz está suscrito a la nube filtrada con `BEST_EFFORT` y `VOLATILE`,
  compatible con el único publicador. El `RobotModel` y la nube permanecen
  disponibles para la revisión visual del operador.
- La lista de nodos contiene sólo cámara, TF temporal, lectura JACO2,
  preprocesador, health, self-filter, robot-state publisher, RViz y listeners
  TF; no contiene supervisor, obstacle bridge, extractor ni nodos de mando.

La implementación del preprocesador aplica `CropBox` tras transformar a
`world`; con los límites activos anteriores, todo punto publicado en
`/thesis/perception/points_filtered` satisface el ROI. No se ejecutó un
segundo probe sobre la nube completa, respetando la medición única solicitada
para el tópico externo.

## Medición de clusters posterior y comparación

Se ejecutó una sola vez `pointcloud_cluster_probe` sobre
`/thesis/jaco_perception/points_external_candidate`, con tolerancia `0.04` m,
mínimo 20 puntos, máximo 200000 y timeout máximo 5 s. Recibió `frame=world`,
250 puntos y un único cluster:

| Sesión | Puntos | Centroide XYZ (m) | Bounds Y (m) | Distancia al origen (m) |
|---|---:|---|---|---:|
| Antes del ROI, cluster lejano | 363 | `(-0.504004, 1.384317, 0.030113)` | `[1.358694, 1.435653]` | 1.473519 |
| Con ROI, único cluster | 243 | `(-0.086313, -0.062930, 0.643087)` | `[-0.113492, -0.016132]` | 0.651898 |

No existe cluster posterior con bounds Y cercanos a
`[1.358694, 1.435653]`; el grupo lejano quedó excluido por `max_y=0.50`.
El cluster restante está dentro del volumen de brazo/escenario de prueba y la
nube válida continúa publicada.

## Alcance preservado

El ROI es exclusivo de esta sesión temporal, no es configuración final ni de
seguridad. En esta fase no se modificaron archivos, YAML, TF, cápsulas, código,
CMake, launch, extrínseca ni defaults permanentes. No se enviaron comandos,
trayectorias, jog ni servicios al JACO2; no se iniciaron supervisor, obstacle
bridge, extractor, GUI física, Gazebo, adaptadores de comando, telemetría de
mando ni el módulo multi-pose. Launch y RViz se mantienen activos para captura.
