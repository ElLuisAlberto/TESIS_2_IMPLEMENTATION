# Fase 7L: residual de rayo equivalente al criterio de sombra

## Dictamen

`PROBE_STILL_NOT_EQUIVALENT`

## Alcance y garantías

Se modificaron únicamente los dos archivos autorizados:

- `src/thesis_jaco_perception/src/jaco_self_residual_probe.cpp`
- `src/thesis_jaco_perception/test/test_self_residual_metrics.cpp`

No se modificaron CMake, el self-filter, sus parámetros, cápsulas, epsilon,
TF, crop, YAML, launch ni configuración operativa. La sonda continúa sin
publicadores, TF propio, servicios ni cambios de parámetros; desactiva rosout
y servicios/eventos de parámetros. Sólo lee
`/thesis/perception/points_filtered` y TF. La sesión ROI, D435i, JACO2 y RViz
permanecieron activos e intactos, con `hardware_output=false`; no se enviaron
comandos físicos ni se iniciaron nodos prohibidos.

## Implementación del residual de rayo

La sonda usa `world -> d435i_depth_optical_frame` al sello exacto de cada
nube, las seis cápsulas actuales y radios `[0.105, 0.105, 0.085, 0.075,
0.085, 0.090]` m. Para cada punto `P` válido en `world`, con origen de cámara
`C`, calcula `r=||P-C||`, forma el rayo `{C, P-C}` y reutiliza directamente:

- `first_ray_capsules_intersection(...)` para la primera intersección `s` y
  su cápsula;
- `classify_observed_point(...)` para la clasificación real.

Ambas funciones son las que usa el self-filter. Con epsilon fijo de esta
sesión, `0.070000` m, las bandas son literalmente:

| Banda | Regla | Clase |
|---|---|---|
| antes del modelo | sin intersección, o `r < s - epsilon` | `EXTERNAL_CONFIRMED` |
| tolerancia | `abs(r-s) <= epsilon` | `SELF_CONFIRMED` |
| detrás del modelo | `r > s + epsilon` | `OCCLUDED_UNKNOWN` |

No se volvió a ejecutar la sonda previa de clearance lateral.

## Validación offline

`colcon build --packages-select thesis_jaco_perception --symlink-install`
terminó correctamente. La prueba única existente
`test_self_residual_metrics` pasó 3/3:

1. primera intersección conocida de rayo (`s=0.90` m);
2. los tres resultados de clasificación con `epsilon=0.07` m;
3. rayo sin intersección y resultado externo.

`git diff --check` terminó correctamente. No se ejecutaron pruebas físicas.

## Medición runtime única

La sonda se ejecutó una sola vez y terminó después de tres muestras (máximo
permitido), con sellos TF exactos `1792622811.818869873`,
`1792622811.852229004` y `1792622811.885588379`. Hubo un descarte inicial por
TF no disponible, cero nubes inválidas y cero puntos no finitos. Los conteos
agregados de las tres nubes fueron:

| Clase | Conteo | Observación |
|---|---:|---|
| `EXTERNAL_CONFIRMED` | 745 | 745 sin intersección de rayo; no hay cápsula asociable |
| `SELF_CONFIRMED` | 2341 | todos en la banda `[-0.07,+0.07]` m |
| `OCCLUDED_UNKNOWN` | 837 | todos con `r > s + 0.07` m |
| Total válido | 3923 | 0 puntos inválidos |

Residual `delta_ray=r-s` por clase y cápsula de primera intersección (m):

| Clase | Cápsula | Conteo | Mín. | Mediana | P90 | Máx. | Banda |
|---|---|---:|---:|---:|---:|---:|---|
| Propio | `shoulder_to_upper_arm` | 118 | -0.005084 | 0.052346 | 0.063876 | 0.069357 | `abs(delta)<=epsilon`: 118 |
| Propio | `upper_arm_to_forearm` | 1184 | -0.057254 | 0.023219 | 0.041683 | 0.058198 | `abs(delta)<=epsilon`: 1184 |
| Propio | `forearm_to_wrist_1` | 281 | -0.018608 | 0.009322 | 0.028949 | 0.035384 | `abs(delta)<=epsilon`: 281 |
| Propio | `wrist_1_to_wrist_2` | 237 | -0.018502 | 0.029998 | 0.051675 | 0.069950 | `abs(delta)<=epsilon`: 237 |
| Propio | `wrist_2_to_tool` | 521 | -0.038380 | 0.037379 | 0.063047 | 0.069562 | `abs(delta)<=epsilon`: 521 |
| Ocluido | `shoulder_to_upper_arm` | 706 | 0.070037 | 0.097636 | 0.119242 | 0.202192 | `delta>epsilon`: 706 |
| Ocluido | `wrist_1_to_wrist_2` | 28 | 0.071521 | 0.075531 | 0.079782 | 0.081015 | `delta>epsilon`: 28 |
| Ocluido | `wrist_2_to_tool` | 103 | 0.070217 | 0.103515 | 0.131593 | 0.137157 | `delta>epsilon`: 103 |

Las entradas de cápsula/clase no listadas tuvieron conteo cero. No hubo punto
con intersección en la banda externa `r < s-epsilon`: los 745 externos no
intersectaron ningún volumen de cápsula.

## Comparación con el diagnóstico del self-filter

La única consulta observacional posterior recibió el diagnóstico publicado:

| Fuente | Muestras | Total | Propio | Externo | Ocluido |
|---|---:|---:|---:|---:|---:|
| Sonda, agregado | 3 | 3923 | 2341 | 745 | 837 |
| Sonda, media por nube | 1 | 1307.667 | 780.333 | 248.333 | 279.000 |
| Diagnóstico observado | 1 | 1305 | 775 | 249 | 281 |
| Diferencia media de sonda − diagnóstico | — | +2.667 | +5.333 | -0.667 | -2.000 |

El diagnóstico fue `SHADOW_CLASSIFICATION_COMPLETE`,
`configuration_valid=true`, `tf_status=AVAILABLE` y
`cloud_stamp_age_sec=0.113520`; su propio encabezado fue
`1792622817.503254997`. No contiene el sello de la nube de entrada que
clasificó. Al estimar ese sello con el valor de edad, corresponde
aproximadamente a `1792622817.389734977`, más de cinco segundos posterior al
último sello de la sonda. Por tanto no hay evidencia de que sea la misma nube,
y los conteos no son una comparación por sello equivalente. La diferencia
literal anterior se registra sin atribuirla a una discrepancia de algoritmo.

## Motivo del dictamen

La métrica de rayo y las tres reglas de clasificación ahora reutilizan las
funciones geométricas del self-filter, y los resultados respetan sus bandas.
Sin embargo, la sonda conserva por restricción los radios y epsilon como
constantes del perfil/sesión actual, en vez de leer un snapshot runtime, y el
diagnóstico del self-filter no expone el sello de nube necesario para emparejar
una muestra. No es posible demostrar la equivalencia runtime punto a punto ni
comparar el diagnóstico de una nube idéntica con la única ejecución autorizada.
Por ello, sin cambiar nada operativo ni repetir la medición, el dictamen
conservador es `PROBE_STILL_NOT_EQUIVALENT`.
