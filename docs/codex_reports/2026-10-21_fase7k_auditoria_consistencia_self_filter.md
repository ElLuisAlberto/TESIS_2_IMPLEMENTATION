# Fase 7K: auditoría de consistencia entre sonda residual y self-filter

## Dictamen

`PROBE_NOT_EQUIVALENT_TO_SELF_FILTER`

## Alcance de la auditoría

Se hizo únicamente inspección estática y consultas ROS de observación. No se
cerró ni reinició la sesión ROI, no se ejecutó otra sonda y no se modificaron
parámetros, archivos, TF, cápsulas, epsilon, crop o rango. No hubo comandos,
trayectorias, jog ni servicios hacia el JACO2; `hardware_output=false` se
mantiene en la sesión activa.

## Contrato de `session_epsilon_total_m`

La trazabilidad de launch sí propaga el epsilon:

1. `jaco_d435i_shadow_session.launch.py:147` declara
   `session_epsilon_total_m` con default literal `""`. Cuando el self-filter
   está habilitado, `_validate_session_epsilon` exige valor numérico, finito y
   no negativo (`:66-79`).
2. El mismo launch lo reenvía como
   `"epsilon_total_m": LaunchConfiguration("session_epsilon_total_m")`
   al sublaunch (`:176-184`).
3. `jaco_rgbd_shadow.launch.py:79` declara `epsilon_total_m` con default
   `"-1.0"` para uso aislado; `:94-95` lo entrega al nodo como
   `ParameterValue(..., value_type=float)`. Por tanto no hay conversión de
   unidad: el texto de launch se convierte a `double` en metros.
4. `jaco_self_filter_node.cpp:94` declara el parámetro real
   `epsilon_total_m`; el filtro lo pasa sin transformación a
   `partition_observed_points` (`:212-213`).

El default de sesión vacío no constituye un epsilon operativo: la validación
lo rechaza al habilitar el filtro. El valor runtime observado es `0.07` m.

## Estado runtime observado

`ros2 node list --no-daemon --spin-time 2` identificó el nodo activo
`/jaco_self_filter_node`. `ros2 param dump /jaco_self_filter_node --no-daemon
--spin-time 2` devolvió literalmente:

| Área | Parámetro y valor efectivo |
|---|---|
| Activación/clasificación | `enabled: true`; `epsilon_total_m: 0.07` |
| Entrada/frame | `input_topic: /thesis/perception/points_filtered`; `expected_frame: world` |
| Cámara/TF | `camera_optical_frame: d435i_depth_optical_frame`; `tf_timeout_sec: 0.05`; `use_sim_time: false` |
| Sello | `max_cloud_age_sec: 0.5` |
| Cápsulas inicio | `[j2n6s300_link_base, j2n6s300_link_1, j2n6s300_link_2, j2n6s300_link_3, j2n6s300_link_4, j2n6s300_link_5]` |
| Cápsulas fin | `[j2n6s300_link_1, j2n6s300_link_2, j2n6s300_link_3, j2n6s300_link_4, j2n6s300_link_5, j2n6s300_end_effector]` |
| Radios (m) | `[0.105, 0.105, 0.085, 0.075, 0.085, 0.09]` |

No existen parámetros runtime de normales, visibilidad, raycasting u
oclusión: esas reglas son parte fija de la lógica compilada. El diagnóstico
observado publicó `SHADOW_CLASSIFICATION_COMPLETE`,
`configuration_valid=true`, `cloud_stamp_age_sec=0.089692`,
`tf_status=AVAILABLE`, y los conteos de esa nube fueron
`total_points=1315`, `self_confirmed_points=794`,
`external_confirmed_points=244`, `occluded_unknown_points=277`. El tópico
`/thesis/jaco_perception/points_external_candidate` tiene como único
publicador a `jaco_self_filter_node` y QoS Best Effort/Volatile.

## Lógica de clasificación real

La implementación está en
`src/thesis_jaco_perception/src/self_filter_geometry.cpp:194-218`, invocada
desde `shadow_filter_logic.cpp:79-106` y
`jaco_self_filter_node.cpp:142-218`.

Antes de clasificar, el nodo exige: filtro habilitado, configuración válida,
`frame_id == "world"`, sello no nulo, no futuro y con edad no mayor a
`max_cloud_age_sec`; después obtiene TF exacto al sello de la nube para origen
de cámara y cada extremo de cápsula. Si TF falta dentro de `tf_timeout_sec`,
no publica una partición normal. Los puntos no finitos se omiten.

Para un punto observado `P`, origen de cámara `C`, dirección
`u=(P-C)/||P-C||` y distancia observada `r=||P-C||`, el filtro calcula:

`s = min distancia t >= 0 donde C + t*u intersecta alguna cápsula`.

Las intersecciones consideran cilindro finito y las dos tapas esféricas de
cada cápsula (`self_filter_geometry.cpp:116-174`), y se elige la primera de
todas (`:176-191`). Con `ε = epsilon_total_m`, la ecuación exacta es:

| Condición | Salida |
|---|---|
| no hay intersección de rayo, o `r < s - ε` | `EXTERNAL_CONFIRMED` → `points_external_candidate` |
| `abs(r - s) <= ε` | `SELF_CONFIRMED` → `points_self_confirmed` |
| en otro caso (`r > s + ε`) | `OCCLUDED_UNKNOWN` → `points_occluded_unknown` |

No se aplican normales ni un margen adicional. Sí hay una condición de
visibilidad/raycasting geométrica: el modelo debe interceptar el mismo rayo
que parte de la cámara. La oclusión es precisamente el caso posterior a la
primera intersección del modelo.

## Comparación con `jaco_self_residual_probe`

| Aspecto | Self-filter | Sonda residual | Resultado |
|---|---|---|---|
| Perfil geométrico | Parámetros runtime de seis cápsulas | Constantes en `jaco_self_residual_probe.cpp:33-42` | Coinciden actualmente con el dump y YAML activo; la sonda no lee los parámetros runtime. |
| Frame de nube/cápsulas | Exige `world`; TF `world` | Exige `world`; TF `world` | Equivalente en frame. |
| Sello TF | TF exacto al sello de la nube (`jaco_self_filter_node.cpp:142-161`) | TF exacto al sello de cada nube recibida (`jaco_self_residual_probe.cpp:105-125`) | Mismo criterio temporal, pero no las mismas muestras: la sonda analiza mensajes ya publicados por el filtro. |
| Cámara y rayo | Usa `world -> d435i_depth_optical_frame`, rayo `C→P` y primera intersección | No consulta origen de cámara ni calcula rayo/intersección | Diferente. |
| Asociación de cápsula | Cápsula cuya superficie corta primero el rayo de cámara | Cápsula con mínimo `d(P, segmento) - radio` | Diferente. |
| Magnitud comparada | Residual radial de profundidad `r-s` | Clearance euclídeo lateral/3D `d(P, segmento)-radio` | Diferente. |
| Epsilon | `0.07` m, comparado con `abs(r-s)` | No usa epsilon ni compara clearance contra 0.07 | Diferente. |
| Resultado | Tres clases mutuamente excluyentes | Sólo estadística `AMBIGUOUS_EXTERNAL_NEAR_CAPSULE` | No reclasifica, por diseño. |

La medición previa de la sonda (722 puntos ya publicados como externos) tuvo
clearance máximo `0.043331 m`. Esa desigualdad
`0.043331 < 0.07` no contradice el self-filter: compara una distancia a la
superficie de la cápsula más cercana, mientras que el filtro decide con la
diferencia de distancia sobre el rayo desde la cámara a la primera superficie
interceptada. Un punto puede ser cercano lateralmente a una cápsula y, aun así,
no tener una intersección en su rayo o estar delante de dicha intersección por
más de epsilon; en ambos casos permanece `EXTERNAL_CONFIRMED`.

## Conclusión

`EPSILON_NOT_PROPAGATED` queda descartado por la cadena de launch y el dump
runtime literal `epsilon_total_m: 0.07`. Aunque el self-filter incorpora la
regla adicional de raycasting/oclusión, la causa determinante de que el
clearance de la sonda no prediga la clase es que la sonda no implementa la
misma magnitud ni el mismo criterio de segmento/rayo. Por ello el dictamen
único es `PROBE_NOT_EQUIVALENT_TO_SELF_FILTER`.
