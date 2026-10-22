# Fase 7S: medición cuantitativa de residuales para `coverage_v1`

## Dictamen

`COVERAGE_V2_MEASUREMENT_READY`

## Alcance y garantías observacionales

Se modificaron exclusivamente los archivos autorizados:

- `src/thesis_jaco_perception/src/jaco_self_residual_probe.cpp`
- `src/thesis_jaco_perception/test/test_self_residual_metrics.cpp`

La sonda sigue siendo de sólo lectura. Sólo crea una suscripción Best Effort a
`/thesis/jaco_perception/points_external_candidate` y un listener TF; no crea
publicadores, servicios, TF, clientes de parámetros ni escritura YAML. Sus
opciones de nodo deshabilitan `rosout`, servicios de parámetros y el
publicador de eventos de parámetros. Procesa como máximo tres nubes, tiene un
timeout de cinco segundos y se apaga sola. La asociación a cápsulas y los
radios requeridos son únicamente diagnósticos: no reclasifican puntos ni
modifican ninguna salida ROS.

No se creó ni cargó `jaco_self_filter_shadow_coverage_v2.yaml`. No se
modificaron parámetros runtime, ROI, epsilon, TF, extrínseca, cápsulas activas,
launches ni YAML operativo. La sesión coverage_v1, D435i, RViz y JACO2 se
mantuvieron activos e intactos; no hubo comandos, trayectorias, jog, acciones
ni servicios físicos. `hardware_output=false` permaneció vigente.

## Implementación de la métrica

Se añadió `--capsule-radii-m`, un CSV estricto de seis radios en metros. El
parser rechaza cantidad distinta de seis, valores no numéricos/no finitos y
valores no positivos antes de inicializar ROS. Si no se entrega, deja explícito
en el log que usa los radios históricos de la sonda; esta medición lo entregó
de forma explícita, por lo que su fuente fue `custom_csv`.

Para cada punto rojo válido, la sonda conserva el raycasting real contra las
seis cápsulas y calcula adicionalmente la distancia al segmento finito de cada
cápsula. La asociación diagnóstica se hace a la mínima distancia de superficie
`distancia_al_eje - radio_v1`. Para esa cápsula calcula:

```text
clearance_m = distance_to_finite_segment_m - radio_v1_m
required_radius_m = distance_to_finite_segment_m + 0.003
```

Las etiquetas se aplican estrictamente: `LOCAL_COVERAGE_CANDIDATE` requiere al
menos 20 puntos, `0 < incremento_P95 <= 0.025 m` y `P99-P95 <= 0.020 m`.
Cualquier grupo escaso, de expansión P95 mayor, de cola P99 amplia o sin
incremento positivo queda conservadoramente como
`EXCESSIVE_OR_AMBIGUOUS_EXPANSION`; cero puntos es `NO_RESIDUAL`.

## Radios v1 y medición única

Comando observacional único:

```text
ros2 run thesis_jaco_perception jaco_self_residual_probe --capsule-radii-m 0.105,0.105,0.100,0.090,0.100,0.105
```

Radios v1 usados (m):
`[0.105, 0.105, 0.100, 0.090, 0.100, 0.105]`.

La sonda usó TF exacto en `world` para las tres muestras, con sellos:

- `1792628226.432065918`
- `1792628226.498781250`
- `1792628226.532139160`

Resumen literal relevante: `samples_used=3`, `points_valid=365`,
`external=365`, `self=0`, `occluded=0`, `no_intersection=365`, `invalid=0`,
`discarded_tf=0`, `discarded_invalid_cloud=0`. Es decir, los 365 puntos de la
entrada ya roja no intersectaron ninguna cápsula en el rayo cámara→punto;
esto no impide su asociación geométrica diagnóstica al segmento más cercano.

| Cápsula | Puntos | Radio v1 m | Distancia eje P50 m | Clearance P50 m | Required P50 m | Required P90 m | Required P95 m | Required P99 m | Incremento P95 m | Incremento P99 m | Máximo required m | Etiqueta |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 0 `base_to_shoulder` | 0 | 0.105 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a | `NO_RESIDUAL` |
| 1 `shoulder_to_upper_arm` | 0 | 0.105 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a | `NO_RESIDUAL` |
| 2 `upper_arm_to_forearm` | 103 | 0.10000000000000001 | 0.1152404435585387 | 0.015240443558538694 | 0.1182404435585387 | 0.12967944900221676 | 0.13128643739932974 | 0.13216047003626613 | 0.031286437399329736 | 0.032160470036266126 | 0.13252507336605851 | `EXCESSIVE_OR_AMBIGUOUS_EXPANSION` |
| 3 `forearm_to_wrist_1` | 72 | 0.089999999999999997 | 0.097982674482047408 | 0.007982674482047411 | 0.10098267448204741 | 0.11026314871239322 | 0.11342283441561084 | 0.1145264918126311 | 0.023422834415610841 | 0.024526491812631099 | 0.11452649214143613 | `LOCAL_COVERAGE_CANDIDATE` |
| 4 `wrist_1_to_wrist_2` | 105 | 0.10000000000000001 | 0.10817308749760933 | 0.0081730874976093243 | 0.11117308749760933 | 0.11998168904405469 | 0.12218581675213991 | 0.12375411521146593 | 0.022185816752139903 | 0.02375411521146592 | 0.12375411521146593 | `LOCAL_COVERAGE_CANDIDATE` |
| 5 `wrist_2_to_tool` | 85 | 0.105 | 0.11482837170678802 | 0.0098283717067880194 | 0.11782837170678802 | 0.1254721879602462 | 0.12591543346805348 | 0.12789819607381778 | 0.020915433468053482 | 0.022898196073817781 | 0.12898382195908234 | `LOCAL_COVERAGE_CANDIDATE` |

## Propuesta numérica sometida a revisión, no aplicada

Sólo las cápsulas con `LOCAL_COVERAGE_CANDIDATE` tienen una propuesta
numérica de cobertura local, basada en su `required_radius_m` P95 de esta
única sesión inmóvil:

| Cápsula | Radio coverage_v2 propuesto m | Base de la propuesta |
|---|---:|---|
| 3 `forearm_to_wrist_1` | 0.11342283441561084 | required P95; 72 puntos; incremento 0.023422834415610841 m |
| 4 `wrist_1_to_wrist_2` | 0.12218581675213991 | required P95; 105 puntos; incremento 0.022185816752139903 m |
| 5 `wrist_2_to_tool` | 0.12591543346805348 | required P95; 85 puntos; incremento 0.020915433468053482 m |

No se deriva ni se activa automáticamente un perfil. La cápsula 2 queda
bloqueada por expansión excesiva/ambigua: su incremento P95 es
`0.031286437399329736 m`, superior a `0.025 m`. Las cápsulas 0 y 1 no tienen
residuales asociados y no reciben propuesta.

## Validación offline

Se ejecutaron exclusivamente:

```text
colcon build --packages-select thesis_jaco_perception --symlink-install
colcon test --packages-select thesis_jaco_perception --ctest-args -R test_self_residual_metrics
git diff --check
```

Resultados:

- Build: `1 package finished`.
- `test_self_residual_metrics`: `9` pruebas, `0` fallos y `0` errores.
  Cubren CSV válido, cantidad incorrecta, radio no positivo/no finito, radio
  requerido conocido, percentiles, candidato local y expansión excesiva o
  ambigua.
- `git diff --check`: sin salida ni errores.

No se ejecutó una segunda medición ni otra prueba física.
