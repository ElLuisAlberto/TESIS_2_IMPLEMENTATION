# 2026-10-21 — Fase 8L: verificación pasiva de nube sin brazo mesh-shadow

## Dictamen

`BLOCKED_RVIZ_VIEW_CONFIGURATION`

La sesión mesh-shadow heredada de Fase 8K permanece activa, el filtro mesh publica su nube externa y el núcleo está sano. Sin embargo, RViz dejó de estar activo durante esta fase: las seis rutas que antes tenían un suscriptor RViz pasaron a `Subscription count: 0`. No se reinició RViz, el launch ni ningún nodo, por la restricción expresa de no reiniciar ni crear otra sesión. Por ello no pueden configurarse las vistas temporales A/B ni realizarse la evaluación visual de retirada del brazo.

## Procesos y seguridad observados

La sesión propia sigue siendo el grupo `110022`, iniciado con `hardware_output:=false`, `start_jaco_read_only:=true`, D435i y mesh-shadow habilitados. Permanecen activos:

- `/jaco_self_filter_node` y `/jaco_mesh_shadow_node`.
- `/camera/d435i` y `/pointcloud_preprocessor`.
- `/d435i_session_candidate_extrinsic`, publicando `world -> d435i_link`.
- `/jaco_hardware_adapter`, `/hardware_readiness` y `robot_state_publisher`.

El registro de la sesión conserva el estado del adaptador: `salida_fisica_permitida=false, armado=false`; el estado de readiness fue `READY`. El launch explícito es sólo lectura. No hubo comandos, trayectorias, jogs, acciones ni servicios dirigidos al JACO2.

## Diagnóstico mesh literal

```text
name: JACO2 mesh shadow
message: MESH_SHADOW_COMPLETE
session_only: true
not_for_safety: true
geometry_kind: collision_mesh
```

La cabecera del diagnóstico estaba en `world`. No se observaron fallos de URDF, TF, cámara, carga de mallas ni publicación.

## Interfaces y aislamiento

`/jaco_mesh_shadow_node` sólo recibe `/thesis/perception/points_filtered` y publica diagnóstico más las tres salidas `mesh_shadow`. No tiene clientes, servidores ni acciones de control. El filtro de cápsulas tiene la misma entrada de nube y sus tres salidas propias. No hay ruta runtime observada hacia supervisor, obstacle bridge, extractor, `obstacle_input` ni control.

| Ruta | Publicadores | Suscriptores RViz actuales |
|---|---:|---:|
| `/thesis/jaco_perception/points_self_confirmed` | 1 | 0 |
| `/thesis/jaco_perception/points_external_candidate` | 1 | 0 |
| `/thesis/jaco_perception/points_occluded_unknown` | 1 | 0 |
| `/thesis/jaco_perception/mesh_shadow/points_self_confirmed` | 1 | 0 |
| `/thesis/jaco_perception/mesh_shadow/points_external_candidate` | 1 | 0 |
| `/thesis/jaco_perception/mesh_shadow/points_occluded_unknown` | 1 | 0 |

La causa inmediata de los ceros es verificable: `ps` no encontró ningún proceso `rviz2`; el único registro de RViz contiene sólo el arranque OpenGL 4.6 y no un error literal. No se cambió temporalmente ningún display porque no existe una instancia RViz sobre la que hacerlo, y volver a abrirla sería un reinicio prohibido en esta fase.

## Muestras pasivas de mesh-shadow

Las muestras son observaciones independientes tomadas secuencialmente. La edad es una cota superior calculada entre la recepción de la muestra y el reloj de pared inmediatamente posterior.

| Salida mesh | Frame | Sello | Puntos (`width`) | Edad disponible |
|---|---|---|---:|---:|
| self confirmed | `world` | `1792635996.058158447` | 162 | <= 0.217628 s |
| external candidate | `world` | `1792635998.526569580` | 1091 | <= 0.218997 s |
| occluded unknown | `world` | `1792636001.028467285` | 12 | <= 0.211756 s |

`NO_SAME_SAMPLE_COMPARISON`: los sellos de las tres muestras mesh son distintos y tampoco se obtuvo un par de cápsulas con sello idéntico. Por tanto no se infiere mejora cuantitativa ni se comparan los conteos. La publicación de `mesh_shadow/points_external_candidate` está demostrada, pero todavía no puede validarse visualmente que represente una nube sin brazo.

## Vistas y capturas

No fue posible crear la Vista A (auditoría) ni la Vista B (sólo mesh external) porque RViz no está activo. No se sobrescribió ni guardó ningún perfil. Existe la utilidad de captura `import`, pero no hay una ventana RViz con las vistas requeridas; por tanto no se generaron:

```text
CAPTURE_UNAVAILABLE
evidence/codex_reports/2026-10-21_fase8l_auditoria_mesh.png   (no creada)
evidence/codex_reports/2026-10-21_fase8l_nube_sin_brazo.png  (no creada)
```

## Integridad de la fase

No hubo cambios persistentes, reinicios, recalibración, ajuste de geometría, caja u objeto de prueba, ni conexión a seguridad/control. La sesión continúa abierta exactamente como se recibió y no se cerró ningún proceso.
