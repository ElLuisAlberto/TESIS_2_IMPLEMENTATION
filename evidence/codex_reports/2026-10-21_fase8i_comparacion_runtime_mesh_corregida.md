# 2026-10-21 — Fase 8I: comparación runtime mesh-shadow corregida

## Dictamen

`BLOCKED_RVIZ_RUNTIME`

La única sesión autorizada llegó a iniciar el nodo de comparación por malla y el
filtro de cápsulas con `hardware_output:=false`, pero RViz no se suscribió a
ninguna de las seis salidas de comparación. Por ello no fue posible verificar
visualmente la superposición, conteos, sellos ni la separación frente a una caja.
No se modificó ninguna configuración ni se reintentó la sesión.

## Precondiciones y configuración usada

- Antes de iniciar, no se observó una sesión `coverage_v1` ni RViz activos.
- Se usó `ROS_DOMAIN_ID=78` y el entorno instalado del espacio de trabajo.
- Se lanzó una sola sesión `jaco_d435i_shadow_session.launch.py` con:
  `start_jaco_read_only:=true`, `start_camera:=true`, `start_rviz:=false`,
  `enable_self_filter:=true`, `enable_mesh_shadow_comparison:=true`,
  `publish_session_extrinsic:=true`, `hardware_output:=false`, el perfil de
  cobertura v1 y la extrínseca v2 solicitados.
- No se proporcionó un XML manual como `robot_description`; el nodo usa la
  descripción canónica tipada de la corrección 8H.

## Evidencia de arranque

El registro de lanzamiento confirmó el inicio de ambos procesos:

- `jaco_mesh_shadow_node-8 process started with pid [108154]`.
- `jaco_self_filter_node-9 process started with pid [108156]`.

También se observó la D435i activa, `hardware_readiness=READY` y el adaptador
en `salida_fisica_permitida=false, armado=false`. `robot_state_publisher`
publicó los segmentos esperados de base, enlaces 1--6, mano y dedos. El grafo
incluyó cámara, transformada de sesión, preprocesador, ambos filtros y los
componentes de preparación/puente sin habilitar salida física.

Se verificó que existían las seis rutas de salida:

- Cápsulas: `points_self_confirmed`, `points_external_candidate` y
  `points_occluded_unknown`.
- Malla: `mesh_shadow/points_self_confirmed`,
  `mesh_shadow/points_external_candidate` y
  `mesh_shadow/points_occluded_unknown`.

También existía `mesh_shadow/diagnostics`.

## Bloqueo de RViz

Se abrió el perfil de comparación RViz existente sin modificarlo. El registro
confirmó contexto OpenGL 4.6 y no registró un error de proceso. Sin embargo,
para cada una de las seis rutas anteriores, la consulta de tipo mostró
literalmente `Publisher count: 1` y `Subscription count: 0`.

En consecuencia, RViz no estaba recibiendo ninguna nube de cápsulas ni de
malla. No hay evidencia válida de visualización, conteos por frame, sellos,
latencia ni de la separación de una caja a 0.8 m. El alcance de esta fase no
autoriza modificar el perfil, QoS o relanzar para corregirlo.

## Cierre seguro

La sesión se cerró con `SIGINT` al grupo de lanzamiento `108116` y RViz con
`SIGINT` al grupo `108369`. Tras esperar diez segundos, no quedó ningún proceso
de esos grupos. No se enviaron trayectorias, jogs, acciones, servicios ni
comandos al JACO2; no hubo movimiento físico, y `hardware_output` se mantuvo
en falso.

No se editaron código, launch, perfiles, mallas, TF, filtros, parámetros ni
archivos de configuración durante esta fase. `git diff --check` finalizó sin
salida (correcto).

## Próximo paso necesario

Corregir estáticamente, en una fase separada, el perfil/configuración de RViz
o su compatibilidad QoS para que se suscriba a las seis salidas, y sólo después
solicitar una nueva autorización para una única sesión comparativa.
