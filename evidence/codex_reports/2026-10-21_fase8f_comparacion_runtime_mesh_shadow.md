# Fase 8F: comparación runtime cápsulas vs mesh-shadow

## Dictamen

`BLOCKED_RUNTIME`

## Precondiciones y cierre de v1

El operador confirmó autorización, brazo inmóvil, área despejada y parada de
emergencia accesible. La sesión `coverage_v1` fue identificada antes de cerrar:
launch PID 85128, grupo 85104, con `hardware_output:=false`, extrínseca visual
v2 y perfil coverage_v1. Sus hijos eran exclusivamente TF temporal,
robot_state_publisher, D435i y `jaco_self_filter_node`; no había RViz propio.
Se envió SIGINT sólo al launch 85128. Tras espera, todos los PID 85128, 85131,
85133, 85139 y 85145 terminaron limpiamente.

## Preparación

ROS Humble e `install/setup.bash`, `ROS_DOMAIN_ID=78`. Existían las rutas de
extrínseca v2, coverage_v1 y perfil RViz original. `--show-args` confirmó
`enable_mesh_shadow_comparison` con default `false` y `robot_description`.
Se creó el único perfil nuevo permitido:
`src/thesis_jaco_perception/rviz/jaco_d435i_mesh_shadow_comparison.rviz`.
No se sobrescribió el perfil previo.

## Sesión única y error literal

Se inició exactamente una sesión nueva con los argumentos solicitados:
`start_jaco_read_only:=true start_camera:=true start_rviz:=false
enable_self_filter:=true enable_mesh_shadow_comparison:=true
publish_session_extrinsic:=true hardware_output:=false enable_crop:=true`,
los límites ROI solicitados, la extrínseca v2, coverage_v1 y el URDF expandido
temporal pasado como `robot_description`.

El launch falló antes de crear `jaco_self_filter_node` y
`jaco_mesh_shadow_node`. Error literal:

```text
Unable to parse the value of parameter robot_description as yaml. If the parameter is meant to be a string, try wrapping it in launch_ros.parameter_descriptions.ParameterValue(value, value_type=str)
```

Por la regla de una sola sesión, no hubo reintento. No fue posible observar
conteos, sellos, seis tópicos, QoS ni comparación visual; no se afirma ninguna
mejora de malla.

## Cierre de la sesión fallida y aislamiento

El launch fallido dejó sólo procesos propios en PGID 106195: TF temporal,
robot_state_publisher, adaptador JACO2 de lectura, `hardware_readiness`, D435i,
preprocesador y health. Se envió SIGINT exclusivamente a ese grupo; tras diez
segundos no permanecía ningún proceso con PGID 106195. No se usó SIGKILL.

No hubo comandos, trayectorias, jog, acciones, servicios, movimiento, caja u
obstáculo. `hardware_output=false` fue parte del comando efectivo. No se inició
RViz ni se conectó tópico alguno a supervisor, bridge, extractor,
`obstacle_input` o control.

`git diff --check` terminó limpio. `git status --short` mostró los cambios y
archivos no rastreados existentes, además del perfil RViz y este informe; no se
modificó código, YAML, launch, TF, epsilon, cápsulas, coverage_v1/v2 ni mallas
durante esta fase.
