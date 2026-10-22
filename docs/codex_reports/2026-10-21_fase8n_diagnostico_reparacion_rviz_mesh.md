# 2026-10-21 — Fase 8N: diagnóstico y reparación RViz mesh sin brazo

## Dictamen

`MESH_ARM_REMOVAL_RVIZ_READY`

## Diagnóstico inicial

La sesión mesh-shadow existente permaneció activa. Los intentos anteriores de
RViz fueron:

| Fase | Comando | Resultado observable |
|---|---|---|
| 8K | `setsid bash -lc '... rviz2 -d ...jaco_d435i_mesh_shadow_comparison.rviz'` | log OpenGL 4.6, proceso no persistente |
| 8M | `setsid bash -lc '... exec rviz2 -d ...jaco_d435i_mesh_arm_removed.rviz'` | proceso no persistente; log vacío |

El ejecutable resuelto inicialmente fue `/opt/ros/humble/bin/rviz2`.
El entorno de la terminal fue `DISPLAY=:1`, `WAYLAND_DISPLAY=` vacío y
`XDG_SESSION_TYPE=x11`; `SNAP`, `SNAP_NAME` y `LD_PRELOAD` estaban vacíos.
`LD_LIBRARY_PATH` contenía únicamente bibliotecas del espacio de trabajo y ROS
Humble. No se afirmó una causa no demostrada: los intentos anteriores no
produjeron error literal. La evidencia sí demuestra que la invocación explícita
del binario RViz Humble persiste, mientras las dos invocaciones anteriores no.

Antes de la reparación, la ruta mesh external tenía tipo
`sensor_msgs/msg/PointCloud2`, un publicador y cero suscriptores. El publicador
`jaco_mesh_shadow_node` ofrecía `BEST_EFFORT`, `VOLATILE` y profundidad
reportada como `UNKNOWN` por la introspección DDS. El perfil contiene ya
`Topic.Value` correcto con Best Effort, Volatile, Keep Last y profundidad 5,
por lo que no requirió modificación durante 8N.

## Estado mesh

`/jaco_mesh_shadow_node` sigue suscrito sólo a
`/thesis/perception/points_filtered` y publica exclusivamente diagnóstico y
las tres salidas mesh-shadow; no tiene acciones, clientes ni servidores de
control. El diagnóstico literal fue:

```text
message: MESH_SHADOW_COMPLETE
session_only: true
not_for_safety: true
geometry_kind: collision_mesh
```

## Intento RViz de Fase 8N

Se usó un solo proceso nuevo (segundo intento no necesario):

```text
setsid bash -lc '... /opt/ros/humble/lib/rviz2/rviz2 -d \
  /home/luis/Escritorio/TESIS_2_IMPLEMENTATION/src/thesis_jaco_perception/rviz/jaco_d435i_mesh_arm_removed.rviz; ...'
```

PID del grupo: `112295`; PID RViz activo: `112320`. El proceso continúa en
ejecución y su salida literal es:

```text
[INFO] [rviz2]: Stereo is NOT SUPPORTED
[INFO] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[INFO] [rviz2]: Stereo is NOT SUPPORTED
```

No se produjo error fatal. Después del arranque, la ruta external muestra:

```text
Publisher count: 1
Subscription count: 1
Publisher QoS: BEST_EFFORT, VOLATILE
RViz subscription QoS: BEST_EFFORT, VOLATILE
```

La compatibilidad QoS está demostrada por la suscripción activa. La captura
confirma visualmente el RobotModel gris y sólo el display magenta
`Mesh External Candidate — Arm Removed`; no aparecen filtered, cápsulas,
mesh self ni mesh occluded. La captura también muestra puntos magenta sobre
partes visibles del brazo. Eso es evidencia de residuales de clasificación,
no una razón para modificar la geometría en esta fase; esta fase sólo restauró
la visualización y no declara retirada completa del brazo.

## Captura y aislamiento

Captura creada: [2026-10-21_fase8n_mesh_external_sin_brazo.png](../../evidence/codex_reports/2026-10-21_fase8n_mesh_external_sin_brazo.png).

La sesión ROS, D435i, JACO2, TF temporal, preprocesador y filtros no se
reiniciaron. `hardware_output=false` permanece en el comando de la sesión;
el adaptador sigue desarmado/solo lectura. No hubo movimiento, comandos,
trayectorias, jogs, acciones ni servicios al JACO2, ni conexión a
`obstacle_input`, supervisor o control. RViz y la sesión quedan abiertos.
