# Fase 6W: sesión única de alineación visual temporal

## Estado final

`BLOCKED_RUNTIME`

La única sesión fue detenida sin reintento. RViz informó incompatibilidad de QoS de fiabilidad para los cuatro `PointCloud2`, por lo que no recibiría las nubes necesarias para la observación visual.

```
New publisher discovered on topic '/thesis/perception/points_filtered', offering incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```

El mismo diagnóstico apareció para `/thesis/jaco_perception/points_self_confirmed`, `/thesis/jaco_perception/points_external_candidate` y `/thesis/jaco_perception/points_occluded_unknown`.

## Launch ejecutado

`ROS_DOMAIN_ID=78 ros2 launch thesis_jaco_perception jaco_d435i_shadow_session.launch.py start_jaco_read_only:=true start_camera:=true start_rviz:=false enable_self_filter:=true publish_session_extrinsic:=true candidate_extrinsic_file:="$PWD/evidence/codex_reports/2026-10-21_d435i_extrinsic_session_candidate.yaml" session_epsilon_total_m:=0.05 hardware_output:=false`

El YAML temporal usado declara exclusivamente `world -> d435i_link` y conserva `d435i_depth_optical_frame` como frame óptico separado. El publicador temporal confirmó `world` a `d435i_link` con traslación `(0.68, -0.03, 0.30)` y cuaternión `(0, -0.0871557427, 0, 0.9961946981)`.

## Estado observado antes del bloqueo

- JACO2: adaptador USB Kinova de solo lectura con `salida_fisica_permitida=false, armado=false`; `hardware_readiness` informó `READY`.
- D435i: RealSense D435i serie `405622074532` inició profundidad y color a `640x480x30`.
- TF y preprocesado: inició el publicador temporal y el preprocesador configuró entrada `/camera/d435i/depth/color/points`, salida `/thesis/perception/points_filtered` y frame objetivo `world`.
- La cadena TF completa y una muestra reciente de nube no se verificaron con clientes adicionales porque el bloqueo de RViz obligó a cerrar la sesión sin reintento.

## Perfil RViz

Se creó `src/thesis_jaco_perception/rviz/jaco_d435i_alignment.rviz` con frame fijo `world`, `RobotModel` desde `/robot_description`, TF y los cuatro displays de nube solicitados: filtrada blanca, self-confirmed verde, external candidate rojo y occluded-unknown amarillo. No contiene displays de supervisor, horizonte ni obstáculos.

## Seguridad y cierre

`hardware_output=false` fue pasado explícitamente y validado por ambos launches. No se iniciaron supervisor, obstacle bridge, extractor, GUI física, adaptadores de comando, Gazebo ni telemetría de mando. No se usó el módulo experimental multi-pose, no se escribió YAML de extrínseca final y no se envió ningún comando físico.

Tras el aviso QoS, se enviaron SIGINT únicamente a RViz y al launch propio; todos sus procesos finalizaron limpiamente, incluida la D435i y el adaptador JACO2 de solo lectura. No queda una sesión activa.
