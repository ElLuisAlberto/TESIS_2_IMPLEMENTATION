# Fase 7B: sesión nativa de sombra epsilon 0.07 m

## Estado

`EPSILON_007_VIEW_READY`

## Terminales nativas abiertas

Se verificó `gnome-terminal` en `/usr/bin/gnome-terminal` y que no había una
sesión previa con RealSense, Kinova, self-filter o RViz de esta sesión. Se
abrieron y permanecen abiertas dos terminales nativas independientes:

1. `TESIS_RGBD_SOMBRA_EPSILON_007`: ROS Humble, `install/setup.bash`,
   `ROS_DOMAIN_ID=78` y el launch de sombra.
2. `TESIS_RVIZ_SOMBRA_EPSILON_007`: RViz con
   `src/thesis_jaco_perception/rviz/jaco_d435i_alignment.rviz`.

El launch usa el mismo YAML temporal v2 existente,
`evidence/codex_reports/2026-10-21_d435i_extrinsic_visual_hypothesis_v2.yaml`,
con `start_jaco_read_only:=true`, `start_camera:=true`,
`start_rviz:=false`, `enable_self_filter:=true`,
`publish_session_extrinsic:=true`, `session_epsilon_total_m:=0.07` y
`hardware_output:=false`.

## Verificación de observación

- JACO2: `/thesis/hardware/connected` publicó `true` y
  `/thesis/hardware/armed` publicó `false`; el adaptador está solo en lectura.
- D435i: el nodo `/camera/d435i` está presente.
- TF v2 sin cambios: `tf2_echo world d435i_link` confirmó traslación
  `(0.030, -0.680, 0.300)` m y RPY `(0.000, -10.000, 90.000)` grados.
- Nube filtrada: `/thesis/perception/points_filtered` entregó `frame_id=world`
  y `2115` puntos.
- RViz: el nodo `/rviz` está suscrito a la nube filtrada con `BEST_EFFORT` y
  `VOLATILE`, exactamente compatibles con el publicador; no se detectó
  incompatibilidad QoS.

| Salida del self-filter | Color RViz | Puntos |
|---|---|---:|
| `/thesis/jaco_perception/points_self_confirmed` | verde | 791 |
| `/thesis/jaco_perception/points_external_candidate` | rojo | 971 |
| `/thesis/jaco_perception/points_occluded_unknown` | amarillo | 382 |

La lista de nodos contiene únicamente cámara, TF temporal, adaptador JACO2 de
solo lectura, health, preprocesador, self-filter, `robot_state_publisher` y
RViz (más listeners TF). No aparecen supervisor, obstacle bridge, extractor ni
nodos de mando.

## Seguridad y alcance

No se modificó código, YAML, TF, cápsulas, launch, CMake ni el perfil RViz. No
se inició GUI física, Gazebo ni adaptadores de comando y no se usó multi-pose.
No se enviaron comandos, trayectorias, jog ni servicios de movimiento al
JACO2. Ambas terminales se dejan abiertas para la captura del operador; no se
envió SIGINT.
