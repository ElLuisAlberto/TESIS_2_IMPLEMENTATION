# 2026-10-21 — Fase 8J: corrección estática del perfil RViz mesh-shadow

## Dictamen

`MESH_SHADOW_RVIZ_PROFILE_READY`

## Causa raíz demostrada

El perfil anterior contenía los seis bloques `PointCloud2`, pero no estaba
serializado con la estructura de propiedades que RViz Humble emplea para un
tópico ROS. En concreto, usaba `Topic: /ruta` como escalar y dejaba
`Reliability Policy`, `Durability Policy`, `History Policy` y `Depth` al nivel
del display; además, no declaraba `Enabled: true` en ninguno de los seis
displays. El perfil funcional
`src/thesis_jaco_perception/rviz/jaco_d435i_alignment.rviz` demuestra la
sintaxis compatible: `Topic` es un mapa con `Value` y las cuatro propiedades
QoS. Esto explica la evidencia 8I de seis publicadores y cero suscriptores de
RViz, sin atribuir el fallo a cámara, TF, mallas, cápsulas ni QoS del emisor.

## Cambio acotado

Sólo se modificó el perfil autorizado
`src/thesis_jaco_perception/rviz/jaco_d435i_mesh_shadow_comparison.rviz` y se
extendió el contrato estático autorizado
`src/thesis_jaco_perception/test/test_shadow_session_contract.py`. Este informe
es el único otro archivo creado. No se modificó
`jaco_d435i_alignment.rviz`: el contrato fija su SHA-256 original
`ca236c66efcb20632b0c4312a7f9ace73bb66a726b2b661e754822f7fc2a3c5c`.

Antes, las seis entradas tenían `Topic` escalar, QoS en el nivel equivocado y
sin `Enabled`. Después, el perfil contiene `Fixed Frame: world`, `RobotModel`,
`TF`, y seis entradas `rviz_default_plugins/PointCloud2` con `Enabled: true` y
un bloque `Topic` de RViz Humble: Best Effort, Volatile, Keep Last y profundidad
5.

| Display | Tipo | Tópico | Enabled | QoS | Color |
|---|---|---|---|---|---|
| Capsules Self Confirmed | PointCloud2 | `/thesis/jaco_perception/points_self_confirmed` | true | Best Effort, Volatile, Keep Last, 5 | verde |
| Capsules External Candidate | PointCloud2 | `/thesis/jaco_perception/points_external_candidate` | true | Best Effort, Volatile, Keep Last, 5 | rojo |
| Capsules Occluded Unknown | PointCloud2 | `/thesis/jaco_perception/points_occluded_unknown` | true | Best Effort, Volatile, Keep Last, 5 | amarillo |
| Mesh Self Confirmed | PointCloud2 | `/thesis/jaco_perception/mesh_shadow/points_self_confirmed` | true | Best Effort, Volatile, Keep Last, 5 | cian |
| Mesh External Candidate | PointCloud2 | `/thesis/jaco_perception/mesh_shadow/points_external_candidate` | true | Best Effort, Volatile, Keep Last, 5 | magenta |
| Mesh Occluded Unknown | PointCloud2 | `/thesis/jaco_perception/mesh_shadow/points_occluded_unknown` | true | Best Effort, Volatile, Keep Last, 5 | naranja |

No se incluyeron displays de supervisor, horizonte, `obstacle_input`,
extractor, comandos ni control.

## Contrato estático

Se añadió la comprobación del YAML de RViz que exige:

- `Fixed Frame` igual a `world`, más `RobotModel` y `TF`.
- Exactamente seis displays `PointCloud2`; cada nombre y tópico aparece una
  vez, está habilitado y tiene el color requerido.
- El bloque `Topic` de cada display es exactamente la forma RViz Humble con
  `Value`, Best Effort, Volatile, Keep Last y profundidad 5.
- Ausencia de rutas y nombres prohibidos de seguridad/control.
- Integridad del perfil original `jaco_d435i_alignment.rviz` mediante SHA-256.

## Validación ejecutada

Se ejecutó únicamente lo autorizado:

```text
colcon build --packages-select thesis_jaco_perception --symlink-install
Starting >>> thesis_jaco_perception
Finished <<< thesis_jaco_perception [0.22s]
Summary: 1 package finished [0.48s]

colcon test --packages-select thesis_jaco_perception --ctest-args -R test_shadow_session_contract
Starting >>> thesis_jaco_perception
Finished <<< thesis_jaco_perception [0.67s]
Summary: 1 package finished [0.94s]

colcon test-result --test-result-base build/thesis_jaco_perception --verbose
Summary: 106 tests, 0 errors, 0 failures, 0 skipped

git diff --check
# sin salida; correcto
```

## Aislamiento

No se inició ROS graph, launch, D435i, JACO2, RViz, TF, Gazebo ni sonda. No se
cerraron procesos y no se enviaron comandos, trayectorias, jogs, acciones ni
servicios al JACO2. No hubo sesión runtime. No se cambiaron nodos, launch,
CMake, `package.xml`, YAML operativo, ROI, epsilon, TF, extrínseca, cápsulas,
mallas ni perfiles coverage; sólo el perfil RViz, su prueba y este informe.
