# Evidencia: `z_ready`, obstáculo seguro y objetivo Z +20 mm

Fecha: 2026-10-18  
Entorno: Gazebo headless, simulación únicamente

## Alcance y arranque

Se inició una única pila nueva con:

```text
ros2 launch thesis_simulation thesis_system.launch.py \
  initial_pose:=z_ready \
  use_demo_obstacle:=true \
  obstacle_x:=2.0 \
  simulation_output_enabled:=true \
  start_gui:=false start_rviz:=false headless:=true
```

Directorio de logs ROS/Gazebo:

```text
/tmp/tesis_z_ready_motion_logs/
```

Log principal:

```text
/tmp/tesis_z_ready_motion_logs/2026-10-18-13-11-53-687993-miNave-76063/launch.log
```

No se modificó código. No se usó hardware físico. Se publicó exactamente un
objetivo cartesiano.

## Precondición READY

El monitor publicó READY aproximadamente 9.08 s después del inicio del launch:

```text
[system_readiness]: SYSTEM READY: all required links are live
```

La comprobación tipada, con plazo global de 20 s, devolvió:

```text
ATTEMPT=1 RC=0 ELAPSED=4s
data: READY
---
READY_VERIFIED=true TOTAL_ELAPSED=4s
```

Una primera invocación del cliente dentro del aislamiento no pudo crear el
transporte DDS (`Operation not permitted`). Se repitió fuera del aislamiento;
la pila ya había registrado READY y la comprobación tipada fue satisfactoria.

## Precondición de proximidad

Consulta separada de
`/thesis/proximity_status thesis_interfaces/msg/ProximityStatus`:

```yaml
reference_frame: world
state: ALLOW
minimum_clearance: 1.2023092669443871
limiting_segment: wrist_2_to_tool
closest_robot_point:
  x: 0.5962133173283253
  y: 0.06955677642211003
  z: 0.9257800583475664
obstacle_center:
  x: 2.0
  y: 0.0
  z: 0.65
obstacle_radius: 0.13999999999999999
```

Resultado de la precondición: `ALLOW` y `1.202309 m > 0.30 m`.

## TF inicial

`tf2_echo world j2n6s300_end_effector`, ejecutado como cliente independiente:

```text
Translation: [0.596, 0.070, 0.926]
Quaternion (xyzw): [-0.287, 0.225, -0.709, 0.603]
RPY (radian): [-0.737, -0.135, -1.680]
```

## Único objetivo publicado

Identificador:

```text
codex-z-ready-20261018-001
```

Contenido:

```yaml
frame_id: world
x: 0.596213367954937
y: 0.069556678319688
z: 0.945785070922883
roll: -0.736707540812486
pitch: -0.135455987635350
yaw: -1.680133902932973
duration:
  sec: 5
  nanosec: 0
```

El objetivo conserva X, Y y orientación de `z_ready`, y suma 0.020 m a Z.

## IK, supervisor y ejecución

Adaptador cartesiano:

```text
CARTESIAN CONVERTED codex-z-ready-20261018-001: iterations=3,
position_error=0.0001 m, orientation_error=0.09 deg,
from_tf=(0.596, 0.070, 0.926), reference_offset=0.0000 m
```

Supervisor:

```text
PREDICTION codex-z-ready-20261018-001: ALLOW,
min_clearance=1.202 m, path=0.00, segment=wrist_2_to_tool

ALLOW FORWARDED id=codex-z-ready-20261018-001 | joints=6 |
proximity=ALLOW | duration=5.00 s | joint_lim=j2n6s300_joint_3 |
v_req=-0.0485 rad/s | v_lim=-0.0485 rad/s

RUNTIME codex-z-ready-20261018-001: ALLOW, d_min=1.202 m,
scale=1.00, t_min=0.000 s, TTC_nom=-1.000 s
```

Ejecución:

```text
GAZEBO ACCEPTED id=codex-z-ready-20261018-001, scale=1.00
GAZEBO SUCCEEDED id=codex-z-ready-20261018-001: Goal successfully reached!
```

## TF final

Lectura independiente posterior al estado terminal:

```text
Translation: [0.596, 0.070, 0.946]
Quaternion (xyzw): [-0.286, 0.226, -0.710, 0.603]
RPY (radian): [-0.737, -0.134, -1.681]
```

A la resolución de `tf2_echo`, X/Y se conservaron y Z pasó de 0.926 m a
0.946 m, es decir, +20 mm. La orientación se mantuvo dentro de aproximadamente
0.001 rad por componente RPY.

## Incidencias de validación y cierre

El proceso `cartesian_sim_validator`, iniciado antes de publicar, no recibió
los cuatro tópicos correlacionados y agotó sus 30 s con:

```text
COMANDO=AUSENTE
CHECK_INTENCION_CARTESIANA=FALLO
CHECK_CONVERSION_IK=FALLO
CHECK_SUPERVISION=FALLO
CHECK_EJECUCION=FALLO
RESULTADO=REVISAR
```

Este fallo del observador contradice los registros directos y correlacionados
del adaptador, supervisor y ejecutor para el mismo `command_id`; por ello se
conserva como incidencia y no se repitió el objetivo.

Tras la lectura TF final se detuvo la pila con SIGINT. Gazebo, los controladores,
`system_readiness`, el supervisor y el adaptador de ejecución cerraron. Durante
el cierre, cinco nodos Python registraron
`RuntimeError: Unable to convert call argument to Python object`; ocurrió
después de `SUCCEEDED` y no se ejecutó ninguna acción adicional.

