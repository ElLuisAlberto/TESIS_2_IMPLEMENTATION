# Prueba funcional GUI cartesiana — Gazebo

Fecha: 2026-10-18  
Conclusión: **BLOQUEADO**

## Alcance

Se reutilizó exclusivamente la configuración de simulación previamente
validada. No se modificó código, parámetros ni configuración; no se inició
hardware físico, cámara, RViz ni cliente gráfico de Gazebo, y no se usó
`sudo`.

La interfaz iba a accionarse mediante su propia clase `JointControlWindow` con
Qt offscreen, para ejercer copiar/ajustar/enviar de la GUI sin usar
`ros2 topic pub`. No se llegó a crear ni enviar una intención porque una
precondición preventiva pasó a STOP antes del primer eje.

## Comandos exactos ejecutados

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
ros2 launch thesis_simulation thesis_system.launch.py \
  initial_pose:=z_ready use_demo_obstacle:=true obstacle_x:=2.0 \
  simulation_output_enabled:=true start_gui:=false start_rviz:=false \
  headless:=true
```

```bash
ros2 topic echo --no-daemon --spin-time 2 \
  /thesis/system_readiness std_msgs/msg/String --once
```

Resultado:

```yaml
data: READY
```

```bash
ros2 topic echo --no-daemon --spin-time 2 \
  /joint_states sensor_msgs/msg/JointState --once
```

```bash
timeout 5s ros2 run tf2_ros tf2_echo world j2n6s300_end_effector
```

```bash
ros2 topic echo --no-daemon --spin-time 2 \
  /thesis/proximity_status thesis_interfaces/msg/ProximityStatus --once
```

Tras detectar STOP, se envió `SIGINT` exclusivamente al proceso de launch y
se comprobó que no quedaran sus procesos hijos.

## Estado inicial registrado

`/joint_states` inicial (rad):

| Joint | Posición |
| --- | ---: |
| j2n6s300_joint_1 | -0.000000778 |
| j2n6s300_joint_2 | 2.617996211 |
| j2n6s300_joint_3 | 3.665248235 |
| j2n6s300_joint_4 | -0.000056997 |
| j2n6s300_joint_5 | 0.349167682 |
| j2n6s300_joint_6 | 0.000009141 |

TF inicial `world -> j2n6s300_end_effector`:

```text
Translation: [0.596, 0.070, 0.926] m
RPY: [-0.737, -0.135, -1.680] rad
```

La primera línea de `tf2_echo` indicó transitoriamente que `world` aún no
existía durante el descubrimiento; las cuatro muestras posteriores fueron
estables con la TF anterior.

## Condición de detención

Aunque `/thesis/system_readiness` publicó `READY`, el estado preventivo
observado fue:

```yaml
reference_frame: world
state: STOP
minimum_clearance: 0.0
limiting_segment: TF_UNAVAILABLE
obstacle_center: [2.0, 0.0, 0.65]
obstacle_radius: 0.14
```

Esto constituye exactamente una condición de detención solicitada: error/TF
indisponible y STOP upstream. No se accionó X, Y ni Z, no se publicó
`/thesis/cartesian_candidate` ni `/thesis/cartesian_jog_intent`, y no hubo
salida supervisada de movimiento.

## Resultado por eje y estado final

| Eje | Solicitud | Tópico de comando | Resultado | Coincidencia de signo |
| --- | --- | --- | --- | --- |
| X | No enviada | Ninguno | No intentado: STOP `TF_UNAVAILABLE` previo | No aplicable |
| Y | No enviada | Ninguno | No intentado: detención tras STOP | No aplicable |
| Z | No enviada | Ninguno | No intentado: detención tras STOP | No aplicable |

No se tomó TF/joints final: la regla exigía detener inmediatamente al detectar
STOP y la simulación se apagó sin emitir comando. Por tanto no existe un estado
final de movimiento que comparar; el único estado observable es la postura
inicial estacionaria anterior a cualquier intención.

## Cierre

Resultado **BLOQUEADO**, no PASS ni FAIL de X/Y/Z: el interbloqueo preventivo
actuó antes de permitir una prueba funcional. La causa a resolver antes de
reintentar es la disponibilidad temporal de TF en `proximity_monitor`
(`TF_UNAVAILABLE`), sin relajar STOP, HOLD ni parámetros de seguridad.

