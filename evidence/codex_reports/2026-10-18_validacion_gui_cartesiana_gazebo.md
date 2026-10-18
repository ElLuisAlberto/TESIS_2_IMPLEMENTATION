# Validación funcional GUI cartesiana en Gazebo tras A-02a

Fecha: 2026-10-18  
Clasificación: **FAIL**

## Alcance ejecutado

Se inició únicamente la simulación Gazebo headless y una instancia temporal de
la clase real `JointControlWindow` en Qt offscreen. No se modificaron archivos,
parámetros persistentes ni configuración; no se usó `ros2 topic pub`, RViz,
cámara, hardware ni `sudo`.

La instancia temporal de GUI se creó en modo simulación con `use_sim_time` y
`require_system_readiness` para reproducir los parámetros que el launch aplica
al nodo GUI integrado. La intención se generó mediante el flujo real de la
ventana: `copy_current_cartesian_pose()`,
`adjust_cartesian_target(0, 0.01)` y pulsación de
`cartesian_send_button`. No se construyó ni publicó ningún mensaje de comando
fuera de `JointControlWindow`.

## Comandos ejecutados

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
QT_QPA_PLATFORM=offscreen python3 -u -
```

El segundo comando ejecutó el observador ROS y la instancia temporal de
`JointControlWindow` descrita arriba. Al detectar STOP, se envió SIGINT sólo al
launch de simulación; no permanecieron procesos de Gazebo ni clientes de la
prueba.

## Precondiciones antes de la intención

Todas las precondiciones iniciales fueron satisfechas antes de accionar X:

- `/thesis/system_readiness`: `READY`.
- `/thesis/proximity_status`: cinco muestras consecutivas `ALLOW`,
  `reference_frame=world`, `limiting_segment=wrist_2_to_tool`.
- `/joint_states`: seis articulaciones presentes y frescas.
- TF `world -> j2n6s300_end_effector` disponible.

Estado inicial:

| Señal | Valor |
|---|---|
| TF xyz (m) | `[0.596213317, 0.069556776, 0.925780059]` |
| J1..J6 (rad) | `[-0.000000778, 2.617996211, 3.665248235, -0.000056998, 0.349167682, 0.000009141]` |

## Resultado por eje

| Eje | Intención GUI | Ruta observada | Posición final / desplazamiento | Estado | Resultado |
|---|---|---|---|---|---|
| X | `+0.010 m`, pose absoluta desde la TF medida | `CartesianCommand` → `JointCommand` del adaptador → `/thesis/supervised_command` → Gazebo | TF xyz final `[0.605724763, 0.069990205, 0.925980672]`; ΔX=`+0.009511446 m` | Decisión `ALLOW/CLEAR`; ejecución `PENDING → ACCEPTED → SUCCEEDED`; controles de ejecución observados `ALLOW/CLEAR` | Movimiento y signo correctos, pero invalidado por STOP preventivo posterior durante la ventana de prueba. |
| Y | No enviada | No aplicable | No aplicable | Detenida tras STOP de proximidad | No ejecutada. |
| Z | No enviada | No aplicable | No aplicable | Detenida tras STOP de proximidad | No ejecutada. |

La GUI publicó `CartesianCommand` con:

```yaml
command_id: cartesian_gui_40565000000
frame_id: world
pose_xyz: [0.606, 0.070, 0.926]
```

El adaptador produjo el `JointCommand` candidato correspondiente, el
supervisor produjo salida supervisada para el mismo `command_id`, y Gazebo
aceptó y completó la trayectoria. No se observó rechazo de IK ni decisión
`HOLD`/`STOP` de `ExecutionControl` para ese ID.

Articulaciones finales de X (rad):

```text
[-0.000932428, 2.576764373, 3.582427856,
 0.023334671, 0.301189825, 0.001970247]
```

## Motivo de FAIL y detención

Durante la ventana de ejecución de X, el observador recibió cuatro muestras:

```yaml
state: STOP
reference_frame: world
limiting_segment: TF_UNAVAILABLE
```

La regla de esta validación exige detenerse inmediatamente ante cualquier
STOP/`TF_UNAVAILABLE`. Por ello no se enviaron Y ni Z y se cerró la simulación.
Aunque la ruta de X fue funcional y su desplazamiento tuvo el signo solicitado,
la presencia de estos STOP impide declarar PASS y muestra que la corrección
offline A-02a todavía no ha eliminado el evento durante la prueba integrada.

## Conclusión

**FAIL.** La ruta GUI → adaptador cartesiano → supervisor → Gazebo está
demostrada para X positivo, con salida supervisada y movimiento coherente. La
validación X/Y/Z completa queda pendiente: debe investigarse el
`TF_UNAVAILABLE` recurrente observado bajo la simulación integrada antes de
probar Y o Z. No se cambió A-02a ni su timeout durante esta prueba.
