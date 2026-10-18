# Evidencia: regresión headless final de Z

Fecha: 2026-10-18  
Entorno: Gazebo headless, exclusivamente simulación

## Alcance

Se reutilizó la pila que había quedado activa tras la interrupción de la tarea,
sin reiniciarla ni aplicar cambios al parche:

```text
ros2 launch thesis_simulation thesis_system.launch.py \
  initial_pose:=z_ready \
  use_demo_obstacle:=true obstacle_x:=2.0 \
  simulation_output_enabled:=true \
  start_gui:=false start_rviz:=false headless:=true
```

No se inició `hardware_output`, no se enviaron órdenes al Kinova Jaco2 físico
y se publicó exactamente una intención cartesiana.

## Precondiciones

Las consultas ROS 2 se ejecutaron por separado con
`--no-daemon --spin-time 2`. Se obtuvieron:

```text
/thesis/system_readiness: READY
/thesis/proximity_status: ALLOW
minimum_clearance: 1.2023092641167632 m
obstacle_center: (2.0, 0.0, 0.65) m
```

La lectura independiente de `tf2_echo` fue estable:

```text
Translation: [0.596, 0.070, 0.926]
RPY (radian): [-0.737, -0.135, -1.680]
```

Antes del envío, `/thesis/cartesian_candidate` tenía dos suscriptores:
`cartesian_command_adapter` y `cartesian_sim_validator`.

## Único objetivo

El objetivo se calculó desde la TF viva y sumó exactamente 0,020 m a Z:

```text
PUBLISHED_ONCE
id=codex-z-ready-20261018-final
xyz=(0.596213320, 0.069556777, 0.945780055) m
rpy=(-0.736673381, -0.135453222, -1.680141413) rad
duration=5 s
subscribers=2
```

## Resultado correlacionado

Salida completa del validador:

```text
=== VALIDACION CARTESIANA EN GAZEBO ===
COMANDO=codex-z-ready-20261018-final
OBJETIVO_XYZ_M=0.5962,0.0696,0.9458
PASO_SOLICITADO_M=0.020000
GIRO_SOLICITADO_GRADOS=0.000
CHECK_INTENCION_CARTESIANA=OK | objetivo X/Y/Z/Roll/Pitch/Yaw observado
CHECK_CONVERSION_IK=OK | JointCommand generado por el adaptador
CHECK_SUPERVISION=OK | ALLOW: ALLOW codex-z-ready-20261018-final; duration=5.000s; scale=1.000000
CHECK_EJECUCION=OK | estado=SUCCEEDED
ERROR_POSICION_M=0.000147
ERROR_ORIENTACION_GRADOS=0.087
CHECK_POSICION_FINAL=OK | error=0.0001 m; límite=0.0150 m
CHECK_ORIENTACION_FINAL=OK | error=0.09 grados; límite=3.00
RESULTADO=OK
```

## Cierre

Se envió SIGINT únicamente al proceso de lanzamiento de esta simulación. Todos
sus procesos terminaron limpiamente. El log de lanzamiento no contiene
`Traceback`, `RuntimeError`, excepciones ni procesos muertos, y la comprobación
final no encontró procesos residuales de la pila.

Log ROS/Gazebo:

```text
/tmp/tesis_z_incidents_regression_logs/2026-10-18-13-23-27-971782-miNave-78444/launch.log
```
