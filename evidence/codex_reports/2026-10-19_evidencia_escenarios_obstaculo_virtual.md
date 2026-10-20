# Evidencia: escenarios de obstáculo virtual JACO2

Fecha: 2026-10-19
Modalidad: observación física desarmada; ningún comando de movimiento.

## Artefacto principal

- Rosbag: `evidence/rosbags/2026-10-19_virtual_obstacle_scenarios_disarmed/`
- Almacenamiento: SQLite3
- Duración: `126.493345537 s`
- Tamaño: `37.8 MiB`
- Mensajes: `16945`
- Cierre: correcto; contiene `metadata.yaml` y un archivo `.db3`.

## Estados observados

| Estado de la única esfera | Muestras | Resultado en vivo | Separación en vivo |
|---|---:|---|---:|
| PARK | 598 mensajes de obstáculo | ALLOW | 1.799150020 m |
| BLOCK_START | 195 mensajes de obstáculo | STOP | 0.014999302 m |
| INSERT_STOP no insertado | incluido en PARK | ALLOW | 1.799150020 m |
| INSERT_STOP insertado | 472 mensajes de obstáculo | STOP | 0.040555118 m actual; 0.014999995 m proyectada |

La captura contiene 1197 muestras ALLOW y 1334 STOP. En cada estado la
separación fue constante: ALLOW `1.7991500197130952 m`; STOP entre
`0.014999302353134666 m` y `0.04055511833565435 m`, correspondientes a los dos
centros STOP y no a oscilaciones.

Todos los mensajes del obstáculo usaron frame `world`, radio `0.10 m`, margen
de incertidumbre `0.02 m` y velocidad cero. El bridge publicó el obstáculo en
el frame del modelo de forma continua.

## Invariantes de seguridad

| Evidencia | Resultado |
|---|---:|
| `/thesis/hardware/armed` | 0 verdaderos de 254 muestras |
| diagnóstico `armed` | false en 254/254 |
| `jog_active` | false en 254/254 |
| modo | hold en 254/254 |
| `fault` | false en 254/254 |
| `state_valid` | true en 254/254 |
| `proximity_valid` | true en 254/254 |
| salida física | 0.000000 Hz en 254/254 |
| rango J1 | 0.0 rad = 0.0 deg |
| tasa media `/joint_states` | 20.0024 Hz |
| hueco máximo `/joint_states` | 50.2193 ms |

`system_ready=false` apareció únicamente durante las 134 muestras diagnósticas
en que la esfera estaba en una pose STOP; fue el comportamiento fail-closed
esperado. Al restaurar PARK, quedó `system_ready=true` y ALLOW.

## Ausencia de órdenes y ejecución

Contadores dentro del rosbag:

```text
/thesis/jog_intent             0
/thesis/candidate_command      0
/thesis/supervised_command     0
/thesis/command_decision       0
/thesis/execution_control      0
/thesis/execution_trajectory   0
/thesis/trajectory_prediction  0
/thesis/pipeline_timing        0
```

Después de cerrar la captura, el grafo ROS informó además cero publicadores
para `/thesis/candidate_command` y cero para `/thesis/jog_intent`.

## TF y visualización

`tf2_echo world j2n6s300_end_effector` resolvió el transform de forma estable;
la traslación observada fue aproximadamente `[-0.072, 0.639, 0.738] m`. La
primera consulta transitoria ocurrió antes de que el cliente descubriera el
árbol y las muestras siguientes fueron válidas.

RViz permaneció activo. El rosbag acredita la alimentación de sus capas:

| Tópico visual | Mensajes |
|---|---:|
| `/thesis/robot_capsules` | 2517 |
| `/thesis/horizon_volume` | 1266 |
| `/thesis/obstacle_visualization` | 2530 |
| `/tf` | 2531 |
| `/tf_static` | 1 |

La evidencia decisoria es numérica; RViz se usó como comprobación de
coherencia espacial.

## Estado final

La única esfera fue restaurada a PARK. El stack permanece desarmado, en hold,
ALLOW/READY, sin JOG y con salida física 0 Hz. No se ejecutó movimiento físico.

ESCENARIOS VIRTUALES LISTOS PARA PRUEBA FÍSICA STOP
