# Evidencia de PARK a 0.8 m

Fecha: 2026-10-19
Rosbag SQLite3: `evidence/rosbags/2026-10-19_park_080_validation_disarmed/`

## Captura antes/después

- Duración: `63.986337223 s`.
- Tamaño: `18.7 MiB`.
- Mensajes: `8569`.
- PARK anterior: 387 mensajes de obstáculo en `(2.0, 0.0, 0.65)`.
- PARK nueva: 253 mensajes de obstáculo en `(1.0, 0.0, 0.65)`.
- Todos los obstáculos: frame `world`, radio `0.10 m`, incertidumbre `0.02 m`.

En el tramo de PARK nueva hubo 504/504 estados ALLOW durante 25.11 s, todos
con separación `0.8042543386313649 m`. La proyección calculada sin publicar
candidato dio `0.7970325880329506 m` para J1 +3° y
`0.8042543386313649 m` para J1 −3°.

En el tramo anterior apareció una sola muestra fail-closed
`STOP/TF_UNAVAILABLE` al comienzo de la captura, entre muestras ALLOW, con la
esfera todavía en la PARK antigua. No ocurrió durante ni después de la
transición a la PARK nueva y no altera su validación estable.

## Seguridad e inmovilidad

| Evidencia | Resultado |
|---|---:|
| armado | 0 verdaderos de 129 muestras |
| diagnóstico `armed=false` | 128/128 |
| `jog_active=false` | 128/128 |
| `mode=hold` | 128/128 |
| `fault=false` | 128/128 |
| `state_valid=true` | 128/128 |
| `proximity_valid=true` | 128/128 |
| `system_ready=true` | 128/128 |
| salida física 0 Hz | 128/128 |
| rango J1 | 0.0° |
| muestras `/joint_states` | 1280 |
| hueco máximo `/joint_states` | 51.067 ms |

Contadores de órdenes y ejecución:

```text
/thesis/jog_intent            0
/thesis/candidate_command     0
/thesis/supervised_command    0
/thesis/command_decision      0
/thesis/execution_control     0
/thesis/execution_trajectory  0
```

## TF y RViz

`tf2_echo world j2n6s300_end_effector` entregó transformaciones válidas y
estables tras el descubrimiento inicial. RViz permaneció activo. La captura
contiene 1281 mensajes de cápsulas, 640 de horizonte, 1280 de visualización de
obstáculo, 1280 de TF y 1 de TF estático. Los valores geométricos, no la
apariencia visual, fueron el criterio decisorio.

## Estado final

La esfera permanece en `(1.0, 0.0, 0.65)`, ALLOW/READY. El JACO2 permanece
desarmado, inmóvil, en hold y con salida 0 Hz.

PARK A 0.8 m VALIDADO Y LISTO PARA PRUEBA FÍSICA STOP
