# Evidencia de cierre: JACO2 físico Joints y Cartesiano

Fecha: 2026-10-20

## Artefactos

- Rosbag sqlite3 válido:
  `evidence/rosbags/20261020_1835_jaco_cartesian_virtual_park_readonly_postfix/`
- Telemetría de la sesión final:
  `evidence/physical_sessions/20261020T233744Z/`
- Informe técnico:
  `docs/codex_reports/2026-10-20_cierre_brazo_fisico_joints_cartesiano.md`

## Resultado de pruebas

```text
thesis_core        175 passed, 1 skipped
thesis_ui           27 passed, 1 skipped
thesis_simulation   43 passed, 1 skipped
thesis_hardware     12 passed, 1 skipped
fallos               0
```

## Resumen del rosbag físico

```text
Bag duration                 35.762974624 s
Joint-state window           34.737497239 s
Joint-state samples          696
Joint-state rate             20.007198 Hz
Maximum joint-state gap      51.902879 ms
Maximum joint range          0.0 deg
Maximum reported velocity    0.0 rad/s
Kinematic TF status          357 VALID, 0 INVALID
Readiness                    71 READY, 0 NOT_READY
Armed                        71 false, 0 true
Proximity                    716 ALLOW
PARK clearance               0.808842384 m
JOG/candidate/decision       0 messages
Supervised/execution         0 messages
```

Diagnóstico durante la captura: `kinova_usb`, USB lock único, `connected=true`,
`api_open=true`, `fault=false`, `state_valid=true`, `system_ready=true`,
`proximity_valid=true`, `armed=false`, `jog_active=false`, `mode=hold`,
`output_command_rate_hz_measured=0.000000`.

## Evidencia de escenarios virtuales

```text
PARK
  center=(1.22, 0.0, 0.60) m
  observed=ALLOW
  current clearance=0.808842384 m
  projected J1 +3 deg=0.800874484 m
  projected J1 -3 deg=0.808842384 m

BLOCK_START
  center=(0.017715572, 0.246297072, 0.542325971) m
  observed=STOP
  current clearance=0.014999399 m

INSERT_STOP inactive
  center=PARK
  observed=ALLOW, readiness=READY

INSERT_STOP active
  center=(0.234428587, -0.129626180, 0.677325971) m
  observed=STOP
  current clearance=0.021785868 m

Return to PARK
  observed=ALLOW, readiness=READY
```

Todos los cambios de escenario se efectuaron con salida física deshabilitada,
`armed=false` y sin candidatos. No hubo movimiento articular.

## Estado final observado

Nodos activos incluyen:
`joint_control_gui_node`, `rviz2`, `cartesian_command_adapter`,
`cartesian_jog_adapter`, `cartesian_trajectory_adapter`, supervisor, bridge,
monitor, readiness y adaptador físico.

```text
output_permitted=true
connected=true
api_open=true
fault=false
state_valid=true
system_ready=true
proximity_valid=true
FK-TF VALID error=0.000000 m tolerance=0.005000 m
armed=false
active_goal=false
jog_active=false
mode=hold
output rate=0.000000 Hz
JOG input rate=0.000000 Hz
```

La esfera es virtual y no sustituye percepción RGB-D.

Tras abrir la interfaz, una interacción externa armó y activó JOG fuera de la
captura controlada. Se desarmó mediante `/thesis/hardware/set_armed` sin emitir
movimiento. La comprobación final posterior fue `armed=false`,
`jog_active=false`, `active_goal=false`, `mode=hold`, entrada JOG `0 Hz` y
salida `0 Hz`.
