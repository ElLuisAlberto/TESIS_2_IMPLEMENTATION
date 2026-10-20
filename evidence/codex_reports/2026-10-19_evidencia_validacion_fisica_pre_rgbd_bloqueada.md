# Evidencia de campaña física pre-RGB-D

## Rosbags

- `evidence/rosbags/2026-10-19_jaco2_pre_rgbd_fase0_b/`
  - 103.875 s; 5,606 mensajes.
  - 2,078 `/joint_states`; rango J1 0 rad; velocidad máxima J1 0 rad/s.
  - 208 diagnósticos; armado HOLD sin JOG y salida 0 Hz.
- `evidence/rosbags/2026-10-19_jaco2_pre_rgbd_fase1_j1/`
  - 39.177 s; 2,108 mensajes.
  - 778 `/joint_states`; rango J1 0 rad; velocidad máxima J1 0 rad/s.
  - 0 candidato, 0 decisión, 0 supervisado, 0 control, 0 trayectoria.

## Rechazo del primer intento

```text
command_id: pre_rgbd_j1_plus_3deg_1
reason_code: POINT_COMMAND_STAMP_STALE
supervisor receive - intent stamp: 1.147 s
```

## Estado final seguro

```text
armed=false
jog_active=false
mode=hold
output_command_rate_hz_measured=0.000000
fault=true  (hardware backend unavailable; no nuevo armado ejecutado)
```
