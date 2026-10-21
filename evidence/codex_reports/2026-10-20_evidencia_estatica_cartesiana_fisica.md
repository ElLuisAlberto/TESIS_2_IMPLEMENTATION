# Evidencia estática cartesiana física

Rosbag:
`evidence/rosbags/20261020_180127_cartesian_physical_readonly/`

```text
Duración: 34.410005444 s
Mensajes totales: 2257
/joint_states: 676
/tf: 688
/thesis/hardware/diagnostics: 67
/thesis/hardware/armed: 68
/thesis/system_readiness: 67
/thesis/proximity_status: 690
/thesis/cartesian_jog_intent: 0
/thesis/jog_intent: 0
/thesis/supervised_jog_command: 0
/thesis/candidate_command: 0
/thesis/supervised_command: 0
/thesis/execution_control: 0
```

```text
JointState rate: 19.9998 Hz
Intervalo mediano/p95/máximo: 49.999/50.117/51.103 ms
Rango articular J1..J6: 0 rad
Velocidad máxima J1..J6: 0 rad/s
TF válidos al sello articular: 676/676
Error FK–TF mediano/p95/máximo: 1.42e-16 m
armed=false: 68/68
readiness=READY: 67/67
fault=false, state_valid=true, mode=hold: 67/67
output_command_rate_hz_measured=0.000000
jog_input_rate_hz_measured=0.000000
```

La proximidad fue `WARNING` en 690/690 muestras, segmento
`wrist_2_to_tool`, despeje constante 0.183257 m. No hubo STOP ni
`TF_UNAVAILABLE`.

