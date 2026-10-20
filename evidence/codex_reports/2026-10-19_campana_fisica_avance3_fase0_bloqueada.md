# Evidencia — Fase 0 bloqueada y contención

## Observación inicial

```text
/thesis/hardware/armed: true
diagnóstico: jog_active=true, mode=jog
output_command_rate_hz_measured=100.048468
jog_input_rate_hz_measured=20.009694
```

## Contención

```text
ros2 service call /thesis/hardware/set_armed std_srvs/srv/SetBool {data:false}
success=True; JACO output disarmed
```

## Observación posterior

```text
/thesis/hardware/armed: false
jog_active=false; mode=hold
output_command_rate_hz_measured=0.000000
/joint_states: válido; velocidades de las seis articulaciones 0.0
/thesis/proximity_status: ALLOW, frame=world, clearance=1.811314 m
```

No se inició rosbag ni se publicó `/thesis/candidate_command`: la Fase 1 no
está autorizada por el fallo de la compuerta Fase 0.
