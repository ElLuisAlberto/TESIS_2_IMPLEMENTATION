# Evidencia de retiro protectivo desde STOP

Fecha: 2026-10-19

Estado físico previo al cambio:

```text
armed=false
system_ready=false
proximity_valid=true
safety_stop_active=true
minimum_clearance=0.04966429526497902 m
limiting_segment=wrist_2_to_tool
fault=false
mode=hold
output_command_rate_hz_measured=0.000000
```

La traza anterior confirmó el ciclo de bloqueo: el supervisor llegó a STOP en
0.050 m, el adaptador desarmó por `STOP or HOLD is active` y readiness publicó
`NOT_READY: proximidad en STOP o inválida`.

Después de compilar, probar y relanzar de forma desarmada:

```text
protective_recovery_active=true
armed=false
jog_active=false
system_ready=false
proximity_valid=true
safety_stop_active=true
fault=false
mode=hold
output_command_rate_hz_measured=0.000000
```

Pruebas automáticas:

- Armado normal sigue exigiendo todas las precondiciones.
- STOP sin recuperación explícita sigue rechazando armado.
- STOP geométrico válido permite armado de recuperación.
- Proximidad obsoleta vuelve a bloquear y desarmar.
- La GUI admite JOG articular en STOP geométrico válido.
- `TF_UNAVAILABLE` cancela y bloquea el modo de recuperación.

No se publicó ningún comando ni se produjo movimiento durante esta validación.
