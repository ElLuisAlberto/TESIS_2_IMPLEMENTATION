# Evidencia — incidente de armado GUI JACO2

Fecha: 2026-10-19 (America/Lima)

| Momento | Evidencia |
| --- | --- |
| Antes de armar | GUI: JACO2 conectado, salida desarmada; modo de control articular continuo activo. |
| Tras confirmar armado | Diagnóstico: `output_permitted=true`, `armed=true`, `jog_active=true`, `mode=jog`, salida física ~100 Hz. |
| Contención | Respuesta de `/thesis/hardware/set_armed`: `success=True`, `JACO output disarmed`. |
| Después | `armed=false`, `jog_active=false`, `mode=hold`, salida física `0.000000 Hz`. |

La validación de 15 s READY/ALLOW con salida 0 Hz no aplica: se detuvo la
prueba en cuanto apareció JOG activo. El brazo quedó desarmado.
