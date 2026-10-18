# Resultado de repetición Z en Gazebo

Fecha: 2026-10-18.

La prueba Z **falló de forma segura en la conversión IK** y el procedimiento se
detuvo. Se usó una pila nueva de Gazebo headless. El primer estado fue
`WAITING: arm_controller no está active` y no se aceptó; `READY` exacto llegó
en el segundo intento, a los 7 s del plazo global de 20 s. Las demás
precondiciones —estado articular, proximidad `ALLOW`, supervisor conectado y
TF inicial— aprobaron.

Se publicó una única intención nominal de +20 mm en Z:

```text
inicio TF mostrado: (0.000, 0.064, 1.181) m
objetivo:           (0.000, 0.064, 1.201) m
orientación:        (0, 0, -90°)
duración:           5 s
```

La TF interna sin redondear midió un paso de 19,923 mm. El adaptador rechazó la
IK con residuo lineal de 19,9 mm y error angular de 0,00°. No generó un
`JointCommand`, por lo que no hubo decisión aceptante del supervisor, action
goal ni ejecución en Gazebo.

La TF final permaneció en `(0.000, 0.064, 1.181) m` y `−90°`: el rechazo no
produjo movimiento. No se publicó un segundo objetivo.

Al apagar la pila, `system_readiness` encontró una excepción de conversión de
mensaje en rclpy después del resultado; no causó el fallo IK, pero queda
registrada para revisión separada. No quedaron procesos activos. Evidencia
completa:
[2026-10-18_repeticion_z_gazebo.txt](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/evidence/codex_reports/2026-10-18_repeticion_z_gazebo.txt).
