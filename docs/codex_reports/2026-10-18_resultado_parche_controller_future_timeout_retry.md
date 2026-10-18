# Resultado del timeout/reintento de `controller_future`

Fecha: 2026-10-18.

El parche quedó implementado y validado dentro del alcance solicitado.

## Cambio

`SystemReadinessNode` asigna ahora un plazo monotónico de 2 s a cada consulta
`list_controllers`. Si una respuesta queda pendiente, cancela y limpia ese
future antes de permitir el siguiente intento. Nunca mantiene más de una
consulta pendiente.

La regresión reproduce una primera consulta sin respuesta y una segunda con
`arm_controller=active`; el resultado final publicado es READY.

## Verificación

- Compilación de `thesis_simulation`: aprobada.
- Regresión dirigida: `1 passed`.
- Suite final del paquete: `41 passed, 1 skipped`.
- Pila headless pasiva: READY verificado en el primer intento, a los 3 s del
  plazo global de 20 s.
- `simulation_output_enabled:=false`; no se publicaron intenciones ni hubo
  movimiento.

Durante el cierre aparecieron anomalías de destrucción de futures/rclpy,
incluidas en la evidencia. Ocurrieron después de READY y en nodos fuera del
alcance autorizado, por lo que no se amplió el parche.

Evidencia completa:
[2026-10-18_parche_controller_future_timeout_retry.txt](../../evidence/codex_reports/2026-10-18_parche_controller_future_timeout_retry.txt).

