# Resultado de validación Z / `spin_node`

Fecha: 2026-10-18.

La compilación y las pruebas unitarias del parche aprobaron, pero la validación
headless se detuvo porque READY no llegó dentro del plazo global de 20 s.

## Resultados

- `thesis_core` y `thesis_simulation`: compilación correcta.
- Dos regresiones Z y pruebas de `spin_node`: `5 passed in 0.07s`.
- Pila pasiva: inició con `simulation_output_enabled:=false`, sin GUI ni RViz.
- READY: permaneció en `WAITING: esperando controller_manager` durante seis
  muestras y expiró a los 20 s (`READY_VERIFIED=false`).
- Cierre: se envió SIGINT únicamente al launch después del timeout. Todos los
  procesos terminaron limpiamente y no reapareció la excepción de `rclpy`.

El resultado global queda **no aprobado**: el parche unitario está validado y
el cierre observado fue limpio, pero no pudo cumplirse el cierre *tras READY*
porque esa precondición falló. Conforme a la regla de detención, no se reintentó
ni se realizaron diagnósticos adicionales.

No se modificó código en esta validación, no se publicaron comandos y no hubo
movimiento.

Evidencia completa:
[2026-10-18_validacion_z_spin_node_ready_fallida.txt](../../evidence/codex_reports/2026-10-18_validacion_z_spin_node_ready_fallida.txt).

