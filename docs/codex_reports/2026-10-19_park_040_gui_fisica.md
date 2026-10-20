# PARK a 0.40 m y apertura de GUI física

Fecha: 2026-10-19

La misma esfera virtual quedó en `(0.59, 0.0, 0.65)` en frame `world`, con
radio geométrico `0.10 m` e incertidumbre `0.02 m`. La separación
superficie–superficie medida es `0.4011967569 m`.

La pose se obtuvo evaluando las cápsulas del robot y los horizontes J1 ±3°:

- J1 +3°: mínimo `0.3942256433 m`.
- J1 −3°: mínimo `0.4011967569 m`.
- Umbral WARNING conservado: `0.30 m`.
- Peor margen sobre WARNING: `0.0942256433 m`.

Se cambió únicamente el objetivo reproducible de PARK y su resolución de
búsqueda a 0.01 m. BLOCK_START e INSERT_STOP no se modificaron. No se cambiaron
umbrales, supervisor, márgenes, watchdog ni lógica de seguridad.

Compilación de `thesis_core` y `thesis_hardware`: correcta. Las cuatro pruebas
de escenarios pasaron y los archivos modificados pasaron `ament_flake8` y
`ament_pep257`.

La GUI física se abrió contra el stack existente con control de armado
disponible. No se accionó el armado ni se publicó movimiento desde esta tarea.
Estado posterior a abrirla: `armed=false`, `jog_active=false`, `mode=hold`,
`system_ready=true`, `fault=false` y salida física `0 Hz`.
