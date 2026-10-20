# Corrección del desarmado automático durante retirada de STOP

Fecha: 2026-10-20

## Síntoma observado

La salida física podía armarse dentro de un STOP geométrico, pero se desarmaba
automáticamente justo cuando un JOG de retirada alcanzaba la región REDUCTION.
El log mostró primero decisiones STOP para JOG en dirección incorrecta, después
un JOG REDUCTION aceptado y, acto seguido, `safety arm interlock lost: STOP or
HOLD is active`.

## Causa raíz

El adaptador mantenía `supervisor_stop_active=true` después de una decisión STOP
anterior. A la vez, `system_ready_callback()` borraba
`protective_recovery_active` en cuanto readiness pasaba a `true`. Como la
decisión STOP histórica y el JOG supervisado nuevo llegan por tópicos distintos,
existía una ventana de carrera STOP -> REDUCTION/READY -> JOG en la que el
interlock veía el STOP antiguo sin la excepción de retirada y desarmaba.

No fue una pérdida del backend, del USB, de TF ni del feedback articular.

## Corrección mínima

- Se modeló explícitamente el ciclo de vida del latch STOP.
- Un STOP nuevo continúa activándolo con prioridad.
- Un movimiento supervisado fresco y admitido lo limpia antes de activar JOG.
- Un desarmado completado elimina el STOP histórico, sin borrar un STOP
  geométrico vigente (que sigue llegando por proximidad).
- La retirada protectiva se conserva durante la carrera STOP -> READY hasta que
  el adaptador admita realmente el JOG nuevo; después se desactiva.
- No se cambiaron umbrales, escalas, watchdog, política STOP ni geometría.

Archivos afectados:

- `src/thesis_hardware_bridge/include/thesis_hardware_bridge/bridge_logic.hpp`
- `src/thesis_hardware_bridge/src/jaco_hardware_node.cpp`
- `src/thesis_hardware_bridge/test/test_bridge_logic.cpp`

## Verificación

- Compilación selectiva: `thesis_hardware_bridge`, correcta.
- Pruebas: 24/24 casos GTest correctos; resumen ament 25 pruebas, cero fallos.
- Casos nuevos: persistencia del STOP ante eventos no admitidos, limpieza al
  admitir movimiento/desarmar y carrera STOP -> READY -> JOG.
- Reinicio controlado del único launch físico, estando `armed=false`.
- Backend después del reinicio: conectado, API abierta, `fault=false`, estado
  válido, feedback 19.99 Hz.
- Estado preventivo actual: REDUCTION, despeje 0.051291 m, segmento
  `wrist_2_to_tool`, frame `world`, `system_ready=true`.
- Estado final: `armed=false`, `mode=hold`, `jog_active=false`,
  `safety_stop_active=false`, salida física 0 Hz.

La prueba dinámica final queda en manos del operador desde la GUI. El agente no
armó ni publicó comandos durante esta corrección.
