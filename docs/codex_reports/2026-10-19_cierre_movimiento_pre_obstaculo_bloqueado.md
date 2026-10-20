# Cierre de movimiento seguro pre-obstáculo: bloqueado

## Resultado

**BLOQUEADO.** La campaña física no se inició porque sus propias precondiciones son incompatibles con la prohibición de publicar obstáculos virtuales o usar obstáculos físicos.

El lanzamiento existente empleaba `use_demo_obstacle=true`. Se cerró ordenadamente mientras el brazo estaba desarmado y se relanzó con `use_demo_obstacle=false`, GUI deshabilitada y salida física sin armar. En esa configuración:

- `/thesis/obstacle_input` tiene cero publicadores;
- no existe el nodo `obstacle_demo_source`;
- `proximity_monitor` no publica un estado cuando no recibe una estimación fresca;
- `hardware_readiness` informa `NOT_READY: proximidad ausente u obsoleta`;
- el adaptador informa `system_ready=false` y `proximity_valid=false`.

El interbloqueo exige proximidad válida y fresca antes de armar. Publicar ALLOW sin una estimación de obstáculo, desactivar `require_proximity_status` o cambiar el comportamiento del monitor alteraría la política de seguridad y violaría explícitamente el encargo.

## Estado seguro final

El backend Kinova está conectado y sano: `fault=false`, `state_valid=true`, lectura articular a aproximadamente 20 Hz y cero fallos de lectura. El brazo permanece `armed=false`, `jog_active=false`, `mode=hold`, velocidad J1 cero y salida física 0 Hz. No se publicó ningún candidato y no hubo movimiento.

## Cambio mínimo pendiente

Se necesita una decisión de alcance, no un cambio de código:

1. permitir durante esta campaña la fuente demo distante ya existente, exclusivamente como entrada de proximidad ALLOW; o
2. proporcionar una fuente de proximidad real aprobada y fresca.

Hasta entonces no es posible cumplir simultáneamente `proximidad ALLOW estable` y `no publicar/use obstáculos` sin eludir el interbloqueo. No se inició ninguna prueba WARNING/REDUCTION/STOP, obstáculo virtual ni RGB-D.
