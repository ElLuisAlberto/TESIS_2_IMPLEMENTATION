# Validación física pre-RGB-D JACO2

## Veredicto

**BLOQUEADO.** Fase 0 fue aprobada. Fase 1 no ejecutó movimiento porque los
candidatos no llegaron al supervisor; después, el adaptador quedó desarmado
con diagnóstico `fault=true`. Fase 2 no se inició.

El hardware quedó desarmado. RGB-D, cámara, percepción y simulación no fueron
modificados.

## Trazabilidad y resultados

| Fase | Objetivo | Posición/rango medido | Error máx. | Latencia mediana/p95/máx. | Asentamiento | Decisión preventiva | Abortos | Resultado |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | 30 s armado sin intención | J1 rango 0.000°, velocidad 0 | 0.000° | N/A | N/A | ALLOW/CLEAR | 0 | **APROBADO** |
| 1/J1 | +3° a 3°/s, retorno y repetición | J1 rango 0.000° | 3.000° solicitado no ejecutado | N/A | N/A | no hubo decisión | 2 abortos seguros | **BLOQUEADO** |
| 2 | segundo joint seguro | no iniciado | N/A | N/A | N/A | N/A | N/A | **BLOQUEADO** |

## Fase 0

Rosbag: `evidence/rosbags/2026-10-19_jaco2_pre_rgbd_fase0_b/`.

Durante el intervalo armado se registraron 2,078 estados articulares y 208
diagnósticos. Los estados de diagnóstico fueron exclusivamente:

- `armed=true`, `jog_active=false`, `mode=hold`, salida física 0 Hz,
  `fault=false`; o
- el estado desarmado equivalente tras el cierre.

No hubo mensajes JOG, no hubo rango articular ni velocidad distinta de cero.
La corrección basada en `arm_epoch` superó por tanto la compuerta de armado
seguro.

## Fase 1 y causa del bloqueo

Primer candidato: el supervisor lo recibió y rechazó como
`POINT_COMMAND_STAMP_STALE`; el sello se creó antes de que el proceso `ros2
topic pub` terminara de inicializarse (desfase medido 1.147 s). No llegó al
adaptador ni produjo movimiento.

Segundo candidato: un nodo ROS efímero creó el sello al publicar, pero el
rosbag registró cero mensajes en candidato, decisión, supervisor, control y
trayectoria. La causa fue que el publicador no esperó a que DDS descubriera la
suscripción del supervisor. No se alcanzó la ruta de ejecución ni se movió el
brazo.

Tras la segunda sesión, ya desarmado, el diagnóstico cambió a
`fault=true`, con texto genérico `hardware backend unavailable`, aunque
`connected=true`, estado articular/proximidad válidos y salida 0 Hz. Ese estado
degradado bloquea correctamente todo nuevo armado. El adaptador no conserva la
causa textual de ese `fault`, por lo que no es responsable reintentar a ciegas.

## Seguridad

- Se usó exclusivamente la ruta ROS candidata -> supervisor -> adaptador.
- No se usó USB directo ni se alteraron STOP, umbrales o interbloqueos.
- La GUI manual fue detenida y el relanzamiento se hizo sin GUI/RViz.
- Tras cada intento se hizo HOLD y desarmado; no se produjo movimiento físico
  no solicitado.

## Siguiente paso

Diagnosticar en seco el origen de `fault=true` y mejorar el adaptador para
preservar su causa diagnóstica. Tras compilar/pruebas sin hardware y recibir
autorización explícita para una nueva campaña, repetir Fase 1 con un publicador
ROS que espere `get_subscription_count() > 0` antes de sellar y publicar. Solo
después de aprobar J1 se elegirá J2 o J3.
