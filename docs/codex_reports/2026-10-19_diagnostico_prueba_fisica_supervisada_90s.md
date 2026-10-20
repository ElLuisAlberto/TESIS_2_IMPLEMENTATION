# Diagnóstico de prueba física supervisada JACO2 — 90 s

Fecha: 2026-10-19 (America/Lima)

## Alcance

Captura pasiva en `sqlite3`, sin publicaciones ni cambios de parámetros por
Codex. El operador controló la GUI. La bolsa cubre 89.355 s y contiene 19,130
mensajes.

## Qué funcionó

- El adaptador permaneció armado en las 180 muestras disponibles y no reportó
  fallos, pérdida de conexión ni estado articular/proximidad inválido.
- `/joint_states` fue continuo a 20 Hz: período mínimo/mediano/máximo de
  46.879/50.001/53.075 ms, sin huecos mayores de 100 ms.
- La prueba fue JOG articular supervisado: 1,788 intenciones y 1,788
  controles de ejecución. Todas resultaron `ALLOW`, `CLEAR`, escala 1.0.
- La proximidad permaneció siempre `ALLOW` (1,787 muestras); no hubo
  `WARNING`, `REDUCTION` ni `STOP`.
- Latencia desde intención hasta publicación al controlador: 5.050 ms mínima,
  14.687 ms mediana, 21.183 ms p95 y 27.466 ms máxima. La predicción tardó
  3.777 ms de mediana y 14.471 ms máxima.

## Movimiento y seguimiento

Los recorridos máximos observados de J1..J6 fueron, en radianes:
1.1938, 0.4137, 2.3040, 0.7985, 0.6519 y 0.2173. Los pasos consecutivos
máximos fueron 0.0140, 0.0157, 0.0175, 0.0157, 0.0183 y 0.0209 rad,
respectivamente, compatibles con la realimentación de 20 Hz y sin saltos.

La velocidad pedida máxima por J1..J6 fue 0.3124, 0.2793, 0.3752, 0.3124,
0.4163 y 0.4634 rad/s. Los picos medidos fueron del mismo orden y no aparece
un sobreimpulso evidente en la telemetría. J6 cambió de signo con frecuencia
(148 cambios) junto con la entrada del operador; no hay una ventana posterior
de reposo que permita distinguir con rigor una oscilación residual.

En los últimos 4.95 s J1 aún recorrió 0.3491 rad, con velocidad máxima de
0.2444 rad/s. Por tanto, el ensayo terminó con entrada activa y no permite
certificar el asentamiento final ni ausencia de temblor después de un HOLD.

## Lo que no se pudo validar

No hubo mensajes en `/thesis/candidate_command`,
`/thesis/supervised_command`, `/thesis/command_decision` ni
`/thesis/execution_trajectory`. Esto significa que no se ejecutaron comandos
puntuales ni trayectorias cartesianas/planificadas por esos canales; solo se
validó la ruta JOG. No hay rechazos, retenciones ni reducciones que analizar.

El tópico solicitado para intención cartesiana no formó parte de la captura y
no existe evidencia de control cartesiano en los tópicos registrados. Para
validarlo se requiere una sesión separada que incluya su tópico de intención y
termine con al menos 5 s sin entrada.

## Riesgos y mejora mínima propuesta

No aparecen movimientos no solicitados en la correlación disponible, ni
retardo excesivo, STOP, fallo de lectura o sobreimpulso claro. La mejora
mínima propuesta no es un cambio de control: para la siguiente prueba, añadir
un período observacional de 5 s con la intención JOG explícitamente cero al
final y registrar también `/thesis/supervised_jog_command` y la intención
cartesiana cuando se pruebe esa ruta. Esto permitiría medir asentamiento,
hold efectivo y correlación extremo a extremo completa.
