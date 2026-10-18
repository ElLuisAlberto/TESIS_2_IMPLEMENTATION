# Diagnóstico TF durante el arranque de la simulación

Fecha: 2026-10-18  
Alcance: observación pasiva de la simulación Gazebo headless. No se modificó
código ni configuración y no se publicó ningún comando articular o cartesiano.

## Veredicto

**Categoría F — carrera de disponibilidad del búfer TF para el sello exacto de
adquisición**, con una manifestación inicial de la categoría A durante el
arranque. No es un desfase entre reloj simulado y obstáculo, ni un frame
incorrecto, ni una fuente de obstáculo incompatible.

El comportamiento fail-closed de `proximity_monitor` es correcto: ante la
ausencia de la transformación exacta publica `STOP/TF_UNAVAILABLE`. La causa
del bloqueo funcional es que esa consulta se efectúa sin espera para el sello
del obstáculo, mientras que la transformación de la misma época puede aún no
estar incorporada en el búfer local.

## Ejecución realizada

Se inició únicamente:

```bash
ros2 launch thesis_simulation thesis_system.launch.py initial_pose:=z_ready use_demo_obstacle:=true obstacle_x:=2.0 simulation_output_enabled:=true start_gui:=false start_rviz:=false headless:=true
```

Un observador Python de solo suscripción registró `/clock`,
`/thesis/obstacle_in_model_frame`, `/thesis/proximity_status` y
`/thesis/system_readiness`; además consultó pasivamente en su propio búfer
`world -> j2n6s300_end_effector` tanto al sello del obstáculo como a la TF más
reciente. Tras finalizar la captura, el launch se cerró mediante SIGINT. No se
inició GUI, RViz, cámara ni hardware.

## Evidencia temporal

| Evento | Tiempo simulado / sello | Observación |
|---|---:|---|
| Primer `STOP/TF_UNAVAILABLE` recibido por el observador | proximidad: `1.700`, `1.801`, `1.900` s; `/clock` al adjuntarse: `1.992` s | El monitor no tenía una TF utilizable para el sello de la medida. |
| Primer resultado no `TF_UNAVAILABLE` del monitor | proximidad `ALLOW`, sello `2.001` s; `/clock` `2.055` s | El monitor dejó de publicar el STOP inicial unos 54 ms simulados después del sello de la medida. |
| Primera observación directa de TF disponible | `/clock` `2.501` s, consulta *latest*: disponible | La cadena `world -> j2n6s300_end_effector` existe. El observador se adjuntó después del arranque, por lo que no permite fechar su primera aparición absoluta. |
| Primer obstáculo observado | sello `2.500` s, frame `world`; `/clock` `2.501` s | El sello coincide con el reloj simulado (diferencia observada: ~1 ms). |
| Consulta TF para el sello exacto `2.500` s | no disponible; consulta *latest*: disponible | No hay reinterpretación de frame ni uso de una TF actual para una medida anterior. |
| Readiness inicial | `/clock` `2.249` y `2.975` s | `WAITING: arm_controller ...`. |
| Readiness READY | `/clock` `3.671` s | Ocurre después del primer `ALLOW` del monitor. |

La captura continuó hasta aproximadamente `18.9` s simulados. En ese período,
los obstáculos siguieron llegando con `frame_id=world` y sellos alineados al
`/clock`; la TF más reciente siguió disponible. Aun así se observaron STOP
intermitentes para el sello exacto (por ejemplo alrededor de `3.000`, `4.000`,
`5.500` y `5.802` s), seguidos habitualmente por `ALLOW` con **el mismo sello
de obstáculo** en la siguiente evaluación (aprox. 50 ms después). Por tanto el
fenómeno no es exclusivamente la carrera transitoria inicial: puede reaparecer
mientras la simulación está estable y ya anuncia `READY`.

## Relación causal comprobada

1. `obstacle_demo_source.py` sella cada muestra con
   `self.get_clock().now()` y el launch le pasa `use_sim_time`; el bridge
   conserva ese sello y normaliza el frame a `world`.
2. `proximity_monitor_node.py` valida el obstáculo con el reloj simulado y
   solicita cada extremo geométrico mediante
   `lookup_transform('world', frame, Time.from_msg(source_stamp))`.
3. Esa consulta del monitor no especifica un tiempo de espera. Si el mensaje
   TF de la misma época no ha llegado todavía a su búfer, lanza
   `TransformException` y el monitor publica `STOP` con
   `limiting_segment=TF_UNAVAILABLE` y el sello original.
4. El `system_readiness` usa deliberadamente reloj de pared para vigilar la
   recepción reciente de tópicos, nodos, controlador y conexiones. Su método
   `evaluate()` sólo comprueba que haya recibido una proximidad reciente: no
   exige que su estado sea `ALLOW` ni que la TF sea consultable para el sello
   exacto del obstáculo.

Las pruebas revisadas respaldan este contrato: A-02a exige consultar TF al
sello de adquisición y exige que una excepción TF produzca STOP; las pruebas
de frescura rechazan sellos cero, antiguos o futuros. No hay una prueba de
integración que modele la llegada de una TF de la misma época justo después de
la muestra de obstáculo.

## Clasificación de alternativas

- **A, carrera transitoria de inicio:** presente como manifestación inicial,
  pues el primer STOP fue seguido de ALLOW antes de `READY`.
- **B, desfase de relojes o sellos:** descartado por los sellos de obstáculo
  alineados con `/clock` y el uso de `use_sim_time` en la fuente, bridge y
  monitor.
- **C, TF ausente o frame incorrecto:** descartado como causa persistente: el
  frame observado fue `world` y la cadena TF más reciente estuvo disponible.
  La ausencia es específica al tiempo exacto solicitado.
- **D, fuente de obstáculo incompatible:** descartada: el demo publica el
  contrato `Obstacle` válido y el bridge lo acepta/publica en `world`.
- **E, readiness publicado prematuramente:** no respecto del STOP inicial,
  porque `READY` llegó después del primer ALLOW. Sin embargo `READY` no es una
  garantía continua de disponibilidad TF por sello: continuaron STOP
  intermitentes después de `READY`.
- **F, otra causa:** confirmada: carrera entre la publicación sellada del
  obstáculo y la incorporación de TF histórica equivalente en el búfer que
  realiza la consulta exacta.

## Estado y recomendación mínima (no implementada)

El STOP es **transitorio al inicio, pero intermitente/persistente como clase de
fallo** durante la sesión observada. Mantener STOP es la decisión segura;
no debe sustituirse por la TF más reciente ni relajarse la verificación de
sello.

El cambio mínimo a evaluar en una tarea acotada es dar a la consulta TF exacta
del monitor una espera acotada, consistente con su período de evaluación, para
que pueda entrar al búfer la TF de **ese mismo sello**. Si vence esa espera,
debe conservarse exactamente el `STOP/TF_UNAVAILABLE`. En paralelo, la
readiness sólo debe interpretarse como disponibilidad de componentes, no como
garantía de una evaluación de proximidad ALLOW por sello. No se implementó ni
se modificó A-02a.

## Resultado

Diagnóstico completado. No se enviaron comandos de movimiento. La prueba
funcional X/Y/Z debe permanecer bloqueada hasta decidir y validar el tratamiento
acotado de esta carrera TF sin debilitar el contrato de sello de A-02a.
