# Validación física JACO2 previa a obstáculo virtual

## Veredicto

**LISTO PARA PRUEBA FÍSICA CON OBSTÁCULO VIRTUAL**

La fuente demo distante permanece activa únicamente como entrada ALLOW: centro `(2.0, 0.0, 0.65) m`, radio efectivo `0.12 m`, despeje mínimo aproximado `1.799 m`. No se ejecutaron escenarios WARNING, REDUCTION o STOP y no se introdujo un obstáculo en la trayectoria.

## Fase A — disponibilidad

Se cerró ordenadamente el lanzamiento sin fuente y se inició una única sesión con `mock_hardware=false`, `hardware_output_enabled=true`, `use_demo_obstacle=true`, GUI/RViz desactivados y control manual inactivo. Se verificaron backend Kinova conectado, API abierta, `fault=false`, estado válido a aproximadamente 20 Hz, TF `world -> j2n6s300_end_effector`, publicador demo único, proximidad ALLOW fresca y `system_ready=true`.

## Fase B — armado estático

La ventana armada previa al primer candidato duró 61.626 s, superando los 30 s requeridos. Incluyó 1,233 muestras de J1, con rango `0.000000000°`, velocidad máxima `0.000000000°/s`, `jog_active=false`, `mode=hold`, cero salida física y ningún candidato o JOG.

## Fase C — movimiento J1

Se usan como campaña final el ciclo 1 del rosbag principal y el ciclo 2 de la toma suplementaria. Ambos siguieron candidato → supervisor → adaptador, con 3° solicitados a 3°/s, decisión ALLOW/CLEAR y escala 1.0.

| Tramo | Objetivo relativo | Error al completar | Velocidad medida máx. | Asentamiento ≤0.5° | Finalización | Rango en HOLD |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Ciclo 1 ida | +3.000° | 0.200° | 4.011°/s | 1.191 s | 1.260 s | 0.000° |
| Ciclo 1 retorno | -2.900° desde el estado aceptado; destino exacto al origen | 0.200° | 4.000°/s | 1.190 s | 1.310 s | 0.000° |
| Ciclo 2 ida | +3.000° | 0.200° | 4.007°/s | 1.190 s | 1.309 s | 0.000° |
| Ciclo 2 retorno | -3.000° | 0.200° | 4.008°/s | 1.139 s | 1.309 s | 0.000° |

La velocidad medida máxima fue 4.011°/s, inferior al límite absoluto autorizado de 5°/s. Después del HOLD, J1 quedó exactamente en `-1.551814113493139 rad`, la posición inicial, con velocidad cero.

## Latencia agregada de cuatro tramos

| Métrica | Mediana | p95 | Máxima |
| --- | ---: | ---: | ---: |
| Intención → decisión | 6.668 ms | 8.993 ms | 9.294 ms |
| Decisión → salida del controlador | 72.161 ms | 104.459 ms | 104.465 ms |
| Intención → salida del controlador | 79.364 ms | 110.330 ms | 110.471 ms |
| Intención → respuesta articular observable | 215.654 ms | 374.843 ms | 400.128 ms |
| Asentamiento ≤0.5° | 1,190.013 ms | 1,190.618 ms | 1,190.659 ms |

El hueco máximo de `/joint_states` fue 66.308 ms en el rosbag principal y 67.912 ms en la toma suplementaria, ambos inferiores a 100 ms. La edad máxima vista por la telemetría durante los tramos fue 18.103 ms. Todos los diagnósticos conservaron `fault=false`, no hubo resets de deadline ni watchdog, y toda la proximidad registrada fue ALLOW.

## Defecto de evidencia y corrección

El rosbag principal perdió un candidato efímero del segundo ciclo aunque el supervisor y el adaptador lo procesaron. El generador destruía su publicador temporal sin esperar confirmación DDS de todos los suscriptores. Se añadió `wait_for_all_acked()` estrictamente después de `publish()`, por lo que el timestamp sigue asignándose inmediatamente antes de publicar y no envejece antes de la admisión. `thesis_hardware` compiló y sus 13 pruebas pasaron (12 aprobadas, 1 omitida).

Se repitió únicamente el ciclo afectado. El rosbag suplementario contiene dos candidatos, dos decisiones, dos comandos supervisados y seis eventos de ejecución completos. No se cambiaron umbrales, timeouts, STOP, joints, velocidad ni amplitud; no fue necesario escalar a 5° o 10°.

## Estado final

`armed=false`, `jog_active=false`, `mode=hold`, `fault=false`, `system_ready=true`, proximidad ALLOW fresca y salida física 0 Hz. La sesión física permanece disponible y desarmada. La prueba con obstáculo virtual no se inició.
