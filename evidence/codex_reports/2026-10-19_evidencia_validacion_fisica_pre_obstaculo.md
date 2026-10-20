# Evidencia de cierre físico pre-obstáculo

## Artefactos

- Campaña principal: `evidence/rosbags/2026-10-19_jaco2_cierre_pre_obstaculo/` — sqlite3, 160.132 s, 8,720 mensajes, `metadata.yaml` válido.
- Repetición del ciclo 2: `evidence/rosbags/2026-10-19_jaco2_cierre_pre_obstaculo_cycle2_retake/` — sqlite3, 54.050 s, 2,952 mensajes, `metadata.yaml` válido.
- Telemetría continua: `evidence/physical_sessions/20261020T041932Z/`.

## Integridad y seguridad

| Comprobación | Resultado |
| --- | --- |
| Compuerta estática | 61.626 s, 1,233 muestras, rango/velocidad J1 = 0 |
| Gap máximo `/joint_states` principal | 66.308 ms |
| Gap máximo `/joint_states` repetición | 67.912 ms |
| Edad máxima de estado en telemetría | 18.103 ms |
| Proximidad | sólo ALLOW, despeje ≈1.799 m |
| Decisiones finales | 4/4 aceptadas, ALLOW/CLEAR |
| Ejecuciones finales | 4/4 SUCCEEDED |
| Error terminal máximo | 0.200° |
| Velocidad medida máxima | 4.011°/s, menor que 5°/s |
| Oscilación en HOLD | 0.000° |
| Watchdog / fault / stale | 0 / 0 / 0 |
| Posición final J1 | -1.551814113493139 rad, igual al origen |
| Estado final | desarmado, HOLD, salida 0 Hz |

El log del adaptador sólo contiene dos `runtime disarm requested`, correspondientes a los desarmados explícitos; no contiene watchdog, error de API, fallo de transmisión ni STOP preventivo.

## Cambio probado

Archivo: `src/thesis_hardware/thesis_hardware/physical_candidate_generator.py`.

La espera de acknowledgements ocurre después de publicar. La prueba automática verifica la llamada de confirmación y conserva la prueba existente de rechazo de timestamp antiguo y aceptación del comando recién sellado. Resultado del paquete: 13 pruebas, 0 fallos, 1 omitida.
