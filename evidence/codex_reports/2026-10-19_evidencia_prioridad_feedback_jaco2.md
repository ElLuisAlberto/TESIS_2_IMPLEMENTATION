# Evidencia de prioridad de feedback JACO2

| Elemento | Resultado |
| --- | --- |
| Cambio | `control_tick()` libera `state_mutex_` antes de `SendBasicTrajectory` |
| Compilación/pruebas | `thesis_hardware_bridge`, 21/21 aprobadas |
| J1 origen | -1.551814113493139 rad |
| Objetivo ida | -1.499454235933 rad (+3°) |
| Objetivo retorno | -1.551814113493139 rad |
| Escala/estado | 1.0; sólo ALLOW/CLEAR |
| Edad `/joint_states` en telemetría | 36.481–48.705 ms durante los dos tramos |
| Watchdog / fault | ninguno / `fault=false` |
| Postura final observada | -1.551814113493139 rad; velocidad cero |
| Estado final | `armed=false`, `mode=hold`, salida 0 Hz |

Fuentes conservadas:

- Telemetría completa del ciclo: `evidence/physical_sessions/20261020T035758Z/`.
- Eventos de ejecución: ida aceptada a 1792468729.119806528 y completada a 1792468730.541729450; retorno aceptado a 1792468737.247299433 y completado a 1792468738.891904354.
- Log del adaptador: `/home/luis/.ros/log/jaco_hardware_node_26608_1792468678018.log`; sólo registra el desarmado solicitado a 1792468744.884955060.
- Rosbag incompleto preservado: `evidence/rosbags/2026-10-19_jaco2_feedback_priority_j1_3deg/`. Contiene 96 `/joint_states` con hueco máximo de 50.473 ms, pero sólo hasta 1792468720.463964744, antes del armado, y no contiene `metadata.yaml`.
