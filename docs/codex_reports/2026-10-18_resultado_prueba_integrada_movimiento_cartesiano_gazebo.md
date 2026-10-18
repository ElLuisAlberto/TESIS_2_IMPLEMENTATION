# Resultado de prueba integrada de movimiento cartesiano en Gazebo

La prueba se ejecutó solo en Gazebo headless con `simulation_output_enabled:=true`, `start_gui:=false`, `start_rviz:=false` y `headless:=true`. Ignition inició como servidor (`ign gazebo -r -s`), sin procesos de GUI.

| Eje | Estado | Inicio TF (m) | Objetivo (m) | Resultado |
| --- | --- | --- | --- | --- |
| X | Aprobado | `(0,000, 0,064, 1,181)` | `(0,020, 0,064, 1,181)` | Supervisor `ALLOW/CLEAR`; Gazebo aceptó y completó la trayectoria; TF final `(0,020, 0,064, 1,181)`. Signo y magnitud correctos. |
| Y | No ejecutado | — | `(0,020, 0,084, 1,181)` previsto | Se detuvo antes de publicar porque no se pudo repetir la verificación obligatoria. |
| Z | No ejecutado | — | No calculado | Se detuvo antes de publicar por la misma condición. |

Antes de X, `/joint_states` tuvo un publicador activo, TF `world -> j2n6s300_end_effector` fue estable, `system_readiness` informó `READY` y proximidad informó `ALLOW` con despeje mínimo de 0,375 m. El adaptador cartesiano convirtió la intención a seis posiciones articulares; el supervisor publicó una decisión aceptada con escala 1,0; y `/thesis/execution_trajectory` registró `PENDING`, `ACCEPTED` y `SUCCEEDED`.

Después de X, TF confirmó el desplazamiento de `+0,020 m` exclusivamente en X. La orientación y las coordenadas Y/Z se mantuvieron dentro de la precisión observada. La duración fue 5 s y se publicó una sola intención cartesiana.

Antes de Y, los comandos de observación ROS fallaron durante el descubrimiento con `xmlrpc.client.Fault: <Fault 1: "<class 'RuntimeError'>:!rclpy.ok()">`. Aunque TF continuó estable, no fue posible repetir las verificaciones de `/joint_states` y `system_readiness`. Se aplicó la regla de detenerse ante una condición incoherente: no se enviaron Y ni Z y se apagó la pila.

La prueba no demuestra todavía el funcionamiento integrado de Y y Z. La siguiente acción mínima es diagnosticar de forma pasiva la inestabilidad de `rclpy` o del descubrimiento ROS 2 y repetir las verificaciones previas antes de solicitar una nueva autorización de movimiento para Y y Z.

No se modificó código, no se usó `sudo`, no hubo hardware físico ni cámara RGB-D, y no quedaron procesos de simulación al finalizar. La evidencia completa está en [2026-10-18_prueba_integrada_movimiento_cartesiano_gazebo.txt](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/evidence/codex_reports/2026-10-18_prueba_integrada_movimiento_cartesiano_gazebo.txt).
