# Evidencia de compuerta sin fuente de obstáculo

Fecha: 2026-10-19.

Lanzamiento físico recuperado con `mock_hardware=false`, `hardware_output_enabled=true`, `use_demo_obstacle=false`, `start_gui=false` y `start_rviz=false`.

| Señal | Valor observado |
| --- | --- |
| Backend | `kinova_usb`, conectado, API abierta |
| `fault` | `false` |
| `state_valid` | `true` |
| Feedback | 20.004 Hz |
| `armed` | `false` |
| `jog_active` | `false` |
| `mode` | `hold` |
| Salida física | 0.000 Hz |
| `system_ready` | `false` |
| `proximity_valid` | `false` |
| J1 | -1.551814113493139 rad, velocidad 0 |
| Publicadores `/thesis/obstacle_input` | 0 |
| `obstacle_demo_source` | ausente |

Log de lanzamiento: `/tmp/jaco2_pre_obstacle_no_demo_20261019.log`.

No se creó rosbag de movimiento porque la compuerta obligatoria falló antes del armado. No hubo comandos candidatos ni movimiento físico.
