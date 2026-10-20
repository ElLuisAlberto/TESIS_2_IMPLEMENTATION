# Evidencia: backend y watchdog JACO2

| Elemento | Evidencia |
| --- | --- |
| Rosbag | `evidence/rosbags/2026-10-19_jaco2_fase1_j1_10deg_recuperacion/` |
| Duración / mensajes | 36.374 s / 2,002 mensajes sqlite3 |
| Hueco máximo `/joint_states` | 354.190 ms, de 1792468297.471654725 a 1792468297.825844676 |
| Límite de watchdog | 200 ms (`state_timeout_sec=0.20`) |
| Log correlacionado | `JACO STOP/HOLD: JACO joint state watchdog expired` a 1792468297.871100269 |
| Proximidad | 728 muestras; hueco máximo 94.028 ms; ALLOW |
| API al relanzar | un dispositivo, `connected=true`, `api_open=true`, `fault=false` |
| Estado final | `armed=false`, `jog_active=false`, `mode=hold`, salida 0.000 Hz |
| Posición J1 final | -1.5518141135 rad; objetivo alcanzado antes del STOP |
| Pruebas | `thesis_hardware_bridge/test_bridge_logic`: 20/20 aprobadas |

El log de la sesión previa no contiene `StartControlAPI failed`, `GetAngularPosition failed`, `SendBasicTrajectory failed` ni evidencia de otra sesión propietaria. La nueva sesión se inició con `mock_hardware=false`, `hardware_output_enabled=true`, `use_demo_obstacle=true`, GUI/RViz desactivados, y quedó desarmada.
