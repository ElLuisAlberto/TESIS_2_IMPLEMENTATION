# Resultado de pruebas cartesianas pasivas

Fecha: 2026-10-18. Se ejecutaron solamente las pruebas pasivas 5, 6 y 7 del plan cartesiano. No se publicó ningún comando ROS ni se realizó una prueba de movimiento.

| Elemento observado | Estado | Evidencia |
| --- | --- | --- |
| `cartesian_command_adapter` | Presente | Suscrito a `/thesis/cartesian_candidate`; publica `/thesis/candidate_command`. |
| `cartesian_jog_adapter` | Presente | Suscrito a `/thesis/cartesian_jog_intent`; publica `/thesis/jog_intent`. |
| `cartesian_trajectory_adapter` | Presente | Suscrito a `/thesis/cartesian_trajectory_candidate`; publica `/thesis/jog_intent` y el estado de trayectoria. |
| `safety_supervisor` | Presente | Nodo observado como `safety_supervisor_node`; recibe candidato y jog, y publica flujos supervisados. |
| `/joint_states` | Presente | Se recibió una muestra con las seis articulaciones del brazo y articulaciones de dedos. |
| TF `world` a `j2n6s300_end_effector` | Presente tras el arranque | Tras un error transitorio de frame inexistente, TF informó repetidamente la transformación `[0.000, 0.064, 1.181]` con yaw aproximado de `-1.571`. |
| Conexiones cartesianas | Presentes | Los tres adaptadores quedan conectados al supervisor y sus salidas a los consumidores de simulación y horizonte. |
| `system_readiness` | Presente, en espera | Publicó `WAITING: sin dato reciente: cálculo de proximidad; sin dato reciente: fuente de obstáculo`. |

Las pruebas 5 y 6 se aprueban para presencia de nodos, tópicos, `/joint_states` y TF. La prueba 7 confirma el grafo del supervisor, pero `system_readiness` no alcanzó `READY`. Ese resultado es coherente con `use_demo_obstacle:=false`: no hubo una fuente de obstáculo ni un cálculo reciente de proximidad.

La ruta observada quedó así: los destinos cartesianos entran en los tres adaptadores; `cartesian_command_adapter` publica `/thesis/candidate_command`; los adaptadores jog y trayectoria publican `/thesis/jog_intent`; `safety_supervisor_node` produce `/thesis/supervised_command`, `/thesis/supervised_jog_command` y `/thesis/execution_control`; `simulation_command_adapter` y `horizon_preview` son suscriptores. Los tópicos de entrada cartesiana no tuvieron publicadores, por lo que ningún X/Y/Z fue enviado.

Se detectó una limitación del launch: aunque `start_gui:=false` desactivó la interfaz de `thesis_ui` y `start_rviz:=false` desactivó RViz, el launch interno de Ignition abrió un proceso de GUI de Gazebo. No se interactuó con él y se cerró junto con todos los procesos al terminar. Antes de repetir una prueba pasiva debe disponerse de una opción de lanzamiento realmente headless.

No se inició ni conectó hardware físico, no se usó la cámara RGB-D, no se habilitó `simulation_output_enabled`, no se usó `sudo`, y no se publicaron comandos ROS. Se verificó que no quedaron procesos de la prueba.

La siguiente acción mínima no se ejecutó: solicitar autorización explícita y una forma de iniciar Ignition sin GUI para repetir la inspección pasiva. No se avanzará a pruebas de movimiento sin autorización expresa.
