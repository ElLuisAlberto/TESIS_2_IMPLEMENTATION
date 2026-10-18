# Resultado de prueba integrada Y/Z en Gazebo

Se actualizaron [AGENTS.md](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/AGENTS.md) y el [plan de pruebas cartesianas](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/docs/codex_reports/2026-10-18_plan_pruebas_cartesiano.md) para exigir `--no-daemon --spin-time 2` en las consultas ROS 2 de nodos y tópicos.

La simulación headless se inició con `simulation_output_enabled:=true`, sin GUI ni RViz. Las precondiciones se confirmaron mediante consultas directas: `/joint_states` emitió una muestra completa, TF fue estable en `(0,000, 0,064, 1,181)`, proximidad informó `ALLOW` con 0,375 m de despeje, el supervisor fue publicador de `/thesis/supervised_command` y `system_readiness` informó `READY` tras un segundo descubrimiento de tres segundos.

| Eje | Resultado | TF inicial/final | Supervisor | Ejecución |
| --- | --- | --- | --- | --- |
| Y | No ejecutado | Inicial `(0,000, 0,064, 1,181)`; sin final | Sin decisión, porque no hubo comando | Sin trayectoria. La revisión automática rechazó la publicación antes de iniciar ROS. |
| Z | No ejecutado | No aplica | No aplica | No se intentó tras el rechazo de Y. |

La revisión automática rechazó el payload de Y por interpretar que `x=0,000` podía ser un retorno de X respecto de la prueba anterior. En esta pila nueva, TF confirmó que la posición inicial era `x=0,000`, pero no se reintentó la publicación. No hubo desviación física o de simulación: no se publicó ningún comando y se detuvo la pila.

La evidencia está en [2026-10-18_prueba_integrada_yz_gazebo.txt](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/evidence/codex_reports/2026-10-18_prueba_integrada_yz_gazebo.txt). No se modificó código, no se usó `sudo`, y no quedaron procesos de simulación.
