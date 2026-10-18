# Resultado de prueba integrada Z en Gazebo

La prueba Z inició una pila de Gazebo nueva e independiente de Y. `/joint_states`, TF inicial `(0,000, 0,064, 1,181)`, proximidad `ALLOW` y el supervisor estuvieron disponibles mediante consultas directas con `--no-daemon --spin-time 2`.

La precondición `system_readiness` no aprobó: su publicador fue visible, pero dos ecos directos consecutivos no recibieron mensaje `READY` y terminaron con código 1. Conforme a la regla de detenerse ante una precondición no verificada, no se publicó el objetivo autorizado `world (0,000, 0,064, 1,201)`.

Por ello no existe TF final de movimiento, decisión del supervisor ni estado de ejecución para Z; TF permaneció en la postura inicial observada. La pila se detuvo y no dejó procesos. No se modificó código ni se usó hardware, cámara o `sudo`.

La evidencia está en [2026-10-18_prueba_integrada_z_gazebo.txt](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/evidence/codex_reports/2026-10-18_prueba_integrada_z_gazebo.txt).
