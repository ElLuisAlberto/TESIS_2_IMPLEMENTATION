# Resultado de prueba integrada Y en Gazebo

La prueba Y usó una pila nueva de Gazebo headless con salida de simulación habilitada. Las precondiciones aprobaron: `/joint_states` completo, TF inicial `(0,000, 0,064, 1,181)`, `system_readiness: READY`, proximidad `ALLOW` y supervisor disponible mediante consultas `--no-daemon --spin-time 2`.

Se publicó un único objetivo absoluto `world (0,000, 0,084, 1,181)`, que solicitaba `+0,020 m` solo en Y. El resultado fue un rechazo seguro en el adaptador cartesiano:

```text
INVERSE_KINEMATICS_FAILED
error lineal=0.0003 m; error angular=0.01 grados
```

El comando no alcanzó el supervisor ni el controlador de Gazebo: no hubo `JointCommand` candidato, decisión del supervisor, trayectoria supervisada ni estado de ejecución. TF final permaneció en `(0,000, 0,064, 1,181)`, por lo que el desplazamiento observado fue cero en los tres ejes. La pila se detuvo sin procesos restantes.

La desviación identificada es la tolerancia de posición de `0,0002 m` configurada para el adaptador, menor que el residuo IK de `0,0003 m` para este objetivo. No se modificó código. La evidencia está en [2026-10-18_prueba_integrada_y_gazebo.txt](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/evidence/codex_reports/2026-10-18_prueba_integrada_y_gazebo.txt).
