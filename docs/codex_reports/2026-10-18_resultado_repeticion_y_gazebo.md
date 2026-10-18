# Resultado de repetición Y en Gazebo

Fecha: 2026-10-18.

La prueba Y **aprobó** en una pila nueva de Gazebo headless. `READY` se verificó
con tipo explícito y llegó en el primer intento, a los 5 s del plazo global de
20 s. También aprobaron `/joint_states`, proximidad `ALLOW`, conexiones del
supervisor y TF inicial estable.

Se publicó una única intención nominal de +20 mm en Y:

```text
inicio TF mostrado: (0.000, 0.064, 1.181) m
objetivo:           (0.000, 0.084, 1.181) m
orientación:        (0, 0, -90°)
duración:           5 s
```

La TF interna sin redondear produjo un paso medido de 19,738 mm. La IK generó
el `JointCommand`, el supervisor respondió `ALLOW` con escala 1,0, Gazebo
aceptó la trayectoria y terminó en `SUCCEEDED`.

El error final fue 0,497 mm y 0,016°. La TF final mostrada fue
`(0.000, 0.084, 1.181) m`; por tanto, el signo y la magnitud Y son correctos y
X/Z/orientación se conservaron dentro de la precisión observada.

No se usó hardware físico ni se publicó otro objetivo. La pila se cerró y no
quedaron procesos. Evidencia completa:
[2026-10-18_repeticion_y_gazebo.txt](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/evidence/codex_reports/2026-10-18_repeticion_y_gazebo.txt).
