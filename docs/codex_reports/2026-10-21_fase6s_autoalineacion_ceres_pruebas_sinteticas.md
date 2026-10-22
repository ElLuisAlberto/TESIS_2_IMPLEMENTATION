# Fase 6s: Ceres y pruebas sintéticas

## Dictamen

`BLOCKED_IMPLEMENTATION`

## Auditoría de convención y causa raíz

La convención requerida es `p_A = T_A_B * p_B`.

| Transformación | Significado | Uso correcto |
|---|---|---|
| `T_world_d435i_link` | Pose optimizada | Lleva `p_d435i_link` a `world` |
| `T_d435i_link_optical` | Intrínseca fija RealSense | Lleva puntos ópticos a `d435i_link` |
| `T_world_optical` | Composición | `T_world_d435i_link * T_d435i_link_optical` |
| Generación sintética | Cámara desde superficie en mundo | `p_optical = inverse(T_world_optical) * p_world` |
| Evaluación | Nube óptica hacia cápsulas en mundo | `p_world = T_world_optical * p_optical` |

Para el caso fallido, cuya rotación era identidad y `T_d435i_link_optical`
se reducía a identidad, la generación `p_optical = p_world - t` y la
proyección `p_world = p_optical + t` son inversas exactas. Por tanto los
signos de `(+0.04, -0.03, +0.02)` no son la causa del resultado
`(-0.02535, +0.04850, -0.00123)`.

La causa es observabilidad insuficiente del coste actual: cada residual toma
la distancia mínima contra cualquiera de las seis cápsulas, sin
correspondencia punto-cápsula. En la nube sintética, ese mínimo permite que
puntos originalmente asociados a una cápsula se reasignen a otra superficie;
Ceres encuentra así un mínimo local con coste bajo pero pose incorrecta. Un
caso inverso deliberado tampoco sería una prueba segura mientras ese coste
permita reasignación libre.

No es admisible fabricar correspondencias por el orden de los puntos, pues la
nube RGB-D real no las aporta. Antes de añadir identidad, dos traslaciones,
rotaciones y pose combinada como pruebas de aceptación, el diseño necesita un
mecanismo físicamente justificable de correspondencia/observabilidad (por
ejemplo asociación por normales y gating geométrico validado). Sin ello esas
pruebas serían una validación engañosa. Los casos adversos actuales siguen
rechazados y `AT_LIMIT` nunca es aceptado.

La causa raíz ya quedó aislada: el primer caso no tenía una pose conocida y era
ambiguo; el generador fue sustituido por superficies de seis cápsulas desde una
traslación conocida. El nuevo caso mejora y es aceptado, pero no recupera la
traslación exigida: estimó `(-0.02535, 0.04850, -0.00123)` frente a
`(0.04, -0.03, 0.02)`. Esto confirma un mínimo espurio o una composición de
transformación aún incorrecta, no una aserción defectuosa. Se conservaron los
límites y el rechazo de solución en límite; no se relajaron tolerancias.

La segunda ejecución produjo 68 pruebas, con 2 fallos agregados por CTest
(el caso de recuperación y su resultado agregado). No se inicia hardware.

Se declaró y localizó Ceres explícitamente mediante `find_package(Ceres REQUIRED CONFIG)` y `Ceres::ceres`. Se añadió un núcleo temporal no ROS que usa pérdida Huber y límites de ±0.15 m y ±20 grados para minimizar distancia punto-a-superficie de seis cápsulas. El objeto de salida permanece `session_only=true` y `not_for_safety=true`; no escribe YAML ni publica TF.

La compilación de `thesis_jaco_perception` fue correcta. La prueba sintética de nube insuficiente pasó, pero la prueba de recuperación/control del optimizador falló: la solución alcanzó un límite (`AT_LIMIT`) donde el caso esperaba rechazo por `NO_IMPROVEMENT`. Por ello el conjunto del paquete no pasó: 67 pruebas, 2 fallos informados por CTest (una prueba objetivo fallida y su resultado agregado).

No se ejecutó ningún launch, cámara, JACO2, RViz, TF ni simulación. No se modificó la extrínseca final ni el self-filter. `git diff --check` se ejecutó como parte de la validación, pero el flujo quedó bloqueado antes de un resultado sintético aceptable.
