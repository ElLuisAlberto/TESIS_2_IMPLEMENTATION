# Resultado: objetivo Z +20 mm desde `z_ready`

Fecha: 2026-10-18

## Resultado funcional

El único objetivo Z +20 mm en Gazebo fue aceptado y terminó en `SUCCEEDED`.
Las precondiciones exigidas se cumplieron antes de publicarlo:

- READY tipado confirmado dentro del plazo de 20 s; el propio monitor llegó a
  READY aproximadamente 9.08 s después de iniciar la pila.
- Proximidad `ALLOW` con clearance `1.202309 m`, superior a `0.30 m`.
- Obstáculo demo observado en `(2.0, 0.0, 0.65) m`.

La IK convergió en tres iteraciones, con residuo lineal `0.0001 m` y angular
`0.09 grados`. El supervisor emitió `PREDICTION ALLOW`, `ALLOW FORWARDED` y
`RUNTIME ALLOW`. El adaptador de Gazebo registró `GAZEBO SUCCEEDED` para
`codex-z-ready-20261018-001`.

## Validación TF

TF inicial:

```text
XYZ = (0.596, 0.070, 0.926) m
RPY = (-0.737, -0.135, -1.680) rad
```

TF final:

```text
XYZ = (0.596, 0.070, 0.946) m
RPY = (-0.737, -0.134, -1.681) rad
```

A la resolución disponible de `tf2_echo`, X/Y permanecieron constantes y Z
aumentó 20 mm. La orientación permaneció prácticamente constante.

## Incidencia y criterio de cierre

La validación funcional directa es positiva, pero el resultado global queda
**con incidencia de observación**: `cartesian_sim_validator` no recibió el
comando ni sus eventos y finalizó con `COMANDO=AUSENTE`, aunque los logs de los
tres productores registran el mismo `command_id`, el ALLOW del supervisor y
`SUCCEEDED`.

En cumplimiento de “detente ante cualquier fallo”, no se publicó un segundo
objetivo. Solo se leyó la TF final y se cerró la pila. El cierre produjo además
excepciones rclpy en cinco nodos Python después del resultado terminal; no
afectaron la ejecución ya completada.

No se modificó código ni se accedió a hardware físico.

Evidencia completa:
[2026-10-18_evidencia_z_ready_z_20mm_gazebo.md](../../evidence/codex_reports/2026-10-18_evidencia_z_ready_z_20mm_gazebo.md).
