# Snapshot técnico multipose

## Acceso

No hubo fallo de filesystem: las seis rutas absolutas existen, son archivos
regulares legibles (modo `0664`) y `sed -n` devolvió contenido con salida 0.
La afirmación previa de contenido no utilizable no tuvo error literal que la
sustentara; fue una captura incompleta de salida, no un problema del terminal.

## Estado actual

- `MultiPoseCapture`, `MultiPoseResult` y `MultiPoseAlignmentEstimator::estimate`
  existen en `multipose_alignment.hpp`. **PARCIAL**: las métricas declaradas no
  se rellenan.
- `MultiPoseCorrespondence` y `associate_multipose` existen. **IMPLEMENTADO**:
  contienen postura, punto, cápsula, ronda, punto óptico y ambas distancias; el
  gate y margen se aplican desde la pose candidata.
- `estimate` ejecuta tres rondas `(0.12,0.015)`, `(0.07,0.010)` y
  `(0.04,0.005)`. **PARCIAL**: Ceres se llama, pero no se calculan costes,
  cobertura, estabilidad, límites activos ni rechazo por mejora.
- El residual `Frozen` usa exclusivamente la `Capsule3` fijada por
  `capsule_index`; no reasigna cápsulas. **IMPLEMENTADO**.
- La prueba multi-pose existe pero solo cubre traslación. **PARCIAL**.

La composición real es `p_link = T_link_optical p_optical` y
`p_world = T_world_link p_link`. No se escribe YAML ni se publica TF.

## CTest

Se registraron 10 pruebas: 9 pasaron y falló `test_multipose_alignment`.

```text
r.pose[0] = -0.06508064612016172; truth[0] = 0.04;
diferencia 0.10508064612016171 > 0.015
r.pose[1] = 0.030625837969948496; truth[1] = -0.03;
diferencia 0.060625837969948498 > 0.015
```

Es fallo de implementación/observabilidad, no de infraestructura. `git diff
--check` terminó correctamente. No se inició hardware ni ROS.

## Siguientes cambios mínimos

1. Evaluar coste congelado antes/después y rechazar límite, no convergencia o
   falta de mejora.
2. Llenar cobertura por postura/cápsula y estabilidad por clave
   `(pose_index, point_index)`; rechazar antes de aceptar.
3. Corregir el generador/composición sintética y diagnosticar asociaciones por
   ronda antes de exigir recuperación 6-DoF.
