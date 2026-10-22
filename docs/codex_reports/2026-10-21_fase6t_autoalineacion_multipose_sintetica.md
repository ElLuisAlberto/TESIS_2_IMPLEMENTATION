# Fase 6T: cierre fail-closed de autoalineación multi-pose

## Dictamen

`EXPERIMENTAL_NOT_VALIDATED`

## Estado actual

El módulo `MultiPoseAlignmentEstimator` permanece aislado y experimental. La
recuperación sintética de una extrínseca común no está demostrada; por ello,
al completar sus tres rondas de cálculo retorna explícitamente
`accepted=false` y `reason="EXPERIMENTAL_NOT_VALIDATED"`. El rechazo existente
para menos de tres capturas se conserva como
`INSUFFICIENT_DISTINCT_POSES`.

El código experimental no constituye una calibración válida, publicable ni
utilizable para seguridad o para operación física.

## Alcance y aislamiento

No existe una ruta automática que cargue el módulo desde un launch, YAML
operativo o nodo ROS. No publica TF, no escribe YAML y no está conectado con
self-filter, supervisor, hardware bridge, pipeline D435i ni control físico.

En esta fase no se inició ROS, D435i, JACO2, RViz, TF, Gazebo ni ningún launch;
no se envió ningún comando físico.

## Validación ejecutada

Se compiló exclusivamente `thesis_jaco_perception` correctamente.

CTest ejecutó 10 pruebas: 10 pasaron y ninguna falló. La prueba multi-pose
verifica el contrato fail-closed (`accepted=false` y
`EXPERIMENTAL_NOT_VALIDATED`) y conserva la recuperación como diagnóstico no
aprobatorio. La discrepancia geométrica sigue sin demostrar una recuperación
válida:

```
r.pose[0] = -0.06508064612016172, truth[0] = 0.04, tolerancia = 0.015
r.pose[1] =  0.030625837969948496, truth[1] = -0.03, tolerancia = 0.015
```

`test_temporary_alignment` pasó y continúa exigiendo el rechazo de una sola
postura mediante `INSUFFICIENT_DISTINCT_POSES`. La prueba multi-pose también
exige que cada componente de la pose estimada sea finito; no exige precisión
geométrica mientras el módulo permanezca experimental.

`git diff --check` terminó sin salida ni errores.
