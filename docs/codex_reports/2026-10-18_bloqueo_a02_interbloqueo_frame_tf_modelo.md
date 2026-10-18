# Cierre A-02 — interbloqueo de frame, TF y coherencia geométrica

Fecha: 2026-10-18  
Estado: **BLOQUEADO**

## Veredicto

A-02 no queda validado. No es seguro afirmar que un `ALLOW` procede de un
frame válido, de TF correspondiente a la medida y de un modelo coherente con
ese TF.

## Archivos modificados

No se modificó ningún archivo funcional. Solo se añadieron este informe y la
evidencia de análisis.

## Causa concreta

`proximity_monitor_node.py` calcula la geometría de cápsulas con el TF más
reciente (`Time()`), mientras que el obstáculo y el `ProximityStatus` conservan
un sello de adquisición. El monitor no dispone de `JointState` sincronizado y
el contrato de `ProximityStatus` no aporta la configuración articular, los
puntos del modelo ni la transformación del origen del modelo a `world`.

En consecuencia, aplicar `kinematic_tf_tolerance_m` exige incorporar una
cadena de estado/modelo sincronizada y definir su relación de frames. Eso es
un rediseño de contrato/topología que excede el alcance exclusivo de A-02. La
instrucción de la tarea exige documentar y detenerse en ese caso.

Además, la revisión confirma que aún deben resolverse dentro de una propuesta
de alcance autorizada: validación estricta de
`ProximityStatus.reference_frame == world`, preservación de STOP upstream y
consultas TF al sello de la medida con rechazo fail-closed.

## Pruebas y validación

No se ejecutaron pruebas ni compilación. La detención ocurrió antes de editar,
por el bloqueo de diseño requerido para el criterio de tolerancia TF--modelo.
No se inició ROS, Gazebo, GUI, cámara ni hardware, y no se publicaron comandos.

La evidencia detallada está en
[2026-10-18_analisis_bloqueo_a02_interbloqueo_frame_tf_modelo.md](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/evidence/codex_reports/2026-10-18_analisis_bloqueo_a02_interbloqueo_frame_tf_modelo.md).

