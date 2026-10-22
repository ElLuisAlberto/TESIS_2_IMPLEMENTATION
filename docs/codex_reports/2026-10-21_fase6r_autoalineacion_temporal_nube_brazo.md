# Fase 6r: autoalineación temporal con nube del brazo

## Dictamen

`BLOCKED_MISSING_DEPENDENCY`

## Inspección realizada

Se revisaron `AGENTS.md`, el paquete `thesis_jaco_perception`, los launch de
sombra y cámara, `jaco_read_only_state.launch.py`, el YAML temporal
`world -> d435i_link`, y el perfil de seis cápsulas del JACO2.

El contrato vigente usa la nube original
`/camera/d435i/depth/color/points` en `d435i_depth_optical_frame`; el árbol
interno previsto es `d435i_link -> d435i_depth_frame ->
d435i_depth_optical_frame`, y el estado de lectura suministra `/joint_states`
y TF del brazo en `world`. Las seis cápsulas reutilizables corresponden a los
enlaces definidos en `jaco_self_filter_capsules.yaml`.

No se encontraron procesos activos de launch, RealSense, JACO2, TF, RViz ni
self-filter antes de esta fase.

## Bloqueo técnico

`thesis_jaco_perception` declara únicamente `rclcpp`, `sensor_msgs`,
`geometry_msgs`, `tf2_ros` y geometría propia. No declara ni dispone de un
solucionador no lineal robusto para minimizar, con seis grados de libertad, el
coste punto-a-superficie de cápsulas con pérdida truncada, límites de caja y
evaluación de observabilidad/confiabilidad.

Eigen3 está presente en el sistema como álgebra lineal, pero Ceres no está
instalado (`/usr/include/ceres/ceres.h` ausente) y no existe otro solucionador
robusto declarado en el paquete. Implementar desde cero un optimizador 6-DoF
con convergencia, rechazo de mínimos espurios y condición numérica verificable
sería precisamente el algoritmo inseguro que la solicitud prohíbe improvisar.

## Acciones no realizadas

No se modificaron paquetes, YAML finales, TF, cápsulas ni parámetros. No se
compiló ni se ejecutaron pruebas porque no se creó implementación. No se inició
cámara, JACO2, RViz, TF ni ningún nodo; por tanto no hubo salida física,
movimientos, escritura de candidato ni sesión de calibración.

Para continuar se requiere autorizar una dependencia de optimización robusta
ya empaquetada para ROS/C++ (por ejemplo Ceres) o proporcionar un solucionador
validado por el proyecto. Solo entonces podrían definirse la función
`min sum rho(abs(distance(point, capsule_surface)))`, los límites ±0.15 m y
±20 grados, y las pruebas sintéticas/ejecución única solicitadas.
