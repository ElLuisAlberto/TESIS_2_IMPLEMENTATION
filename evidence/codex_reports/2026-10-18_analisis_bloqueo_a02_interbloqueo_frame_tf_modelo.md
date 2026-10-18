# Evidencia de análisis — A-02 interbloqueo de frame, TF y modelo

Fecha: 2026-10-18  
Resultado: **BLOQUEADO ANTES DE CAMBIOS FUNCIONALES**

## Método

Se revisaron únicamente los archivos autorizados para A-02 y sus contratos
inmediatos:

- `src/thesis_core/thesis_core/safety_supervisor_node.py`
- `src/thesis_core/thesis_core/proximity_monitor_node.py`
- `src/thesis_core/thesis_core/obstacle_input_bridge.py`
- `src/thesis_core/thesis_core/jaco_kinematics.py` (solo para comprobar el
  contrato geométrico ya consumido por las rutas anteriores)
- `src/thesis_interfaces/msg/Obstacle.msg`
- `src/thesis_interfaces/msg/ProximityStatus.msg`
- `src/thesis_core/config/jaco_capsules.yaml`
- pruebas directas bajo `src/thesis_core/test/` relacionadas con proximidad,
  frescura y política.

No se editó código, YAML, launch, mensajes ni pruebas. No se compiló ni se
ejecutaron pruebas, ROS, Gazebo, GUI, cámara ni hardware.

## Hechos verificables

1. `ProximityMonitorNode._frame_point()` llama a
   `lookup_transform(reference_frame, frame_name, Time())`. `Time()` solicita
   el TF más reciente, no el TF correspondiente a `source_stamp` de la
   observación. `evaluate()` conserva `source_stamp` al publicar
   `ProximityStatus`, por lo que el sello de la salida y la geometría usada
   pueden representar instantes diferentes.
2. `ObstacleInputBridge` sí usa
   `Time.from_msg(message.header.stamp)` al transformar un obstáculo. Por tanto
   ya existe un contrato de tiempo de adquisición que el monitor no puede
   validar con el TF que consulta actualmente.
3. `ProximityMonitorNode` no se suscribe a `JointState` ni conserva posiciones
   articulares con sello. `ProximityStatus.msg` tampoco contiene posiciones
   articulares, puntos de cápsula del modelo ni el frame/origen del modelo.
4. `capsule_segments()` en `jaco_kinematics.py` parte de una matriz identidad;
   aunque sus comentarios denominan el resultado `world`, no recibe ni aplica
   una transformación publicada entre la base del modelo y el `world`
   configurado. Para comparar sus seis segmentos contra TF haría falta definir
   y validar explícitamente esa relación.
5. El supervisor declara y valida `kinematic_tf_tolerance_m`, pero no dispone
   de los puntos TF de las seis cápsulas ni de la configuración articular con
   que se debería calcular el modelo. Por ello no puede evaluar una discrepancia
   métrica con ese parámetro a partir de `ProximityStatus`.
6. El supervisor acepta actualmente `ProximityStatus.reference_frame` sin
   exigir `world`, y `apply_proximity_policy()` permite que una predicción
   sustituya el estado upstream antes de decidir. Son riesgos reales descritos
   por A-02, pero una corrección parcial no satisfaría el interbloqueo completo
   de TF--modelo requerido.

## Motivo del bloqueo

El requisito de usar `kinematic_tf_tolerance_m` como interbloqueo fail-closed
necesita, como mínimo, una fuente sincronizada de posiciones articulares,
definir el frame del modelo respecto de `world`, consultar los TF de cada punto
al mismo sello de la medida y transportar o concentrar un resultado verificable
de esa comparación antes de que el supervisor admita `ALLOW`.

Ese flujo no está representado en los mensajes disponibles y obliga a ampliar
las interfaces o a introducir una ruta de estado/modelo nueva, además de tocar
la topología de datos fuera del cambio mínimo autorizado. La consigna A-02
ordena no implementar la tolerancia si ello exige un rediseño amplio; por tanto
no se aplicaron cambios parciales de frame, STOP o TF que pudieran dar una
impresión de validación completa.

## Estado de verificación

| Elemento | Estado |
| --- | --- |
| Frame `world` impuesto en supervisor | NO IMPLEMENTADO para A-02 |
| STOP upstream prioritario | NO VERIFICADO/CORREGIDO para A-02 |
| TF ausente, vencido o incompatible bloquea `ALLOW` | NO IMPLEMENTADO para A-02 |
| `kinematic_tf_tolerance_m` aplicado | NO IMPLEMENTADO para A-02 |
| Pruebas A-02 requeridas | NO EJECUTADAS: detención obligatoria previa a cambios |
| Compilación de paquetes afectados | NO EJECUTADA: detención obligatoria previa a cambios |

