# Fase 2: nodo sombra de auto-filtrado JACO2

Fecha: 2026-10-21
Alcance: ampliación exclusiva de `src/thesis_jaco_perception`; no se inició ROS, D435i, RViz, Gazebo ni el JACO2.

## Resultado

Se creó el ejecutable ROS 2 `jaco_self_filter_node`. Es un nodo de modo sombra: recibe una nube que ya debe estar en `world`, consulta TF **al sello exacto de adquisición** y la particiona usando la biblioteca geométrica de Fase 1. No consulta `JointState`, no usa TF más reciente y no modifica ninguna decisión preventiva.

## Archivos modificados o creados

- `src/thesis_jaco_perception/CMakeLists.txt`: biblioteca auxiliar pura y ejecutable nuevo, con dependencias ROS limitadas al nodo.
- `src/thesis_jaco_perception/package.xml`: dependencias de compilación del nodo (`rclcpp`, `sensor_msgs`, `tf2_ros`, diagnósticos y mensajes geométricos).
- `src/thesis_jaco_perception/include/thesis_jaco_perception/shadow_filter_logic.hpp`: tipos y funciones puras de validación/partición.
- `src/thesis_jaco_perception/src/shadow_filter_logic.cpp`: validación de configuración/cabecera y partición testeable.
- `src/thesis_jaco_perception/src/jaco_self_filter_node.cpp`: nodo sombra, TF exacto, serialización de nubes y diagnósticos.
- `src/thesis_jaco_perception/test/test_shadow_filter_logic.cpp`: pruebas sintéticas de partición y rechazos.
- `src/thesis_jaco_perception/test/test_package_boundary.py`: fronteras de dependencias, salida, frame/sello y ausencia de `JointState`.
- `src/thesis_jaco_perception/README.md`: documentación del nodo sombra y sus restricciones.

## Arquitectura de entradas y salidas

```text
/thesis/perception/points_world [PointCloud2, frame=world]
  -> jaco_self_filter_node
       TF exacto al header.stamp:
         world <- camera_optical_frame
         world <- capsule_start_frame[i]
         world <- capsule_end_frame[i]
  -> /thesis/jaco_perception/points_self_confirmed
  -> /thesis/jaco_perception/points_external_candidate
  -> /thesis/jaco_perception/points_occluded_unknown
  -> /thesis/jaco_perception/diagnostics
```

Las tres nubes usan `SensorDataQoS().keep_last(1)`, preservan la estructura/campos y sello de entrada mediante copia, fijan `frame_id=world` y contienen solo los índices de su clase. El nodo no publica ni menciona `/thesis/obstacle_input` como salida.

## Parámetros inválidos por defecto

El nodo comienza seguro e inhabilitado:

- `enabled=false`.
- `camera_optical_frame=""`.
- Listas de inicio, fin y radios de cápsulas vacías.
- `epsilon_total_m=-1.0`.

No son valores de calibración supuestos: impiden clasificación/publishing de nubes hasta que una fase posterior provea extrínseca real, frame óptico, radios interiores y presupuesto de incertidumbre validados. `expected_frame` debe ser exactamente `world`; `max_cloud_age_sec` es 0.5 s y `tf_timeout_sec` es 0.05 s, validado además como breve (máximo 0.1 s).

## Rechazo y diagnósticos

- Con `enabled=false`, no procesa ni publica nubes clasificadas y emite `DISABLED`.
- Configuración o calibración incompleta: `ERROR CALIBRATION_OR_CONFIGURATION_INVALID`; no publica nube externa.
- Nube sin sello, futura, vencida o con frame distinto de `world`: `ERROR CLOUD_INVALID`.
- Falta TF al sello: `ERROR TF_AT_STAMP_UNAVAILABLE`; no se consulta ni reutiliza TF latest.
- Los diagnósticos incluyen enabled, validez de configuración, edad del sello, conteos total/por clase, estado TF y mensaje de rechazo legible.

## Verificación ejecutada

```text
source /opt/ros/humble/setup.bash
colcon build --packages-select thesis_jaco_perception
# Resultado: 1 paquete terminado correctamente.

source install/setup.bash
colcon test --packages-select thesis_jaco_perception
colcon test-result --test-result-base build/thesis_jaco_perception --verbose
# Resultado final: 19 tests, 0 errors, 0 failures, 0 skipped.
```

También se ejecutó `git diff --check`, sin errores. Las pruebas cubren la partición de nube sintética (externo, propio y oculto), frame/sellos inválidos, listas/radios/epsilon inválidos, timeout TF inválido, modo disabled, frontera de tópicos, conservación de frame/sello por copia y ausencia de suscripción a `JointState`.

## Frontera física y limitaciones

No se modificaron `thesis_perception`, `thesis_core`, `thesis_hardware`, `thesis_hardware_bridge`, `thesis_description` ni `thesis_interfaces`; se preservaron los cambios preexistentes de Fase 0 y Fase 1. No se inició cámara ni hardware, no se activó `hardware_output` y no existe conexión del nuevo nodo al supervisor preventivo.

Siguen pendientes y bloquean cualquier promoción: extrínseca física `world <- camera_optical_frame`, identidad del frame óptico real, radios interiores calibrados, `epsilon_total_m`, presupuesto de latencia y validación/calibración física integral. No se implementó ninguna fase posterior.
