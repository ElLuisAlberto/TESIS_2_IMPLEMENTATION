# Fase 1: geometría pura de auto-filtrado JACO2

Fecha: 2026-10-21
Alcance: creación aislada de la biblioteca geométrica preparatoria; no se inició ningún proceso ROS, cámara, simulador, RViz ni hardware.

## Resultado

Se creó `src/thesis_jaco_perception`, un paquete ROS 2 `ament_cmake` C++17 que expone únicamente una biblioteca de geometría pura. No depende de `rclcpp`, PCL, tf2, Gazebo, RealSense ni hardware, y no contiene nodos, publicaciones, suscripciones, launch ni archivos de calibración.

## Archivos creados

- `src/thesis_jaco_perception/CMakeLists.txt`
- `src/thesis_jaco_perception/package.xml`
- `src/thesis_jaco_perception/include/thesis_jaco_perception/self_filter_geometry.hpp`
- `src/thesis_jaco_perception/src/self_filter_geometry.cpp`
- `src/thesis_jaco_perception/test/test_self_filter_geometry.cpp`
- `src/thesis_jaco_perception/test/test_package_boundary.py`
- `src/thesis_jaco_perception/README.md`

## Funciones y tipos implementados

- Tipos: `Point3`, `Ray3`, `Capsule3`, `RayCapsuleIntersection` y `PointClassification`.
- Validación explícita de punto, rayo, cápsula y `epsilon_total`; los datos no finitos, radio no positivo, eje degenerado, rayo nulo y tolerancia negativa/no finita lanzan `std::invalid_argument`.
- Distancia punto-segmento.
- Primera intersección positiva rayo-cápsula, incluyendo cilindro, tangencias y tapas esféricas.
- Primera intersección sobre un conjunto de cápsulas, con el índice de la más cercana.
- Clasificación de punto observado desde la cámara según la regla solicitada:
  - sin intersección, o punto delante de `d_modelo - epsilon_total`: `EXTERNAL_CONFIRMED`;
  - diferencia absoluta con `d_modelo` dentro de `epsilon_total`: `SELF_CONFIRMED`;
  - punto detrás de `d_modelo + epsilon_total`: `OCCLUDED_UNKNOWN`.

`OCCLUDED_UNKNOWN` queda como estado explícito y nunca se convierte en espacio libre. Los radios/cápsulas usados están únicamente dentro de las pruebas sintéticas; no se creó una máscara física del JACO2 ni se reutilizaron las cápsulas preventivas del supervisor.

## Pruebas ejecutadas

```text
source /opt/ros/humble/setup.bash
colcon build --packages-select thesis_jaco_perception
# Resultado: 1 paquete terminado correctamente.

source install/setup.bash
colcon test --packages-select thesis_jaco_perception
colcon test-result --test-result-base build/thesis_jaco_perception --verbose
# Resultado: 12 tests, 0 errors, 0 failures, 0 skipped.
```

La suite cubre puntos delante/sobre/detrás de la cápsula, rayo sin contacto, entradas inválidas, selección de la primera cápsula, tangencias, tapas de cápsula y distancia punto-segmento. La prueba estática de frontera confirma que la biblioteca no incorpora dependencias de middleware, sensores, simulación o hardware y que no existen launch ni YAML de calibración en el paquete.

También se ejecutó `git diff --check`, sin errores.

## Ruta física y límites preservados

No se modificaron `thesis_perception`, `thesis_core`, `thesis_hardware`, `thesis_hardware_bridge`, `thesis_description` ni `thesis_interfaces` durante esta fase. Las modificaciones de Fase 0 que ya estaban en el árbol se preservaron. No se creó ni modificó ningún contrato ROS, tópico, launch o ruta hacia `/thesis/obstacle_input`; la D435i continúa aislada del supervisor preventivo y `hardware_output` no fue activado.

Esta fase no procesa `PointCloud2`, no transforma frames, no recibe `JointState`, no publica puntos/obstáculos y no contiene intrínsecos, extrínsecos, radios internos ni incertidumbres físicas. Por tanto no puede filtrar una nube real todavía.

## Siguiente fase propuesta

Crear un nodo sombra independiente que reciba `PointCloud2` expresada en `world`, consulte posteriormente estado/TF al sello de adquisición y use esta biblioteca para emitir resultados diagnósticos aislados. Debe permanecer sin conexión a `/thesis/obstacle_input` hasta completar calibración física, presupuesto de incertidumbre y validación de seguridad.
