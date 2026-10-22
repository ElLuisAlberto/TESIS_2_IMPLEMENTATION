# Fase 6B — Calibración temporal y sombra segura

Fecha: 2026-10-21. Se implementó únicamente software inactivo por defecto. No
se inició D435i, RViz, JACO2, hardware, Gazebo, supervisor ni TF estático.

## Dependencias

`thesis_jaco_perception` declara actualmente `rclcpp`, `sensor_msgs`,
`geometry_msgs`, `diagnostic_msgs` y `tf2_ros`, pero no declara OpenCV,
`cv_bridge`, `image_transport`, ArUco, ChArUco, `apriltag_ros` ni `aruco_ros`.
No se instaló ninguna dependencia. La implementación deja un contrato y mocks
unitarios independientes del detector; el nodo declara por defecto
`detector_available=false` y no calcula una pose mientras no haya detector
local revisado.

## Archivos creados/modificados

- `src/thesis_jaco_perception/include/thesis_jaco_perception/calibration_candidate_logic.hpp`
- `src/thesis_jaco_perception/src/calibration_candidate_logic.cpp`
- `src/thesis_jaco_perception/src/d435i_calibration_candidate_node.cpp`
- `src/thesis_jaco_perception/config/d435i_calibration_candidate.yaml`
- `src/thesis_jaco_perception/config/d435i_extrinsic_final.yaml`
- `src/thesis_jaco_perception/launch/d435i_calibration_shadow.launch.py`
- `src/thesis_jaco_perception/test/test_calibration_candidate_logic.cpp`
- `src/thesis_jaco_perception/test/test_calibration_contract.py`
- `src/thesis_jaco_perception/CMakeLists.txt`

El núcleo valida número de muestras, reproyección, dispersión de traslación y
rotación, y marca todo resultado como `session_only=true`. La persistencia
requiere una aceptación explícita posterior; el nodo no escribe archivos y la
configuración final comienza inválida (`valid: false`, marcos vacíos).

El nodo recibe los contratos RGB/CameraInfo y expone solo un tópico de
candidato (`/thesis/calibration/candidate_pose`) y diagnósticos. No contiene
`tf2_ros::TransformBroadcaster`, no publica TF, no arranca auto-filtrado y no
se conecta a extractor, `/thesis/obstacle_input` ni supervisor.

El launch `d435i_calibration_shadow.launch.py` usa `enabled=false` por defecto.
Solo acepta un fichero candidato mediante el argumento explícito
`candidate_extrinsic_file`; tampoco publica TF ni inicia
`jaco_self_filter_node`.

## Pruebas ejecutadas

```text
colcon build --packages-select thesis_jaco_perception --symlink-install
colcon test --packages-select thesis_jaco_perception --event-handlers console_direct+
```

Resultado: compilación correcta y 6/6 pruebas correctas. Las nuevas pruebas
cubren muestras insuficientes, error de reproyección alto, dispersión alta,
persistencia sin aceptación, YAML final inválido, extrínseca candidata temporal
y launch inactivo sin TF ni conexión de seguridad.

## Repetición cuando exista soporte fijo

1. Instalar o declarar mediante un cambio revisado la dependencia del detector
   elegida (preferentemente OpenCV/ChArUco), sin reutilizar una pose temporal.
2. Fijar el soporte de cámara y montar el tablero registrable definido en Fase
   6A respecto a la base.
3. Implementar el adaptador de detector que convierta esquinas/poses en
   `CalibrationSample`; mantener `session_only=true`.
4. Recoger al menos 20 muestras estáticas y aceptar solo las que cumplan las
   métricas configuradas.
5. Revisar visualmente la candidata. Una autorización separada debe aceptar la
   persistencia y, en otra fase, permitir TF de validación; no está habilitado
   por este cambio.

## Seguridad

No hubo hardware, cámara real, TF activo ni conexión al supervisor. El JACO2
no se inició ni movió. No se publicaron comandos ni mensajes en
`/thesis/obstacle_input`.
