# Fase 3: contrato de calibración sombra RGB-D/JACO2

Fecha: 2026-10-21
Alcance: cambios exclusivos dentro de `src/thesis_jaco_perception`; no se ejecutó ningún launch ni se inició componente físico o de percepción.

## Resultado

Se creó un contrato de configuración seguro y un launch de modo sombra para `jaco_self_filter_node`. Ambos preservan el bloqueo inicial: el YAML carga `enabled=false` y parámetros físicos vacíos o inválidos intencionalmente. No representa ni contiene una calibración física.

## Archivos creados o modificados

- `src/thesis_jaco_perception/config/jaco_d435i_shadow.yaml` (nuevo).
- `src/thesis_jaco_perception/launch/jaco_rgbd_shadow.launch.py` (nuevo).
- `src/thesis_jaco_perception/CMakeLists.txt`: instala los directorios `config` y `launch`.
- `src/thesis_jaco_perception/package.xml`: declara dependencias de ejecución para launch y prueba YAML.
- `src/thesis_jaco_perception/README.md`: documenta arquitectura Fases 1–3, límites y mediciones pendientes.
- `src/thesis_jaco_perception/test/test_package_boundary.py`: prueba estática del contrato y del aislamiento del launch.

## YAML y comportamiento seguro por defecto

`jaco_d435i_shadow.yaml` configura `jaco_self_filter_node` con:

```text
enabled: false
input_topic: /thesis/perception/points_world
expected_frame: world
camera_optical_frame: ""
capsule_start_frames: []
capsule_end_frames: []
capsule_inner_radii_m: []
epsilon_total_m: -1.0
max_cloud_age_sec: 0.5
tf_timeout_sec: 0.05
```

Los comentarios declaran que los valores vacíos/invalidantes son deliberados, que no se deben copiar radios preventivos de `thesis_core` y que el archivo no es una calibración. Al cargar este contrato, el nodo permanece deshabilitado y no clasifica ni publica nubes externas.

## Launch aislado

`jaco_rgbd_shadow.launch.py` inicia exclusivamente un `Node`:

```text
thesis_jaco_perception / jaco_self_filter_node
```

Carga el YAML anterior por defecto y expone `config_file` para una futura configuración física validada. No inicia RealSense, RViz, Gazebo, `robot_state_publisher`, launch físico ni publicador de TF estático. No contiene remapeos ni referencias a `/thesis/obstacle_input`, argumentos de armado ni `hardware_output`; por tanto sigue aislado del supervisor y de la ruta física.

## Verificación ejecutada

```text
source /opt/ros/humble/setup.bash
colcon build --packages-select thesis_jaco_perception
# Resultado: 1 paquete terminado correctamente.

source install/setup.bash
colcon test --packages-select thesis_jaco_perception
colcon test-result --test-result-base build/thesis_jaco_perception --verbose
# Resultado: 20 tests, 0 errors, 0 failures, 0 skipped.
```

También se ejecutó `git diff --check`, sin errores. No se ejecutó `ros2 launch` ni se inició ROS, D435i, RViz, Gazebo, JACO2 o `hardware_output`.

## Condiciones que bloquean una habilitación futura

El nodo debe seguir deshabilitado hasta medir y validar externamente:

1. La pose física `world -> cámara`.
2. El frame óptico real de la D435i.
3. Los radios interiores del modelo de auto-filtrado.
4. `epsilon_total_m` y sus contribuciones de incertidumbre.
5. La latencia extremo a extremo.
6. El volumen observable validado.

Sin esas condiciones no se debe sustituir el YAML sombra por una configuración física, ni conectar el nodo a `/thesis/obstacle_input` o al supervisor. No se implementó ninguna fase posterior.
