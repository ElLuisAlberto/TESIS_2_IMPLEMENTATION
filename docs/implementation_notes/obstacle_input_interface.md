# Validar la interfaz de obstáculos sin cámara

La interfaz `thesis_interfaces/msg/Obstacle` es el contrato de entrada para una
estimación conservadora de un obstáculo esférico. Incluye marca temporal de
adquisición, marco de referencia, centro, velocidad, radio e incertidumbre
radial. `obstacle_input_bridge` valida vigencia y valores finitos, transforma
centro y velocidad al marco `world`, y publica solo estimaciones aceptadas.
`proximity_monitor` suma radio e incertidumbre y compara el obstáculo con las
cápsulas del JACO2. Si el flujo deja de estar vigente en modo `topic`, el monitor
deja de publicar proximidad y el supervisor existente debe detener/rechazar
comandos por datos vencidos.

Esta prueba usa un obstáculo sintético; todavía no convierte nubes de puntos de
la D435i en geometría. Su propósito es verificar contrato, transformación,
evaluación geométrica y fallo conservador antes de conectar el sensor.

## Compilar y probar (Ubuntu 22.04 / ROS 2 Humble)

Desde la raíz del repositorio:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to thesis_simulation
source install/setup.bash
colcon test --packages-select thesis_core thesis_interfaces
colcon test-result --verbose
```

## Prueba integrada

Terminal 1, levantar la simulación con el monitor leyendo el contrato nuevo:

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch thesis_simulation jaco_gazebo.launch.py \
  obstacle_source_mode:=topic
```

Terminal 2, levantar el supervisor y el adaptador:

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch thesis_simulation safety_pipeline.launch.py \
  simulation_output_enabled:=true \
  require_proximity_status:=true \
  use_sim_time:=true
```

Terminal 3, publicar el obstáculo sintético con sello temporal actualizado:

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run thesis_core obstacle_demo_source --ros-args -p use_sim_time:=true
```

Terminal 4, observar la salida ya transformada y la distancia calculada:

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /thesis/obstacle_in_model_frame
```

En otra terminal, comprobar `/thesis/proximity_status`:

```bash
ros2 topic echo /thesis/proximity_status
```

El estado y la distancia dependen de la postura actual del brazo. Al terminar la
fuente sintética con `Ctrl+C`, en modo `topic` el monitor debe dejar de publicar
un estado de proximidad vigente; el supervisor debe responder
`PROXIMITY_UNAVAILABLE_OR_STALE` ante un nuevo comando. En modo `gazebo` (el
predeterminado) se conserva el comportamiento previo.

## Transformar desde otro marco

Cuando exista un `TF` entre el marco de la cámara y `world`, publicar el
`Obstacle` de entrada con ese nombre en `header.frame_id` y el tiempo de
adquisición en `header.stamp`. El puente solicita el `TF` correspondiente a ese
sello. Si falta la transformación, el sello está vencido o los valores no son
válidos, la muestra se descarta; no se reutiliza como obstáculo actual.
