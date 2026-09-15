# Prevención continua durante la ejecución

Esta etapa agrega el lazo de supervisión en línea para simulación:

- `safety_supervisor` evalúa el horizonte corto a 10 Hz.
- `ExecutionControl` publica `ALLOW`, `WARNING`, `REDUCTION` o `STOP`.
- `simulation_command_adapter` conserva el `goal_handle` de Gazebo.
- `REDUCTION` cancela y reenvía el tramo restante con menor velocidad.
- `STOP` cancela el goal activo y deja la ejecución detenida.
- `horizon_preview` cambia el color del volumen según el estado runtime.
- El obstáculo puede actualizarse mediante `/world/jaco_world/pose/info`.

## Aplicación

Antes de extraer esta etapa, conserva una copia de seguridad de `src` y no
incluyas archivos de firmware o mapas de hardware en la extracción.

Después de aplicar los archivos:

```bash
source /opt/ros/humble/setup.bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
colcon build --symlink-install \
  --packages-select thesis_interfaces thesis_core thesis_simulation thesis_ui
source install/setup.bash
```

La primera prueba debe mantenerse en Gazebo. La salida del adaptador debe
estar explícitamente habilitada solo cuando se quiera observar el movimiento:

```bash
ros2 launch thesis_simulation safety_pipeline.launch.py \
  simulation_output_enabled:=true
```

El lanzamiento de Gazebo incorpora un puente para:

```text
/world/jaco_world/pose/info -> tf2_msgs/msg/TFMessage
```

Si el puente no acepta `ignition.msgs.Pose_V`, se debe comprobar la versión
de `ros_gz_bridge` antes de continuar con una prueba de obstáculo móvil.

## Verificación mínima

```bash
ros2 topic list -t | grep -E \
  'execution_control|execution_trajectory|proximity_status|horizon_volume'

ros2 topic echo /thesis/execution_control
```

Durante una ejecución se espera observar `ALLOW` o `WARNING`; al acercar el
obstáculo al volumen, debe aparecer `REDUCTION` y una nueva referencia
aceptada por Gazebo con mayor duración. Si la distancia proyectada alcanza
el umbral de parada, debe aparecer `STOP` y el goal debe cancelarse.
