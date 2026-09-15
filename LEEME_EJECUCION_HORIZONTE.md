# Etapa 2: referencia ejecutada y volumen de horizonte

Esta etapa conecta tres datos que antes estaban separados:

1. El `safety_supervisor` publica el comando supervisado.
2. El `simulation_command_adapter` envía ese comando aceptado a
   `/arm_controller/follow_joint_trajectory` y publica su ciclo de vida en
   `/thesis/execution_trajectory`.
3. `horizon_preview` usa la referencia aceptada, el tiempo simulado y el
   estado articular para actualizar el volumen en
   `/thesis/horizon_volume`.

El modo `source:=auto` muestra la referencia aceptada en ámbar mientras está
activa. Si no existe una trayectoria aceptada, muestra en azul la intención
nominal publicada por la GUI cuando se activa el botón de volumen continuo.

## Copiar la etapa al repositorio

Desde el repositorio local:

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION

tar -czf "$HOME/Descargas/respaldo_antes_etapa2_$(date +%Y%m%d_%H%M%S).tar.gz" src

tar --no-same-owner -xzf \
  "$HOME/Descargas/tesis_horizonte_ejecucion_etapa2.tar.gz"
```

## Compilar y comprobar

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select thesis_interfaces thesis_core thesis_simulation thesis_ui

source install/setup.bash

colcon test --packages-select \
  thesis_core thesis_interfaces thesis_simulation thesis_ui
colcon test-result --verbose
```

La prueba nueva valida que el muestreo use un tiempo absoluto de ejecución,
que los joints continuos tomen el giro corto y que se rechacen parámetros
inválidos.

## Ejecución en Gazebo

Mantén la simulación y RViz en una terminal:

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch thesis_simulation jaco_gazebo.launch.py
```

Cuando `/joint_states` y `/arm_controller` estén disponibles, inicia el
supervisor y el adaptador en otra terminal. Para que exista una referencia de
ejecución real en Gazebo usa `simulation_output_enabled:=true`:

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch thesis_simulation safety_pipeline.launch.py \
  simulation_output_enabled:=true
```

En una tercera terminal inicia el visualizador con tiempo simulado:

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run thesis_core horizon_preview --ros-args \
  -p use_sim_time:=true \
  -p source:=auto
```

Finalmente inicia la interfaz:

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run thesis_ui joint_gui
```

En RViz, el display `MarkerArray` debe apuntar a
`/thesis/horizon_volume`. El frame del marcador es
`j2n6s300_link_base`; el Fixed Frame puede continuar siendo `world` si TF
publica la transformación del brazo.

## Verificación rápida

```bash
ros2 topic echo /thesis/execution_trajectory --once
ros2 topic hz /thesis/horizon_volume
ros2 topic echo /arm_controller/controller_state --once
```

Al enviar un comando permitido se deben observar los estados `PENDING` y
`ACCEPTED` en `/thesis/execution_trajectory`. Mientras la referencia está
activa, el volumen ámbar avanza desde la postura medida hacia el objetivo. La
GUI muestra el estado del adaptador y el volumen continúa actualizándose aun
cuando el objetivo no cambia.

Con `simulation_output_enabled:=false`, el adaptador publica `DRY_RUN` y no
hay referencia aceptada; ese modo sirve para comprobar la GUI y el volumen
nominal sin mover el brazo.

## Alcance de esta etapa

El volumen usa cápsulas cinemáticas muestreadas y un margen visual. La
referencia se sincroniza con el comando enviado y el visualizador muestra el
error articular disponible del controlador. Esta etapa todavía no cancela ni
reemplaza un goal activo y todavía no aplica una reducción de velocidad o un
`STOP` durante una trayectoria ya aceptada. El siguiente incremento debe
añadir esa decisión en línea, con cancelación del goal y una nueva referencia
de frenado publicada para que el volumen y la acción permanezcan coherentes.
