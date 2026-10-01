# Demostraciones visuales de horizonte corto

Estos estímulos generan objetivos articulares por `/thesis/candidate_command`,
el mismo flujo preventivo que usa la interfaz. Cada objetivo produce una
decisión y una predicción asociadas a su `command_id`; el script imprime el
estado, la distancia mínima prevista, el segmento limitante, la posición de la
muestra crítica y los tiempos a los eventos protectores. RViz2 permite ver el
volumen previsto en `/thesis/horizon_volume`.

## Antes de ejecutar

En cuatro terminales, desde el workspace de la tesis y con ROS 2 Humble:

1. Inicia Gazebo, RViz2 y el obstáculo:

   ```bash
   ros2 launch thesis_simulation jaco_gazebo.launch.py
   ```

2. Inicia la cadena preventiva con salida habilitada solo hacia el controlador
   de Gazebo:

   ```bash
   ros2 launch thesis_simulation safety_pipeline.launch.py \
     simulation_output_enabled:=true \
     require_proximity_status:=true \
     use_sim_time:=true
   ```

3. Inicia la visualización predictiva:

   ```bash
   ros2 run thesis_core horizon_preview --ros-args \
     -p use_sim_time:=true -p source:=auto \
     -p horizon_sec:=1.0 -p samples:=21 -p margin_m:=0.02 \
     -p rate_hz:=10.0
   ```

4. Opcionalmente abre la GUI para mostrar en paralelo sus indicadores:

   ```bash
   ros2 run thesis_ui joint_gui
   ```

Espera a que el controlador esté activo y que la articulación inicial coincida
con la pose de inicio de la GUI: `[0, 180, 180, 0, 0, 0]` grados. No actives el
modo manual mientras corre el estímulo.

## Ejecutar

En otra terminal, con `source /opt/ros/humble/setup.bash` y
`source install/setup.bash`:

```bash
python3 tools/demos/horizon_motion_demo.py --scenario all
```

Escenarios individuales: `pick_place`, `barrido` y `elevacion`. Cada destino
combina varias articulaciones para que cambie visiblemente la orientación y la
envolvente del brazo. El guion muestra los seis ángulos objetivo en grados.
Para ejecutar solo uno:

```bash
python3 tools/demos/horizon_motion_demo.py --scenario barrido
```

La duración predeterminada de cada objetivo es 2 s. Con el horizonte habitual
de 1 s, la visualización representa aproximadamente la primera mitad del
movimiento (sujeto a los límites de velocidad). Se puede cambiar con
`--duration 2.0`. El horizonte de predicción sigue siendo el parámetro del
supervisor; el script no lo modifica.

## Cinco secuencias articulares adicionales

El paquete `horizon_5_scripts.zip` incluye cinco guiones independientes. Cada
uno mueve varias articulaciones en el mismo comando y vuelve a HOME entre
objetivos para que cada recorrido empiece desde una configuración conocida:

```bash
python3 tools/demos/demo_01_coordinated_sweep.py
python3 tools/demos/demo_02_wrist_reorientation.py
python3 tools/demos/demo_03_elbow_levels.py
python3 tools/demos/demo_04_mirrored_transfer.py
python3 tools/demos/demo_05_compound_sequence.py
```

Estos cinco guiones usan 4 s por objetivo para que los cambios de varias
articulaciones sean visibles y la animación cubra el primer segundo previsto.
Parten de HOME `[0, 180, 180, 0, 0, 0]` grados. Con el obstáculo en su posición
predeterminada `(0.60, 0.0, 0.65) m`, la comprobación del modelo de cápsulas
estima un despeje mínimo de entre 0.365 m y 0.395 m en los recorridos
programados. Esto sirve como referencia para esa configuración; si el robot no
está en HOME o cambias la ubicación del obstáculo, revisa la predicción en RViz
y el resultado que informa el supervisor antes de continuar.

## Qué mostrar y cómo interpretarlo

- **Pick and place ilustrativo:** aproximación, descenso, elevación, traslado y
  retorno como configuraciones articulares. Sirve para observar la evolución
  prevista entre movimientos consecutivos.
- **Barrido:** cambios amplios de base hacia ambos lados, útiles para comparar
  cómo cambia la envolvente geométrica según la dirección.
- **Elevación:** movimientos de hombro y codo para contrastar qué eslabón limita
  la distancia prevista.

La animación muestra las muestras geométricas estimadas dentro del horizonte;
no es una simulación de objetos recogidos. Los escenarios no son trayectorias
cartesianas planificadas ni accionan un gripper. El resultado (ALLOW, WARNING,
REDUCTION o STOP) depende de la pose medida, la ubicación del obstáculo y los
parámetros actuales; no se debe presentar un estado esperado como garantizado.
Para una demostración comparativa, conserva la misma pose inicial y ubicación
del obstáculo en cada repetición.
