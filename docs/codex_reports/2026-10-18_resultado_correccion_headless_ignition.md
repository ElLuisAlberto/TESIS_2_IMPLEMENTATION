# Resultado de la corrección headless de Ignition

Se corrigió el inicio gráfico no deseado de Ignition mediante un argumento de lanzamiento `headless`, con valor predeterminado `false`. El cambio afecta únicamente a [`jaco_gazebo.launch.py`](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/src/thesis_simulation/launch/jaco_gazebo.launch.py) y [`thesis_system.launch.py`](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/src/thesis_simulation/launch/thesis_system.launch.py).

Cuando se usa `headless:=true`, el launch inicia `ign gazebo -r -s -v 3 <world>`. La opción `-s` inicia solo el servidor de Ignition y evita su cliente gráfico. Si no se especifica el nuevo argumento, permanece el comando previo `ign gazebo -r -v 3 <world>`, por lo que el comportamiento predeterminado no cambia.

La compilación de `thesis_simulation` terminó correctamente. Sus pruebas unitarias dieron 40 aprobadas y 1 omitida; no hubo fallos. Los avisos fueron preexistentes de recolección de una clase de prueba y deprecación de una dependencia de pytest.

La prueba pasiva se ejecutó con:

```bash
ros2 launch thesis_simulation thesis_system.launch.py \
  simulation_output_enabled:=false start_gui:=false start_rviz:=false \
  use_demo_obstacle:=false headless:=true
```

El árbol de procesos confirmó `ign gazebo -r -s ...` y no mostró `ign gui` ni `gz gui`. La pila ROS 2 quedó disponible, incluidos los tres adaptadores cartesianos, `safety_supervisor_node` y `/joint_states`; este último tuvo un publicador y emitió una muestra válida. Al finalizar, no quedó ningún proceso de launch, Ignition, GUI, supervisor, adaptador ni controlador de simulación.

No se modificó lógica de seguridad ni control cartesiano. No se publicaron comandos ROS, no se ejecutaron movimientos, `simulation_output_enabled` permaneció desactivado y no se utilizó hardware físico, cámara RGB-D ni `sudo`.

La evidencia completa está en [2026-10-18_prueba_headless_ignition.txt](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/evidence/codex_reports/2026-10-18_prueba_headless_ignition.txt).
