# Etapa 1: volumen nominal continuo

Contiene GUI modificada, setup.py/package.xml de thesis_core y horizon_preview.py.
No cambia el supervisor ni el adaptador. Hacer respaldo de src antes de extraer.

Compilar: colcon build --symlink-install --packages-select thesis_core thesis_ui

## Abrir cuatro terminales

En cada terminal cargar /opt/ros/humble/setup.bash e install/setup.bash.

1. ros2 launch thesis_simulation jaco_gazebo.launch.py
2. ros2 launch thesis_simulation safety_pipeline.launch.py simulation_output_enabled:=false
3. ros2 run thesis_core horizon_preview
4. ros2 run thesis_ui joint_gui

RViz: Add > By topic > /thesis/horizon_volume > MarkerArray.
Fixed Frame: world; requiere TF a base_link del modelo.
GUI: activar ACTIVAR VOLUMEN CONTINUO y cambiar sliders/duración.
No hace falta ENVIAR COMANDO CANDIDATO para visualizar.
Al desactivar, desaparece tras 0.5 s más un ciclo. Marcadores expiran en 1 s.

Parámetros de arranque del predictor:
horizon_sec=1.0, samples=21, margin_m=0.02, rate_hz=10.0.
Ejemplo: ros2 run thesis_core horizon_preview --ros-args -p horizon_sec:=0.5
El texto en RViz muestra valores efectivos. La nota de GUI indica los defaults.

## Alcance

Azul significa ocupación nominal, NO ALLOW ni ausencia de colisión.
Todas las cápsulas futuras se aproximan mediante esferas solapadas. La inflación
espacial cubre cada cápsula muestreada, pero no certifica huecos temporales.
Margen 2 cm experimental, no calibrado. No incluye TTC, frenado, autocollision
ni evaluación de obstáculos. No activa movimiento continuo.

La duración se interpreta desde el estado actual hasta el objetivo en cada
ciclo. Velocidades limitadas a 18/24 grados/s según la GUI. No reproduce la
trayectoria activa del controlador ni mantiene su tiempo restante.
Mantener salida simulada desactivada para esta etapa. El botón de envío original
conserva su comportamiento. No es protección de seguridad durante movimiento.

Frecuencia: timer observado. Compute: cálculo antes de publicar, no latencia
extremo a extremo. Recepciones vencen a 0.5 s con reloj monotónico, pero no se
comprueba edad de adquisición por stamp. Usar Gazebo sin pausa para la prueba.

Sintaxis e interpolación verificadas sin ROS. Pendiente validación visual y
compilación en ROS 2 Humble del usuario. Próxima etapa: ejecución coherente con
predicción, reducción/parada, obstáculos y registro de métricas.
