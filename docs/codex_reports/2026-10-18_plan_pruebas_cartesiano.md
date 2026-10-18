# Plan de pruebas cartesianas en simulación

Fecha: 2026-10-18

## Límites del plan

Este plan no contempla `hardware_output`, `sudo`, nodos de hardware, ni publicaciones manuales en tópicos de mando. Las pruebas de ejecución son de observación pasiva y deben iniciar la pila solamente con `simulation_output_enabled:=false`, sin GUI ni RViz. No ejecutar `ros2 topic pub`, los validadores que emitan objetivos, ni controles de la GUI durante estas pruebas.

## Preparación común

Ejecutar desde la raíz del espacio de trabajo:

```bash
colcon build --packages-select thesis_interfaces thesis_core thesis_simulation thesis_ui
source install/setup.bash
```

## Regla de observación ROS 2

Toda consulta de nodos o tópicos debe usar `--no-daemon --spin-time 2`. Ejecutar los clientes de observación uno por uno y esperar su salida. `tf2_echo` no acepta esas opciones porque es una herramienta independiente; debe detenerse sin afectar al launch.

## Prueba 1: interfaces y dependencias cartesianas

- **Objetivo:** confirmar que los mensajes cartesianos y sus dependencias se generan antes de probar la lógica.
- **Comando:**
  ```bash
  colcon test --packages-select thesis_interfaces thesis_core thesis_simulation thesis_ui
  colcon test-result --verbose
  ```
- **Tópico observado:** no aplica; es una prueba de construcción y pruebas registradas.
- **Resultado esperado:** compilación de los cuatro paquetes y resultado de pruebas sin fallos de importación de `CartesianCommand` o `CartesianTrajectoryCommand`.
- **Aprobación / fallo:** aprueba si `colcon test-result --verbose` no informa fallos. Falla si hay errores de generación de mensajes, importación o descubrimiento de paquetes; se detiene el plan y se registra el error.

## Prueba 2: IK local, alineación TF y límites

- **Objetivo:** aislar si la conversión de X/Y/Z falla antes de ROS por IK, límites articulares o compensación entre el modelo y TF.
- **Comando:**
  ```bash
  colcon test --packages-select thesis_core --pytest-args -q
  colcon test-result --verbose
  ```
- **Tópico observado:** no aplica; usa funciones puras de cinemática.
- **Resultado esperado:** pasan los casos de convergencia para desplazamientos cercanos, compensación de offset TF, tolerancia y rechazo de objetivos inalcanzables.
- **Aprobación / fallo:** aprueba si `test_cartesian_kinematics.py` y `test_cartesian_path.py` pasan. Falla si converge de forma incorrecta, si falla la compensación TF o si no se rechaza un objetivo inalcanzable; el foco queda en `cartesian_kinematics.py`, `jaco_kinematics.py` o `rigid_transform.py`.

## Prueba 3: modelos puros de JOG y trayectoria

- **Objetivo:** verificar X/Y/Z en marco `world`, los límites del twist y el progreso condicionado por el supervisor sin publicar un comando ROS.
- **Comando:**
  ```bash
  colcon test --packages-select thesis_simulation --pytest-args -q
  colcon test-result --verbose
  ```
- **Tópico observado:** no aplica; usa `cartesian_jog_model.py` y `cartesian_trajectory_model.py` directamente.
- **Resultado esperado:** X/Y/Z se integran con el signo y desplazamiento correctos, se rechazan velocidades fuera de límite, la IK local soporta cada eje desde la postura recomendada y una trayectoria no avanza con `speed_scale=0`.
- **Aprobación / fallo:** aprueba si pasan `test_cartesian_jog_model.py` y `test_cartesian_trajectory_model.py`. Falla si un eje XYZ no integra correctamente, si el límite no se aplica o si el progreso avanza estando detenido; el foco queda en los modelos o en sus parámetros del lanzador.

## Prueba 4: precondiciones y envío desde la GUI

- **Objetivo:** confirmar que la interfaz genera los tipos de intención correctos y bloquea los modos incompatibles sin lanzar la GUI real.
- **Comando:**
  ```bash
  colcon test --packages-select thesis_ui --pytest-args -q
  colcon test-result --verbose
  ```
- **Tópico observado:** no aplica; las pruebas sustituyen los publicadores por objetos simulados.
- **Resultado esperado:** la prueba verifica que el JOG cartesiano produce velocidades acotadas en `world`, que existen los controles X/Y/Z y que una trayectoria bloquea otros modos.
- **Aprobación / fallo:** aprueba si pasa `test_gui_state.py`. Falla si el JOG produce una velocidad no acotada, si la GUI habilita un comando sin precondiciones o si permite modos simultáneos; el foco queda en `joint_control_gui.py`.

## Prueba 5: presencia pasiva de nodos y tópicos cartesianos

- **Objetivo:** determinar si el problema es que un adaptador no está levantado o no tiene conexiones, sin habilitar salida de movimiento.
- **Comando:** en una primera terminal:
  ```bash
  ros2 launch thesis_simulation thesis_system.launch.py simulation_output_enabled:=false start_gui:=false start_rviz:=false use_demo_obstacle:=false
  ```
  En una segunda terminal, tras cargar `install/setup.bash`:
  ```bash
  ros2 node list --no-daemon --spin-time 2
  ros2 topic info --no-daemon --spin-time 2 -v /thesis/cartesian_candidate
  ros2 topic info --no-daemon --spin-time 2 -v /thesis/cartesian_jog_intent
  ros2 topic info --no-daemon --spin-time 2 -v /thesis/cartesian_trajectory_candidate
  ros2 topic info --no-daemon --spin-time 2 -v /thesis/candidate_command
  ros2 topic info --no-daemon --spin-time 2 -v /thesis/jog_intent
  ```
- **Tópico observado:** `/thesis/cartesian_candidate`, `/thesis/cartesian_jog_intent`, `/thesis/cartesian_trajectory_candidate`, `/thesis/candidate_command` y `/thesis/jog_intent`.
- **Resultado esperado:** aparecen `cartesian_command_adapter`, `cartesian_jog_adapter` y `cartesian_trajectory_adapter`; cada tópico de entrada tiene su suscriptor y las salidas tienen al supervisor como receptor.
- **Aprobación / fallo:** aprueba si todos los nodos y suscriptores están presentes. Falla si falta un adaptador o una conexión; el problema se localiza en `thesis_system.launch.py`, `setup.py` o la instalación del paquete, antes de IK.

## Prueba 6: estado articular y referencia TF pasivos

- **Objetivo:** verificar el prerrequisito común que diferencia la ruta cartesiana: estado articular completo y TF vivo en el mismo sistema de simulación.
- **Comando:** con la pila pasiva de la prueba 5 en ejecución:
  ```bash
  ros2 topic echo --once --no-daemon --spin-time 2 /joint_states
  ros2 run tf2_ros tf2_echo world j2n6s300_end_effector
  ```
  Detener `tf2_echo` después de observar varias actualizaciones, sin publicar mensajes.
- **Tópico observado:** `/joint_states` y TF `world -> j2n6s300_end_effector`.
- **Resultado esperado:** `/joint_states` contiene las seis articulaciones esperadas y TF se actualiza sin errores de transformación.
- **Aprobación / fallo:** aprueba si ambos flujos existen y son recientes. Falla si falta una articulación, TF no existe o se informa extrapolación; el foco queda en Gazebo, `robot_state_publisher`, puente TF o sincronización, antes de los adaptadores.

## Prueba 7: supervisión pasiva y causas de pausa

- **Objetivo:** comprobar que la cadena preventiva está disponible para recibir las futuras salidas cartesianas, sin crear una intención de movimiento.
- **Comando:** con la pila pasiva de la prueba 5, verificar `READY` con tipo
  explícito y un plazo global máximo de 20 s. Cada invocación termina antes de
  iniciar la siguiente y conserva tanto `READY` como los diagnósticos
  `WAITING`:
  ```bash
  ros2 topic info --no-daemon --spin-time 2 -v /thesis/supervised_command
  ros2 topic info --no-daemon --spin-time 2 -v /thesis/supervised_jog_command
  ros2 topic info --no-daemon --spin-time 2 -v /thesis/execution_control

  deadline=$((SECONDS + 20))
  ready=false
  while (( SECONDS < deadline )); do
    remaining=$((deadline - SECONDS))
    output=$(timeout "${remaining}s" ros2 topic echo --once \
      --no-daemon --spin-time 2 \
      /thesis/system_readiness std_msgs/msg/String 2>&1)
    rc=$?
    printf '%s\n' "$output"
    if (( rc == 0 )) && printf '%s\n' "$output" | grep -qx 'data: READY'; then
      ready=true
      break
    fi
  done
  test "$ready" = true
  ```
- **Tópico observado:** `/thesis/supervised_command`, `/thesis/supervised_jog_command`, `/thesis/execution_control` y `/thesis/system_readiness`.
- **Resultado esperado:** los dos tópicos supervisados tienen productores del
  supervisor y consumidores de salida de simulación; `system_readiness` emite
  el texto exacto `READY` antes del plazo. Es normal que no exista un mensaje
  de `execution_control` sin intención activa.
- **Aprobación / fallo:** aprueba únicamente con `data: READY`. Un mensaje
  `WAITING` se conserva como diagnóstico pero no aprueba. Falla si vence el
  plazo, no llega ningún mensaje, falta el publicador, persiste `WAITING` o
  termina el launch. En caso de fallo se registran la última causa y las
  dependencias ausentes; no se oculta el fallo ampliando tolerancias de
  disponibilidad.

## Interpretación

Si las pruebas 1 a 4 pasan y las pruebas 5 a 7 muestran nodos, estado y TF correctos, la causa restante requiere capturar una única intención cartesiana en simulación con salida aún desactivada. Esa captura implicaría publicar desde la GUI o un publicador de prueba y no forma parte de este plan sin una autorización posterior.
