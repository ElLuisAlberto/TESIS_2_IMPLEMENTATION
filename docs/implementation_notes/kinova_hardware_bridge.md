# Adaptador físico Kinova JACO2

## Alcance

La integración física usa directamente la API USB incluida en
`kinova-ros/kinova_driver`. No ejecuta ROS 1, `roscore` ni
`ros1_bridge`. `kinova-ros` se conserva como dependencia externa porque
aporta los headers y la biblioteca binaria del fabricante.

El adaptador recibe exclusivamente las salidas supervisadas:

- `/thesis/supervised_command`;
- `/thesis/supervised_jog_command`;
- `/thesis/execution_control`.

Publica `/joint_states`, `/thesis/execution_trajectory`,
`/thesis/pipeline_timing`, `/thesis/hardware/diagnostics`,
`/thesis/hardware/connected` y `/thesis/hardware/armed`.

## Barreras de activación

El movimiento requiere las dos condiciones siguientes:

1. arrancar con `hardware_output_enabled:=true`;
2. armar en tiempo de ejecución mediante
   `/thesis/hardware/set_armed`.

Los valores predeterminados mantienen el brazo en solo lectura. El adaptador
rechaza comandos directos que no respeten los seis nombres canónicos, límites
articulares o velocidades operacionales de la tesis.

Existe un único ejecutable operativo, `jaco_hardware_node`. El mismo binario
ofrece lectura cuando `hardware_output_enabled:=false` y movimiento con doble
habilitación cuando el parámetro es `true`; se retiró el lector USB paralelo
para impedir que dos procesos del repositorio compitan por la API del brazo.
Además, cada instancia física toma un bloqueo no bloqueante en
`/tmp/thesis_kinova_jaco2_usb.lock` antes de llamar `InitAPI`; una segunda
instancia del adaptador falla cerrada. El bloqueo no sustituye la obligación de
cerrar software externo que acceda al mismo SDK.

La interfaz recibe el parámetro `operation_mode:=hardware`, muestra de forma
explícita la conexión USB y el estado de armado, y solicita confirmación antes
de llamar al servicio. Al cerrarse intenta desarmar el adaptador antes de
destruir su nodo ROS 2. La interfaz de simulación conserva
`operation_mode:=simulation` y no crea controles de armado físico.

## Respuesta fail-safe

El bucle de control y envío trabaja en un hilo periódico dedicado a 100 Hz,
frecuencia requerida por el DSP para comandos de velocidad. El hilo conserva
su reloj absoluto y recupera en el ciclo siguiente un retraso corto producido
por una transacción USB, pero descarta retrasos superiores a cinco periodos
para impedir ráfagas prolongadas. La posición USB se adquiere a 20 Hz por
defecto y la velocidad se estima mediante diferencias articulares con
tratamiento circular. El esfuerzo se deja sin publicar porque no interviene en
la supervisión preventiva. Un mutex recursivo serializa todas las llamadas al
SDK del fabricante y otro protege el estado compartido entre ROS 2 y el hilo
de control. El diagnóstico distingue la frecuencia del hilo, la frecuencia de
feedback y la frecuencia de comandos realmente enviados, además de contar las
resincronizaciones del reloj. El adaptador envía velocidad cero y limpia la
cola Kinova cuando ocurre cualquiera de estos eventos:

- decisión `STOP`;
- vencimiento del flujo JOG;
- vencimiento de `/thesis/execution_control`;
- pérdida repetida del estado articular;
- error de transmisión USB;
- desarmado o cierre del nodo.

Una referencia articular recién aceptada permanece inicialmente con velocidad
cero. El adaptador comienza a seguirla solo después de recibir la primera
muestra correlacionada de `/thesis/execution_control`; si esta no llega dentro
del watchdog configurado, cancela la referencia sin iniciar el movimiento.

`REDUCTION` vuelve a fijar la referencia desde el estado medido, actualiza la
duración restante y publica una nueva referencia `ACCEPTED` para que el
supervisor evalúe el horizonte reducido.

Durante el arranque, una lectura USB aislada puede fallar mientras la API y el
dispositivo terminan de estabilizarse. El adaptador permite por defecto diez
intentos de lectura separados 200 ms y registra cada fallo. La inicialización
solo continúa después de una muestra articular válida; si se agotan los
intentos, el proceso termina sin habilitar la salida física. Los parámetros
`initialization_read_attempts` e `initialization_retry_delay_ms` permiten
ajustar esta ventana sin debilitar la condición de estado válido.

Algunos firmwares no rellenan `KinovaDevice.SerialNumber`. En ese caso solo se
permite la selección implícita cuando existe exactamente un dispositivo y el
diagnóstico usa el identificador explícito `KINOVA_USB_DEVICE_0`, junto con
`serial_reported=false`, el número de dispositivos y el índice elegido. Si se
configura `expected_serial_number`, la coincidencia exacta sigue siendo
obligatoria y el arranque falla si no puede demostrarse.

## Compilación

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  thesis_interfaces \
  thesis_description \
  thesis_core \
  thesis_simulation \
  thesis_ui \
  thesis_hardware_bridge \
  thesis_hardware \
  --cmake-args \
    -DKINOVA_ROOT="$HOME/Escritorio/TESIS_2_DEPENDENCIES/kinova-ros/kinova_driver"
```

## Validación sin hardware

```bash
source install/setup.bash

ros2 launch thesis_hardware jaco_physical_system.launch.py \
  mock_hardware:=true \
  hardware_output_enabled:=true \
  use_demo_obstacle:=true \
  start_gui:=false \
  start_rviz:=false
```

En otra terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/Escritorio/TESIS_2_IMPLEMENTATION/install/setup.bash

ros2 service call /thesis/hardware/set_armed \
  std_srvs/srv/SetBool "{data: true}"
```

La validación automática del backend mock se ejecuta con:

```bash
bash tools/validation/run_hardware_bridge_mock.sh
```

## Primera conexión física

La primera sesión se realiza con `hardware_output_enabled:=false`. Esta fase
solo confirma enumeración USB, identidad disponible, frecuencia y coherencia de
`/joint_states`, diagnóstico, TF y RViz. No debe ejecutarse simultáneamente
ROS 1 ni otro proceso que abra la API Kinova.

El movimiento físico se habilita en una fase posterior, después de verificar
la correspondencia entre los seis ángulos medidos y la postura real, la fuente
de proximidad, el pulsador físico de parada y el espacio libre de trabajo.

El sistema físico completo se inicia desde un launch distinto al de Gazebo:

```bash
ros2 launch thesis_hardware jaco_physical_system.launch.py \
  mock_hardware:=false \
  hardware_output_enabled:=false \
  use_demo_obstacle:=false \
  start_gui:=true \
  start_rviz:=true
```

`hardware_config_file`, `capsule_config_file` y `obstacle_input_topic` son
argumentos del launch. Permiten mantener perfiles alternativos de comunicación,
geometría o percepción sin modificar código ni mezclar configuración física
con la de Gazebo.

Una vez aprobadas la lectura física, la prueba HOLD y la validación mock, el
micromovimiento inicial se ejecuta desde una terminal nueva:

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
bash tools/validation/run_kinova_physical_micro_motion.sh
```

El ensayo solicita literalmente `MOVER J6`, usa un obstáculo sintético lejano,
arma el adaptador, ordena J6 +2° durante 4 s, vuelve a la postura medida y
desarma. Su aceptación exige `ACCEPTED` y `SUCCEEDED` correlacionados, decisiones
runtime sin `STOP`, desplazamiento acotado, retorno con error máximo de 0.6°,
ausencia de movimiento relevante en los otros ejes y diagnóstico sin fallo.
El obstáculo sintético pertenece exclusivamente a esta prueba controlada; la
operación normal debe usar la fuente perceptual real.

La configuración geométrica y su visualizador se alojan en `thesis_core`
porque son comunes a simulación y hardware. `thesis_simulation` conserva solo
Gazebo, sus controladores y el adaptador del controlador simulado.
