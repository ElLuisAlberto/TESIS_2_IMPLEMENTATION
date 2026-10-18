# Diagnóstico de `RuntimeError: !rclpy.ok()` en descubrimiento ROS 2

## Reproducción

La falla se reprodujo con una pila nueva, headless y pasiva: `simulation_output_enabled:=false`, `start_gui:=false`, `start_rviz:=false` y `headless:=true`. No hubo publicación de comandos ni movimiento.

`ros2 daemon status` respondió `The daemon is running` con código 0. Inmediatamente después, los comandos normales `ros2 node list`, `ros2 topic info -v /joint_states` y `ros2 topic info -v /thesis/system_readiness` devolvieron código 1 con:

```text
xmlrpc.client.Fault: <Fault 1: "<class 'RuntimeError'>:!rclpy.ok()">
```

La traza llega desde `xmlrpc.client`, por la conexión del cliente `ros2` con el daemon, antes de obtener información de los nodos de tesis.

## Causa probable y ámbito

La causa probable es un daemon de `ros2cli` que conserva un servidor XML-RPC accesible pero cuyo contexto `rclpy` ya no está activo. La comprobación de estado solo confirma la conectividad del servidor, por eso indica que está en ejecución aunque no pueda responder a operaciones de grafo ROS.

No depende de la prueba X: se reprodujo desde una pila nueva con salida de simulación desactivada y sin intención cartesiana. Tampoco hay evidencia de fallo en el launch: los nodos de simulación permanecieron disponibles y las consultas directas descubrieron sus tópicos y estados.

El problema pertenece a la herramienta de descubrimiento `ros2cli` y deja al procedimiento de prueba vulnerable cuando usa el daemon por defecto. Los comandos `timeout` de la sesión anterior coincidieron temporalmente con la primera aparición, pero esta prueba no demuestra que fueran la causa.

## Comparación directa

ROS 2 Humble instala `--no-daemon` para evitar el daemon por comando. Con ese argumento y `--spin-time 2`:

- `ros2 node list` aprobó tres repeticiones consecutivas.
- `ros2 topic info -v /joint_states` informó un publicador y ocho suscriptores.
- `ros2 topic echo --once /thesis/system_readiness` obtuvo `READY`.
- `/joint_states` y `/thesis/proximity_status` entregaron muestras válidas.

La variable `ROS2CLI_DISABLE_DAEMON=1` no evitó el daemon en esta instalación; el argumento explícito `--no-daemon` sí lo hizo.

## Corrección mínima propuesta

No requiere cambiar código ni launch. Ajustar el procedimiento de prueba para todas las consultas de grafo y observación:

```bash
ros2 node list --no-daemon --spin-time 2
ros2 topic info --no-daemon --spin-time 2 -v /joint_states
ros2 topic echo --once --no-daemon --spin-time 2 /thesis/system_readiness
```

Usar clientes de observación individuales y esperar su terminación antes de iniciar el siguiente. Para TF, detener solo el cliente de TF, sin enviar señales al launch de simulación. Reiniciar el daemon es una posible recuperación operativa, pero no fue realizado ni es necesario para esta corrección de procedimiento.

La simulación pasiva se detuvo al finalizar y no quedaron procesos. La evidencia completa está en [2026-10-18_diagnostico_rclpy_descubrimiento.txt](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/evidence/codex_reports/2026-10-18_diagnostico_rclpy_descubrimiento.txt).
