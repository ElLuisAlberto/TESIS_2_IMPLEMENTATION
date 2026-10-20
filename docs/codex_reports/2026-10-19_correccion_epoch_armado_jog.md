# Corrección de raíz: JOG residual al armar

## Hallazgo

La defensa previa basada en sello temporal e identificador observado tenía una
carrera: una intención JOG creada antes del servicio de armado podía estar en
la cola DDS y ser recibida por el adaptador después de que `armed` cambiara a
verdadero. Si aún no había sido observada, no existía identificador que
bloquear. La GUI anterior además podía publicar el ID fijo `jog_continuous` a
20 Hz.

La cadena afectada es:

`JointControlWindow` -> `/thesis/jog_intent` ->
`SafetySupervisorNode.publish_supervised_jog` ->
`/thesis/supervised_jog_command` -> `JacoHardwareNode.jog_callback`.

El supervisor conserva correctamente `command_id` y `stamp`; no se alteró su
política preventiva.

## Corrección

El adaptador publica el tópico latched `/thesis/hardware/arm_epoch` y genera
un token nuevo (`arm-N`) en cada transición válida a armado. Solo admite un
JOG cuyo ID tenga la forma `jog:arm-N:<input-nuevo>`. Por tanto cualquier
mensaje prearmado, incluido el ID legado, se rechaza sin depender de orden,
latencia o temporizadores.

La GUI:

- inicia manual continuo desactivado;
- mientras el hardware está desarmado o sin época válida, mantiene JOG inerte;
- limpia JOG antes de solicitar armado o desarmado;
- crea el ID de la época actual únicamente al recibir una rueda posterior al
  armado.

El adaptador continúa limpiando objetivo, velocidad, temporizadores y estado
JOG al armar, guarda HOLD en la postura medida y no transmite a 100 Hz cuando
está armado pero ocioso.

## Archivos modificados

- `src/thesis_hardware_bridge/src/jaco_hardware_node.cpp`
- `src/thesis_hardware_bridge/include/thesis_hardware_bridge/bridge_logic.hpp`
- `src/thesis_hardware_bridge/test/test_bridge_logic.cpp`
- `src/thesis_ui/thesis_ui/joint_control_gui.py`
- `src/thesis_ui/test/test_gui_state.py`

## Validación sin hardware

| Paquete | Resultado |
| --- | --- |
| `thesis_hardware_bridge` | 20/20 pruebas correctas |
| `thesis_ui` | 25 correctas, 1 omitida |

La cobertura añadida verifica: rechazo del ID legado y de una época previa,
aceptación exclusiva de la época actual, JOG físico inerte antes del armado y
emisión solo tras input nuevo. No se realizó prueba física ni se armó el JACO2.
