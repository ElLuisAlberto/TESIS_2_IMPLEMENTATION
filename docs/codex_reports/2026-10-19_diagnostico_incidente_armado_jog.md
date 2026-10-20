# Diagnóstico y corrección: armado con JOG residual

## Alcance

No se armó ni se movió el JACO2 durante este trabajo. Al cierre, la observación
de `/thesis/hardware/armed` informó `false`.

## Cadena causal

La ruta confirmada fue:

`joint_control_gui.py` -> `/thesis/jog_intent` ->
`safety_supervisor_node.py` -> `/thesis/supervised_jog_command` ->
`jaco_hardware_node.cpp`.

La GUI mantenía el botón de JOG activo y su temporizador de 50 ms publicaba
`jog_continuous`, incluso cuando no había una nueva acción de rueda. El
supervisor conservaba el sello y el identificador y reenviaba la intención
validada. El adaptador, al completar el servicio de armado, solo fijaba
`armed_=true`; no reiniciaba `jog_active_` ni impedía el flujo que ya estaba
llegando. Su bucle de 100 Hz transmitía entonces la velocidad JOG. Además,
incluso en HOLD armado el bucle transmitía velocidades cero a 100 Hz.

## Corrección mínima

- La GUI cancela el JOG local antes de solicitar armar o desarmar. Activar el
  modo manual sin una rueda real no publica intención. Cada rueda real abre un
  identificador JOG nuevo; al cesar, se publica un único HOLD y se termina el
  flujo.
- El adaptador crea una época de armado: limpia JOG/objetivo, guarda la postura
  medida como referencia HOLD, bloquea el identificador que estaba presente
  antes de armar y rechaza sellos anteriores o iguales al instante de armado.
- Una intención supervisada de velocidad nula entra en HOLD, no en JOG activo.
  El adaptador conserva el pulso acotado de velocidad cero al entrar en
  control/HOLD, pero no transmite paquetes periódicos mientras está armado y
  ocioso.

No se modificaron umbrales, interbloqueos, STOP ni decisiones del supervisor.

## Archivos modificados

- `src/thesis_ui/thesis_ui/joint_control_gui.py`
- `src/thesis_ui/test/test_gui_state.py`
- `src/thesis_hardware_bridge/src/jaco_hardware_node.cpp`
- `src/thesis_hardware_bridge/include/thesis_hardware_bridge/bridge_logic.hpp`
- `src/thesis_hardware_bridge/test/test_bridge_logic.cpp`

## Validación sin hardware

Se compilaron únicamente `thesis_hardware_bridge` y `thesis_ui`.

- `thesis_hardware_bridge`: 19/19 pruebas correctas, incluida HOLD armado sin
  transmisión periódica.
- `thesis_ui`: 24 correctas, 1 omitida; se verificó que activar JOG sin rueda
  no publica y que la transición física deja HOLD con objetivo igual al estado
  medido.

La prueba estática física de 30 s permanece pendiente de una autorización
explícita nueva para armar. Esa prueba deberá usar el binario corregido y no
incluir ninguna entrada manual.
