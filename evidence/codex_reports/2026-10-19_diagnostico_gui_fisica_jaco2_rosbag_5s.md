# Diagnóstico de interfaz física JACO2 — rosbag de 5 s

Fecha: 2026-10-19 (America/Lima)

## Evidencia

Se inició el stack físico con `mock_hardware=false`,
`hardware_output_enabled=true` y `use_demo_obstacle=true`. La captura está en
`evidence/rosbags/2026-10-19_jaco2_gui_diagnostico_5s/`.

El análisis usa la primera ventana de 4.988 s del rosbag, sin publicar ni
enviar movimiento.

## Resultado de estabilidad

- `/joint_states`: 100 muestras, aproximadamente 20 Hz.
- Las 12 articulaciones publicadas tuvieron rango de posición de `0.0 rad`;
  el brazo se mantuvo inmóvil durante toda la ventana.
- Diagnóstico final: conexión real activa, estado articular válido, cero fallos
  de lectura, realimentación de 20.003 Hz, `mode=hold`, sin objetivo ni jog
  activo y tasa de comandos físicos `0.0 Hz`.
- No hubo mensajes en `/thesis/candidate_command`,
  `/thesis/supervised_command`, `/thesis/command_decision` ni
  `/thesis/execution_trajectory`.

## Causa de que no haya movimiento

El adaptador informó `output_permitted=true` pero `armed=false` en las once
muestras de `/thesis/hardware/armed`. Por diseño, el adaptador rechaza salida
física si no está armado, aunque `hardware_output_enabled` esté activo. No es
un fallo de conexión, de SDK ni de estabilidad.

La GUI recibió `READY` en sus ocho muestras. El control de armado desde GUI
está habilitado (`allow_hardware_arm_control=true`). El texto rojo para
`JACO2 FÍSICO: CONECTADO Y ARMADO` es una advertencia visual deliberada, no un
error: rojo significa que la salida física ya podría ejecutar comandos
supervisados. Durante esta captura el estado real fue desarmado.

## Observación adicional

`/thesis/proximity_status` tuvo 87 muestras `ALLOW` y 13 `STOP` transitorios
por indisponibilidad de TF. El estado de disponibilidad muestreado fue
`READY`; aun así, un `STOP` transitorio bloquearía cualquier comando que
coincida con él. Conviene estabilizar la disponibilidad de TF antes de una
prueba de movimiento, aunque no es la causa del bloqueo actual: este es el
estado desarmado.

## Resolución propuesta

No se requiere modificar código. Con el sistema en `READY`, el operador debe
usar **Armar hardware** en la GUI y confirmar el diálogo de seguridad. Esa
acción no debe ejecutarse hasta contar con una confirmación explícita para
armar. Después, cada prueba de movimiento debe solicitar autorización propia,
empezando por un desplazamiento mínimo y velocidad reducida.
