# Diagnóstico de `controller_manager` en READY headless

Fecha: 2026-10-18.

## Causa confirmada

`controller_manager` no estuvo ausente. `gz_ros2_control` lo crea dentro del
proceso `ign`; en la ejecución fallida cargó y activó
`joint_state_broadcaster` y `arm_controller`.

El fallo fue una carrera en la primera consulta de `system_readiness`:

1. El servicio `/controller_manager/list_controllers` se descubrió durante el
   arranque.
2. `system_readiness` creó una llamada asíncrona.
3. El servidor registró `failed to send response ... (timeout): client will
   not receive response`.
4. Ese future nunca terminó. `_update_controller_state()` retorna mientras
   exista un future pendiente y no tiene timeout ni reintento.
5. El texto quedó congelado en `esperando controller_manager`, aunque los
   controladores y el action server ya estaban activos.

Por tanto, el mensaje READY fallido describe el estado interno obsoleto del
cliente, no la ausencia del manager.

## Comparación y reproducción

- Ejecución fallida: timeout de respuesta a `list_controllers`, ambos
  controladores activados y WAITING permanente.
- Evidencia anterior aprobada: no hubo timeout y READY llegó después de activar
  `arm_controller`.
- Única reproducción nueva: `list_controllers` respondió, el estado cambió a
  `arm_controller no está active`, ambos controladores se activaron y READY
  llegó a los aproximadamente 7 s desde el inicio de `system_readiness`.

El launch usa temporizadores de 5 s y 6 s para los dos spawners. Estos esperan
el servicio y pueden completar en distinto orden; esa variación es normal. La
diferencia determinante es si la primera respuesta consumida por readiness se
entrega o queda huérfana.

## Parche mínimo propuesto

Añadir un plazo monotónico al `controller_future`. Si no termina dentro del
plazo, cancelarlo, limpiarlo y permitir que una evaluación posterior vuelva a
llamar `list_controllers`. Acompañarlo con una regresión donde la primera
llamada nunca completa y la segunda devuelve `arm_controller=active`.

No se recomienda retrasar el launch: un `TimerAction` mayor solo reduce la
probabilidad y no elimina la carrera.

Autorizar el parche si la prueba garantiza un único request pendiente, el
reintento recupera READY y una pila nueva llega a READY en 20 s aun perdiendo
la primera respuesta.

No se modificó código ni se publicaron comandos. Evidencia completa:
[2026-10-18_diagnostico_controller_manager_ready_headless.txt](../../evidence/codex_reports/2026-10-18_diagnostico_controller_manager_ready_headless.txt).

