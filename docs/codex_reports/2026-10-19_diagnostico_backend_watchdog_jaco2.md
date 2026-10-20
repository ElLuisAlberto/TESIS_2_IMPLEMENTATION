# Diagnóstico de `hardware backend unavailable` y watchdog de JACO2

## Alcance

Diagnóstico y recuperación con el brazo sin armar. No se publicó ningún candidato durante la recuperación. La prueba anterior de J1 ya había alcanzado `-1.5518141135 rad` (10° desde `-1.7263443755 rad`) antes de abortar; por seguridad no se ordenó el retorno.

## Causa raíz

No hubo una pérdida del backend USB. El proceso `jaco_hardware_node` inició la API Kinova correctamente, seleccionó el único dispositivo y siguió publicando estado articular válido. Durante la ejecución de J1 hubo una ausencia de realimentación de 354.190 ms: el rosbag registra el último `/joint_states` a `1792468297.471654725` y el siguiente a `1792468297.825844676`. Esto excede `state_timeout_sec=0.20` s. A `1792468297.871100269`, el log del nodo registró exactamente `JACO STOP/HOLD: JACO joint state watchdog expired`.

La proximidad no fue la causa: mantuvo 728 mensajes, con hueco máximo de 94.028 ms y estado ALLOW. Tampoco fue una contención: tras cerrar ordenadamente el grupo del lanzamiento anterior, el bloqueo `/tmp/thesis_kinova_jaco2_usb.lock` quedó libre y la nueva sesión volvió a abrir USB/API con un solo dispositivo.

El diagnóstico genérico era una segunda deficiencia de observabilidad: la ruta del watchdog establecía `fault_=true` pero no asignaba `last_error_`; el publicador por tanto usaba el texto de reserva `hardware backend unavailable`, aun cuando `connected=true`, `api_open=true` y las lecturas ya se habían reanudado.

## Rutas que pueden fijar `fault=true`

En `jaco_hardware_node.cpp` son: fallo de envío físico (`handle_output_failure`), excepción al interpretar `ExecutionControl`, excepción de replanificación, fallo de `StartControlAPI` durante armado, máximo de lecturas de estado consecutivas fallidas y expiración de `state_is_fresh()` en el bucle de control. La única ruta observada en este incidente fue la última; no hay log de `StartControlAPI`, `GetAngularPosition` ni `SendBasicTrajectory` fallidos.

## Cambio mínimo aplicado

En la rama del watchdog se conserva `last_error_ = "JACO joint state watchdog expired"` antes de `stop_motion`. No se modificaron el umbral de 200 ms, la política de parada, el interbloqueo, el armado ni la respuesta a un estado stale. Así, una futura ocurrencia informa la causa real sin ocultar ni limpiar `fault`.

## Recuperación segura y resultado

Se envió SIGINT al grupo dueño del lanzamiento anterior, se confirmó que no quedaban procesos del adaptador ni dueño del lock USB y se inició una nueva sesión física con salida permitida pero desarmada. Tras 15 s, el diagnóstico fue: `connected=true`, `api_open=true`, `state_valid=true`, `fault=false`, `armed=false`, `jog_active=false`, `mode=hold`, `output_command_rate_hz_measured=0.000000`, realimentación 20.001 Hz y cero resets de control. El backend está disponible; `control_api_started=false` es el estado esperado mientras está desarmado.

## Validación de software

Se compiló únicamente `thesis_hardware_bridge` y se ejecutó `test_bridge_logic`: 20/20 pruebas aprobadas. `colcon test-result --all` también mostró un fallo preexistente y ajeno en `thesis_interfaces.xmllint` por no poder descargar el esquema ROS; el paquete modificado no tuvo fallos.

## Estado y siguiente paso

La Fase 1 permanece bloqueada por el hueco transitorio de feedback, no por un backend cerrado o bloqueado. El brazo permanece desarmado y en HOLD, sin salida física. El retorno de J1 y cualquier nuevo armado requieren autorización física nueva, porque el STOP ocurrió durante el tramo autorizado y el brazo quedó a +10°.
