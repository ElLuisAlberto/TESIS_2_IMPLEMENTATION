# Fase 1/J1: corrección de generador y bloqueo de backend

## Alcance

No se modificó el límite de antigüedad ni la política del supervisor. No se
armó ni movió el JACO2 durante esta corrección.

## Causa del rechazo `POINT_COMMAND_STAMP_STALE`

El comando se construía y sellaba en la shell antes de que `ros2 topic pub`
terminara de crear y descubrir su publicador. El supervisor lo recibió 1.147 s
después, por encima de su contrato de frescura. El supervisor lo rechazó
correctamente.

## Corrección aplicada

Se añadió `thesis_hardware.physical_candidate_generator`.

1. Espera que el publicador confirme una suscripción DDS del supervisor.
2. Solo después valida la entrada y construye el candidato.
3. Lee `node.get_clock().now()`, asigna el sello y llama `publish()` de forma
   consecutiva, sin servicio, diálogo, espera ni cálculo entre ambos pasos.

La lógica usa exclusivamente `/thesis/candidate_command`; no accede al USB ni
al adaptador físico directamente.

## Pruebas sin hardware

`colcon build --packages-select thesis_hardware --symlink-install`

`colcon test --packages-select thesis_hardware`

Resultado: 12 pruebas correctas, 1 omitida. Los nuevos casos verifican que el
contrato existente rechaza un sello antiguo y acepta un mensaje recién creado
por el generador. No se cambió el contrato del supervisor.

## Diagnóstico de backend, solo lectura

El adaptador actual informa persistentemente:

```text
armed=false
fault=true
message=hardware backend unavailable
connected=true
state_valid=true
system_ready=true
proximity_valid=true
read_failures=0
output_command_rate_hz_measured=0
```

La causa inmediata exacta no puede recuperarse de esta instancia: el
diagnóstico usa el texto genérico cuando `fault_` es verdadero y `last_error_`
está vacío. El código permite que ciertos fallos de watchdog marquen `fault_`
sin conservar el motivo en `last_error_`; la evidencia disponible no permite
atribuirlo de forma responsable a una condición concreta.

## Estado y siguiente paso

No se reintentó Fase 1 porque el requisito “diagnóstico sano” no se cumple.
Antes de una nueva campaña se necesita restaurar el backend a `fault=false` y
conservar la causa de cualquier fallo en el diagnóstico. Una vez sano, la
autorización física vigente permite repetir exclusivamente J1: +3°/1 s, HOLD
3 s, retorno y HOLD 5 s, usando el generador corregido y un rosbag nuevo.
