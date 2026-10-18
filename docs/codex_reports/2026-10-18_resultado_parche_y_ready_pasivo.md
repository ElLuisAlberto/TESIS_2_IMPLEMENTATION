# Resultado del parche Y y de la comprobación pasiva de READY

Fecha: 2026-10-18.

## Resultado

El parche mínimo quedó implementado y verificado sin movimientos. En el
`cartesian_command_adapter` del launch integrado se cambió únicamente
`position_tolerance_m` de `0.0002` a `0.0005`. No se modificó el solver ni
`SystemReadinessNode`.

Se añadieron dos regresiones para un objetivo de +20 mm en Y desde una muestra
determinista muy próxima a la postura inicial de Gazebo:

- con 0,2 mm, la IK falla con residuo de 0,366664811 mm;
- con 0,5 mm, la IK aprueba con residuo de 0,496866449 mm.

El rechazo existente de objetivos lejanos/inalcanzables continúa aprobando.

## Procedimiento READY

La prueba 7 del plan cartesiano ahora exige:

- tipo explícito `std_msgs/msg/String`;
- clientes de observación con `--no-daemon --spin-time 2`;
- plazo global máximo de 20 s;
- aprobación solo ante el texto exacto `data: READY`;
- conservación de `WAITING` como diagnóstico, sin tratarlo como aprobación.

No se relajaron dependencias, periodos ni timeouts de `SystemReadinessNode`.

## Compilación y pruebas

La compilación limitada a `thesis_core` y `thesis_simulation` terminó
correctamente. Las pruebas afectadas dieron:

| Paquete | Aprobadas | Omitidas | Resultado |
| --- | ---: | ---: | --- |
| `thesis_core` | 135 | 1 | Aprobado |
| `thesis_simulation` | 40 | 1 | Aprobado |

Las únicas advertencias fueron las ya conocidas sobre `SelectableGroups` y la
clase `TestCommandNode` no recopilable por tener `__init__`.

## Pila pasiva

Se inició una pila nueva con:

```text
simulation_output_enabled:=false
start_gui:=false
start_rviz:=false
headless:=true
```

El adaptador confirmó `salida simulada desactivada`. El monitor pasó de
`WAITING` durante el arranque a `SYSTEM READY`. La lectura con tipo explícito
obtuvo `data: READY` en el primer intento, a los 4 s de la ventana global de
20 s. Una consulta independiente obtuvo `/thesis/system_ready: true` y se
recibió un `/joint_states` completo.

No se publicaron intenciones cartesianas, comandos articulares ni comandos de
hardware. La pila se cerró por SIGINT y no quedaron procesos de simulación.

La evidencia detallada está en
[2026-10-18_parche_y_ready_pasivo.txt](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/evidence/codex_reports/2026-10-18_parche_y_ready_pasivo.txt).
