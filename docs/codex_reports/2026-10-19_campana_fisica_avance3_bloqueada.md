# Campaña física mínima Avance 3 — bloqueada en Fase 0

## Veredicto

**BLOQUEADO.** No se ejecutó Fase 1, no se publicó ningún comando candidato y
no hubo movimiento autónomo de J1.

## Trazabilidad de la compuerta Fase 0

Antes de enviar intenciones, la observación del launch físico actual encontró:

| Campo | Valor observado | Requisito |
| --- | ---: | ---: |
| `armed` | `true` | `true` |
| `jog_active` | `true` | `false` |
| `mode` | `jog` | `hold` |
| salida física | 100.048 Hz | 0 Hz |
| entrada JOG | 20.010 Hz | intención cero |

Por lo tanto la compuerta falló y se aplicó inmediatamente el servicio
`/thesis/hardware/set_armed` con `data=false`. La verificación posterior fue:

| Campo | Valor tras contención |
| --- | ---: |
| `armed` | `false` |
| `jog_active` | `false` |
| `mode` | `hold` |
| salida física | 0 Hz |
| `/joint_states` | válido, velocidades 0 |
| proximidad | `ALLOW`, `world`, 1.811 m |

## Causa

El proceso `jaco_hardware_node` en ejecución inició a las 22:01:49. La
corrección de epoch de armado/HOLD se compiló a las 22:22:45; el proceso vivo
conservaba el ejecutable previamente cargado y la GUI mantenía el flujo JOG
heredado. El launch actual no fue reiniciado, por instrucción explícita de
mantenerlo, por lo que no era válido armar de nuevo para la Fase 0.

La corrección ya compilada requiere iniciar una nueva instancia para estar
efectiva: cancela JOG antes de armar, descarta el flujo previo, exige entrada
nueva y deja el adaptador armado en HOLD sin paquetes periódicos.

## Seguridad y alcance

No se modificaron parámetros de seguridad, interbloqueos ni umbrales. No hubo
rosbag de Fase 1 porque esa fase depende de aprobar Fase 0. El hardware quedó
desarmado.

## Siguiente paso concreto

Con autorización separada para detener y relanzar el stack físico con el
binario corregido, repetir únicamente la Fase 0 estática de 30 s. Si sus cinco
criterios pasan, solicitar confirmación antes de iniciar el rosbag y el primer
movimiento J1. Tras completar J1, el siguiente avance será integrar percepción
RGB-D, primero con obstáculos estáticos en simulación y después con las fases
físicas ALLOW/REDUCTION/STOP autorizadas.
