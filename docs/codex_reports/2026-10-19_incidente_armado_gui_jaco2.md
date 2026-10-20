# Incidente de armado desde GUI física del JACO2

Fecha: 2026-10-19 (America/Lima)

## Solicitud y límites

El operador autorizado solicitó armar el adaptador desde la GUI sin enviar
movimientos. Se confirmó el diálogo gráfico de armado. No se publicó una
trayectoria articular o cartesiana.

## Resultado observado

El armado fue aceptado: `hardware_output_enabled=true`, `armed=true`,
`output_permitted=true`, conexión, estado articular y proximidad válidos.
Sin embargo, inmediatamente el diagnóstico informó `jog_active=true`,
`mode=jog` y `output_command_rate_hz_measured` aproximadamente 100 Hz.

La GUI mostraba, antes del armado, el botón **DESACTIVAR CONTROL ARTICULAR
CONTINUO**, lo que confirma que el modo JOG continuo ya estaba habilitado.
Tras armar, las posiciones medidas visibles cambiaron (por ejemplo J2 de
208.85° a 203.65°, J3 de 110.17° a 120.37° y J4 de 279.07° a 290.17°). Por
ello no se cumplió el requisito de una salida armada con 0 Hz de comandos.

## Contención

Se ejecutó inmediatamente el servicio de desarmado/hold del adaptador. El
diagnóstico posterior confirmó:

- `armed=false`;
- `jog_active=false`;
- `mode=hold`;
- `output_command_rate_hz_measured=0.000000`;
- sin fallo, con conexión y estado articular válidos.

No se hizo la observación de 15 s solicitada porque el estado de salida cero
ya había fallado. No se modificó código durante este incidente.

## Causa y siguiente acción requerida

La causa inmediata es el modo de control articular continuo habilitado antes
de armar. El flujo de armado actual no bloquea ni exige desactivar ese modo
antes de habilitar la salida física. No es seguro rearmar hasta introducir y
validar un interbloqueo que rechace el armado cuando exista JOG continuo o
intención de movimiento activa, y que requiera una intención cero explícita.
