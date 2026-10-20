# Diagnóstico y estabilización de TF del JACO2 físico

Fecha: 2026-10-19 (America/Lima)

## Alcance y protección física

La sesión usó `mock_hardware=false`, `hardware_output_enabled=true` y el
obstáculo sintético ya autorizado. El adaptador permaneció `armed=false`; no
se publicaron comandos, no se alteraron umbrales ni políticas de proximidad y
no hubo movimiento.

## Causa raíz

El receptor es `proximity_monitor`. Para preservar seguridad temporal,
consulta cada segmento como `world -> <frame del segmento>` al sello exacto
del obstáculo. Si no existe una transformación que abarque ese sello dentro
del límite de 40 ms, conserva el comportamiento fail-closed y publica
`STOP/TF_UNAVAILABLE`.

En la captura de régimen previa, el emisor de las transformaciones dinámicas,
`robot_state_publisher`, recibió `joint_states` a 20 Hz pero publicó `/tf`
mayoritariamente cada 100 ms. Su límite efectivo `publish_frequency=20 Hz`
descartaba muestras alternas por el jitter normal de un período cercano a 50
ms. Los 116 STOP correlacionados compartieron el mismo patrón:

- el sello de obstáculo quedaba entre 18.088 y 18.770 ms por delante del
  último TF dinámico disponible;
- el último TF había llegado 67.665 a 76.435 ms antes de que se registrara el
  STOP;
- faltaba la cadena dinámica publicada por `robot_state_publisher`:
  `j2n6s300_link_base -> j2n6s300_link_1 -> ... ->
  j2n6s300_link_6` (incluidos los frames usados por los seis segmentos).
  Los enlaces estáticos `world -> root -> j2n6s300_link_base` y
  `j2n6s300_link_6 -> j2n6s300_end_effector` sí estaban presentes en
  `/tf_static`.

No fue un fenómeno exclusivo de arranque: ocurrió durante los 30.5 s de
régimen.

## Corrección mínima

Se modificó `src/thesis_hardware/launch/jaco_physical_system.launch.py`:
`robot_state_publisher.publish_frequency` pasó de su valor por defecto de
20 Hz a 40 Hz. Esto deja el límite por encima de la fuente medida de 20 Hz,
por lo que cada muestra articular puede distribuir su TF. No añade esperas,
no usa TF más reciente como sustituto y no modifica el STOP conservador.

## Validación

Se compiló exclusivamente `thesis_hardware` y se ejecutaron sus pruebas:
11 pruebas, 0 fallos, 0 errores, 1 omitida.

La repetición de 30.219 s tuvo 605 `joint_states`, 605 mensajes `/tf`, 605
estados de proximidad `ALLOW` y cero `STOP/TF_UNAVAILABLE`. La cadencia TF
fue 49.773/50.001/50.264 ms (mínima/mediana/máxima). El diagnóstico final
confirmó conexión, estado articular y proximidad válidos, sin fallos de
lectura, `armed=false`, modo `hold` y 0 Hz de comandos físicos.
