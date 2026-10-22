# Fase 6F — Corrección de launch y reintento de sombra

Fecha: 2026-10-21. Resultado: **BLOCKED_HARDWARE_LAUNCH_CONTRACT**.

## Causa confirmada

El contrato de sesión intenta incluir
`thesis_hardware/launch/thesis_hardware.launch.py`, y la Fase 6E confirmó que
esa ruta no existe en el paquete instalado. La sesión se detuvo antes de crear
procesos o establecer conexión con el JACO2.

## Inspección y decisión

Esta fase exige localizar el launch físico real y verificar en sus argumentos
declarados cuál garantiza explícitamente `hardware_output=false`. Sin esa
confirmación, sustituir la ruta por un nombre inferido o pasar argumentos no
declarados sería un fallback/adivinanza prohibido por el procedimiento.

No se modificó `jaco_d435i_shadow_session.launch.py`, su prueba, ni ningún otro
archivo de producto. No se compiló ni ejecutó un reintento físico porque el
prerrequisito de contrato no quedó verificable de forma segura.

## Requisito para desbloquear

Se necesita una referencia verificable al archivo de launch instalado de
`thesis_hardware` que exponga la modalidad de solo lectura y documente el
argumento que mantiene la salida física desarmada. Con ese nombre y esos
argumentos reales se podrá hacer una corrección de una sola ruta, compilar los
dos paquetes afectados, validar `--show-args` y realizar el único reintento
autorizado de Fase 6E.

## Seguridad

No se inició JACO2, D435i, RViz, hardware_output, TF temporal, self-filter,
extractor ni supervisor. No hubo movimiento, publicaciones a
`/thesis/obstacle_input` ni reintentos físicos.
