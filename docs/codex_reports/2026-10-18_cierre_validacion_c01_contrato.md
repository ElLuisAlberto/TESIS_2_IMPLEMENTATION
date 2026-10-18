# Cierre de validación C-01 — contrato de admisión puntual

Fecha: 2026-10-18  
Resultado: **C-01 VALIDADA EN PRUEBAS UNITARIAS**

## Causa y corrección

La implementación ya cumplía el contrato: `admit_point_command()` retorna un
`reason_code` estructurado y el callback llama
`reject_point_command(msg, reason_code, reason)`. La prueba confundía el
tercer argumento, el texto diagnóstico, con el segundo, el código.

Se corrigió exclusivamente
`src/thesis_core/test/test_point_command_admission.py`: las aserciones leen
el código de `rejections[0][1]` y conservan la comprobación de que el texto
diagnóstico contiene ese código.

## Contrato final

- Códigos estructurados: `POINT_COMMAND_STAMP_ZERO`, `_STALE`, `_FUTURE`,
  `POINT_COMMAND_ID_EMPTY` y `_REUSED`.
- Diagnóstico: texto legible independiente que incluye el código.
- Un punto rechazado no se publica; si coincide con punto activo o pendiente,
  sigue publicándose el STOP de seguridad ya definido.
- No hubo cambios a frescura, IDs, STOP, límites ni mensajes ROS.

## Resultado

- Admisión puntual del supervisor: 8/8 aprobadas.
- Dependencias directas C-01 del supervisor: 6/6 aprobadas.
- Prueba directa del puente: 1/1 aprobada.
- `thesis_core` y `thesis_hardware_bridge`: compilados correctamente.

C-01 queda validada a nivel unitario. Esta validación no equivale a una
autorización de hardware físico; permanecen aplicables los demás bloqueantes
del informe de auditoría previa a físico.

Evidencia completa:
[2026-10-18_validacion_c01_contrato_admision.md](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/evidence/codex_reports/2026-10-18_validacion_c01_contrato_admision.md).
