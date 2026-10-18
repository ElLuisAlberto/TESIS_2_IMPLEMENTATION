# Corrección C-01 — frescura y replay de punto/ExecutionControl

Fecha: 2026-10-18  
Estado: **NO VALIDADA — pruebas bloqueadas por entorno**

## Cambios aplicados

- El supervisor valida antes de publicar un comando puntual: `command_id` no
  vacío, no visto en la ventana reciente, sello no cero, edad máxima y futuro
  dentro de tolerancia. Un rechazo no llega a
  `/thesis/supervised_command`; si coincide con un punto activo o pendiente,
  publica STOP runtime.
- El puente repite la validación antes de crear `active_goal_`. Para un
  rechazo no se crea trayectoria ni se llama a la salida; si ya había objetivo
  o JOG activo, aplica el HOLD/zero existente.
- El puente acepta `ExecutionControl` no STOP solo si su sello es válido y
  fresco, su ID no está vacío y coincide con `active_goal_`. Controles frescos
  repetidos del mismo ID siguen renovando el watchdog. STOP conserva prioridad
  inmediata, incluso ante sello o ID defectuoso, porque detener es seguro.
- Ambos lados usan una memoria local acotada a 256 IDs. Se reserva un ID no
  vacío aun cuando el intento termina rechazado, para impedir replay dentro de
  la ventana. No se modificaron mensajes, YAML ni launch.

## Política final

- Edad máxima: reutiliza `max_jog_message_age_sec` (0,15 s por defecto).
- Futuro permitido: reutiliza la tolerancia JOG existente de 0,05 s.
- `stamp == 0`, vencido o futuro fuera de tolerancia: rechazo.
- Punto con ID vacío, activo o terminal reciente: rechazo.
- Control no STOP con ID vacío, ajeno al activo o sello inválido: rechazo y,
  si existe objetivo, cancelación/HOLD/zero.

## Pruebas y resultado

Se añadieron pruebas unitarias de admisión de punto y de `ExecutionControl`,
incluyendo sellos válido/cero/vencido/futuro, ID vacío/reutilizado/ajeno y
prioridad STOP. La comprobación `py_compile` fue correcta.

La colección de las pruebas Python falló antes de ejecutar casos porque el
intérprete no pudo importar `rclpy`, incluso tras cargar
`/opt/ros/humble/setup.bash`. Por la regla de detención, no se compiló ni se
probó el puente C++ adicionalmente. Evidencia completa:
[2026-10-18_pruebas_c01_frescura_replay.md](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/evidence/codex_reports/2026-10-18_pruebas_c01_frescura_replay.md).

## Decisión pendiente

Restablecer un entorno ROS de pruebas que exponga `rclpy`, ejecutar las
pruebas C-01 y compilar únicamente `thesis_core` y `thesis_hardware_bridge`.
Hasta entonces esta corrección no autoriza hardware físico.
