# Preflight físico JACO2 y primera prueba articular

Este procedimiento mantiene el brazo en solo lectura. No usa `HOME`,
`RETRACT`, joystick, comandos articulares/cartesianos ni publicaciones con
`ros2 topic pub`.

## 1. Inspección estática sin brazo

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
python3 tools/physical/physical_preflight.py --static-only
```

Un resultado `STATIC_ONLY` confirma únicamente archivos, dependencias y
defaults. No valida USB, conexión, estado articular, TF ni percepción real.

## 2. Conexión física en solo lectura

Condiciones previas del operador: brazo inmóvil, espacio despejado, parada
física accesible, sin joystick, y ningún otro proceso Kinova activo.

Terminal 1:

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
ros2 launch thesis_hardware jaco_physical_system.launch.py \
  mock_hardware:=false hardware_output_enabled:=false \
  use_demo_obstacle:=false start_gui:=false start_rviz:=false \
  allow_gui_arm_control:=false \
  telemetry_output_directory:="$PWD/evidence/physical_sessions"
```

Terminal 2, únicamente después de que el launch quede estable:

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
python3 tools/physical/physical_preflight.py \
  --duration-sec 10 \
  --output evidence/codex_reports/2026-10-18_preflight_fisico_solo_lectura.json
```

El observador no crea publishers ni clientes de servicio/acción. Comprueba el
valor efectivo de `mock_hardware=false` y
`hardware_output_enabled=false`, `armed=false`, conexión, seis joints, tasa y
sellos, TF `world -> j2n6s300_end_effector` al sello articular, proximidad,
readiness, supervisor, modo `hold`, ausencia de comandos, ausencia de eventos
de ejecución y ausencia de movimiento. Detener el launch con `Ctrl-C` en su
propia terminal.

No continuar si el resultado no es `PASS`. La falta de obstáculo RGB-D real
puede mantener readiness en `NOT_READY`; no debe sustituirse por un obstáculo
sintético para declarar lectura física validada.

## 3. Primera prueba articular posterior (no ejecutar aún)

Esta fase exige primero un preflight físico `PASS`, revisión de su evidencia y
autorización explícita del usuario. A‑02b permanece bloqueada, por lo que esta
prueba es exclusivamente articular.

- Un operador permanece junto al STOP físico y confirma que funciona antes de
  habilitar salida.
- Registrar postura inicial y escoger solo J6, una única dirección, máximo
  `+1°` y duración mínima de `5 s` (velocidad nominal `0.2°/s`).
- Usar exclusivamente `/thesis/candidate_command`; nunca publicar al
  controlador Kinova ni a `/thesis/supervised_command`.
- Exigir `ALLOW` previo, watchdog activo y correlación por un `command_id`
  nuevo, no vacío.
- Detener y desarmar ante STOP/HOLD, pérdida de conexión, JointState, TF,
  proximidad, readiness, supervisor o telemetría.
- Después del único desplazamiento, desarmar. El retorno es otra sesión y
  requiere nueva autorización; no debe encadenarse automáticamente.

El script histórico `tools/validation/run_kinova_physical_micro_motion.sh`
realiza ida y retorno de 2° y no corresponde a esta primera prueba. No usarlo
para el gate inicial.

No se proporciona aquí un comando automático que cambie
`hardware_output_enabled` o arme el adaptador: esos pasos quedan reservados a
una sesión física posterior, presencial y autorizada.
