# TESIS_2_IMPLEMENTATION

Plataforma experimental en ROS 2 para supervisar comandos de movimiento de un
manipulador Kinova Jaco2 antes de enviarlos a un backend de ejecución.

La propuesta separa la intención del usuario, la cinemática, la predicción de
corto horizonte, la evaluación geométrica, la decisión de seguridad y el
adaptador de ejecución. Esto permite estudiar la misma política en simulación
y, de forma controlada, frente al brazo físico.

## Estado de la tesis

Estado al 18/10/2026: **simulación validada y sistema físico preparado para
conexión; lectura física aún no validada**.

- La simulación cartesiana y articular tienen pruebas offline y validaciones en
  Gazebo. La prueba integrada GUI → supervisor → Gazebo quedó demostrada para
  X positivo; la validación completa X/Y/Z requiere resolver eventos
  `TF_UNAVAILABLE`.
- El adaptador físico es nativo ROS 2 y usa la API USB de Kinova. La salida
  empieza deshabilitada (`hardware_output_enabled=false`) y el sistema inicia
  desarmado (`armed=false`).
- A-01, A-02a, A-03 y C-01 cuentan con cobertura offline. A-02b está bloqueada
  porque todavía no existe un contrato completo para comparar TF y modelo
  articulado con sello, posición y orientación.
- La ruta cartesiana física permanece bloqueada hasta cerrar A-02b.
- La cámara RGB-D no está conectada y su integración con el supervisor no está
  validada. El Avance 3 no se declara terminado.

La evidencia técnica está organizada en
[`docs/codex_reports/`](docs/codex_reports/) y
[`evidence/codex_reports/`](evidence/codex_reports/).

## Arquitectura

```text
GUI o fuente de intención
        ↓
comando candidato
        ↓
cinemática / trayectoria / predicción
        ↓
evaluación geométrica
        ↓
supervisor: ALLOW · WARNING · REDUCTION · STOP
        ↓
adaptador de simulación o adaptador Kinova
        ↓
Gazebo o Jaco2 físico
```

La GUI no se comunica directamente con el hardware. En simulación publica al
adaptador de simulación; en hardware la única ruta de salida pasa por
`jaco_hardware_adapter` y la API Kinova.

## Código principal

| Paquete o carpeta | Función |
| --- | --- |
| `src/thesis_interfaces` | Mensajes y contratos compartidos |
| `src/thesis_core` | Supervisor, predicción, cinemática y geometría |
| `src/thesis_simulation` | Gazebo y adaptadores de simulación |
| `src/thesis_hardware_bridge` | Adaptador ROS 2–Kinova |
| `src/thesis_hardware` | Launch físico, readiness y precondiciones |
| `src/thesis_ui` | Interfaz para simulación y hardware |
| `src/thesis_telemetry` | Registro de decisiones y ejecución |
| `src/thesis_validation` | Validadores y escenarios |
| `tools/physical` | Preflight físico y protocolo articular |
| `tools/realsense_d435i_test` | Pruebas independientes de la cámara |

Los directorios `build/`, `install/` y `log/` son artefactos locales de
compilación y no forman parte del repositorio.

## Modos de trabajo

| Modo | Interfaz | Estado |
| --- | --- | --- |
| Simulación | GUI en `operation_mode=simulation` | Disponible para pruebas |
| Hardware en lectura | GUI física sin armado | Preparado, pendiente de conexión |
| Movimiento articular físico | GUI → supervisor → Kinova | Requiere preflight y autorización |
| Movimiento cartesiano físico | Ruta cartesiana supervisada | Bloqueado por A-02b |
| RGB-D conectado al supervisor | Cámara → obstáculo → supervisor | No validado |

Las rutas deben ejecutarse por separado. No se deben iniciar simultáneamente
Gazebo y el adaptador físico en el mismo dominio ROS.

## Ejecución en simulación

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

ros2 launch thesis_simulation thesis_system.launch.py \
  simulation_output_enabled:=true \
  use_demo_obstacle:=true \
  start_gui:=true \
  start_rviz:=false
```

Este modo permite presentar la interfaz, observar la decisión del supervisor y
probar el flujo de comandos sin conectar el brazo físico.

## Preparación física

El procedimiento de conexión, el observador de solo lectura y la primera
prueba articular están documentados en
[`tools/physical/README.md`](tools/physical/README.md).

La inspección estática puede ejecutarse sin brazo:

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
python3 tools/physical/physical_preflight.py --static-only
```

Con el brazo conectado, el preflight debe confirmar USB, API Kinova,
JointState, frecuencia, TF, proximidad, readiness, supervisor, ausencia de
movimiento y `armed=false`. Si no devuelve `PASS`, no se habilita salida física.

La primera prueba física prevista es articular: un solo joint, una dirección,
velocidad mínima, desplazamiento pequeño, `ALLOW` previo, watchdog y STOP
físico disponible. No se preparan primero trayectorias cartesianas, movimientos
largos ni automatización de evasión.

## Percepción RGB-D

La cámara todavía no participa en la decisión preventiva. El contrato previsto
es:

```text
cámara → percepción → obstáculo válido → obstacle_input_bridge
→ proximity_monitor → safety_supervisor
```

Antes de declararla validada faltan cámara real, calibración cámara→`world`,
sellos de adquisición, incertidumbre, representación de obstáculos y pruebas
reales de `ALLOW` y `STOP`.

## Verificación offline

La evidencia completa de las pruebas está en `evidence/codex_reports/`. Como
comprobación focal de la preparación física, se compilan solo los paquetes
afectados y se ejecutan los contratos de *readiness*, telemetría y bridge. Los
movimientos físicos no sustituyen una prueba unitaria.

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

colcon build --packages-select \
  thesis_core thesis_hardware_bridge thesis_hardware thesis_telemetry

python3 -m pytest -q \
  src/thesis_hardware/test/test_readiness_contract.py \
  src/thesis_hardware/test/test_readiness_logging.py \
  src/thesis_telemetry/test/test_telemetry_math.py

ctest --test-dir build/thesis_hardware_bridge \
  --output-on-failure -R '^test_bridge_logic$'
```

## Limitaciones y siguiente etapa

- La conexión física, la lectura real y el movimiento físico siguen pendientes.
- A-02b debe definir y validar la coherencia TF-modelo antes de autorizar
  movimiento cartesiano.
- `TF_UNAVAILABLE` observado en la validación integrada de Gazebo debe
  investigarse por separado.
- La RGB-D debe integrarse gradualmente y permanecer fail-closed ante datos
  ausentes, vencidos o inconsistentes.

El detalle de cada decisión, prueba y bloqueo se conserva en los informes; este
README resume el proyecto para presentación y consulta rápida.
