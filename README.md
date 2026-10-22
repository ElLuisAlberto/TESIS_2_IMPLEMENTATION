# TESIS_2_IMPLEMENTATION

Plataforma experimental en ROS 2 para supervisar comandos de movimiento de un
manipulador Kinova Jaco2 antes de enviarlos a un backend de ejecución.

La propuesta separa la intención del usuario, la cinemática, la predicción de
corto horizonte, la evaluación geométrica, la decisión de seguridad y el
adaptador de ejecución. Esto permite estudiar la misma política en simulación
y, de forma controlada, frente al brazo físico.

## Estado de la tesis

Estado al 22/10/2026: **simulación y movimiento articular físico J1 validados;
percepción RGB-D en observación, todavía fuera de la decisión preventiva**.

- El JACO2 funciona mediante la ruta candidato → supervisor preventivo →
  adaptador Kinova. El backend USB, feedback a 20 Hz, watchdog, TF y retorno a
  `hold` fueron comprobados físicamente.
- Dos ciclos J1 de +3° y retorno al origen terminaron dentro de 0.5° de error,
  sin watchdog, órdenes obsoletas ni movimiento durante `hold`.
- El armado inicia sin conservar JOG anteriores. Un STOP geométrico permite
  únicamente una retirada supervisada y no desarma por un STOP histórico.
- Existe una esfera virtual reproducible con escenarios `PARK`, `BLOCK_START`
  e `INSERT_STOP`. PARK está calibrado aproximadamente a 0.40 m; los escenarios
  STOP aún requieren su prueba física separada.
- La Intel RealSense D435i entrega RGB, profundidad y nube. Se prepararon un
  preprocesamiento de corto alcance y filtros sombra de cápsulas y mallas para
  distinguir el brazo de posibles puntos externos, sin conectarlos al
  supervisor ni habilitar salida física.
- Las capturas sincronizadas y la cadena TF de cámara se verificaron, pero el
  ajuste extrínseco basado en bordes se rechazó: se alineaba con el fondo y
  el robot rojo. Hay un tablero ChArUco y un detector offline preparados para
  una calibración temporal con referencia física; faltan su montaje y pose
  medida respecto de `world`.
- La ruta cartesiana física y la integración RGB-D → obstáculos siguen
  pendientes. Por ello no se declara terminado el Avance 3.

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
| `src/thesis_jaco_perception` | Auto-filtrado diagnóstico de cápsulas y mallas, aislado del supervisor |
| `thesis_perception` | Cámara D435i, preprocesamiento y extracción de obstáculos |
| `tools/physical` | Preflight físico y protocolo articular |
| `tools/perception` | Capturas diagnósticas y análisis offline de calibración |
| `tools/realsense_d435i_test` | Pruebas independientes de la cámara |

Los directorios `build/`, `install/` y `log/` son artefactos locales de
compilación y no forman parte del repositorio.

## Modos de trabajo

| Modo | Interfaz | Estado |
| --- | --- | --- |
| Simulación | GUI en `operation_mode=simulation` | Disponible para pruebas |
| Hardware en lectura | GUI física sin armado | Validado |
| Movimiento articular físico | GUI → supervisor → Kinova | J1 ±3° validado |
| Movimiento cartesiano físico | Ruta cartesiana supervisada | Bloqueado por A-02b |
| Visualización RGB-D | D435i → RViz | Disponible |
| Auto-filtrado sombra JACO2 | D435i → cápsulas/mallas → diagnóstico | Experimental, sin uso preventivo |
| Extrínseca D435i → `world` | ChArUco sobre base del JACO2 | Pendiente de pose física medida y captura válida |
| RGB-D conectado al supervisor | Cámara → obstáculo → supervisor | Pendiente |

Las rutas deben ejecutarse por separado. No se deben iniciar simultáneamente
Gazebo y el adaptador físico en el mismo dominio ROS.

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

El protocolo validado comienza con un solo joint, velocidad reducida,
desplazamiento pequeño, `ALLOW` previo, watchdog y parada física disponible.
No se deben improvisar trayectorias cartesianas, movimientos largos ni
automatización de evasión.

## Comandos de ejecución

Ejecuta primero esta preparación en cada terminal nueva:

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Si VS Code fue instalado como Snap y aparece un conflicto con bibliotecas GTK o
`libpthread`, abre una terminal normal del sistema. No hacen falta `sudo` ni
paquetes adicionales.

### Simulación completa

```bash
ros2 launch thesis_simulation thesis_system.launch.py \
  simulation_output_enabled:=true \
  use_demo_obstacle:=true \
  start_gui:=true \
  start_rviz:=true
```

Abre Gazebo, la GUI y RViz sin utilizar el USB del brazo.

### JACO2 físico en observación

```bash
ros2 launch thesis_hardware jaco_physical_system.launch.py \
  mock_hardware:=false \
  hardware_output_enabled:=false \
  use_demo_obstacle:=true \
  demo_obstacle_scenario:=PARK \
  start_gui:=true \
  start_rviz:=true \
  allow_gui_arm_control:=false
```

Conecta el brazo y publica su estado real, pero no permite armar ni mover.

### JACO2 físico con movimiento supervisado

```bash
ros2 launch thesis_hardware jaco_physical_system.launch.py \
  mock_hardware:=false \
  hardware_output_enabled:=true \
  use_demo_obstacle:=true \
  demo_obstacle_scenario:=PARK \
  demo_insert_stop_active:=false \
  start_gui:=true \
  start_rviz:=true \
  allow_gui_arm_control:=true
```

Habilita el botón de armado, pero el launch siempre comienza con
`armed=false`, `mode=hold` y salida 0 Hz. Úsalo únicamente con el área despejada,
la parada de emergencia accesible y proximidad `ALLOW/READY`. No abras dos
launches físicos a la vez: el bloqueo USB admite un solo dueño.

### Reabrir solo la GUI física

```bash
ros2 run thesis_ui joint_gui --ros-args \
  -r __node:=joint_control_gui_node \
  -p use_sim_time:=false \
  -p require_system_readiness:=true \
  -p operation_mode:=hardware \
  -p allow_hardware_arm_control:=true
```

Úsalo cuando el stack físico ya está activo y únicamente se cerró la ventana.

### Reabrir solo el RViz del robot

```bash
rviz2 -d "$(ros2 pkg prefix --share thesis_description)/rviz/jaco.rviz"
```

Muestra el modelo medido, cápsulas, horizonte y esfera virtual sin iniciar otro
adaptador USB.

### RealSense D435i y RViz RGB-D

En una terminal inicia color, profundidad alineada, RGB-D y nube de puntos:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  enable_rgbd:=true \
  enable_sync:=true \
  align_depth.enable:=true \
  pointcloud.enable:=true
```

En otra terminal abre la configuración de nube de puntos:

```bash
rviz2 \
  -d /opt/ros/humble/share/realsense2_camera/examples/pointcloud/rviz/pointcloud.rviz \
  --ros-args -r __node:=rviz2_rgbd
```

Los tópicos principales son `/camera/camera/color/image_raw`,
`/camera/camera/aligned_depth_to_color/image_raw`, `/camera/camera/rgbd` y
`/camera/camera/depth/color/points`. Esta ejecución solo visualiza la cámara;
no la conecta todavía al supervisor preventivo.

Detén cualquier launch con `Ctrl+C`. Antes de reiniciar el stack físico,
confirma que la sesión anterior terminó para no competir por el USB.

## Percepción RGB-D

La cámara ya funciona como fuente RGB-D independiente y los filtros sombra
publican resultados diagnósticos en una sesión de sólo lectura. Esto **no**
demuestra que la nube externa esté libre de puntos del brazo ni autoriza usarla
para seguridad: persisten residuos sobre el brazo y no hay una extrínseca
validada. La prueba de alineación de silueta de [Fase 8T](docs/codex_reports/2026-10-22_fase8t_cadena_tf_y_candidato_extrinseco.md)
se rechazó. La [Fase 8U](docs/codex_reports/2026-10-22_fase8u_preparacion_charuco_temporal.md)
dejó preparado el tablero ChArUco; la captura siguiente espera la pose del
tablero medida por el operador.

El contrato de integración aún pendiente es:

```text
cámara → percepción → obstáculo válido → obstacle_input_bridge
→ proximity_monitor → safety_supervisor
```

Antes de declararla integrada faltan calibración cámara→`world` con referencia
física, validación del auto-filtrado y de la extracción de obstáculos, sellos de
adquisición, incertidumbre y pruebas físicas de `ALLOW`, `REDUCTION` y `STOP`.

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

Las pruebas focales de la ruta cartesiana, JOG continuo, GUI, sincronización
FK-TF y escenarios de obstáculo virtual se ejecutan por separado:

```bash
python3 -m pytest -q \
  src/thesis_core/test/test_cartesian_kinematics.py \
  src/thesis_core/test/test_a02a_frame_tf_interlock.py \
  src/thesis_core/test/test_virtual_obstacle_scenarios.py \
  src/thesis_simulation/test/test_cartesian_jog_model.py \
  src/thesis_ui/test/test_gui_state.py
```

La prueba de contrato de percepción RGB-D permanece independiente del brazo:

```bash
python3 -m pytest -q \
  thesis_perception/test/test_perception_safety_contract.py
```

## Limitaciones y siguiente etapa

- El movimiento físico validado se limita a la ruta articular supervisada y a
  desplazamientos pequeños; no cubre todavía control cartesiano.
- A-02b debe definir y validar la coherencia TF-modelo antes de autorizar
  movimiento cartesiano.
- Los escenarios virtuales STOP están calibrados, pero su ensayo físico debe
  ejecutarse como una etapa separada y controlada.
- La RGB-D debe conectarse gradualmente al pipeline y permanecer fail-closed
  ante datos ausentes, vencidos o inconsistentes.
- Los filtros de cápsulas y mallas son diagnósticos de sesión, no una fuente de
  `obstacle_input` ni una garantía de espacio libre. La calibración ChArUco
  requiere primero imprimir, fijar y medir el tablero; no hay candidato activo.

El detalle de cada decisión, prueba y bloqueo se conserva en los informes; este
README resume el proyecto para presentación y consulta rápida.
