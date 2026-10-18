# Preparación técnica del Avance 3 físico

Fecha: 2026-10-18  
Alcance: auditoría, interbloqueos, preflight y pruebas offline  
Resultado general: **PREPARADO PARA CONEXIÓN; VALIDACIÓN FÍSICA PENDIENTE**

## 1. Resumen ejecutivo

El proyecto queda preparado para conectar el Kinova Jaco2 en **solo lectura**.
No se detectó el dispositivo USB Kinova `22cd:0000`, por lo que, siguiendo el
gate de la tarea, no se inició el backend físico. No se ejecutaron Gazebo,
RViz, cámara, GUI, comandos ROS de movimiento, `HOME`, `RETRACT`, joystick ni
movimientos físicos.

Se reforzó el rechazo fail-closed de JointState y proximidad en el readiness y
en el adaptador físico; la GUI y su control de armado quedaron desactivados por
defecto en el launch físico; se añadió un preflight observacional que no crea
publishers ni clientes de servicio/acción; y la telemetría registra el ciclo de
ejecución con un único terminal por `command_id`.

El worktree ya contenía cambios de etapas anteriores. Todos se preservaron; no
se borró, movió ni revirtió ningún archivo.

## 2. Arquitectura y adaptador físico auditados

La ruta conservada es:

```text
GUI/intención -> /thesis/candidate_command -> safety_supervisor
-> /thesis/supervised_command + /thesis/execution_control
-> jaco_hardware_adapter -> SDK USB Kinova -> Jaco2
```

- El adaptador es ROS 2 nativo: `thesis_hardware_bridge`.
- Usa directamente la API USB Kinova mediante
  `Kinova.API.USBCommandLayerUbuntu.h` y
  `USBCommandLayerUbuntu.so`.
- No se requiere ROS 1 ni `ros1_bridge` para la ruta existente.
- `InitAPI`, `GetDevices`, `SetActiveDevice` y `GetAngularPosition` forman la
  ruta de conexión/lectura. `StartControlAPI`, `SetAngularControl`,
  `EraseAllTrajectories` y `SendBasicTrajectory` solo se alcanzan al armar o
  enviar salida.
- El adaptador publica `/joint_states` a 20 Hz configurados y publica los seis
  joints canónicos más dedos.
- `robot_state_publisher` publica la TF desde el URDF cuya raíz es `world`; el
  preflight exige `world -> j2n6s300_end_effector` al sello articular.
- El único componente con llamadas a la API de movimiento Kinova es
  `jaco_hardware_adapter`.
- La GUI publica intención candidata; no publica directamente a la API ni al
  controlador. El modo hardware bloquea sus funciones cartesianas.
- Simulación usa `simulation_command_adapter`; el launch físico usa
  `jaco_hardware_adapter`. No se encontró inclusión cruzada.

## 3. Archivos revisados

Se revisaron los informes relevantes de `docs/codex_reports/` y
`evidence/codex_reports/`, especialmente la auditoría pre-física, la matriz y
plan del Avance 3, A-01, A-02a/A-02b, A-03, C-01, la validación GUI/Gazebo y el
diagnóstico `TF_UNAVAILABLE`.

Código y configuración principales revisados:

- `src/thesis_hardware/launch/jaco_physical_system.launch.py`
- `src/thesis_hardware/thesis_hardware/hardware_readiness_node.py`
- `src/thesis_hardware/thesis_hardware/readiness_contract.py`
- `src/thesis_hardware/udev/70-thesis-kinova.rules`
- `src/thesis_hardware_bridge/src/jaco_hardware_node.cpp`
- `src/thesis_hardware_bridge/include/thesis_hardware_bridge/bridge_logic.hpp`
- `src/thesis_hardware_bridge/config/jaco_hardware.yaml`
- `src/thesis_hardware_bridge/launch/jaco_hardware.launch.py`
- `src/thesis_core/thesis_core/safety_supervisor_node.py`
- `src/thesis_core/thesis_core/proximity_monitor_node.py`
- `src/thesis_core/thesis_core/obstacle_input_bridge.py`
- `src/thesis_ui/thesis_ui/joint_control_gui.py`
- `src/thesis_telemetry/thesis_telemetry/telemetry_logger.py`
- `src/thesis_validation/` y `tools/validation/`
- `tools/realsense_d435i_test/`
- interfaces `JointCommand`, `ExecutionControl`, `ExecutionTrajectory`,
  `ProximityStatus` y `Obstacle`.

## 4. Archivos modificados en esta tarea

- `src/thesis_hardware/launch/jaco_physical_system.launch.py`
- `src/thesis_hardware/package.xml`
- `src/thesis_hardware/thesis_hardware/hardware_readiness_node.py`
- `src/thesis_hardware/thesis_hardware/readiness_contract.py`
- `src/thesis_hardware/test/test_readiness_contract.py`
- `src/thesis_hardware_bridge/include/thesis_hardware_bridge/bridge_logic.hpp`
- `src/thesis_hardware_bridge/src/jaco_hardware_node.cpp`
- `src/thesis_hardware_bridge/test/test_bridge_logic.cpp`
- `src/thesis_telemetry/package.xml`
- `src/thesis_telemetry/thesis_telemetry/telemetry_logger.py`
- `src/thesis_telemetry/thesis_telemetry/telemetry_math.py`
- `src/thesis_telemetry/test/test_telemetry_math.py`
- `tools/physical/physical_preflight.py` (nuevo)
- `tools/physical/README.md` (nuevo)
- este informe (nuevo).

Los cambios anteriores ya presentes en esos archivos, en particular C-01 y
A-03, no se revirtieron ni se atribuyen a esta ejecución.

## 5. Interbloqueos verificados

Verificados por código y pruebas offline:

- `hardware_output_enabled=false` por defecto en YAML y launch.
- `armed=false` al iniciar; el API de control no se inicia durante conexión de
  solo lectura.
- rechazo de armado por salida no permitida, conexión ausente, JointState no
  fresco, readiness falso/vencido, proximidad inválida/vencida, supervisor
  ausente o STOP/HOLD activo;
- desarmado al perder esas precondiciones;
- JointState canónico completo, finito, con sello no cero, no futuro y fresco;
- proximidad con estado conocido, frame exactamente `world`, valores finitos,
  sello no cero, no futuro y fresco;
- STOP de proximidad mantiene readiness en `NOT_READY`;
- comando puntual con ID vacío/repetido y sello cero/vencido/futuro;
- control de ejecución con ID ajeno, sello cero/vencido/futuro;
- STOP conserva prioridad aunque ID o sello sean inválidos;
- watchdog de estado, JOG y `ExecutionControl` existentes;
- límites articulares, límite de velocidad y salida cero ante fallos;
- GUI y control de armado desactivados por defecto en launch físico;
- launch de simulación separado del adaptador físico.

Limitación: la presencia del supervisor se detecta por grafo; no existe un
contrato de heartbeat independiente. Durante una ejecución, la pérdida de
`ExecutionControl` queda cubierta por su watchdog.

## 6. Estado efectivo y conexión

Inspección estática del 2026-10-18:

- cabecera Kinova: presente;
- biblioteca x86_64 Kinova: presente y con dependencias resueltas;
- 10 símbolos requeridos de lectura/control: presentes;
- regla udev existente para `22cd:0000` con grupo `plugdev` y modo `0660`;
- `lsusb`: dispositivo Kinova `22cd:0000` **no detectado**;
- procesos Jaco/Gazebo activos: ninguno observado;
- `mock_hardware=false`, `hardware_output_enabled=false`, GUI desactivada y
  control GUI de armado desactivado en defaults.

No hubo runtime físico. Por tanto los valores efectivos del nodo, permisos del
dispositivo, conexión, JointState, TF, readiness, proximidad, ausencia de
movimiento y controladores no están físicamente validados. No se fabricó un
resultado mock.

## 7. A-02b, M-01 y RGB-D

### A-02b

**BLOCKED.** Sigue faltando el contrato explícito para comparar el estado
articular sincronizado y el modelo geométrico con TF en `world`, incluyendo
posición y orientación. No se inventó topología ni tolerancia y no se cambió
A-02a. `kinematic_tf_tolerance_m` no puede declararse validado.

Consecuencia: el movimiento cartesiano físico no está autorizado. El
`TF_UNAVAILABLE` intermitente observado en Gazebo permanece como problema
separado y no fue ocultado ni modificado.

### M-01

**PARCIAL / evidencia física incompleta.** El logger ahora observa
`ExecutionTrajectory`, crea `execution_events.csv`, registra estado, detalle,
inicio/objetivo/posición leída, conexión, armado, permiso de salida y contexto
de proximidad, y deduplica estados terminales por `command_id`. El launch
físico inicia este observador por defecto.

Falta validarlo en sesión física y no existe aún un campo contractual completo
de coherencia TF-modelo. Por ello M-01 no se declara cerrada.

### RGB-D

**NOT_VALIDATED.** No existe `src/thesis_perception/`. Hay herramientas
independientes para una RealSense D435i y un contrato genérico `Obstacle`, pero
no se encontró en el pipeline de tesis un driver, ROI, filtrado, downsampling,
agrupamiento, calibración cámara→`world`, extracción de obstáculos ni health
monitor integrados. La cámara no está conectada y no se ejecutó.

El único flujo preparado es el contrato:

```text
fuente futura RGB-D -> /thesis/obstacle_input -> obstacle_input_bridge
-> proximity_monitor -> safety_supervisor
```

## 8. Compilación

```bash
colcon build --packages-select \
  thesis_hardware_bridge thesis_hardware thesis_telemetry \
  --event-handlers console_direct+
```

Resultado exacto: **3 paquetes compilados correctamente** en 17.5 s.

Después del último ajuste de deduplicación se recompiló únicamente
`thesis_telemetry`: **1 paquete compilado correctamente** en 1.20 s. El comando
compuesto intentó volver a cargar `install/setup.bash` con `nounset` activo y
terminó después de la compilación con código 127 (`COLCON_TRACE` no definida);
se corrigió solo el entorno temporal (`set +u` durante `source`) y la prueba
posterior pasó 5/5. El módulo instalado expone el helper nuevo correctamente.

Tras añadir los campos diagnósticos finales (`api_open`,
`control_api_started`, `active_goal` y `jog_active`) se recompiló únicamente
`thesis_hardware_bridge`: **1 paquete compilado correctamente** en 14.6 s y
`test_bridge_logic` volvió a pasar 1/1.

## 9. Pruebas ejecutadas

Pruebas Python seleccionadas de hardware, readiness, telemetría, watchdog,
seguridad, frescura, A-02a, percepción contractual y separación GUI:

```text
74 passed in 0.61s
```

La repetición final específica de telemetría produjo `5 passed in 0.01s`.

Prueba C++ del bridge:

```text
test_bridge_logic: 1/1 passed (0.06 s)
```

Adicionalmente:

- `py_compile`: correcto;
- `flake8` de archivos afectados: correcto;
- generación offline del launch físico: 28 entidades, correcta;
- `git diff --check`: correcto;
- preflight `--static-only`: `STATIC_ONLY`, SDK correcto y USB Kinova ausente.

No se ejecutaron pruebas físicas, Gazebo, RViz ni cámara.

## 10. Preflight seguro para el usuario

Procedimiento completo:

`tools/physical/README.md`

Inspección sin brazo:

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
python3 tools/physical/physical_preflight.py --static-only
```

Launch futuro de solo lectura, con brazo conectado:

```bash
ros2 launch thesis_hardware jaco_physical_system.launch.py \
  mock_hardware:=false hardware_output_enabled:=false \
  use_demo_obstacle:=false start_gui:=false start_rviz:=false \
  allow_gui_arm_control:=false \
  telemetry_output_directory:="$PWD/evidence/physical_sessions"
```

Observador, en otra terminal:

```bash
python3 tools/physical/physical_preflight.py \
  --duration-sec 10 \
  --output evidence/codex_reports/2026-10-18_preflight_fisico_solo_lectura.json
```

El preflight comprueba USB/permisos/API, valores efectivos, conexión,
JointState/tasa/nombres/sellos, TF, proximidad, readiness, supervisor, modo
hold, ausencia de comandos/eventos/movimiento y fallos críticos. El backend es
USB Kinova nativo; `controller_manager` no aplica.

## 11. Primera prueba articular preparada, no autorizada ni ejecutada

El protocolo futuro usa solo J6, una dirección, máximo `+1°`, duración mínima
de 5 s, `ALLOW` previo, ID nuevo, watchdog, telemetría, STOP físico accesible y
desarmado inmediato ante pérdida de conexión, JointState, TF, proximidad,
readiness o supervisor. Al terminar el único desplazamiento se desarma; el
retorno requiere otra autorización y otra sesión.

No se incluyó activación automática de `hardware_output_enabled` ni armado. El
script histórico de ida/retorno de 2° no cumple este gate inicial y está
marcado en el procedimiento como **no usar** para la primera prueba.

## 12. Bloqueos restantes

1. Jaco2 no conectado: lectura física no validada.
2. Permisos USB reales no comprobables sin el dispositivo.
3. JointState, tasa, TF, readiness y proximidad físicas no observados.
4. STOP físico y desarmado por pérdida de señales no probados en banco.
5. A-02b bloqueada por contrato TF-modelo.
6. M-01 requiere evidencia física y contrato TF-modelo completo.
7. Cámara/RGB-D ausente y pipeline de percepción no implementado.
8. `TF_UNAVAILABLE` intermitente de Gazebo no resuelto; no se extrapola al
   hardware.

## 13. Matriz final

```text
AVANCE3_T13_LECTURA_FISICA=NOT_VALIDATED
AVANCE3_T14_PRUEBA_ARTICULAR=BLOCKED
AVANCE3_T14_CARTESIANO=BLOCKED
AVANCE3_T15_RGBD=NOT_VALIDATED
PREPARADO_PARA_CONEXION=SI
LECTURA_FISICA_VALIDADA=NO
PREPARADO_PARA_PRUEBA_ARTICULAR=NO
PREPARADO_PARA_MOVIMIENTO_CARTESIANO=BLOCKED
MOVIMIENTO_FISICO_VALIDADO=NO
RGBD_VALIDADA=NO
HARDWARE_OUTPUT_ENABLED=false
ARMED=false
MOVIMIENTO_FISICO_EJECUTADO=NO
```

`HARDWARE_OUTPUT_ENABLED=false` y `ARMED=false` indican la configuración
preservada y el estado inicial obligatorio; al no existir runtime físico, su
lectura efectiva sigue incluida en el preflight pendiente. El Avance 3 **no se
declara terminado**.
