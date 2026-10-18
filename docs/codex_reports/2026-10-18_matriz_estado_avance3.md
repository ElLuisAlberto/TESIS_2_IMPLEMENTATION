# Matriz de estado verificable — Avance 3

Fecha: 2026-10-18

## Criterio de lectura

- **Implementado:** hay código, lanzamiento, configuración o prueba fuente en el repositorio.
- **Validado realmente:** hay un resultado de ejecución, log, métrica, rosbag, vídeo o evidencia equivalente. La sola existencia de una prueba o de un plan no valida el comportamiento.
- **Planificado:** aparece en los documentos de planificación, sin una implementación o evidencia que lo demuestre.

En esta revisión no se ejecutaron pruebas, simulación ni hardware. `evidence/codex_reports/` no contiene archivos. La evidencia histórica localizada es `evidence/avance2/scenario_matrix_e01_e08.csv`; contiene escenarios E01–E08 con veredictos `PASS`, pero no acredita las funciones nuevas de Avance 3 ni una ejecución física.

| Fase | Estado actual | Implementado | Validado realmente | Solo planificado |
| --- | --- | --- | --- | --- |
| A. Simulación e interfaz cartesiana | PARCIAL | Sí | No | La secuencia completa de pruebas y corrección |
| B. Integración física en solo lectura | NO VERIFICADO | Sí | No | Primera conexión y lecturas reales |
| C. Movimiento físico lento por joints | NO VERIFICADO | Sí | No | Sesiones de micromovimiento supervisado |
| D. Cámara RGB-D, TF y nube de puntos | NO VERIFICADO | Sí | No | Caracterización y calibración física |
| E. Filtrado, reducción y agrupamiento | PARCIAL | Sí | No | Métricas con nubes grabadas o cámara real |
| F. Percepción + supervisor + brazo físico | NO VERIFICADO | Parcialmente | No | Integración extremo a extremo física |
| G. Evidencias, métricas y demostración final | PARCIAL | Sí | Solo evidencia histórica de Avance 2 | Evidencia y demostración de Avance 3 |

## A. Simulación e interfaz cartesiana — PARCIAL

**Evidencia verificable.**

- Implementación de las tres rutas: `src/thesis_simulation/thesis_simulation/cartesian_command_adapter.py`, `cartesian_jog_adapter.py` y `cartesian_trajectory_adapter.py`.
- Nodos declarados en `src/thesis_simulation/setup.py` y lanzados en `src/thesis_simulation/launch/thesis_system.launch.py`.
- Interfaz X/Y/Z y mensajes en `src/thesis_ui/thesis_ui/joint_control_gui.py`, `src/thesis_interfaces/msg/CartesianCommand.msg` y `CartesianTrajectoryCommand.msg`.
- Pruebas fuente: `src/thesis_core/test/test_cartesian_kinematics.py`, `test_cartesian_path.py`, `src/thesis_simulation/test/test_cartesian_jog_model.py`, `test_cartesian_trajectory_model.py` y `src/thesis_ui/test/test_gui_state.py`.
- El diagnóstico y el plan de pruebas documentan hipótesis y comandos, pero declaran que no se ejecutaron.

**Faltantes exactos.** Resultado de compilación y pruebas, observación de TF y `/joint_states`, trazas de los tópicos cartesianos, y una demostración en Gazebo que confirme X, Y y Z. La causa del comportamiento anómalo sigue sin confirmación experimental.

**Paquetes y archivos.** `thesis_interfaces`, `thesis_core`, `thesis_simulation`, `thesis_ui`; los archivos anteriores y `src/thesis_simulation/launch/thesis_system.launch.py`.

**Siguiente acción mínima.** Ejecutar las pruebas unitarias de `2026-10-18_plan_pruebas_cartesiano.md`, sin lanzar simulación ni publicar comandos.

**Brazo conectado / autorización.** No requiere brazo. No requiere autorización manual para pruebas unitarias; una futura prueba de movimiento en Gazebo debe autorizarse por separado según el plan.

## B. Integración física en solo lectura — NO VERIFICADO

**Evidencia verificable.**

- Adaptador C++ y configuración física: `src/thesis_hardware_bridge/src/jaco_hardware_node.cpp` y `src/thesis_hardware_bridge/config/jaco_hardware.yaml`.
- Lanzadores separados de Gazebo: `src/thesis_hardware_bridge/launch/jaco_hardware.launch.py` y `src/thesis_hardware/launch/jaco_physical_system.launch.py`.
- Ambos lanzadores declaran `hardware_output_enabled` con valor predeterminado `false`.
- Nodo y contratos de disponibilidad: `src/thesis_hardware/thesis_hardware/hardware_readiness_node.py`, `readiness_contract.py`, `readiness_logging.py`, con pruebas `test_readiness_contract.py` y `test_readiness_logging.py`.
- `docs/implementation_notes/kinova_hardware_bridge.md` describe explícitamente la primera conexión en solo lectura.

**Faltantes exactos.** Ningún registro de enumeración USB real, `/joint_states`, TF, diagnósticos, frecuencia de lectura, desconexión/reconexión o telemetría física. La documentación describe el procedimiento, no una sesión ejecutada.

**Paquetes y archivos.** `thesis_hardware_bridge`, `thesis_hardware`, `thesis_description`, `thesis_core`, `thesis_telemetry`, `thesis_validation`; los lanzadores y archivos de disponibilidad indicados.

**Siguiente acción mínima.** Revisar y ejecutar solo las pruebas unitarias de disponibilidad sin conectar el brazo; después solicitar autorización para una sesión física de solo lectura con salida deshabilitada.

**Brazo conectado / autorización.** El objetivo de fase requiere brazo conectado. Requiere autorización manual para conectar el equipo e iniciar la pila física, incluso con salida deshabilitada.

## C. Movimiento físico lento y supervisado por joints — NO VERIFICADO

**Evidencia verificable.**

- El adaptador físico documenta doble habilitación: parámetro `hardware_output_enabled` y servicio `/thesis/hardware/set_armed` en `docs/implementation_notes/kinova_hardware_bridge.md`.
- Existen validadores `src/thesis_hardware/thesis_hardware/point_settling_validator.py` y `continuous_control_validator.py`, instalados desde `src/thesis_hardware/setup.py`.
- El lanzamiento físico integra `jaco_hardware_node`, `safety_supervisor`, `horizon_preview`, `proximity_monitor`, `hardware_readiness` y `joint_gui` en `src/thesis_hardware/launch/jaco_physical_system.launch.py`.
- La evidencia de Avance 2 contiene decisiones de supervisión y métricas de escenarios E01–E08, pero los campos de controlador aparecen como `UNAVAILABLE`; por tanto, no acreditan ejecución del Jaco2 físico.

**Faltantes exactos.** Resultado de armado/desarmado real, una orden articular lenta, HOLD, parada/reducción frente a obstáculo, estados de ejecución correlacionados, vídeo y telemetría física. Tampoco hay acta de autorización manual.

**Paquetes y archivos.** `thesis_hardware_bridge`, `thesis_hardware`, `thesis_core`, `thesis_ui`, `thesis_telemetry`, `thesis_validation`; `jaco_physical_system.launch.py`, `jaco_hardware_node.cpp`, validadores físicos y `safety_supervisor_node.py`.

**Siguiente acción mínima.** No mover el brazo: completar B y revisar de forma estática los parámetros conservadores, el desarmado y los validadores antes de solicitar una sesión de micromovimiento.

**Brazo conectado / autorización.** Sí. Requiere autorización manual explícita antes de cualquier movimiento y antes de armar la salida física.

## D. Cámara RGB-D, TF y nube de puntos — NO VERIFICADO

**Evidencia verificable.**

- Lanzadores de cámara y procesamiento: `thesis_perception/launch/d435i_standalone.launch.py` y `perception_processing.launch.py`.
- Nodos compilados en `thesis_perception/CMakeLists.txt`: `pointcloud_preprocessor_node`, `obstacle_extractor_node` y `perception_health_node`.
- Configuración de nube, QoS, TF y salud en `thesis_perception/config/perception.yaml`.
- Prueba de contrato: `thesis_perception/test/test_perception_safety_contract.py` verifica que la salida segura por defecto es `/thesis/perception/obstacle_candidate` y que la TF de cámara a `world` es opt-in.

**Faltantes exactos.** No hay capturas RGB/de profundidad, `PointCloud2`, árbol TF real, calibración de extrínseca, mediciones de frecuencia/latencia ni evidencia de reinicio de D435i. No hay resultados de la prueba de contrato.

**Paquetes y archivos.** `thesis_perception`, `thesis_description`, `thesis_interfaces`; lanzadores, YAML, C++ y prueba indicados.

**Siguiente acción mínima.** Ejecutar pruebas de contrato y revisar la configuración sin iniciar la cámara; luego solicitar autorización para una caracterización de cámara sin el brazo conectado.

**Brazo conectado / autorización.** No requiere brazo para la caracterización inicial. Requiere autorización manual para conectar y usar la cámara física.

## E. Filtrado, reducción y agrupamiento de obstáculos — PARCIAL

**Evidencia verificable.**

- Preprocesamiento con recorte, TF y voxelización en `thesis_perception/src/pointcloud_preprocessor_node.cpp`; parámetros `enable_voxel` y `voxel_leaf_size` en `thesis_perception/config/perception.yaml`.
- Agrupamiento PCL y selección de candidato en `thesis_perception/src/obstacle_extractor_node.cpp`; incluye tolerancia y límites de tamaño de cluster.
- Control de frescura y frame en `thesis_perception/src/perception_health_node.cpp`.
- El contrato aguas abajo existe en `src/thesis_core/thesis_core/obstacle_input_bridge.py`, `proximity_monitor_node.py` y en `docs/implementation_notes/obstacle_input_interface.md`.

**Faltantes exactos.** No hay pruebas unitarias específicas para resultados de filtrado o clusters, nubes grabadas, resultados de reducción, tasa de falsos positivos/negativos, latencia extremo a extremo ni remapeo validado hacia `/thesis/obstacle_input`.

**Paquetes y archivos.** `thesis_perception`, `thesis_core`, `thesis_interfaces`, `thesis_validation`; fuentes de preprocesamiento/extracción/salud, `perception.yaml`, `obstacle_input_bridge.py` y `proximity_monitor_node.py`.

**Siguiente acción mínima.** Crear o identificar un conjunto de nubes de prueba y pruebas reproducibles de nube vacía, ruido, obstáculo único y dato obsoleto, sin conectar salida al supervisor físico.

**Brazo conectado / autorización.** No para pruebas offline. Requiere autorización manual únicamente si se usa la cámara física en vivo.

## F. Integración percepción + supervisor + brazo físico — NO VERIFICADO

**Evidencia verificable.**

- El procesador de percepción publica por defecto en `/thesis/perception/obstacle_candidate`, no directamente en la entrada del supervisor.
- `src/thesis_core/thesis_core/obstacle_input_bridge.py` valida y transforma obstáculos antes de `/thesis/obstacle_in_model_frame`; `proximity_monitor_node.py` y `safety_supervisor_node.py` implementan la cadena preventiva.
- El lanzamiento físico acepta `obstacle_input_topic` y crea `obstacle_input_bridge`, `proximity_monitor` y `safety_supervisor` en `src/thesis_hardware/launch/jaco_physical_system.launch.py`.
- `docs/implementation_notes/obstacle_input_interface.md` documenta la interfaz sintética y explícitamente indica que todavía no convierte las nubes D435i en geometría validada.

**Faltantes exactos.** No existe lanzamiento o evidencia que remapee percepción real a `/thesis/obstacle_input`; no hay rosbag sincronizado de nube, TF, obstáculo, decisión y juntas; no hay pruebas de reducción, STOP o FAIL-SAFE por pérdida de cámara/TF con brazo físico.

**Paquetes y archivos.** `thesis_perception`, `thesis_core`, `thesis_hardware`, `thesis_hardware_bridge`, `thesis_telemetry`, `thesis_validation`, `thesis_interfaces`; `perception_processing.launch.py`, `jaco_physical_system.launch.py`, puente, monitor y supervisor.

**Siguiente acción mínima.** Completar y validar D y E; después preparar una integración pasiva con `hardware_output_enabled:=false` y sin mover el brazo.

**Brazo conectado / autorización.** La fase final requiere brazo conectado. La integración pasiva requiere autorización manual para conectarlo; cualquier salida física exige autorización explícita adicional.

## G. Evidencias, métricas y demostración final — PARCIAL

**Evidencia verificable.**

- Infraestructura de telemetría: `src/thesis_telemetry/thesis_telemetry/telemetry_logger.py`, `telemetry_math.py`, su lanzamiento y `test_telemetry_math.py`.
- Infraestructura de validación: `src/thesis_validation/thesis_validation/scenario_recorder.py`, `evaluation.py`, `summary.py`, con `test_contracts.py`, `test_evaluation.py` y `test_scenario_recorder.py`.
- Evidencia histórica: `evidence/avance2/scenario_matrix_e01_e08.csv`, fechada el 2026-09-25, con métricas de distancia, escala, latencia y veredictos de escenarios.
- Los documentos de `docs/codex_reports/` son planificación y diagnóstico; no son evidencias de ejecución.

**Faltantes exactos.** No hay resultados de pruebas de Avance 3, registros en `evidence/codex_reports/`, rosbag, vídeo, manifiesto de versiones, tabla de métricas de cámara o física, ni demostración final reproducible. No hay evidencia de los escenarios de las fases A–F.

**Paquetes y archivos.** `thesis_telemetry`, `thesis_validation`, `thesis_core`, `thesis_perception`, `thesis_hardware`, `thesis_simulation`; los archivos de telemetría/validación indicados y `evidence/avance2/`.

**Siguiente acción mínima.** Definir el manifiesto de evidencias de Avance 3 y ejecutar primero las pruebas no físicas de A, guardando su salida fechada en `evidence/codex_reports/`.

**Brazo conectado / autorización.** No para consolidar evidencias de simulación. Requiere brazo y autorización manual solo para las evidencias de las fases físicas C y F y la demostración física.

## Conclusión operativa

El repositorio contiene una base implementada para las siete fases, pero ninguna fase de Avance 3 tiene evidencia de ejecución archivada. La prioridad verificable es A en modo no físico; B, C, D y F no deben calificarse como validadas hasta que existan registros de sus entornos reales. La integración cartesiana física continúa fuera de alcance y no es un faltante bloqueante para este avance.
