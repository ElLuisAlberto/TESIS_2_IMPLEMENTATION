# Plan maestro — Avance 3

Fecha: 2026-10-18  
Objetivo: completar y demostrar la seguridad predictiva del Kinova Jaco2 en ROS 2 Humble con separación estricta entre simulación y hardware.

## Reglas de ejecución

- El control cartesiano físico queda fuera de alcance de este Avance 3.
- Ningún lanzamiento debe mezclar Gazebo con el brazo físico.
- La primera fase física es exclusivamente de lectura y mantiene `hardware_output_enabled:=false`.
- Todo movimiento físico requiere autorización manual explícita, brazo despejado y una prueba de la fase anterior aprobada.
- Las evidencias de ejecución se guardan en `evidence/codex_reports/` con fecha y descripción. Los planes y diagnósticos permanecen en `docs/codex_reports/`.

## Resumen de alcance

| Fase | Alcance | Obligatoria para Avance 3 | Brazo conectado | Autorización manual |
| --- | --- | --- | --- | --- |
| A | Simulación e interfaz cartesiana | Sí | No | No, salvo una prueba de movimiento simulado posterior |
| B | Integración física en solo lectura | Sí | Sí | Sí, para conectar e iniciar la pila física sin salida |
| C | Movimiento físico lento por joints | Sí, si B aprueba | Sí | Sí, antes de cada sesión y movimiento |
| D | Cámara RGB-D, TF y nube de puntos | Sí | Cámara sí; brazo no para caracterización inicial | Sí, para usar la cámara y su entorno físico |
| E | Filtrado, reducción y agrupamiento | Sí | No para pruebas grabadas; cámara sí para validación en vivo | Sí, para validación en vivo |
| F | Percepción + supervisor + brazo físico | Sí, si C, D y E aprueban | Sí | Sí, sesión controlada |
| G | Evidencias y demostración final | Sí | Según el escenario final | Sí, si incluye el brazo |

## A. Simulación e interfaz cartesiana

**Objetivo.** Determinar y corregir la causa del comportamiento X/Y/Z en simulación, manteniendo separadas las rutas de objetivo puntual, JOG y trayectoria larga.

**Paquetes y nodos.** `thesis_interfaces`, `thesis_core`, `thesis_simulation` y `thesis_ui`; `cartesian_command_adapter`, `cartesian_jog_adapter`, `cartesian_trajectory_adapter`, `safety_supervisor`, `horizon_preview`, `system_readiness` y `joint_gui` en modo simulación.

**Cambios de código previsibles.** Solo después de ejecutar y registrar el plan cartesiano: correcciones acotadas de sincronización TF/`/joint_states`, parámetros de IK, límites de paso o duración, condiciones de GUI y pruebas de regresión. No se traslada código cartesiano al paquete físico.

**Pruebas requeridas.** Ejecutar primero `2026-10-18_plan_pruebas_cartesiano.md`: pruebas unitarias de IK, JOG, trayectoria y GUI; después, observación pasiva con `simulation_output_enabled:=false`; por último, si se autoriza, una prueba de movimiento exclusivamente en Gazebo para cada eje X/Y/Z y para un rechazo controlado.

**Evidencias.** Resultado de `colcon test`, configuración del lanzamiento, lista de nodos y tópicos, capturas de `/joint_states`, TF, `/thesis/cartesian_candidate`, `/thesis/candidate_command`, `/thesis/jog_intent`, decisiones del supervisor y trayectoria de Gazebo/RViz.

**Criterio de aprobación.** Los tres ejes se desplazan con signo y magnitud esperados desde una postura válida; las órdenes inválidas se rechazan con razón trazable; no hay pérdida de TF ni estados obsoletos; las pruebas de regresión pasan.

**Riesgo de seguridad.** Bajo: el riesgo es un diagnóstico falso por simulación mal configurada. Se controla deshabilitando salida física y manteniendo los lanzamientos aislados.

**Brazo conectado / autorización.** Brazo no conectado. No requiere autorización manual para pruebas unitarias y observación pasiva; cualquier movimiento en Gazebo se solicita y registra por separado.

**Estado.** Obligatoria. No diferible porque valida la interfaz y el comportamiento seguro antes de cualquier expansión.

## B. Integración física en solo lectura

**Objetivo.** Verificar comunicación, estados articulares, TF, disponibilidad y telemetría del Jaco2 sin emitir ninguna orden de movimiento.

**Paquetes y nodos.** `thesis_hardware_bridge`, `thesis_hardware`, `thesis_description`, `thesis_core`, `thesis_telemetry` y `thesis_validation`; `jaco_hardware_node`, `robot_state_publisher`, `hardware_readiness`, `safety_supervisor` y `telemetry_logger`.

**Cambios de código previsibles.** Adaptación de contratos de disponibilidad, nombres de juntas, tasas de lectura, sellos temporales, diagnósticos y configuración de lanzadores. Ningún cambio debe habilitar salida física por defecto.

**Pruebas requeridas.** Lanzar solamente la pila física con `hardware_output_enabled:=false`; observar `/joint_states`, TF, `/thesis/hardware/connected`, `/thesis/hardware/armed`, estado de disponibilidad, diagnósticos de comunicación y telemetría. Ejecutar las pruebas de `thesis_hardware` y `thesis_validation` antes de conectar el brazo.

**Evidencias.** Registro de arranque, listado de nodos y tópicos, muestra temporal de estados, árbol TF, tasa de actualización, eventos de desconexión/reconexión y telemetría CSV o resumen.

**Criterio de aprobación.** Las seis juntas se leen de forma estable, TF es consistente, `armed` permanece falso, no aparecen mensajes en la ruta de salida y una desconexión se detecta y deja el sistema no disponible.

**Riesgo de seguridad.** Medio: una configuración errónea podría habilitar salida. Se controla verificando de forma explícita el parámetro de salida falso, manteniendo la GUI física sin armado y evitando publicadores de comandos.

**Brazo conectado / autorización.** Sí. Requiere autorización manual para conectar el equipo y ejecutar la pila de solo lectura.

**Estado.** Obligatoria. No diferible: es la base para demostrar integración física segura.

## C. Movimiento físico lento y supervisado por joints

**Objetivo.** Demostrar que el supervisor preventivo limita o detiene movimientos articulares físicos lentos y trazables. El control cartesiano físico permanece excluido.

**Paquetes y nodos.** `thesis_hardware_bridge`, `thesis_hardware`, `thesis_core`, `thesis_ui`, `thesis_telemetry` y `thesis_validation`; `jaco_hardware_node`, `safety_supervisor`, `horizon_preview`, `proximity_monitor`, `hardware_readiness`, `joint_gui`, `telemetry_logger`, `point_settling_validator` y `continuous_control_validator` cuando aplique.

**Cambios de código previsibles.** Límites físicos conservadores configurables, armado explícito, interbloqueos de disponibilidad, watchdog de comunicación, límite de velocidad y validación de parada/HOLD. No se añaden adaptadores cartesianos al lanzamiento físico.

**Pruebas requeridas.** Después de B aprobada y autorización explícita: validar armado/desarmado, un único objetivo articular corto y lento, HOLD, rechazo por falta de disponibilidad, reducción de velocidad y parada ante un obstáculo controlado. Una persona supervisa y puede desarmar de inmediato.

**Evidencias.** Parámetros de seguridad, confirmación de armado, comandos e identificadores, `/joint_states`, `/thesis/command_decision`, `/thesis/execution_control`, distancia mínima, escala de velocidad, vídeo, telemetría y resultado de validadores.

**Criterio de aprobación.** Cada movimiento termina dentro de tolerancia o se detiene de forma segura; el supervisor publica razones y escala coherentes; pérdida de disponibilidad, comunicación u obstáculo produce HOLD/desarmado; no hay movimiento no solicitado.

**Riesgo de seguridad.** Alto: movimiento físico y colisión. Se controla con espacio despejado, velocidad y recorrido mínimos, operador presente, parada accesible, un solo comando activo, salida física armada solo durante la prueba y desarmado posterior.

**Brazo conectado / autorización.** Sí. Requiere autorización manual explícita antes de cada sesión y cada grupo de movimientos.

**Estado.** Obligatoria para la demostración física. El control cartesiano físico se difiere sin bloquear esta fase.

## D. Integración de cámara RGB-D, TF y nube de puntos

**Objetivo.** Obtener una nube RGB-D estable y trazable al marco del sistema, sin acoplarla todavía al brazo en movimiento.

**Paquetes y nodos.** `thesis_perception`, `thesis_description` y `thesis_interfaces`; lanzadores `d435i_standalone.launch.py` y `perception_processing.launch.py`; `realsense2_camera`, `static_transform_publisher`, `pointcloud_preprocessor`, `obstacle_extractor` y `perception_health`.

**Cambios de código previsibles.** Calibración y documentación de la extrínseca cámara-mundo, parámetros de QoS, tasa/resolución, validación de sellos temporales, contrato de frames y chequeos de salud. La publicación de `world -> camera` conserva opt-in explícito hasta medir la pose.

**Pruebas requeridas.** Caracterizar cámara sin brazo en movimiento: verificar imagen/profundidad/nube, TF de cámara, frecuencia, latencia, frames inválidos y reinicio de cámara. Probar primero con tema seguro `/thesis/perception/obstacle_candidate`, sin remapear a la entrada del supervisor.

**Evidencias.** Capturas RGB y profundidad, `ros2 topic hz`, árbol TF, muestras de `PointCloud2`, latencia, configuración de cámara y calibración/extrínseca fechada.

**Criterio de aprobación.** La nube se publica de forma estable, tiene frame y sello válidos, la transformación es verificable y el nodo de salud detecta pérdida o envejecimiento de datos.

**Riesgo de seguridad.** Bajo para el brazo; medio para validez de percepción. Una TF errónea puede convertirse en riesgo alto cuando se conecte al supervisor, por lo que esta fase no publica aún a `/thesis/obstacle_input`.

**Brazo conectado / autorización.** No para caracterización inicial; cámara conectada sí. Requiere autorización manual para usar el equipo de cámara en el entorno físico.

**Estado.** Obligatoria. La calibración fina de IMU o cámara móvil puede diferirse si la cámara es fija y la extrínseca queda validada.

## E. Filtrado, reducción y agrupamiento de obstáculos

**Objetivo.** Convertir la nube en obstáculos compactos, conservadores y aptos para el monitor de proximidad, con latencia y trazabilidad medibles.

**Paquetes y nodos.** `thesis_perception`, `thesis_core`, `thesis_interfaces` y `thesis_validation`; `pointcloud_preprocessor`, `obstacle_extractor`, `perception_health`, `obstacle_input_bridge` y `proximity_monitor`.

**Cambios de código previsibles.** Ajuste de recorte espacial, voxelización, eliminación de suelo/ruido, agrupamiento, radio/incertidumbre de obstáculos, caducidad de datos y conversión de frame. Cualquier remapeo a `/thesis/obstacle_input` se hace explícito y se prueba primero sin movimiento.

**Pruebas requeridas.** Pruebas unitarias y con nubes grabadas: nube vacía, obstáculo único, múltiples obstáculos, ruido, suelo, dato obsoleto y obstáculo fuera del volumen. Medir tasa de reducción, falsos positivos/negativos, error de posición y latencia extremo a extremo.

**Evidencias.** Nubes de entrada y salida o rosbag, parámetros, tablas de puntos antes/después, grupos detectados, métricas de latencia, tasa de reducción, precisión y eventos de salud.

**Criterio de aprobación.** Los obstáculos relevantes se mantienen con margen conservador; los datos obsoletos no generan decisiones activas; la latencia queda dentro del presupuesto definido para el supervisor y los falsos negativos se documentan como cero en los escenarios de referencia.

**Riesgo de seguridad.** Medio-alto: subestimar o perder un obstáculo puede habilitar un movimiento inseguro. Se controla con márgenes, caducidad conservadora, validación offline y sin conectar aún la salida al brazo.

**Brazo conectado / autorización.** No para pruebas grabadas; opcional para validación de escena fija. Requiere autorización manual si se usa cámara en vivo.

**Estado.** Obligatoria. La optimización estética de visualización o modelos de clasificación avanzados puede diferirse.

## F. Integración percepción + supervisor + brazo físico

**Objetivo.** Demostrar que obstáculos detectados por RGB-D influyen de forma segura en la supervisión de movimientos articulares físicos lentos.

**Paquetes y nodos.** `thesis_perception`, `thesis_hardware`, `thesis_hardware_bridge`, `thesis_core`, `thesis_telemetry`, `thesis_validation` y `thesis_interfaces`; `obstacle_extractor`, `perception_health`, `obstacle_input_bridge`, `proximity_monitor`, `safety_supervisor`, `horizon_preview`, `jaco_hardware_node`, `hardware_readiness` y `telemetry_logger`.

**Cambios de código previsibles.** Contrato explícito de entrada de obstáculos, propagación de frame/sello/incertidumbre, rechazo de percepción obsoleta, límites conservadores, métricas correlacionadas y un lanzamiento físico separado. No se habilita control cartesiano físico.

**Pruebas requeridas.** Con C, D y E aprobadas: integrar primero con `hardware_output_enabled:=false` y observar decisiones; después, con autorización, ejecutar objetivos articulares lentos mientras se presentan escenarios de obstáculo lejos, cercano, aproximación y pérdida de cámara. Incluir prueba de FAIL-SAFE por nube o TF obsoletas.

**Evidencias.** Rosbag sincronizado de cámara/obstáculos/TF/estados, decisiones del supervisor, escala de velocidad, distancia mínima, eventos de disponibilidad, vídeo de la escena y resumen automático de escenarios.

**Criterio de aprobación.** Obstáculo lejano permite el movimiento dentro de política; obstáculo cercano reduce o detiene antes de invadir el margen; pérdida de percepción o TF activa la política conservadora; toda decisión se puede reconstruir desde la evidencia.

**Riesgo de seguridad.** Alto: errores de frame, latencia o detección inciden directamente sobre un brazo en movimiento. Se controla con velocidad mínima, alcance reducido, operador, zona despejada, validación inicial sin salida y margen de seguridad adicional.

**Brazo conectado / autorización.** Sí. Requiere autorización manual explícita para cada sesión con salida física habilitada.

**Estado.** Obligatoria para demostrar la tesis. El uso de múltiples cámaras o seguimiento dinámico sofisticado puede diferirse.

## G. Evidencias, métricas y demostración final

**Objetivo.** Consolidar una demostración reproducible que muestre trazabilidad entre percepción, predicción, decisión preventiva y ejecución.

**Paquetes y nodos.** `thesis_telemetry`, `thesis_validation`, `thesis_core`, `thesis_perception`, `thesis_hardware` y `thesis_simulation`; `telemetry_logger`, `scenario_recorder`, herramientas de `evaluation` y `summary`, más los nodos de cada escenario.

**Cambios de código previsibles.** Plantillas de escenarios, campos de telemetría, cálculo de métricas, exportación de resúmenes y verificación de integridad de evidencias. Deben ser aditivos y no alterar los umbrales de seguridad sin una justificación documentada.

**Pruebas requeridas.** Repetir escenarios de referencia en simulación y, cuando corresponda, físicos: sin obstáculo, obstáculo estático, aproximación, parada, reducción, pérdida de percepción y recuperación. Ejecutar pruebas de contratos, evaluación, grabador de escenarios y matemáticas de telemetría.

**Evidencias.** En `evidence/codex_reports/`: comandos y versiones, configuraciones, resultados de prueba, logs, CSV, rosbag, capturas TF/RViz, tablas de métricas, vídeos y acta de aprobación manual. En `docs/codex_reports/`: protocolo, diagnóstico y conclusión.

**Criterio de aprobación.** Cada escenario tiene evidencia reproducible; se reportan al menos latencia de percepción a decisión, distancia mínima, escala de velocidad, tiempo de parada, tasa de falsos negativos de los escenarios de referencia y cumplimiento de HOLD/FAIL-SAFE. La demostración final se puede repetir sin cambios manuales no documentados.

**Riesgo de seguridad.** Medio durante preparación y alto si incluye brazo. Se controla reutilizando solo escenarios aprobados, revisando la configuración antes de cada ejecución y conservando autorización manual para toda salida física.

**Brazo conectado / autorización.** El brazo solo se conecta para los escenarios físicos ya aprobados. Requiere autorización manual para toda demostración con movimiento.

**Estado.** Obligatoria. La edición audiovisual avanzada y paneles de visualización no esenciales pueden diferirse después de conservar los datos crudos y las métricas.

## Tareas diferibles sin bloquear Avance 3

- Control cartesiano en hardware.
- Trayectorias cartesianas físicas y JOG cartesiano físico.
- Múltiples cámaras, cámara móvil e integración IMU no requerida por una cámara fija.
- Clasificación semántica avanzada, optimización de GPU y visualizaciones no necesarias para la decisión preventiva.
- Ampliación del rango de velocidad, automatización de demostraciones y mejoras de interfaz que no alteren la trazabilidad ni la política de seguridad.

## Condición de cierre del Avance 3

El avance queda listo cuando A, B, C, D, E, F y G tengan criterio de aprobación satisfecho, evidencias archivadas y configuraciones reproducibles. Cualquier fallo en TF, frescura de datos, supervisor, telemetría o disponibilidad física vuelve la ejecución a la fase correspondiente; no se compensa habilitando movimiento físico adicional.
