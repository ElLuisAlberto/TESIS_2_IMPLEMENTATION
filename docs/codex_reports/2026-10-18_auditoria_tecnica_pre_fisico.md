# Auditoría técnica estática previa a hardware físico

Fecha: 2026-10-18  
Método: revisión estática de código fuente, launch, YAML, mensajes y pruebas
relacionadas. No se ejecutaron pruebas, compilación, ROS, simulación ni acceso
al Jaco físico.

## 1. Veredicto general

**NO APTO PARA FÍSICO** para habilitar movimiento.

La ruta física tiene defensas importantes de armado, límites, watchdogs y
parada, pero existen vías estáticas verificables para aceptar una orden puntual
o un control de ejecución antiguos. También hay una vía para considerar fresca
una geometría recibida tarde y no se impone que la geometría usada por el
supervisor esté expresada en `world` ni se verifica el desfase modelo--TF. Esas
condiciones pueden invalidar una decisión preventiva o iniciar movimiento tras
un armado posterior.

Estado de evidencia usado en este informe:

- **IMPLEMENTADO:** comportamiento visible en el código citado.
- **PROBADO:** existe la prueba unitaria indicada; no fue ejecutada en esta
  auditoría. Cuando se menciona un resultado, procede de los informes previos.
- **VALIDADO EN SIMULACIÓN:** el cierre
  `2026-10-18_cierre_simulacion_cartesiana.md` confirmó una Z de +20 mm con
  `ALLOW`, `SUCCEEDED` y el validador en `OK`; no valida USB, armado, driver,
  percepción real ni parada física.
- **NO VERIFICADO:** toda interacción con Jaco, driver Kinova, cableado,
  tiempos físicos, botón de parada y RGB-D real.

## 2. Hallazgos reales

| Severidad | Hallazgo | Ámbito | Estado |
| --- | --- | --- | --- |
| Crítico | La ruta puntual no rechaza comandos ni controles de ejecución obsoletos; el `command_id` tampoco se valida como único/no vacío. | Físico; la omisión existe también en la ruta común del supervisor. | IMPLEMENTADO; las pruebas de frescura cubren solo JOG. |
| Alto | El supervisor y el monitor de proximidad usan hora de recepción, no el timestamp de origen, para aceptar estado/proximidad. | Ambos; crítico para percepción física. | IMPLEMENTADO; el contrato de entrada sí se prueba, la cadena completa no. |
| Alto | No se verifica el frame de `ProximityStatus` ni la concordancia TF--modelo antes de predecir; el parámetro de tolerancia TF está declarado pero no se aplica. | Ambos; especialmente físico. | IMPLEMENTADO; NO VERIFICADO con TF físico. |
| Alto | El armado del puente no depende de `system_ready`, proximidad ni supervisor; `hardware_readiness` solo informa estado y no interbloquea el puente. | Físico. | IMPLEMENTADO; NO VERIFICADO en USB. |
| Medio | El lanzamiento físico no inicia telemetría ni recorder y el logger CSV no suscribe estados `ExecutionTrajectory`. | Físico. | IMPLEMENTADO; NO VERIFICADO en una sesión física. |

### C-01 — Replay/obsolescencia en órdenes puntuales y control de ejecución

- **Severidad:** Crítico.
- **Ruta y línea aproximada:**
  `thesis_core/safety_supervisor_node.py:1902-2035` recibe y reenvía
  `JointCommand` puntual sin comprobar `msg.stamp`; incluso calcula
  `input_latency_ms` en `2023-2034`, pero no lo usa como criterio.
  `thesis_hardware_bridge/src/jaco_hardware_node.cpp:730-802` acepta el
  `/thesis/supervised_command` sin edad máxima, y `851-887` acepta
  `ExecutionControl` coincidente sin validar su `stamp`. Ninguna de esas
  rutas exige `command_id` no vacío, único o no reutilizado.
- **Condición de activación:** una orden puntual antigua queda en cola o se
  reentrega después de que el brazo sea armado; o un `ALLOW`/`REDUCTION`
  antiguo coincide con un `command_id` reutilizado. La orden pasa los límites,
  la predicción y el armado porque su antigüedad no participa en ninguna
  decisión.
- **Impacto técnico:** puede empezar movimiento físico no solicitado. En
  particular, el puente crea un objetivo con velocidad cero y espera el primer
  `ExecutionControl`; un `ALLOW` viejo con el mismo identificador puede ser
  ese primer control y liberar movimiento. Un `STOP` viejo es conservador, pero
  un `ALLOW` viejo no lo es.
- **Evidencia concreta:** el chequeo de edad existe solo para JOG:
  supervisor `838-856` y puente `804-821`; las pruebas
  `test_jog_message_age.py` y `test_bridge_logic.cpp:105-113` cubren ese
  contrato, no el comando puntual ni `ExecutionControl`.
- **Afecta:** físico directamente; ambos comparten la aceptación inicial en
  el supervisor.
- **Corrección mínima propuesta:** antes de predecir/retransmitir un punto y
  antes de aplicar `ExecutionControl`, rechazar `stamp` cero, futuro fuera de
  tolerancia o superior a un límite configurado. Exigir `command_id` no vacío
  y registrar una ventana acotada de IDs terminales/activos para rechazar
  reutilización. El puente debe aplicar el mismo límite, no confiar solo en el
  supervisor.
- **Prueba mínima requerida antes de hardware:** prueba unitaria de ambos
  callbacks con punto y `ExecutionControl` vencidos, futuros y con ID
  reutilizado; deben producir `REJECTED`/`CANCELED`, ráfaga de cero si había
  objetivo activo y cero llamadas de salida. Después, validación USB de banco
  con salida permitida pero desarmada y trazas que demuestren el rechazo.

### A-01 — Proximidad y estado pueden volver a parecer frescos por recepción tardía

- **Severidad:** Alto.
- **Ruta y línea aproximada:** el supervisor renueva la frescura de estado y
  proximidad al recibirlos, sin revisar su sello:
  `safety_supervisor_node.py:354-373`, `539-563` y `1646-1677`.
  El puente de obstáculos valida correctamente el sello de adquisición
  (`obstacle_input_bridge.py:64-102`) y lo conserva (`104-112`), pero
  `proximity_monitor_node.py:142-163` descarta ese sello y usa
  `time.monotonic()` de recepción; después emite una nueva
  `ProximityStatus` con sello actual (`380-392`).
- **Condición de activación:** latencia, cola DDS o reordenamiento entre el
  bridge de obstáculos y el monitor; el monitor puede evaluar y publicar como
  actual una esfera cuya adquisición ya superó `max_age_sec`. El supervisor
  considera fresca la nueva publicación aunque la medición de origen no lo
  sea. El mismo patrón de “hora de recepción” se aplica a `JointState` en el
  supervisor.
- **Impacto técnico:** una predicción puede usar obstáculo o estado físico
  retrasado como si fuese actual, aceptar `ALLOW`/`REDUCTION` indebidos y no
  emitir STOP ante un cambio rápido del entorno.
- **Evidencia concreta:** `Obstacle.msg` documenta que su sello es la hora de
  adquisición; `validate_obstacle()` rechaza datos vencidos y está cubierto por
  `test_obstacle_contract.py`. No existe verificación estática que preserve y
  compare esa edad hasta `ProximityStatus`/supervisor.
- **Afecta:** ambos; es bloqueante para percepción física.
- **Corrección mínima propuesta:** conservar la edad de origen de la esfera
  al evaluar/publicar proximidad (o incluir `source_stamp` explícito en el
  estado) y hacer que el supervisor exija tanto edad de recepción como edad de
  origen. Aplicar el mismo criterio al `JointState` usando su `header.stamp`.
- **Prueba mínima requerida antes de hardware:** prueba de integración con
  mensaje de obstáculo entregado después de su TTL y con `JointState` sellado
  vencido; el monitor/supervisor debe retener o producir STOP, nunca `ALLOW`.

### A-02 — Frame y modelo geométrico no están interbloqueados

- **Severidad:** Alto.
- **Ruta y línea aproximada:** `safety_supervisor_node.py:1646-1703` acepta
  cualquier `ProximityStatus.reference_frame`; `1480-1522` usa sus coordenadas
  de obstáculo como entrada del modelo cinemático en `world`. El parámetro
  `kinematic_tf_tolerance_m` se declara y valida en `152` y `290-297`, pero
  no se usa después. El monitor toma TF más reciente con `Time()` en
  `proximity_monitor_node.py:238-249`, mientras la entrada de obstáculo se
  transforma al tiempo del sello en `obstacle_input_bridge.py:86-99`.
- **Condición de activación:** configuración errónea de frame, desfase entre
  la geometría URDF/modelo y TF de hardware, o una fuente que publique
  `ProximityStatus` con frame distinto. El estado upstream `STOP` también se
  reemplaza por la predicción del modelo en `1697-1703`.
- **Impacto técnico:** una distancia calculada en un frame distinto puede
  tratarse como `world`; un desacuerdo TF--modelo puede hacer que el
  supervisor acepte una trayectoria cuando el monitor está observando una
  proximidad distinta. La pérdida del STOP upstream deja de ser conservadora.
- **Evidencia concreta:** la interfaz expone `reference_frame` y el launch
  físico fija `world` en el bridge (`jaco_physical_system.launch.py:197-203`),
  pero el supervisor no lo verifica ni ejecuta la tolerancia declarada.
  La simulación final valida una postura/TF de Gazebo, no la calibración física
  de las seis cápsulas.
- **Afecta:** ambos; bloqueante para movimiento físico.
- **Corrección mínima propuesta:** rechazar/STOP si
  `reference_frame != 'world'`; no elevar un `STOP` de proximidad a
  `ALLOW`/`WARNING`; implementar la comparación de las cápsulas/efector del
  modelo con TF y usar `kinematic_tf_tolerance_m` como interbloqueo fail-closed.
- **Prueba mínima requerida antes de hardware:** pruebas de frame inválido,
  `STOP` upstream y discrepancia TF--modelo superior a 5 mm; las tres deben
  producir STOP. Luego, lectura física sin movimiento que compare los seis
  segmentos con TF y la geometría calibrada.

### A-03 — Armado sin interbloqueo de disponibilidad de seguridad

- **Severidad:** Alto.
- **Ruta y línea aproximada:** el servicio del puente solo exige permiso de
  salida, conexión y `JointState` fresco:
  `jaco_hardware_node.cpp:935-984`. La GUI habilita ARM con conexión y servicio
  disponibles (`joint_control_gui.py:1982-1992`).
  `hardware_readiness_node.py:46-119` publica `/thesis/system_ready`, pero el
  puente no se suscribe a ese tópico ni a proximidad/supervisor. El launch
  inicia todos los nodos en paralelo (`jaco_physical_system.launch.py:157-320`).
- **Condición de activación:** `hardware_output_enabled:=true` y solicitud de
  armado antes de que proximidad, TF o supervisor hayan publicado datos
  válidos; también es posible mediante llamada directa al servicio, sin GUI.
- **Impacto técnico:** el adaptador USB queda armado cuando el pipeline
  preventivo aún no es demostrablemente operativo. Por sí solo no manda
  velocidad, pero amplifica C-01: una orden previa/atrasada puede tener efecto
  apenas se arma.
- **Evidencia concreta:** `hardware_readiness` es solo un publicador; no hay
  consumidor suyo en el puente. El servicio no inspecciona proximidad ni
  `system_ready`.
- **Afecta:** físico.
- **Corrección mínima propuesta:** hacer que el puente mantenga un latch de
  disponibilidad que exija `connected`, estado y proximidad con sello válido,
  supervisor disponible y `system_ready==true` antes de armar; desarmar/HOLD
  al perderlo. La GUI debe ser una defensa adicional, no el único filtro.
- **Prueba mínima requerida antes de hardware:** pruebas de servicio para
  intentos de ARM sin cada precondición y tras expirar cada una; todas deben
  fallar y confirmar salida desarmada. Ensayo USB solo lectura: verificar que
  el estado armado permanece falso hasta completar el latch.

### M-01 — Trazabilidad física incompleta por composición de lanzamiento

- **Severidad:** Medio.
- **Ruta y línea aproximada:** el launch físico no inicia
  `thesis_telemetry/telemetry_logger` ni `thesis_validation/scenario_recorder`
  (`jaco_physical_system.launch.py:157-320`). Aunque el puente publica
  `ExecutionTrajectory` (`jaco_hardware_node.cpp:1154-1175`), el logger CSV
  solo se suscribe a timing, estado, proximidad, comandos supervisados y
  `ExecutionControl` (`telemetry_logger.py:119-150`), no a
  `/thesis/execution_trajectory`.
- **Condición de activación:** una sesión física se ejecuta con el launch
  estándar, o la telemetría se inicia sin un recorder separado.
- **Impacto técnico:** una decisión puede quedar correlacionada hasta el
  controlador, pero no existe registro CSV obligatorio y persistente del
  terminal `ACCEPTED`/`SUCCEEDED`/`FAILED`/`CANCELED` asociado al mismo
  `command_id`. Esto debilita la evidencia de STOP, fallo y ejecución física.
- **Evidencia concreta:** `scenario_recorder` sí observa
  `/thesis/execution_trajectory`, pero es una herramienta manual y acotada,
  no parte del launch físico (`scenario_recorder.py:40-48, 376-377`).
- **Afecta:** físico; la simulación final tuvo un validador explícito, no esta
  composición de telemetría.
- **Corrección mínima propuesta:** incluir telemetría en el launch físico con
  directorio de sesión obligatorio y añadir la suscripción/filas de
  `ExecutionTrajectory` al logger, manteniendo `command_id` y estado terminal.
- **Prueba mínima requerida antes de hardware:** ejecución mock sin movimiento
  que produzca `DRY_RUN`, `ACCEPTED`, `CANCELED` y `FAILED`; comprobar que cada
  terminal queda persistido una vez y correlacionado con decisión/timing.

## 3. Rutas revisadas sin hallazgo verificable adicional

- **Simulación vs. físico:** el launch de simulación usa
  `simulation_command_adapter`; el launch físico usa exclusivamente
  `jaco_hardware_node`. No se encontró una inclusión del adaptador simulado en
  `jaco_physical_system.launch.py` ni una ruta cartesiana habilitada en GUI
  física; las funciones cartesianas retornan en modo hardware.
- **Salida física por defecto:** ambos launch/config físicos declaran
  `hardware_output_enabled:=false`; el puente responde `DRY_RUN` cuando falta
  permiso o armado. Esto está **IMPLEMENTADO** y no se ha validado con USB.
- **STOP/HOLD básico:** para STOP, fallo de salida, estado vencido, pérdida de
  control y caducidad JOG, el puente borra trayectorias y envía una ráfaga de
  ceros. Las rutas están en `jaco_hardware_node.cpp:650-687` y
  `1039-1105`. La lógica pura de escala STOP y frescura JOG tiene pruebas;
  la parada física real es **NO VERIFICADA**.
- **Límites y velocidad:** el puente vuelve a validar orden canónico, límites
  y velocidad, y limita la salida angular antes de `SendBasicTrajectory`
  (`689-711`, `583-629`). La lógica pura está **PROBADA** en
  `test_bridge_logic.cpp`; el SDK Kinova real es **NO VERIFICADO**.
- **Identidad de dispositivo:** hay bloqueo USB exclusivo y rechazo de más de
  un equipo sin serial esperado (`293-320`, `365-409`). Está
  **IMPLEMENTADO**, sin validación con dispositivos físicos.
- **Ciclo de vida de simulación:** el timeout/reintento de
  `controller_manager` y el manejo de SIGINT se documentan como corregidos y
  las regresiones unitarias existen. Es **VALIDADO EN SIMULACIÓN**; el puente
  físico no usa `controller_manager`, sino el SDK USB, por lo que no se
  extrapola a hardware.
- **Percepción RGB-D:** no existe un paquete `thesis_perception` ni driver
  RGB-D en `src/`; por tanto no hay ruta de percepción real que auditar. La
  entrada genérica `Obstacle` sí exige frame, finitud, incertidumbre y frescura
  en `obstacle_input_bridge`, pero su integración con cámara es
  **NO VERIFICADA**. Sin obstáculos frescos, el supervisor falla cerrado; no
  se encontró una vía que habilite movimiento por ausencia de ese tópico.

## 4. Condiciones correctamente protegidas

- La salida física inicia deshabilitada y el armado runtime requiere permiso
  explícito más conexión/estado fresco.
- El puente acepta solo seis articulaciones canónicas, finitas y dentro de sus
  límites, con límite independiente de velocidad antes de enviar al SDK.
- Un objetivo puntual queda inmóvil hasta el primer `ExecutionControl` y el
  watchdog de control lo cancela tras 0,35 s.
- El JOG comprueba frescura de sello en supervisor y puente, tiene QoS de una
  muestra, watchdog propio y HOLD al caducar.
- La pérdida de estado del Jaco, fallos de lectura repetidos, fallo de envío y
  timeout de objetivo generan HOLD/STOP; los fallos de lectura desarman.
- STOP tiene prioridad en el puente: cualquier STOP recibido borra
  trayectorias, publica ceros y no exige coincidencia de ID para detener.
- El bridge de obstáculos valida finitud, radio, incertidumbre, frame y edad
  de adquisición; usa TF al tiempo de la medida para la transformación.
- La pérdida de proximidad o estado durante monitorización runtime genera STOP
  en el supervisor; la recuperación de velocidad está limitada temporalmente.
- La GUI física no expone el envío cartesiano y desarma de la mejor manera
  posible al cerrarse.

## 5. Checklist de autorización

### A. Conectar Jaco solo lectura

- [ ] Lanzar únicamente con `hardware_output_enabled:=false` y confirmar en
  diagnóstico `output_permitted=false`, `armed=false`.
- [ ] Mantener `mock_hardware:=false`, usar `expected_serial_number` explícito
  y confirmar un único dispositivo/lock USB.
- [ ] No iniciar GUI de armado o fijar `allow_gui_arm_control:=false` para la
  primera conexión.
- [ ] Verificar de forma pasiva que `/joint_states` contiene seis juntas
  canónicas, es estable y su signo/unidad coincide con el modelo/TF.
- [ ] Verificar de forma pasiva `world ->` todos los frames de cápsulas y que
  la pose/base física coincide con la instalación.
- [ ] Mantener salida desarmada; no publicar comandos ni llamar al servicio de
  armado.

**Autorización estática:** condicionada. El diseño de solo lectura está
protegido por defecto, pero sigue **NO VERIFICADO** hasta completar esta
sesión pasiva en el equipo real.

### B. Habilitar movimiento físico lento

- [ ] Corregir y probar C-01, A-01, A-02 y A-03.
- [ ] Añadir y comprobar la trazabilidad de M-01 antes de cualquier movimiento.
- [ ] Validar el latch de armado con todas sus entradas, incluida pérdida de
  proximidad/estado/supervisor.
- [ ] Verificar físicamente la calibración TF--modelo/cápsulas y márgenes antes
  de usar percepción para decidir ALLOW.
- [ ] Ejecutar primero pruebas mock y después una prueba física controlada de
  velocidad mínima, zona despejada, botón de parada accesible y operador
  dedicado; verificar STOP/HOLD y desarmado reales.

**Autorización:** no concedida.

### C. Conectar percepción RGB-D al supervisor

- [ ] Incorporar o identificar el adaptador de percepción real: no hay
  `thesis_perception` en el árbol revisado.
- [ ] Definir y comprobar frame óptico, extrínsecos cámara--`world`, sello de
  adquisición, incertidumbre conservadora y QoS de la fuente.
- [ ] Corregir A-01 y A-02 y probar pérdida, retraso, frame inválido y medida
  futura de la cámara; cada caso debe terminar en STOP/indisponibilidad.
- [ ] Medir y documentar latencia completa cámara--`Obstacle`--proximidad--
  supervisor y ajustar el margen únicamente con evidencia.

**Autorización:** no concedida; no existe implementación de percepción RGB-D
en el alcance revisado.

## 6. Conclusión explícita

Antes de habilitar movimiento físico deben corregirse obligatoriamente C-01,
A-01, A-02 y A-03, y debe quedar trazabilidad terminal mínima de M-01. No es
seguro compensarlos con velocidad menor, supervisión manual o pruebas en el
Jaco: son fallos de aceptación/frescura e interbloqueo.

Puede diferirse la integración RGB-D completa mientras la pila permanezca sin
movimiento físico y la ausencia de proximidad siga fallando cerrada. También
pueden diferirse herramientas de validación de escenario no necesarias para
la primera lectura, siempre que no se afirme trazabilidad de ejecución física.

Está listo únicamente, a nivel estático, el procedimiento de **sesión física
de solo lectura** con salida explícitamente deshabilitada, desarmada y sin
comandos. Esa sesión sigue siendo **NO VERIFICADA** en hardware y debe usarse
para contrastar serial, joint states, TF, signos, geometría y diagnósticos
antes de reconsiderar la autorización de movimiento.
