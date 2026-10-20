# Prioridad de realimentación frente a salida física JACO2

## Hallazgo arquitectónico

`JacoHardwareNode` usa un hilo de control a 100 Hz y un temporizador ROS de feedback a 20 Hz. Antes de este cambio, `control_tick()` retenía `state_mutex_` mientras calculaba y ejecutaba `SendBasicTrajectory`. `feedback_tick()` requiere ese mutex antes de poder llamar a `GetAngularPosition` y publicar `/joint_states`. Una llamada de salida lenta podía por tanto impedir que el temporizador actualizara `last_state_at_`, lo que producía un STOP del watchdog aunque la conexión USB siguiera sana.

La API Kinova continúa protegida por `api_mutex_`: no se introdujeron llamadas concurrentes a una API no garantizada como thread-safe. El watchdog sigue comparando la hora de una lectura física real con el límite de 200 ms; no se modificó su umbral ni se convirtió un STOP en ALLOW.

## Corrección mínima

`control_tick()` ahora toma una instantánea y calcula el comando bajo `state_mutex_`, libera ese mutex y solo entonces llama a `send_velocity()`. Tras el resultado vuelve a adquirir el mutex para actualizar contadores y telemetría. Así, `SendBasicTrajectory` ya no posee el bloqueo de estado que requiere la adquisición/publicación de `/joint_states`.

La excepción inevitable es una llamada SDK que bloquee el propio `api_mutex_` durante más de 200 ms: el watchdog debe seguir detectando ese caso porque no existe una lectura física nueva. La evidencia anterior no permitía atribuir los 354.190 ms a `SendBasicTrajectory` o a `GetAngularPosition`; no había instrumentación de duración por llamada. El reset de deadline del control y el hueco sólo probaban una sección crítica prolongada. La corrección elimina la contención de `state_mutex_`, que sí era determinista en el diseño.

## Validación sin hardware

Se compiló únicamente `thesis_hardware_bridge` y `test_bridge_logic` aprobó 21/21. La prueba nueva `SlowOutputDoesNotHoldFeedbackStateLock` modela una salida de 250 ms y confirma que el lock dedicado a actualización de estado permanece disponible en menos de 100 ms. No pretende emular ni ocultar una llamada Kinova API bloqueada.

## Validación física limitada

Se relanzó el adaptador desarmado, con API sana/READY/ALLOW, y se ejecutó un solo ciclo J1 relativo a la postura inicial medida:

1. `+3°` a 3°/s (`duration=1.34 s`);
2. retorno `-3°` a la postura inicial (`duration=1.34 s`);
3. HOLD aproximado de 6 s y desarmado explícito.

La telemetría del stack muestra dos comandos aceptados y completados, sólo `ALLOW/CLEAR`, escala 1.0, sin watchdog ni fault. La edad de estado observada por el supervisor durante movimiento estuvo entre 36.481 y 48.705 ms; el estado final de J1 fue exactamente `-1.551814113493139 rad`, igual al origen, con velocidad cero. El único STOP del log corresponde al desarmado ordenado.

## Veredicto

**REQUIERE REPETICIÓN DE EVIDENCIA, SIN ESCALAR.** El proceso de rosbag iniciado para esta prueba no sobrevivió al cierre de la sesión de terminal y sólo dejó una base sqlite3 sin `metadata.yaml`, con datos hasta antes del armado. Por ello no existe un rosbag que demuestre directamente el máximo hueco de `/joint_states` durante el ciclo, y no se puede declarar aprobada la validación física con el criterio solicitado. No se realizará otro movimiento sin autorización explícita para repetir el mismo ciclo de ±3° con grabación persistente.
