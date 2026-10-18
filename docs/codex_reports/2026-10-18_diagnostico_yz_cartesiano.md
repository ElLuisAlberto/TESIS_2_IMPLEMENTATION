# Diagnóstico técnico de los casos cartesianos Y y Z

Fecha: 2026-10-18.

## Alcance y evidencia revisada

Este diagnóstico es de solo lectura sobre código, configuración, pruebas e
informes existentes. No se modificó código, no se inició ROS ni Gazebo, no se
publicaron comandos ROS y no se usaron hardware físico, cámara RGB-D o `sudo`.

Se revisaron los tres resultados solicitados y sus evidencias en
`evidence/codex_reports/`. El árbol ya contenía cambios sin confirmar en los
archivos cartesianos y de disponibilidad; se preservaron. En particular, la
tolerancia de 0,2 mm y las dependencias cartesianas añadidas a
`system_readiness` forman parte de esos cambios existentes, no de este
diagnóstico.

## Caso Y: rechazo de la IK

### Recorrido de la tolerancia

La tolerancia efectiva de **0,0002 m** se fija en
`src/thesis_simulation/launch/thesis_system.launch.py:161`, dentro de los
parámetros del nodo `cartesian_command_adapter`. Esa configuración sobrescribe
el valor por defecto de **0,003 m** declarado por el adaptador en
`src/thesis_simulation/thesis_simulation/cartesian_command_adapter.py:44`.

El adaptador la lee en las líneas 57–59 y la entrega a
`solve_inverse_kinematics_transform()` en las líneas 251–256. El solver declara
por defecto **0,002 m** en
`src/thesis_core/thesis_core/cartesian_kinematics.py:194` y acepta una solución
solo cuando se cumplen simultáneamente
`position_norm <= position_tolerance_m` y la tolerancia angular (líneas
232–250). Por tanto, el lanzamiento integrado usa un umbral diez veces más
estricto que el valor por defecto del solver y quince veces más estricto que el
valor por defecto del adaptador.

### Causa confirmada

1. La prueba Y solicitó un paso de +0,020 m manteniendo la orientación y el
   adaptador devolvió `INVERSE_KINEMATICS_FAILED`, con error lineal mostrado
   como 0,0003 m y error angular de 0,01 grados. No se generó un
   `JointCommand`; el supervisor y Gazebo nunca recibieron esa intención.
2. El rechazo lo determina exclusivamente la comparación anterior: el residuo
   mostrado es mayor que 0,0002 m. El texto se formatea con cuatro decimales en
   `cartesian_command_adapter.py:264-271`; por ello “0,0003 m” no identifica el
   valor interno exacto (por redondeo, es aproximadamente 0,25–0,35 mm).
3. La postura inicial de Gazebo se declara como aproximadamente
   `(0, π, π, 0, 0, 0)` en
   `src/thesis_simulation/urdf/jaco_gazebo.ros2_control.xacro:52-117`, y la
   evidencia de X registró valores articulares muy próximos. Al evaluar el
   Jacobiano numérico implementado por el proyecto en esa postura, la fila de
   traslación Y contiene términos del orden de `10^-7 m/rad`, mientras la de X
   contiene términos de hasta aproximadamente `0,91 m/rad`. Esto confirma que
   la postura de arranque está singular o muy mal condicionada para una
   traslación Y de primer orden.
4. La IK es local, por mínimos cuadrados amortiguados, con Jacobiano numérico,
   amortiguamiento 0,04, pasos articulares limitados a 0,15 rad, límites
   articulares, búsqueda lineal y hasta 120 iteraciones
   (`cartesian_kinematics.py:190-325`). Estas propiedades explican que una
   dirección mal condicionada pueda terminar con un residuo pequeño no nulo.

La causa inmediata confirmada es, por tanto, una tolerancia efectiva de 0,2 mm
menor que el residuo de una IK local resuelta desde una postura singular para
Y. No hay evidencia de un error de ejecución de Gazebo en este caso, porque el
comando fue detenido antes de llegar a Gazebo.

### Hipótesis y límites de la evidencia

- Es muy probable que el residuo sea el suelo numérico de esa combinación de
  semilla casi singular, amortiguamiento, conservación de orientación y
  búsqueda lineal. No puede confirmarse qué condición terminó exactamente la
  iteración: la evidencia Y no guardó las seis posiciones iniciales con máxima
  precisión, las iteraciones ni la solución articular fallida, y el objeto
  `IkResult` informa `maximum_iterations` también cuando el lazo sale antes por
  sistema singular o por falta de mejora.
- Un pequeño desfase entre `/joint_states` y la TF usada para alinear el
  objetivo puede contribuir al residuo. No está demostrado con la captura
  disponible.
- No se debe interpretar el residuo como precisión física del Jaco2. Esta
  prueba solo caracteriza el modelo y la cadena de simulación actuales.

### Comparación con pruebas y precisión operativa

| Contexto | Tolerancia lineal | Observación |
| --- | ---: | --- |
| Lanzamiento, adaptador puntual | 0,2 mm | Umbral que rechazó Y. |
| Solver IK, valor por defecto | 2 mm | Diez veces más permisivo. |
| Adaptador puntual, valor por defecto | 3 mm | Queda sobrescrito por el launch. |
| Prueba `test_three_millimetre_tolerance_accepts_bounded_local_residual` | 2 y 3 mm | Comprueba rechazo a 2 mm y aceptación a 3 mm para otro objetivo. |
| Prueba JOG por ejes | 0,2 mm | Pasa desde la postura recomendada `(-45°, 160°, 200°, 0°, 15°, 0°)`, con pasos locales de 4 mm; no cubre la postura singular de arranque ni un objetivo puntual de 20 mm. |
| Adaptador de trayectoria, waypoints | 3 mm | Usa una IK encadenada distinta. |
| Criterio de terminación/validador en simulación | 15 mm | Evalúa error final medido, no convergencia interna de la IK. |

La evidencia integrada de X solo reporta TF a tres decimales (resolución
mostrada de 1 mm), y el criterio final existente admite 15 mm. Exigir 0,2 mm en
la conversión IK es más estricto que lo observable en esos informes y mucho
más estricto que los demás contratos puntuales del proyecto. Eso no justifica
usar 15 mm para IK: conversión y ejecución deben conservar márgenes separados.
Para esta simulación, un umbral de submilímetro con validación final independiente
es razonable; no se deriva de aquí ninguna afirmación sobre exactitud del brazo
físico.

### Cambio mínimo propuesto

Cambiar **solo** el parámetro `position_tolerance_m` del
`cartesian_command_adapter` en `thesis_system.launch.py` de **0,0002 m a
0,0005 m**. No se propone cambiar el solver, la tolerancia angular, los límites
de paso, el JOG, las trayectorias ni el criterio final de 15 mm.

0,5 mm es el menor escalón decimal práctico que queda por encima de todo el
intervalo que puede haberse mostrado como 0,0003 m, conserva precisión
submilimétrica y limita la relajación a 2,5 % del paso solicitado de 20 mm.
Adoptar directamente 2–3 mm sería coherente con valores existentes, pero no
sería el ajuste mínimo.

### Regresión requerida y criterio de autorización

Antes de autorizar el cambio:

1. Añadir una prueba pura que reproduzca el objetivo Y de +20 mm desde la
   postura inicial real, conservando orientación, y guardar residuo exacto,
   iteraciones, semilla y solución. Debe fallar a 0,2 mm y aprobar a 0,5 mm.
2. Parametrizar X, Y y Z tanto desde la postura inicial como desde la postura
   recomendada. Cada resultado aceptado debe respetar 0,5 mm, la tolerancia
   angular y todos los límites articulares.
3. Mantener los casos de objetivo lejano/inalcanzable y añadir un caso apenas
   fuera de 0,5 mm para demostrar que no se convierte cualquier no
   convergencia en éxito.
4. Ejecutar las pruebas unitarias de `thesis_core` y `thesis_simulation`, y una
   pila headless pasiva con `simulation_output_enabled:=false` para comprobar
   parámetros, TF y estados sin movimiento.
5. Solo con autorización separada, repetir Y en Gazebo y exigir: conversión IK,
   `ALLOW`, ejecución `SUCCEEDED`, error TF final dentro del criterio vigente y
   ausencia de desplazamiento material en X/Z u orientación.

Se puede autorizar la implementación si el residuo exacto es estable y no
supera 0,5 mm, las regresiones de inalcanzabilidad siguen rechazando y la
prueba pasiva no muestra incoherencias TF/estado. Si varias pilas nuevas superan
0,5 mm o el residuo varía fuertemente, no se debe seguir elevando la tolerancia:
corresponde cambiar la postura de inicio a una no singular o revisar el solver
y la sincronización TF/estado.

## Caso Z: disponibilidad del sistema

### Cómo calcula y publica `READY`

`SystemReadinessNode` usa tiempo monotónico de pared (`use_sim_time: false` en
el launch) y ejecuta `evaluate()` mediante un temporizador. Los valores
integrados son:

- `readiness_rate_hz = 1,0 Hz`: una evaluación/publicación por segundo.
- `readiness_timeout_sec = 0,75 s`: edad máxima de cada muestra requerida.
- Publicaciones con QoS por defecto, profundidad 10 y durabilidad volátil:
  `/thesis/system_ready` (`Bool`) y `/thesis/system_readiness` (`String`). Un
  suscriptor nuevo no recibe retrospectivamente el último estado.

En cada evaluación, el nodo exige simultáneamente:

1. **Grafo sin ausencias ni duplicados:** puentes/reloj, estado del robot,
   monitor y supervisor preventivo, salida de simulación, tres adaptadores
   cartesianos, `horizon_preview` y el propio monitor. En modo `topic` también
   exige `obstacle_input_bridge` y, si está habilitado, `obstacle_demo_source`;
   la GUI solo se exige cuando `start_gui:=true`.
2. **Datos recibidos en los últimos 0,75 s:** `/clock`, `/joint_states`,
   `/thesis/proximity_status` y la fuente de obstáculo. En el lanzamiento
   analizado, el modo por defecto es `topic`, por lo que la última se actualiza
   desde `/thesis/obstacle_in_model_frame`.
3. **Conexiones:** al menos un suscriptor para las entradas del supervisor y
   adaptador (`candidate_command`, `jog_intent`, `proximity_status`, las dos
   salidas supervisadas y las tres entradas cartesianas).
4. **Controlador:** respuesta de `/controller_manager/list_controllers` que
   incluya `arm_controller` en estado `active` y servidor de acción
   `/arm_controller/follow_joint_trajectory` disponible.

Si no falta nada publica `Bool(true)` y el texto exacto `READY`; en cualquier
otro caso publica `Bool(false)` y `WAITING: ...` con todas las causas halladas.

### Condiciones de inicio y causa confirmada

El launch inicia `system_readiness` junto con la mayor parte de la pila, pero
retrasa explícitamente el `joint_state_broadcaster` hasta 5 s y el
`arm_controller` hasta 6 s
(`jaco_gazebo.launch.py:386-415`). La consulta asíncrona a
`list_controllers` solo se inicia en una evaluación y su respuesta se procesa
en una evaluación posterior. En condiciones ideales, el primer `READY` no es
esperable antes de aproximadamente 7–8 s desde el arranque; el descubrimiento
DDS, el arranque de Gazebo y la llegada conjunta de las cuatro muestras pueden
alargarlo.

En Z está confirmado únicamente que dos clientes nuevos, cada uno limitado a
su propia ventana de descubrimiento, no descubrieron tipo/mensaje y terminaron
con código 1, aunque una consulta separada sí vio el publicador. No está
confirmado que `system_readiness` dejara de evaluar o publicar. La prueba Y y el
intento Y/Z anterior sí recibieron `READY` en pilas nuevas; en este último fue
necesario repetir la consulta y usar una ventana mayor. La explicación más
ajustada a la evidencia es una carrera de arranque/descubrimiento del cliente,
no un fallo funcional demostrado del monitor.

### Hipótesis adicionales

- Como la publicación es volátil y ocurre a 1 Hz, un `echo --once` iniciado
  justo después de una publicación debe esperar al siguiente ciclo; parte de
  su tiempo también se consume en descubrir el tipo y emparejar endpoints.
- El timeout de frescura (0,75 s) es menor que el periodo de evaluación (1 s).
  Las fuentes nominales son más rápidas, por lo que esto no impide `READY` por
  sí solo, pero aumenta la sensibilidad a pausas de arranque o carga.
- No se conservó el log de estados `WAITING` del nodo para Z. Por ello no puede
  descartarse una dependencia realmente ausente o una respuesta tardía del
  controlador en esa ejecución concreta.

### Comprobación robusta propuesta

La prueba debe usar un plazo global de arranque, no interpretar un único
`echo --once` de 2 s como plazo para toda la pila:

1. Iniciar solo la pila headless pasiva con
   `simulation_output_enabled:=false`, `start_gui:=false`,
   `start_rviz:=false` y `headless:=true`.
2. Esperar las condiciones estructurales conocidas: proceso de launch vivo,
   vencimiento de los temporizadores de 5–6 s y presencia del publicador. Toda
   consulta ROS debe mantenerse como cliente independiente con
   `--no-daemon --spin-time 2` y debe terminar antes de iniciar la siguiente.
3. Durante un plazo global sugerido de 20 s desde el arranque, efectuar
   intentos acotados de lectura indicando explícitamente el tipo
   `std_msgs/msg/String`, para no depender de inferirlo. Conservar cada mensaje
   recibido. Un `WAITING` no aprueba: sus causas se usan para comprobar el nodo,
   tópico, conexión, muestra o controlador concreto y se vuelve a evaluar
   dentro del mismo plazo.
4. Aprobar solo al recibir el texto exacto `READY`; opcionalmente confirmar en
   una lectura independiente que `/thesis/system_ready` es `true`. Fallar si al
   vencer el plazo no hay publicador, no llega ningún mensaje, el último estado
   sigue en `WAITING`, el launch terminó o una dependencia permanece ausente.
5. En el fallo, adjuntar el último `WAITING`, tiempo desde inicio y comprobaciones
   separadas del publicador, `/joint_states`, proximidad, servicio/controlador y
   acción. Esto distingue retraso de descubrimiento de indisponibilidad real y
   evita convertir silencio en éxito.

No se recomienda ocultar el problema con una espera fija seguida de aprobación
implícita, aceptar cualquier mensaje de `system_readiness`, ni omitir el
monitor. El plazo debe absorber el arranque esperado, pero `READY` exacto sigue
siendo obligatorio.

### Cambio mínimo propuesto y criterio de autorización

Para Z, el cambio mínimo es inicialmente **de procedimiento de prueba**, no de
código: tipo explícito, plazo global coherente con el arranque y diagnóstico de
cada `WAITING`. No hay evidencia suficiente para aumentar
`readiness_timeout_sec`, reducir dependencias o hacer latched el estado.

La implementación de un cambio en `system_readiness` solo debe autorizarse si
una prueba pasiva repetida conserva mensajes `WAITING` que demuestren una causa
interna concreta, o si un observador de larga vida y tipo explícito sigue sin
recibir publicaciones mientras el nodo está activo. En ese caso serían
necesarias pruebas unitarias de `evaluate()` para estado inicial, conjunto
completo, cada dependencia ausente, controlador asíncrono y transición
`WAITING -> READY -> WAITING`, además de una prueba de lanzamiento pasiva en
varias pilas nuevas. No debe relajarse ninguna condición de seguridad para
hacer pasar la prueba.

## Verificación ejecutada durante este diagnóstico

Se ejecutaron únicamente pruebas Python puras, sin ROS:

```text
python3 -m pytest -q \
  src/thesis_core/test/test_cartesian_kinematics.py \
  src/thesis_simulation/test/test_cartesian_jog_model.py \
  src/thesis_simulation/test/test_readiness_contract.py

20 passed in 0.27s
```

Estas pruebas confirman los contratos unitarios existentes, pero no reproducen
el objetivo Y desde la postura inicial ni ejercitan `SystemReadinessNode` como
nodo; ambos huecos están incluidos en las regresiones propuestas.
