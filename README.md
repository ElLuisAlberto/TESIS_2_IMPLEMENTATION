# TESIS_2_IMPLEMENTATION

Plataforma de tesis para evaluar control preventivo de un manipulador JACO2 mediante predicción geométrica de horizonte corto. Estado documental: **17/09/2026, Fases 1–4 de control manual y horizonte**.

## Estado y alcance

El entorno de trabajo utilizado es **Ubuntu 22.04, ROS 2 Humble y Gazebo Fortress**, con `ros_gz`, `gz_ros2_control`, RViz2 y GUI PyQt5. La simulación utiliza seis articulaciones del brazo; el control del gripper y la integración física tienen un alcance separado.

Implementado en la versión local:

- Control por objetivo articular y duración, con evaluación del comando candidato.
- Control manual continuo mediante la rueda del mouse sobre la barra de cada articulación, con Vmáx individual y barras que muestran el estado medido.
- Flujo manual supervisado antes de entregar trayectorias cortas al controlador simulado.
- Predicción desde el estado actual, búsqueda de escala de velocidad y reevaluación geométrica del horizonte.
- Volumen en RViz, trazas amarillas y configuraciones futuras para hacer visible el recorrido previsto.
- Control de antigüedad de datos y vigilancia de interrupción del flujo manual.

La aplicación de los parches y la compilación de los tres paquetes modificados terminaron correctamente en el equipo de desarrollo. Se dispone de evidencia visual de operación y de publicaciones de intención próximas a 20 Hz. **Esto no acredita todavía cumplimiento de Vmáx medida, frenado, latencia máxima ni ausencia de colisiones entre muestras.**

El criterio de investigación es reducir el movimiento por ocupación futura incompatible con el entorno. La proximidad actual por sí sola no debería ralentizar un movimiento admisible. El cálculo incluye el estado inicial: si ya está dentro del margen, todavía debe verificarse el comportamiento de retirada. No se presenta ese caso como resuelto.

## Compilación

Antes de recompilar: desactivar el control manual y cerrar los procesos de las terminales 4, 3, 2 y 1 con `Ctrl+C`, en ese orden. Usar una terminal nueva para evitar cargar el workspace como su propio underlay.

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  thesis_interfaces thesis_description thesis_core thesis_simulation thesis_ui
```

Los paquetes y dependencias de simulación deben estar instalados previamente. Los paquetes del SDK físico no son necesarios para esta compilación selectiva. No mezclar entornos Humble y Jazzy.

## Arranque: cuatro terminales y una de diagnóstico

Mantener abiertas las terminales 1–4. Abrir cada proceso una sola vez. Cada bloque prepara su propio entorno.

### Terminal 1 — Gazebo, RViz, estado y proximidad

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch thesis_simulation jaco_gazebo.launch.py
```

Esperar `Configured and activated arm_controller`. Este launch también inicia el puente de reloj, la representación por cápsulas y el monitor de proximidad. La esfera se coloca por defecto en `(0.60, 0.0, 0.65)` m; los argumentos `obstacle_x`, `obstacle_y`, `obstacle_z` permiten configurar su posición inicial. El launch configura las rutas de recursos y plugins de Gazebo.

### Terminal 2 — Supervisor y adaptador

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch thesis_simulation safety_pipeline.launch.py \
  simulation_output_enabled:=true \
  require_proximity_status:=true \
  use_sim_time:=true
```

### Terminal 3 — Horizonte corto

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run thesis_core horizon_preview --ros-args \
  -p use_sim_time:=true \
  -p source:=auto \
  -p horizon_sec:=1.0 \
  -p samples:=21 \
  -p margin_m:=0.02 \
  -p rate_hz:=10.0
```

En RViz usar `Fixed Frame: world`. Si falta el volumen, añadir una visualización `MarkerArray` con tópico `/thesis/horizon_volume`. Las cápsulas actuales usan `/thesis/robot_capsules`. Guardar la configuración de RViz es opcional; verificar por separado cualquier cambio en ese archivo antes de versionarlo.

### Terminal 4 — Interfaz articular

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run thesis_ui joint_gui
```

### Terminal 5 — Diagnóstico, sin cerrar las anteriores

Con el modo manual activado:

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic info /thesis/jog_intent
ros2 topic info /thesis/supervised_jog_command
timeout 5s ros2 topic hz /thesis/jog_intent
timeout 5s ros2 topic hz /thesis/supervised_jog_command
timeout 5s ros2 topic echo /thesis/execution_control --once
```

Cada medición termina automáticamente. Un aviso inicial de `topic hz` puede aparecer mientras descubre el publicador; comprobar si después llegan muestras. La existencia de nodos o indicadores verdes no demuestra por sí sola que se estén publicando comandos. La frecuencia observada puede diferir de la solicitada y las mediciones anteriores son secuenciales.

## Uso de los dos modos

### Control manual con rueda

1. Esperar a recibir `/joint_states` y comprobar las conexiones.
2. Configurar Vmáx por articulación y pulsar **ACTIVAR CONTROL MANUAL CONTINUO**.
3. Colocar el cursor sobre la barra del joint y girar la rueda del mouse. La barra representa su ángulo medido; no acumula un objetivo lejano al girar más deprisa.
4. Comparar movimiento lento y rápido de la rueda sobre la misma articulación y desde posturas semejantes.
5. Dejar de girar y comprobar intención cero, detención y contracción del horizonte. Desactivar el modo antes de abandonar la prueba.

En este modo, el arrastre y los saltos por clic de la barra están deshabilitados. La intención se estima con una ventana de 0.25 s y sensibilidad de 1° por paso estándar de rueda. Sin nuevos pulsos, desaparece al vaciarse esa ventana; no significa frenado físico instantáneo. La GUI solicita actualizaciones a 20 Hz.

Los valores iniciales son 18°/s en J1–J3 y 24°/s en J4–J6; los máximos configurables son 36°/s y 48°/s, respectivamente. Son límites de intención implementados, no una garantía ya medida de la velocidad real del controlador interpolado. La supervisión puede reducirlos más.

### Objetivo y duración

Con el modo manual desactivado, ajustar los ángulos objetivo y la duración, y enviar el comando candidato. La previsualización nominal permite inspeccionar una intención; no equivale a autorización de movimiento. No activar ambos modos para la misma prueba. Los paneles de evaluación de candidatos y su historial no deben interpretarse como registro completo del control manual continuo.

## Horizonte: qué representa y cómo observarlo

El horizonte temporal permanece en **1 s**. Lo que debe cambiar con la velocidad es el recorrido proyectado desde la postura actual y, por tanto, el espacio ocupado por sus muestras. Si la intención queda limitada a Vmáx, aumentar más el giro de la rueda no debe seguir aumentando la velocidad solicitada.

El volumen incluye el grosor del robot y el margen geométrico de 0.02 m: incluso detenido conserva una envolvente. Las trazas amarillas y las posturas futuras ayudan a distinguir desplazamiento de grosor. El recorrido de un extremo de segmento no es una medición del volumen ni de la distancia al obstáculo. El efecto visual depende también de la articulación y de la postura; girar la base con el brazo casi vertical puede producir poco cambio visible.

Se utilizan 21 muestras y una actualización solicitada de 10 Hz. La visualización elimina puntos repetidos y representa la geometría a escala real. El color o la presencia de marcadores no sustituyen el registro de la decisión, la escala aplicada y la velocidad medida.

## Arquitectura y datos

| Componente | Responsabilidad |
| --- | --- |
| `thesis_interfaces` | Mensajes compartidos |
| `thesis_description` | Modelo, mallas y configuración RViz |
| `thesis_core` | Proximidad, supervisor, predicción y horizonte |
| `thesis_simulation` | Gazebo, cápsulas, controladores y adaptador |
| `thesis_ui` | Entrada manual, objetivos y presentación de estado |
| `thesis_hardware` / `thesis_hardware_bridge` | Herramientas físicas y puente de lectura, separados de esta guía de simulación |

| Tópico o interfaz | Uso |
| --- | --- |
| `/joint_states` | Posiciones y velocidades medidas; asociar por nombre, no por índice supuesto |
| `/clock` | Tiempo de simulación |
| `/thesis/jog_intent` | Intención manual de la GUI |
| `/thesis/supervised_jog_command` | Intención manual autorizada o escalada |
| `/arm_controller/joint_trajectory` | Trayectoria corta enviada por el adaptador |
| `/thesis/candidate_command` → `/thesis/supervised_command` | Flujo de objetivos con duración |
| `/arm_controller/follow_joint_trajectory` | Acción utilizada para ejecutar objetivos |
| `/thesis/proximity_status` | Distancia geométrica actual |
| `/thesis/trajectory_prediction` | Evaluación del candidato |
| `/thesis/execution_control` | Decisión durante ejecución, escala y mínimo proyectado |
| `/thesis/execution_trajectory` | Referencia/estado de ejecución |
| `/thesis/horizon_volume` | Marcadores de horizonte en RViz |

El adaptador manual usa trayectorias cortas, un periodo configurado de 0.1 s y vigilancia de recepción de 0.25 s. El supervisor conserva la decisión antes de la salida simulada. No publicar directamente al controlador para probar esta cadena.

Las distancias son entre superficies del modelo geométrico. Las medidas de proximidad y predicción llegan por separado. En el mensaje de control, `time_to_collision = -1` significa que no se encontró solapamiento en las muestras evaluadas; no demuestra seguridad fuera del horizonte ni entre muestras.

## Validación pendiente y siguiente evidencia

Registrar simultáneamente intención, salida supervisada, estado articular y decisión preventiva en estas pruebas:

- Rueda lenta, rápida y saturada: comparar velocidad solicitada, autorizada y medida.
- Cese de pulsos: medir latencia de intención cero y tiempo de parada real.
- Aproximación al obstáculo: comprobar reducción gradual y recalcular el horizonte reducido.
- Movimiento cercano que se aleja del obstáculo: comprobar si se permite sin reducción innecesaria.
- Cambio de sentido, pérdida de datos y desactivación del modo manual.

Se requiere una medición temporal para afirmar cumplimiento de los límites. La discretización, el seguimiento del controlador y los retardos siguen siendo limitaciones del prototipo. La evidencia de simulación no valida el control preventivo del brazo físico.

Siguientes etapas de investigación: caracterización de latencias y desaceleración, evaluación de casos límite, percepción RGB-D e incertidumbre, integración física controlada y ensayos reproducibles. El gripper/FSR conserva su línea de trabajo independiente.

## Cierre y reinicio

Desactivar el control manual y comprobar que el brazo deja de moverse. Cerrar con `Ctrl+C`: terminal 4 (GUI), 3 (horizonte), 2 (pipeline) y 1 (Gazebo/RViz). Para actualizar solo la GUI, desactivar el modo y cerrar su proceso antes de recompilar; reiniciar todos los nodos afectados cuando cambie el supervisor o el adaptador. No dejar versiones antiguas ejecutándose tras aplicar parches.

## Hardware, atribución e historial

El proyecto conserva el puente de lectura del JACO físico y sus herramientas de diagnóstico. La lectura física y el ensayo de movimiento controlado documentados históricamente no equivalen a tener validado el supervisor sobre hardware. Los archivos locales de firmware y configuración no se publican.

La atribución del modelo Kinova se mantiene en [UPSTREAM.md](src/thesis_description/UPSTREAM.md); conservar las licencias incluidas con el modelo.

El [README histórico completo](docs/historico/README_ANTES_FASE4.md) conserva el registro anterior, las pruebas, integración USB, configuración de hardware y la evolución de la tesis. Sus comandos y estados son históricos; los enlaces relativos allí escritos se interpretaban desde la raíz del repositorio. Los documentos `LEEME_*.md` permanecen disponibles como evidencia de etapas anteriores. Para arrancar la versión actual usar esta guía.
