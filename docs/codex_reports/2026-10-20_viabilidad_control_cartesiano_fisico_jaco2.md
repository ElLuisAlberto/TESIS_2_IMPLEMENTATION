# Viabilidad del control cartesiano continuo en el JACO2 físico

Fecha: 2026-10-20  
Entorno objetivo: ROS 2 Humble, Kinova JACO2 `j2n6s300`, dominio físico 78  
Dictamen: **NO VIABLE AÚN**

## Alcance y seguridad

La revisión fue estática y de observación. No se armó el adaptador, no se
publicaron candidatos ni intenciones y no se envió movimiento. No se modificó
código funcional porque la arquitectura actual no satisface todavía los
contratos necesarios para habilitar esta función con seguridad.

La GUI física debe permanecer articular. Esto ya ocurre: en modo `hardware`
solo se añade el grupo de JOG articular; las pestañas cartesianas se construyen
exclusivamente en modo `simulation`.

## Ruta encontrada

| Etapa | Implementación actual | Resultado para hardware |
| --- | --- | --- |
| GUI continua | `joint_control_gui.py` publica `TwistStamped` en `/thesis/cartesian_jog_intent`, con `frame_id=world`, a partir de rueda X/Y/Z/Roll/Pitch/Yaw | Los controles no se muestran en modo físico |
| Semántica | X/Y/Z son m/s; las entradas angulares visibles en grados/s se convierten a rad/s; el temporizador corre a 20 Hz y la intención de rueda vence tras 0.25 s | El mecanismo de cese/hold existe en simulación |
| Conversión | `cartesian_jog_adapter.py` toma `/joint_states` y TF `world -> j2n6s300_end_effector`, integra un paso local y resuelve IK numérica por mínimos cuadrados amortiguados | El nodo se declara explícitamente “simulation only” |
| Salida de conversión | Publica `JointCommand` en `/thesis/jog_intent` con horizonte de 1 s | Usa el ID fijo `cartesian_jog_continuous` |
| Supervisión | `safety_supervisor_node.py` predice el horizonte articular/cápsulas y publica `/thesis/supervised_jog_command` con ALLOW/WARNING/REDUCTION/STOP e histéresis | Es reutilizable si recibe una intención articular válida |
| Adaptador físico | `jaco_hardware_node.cpp` consume `/thesis/supervised_jog_command` | Rechaza todo JOG cuyo ID no empiece por `jog:<arm_epoch>:` |
| Launch físico | `jaco_physical_system.launch.py` inicia estado, TF, obstáculo, proximidad, supervisor, horizonte, bridge, GUI y RViz | No inicia ningún adaptador cartesiano |

Ruta efectiva actual:

```text
GUI cartesiana (solo simulación)
  -> /thesis/cartesian_jog_intent
  -> cartesian_jog_adapter (paquete thesis_simulation)
  -> /thesis/jog_intent
  -> safety_supervisor
  -> /thesis/supervised_jog_command
  -X-> jaco_hardware_adapter (ID ajeno a la época de armado)
```

No hay bypass directo a Kinova. El problema es que tampoco existe una ruta
cartesiana física admisible de extremo a extremo.

## Evaluación técnica

### X, Y y Z

El código de simulación integra una velocidad expresada en `world`, limita el
paso de IK a 5 mm y respeta límites articulares y velocidades. Las pruebas
offline cubren los tres ejes desde una postura recomendada. Eso demuestra la
lógica local, pero no que el Jacobiano interno represente al brazo físico en
el mismo instante y frame que TF.

### Roll, Pitch y Yaw

En JOG, los componentes angulares no son tasas de Euler intrínsecas: se tratan
como un vector de rotación alrededor de ejes de `world` y se premultiplica el
incremento. La pose mostrada por la GUI sí se convierte a Roll/Pitch/Yaw. Esta
diferencia es matemáticamente válida si se documenta como tal, pero hoy las
etiquetas no distinguen “giro sobre eje world” de “tasa RPY”. Cerca de la
singularidad de Euler, la visualización RPY también puede cambiar de forma no
intuitiva aunque la orientación física sea continua.

### Límites, alcance y continuidad

La IK recorta contra los límites articulares configurados, limita el paso por
iteración, usa búsqueda de línea y devuelve HOLD si no converge. El supervisor
vuelve a saturar por velocidad, predice 1 s y aplica la geometría de cápsulas.
La GUI no acumula indefinidamente la rueda y dispone de intención cero.

Esto es una base útil, pero la IK no devuelve distancia a límites, causa de
inalcanzabilidad ni discontinuidad entre soluciones. Un recorte que todavía
reduce el coste puede terminar cerca de un límite sin una clasificación
específica para la GUI física.

### Singularidades

El solucionador emplea amortiguamiento fijo (`damping=0.04`), pero no calcula
ni expone valor singular mínimo, manipulabilidad o número de condición. Por
tanto no hay umbral explícito para rechazar o degradar suavemente un comando
cerca de una singularidad, requisito necesario antes de mandar velocidades
derivadas al brazo real. La convergencia o el fallo final de IK no sustituyen
ese interbloqueo.

### TF, modelo y estado real: bloqueo A-02b

El URDF publica la cadena desde `world` y el bridge entrega `/joint_states`,
pero la cinemática local usa constantes propias en `jaco_kinematics.py`. La
función `align_relative_target()` alinea incrementos mediante el TF vivo; no
comprueba que los ejes, offsets y orientación del modelo articulado coincidan
con ese TF para el mismo estado y sello temporal.

A-02b sigue bloqueada por contrato: falta una muestra articular sincronizada,
una relación explícita modelo→`world` y la comparación de posición **y
orientación** entre FK y TF. El parámetro `kinematic_tf_tolerance_m` se declara
y valida, pero no se usa en una comprobación de coherencia en tiempo de
ejecución. Sin esto, una IK puede calcular una dirección articular a partir de
un Jacobiano incoherente con el brazo observado, aunque TF exista y esté
fresco. Este bloqueo afecta las seis componentes cartesianas.

### Supervisor, cápsulas, recuperación y watchdog

Una vez obtenida una intención articular correcta, la infraestructura común sí
es apropiada: predicción de 1 s, cápsulas, proximidad, STOP fail-closed,
reducción y recuperación gradual mediante `ControlStabilityFilter`. El bridge
también exige frescura, época de armado y detiene por watchdog. No se debe
duplicar esa seguridad.

Sin embargo, reutilizar directamente el nodo de `thesis_simulation` no basta:
su ID fijo infringe la época de armado y no tiene estado de armado para borrar
intenciones previas. Además, el paquete y el launch lo definen como componente
de simulación. Hace falta una capa cartesiana compartida o de producción que
termine siempre en `/thesis/jog_intent`, nunca en la API Kinova.

## Observación en vivo, solo lectura

Se encontró una sesión física previa todavía ejecutándose en el dominio por
defecto (sus procesos no tenían `ROS_DOMAIN_ID`), mientras el contrato de esta
tarea fija el dominio 78. En esa sesión se observó:

- `/thesis/hardware/armed`: `false`;
- `jaco_hardware_adapter`, supervisor y cadena preventiva presentes;
- ningún `cartesian_jog_adapter`;
- no se recibió una muestra de `/joint_states` durante la consulta;
- `tf2_echo world j2n6s300_end_effector` no obtuvo una cadena conectada.

Esta observación no se usa como prueba de un fallo permanente del backend; sí
impide cualquier validación física cartesiana en la sesión actual. No se
reinició ni alteró la sesión.

## Trabajo mínimo antes de habilitarlo

1. Cerrar A-02b con un contrato temporal único: estado articular y TF del
   efector correspondientes al mismo sello, calibración documentada de
   feedback Kinova→URDF/modelo y comparación FK–TF de posición y orientación.
   Toda ausencia o discrepancia debe terminar en HOLD/STOP.
2. Convertir la lógica cartesiana reutilizable en un adaptador de producción
   compartido, separado del backend Kinova. Debe partir del estado real fresco,
   emitir únicamente `/thesis/jog_intent` y conservar la ruta por supervisor.
3. Integrar el protocolo de armado: inerte desarmado, limpieza al cambiar de
   época, rechazo de inputs con sello anterior al armado y generación de
   `command_id=jog:<arm_epoch>:<id fresco>` solo tras un input nuevo.
4. Añadir métrica de condicionamiento del Jacobiano, umbrales conservadores,
   reducción/rechazo cerca de singularidad, margen de límites y rechazo de
   soluciones discontinuas o inalcanzables.
5. Definir explícitamente la convención angular. La opción mínima coherente con
   el código es “velocidad angular sobre X/Y/Z de `world`”, mostrando RPY solo
   como pose informativa.
6. Añadir el adaptador al launch físico detrás de un argumento desactivado por
   defecto y mostrar la pestaña física solo cuando el contrato esté disponible
   y sano. `hardware_output_enabled` debe seguir siendo `false` por defecto.
7. Probar offline ambos signos de los seis ejes, saturación, límites,
   inalcanzabilidad, singularidad, salto de solución, TF/estado ausente u
   obsoleto, armado/época, watchdog/HOLD, STOP y recuperación REDUCTION→ALLOW.
8. Validar en simulación y mock hardware. Solo después, solicitar autorización
   separada para una prueba física incremental con salida inicialmente
   desarmada.

## Archivos modificados

No se modificó código ni configuración. Se añadieron únicamente este informe y
su evidencia asociada.

## Compilación y pruebas

No se compiló: no hubo paquetes funcionales afectados. Las regresiones offline
existentes terminaron correctamente:

- 87 pruebas Python: cinemática/trayectoria cartesiana, estabilidad,
  política preventiva, modelo JOG/trayectoria y GUI;
- 25 pruebas de `thesis_hardware_bridge`, incluidas frescura y pertenencia del
  JOG a la época de armado.

Estas pruebas demuestran la base de simulación y las defensas existentes; no
cubren los cuatro bloqueos descritos y no autorizan movimiento cartesiano
físico.

## Procedimiento de arranque seguro, sin salida física

Terminal 1 — interfaz física articular y RViz del robot:

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=78
ros2 launch thesis_hardware jaco_physical_system.launch.py \
  mock_hardware:=false \
  hardware_output_enabled:=false \
  use_demo_obstacle:=true \
  demo_obstacle_scenario:=PARK \
  demo_insert_stop_active:=false \
  start_gui:=true \
  start_rviz:=true \
  allow_gui_arm_control:=false
```

Terminal 2 — cámara RGB-D independiente:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=78
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  enable_rgbd:=true \
  enable_sync:=true \
  align_depth.enable:=true \
  pointcloud.enable:=true
```

Terminal 3 — RViz de la cámara:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=78
rviz2 \
  -d /opt/ros/humble/share/realsense2_camera/examples/pointcloud/rviz/pointcloud.rviz \
  --ros-args -r __node:=rviz2_rgbd
```

La cámara queda solo para visualización; no alimenta al supervisor. Antes del
primer comando debe cerrarse cualquier sesión física previa para que exista un
único dueño del USB y un único dominio ROS.

