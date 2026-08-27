# TESIS_2_IMPLEMENTATION

## Plataforma desacoplada de validación preventiva de seguridad para manipuladores robóticos colaborativos

Repositorio de implementación experimental de la tesis orientada al desarrollo de una **capa preventiva desacoplada en ROS 2** para validar comandos de movimiento antes de que lleguen al controlador del manipulador.

La plataforma busca interceptar un comando candidato, estimar su efecto a corto horizonte y decidir si el movimiento puede:

- **ALLOW**: ejecutarse sin modificación.
- **WARNING**: ejecutarse, pero reportando proximidad o condición de atención.
- **REDUCTION**: ejecutarse de forma limitada o reducida.
- **CRITICAL / STOP**: bloquearse o rechazarse antes de su ejecución física.

La arquitectura está diseñada para que la misma lógica de seguridad pueda utilizarse tanto en **simulación** como posteriormente con el **manipulador real**, evitando duplicar los algoritmos principales.

---

# 1. Objetivo de implementación

La implementación busca construir progresivamente el siguiente flujo:

```text
Fuente de comando
      │
      ▼
/thesis/candidate_command
      │
      ▼
Supervisor preventivo
      │
      ├── Estado cinemático del robot
      ├── Estado del entorno
      ├── Predicción a corto horizonte
      ├── Distancia mínima proyectada
      ├── Velocidad relativa
      ├── Time-to-Collision (TTC)
      └── Márgenes de incertidumbre
      │
      ▼
Clasificación de riesgo
      │
      ├── SAFE
      ├── WARNING
      ├── REDUCTION
      └── CRITICAL
      │
      ▼
/thesis/supervised_command
      │
      ▼
Controlador del manipulador
```

El sistema preventivo **no reemplaza el controlador interno del manipulador** ni pretende implementar un nuevo controlador de bajo nivel. Su función es validar o modificar comandos antes de permitir que lleguen al robot.

---

# 2. Estado actual del proyecto

## 2.1 Resumen general

| Componente | Estado | Observación |
|---|---:|---|
| Workspace ROS 2 | ✅ | ROS 2 Humble operativo |
| Estructura modular | ✅ | Paquetes separados por responsabilidad |
| Modelo Kinova JACO | ✅ | `j2n6s300` |
| URDF / Xacro | ✅ | Validado con `check_urdf` |
| Meshes visuales | ✅ | Cargados en RViz y Gazebo |
| Geometría de colisión | ✅ | Presente en el modelo |
| Propiedades inerciales | ✅ | Presentes en los links físicos |
| RViz | ✅ | Robot completo visible |
| Configuración RViz | ✅ | `jaco.rviz` guardado |
| Mock ros2_control | ✅ | `GenericSystem` operativo |
| `joint_trajectory_controller` | ✅ | Control de J1–J6 |
| Gazebo Fortress | ✅ | Versión 6.18.0 |
| `gz_ros2_control` | ✅ | `GazeboSimSystem` activo |
| Feedback de posición | ✅ | Publicado en `/joint_states` |
| Feedback de velocidad | ✅ | Publicado desde Gazebo |
| Primer movimiento en mock | ✅ | J1: `0 → 0.20 → 0 rad` |
| Primer movimiento en Gazebo | ✅ | J1: `0 → 0.20 → 0 rad` |
| Gripper controlado | ⏳ | Actualmente solo representado / publicado como joints extra |
| Nodo de comandos candidatos | ⏳ | Pendiente |
| Supervisor preventivo | ⏳ | Pendiente |
| Obstáculos simulados | ⏳ | Pendiente |
| Distancia mínima | ⏳ | Pendiente |
| TTC | ⏳ | Pendiente |
| Percepción RGB-D | ⏳ | Pendiente |
| Adaptador JACO real | ⏳ | Pendiente |
| Soft gripper + FSR | ⏳ | Pendiente |
| UI / logging experimental | ⏳ | Pendiente |
| Validación experimental completa | ⏳ | Pendiente |

---

# 3. Plataforma utilizada

## 3.1 Software

| Elemento | Configuración actual |
|---|---|
| Sistema operativo | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| Visualización | RViz 2 |
| Descripción | URDF + Xacro |
| Control | ros2_control |
| Controlador | `joint_trajectory_controller` |
| Simulación | Gazebo Fortress 6.18 |
| Integración Gazebo / ROS 2 | `gz_ros2_control` |
| Lenguaje principal | Python |
| Build system | colcon |
| Workspace | `~/ROS2/TESIS_IMPLEMENTATION` |

En la máquina también se encuentra instalado Gazebo 8.x mediante `gz sim`, pero para este proyecto en ROS 2 Humble se está utilizando **Gazebo Fortress 6.18 mediante `ign gazebo`**.

---

## 3.2 Manipulador

El modelo actual corresponde a la familia **Kinova JACO / JACO2**.

Configuración utilizada en el digital twin:

```text
Modelo: j2n6s300
Articulaciones del brazo: 6
Gripper: 3 dedos
```

Joints principales:

```text
j2n6s300_joint_1
j2n6s300_joint_2
j2n6s300_joint_3
j2n6s300_joint_4
j2n6s300_joint_5
j2n6s300_joint_6
```

Joints del gripper:

```text
j2n6s300_joint_finger_1
j2n6s300_joint_finger_tip_1

j2n6s300_joint_finger_2
j2n6s300_joint_finger_tip_2

j2n6s300_joint_finger_3
j2n6s300_joint_finger_tip_3
```

El manipulador físico fue identificado por USB como un dispositivo Kinova Robotics Jaco Robotic Arm. Sin embargo, **la etapa actual de desarrollo utiliza únicamente el digital twin**. No se está enviando movimiento al robot físico durante las pruebas de simulación.

---

# 4. Arquitectura del repositorio

```text
TESIS_IMPLEMENTATION/
├── docs/
├── src/
│   ├── thesis_core/
│   ├── thesis_description/
│   ├── thesis_hardware/
│   ├── thesis_interfaces/
│   ├── thesis_simulation/
│   └── thesis_ui/
├── README.md
└── .gitignore
```

## 4.1 Responsabilidad de cada paquete

| Paquete | Responsabilidad |
|---|---|
| `thesis_core` | Algoritmos preventivos independientes de simulación/hardware |
| `thesis_description` | URDF, Xacro, meshes, frames y configuración visual |
| `thesis_simulation` | Mock, Gazebo, mundos y adaptadores de simulación |
| `thesis_hardware` | Futuro adaptador al JACO real, RGB-D, gripper y FSR |
| `thesis_interfaces` | Mensajes y servicios comunes del proyecto |
| `thesis_ui` | Visualización de estados, métricas, alertas y logging |

---

# 5. `thesis_description`

Este paquete contiene la descripción común del manipulador.

Elementos principales:

```text
src/thesis_description/
├── launch/
│   └── view_jaco.launch.py
├── meshes/
├── rviz/
│   └── jaco.rviz
├── urdf/
│   ├── j2n6s300.xacro
│   ├── j2n6s300_standalone.xacro
│   ├── kinova_common.xacro
│   ├── kinova_finger_set.xacro
│   ├── kinova_inertial.xacro
│   └── kinova.gazebo
├── KINOVA_LICENSE
├── UPSTREAM.md
├── CMakeLists.txt
└── package.xml
```

La descripción se obtuvo a partir del modelo oficial de Kinova y se adaptó para utilizarse dentro del workspace de la tesis.

Los recursos originales de Gazebo Classic / ROS 1 que interferían con Fortress fueron deshabilitados en la ruta activa de simulación. El archivo histórico `kinova.gazebo` puede conservar referencias antiguas, pero ya no se invoca para la simulación ROS 2 actual.

---

# 6. Validación del URDF

## 6.1 Generación del modelo

```bash
cd ~/ROS2/TESIS_IMPLEMENTATION

source /opt/ros/humble/setup.bash
source install/setup.bash

xacro \
src/thesis_description/urdf/j2n6s300_standalone.xacro \
> /tmp/jaco_twin.urdf
```

## 6.2 Verificación

```bash
check_urdf /tmp/jaco_twin.urdf
```

Resultado esperado:

```text
robot name is: j2n6s300
---------- Successfully Parsed XML ---------------
```

## 6.3 Geometría verificada

En la versión utilizada actualmente se comprobó la presencia de:

```text
Visual:     19
Collision:  13
Inertial:   13
```

Esto permite utilizar la misma descripción para:

- visualización;
- cinemática;
- física;
- colisiones;
- futura simplificación geométrica para seguridad.

---

# 7. Visualización en RViz

Launch actual:

```bash
cd ~/ROS2/TESIS_IMPLEMENTATION

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch thesis_description view_jaco.launch.py
```

Configuración:

```text
Fixed Frame: world
RobotModel: enabled
TF: enabled
Description Topic: /robot_description
```

El archivo de configuración se encuentra en:

```text
src/thesis_description/rviz/jaco.rviz
```

Estado validado:

```text
Global Status: OK
RobotModel: OK
```

---

# 8. Primera etapa de control: mock ros2_control

Antes de introducir física, se validó la cadena de control utilizando:

```text
mock_components/GenericSystem
```

Arquitectura:

```text
JointTrajectory
      │
      ▼
joint_trajectory_controller
      │
      ▼
controller_manager
      │
      ▼
JacoMockSystem
      │
      ▼
/joint_states
      │
      ▼
robot_state_publisher
      │
      ▼
RViz
```

Launch:

```bash
ros2 launch thesis_simulation jaco_mock_control.launch.py
```

## 8.1 Controladores validados

```bash
ros2 control list_controllers
```

Resultado:

```text
joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster        active
arm_controller           joint_trajectory_controller/JointTrajectoryController active
```

## 8.2 Interfaces de hardware

```bash
ros2 control list_hardware_components
```

Estado verificado:

```text
JacoMockSystem
state: active
```

J1–J6:

```text
position [available] [claimed]
```

---

# 9. Código de prueba del mock

## 9.1 Posición inicial

Configuración inicial utilizada:

| Joint | Posición inicial |
|---|---:|
| J1 | 0 rad |
| J2 | π rad |
| J3 | π rad |
| J4 | 0 rad |
| J5 | 0 rad |
| J6 | 0 rad |

## 9.2 Movimiento de prueba

Movimiento:

```text
J1: 0.00 rad → 0.20 rad
Tiempo: 3 s
```

Comando:

```bash
ros2 topic pub --once \
/arm_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory \
"{
  joint_names: [
    'j2n6s300_joint_1',
    'j2n6s300_joint_2',
    'j2n6s300_joint_3',
    'j2n6s300_joint_4',
    'j2n6s300_joint_5',
    'j2n6s300_joint_6'
  ],
  points: [
    {
      positions: [
        0.20,
        3.141592653589793,
        3.141592653589793,
        0.0,
        0.0,
        0.0
      ],
      time_from_start: {
        sec: 3,
        nanosec: 0
      }
    }
  ]
}"
```

Resultado validado:

```text
j2n6s300_joint_1 = 0.2 rad
```

## 9.3 Retorno

```bash
ros2 topic pub --once \
/arm_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory \
"{
  joint_names: [
    'j2n6s300_joint_1',
    'j2n6s300_joint_2',
    'j2n6s300_joint_3',
    'j2n6s300_joint_4',
    'j2n6s300_joint_5',
    'j2n6s300_joint_6'
  ],
  points: [
    {
      positions: [
        0.0,
        3.141592653589793,
        3.141592653589793,
        0.0,
        0.0,
        0.0
      ],
      time_from_start: {
        sec: 3,
        nanosec: 0
      }
    }
  ]
}"
```

Resultado:

```text
J1 ≈ 0 rad
```

---

# 10. Digital twin dinámico en Gazebo

El siguiente nivel reemplazó el hardware mock por un sistema físico simulado.

Arquitectura actual:

```text
JointTrajectory
      │
      ▼
arm_controller
      │
      ▼
controller_manager
      │
      ▼
gz_ros2_control
      │
      ▼
GazeboSimSystem
      │
      ▼
Gazebo Fortress Physics
      │
      ├── posición
      ├── velocidad
      ├── gravedad
      ├── inercia
      └── colisiones
      │
      ▼
/joint_states
      │
      ▼
robot_state_publisher
      │
      ▼
RViz
```

---

# 11. Gazebo Fortress

Versión utilizada:

```bash
ign gazebo --versions
```

Resultado:

```text
6.18.0
```

La instalación también contiene Gazebo 8.x:

```bash
gz sim --versions
```

pero esa versión **no es la utilizada actualmente para este workspace ROS 2 Humble**.

---

# 12. `gz_ros2_control`

Versión verificada:

```text
gz_ros2_control 0.7.20
```

Plugin utilizado:

```xml
<plugin>
  gz_ros2_control/GazeboSimSystem
</plugin>
```

System plugin:

```text
gz_ros2_control-system
```

Biblioteca instalada:

```text
/opt/ros/humble/lib/libgz_ros2_control-system.so
```

El plugin instalado en ROS 2 Humble está enlazado con:

```text
libignition-gazebo6
```

por lo que la configuración activa corresponde a Gazebo Fortress.

---

# 13. Archivos de Gazebo

Estructura actual de simulación:

```text
src/thesis_simulation/
├── config/
│   ├── jaco_controllers.yaml
│   └── jaco_gazebo_controllers.yaml
├── launch/
│   ├── jaco_mock_control.launch.py
│   └── jaco_gazebo.launch.py
├── urdf/
│   ├── jaco_mock.ros2_control.xacro
│   └── jaco_gazebo.ros2_control.xacro
└── worlds/
    └── jaco_empty.sdf
```

---

# 14. Arranque actual de Gazebo

## 14.1 Variables requeridas en el estado actual

Por el momento se utilizan:

```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="/opt/ros/humble/lib${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"
```

y:

```bash
export IGN_GAZEBO_RESOURCE_PATH="$(ros2 pkg prefix thesis_description)/share${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
```

Estas variables permiten a Fortress encontrar:

- el plugin `gz_ros2_control`;
- los meshes de `thesis_description`.

**Pendiente inmediato:** hacer que `jaco_gazebo.launch.py` configure estas rutas automáticamente.

## 14.2 Launch

```bash
cd ~/ROS2/TESIS_IMPLEMENTATION

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch thesis_simulation jaco_gazebo.launch.py
```

---

# 15. Estado de Gazebo validado

Nodos observados:

```text
/arm_controller
/controller_manager
/gz_ros2_control
/joint_state_broadcaster
/robot_state_publisher
/rviz
```

Controladores:

```text
joint_state_broadcaster ... active
arm_controller          ... active
```

Hardware:

```text
GazeboSimSystem
state: active
```

Interfaces de comando:

```text
j2n6s300_joint_1/position [available] [claimed]
j2n6s300_joint_2/position [available] [claimed]
j2n6s300_joint_3/position [available] [claimed]
j2n6s300_joint_4/position [available] [claimed]
j2n6s300_joint_5/position [available] [claimed]
j2n6s300_joint_6/position [available] [claimed]
```

Interfaces de estado:

```text
J1 position
J1 velocity
J2 position
J2 velocity
...
J6 position
J6 velocity
```

---

# 16. Primer movimiento físico simulado

Se repitió el mismo movimiento utilizado en el mock:

```text
J1: 0.00 → 0.20 rad
Tiempo: 3 s
```

Resultado medido:

```text
j2n6s300_joint_1 = 0.1999999999999987 rad
```

Posteriormente se envió el retorno:

```text
J1: 0.20 → 0.00 rad
```

Resultado:

```text
j2n6s300_joint_1 ≈ 8.0e-21 rad
```

Para efectos prácticos:

```text
J1 = 0 rad
```

---

# 17. Verificación de estados

Comando:

```bash
ros2 topic echo /joint_states --once
```

Ejemplo de estado estable después de volver a cero:

```text
J1 ≈ 0
J2 ≈ π
J3 ≈ π
J4 ≈ 0
J5 ≈ 0
J6 ≈ 0
```

Las velocidades en reposo son valores numéricos muy cercanos a cero.

Ejemplos:

```text
~1e-13 rad/s
~1e-18 rad/s
```

Esto confirma que, a diferencia del mock, Gazebo sí está proporcionando feedback dinámico.

El campo `effort` aparece como `.nan` para J1–J6 porque actualmente no se ha configurado `effort` como `state_interface`. Esto no impide las pruebas de posición actuales.

---

# 18. Gripper

El gripper está representado visual y cinemáticamente.

Actualmente se publican sus 6 joints adicionales mediante:

```text
joint_state_broadcaster.extra_joints
```

con estado inicial cero.

Esto permite mantener:

```text
RobotModel: OK
```

en RViz aunque todavía no se implemente control independiente del gripper.

Control dinámico del gripper:

```text
PENDIENTE
```

---

# 19. Topics ROS 2 relevantes

| Topic | Función | Estado |
|---|---|---:|
| `/robot_description` | Descripción URDF | ✅ |
| `/joint_states` | Estado articular | ✅ |
| `/tf` | Transformaciones dinámicas | ✅ |
| `/tf_static` | Transformaciones estáticas | ✅ |
| `/arm_controller/joint_trajectory` | Entrada de trayectoria | ✅ |
| `/arm_controller/controller_state` | Estado interno del controlador | ⚠️ En revisión |
| `/arm_controller/state` | Estado del controlador | ⚠️ En revisión |
| `/thesis/candidate_command` | Comando candidato normalizado | ⏳ |
| `/thesis/supervised_command` | Comando validado | ⏳ |
| `/thesis/safety_state` | Estado de seguridad | ⏳ |
| `/thesis/safety_metrics` | Métricas preventivas | ⏳ |
| `/thesis/environment` | Entorno normalizado | ⏳ |

---

# 20. Estado del `JointTrajectoryController`

El controlador está activo y ejecuta correctamente trayectorias.

El log de inicio de Gazebo confirma:

```text
Controller state will be published at 50.00 Hz.
```

Sin embargo, durante las pruebas:

```bash
ros2 topic echo /arm_controller/controller_state --once
```

y:

```bash
ros2 topic echo /arm_controller/state --once
```

no entregaron un mensaje observable antes de cancelar manualmente el comando.

Esto queda registrado como una **tarea secundaria de diagnóstico**.

No bloquea actualmente el desarrollo porque:

- el comando sí se ejecuta;
- `/joint_states` refleja la posición alcanzada;
- Gazebo refleja el movimiento;
- RViz refleja el movimiento;
- `arm_controller` permanece `active`.

---

# 21. Comandos útiles de diagnóstico

## Nodos

```bash
ros2 node list
```

## Controladores

```bash
ros2 control list_controllers
```

## Hardware

```bash
ros2 control list_hardware_components
```

## Interfaces

```bash
ros2 control list_hardware_interfaces
```

## Estado articular

```bash
ros2 topic echo /joint_states --once
```

## Topics del controlador

```bash
ros2 topic list | grep arm_controller
```

## Información del tópico de estado

```bash
ros2 topic info /arm_controller/controller_state -v
```

---

# 22. Build del workspace

Build completo:

```bash
cd ~/ROS2/TESIS_IMPLEMENTATION

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

Build solo de descripción y simulación:

```bash
colcon build \
  --symlink-install \
  --packages-select thesis_description thesis_simulation
```

---

# 23. Arquitectura objetivo de la tesis

La arquitectura final pretende separar estrictamente:

```text
FUENTE DEL COMANDO
        │
        ▼
NORMALIZACIÓN
        │
        ▼
SUPERVISIÓN PREVENTIVA
        │
        ├── estado robot
        ├── percepción
        ├── predicción
        ├── geometría
        ├── distancia
        ├── TTC
        └── incertidumbre
        │
        ▼
DECISIÓN
        │
        ├── ALLOW
        ├── WARNING
        ├── REDUCTION
        └── STOP
        │
        ▼
ADAPTADOR DE SALIDA
        │
        ├── Gazebo
        └── JACO real
```

La lógica de seguridad debe vivir principalmente en:

```text
thesis_core
```

mientras que:

```text
thesis_simulation
```

y:

```text
thesis_hardware
```

solo deben adaptar entradas y salidas.

---

# 24. Topics conceptuales objetivo

## Entradas

```text
/thesis/joint_states
/thesis/environment
/thesis/candidate_command
```

## Salidas

```text
/thesis/supervised_command
/thesis/safety_state
/thesis/safety_metrics
```

---

# 25. Requisitos de validación objetivo

Estos valores corresponden a los objetivos experimentales definidos para la plataforma y deberán ser verificados durante las etapas posteriores.

| Métrica | Objetivo |
|---|---:|
| Frecuencia de validación preventiva | ≥ 10 Hz |
| Latencia de decisión / procesamiento | ≤ 100 ms |
| Actualización del estado del robot | ≥ 50 Hz |
| Percepción RGB-D | ≥ 15 FPS |
| Distancia mínima experimental inicial | 5 cm |
| Error de estimación geométrica objetivo | ≤ 2 cm |
| Colisiones en escenarios controlados validados | 0 |
| Intercepción de comandos candidatos | 100 % |

Estos valores deben tratarse como **requisitos experimentales del proyecto**, no como límites universales de seguridad industrial.

---

# 26. Siguientes pasos de implementación

A partir del estado actual, el desarrollo continuará en el siguiente orden.

---

## Fase 1 — Reproducibilidad del digital twin

### Objetivo

Ejecutar el digital twin con un único launch desde una terminal limpia.

Actualmente se requiere configurar:

```text
IGN_GAZEBO_SYSTEM_PLUGIN_PATH
IGN_GAZEBO_RESOURCE_PATH
```

Pendiente:

- mover estas rutas al launch;
- eliminar los exports manuales;
- verificar arranque reproducible;
- documentar la ejecución definitiva.

Resultado esperado:

```bash
ros2 launch thesis_simulation jaco_gazebo.launch.py
```

sin preparación adicional.

---

## Fase 2 — Nodo de comandos candidatos

Crear una fuente de prueba que permita reemplazar los largos comandos:

```bash
ros2 topic pub ...
```

por una interfaz propia de la tesis.

Arquitectura:

```text
test_command_node
        │
        ▼
/thesis/candidate_command
```

El mensaje candidato debe contener como mínimo:

- joints;
- posiciones objetivo;
- tiempo de ejecución;
- timestamp;
- identificador del comando.

---

## Fase 3 — Supervisor transparente inicial

Primera versión de `thesis_core`.

Inicialmente:

```text
candidate_command
        │
        ▼
supervisor
        │
        ▼
ALLOW siempre
        │
        ▼
supervised_command
```

El objetivo de esta fase será validar la arquitectura de intercepción **antes de implementar seguridad**.

---

## Fase 4 — Adaptador hacia `arm_controller`

Implementar:

```text
/thesis/supervised_command
        │
        ▼
JointTrajectory
        │
        ▼
/arm_controller/joint_trajectory
```

Esto desacoplará completamente la lógica principal del controlador concreto.

---

## Fase 5 — Obstáculo controlado en Gazebo

Agregar un obstáculo básico:

```text
cubo
```

con:

- dimensiones conocidas;
- pose conocida;
- frame conocido;
- posición controlable.

Primer escenario:

```text
JACO                CUBO
  \                  █
   \_________________█
        distancia d
```

---

## Fase 6 — Representación común del entorno

Crear:

```text
/thesis/environment
```

La primera versión podrá utilizar objetos geométricos simples.

Más adelante se alimentará desde percepción RGB-D.

---

## Fase 7 — Geometría simplificada del robot

Implementar una representación eficiente para cálculos preventivos.

Principal alternativa:

```text
cápsulas
```

por link.

Ejemplo conceptual:

```text
o==========o
```

Ventajas:

- cálculo rápido;
- distancia punto-segmento;
- fácil incorporación de margen;
- adecuada para evaluación ≥10 Hz.

---

## Fase 8 — Distancia mínima

Calcular:

```text
d_min
```

entre:

```text
geometría del robot
```

y:

```text
geometría del entorno
```

Validaciones iniciales:

- robot quieto;
- obstáculo quieto;
- varias posiciones conocidas;
- comparación con referencia.

---

## Fase 9 — Predicción cinemática a corto horizonte

A partir de:

```text
q(t)
```

y:

```text
candidate_command
```

predecir:

```text
q(t + Δt)
```

para múltiples muestras en un horizonte corto.

Flujo:

```text
q0
 │
 ├── q1
 ├── q2
 ├── q3
 └── qN
```

En cada estado se evaluará la proximidad al entorno.

---

## Fase 10 — Distancia mínima proyectada

Calcular:

```text
d_min_projected
```

sobre toda la trayectoria predicha.

Esto permitirá detectar comandos que todavía son seguros en el instante actual, pero llevarían al manipulador hacia una colisión.

---

## Fase 11 — Time-to-Collision

Implementar TTC utilizando:

- distancia;
- movimiento relativo;
- evolución temporal proyectada.

Salida conceptual:

```text
TTC = tiempo estimado hasta condición de colisión
```

---

## Fase 12 — Clasificación de riesgo

Estados iniciales:

| Estado | Interpretación |
|---|---|
| SAFE | Movimiento permitido |
| WARNING | Movimiento permitido con alerta |
| REDUCTION | Movimiento limitado |
| CRITICAL | Movimiento bloqueado |

---

## Fase 13 — Intervención preventiva

Acciones:

```text
SAFE
  → command unchanged

WARNING
  → command + warning

REDUCTION
  → scaled / limited command

CRITICAL
  → reject / stop
```

Toda intervención debe registrar:

- comando original;
- comando resultante;
- motivo;
- métricas utilizadas;
- timestamp;
- latencia de decisión.

---

## Fase 14 — Incertidumbre

Incorporar margen adicional debido a:

- error de percepción;
- error geométrico;
- frecuencia de actualización;
- latencia;
- simplificación por cápsulas;
- incertidumbre de estado.

---

## Fase 15 — RGB-D

Integrar el sensor seleccionado.

Pipeline esperado:

```text
RGB-D
 │
 ▼
PointCloud / Depth
 │
 ▼
Extracción de obstáculos
 │
 ▼
Transformación TF
 │
 ▼
/thesis/environment
```

Métricas:

- FPS;
- latencia;
- precisión;
- error de profundidad;
- estabilidad temporal.

---

## Fase 16 — Adaptador del JACO físico

Solo después de validar la lógica en simulación.

Responsabilidades:

```text
thesis_hardware
├── leer joints reales
├── normalizar estado
├── recibir supervised_command
└── adaptar comando al driver Kinova
```

El objetivo es que `thesis_core` **no cambie** al pasar de simulación a hardware.

---

## Fase 17 — Soft gripper y FSR

Integrar:

- gripper TPU;
- sensores FSR;
- adquisición de contacto;
- logging.

Los FSR serán una señal complementaria para confirmar contacto local y analizar interacción física.

No reemplazarán al supervisor preventivo basado en distancia / predicción.

---

## Fase 18 — UI y logging

Desarrollar visualización de:

```text
Safety State
Current d_min
Projected d_min
TTC
Candidate Command
Supervised Command
Intervention
Latency
```

Además:

- guardar rosbag;
- guardar CSV de métricas;
- registrar escenarios;
- facilitar repetibilidad experimental.

---

## Fase 19 — Validación experimental

La validación se realizará progresivamente.

### Etapa A — Simulación

- obstáculos estáticos;
- obstáculos dinámicos;
- comandos seguros;
- comandos inseguros;
- aproximaciones rápidas;
- trayectorias cercanas al límite.

### Etapa B — Hardware controlado

- velocidades reducidas;
- área experimental limitada;
- supervisión humana;
- escenarios previamente validados en simulación.

---

# 27. Escenarios de prueba previstos

| ID | Escenario | Resultado esperado |
|---|---|---|
| T01 | Robot quieto, obstáculo lejano | SAFE |
| T02 | Movimiento alejándose del obstáculo | SAFE |
| T03 | Aproximación moderada | WARNING |
| T04 | Aproximación al margen mínimo | REDUCTION |
| T05 | Trayectoria proyectada hacia colisión | CRITICAL |
| T06 | Obstáculo entra en trayectoria | Intervención preventiva |
| T07 | Percepción con incertidumbre elevada | Mayor margen |
| T08 | Cambio rápido de comando | Revalidación |
| T09 | Fuente de comando diferente | Misma capa preventiva |
| T10 | Ejecución equivalente simulación/hardware | Arquitectura desacoplada |

---

# 28. Evidencia que deberá registrarse

Para cada escenario:

```text
timestamp
candidate_command
supervised_command
joint_states
environment
d_min_current
d_min_projected
TTC
risk_state
intervention
decision_latency
total_latency
```

Preferentemente mediante:

```text
rosbag2
+
CSV de métricas procesadas
```

---

# 29. Principio de diseño principal

La implementación debe conservar siempre esta separación:

```text
             thesis_core
          safety algorithms
                 │
        ┌────────┴────────┐
        │                 │
simulation adapter   hardware adapter
        │                 │
        ▼                 ▼
     Gazebo            JACO real
```

No se deben duplicar:

- cálculo de distancia;
- TTC;
- predicción;
- clasificación de riesgo;
- lógica de intervención.

Los paquetes de simulación y hardware solo deben adaptar interfaces.

---

# 30. Estado actual resumido

El proyecto se encuentra actualmente en el punto:

```text
Robot description             ✅
        ↓
RViz                          ✅
        ↓
ros2_control mock             ✅
        ↓
control articular             ✅
        ↓
Gazebo Fortress               ✅
        ↓
gz_ros2_control               ✅
        ↓
GazeboSimSystem               ✅
        ↓
feedback position + velocity  ✅
        ↓
trayectoria J1 validada       ✅
        ↓
────────────────────────────────
        ↓
launch reproducible           ← SIGUIENTE
        ↓
candidate_command
        ↓
supervisor pass-through
        ↓
obstáculo simulado
        ↓
distancia mínima
        ↓
predicción
        ↓
TTC
        ↓
intervención preventiva
        ↓
RGB-D
        ↓
JACO real
        ↓
validación experimental
```

---

# 31. Nota de seguridad

Durante la etapa actual:

> El control y las pruebas de movimiento se realizan únicamente sobre el digital twin.

El manipulador físico no debe conectarse al flujo de control del simulador ni recibir comandos hasta que:

1. la arquitectura preventiva haya sido validada en simulación;
2. existan límites de movimiento apropiados;
3. se hayan definido procedimientos de prueba;
4. se realicen pruebas iniciales controladas y supervisadas.

---

# 32. Licencia y atribución

La descripción del Kinova JACO reutiliza recursos provenientes del repositorio de Kinova Robotics.

Consultar:

```text
src/thesis_description/UPSTREAM.md
src/thesis_description/KINOVA_LICENSE
```

para información de procedencia y licencia de los recursos originales.

---

# 33. Actualización de avance — 27/08/2026

Esta sección registra únicamente los avances realizados después de la versión
anterior del README. El contenido previo se conserva sin modificaciones.

## 33.1 Digital twin reproducible

Se completó la tarea pendiente de hacer que el launch de Gazebo sea
autosuficiente.

El archivo:

```text
src/thesis_simulation/launch/jaco_gazebo.launch.py
```

ahora configura internamente las rutas necesarias para que Gazebo Fortress
encuentre:

- los plugins de `gz_ros2_control`;
- los recursos y meshes de `thesis_description`.

Por tanto, ya no es necesario ejecutar manualmente:

```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=...
export IGN_GAZEBO_RESOURCE_PATH=...
```

antes del launch.

La prueba fue realizada desde una terminal sin dichas variables exportadas y se
verificó nuevamente:

```text
joint_state_broadcaster  active
arm_controller           active
GazeboSimSystem          active
/joint_states            operativo
```

Commit asociado:

```text
f512425 Make JACO Gazebo launch self-contained
```

Estado:

```text
Reproducibilidad del digital twin: COMPLETADA
```

---

## 33.2 Interfaz común de comandos candidatos

Se implementó la primera interfaz propia de la arquitectura desacoplada:

```text
src/thesis_interfaces/msg/JointCommand.msg
```

Contenido:

```text
builtin_interfaces/Time stamp
string command_id
string[] joint_names
float64[] positions
builtin_interfaces/Duration duration
```

Esta interfaz permite representar un comando articular candidato sin acoplar
`thesis_core` directamente a:

```text
trajectory_msgs/msg/JointTrajectory
```

o al controlador concreto utilizado actualmente.

La separación queda:

```text
Fuente de comando
      ↓
JointCommand
      ↓
thesis_core
      ↓
adaptador específico
      ↓
controlador
```

Validación realizada:

```bash
ros2 interface show thesis_interfaces/msg/JointCommand
```

ROS 2 reconoció correctamente todos los campos del mensaje.

Commit asociado:

```text
25ef660 Add candidate joint command interface and test publisher
```

---

## 33.3 Nodo generador de comandos candidatos

Se implementó:

```text
src/thesis_simulation/thesis_simulation/test_command_node.py
```

Ejecutable:

```text
thesis_simulation test_command
```

El nodo publica comandos de prueba en:

```text
/thesis/candidate_command
```

Prueba realizada:

```bash
ros2 run thesis_simulation test_command
```

Salida validada:

```text
Candidate command published:
J1=0.200 rad
duration=3.00 s
```

Ejemplo del mensaje recibido:

```text
command_id: test_<timestamp>

joint_names:
- j2n6s300_joint_1
- j2n6s300_joint_2
- j2n6s300_joint_3
- j2n6s300_joint_4
- j2n6s300_joint_5
- j2n6s300_joint_6

positions:
- 0.2
- 3.141592653589793
- 3.141592653589793
- 0.0
- 0.0
- 0.0

duration:
  sec: 3
  nanosec: 0
```

El nodo puede recibir parámetros desde ROS 2, por ejemplo:

```bash
ros2 run thesis_simulation test_command \
  --ros-args \
  -p joint_1_target:=0.35 \
  -p duration_sec:=4.0
```

Este nodo representa temporalmente una fuente externa de movimiento.

Posteriormente podrá ser sustituido por:

- GUI;
- joystick;
- MoveIt;
- otro planificador;
- otra fuente externa;

sin cambiar la lógica central de supervisión.

---

## 33.4 Topic `/thesis/candidate_command`

El topic:

```text
/thesis/candidate_command
```

ya se encuentra implementado y validado.

Tipo:

```text
thesis_interfaces/msg/JointCommand
```

Prueba de recepción:

```bash
ros2 topic echo \
/thesis/candidate_command \
thesis_interfaces/msg/JointCommand
```

Estado:

```text
/thesis/candidate_command: OPERATIVO
```

---

## 33.5 Supervisor pass-through inicial

Se implementó el primer supervisor en:

```text
src/thesis_core/thesis_core/safety_supervisor_node.py
```

Ejecutable:

```text
thesis_core safety_supervisor
```

Entrada:

```text
/thesis/candidate_command
```

Salida:

```text
/thesis/supervised_command
```

La versión actual todavía no implementa la evaluación preventiva completa.

Actualmente realiza:

- validación de que exista al menos un joint;
- validación de igualdad entre longitud de `joint_names` y `positions`;
- rechazo de posiciones no finitas;
- validación de duración mayor que cero;
- cálculo inicial de latencia de recepción;
- publicación de comandos válidos;
- rechazo de comandos estructuralmente inválidos.

El comportamiento actual es:

```text
comando válido
      ↓
ALLOW
      ↓
/thesis/supervised_command
```

y:

```text
comando inválido
      ↓
REJECTED
      ↓
no se publica salida supervisada
```

---

## 33.6 Primera prueba ALLOW

Se utilizó el comando candidato:

```text
J1 = 0.20 rad
J2 = π rad
J3 = π rad
J4 = 0 rad
J5 = 0 rad
J6 = 0 rad

duration = 3 s
```

Resultado del supervisor:

```text
ALLOW id=test_1787793366439802876 | joints=6 | input_latency=1.432 ms
```

El mismo comando fue observado correctamente en:

```text
/thesis/supervised_command
```

Por tanto, ya se verificó el flujo:

```text
test_command_node
        ↓
/thesis/candidate_command
        ↓
safety_supervisor_node
        ↓
ALLOW
        ↓
/thesis/supervised_command
```

---

## 33.7 Primera medición de latencia

La primera medición registrada fue:

```text
input_latency = 1.432 ms
```

Esta medición corresponde únicamente al intervalo entre:

```text
timestamp del comando candidato
```

y:

```text
recepción del mensaje por thesis_core
```

Todavía no representa la latencia preventiva total porque aún no se ejecutan:

- modelado geométrico;
- predicción;
- distancia mínima;
- TTC;
- márgenes de incertidumbre;
- clasificación de riesgo.

Esta medición será utilizada como línea base para las futuras pruebas de
latencia.

Requisito experimental de referencia:

```text
latencia de decisión / procesamiento ≤ 100 ms
```

---

## 33.8 Prueba de rechazo estructural

Se publicó manualmente un comando inválido:

```text
command_id: invalid_test

joint_names:
- j2n6s300_joint_1
- j2n6s300_joint_2

positions:
- 0.2

duration:
  sec: 3
```

El mensaje contenía:

```text
2 joint_names
1 position
```

Resultado:

```text
REJECTED invalid_test:
joint_names and positions have different sizes
```

El comando inválido no apareció en:

```text
/thesis/supervised_command
```

Esto confirma que el supervisor ya funciona como un punto real de
intercepción y puede detener comandos antes de que continúen hacia el
controlador.

---

## 33.9 Estado de `/thesis/supervised_command`

El topic:

```text
/thesis/supervised_command
```

ya está implementado.

Tipo:

```text
thesis_interfaces/msg/JointCommand
```

Estado:

```text
comando válido   → publicado
comando inválido → no publicado
```

Prueba utilizada:

```bash
ros2 topic echo \
/thesis/supervised_command \
thesis_interfaces/msg/JointCommand
```

---

## 33.10 Correspondencia con el flujo funcional de la tesis

La implementación continúa siguiendo el flujo funcional definido para el
subsistema computacional.

| Función definida en el flujo de la tesis | Implementación actual | Estado |
|---|---|---:|
| Adquirir comandos candidatos externos | `test_command_node` como fuente de prueba | ✅ |
| Normalizar comando candidato | `JointCommand.msg` | ✅ Inicial |
| Comando candidato adquirido | `/thesis/candidate_command` | ✅ |
| Evaluar condiciones preventivas | `safety_supervisor_node` | 🟡 Básico |
| Clasificar / decidir | `ALLOW` / `REJECT` estructural | 🟡 Básico |
| Generar comando supervisado | `/thesis/supervised_command` | ✅ |
| Convertir y enviar comando supervisado al controlador | `trajectory_adapter_node` | ⏳ |
| Controlador del manipulador | `arm_controller` | ✅ |
| Manipulador | JACO digital twin en Gazebo | ✅ |

La evaluación preventiva completa todavía deberá incorporar:

```text
estado cinemático del robot
        ↓
estimación del estado
        ↓
actualización de representación geométrica
        ↓
predicción cinemática de corto horizonte
        ↓
representación del entorno
        ↓
distancia robot-entorno
        ↓
métricas de riesgo
        ↓
TTC
        ↓
márgenes de incertidumbre
        ↓
restricciones cinemáticas y espaciales
        ↓
clasificación de zona de seguridad
        ↓
selección de acción preventiva
```

La señal de parada de emergencia permanecerá como una rama de prioridad
independiente, coherente con el flujo funcional de la tesis.

---

## 33.11 Correspondencia con los paquetes ROS 2

La separación actual es:

| Función | Paquete |
|---|---|
| Mensajes comunes | `thesis_interfaces` |
| Fuente de prueba | `thesis_simulation` |
| Supervisor | `thesis_core` |
| Digital twin | `thesis_simulation` |
| Descripción del robot | `thesis_description` |
| Adaptador al robot real | `thesis_hardware` — pendiente |
| UI y métricas | `thesis_ui` — pendiente |

Esta organización mantiene la regla de no duplicar los algoritmos preventivos
entre simulación y hardware.

---

## 33.12 Estado actual de la cadena

El flujo actualmente implementado es:

```text
test_command_node                ✅
        ↓
/thesis/candidate_command        ✅
        ↓
safety_supervisor_node           ✅
        ↓
validación estructural           ✅
        ↓
ALLOW / REJECT básico            ✅
        ↓
/thesis/supervised_command       ✅
        ↓
trajectory_adapter              ← SIGUIENTE
        ↓
/arm_controller/joint_trajectory
        ↓
JACO Gazebo
```

Por tanto, en este momento:

```text
ros2 run thesis_simulation test_command
```

todavía **no debe mover el JACO**.

El comportamiento es correcto porque todavía falta el adaptador de salida.

---

## 33.13 Siguiente bloque: `trajectory_adapter_node`

El siguiente nodo deberá recibir:

```text
/thesis/supervised_command
```

y convertir:

```text
thesis_interfaces/msg/JointCommand
```

a:

```text
trajectory_msgs/msg/JointTrajectory
```

para publicar posteriormente en:

```text
/arm_controller/joint_trajectory
```

Arquitectura objetivo inmediata:

```text
test_command_node
        ↓
/thesis/candidate_command
        ↓
safety_supervisor_node
        ↓
/thesis/supervised_command
        ↓
trajectory_adapter_node
        ↓
/arm_controller/joint_trajectory
        ↓
GazeboSimSystem
        ↓
JACO
```

Esta prueba permitirá demostrar por primera vez que el manipulador simulado
recibe movimiento únicamente después de pasar por la capa de supervisión.

Criterios de validación previstos:

```text
supervisor activo + ALLOW
        → JACO se mueve

comando REJECTED
        → JACO no recibe movimiento

supervisor apagado
        → no existe supervised_command
        → JACO no recibe movimiento desde la interfaz de tesis
```

---

## 33.14 Siguiente entrada del supervisor: estado cinemático normalizado

Después del adaptador de trayectorias se deberá incorporar el estado del robot
al flujo del supervisor.

No se recomienda que `thesis_core` dependa directamente de:

```text
/joint_states
```

producido por Gazebo.

La arquitectura objetivo será:

```text
Gazebo
  ↓
/joint_states
  ↓
thesis_simulation adapter
  ↓
/thesis/joint_states
  ↓
thesis_core
```

Para el manipulador real:

```text
JACO real
  ↓
driver / API Kinova
  ↓
thesis_hardware adapter
  ↓
/thesis/joint_states
  ↓
thesis_core
```

De esta forma, `thesis_core` utilizará la misma interfaz normalizada sin saber
si el origen es simulación o hardware.

---

## 33.15 Orden actualizado de implementación

Desde el estado actual, el orden previsto es:

```text
1.  JointCommand.msg                          ✅
2.  test_command_node                         ✅
3.  /thesis/candidate_command                 ✅
4.  supervisor pass-through                   ✅
5.  ALLOW / REJECT estructural                ✅
6.  /thesis/supervised_command                ✅
7.  corregir cierre limpio del supervisor     ← ajuste menor
8.  trajectory_adapter_node                   ← SIGUIENTE BLOQUE
9.  movimiento JACO pasando por supervisor
10. /thesis/joint_states
11. estado cinemático normalizado
12. obstáculo controlado en Gazebo
13. /thesis/environment
14. representación geométrica del robot
15. distancia robot-entorno
16. predicción cinemática
17. distancia mínima proyectada
18. TTC
19. márgenes de incertidumbre
20. restricciones cinemáticas y espaciales
21. SAFE / WARNING / REDUCTION / CRITICAL
22. intervención preventiva
23. logging y métricas
24. parada de emergencia
25. RGB-D
26. JACO real
27. soft gripper + FSR
28. validación experimental
```

---

## 33.16 Observación pendiente del cierre del supervisor

Durante la prueba funcional, el nodo operó correctamente, pero al cerrarlo con:

```text
Ctrl+C
```

se observó:

```text
RCLError:
failed to shutdown:
rcl_shutdown already called on the given context
```

Este error aparece únicamente durante el cierre del proceso y no invalida las
pruebas de:

```text
ALLOW
REJECT
/thesis/supervised_command
```

La corrección prevista es proteger el shutdown mediante:

```python
if rclpy.ok():
    rclpy.shutdown()
```

Esta corrección es menor y debe aplicarse antes de integrar el
`trajectory_adapter_node`.

---

## 33.17 Estado actualizado resumido

```text
Robot description               ✅
        ↓
RViz                            ✅
        ↓
ros2_control mock               ✅
        ↓
control articular               ✅
        ↓
Gazebo Fortress                 ✅
        ↓
gz_ros2_control                 ✅
        ↓
GazeboSimSystem                 ✅
        ↓
feedback position + velocity    ✅
        ↓
trayectoria J1 validada         ✅
        ↓
launch reproducible             ✅
        ↓
JointCommand.msg                ✅
        ↓
test_command_node               ✅
        ↓
/thesis/candidate_command       ✅
        ↓
safety_supervisor pass-through  ✅
        ↓
ALLOW / REJECT estructural      ✅
        ↓
/thesis/supervised_command      ✅
        ↓
trajectory_adapter              ← SIGUIENTE
        ↓
movimiento supervisado JACO
        ↓
/thesis/joint_states
        ↓
obstáculo simulado
        ↓
/thesis/environment
        ↓
representación geométrica
        ↓
distancia mínima
        ↓
predicción
        ↓
TTC
        ↓
intervención preventiva
        ↓
RGB-D
        ↓
JACO real
        ↓
validación experimental
```


