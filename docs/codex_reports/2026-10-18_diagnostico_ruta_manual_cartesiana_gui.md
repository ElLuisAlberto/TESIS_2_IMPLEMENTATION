# Diagnóstico estático — ruta manual cartesiana de la GUI

Fecha: 2026-10-18  
Método: lectura estática exclusiva de GUI, mensajes y adaptadores cartesianos.
No se modificó ni ejecutó código, ROS, Gazebo, GUI, cámara ni hardware.

## Resultado

La GUI dispone de tres rutas cartesianas simuladas, que no equivalen a la ruta
manual por joints: objetivo puntual, JOG continuo y trayectoria larga. Todas
usan `world`, pero solo JOG es una orden de velocidad; las otras son una pose
objetivo o un desplazamiento/pose de trayectoria.

| Control | Valores y signo en GUI | Relativo o absoluto | Publicación | Adaptador receptor |
| --- | --- | --- | --- | --- |
| JOG por rueda X/Y/Z/Roll/Pitch/Yaw | Una rueda positiva suma intención positiva; X/Y/Z en m/s, Roll/Pitch/Yaw en °/s convertidos a rad/s. Predeterminados: 0.04 m/s y 10 °/s; máximos: 0.15 m/s y 30 °/s. | Velocidad momentánea, no destino. Expira a cero tras 0.25 s sin pulsos. | `/thesis/cartesian_jog_intent`, `geometry_msgs/TwistStamped`, `header.frame_id=world`; `linear.{x,y,z}` y `angular.{x,y,z}`. | `cartesian_jog_adapter.py`, que produce `JointCommand` en `/thesis/jog_intent`. |
| Objetivo cartesiano puntual | Botones ±: X/Y/Z ±0.01 m; Roll/Pitch/Yaw ±5°. La GUI convierte las rotaciones a rad. | **Absoluto**: primero copia la pose `world` actual y luego altera los seis campos. | `/thesis/cartesian_candidate`, `CartesianCommand`, `frame_id=world`, metros y radianes. | `cartesian_command_adapter.py`, que produce `JointCommand` en `/thesis/candidate_command`. |
| Trayectoria cartesiana | X/Y/Z en m; orientación en ° convertidos a rad; velocidad lineal en m/s y angular en °/s convertidos a rad/s. | Selector: `RELATIVE` por defecto (deltas de la pose medida) o `ABSOLUTE` en `world`. | `/thesis/cartesian_trajectory_candidate`, `CartesianTrajectoryCommand`, `frame_id=world`. | `cartesian_trajectory_adapter.py`, que planifica IK y publica `JointCommand` rodante en `/thesis/jog_intent`. |

Evidencia principal: GUI [publicadores y mensajes](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/src/thesis_ui/thesis_ui/joint_control_gui.py:236), [mapeo JOG](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/src/thesis_ui/thesis_ui/joint_control_gui.py:2828), [objetivo puntual](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/src/thesis_ui/thesis_ui/joint_control_gui.py:3009) y [trayectoria](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/src/thesis_ui/thesis_ui/joint_control_gui.py:3092). Los contratos definen metros y RPY intrínseco en radianes: [CartesianCommand](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/src/thesis_interfaces/msg/CartesianCommand.msg:1) y [CartesianTrajectoryCommand](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/src/thesis_interfaces/msg/CartesianTrajectoryCommand.msg:1).

## Signos y semántica de orientación

No hay inversión de signo en la GUI: el signo de la rueda o de los botones se
copia al componente X/Y/Z o Roll/Pitch/Yaw correspondiente. Para JOG, los tres
componentes lineales y angulares se interpretan en `world`; la integración
aplica el incremento angular en el lado mundial. En cambio, los mensajes de
pose usan RPY intrínseco, y la trayectoria relativa suma los deltas RPY a la
orientación RPY medida. Por ello, aunque las etiquetas sean las mismas, una
rotación continua y una pose RPY puntual no tienen una equivalencia geométrica
trivial cerca de singularidades o cambios grandes de orientación.

## Por qué joints puede funcionar mientras X/Y/Z se comporta distinto

La ruta por joints publica directamente un `JointCommand`; no necesita TF del
efector ni resolver IK. La ruta cartesiana, por el contrario, exige:

1. `JointState` completo y reciente;
2. TF disponible `world -> j2n6s300_end_effector`;
3. conversión entre el TF medido y el modelo mediante `align_relative_target`;
4. una IK local alcanzable; y
5. límites adicionales de paso/integración/velocidad.

En el objetivo puntual se rechazan pasos mayores de 0.10 m o 20°; el JOG
integra como máximo 5 mm y 1° por paso y, si falta TF, falla IK o se supera un
límite, publica el objetivo articular actual (HOLD). La propia GUI advierte que
la postura vertical puede ser singular. Así, una orden articular puede ser
aceptable mientras una intención X/Y/Z equivalente no genera movimiento por
TF, IK, frame, alcance o singularidad; no hay evidencia estática de una
inversión X/Y/Z en la GUI.

Evidencia: el adaptador puntual exige `world`, estado y TF
[aquí](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/src/thesis_simulation/thesis_simulation/cartesian_command_adapter.py:162); el JOG valida `world`, integra e invoca IK
[aquí](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/src/thesis_simulation/thesis_simulation/cartesian_jog_adapter.py:184); y la trayectoria resuelve modo relativo/absoluto
[aquí](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/src/thesis_simulation/thesis_simulation/cartesian_trajectory_adapter.py:237).

## Cobertura automática faltante

`test_gui_state.py` solo demuestra que existen seis controles, que se puede
incrementar X en el formulario puntual y que una intención positiva de rueda
en X llega al *fake node* con velocidad acotada. No captura un mensaje ROS
real ni cubre Y, Z, Roll, Pitch, Yaw, signo negativo, conversión °→rad,
selección `RELATIVE`/`ABSOLUTE`, ni el adaptador que transforma la intención en
`JointCommand` o HOLD.

Falta una prueba headless extremo a extremo GUI→mensaje→adaptador, con TF y
`JointState` falsos controlados, parametrizada para los seis ejes y ambos
signos. Debe comprobar el campo exacto y sus unidades en `TwistStamped`,
`CartesianCommand` y `CartesianTrajectoryCommand`, y que el adaptador recibe
la orden correcta y produce un `JointCommand` esperado o un HOLD explícito.
Eso demostraría la GUI completa sin iniciar ROS ni simulación.

