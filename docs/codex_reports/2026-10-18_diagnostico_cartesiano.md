# Diagnóstico del control cartesiano en simulación

Fecha: 2026-10-18. Diagnóstico estático: no se inició ROS, Gazebo, hardware ni se publicaron comandos.

## Flujo identificado

| Ruta GUI | Tópico y tipo | Nodo receptor | Salida articular |
| --- | --- | --- | --- |
| Objetivo puntual X/Y/Z | `/thesis/cartesian_candidate` (`CartesianCommand`) | `cartesian_command_adapter` | `/thesis/candidate_command` (`JointCommand`) |
| JOG por rueda X/Y/Z | `/thesis/cartesian_jog_intent` (`TwistStamped`) | `cartesian_jog_adapter` | `/thesis/jog_intent` (`JointCommand`) |
| Trayectoria larga | `/thesis/cartesian_trajectory_candidate` (`CartesianTrajectoryCommand`) | `cartesian_trajectory_adapter` | `/thesis/jog_intent` (`JointCommand`) |

Desde `/thesis/candidate_command` o `/thesis/jog_intent`, el supervisor preventivo de `thesis_core` procesa la orden y la salida de simulación consume los comandos supervisados. `thesis_system.launch.py` inicia los tres adaptadores y usa `simulation_output_enabled`; no se identificó una ruta cartesiana hacia `hardware_output`.

## 1. Generación de X/Y/Z

La GUI tiene tres flujos distintos:

- **Puntual:** `copy_current_cartesian_pose()` lee TF `world -> j2n6s300_end_effector`; los controles ajustan X/Y/Z en pasos de 0.01 m. `send_cartesian_candidate()` conserva X/Y/Z en metros, convierte solo R/P/Y a radianes y publica un objetivo absoluto `frame_id=world`.
- **JOG:** cada 50 ms, `publish_cartesian_jog_intent()` publica las velocidades de X/Y/Z en `TwistStamped.twist.linear`, m/s y marco `world`. Sus límites de interfaz son 0.15 m/s y el valor inicial es 0.04 m/s. Un twist nulo solicita HOLD.
- **Trayectoria:** `send_cartesian_trajectory()` publica X/Y/Z en metros más modo `RELATIVE` o `ABSOLUTE` y velocidades nominales. En `RELATIVE`, XYZ son deltas de la TF actual; en `ABSOLUTE`, son coordenadas de `world`.

`CartesianCommand` define pose, `command_id`, `frame_id` y duración. `CartesianTrajectoryCommand` define esos valores cartesianos, el modo y velocidades lineal/angular. La GUI exige `/joint_states` reciente, estado `READY`, adaptador suscrito, y que no haya otra ruta de movimiento activa.

## 2. Conversión a articulaciones

`jaco_kinematics.py` calcula FK para seis articulaciones. `cartesian_kinematics.py` usa IK local por Jacobiano numérico y mínimos cuadrados amortiguados: hasta 120 iteraciones, paso máximo de 0.15 rad y límites articulares en cada iteración.

Antes de IK, los adaptadores alinean el objetivo TF vivo con la referencia del modelo:

```text
T_model_target = T_model_current · inverse(T_world_current) · T_world_target
```

El adaptador puntual resuelve una IK y valida que la duración permita las velocidades articulares. El JOG integra el twist durante un paso limitado a 5 mm o 1°, resuelve IK y extrapola la velocidad articular a un horizonte de 1 s. La trayectoria larga interpola posición linealmente y orientación con SLERP; la divide en pasos de hasta 1 cm o 2°, encadena IK con la solución anterior como semilla y publica objetivos rodantes de 1 s.

## 3. Dónde puede perderse o bloquearse el movimiento

1. **Bloqueo en GUI:** faltan `/joint_states`, `READY`, suscriptor del adaptador, TF copiada para el objetivo puntual, o existe un JOG/ejecución/trayectoria activa.
2. **TF ausente o desfasada:** los tres adaptadores requieren `world -> j2n6s300_end_effector`. Puntual y trayectoria rechazan; JOG publica HOLD. Un desfase temporal entre TF y `/joint_states` puede dar una semilla articular distinta de la pose usada para la alineación.
3. **Límites de entrada:** puntual rechaza pasos mayores que 0.10 m o 20°; trayectoria rechaza más de 0.60 m, 180° o velocidades fuera de rango. JOG valida la norma del vector completo contra 0.15 m/s, por lo que una combinación de ejes puede exceder el límite aunque cada componente parezca razonable.
4. **IK local:** cerca de singularidades, límites articulares o posturas alejadas de la recomendada, puede no converger. JOG devuelve HOLD; puntual y trayectoria rechazan la solicitud.
5. **Duración y velocidad:** puntual exige duración suficiente para todas las articulaciones. Trayectoria exige una duración no menor que el mínimo articular multiplicado por el margen 1.20.
6. **Supervisor preventivo:** la conversión IK no implica ejecución. En trayectoria, `speed_scale=0` deja el estado `PAUSED`; si una decisión preventiva caduca tras 0.50 s, termina en `FAILED` y HOLD.
7. **Rutas distintas:** un error exclusivo del JOG debe concentrarse en la rueda, `cartesian_jog_adapter` o su IK de pasos de 5 mm. Un error del botón puntual debe concentrarse antes en `cartesian_command_adapter` y sus límites de 10 cm.

## Evidencia existente

- `thesis_core/test/test_cartesian_kinematics.py`: convergencia local, objetivo inalcanzable, alineación del offset TF y tolerancia de 3 mm.
- `thesis_core/test/test_cartesian_path.py`: interpolación, duración y discretización de ruta.
- `thesis_simulation/test/test_cartesian_jog_model.py`: integración de X/Y/Z en `world`, límites, paso local e IK de seis ejes desde la postura recomendada.
- `thesis_simulation/test/test_cartesian_trajectory_model.py`: avance condicionado por `speed_scale` y horizonte rodante.
- `thesis_ui/test/test_gui_state.py`: velocidad cartesiana acotada y bloqueo mutuo entre modos.

## Pruebas seguras recomendadas

No se ejecutaron. Estas pruebas no inician Gazebo, hardware ni publican movimientos:

```bash
colcon build --packages-select thesis_interfaces thesis_core thesis_simulation thesis_ui
source install/setup.bash
colcon test --packages-select thesis_core thesis_simulation thesis_ui --pytest-args -q
colcon test-result --verbose
```

Una futura revisión de integración debe comenzar con salida de simulación desactivada y sin GUI, para inspeccionar solo nodos, TF, tópicos y suscriptores. Requiere autorización explícita porque iniciaría la pila ROS.

## Conclusión

El código revisado mantiene las acciones cartesianas fuera del modo hardware y no revela una ruta hacia el Jaco2 físico. La causa exacta no puede confirmarse sin telemetría de ejecución. Las causas más probables, por ser exclusivas de la ruta cartesiana mientras joints funciona, son TF o desfase TF/estado articular, no convergencia de IK local, y límites específicos de paso, duración o `speed_scale`.
