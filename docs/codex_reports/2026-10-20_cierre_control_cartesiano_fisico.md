# Cierre del control cartesiano físico continuo

Fecha: 2026-10-20  
Dictamen: **BLOQUEADO PARA LA APERTURA ARMABLE FINAL**

## Resultado

La ruta cartesiana continua X/Y/Z/Roll/Pitch/Yaw quedó integrada como:

`GUI -> /thesis/cartesian_jog_intent -> cartesian_jog_adapter ->
/thesis/jog_intent -> safety_supervisor ->
/thesis/supervised_jog_command -> jaco_hardware_adapter -> Kinova`.

No existe una ruta GUI–Kinova directa. El brazo no fue armado ni recibió
movimiento físico durante este cierre.

La apertura final solicitada no se realizó. Con el argumento obligatorio
`use_demo_obstacle=false`, `/thesis/obstacle_input` tuvo cero publicadores y
el sistema informó `NOT_READY: proximidad ausente u obsoleta`. Abrir una sesión
armable en ese estado incumpliría el criterio `system_readiness=READY`. No se
alteró el supervisor ni se inventó una fuente de seguridad RGB-D.

## Correcciones

- `cartesian_kinematics.py`: rechazo fail-closed por manipulabilidad de
  Yoshikawa bajo el umbral crítico; la salida rechazada mantiene velocidades
  articulares nulas.
- `safety_supervisor_node.py`: comparación periódica de FK con TF exacta al
  sello de `/joint_states`; bloquea punto/JOG/ejecución si falta TF o el error
  excede `kinematic_tf_tolerance_m`. Publica
  `/thesis/kinematic_tf_status` para la GUI.
- `cartesian_jog_adapter.py` y `cartesian_jog_model.py`: ID ligado al arm
  epoch, rechazo de input pre-arm, zona muerta, limitación de aceleración,
  incremento IK acotado y cierre explícito del stream al entrar a hold.
- `joint_control_gui.py`: pestañas articular/cartesiana en hardware, exclusión
  mutua, requisito de arm epoch y FK–TF válido, unidades visibles y un único
  cero al vencer el watchdog. No retransmite ceros indefinidamente.
- `jaco_physical_system.launch.py` y `thesis_hardware/package.xml`: arranque de
  los tres adaptadores cartesianos mediante la arquitectura existente.
- Pruebas focales actualizadas para singularidad, TF inconsistente, arm epoch,
  seis ejes, signos/unidades, límites, zona muerta, aceleración y transición
  única a hold.

## Compilación y pruebas

Compilaron únicamente `thesis_core`, `thesis_simulation`, `thesis_ui` y
`thesis_hardware`. La repetición con resultados aislados terminó con:

```text
259 tests, 0 errors, 0 failures, 4 skipped
```

La prueba focal directa terminó con `59 passed`. Los XML están en
`evidence/codex_reports/2026-10-20_cartesian_close_test_results/`.

## Validación integrada

En mock, sin salida, se observaron los tres adaptadores, `READY`, FK–TF
`0.000000 m` frente a tolerancia `0.005000 m`, armado falso, hold y salida
0 Hz.

La captura física previa de 34.410 s registró 676 estados a 20 Hz, hueco máximo
51.103 ms, 676/676 TF reconstruibles al mismo sello y error FK–TF máximo
`1.422e-16 m`. No hubo intenciones ni comandos y el brazo permaneció
desarmado. Esto cierra A-02b para coherencia posicional estática real; falta
validarla durante un movimiento físico manual mínimo.

El preflight físico final, con salida deshabilitada y sin fuente demo, confirmó:

```text
backend=kinova_usb, connected=true, api_open=true
fault=false, state_valid=true, feedback=19.999693 Hz
armed=false, jog_active=false, mode=hold, output=0 Hz
FK–TF=VALID, error=0.000000 m
/thesis/obstacle_input publishers=0
proximity_valid=false, system_ready=false
```

## Limitaciones y siguiente paso

El control cartesiano está implementado y validado offline/mock y en reposo
con estado real, pero **no está validado mediante movimiento físico**. Para
abrir la sesión armable se necesita una fuente de obstáculo válida en
`/thesis/obstacle_input`. Las opciones seguras son autorizar explícitamente el
escenario de banco con `use_demo_obstacle=true`, o integrar y calibrar una
fuente real antes de conservar `use_demo_obstacle=false`.

Una vez recuperado `READY`, la primera prueba debe ser manual: revisar
diagnóstico y FK–TF, seleccionar un único eje lineal que aumente el despeje,
armar desde la GUI con el área libre y producir un solo pulso de hasta 5 mm,
sin orientación.
