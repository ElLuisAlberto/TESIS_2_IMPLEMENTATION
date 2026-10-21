# Cierre funcional del JACO2: Joints y Cartesiano

Fecha: 2026-10-20

## Resultado

El stack físico quedó abierto con la GUI y RViz del JACO2, salida física
permitida pero desarmada. La validación controlada y su rosbag se ejecutaron
sin armar ni publicar movimiento.

La ruta conservada para JOG articular y cartesiano es:

`GUI -> intención candidata -> horizonte de 1 s -> supervisor preventivo ->`
`comando supervisado -> jaco_hardware_adapter -> Kinova`.

No existe un bypass desde la GUI al SDK. Las pestañas `Joints` y `Cartesiano`
se instancian también en modo hardware; el control cartesiano expone X/Y/Z en
m/s y Roll/Pitch/Yaw en grados/s, con zona muerta, límites, suavizado,
watchdog, arm epoch, exclusión mutua y rechazo fail-closed por estado, TF, IK,
límites o singularidad.

## Corrección de sincronización FK-TF

Una primera captura real encontró siete estados transitorios
`KINEMATIC_TF_UNAVAILABLE`: el supervisor comprobaba la muestra articular más
nueva antes de que `robot_state_publisher` hubiera distribuido su TF con el
mismo sello. No era una inconsistencia geométrica.

La corrección mínima conserva un historial acotado de diez estados y selecciona
la muestra fresca más nueva que ya tenga TF con sello exactamente igual. No
interpola, no espera, no amplía timeouts y sigue fallando de forma segura si no
existe una pareja fresca. La prueba focal reproduce que el TF nuevo está en
tránsito y exige usar la pareja exacta anterior.

## Obstáculo virtual calibrado

Se reutiliza el único `obstacle_demo_source` y su bridge. La referencia medida
del JACO2 fue:

```text
[2.2334760271614935, 2.483514895586853, 1.0155264405650484,
 5.740923098662012, 5.969486236056973, 2.079310957978987] rad
```

Esfera: radio `0.10 m`, incertidumbre `0.02 m`, radio efectivo `0.12 m`, frame
`world`.

| Caso | Centro world (m) | Resultado real sin movimiento |
|---|---|---|
| PARK | `(1.22, 0.00, 0.60)` | ALLOW, despeje `0.808842 m` |
| BLOCK_START | `(0.017716, 0.246297, 0.542326)` | STOP, despeje positivo `0.014999 m` |
| INSERT_STOP | `(0.234429, -0.129626, 0.677326)` | STOP al insertar, despeje positivo `0.021786 m` |

Para PARK, la mínima proyectada de J1 +3 grados es `0.800874 m` y para J1
-3 grados es `0.808842 m`; ambas permanecen ALLOW. La prueba focal adicional
verifica WARNING estático, REDUCTION por intrusión predictiva y las pruebas de
política existentes verifican retirada protectora e histéresis gradual.

Este obstáculo es una entrada virtual determinista para comprobar el
supervisor. No representa una detección RGB-D ni acredita percepción del
entorno real.

## Compilación y pruebas

Solo se compilaron `thesis_core`, `thesis_simulation`, `thesis_hardware` y
`thesis_ui`. Resultado conjunto:

```text
thesis_core:       175 passed, 1 skipped
thesis_ui:          27 passed, 1 skipped
thesis_simulation:  43 passed, 1 skipped
thesis_hardware:    12 passed, 1 skipped
Total:             257 passed, 4 skipped, 0 failed
```

Mock, dominio 79: tres adaptadores cartesianos activos, READY, armado falso,
hold y FK-TF `0.000000 m` frente a tolerancia `0.005000 m`.

## Validación física estática

Bag:
`evidence/rosbags/20261020_1835_jaco_cartesian_virtual_park_readonly_postfix/`

- duración de bag: `35.762975 s`;
- ventana articular: `34.737497 s`, 696 muestras, `20.0072 Hz`;
- hueco máximo de `/joint_states`: `51.902879 ms`;
- rango de los seis joints y dedos: `0.0 grados`;
- velocidad máxima informada: `0.0 rad/s`;
- FK-TF: `357/357 VALID`, error `0.000000 m`, ningún INVALID;
- proximidad: `716/716 ALLOW`, `0.808842384 m`, frame `world`;
- readiness: `71/71 READY`;
- armado: `71/71 false`;
- backend: conectado, API abierta, estado válido, `fault=false`;
- diagnóstico: `jog_active=false`, `mode=hold`, entrada y salida `0 Hz`;
- cero JOG, candidatos, decisiones, comandos supervisados y ejecuciones.

BLOCK_START e INSERT_STOP se comprobaron cambiando solo la pose de la esfera
con salida deshabilitada y robot desarmado. Ambos produjeron STOP antes de
contacto; al volver a PARK se recuperaron ALLOW y READY.

## Estado final abierto

Launch físico en `ROS_DOMAIN_ID=78` con:

```text
mock_hardware=false
hardware_output_enabled=true
use_demo_obstacle=true
demo_obstacle_scenario=PARK
start_gui=true
start_rviz=true
allow_gui_arm_control=true
```

Comprobación final:

```text
GUI y RViz activos
USB Kinova conectado, connected=true, api_open=true
output_permitted=true, fault=false, state_valid=true
system_readiness=READY
FK-TF=VALID, error=0.000000 m
proximidad=ALLOW, clearance=0.808842384 m
armed=false, mode=hold, jog_active=false
active_goal=false, output=0 Hz, JOG input=0 Hz
```

La validación de movimiento cartesiano real sigue siendo manual: el operador
debe armar desde la GUI y comenzar con un único pulso muy pequeño, zona
despejada y parada física accesible.

Después de abrir la GUI final se detectó una interacción externa: el estado
pasó a armado/JOG. El adaptador aplicó sus interbloqueos y watchdog; para
cumplir el estado de entrega se solicitó desarmado por el servicio normal del
adaptador, sin enviar movimiento. La última lectura confirmó nuevamente
`armed=false`, `jog_active=false`, `mode=hold` y salida `0.000000 Hz`.

## Dictamen

`LISTO_PARA_PRUEBAS_MANUALES_JOINTS_Y_CARTESIANO=SI`
