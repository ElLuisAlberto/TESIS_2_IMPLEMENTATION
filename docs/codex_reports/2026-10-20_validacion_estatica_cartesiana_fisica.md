# Validación estática del control cartesiano físico

Fecha: 2026-10-20  
Resultado: **COMPUERTA FK–TF APROBADA, MOVIMIENTO NO AUTORIZADO**

Se cerró ordenadamente la sesión física anterior, se verificó libre el bloqueo
USB y se inició una única sesión en `ROS_DOMAIN_ID=78` con backend Kinova real,
`hardware_output_enabled=false`, GUI habilitada y armado desactivado.

Durante 34.41 s se registraron 676 muestras de `/joint_states` a 20.000 Hz. El
hueco máximo fue 51.103 ms. Las 676 configuraciones tuvieron TF reconstruible
al mismo sello; error FK–TF mediano, p95 y máximo: `1.42e-16 m`, muy inferior a
la tolerancia de 0.005 m.

Los tres adaptadores cartesianos, supervisor, bridge, readiness y GUI estuvieron
activos. No aparecieron `TF_UNAVAILABLE`, estados obsoletos ni STOP. El
diagnóstico permaneció `fault=false`, `state_valid=true`, `armed=false`,
`mode=hold`, entrada JOG 0 Hz y salida física 0 Hz.

No hubo mensajes en intención cartesiana, intención JOG, candidato, comando
supervisado ni control de ejecución. Las seis articulaciones tuvieron rango y
velocidad cero.

Observación: el obstáculo PARK actual quedó en `WARNING` estable, con despeje
0.183257 m respecto a `wrist_2_to_tool`. No es un fallo FK–TF ni produjo STOP,
pero debe recuperarse ALLOW estable antes de cualquier prueba de movimiento.

No se armó ni movió el JACO2. La sesión queda abierta en modo de observación,
sin posibilidad de armar desde la GUI.

