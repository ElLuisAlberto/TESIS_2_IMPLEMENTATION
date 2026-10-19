# Corrección de fluidez TF en simulación

Fecha: 2026-10-18
Resultado: **VALIDADA EN SIMULACIÓN**

## Causa confirmada

`proximity_monitor` ejecutaba sus búsquedas TF exactas desde el mismo
ejecutor que recibía `/tf`. Al esperar una TF dentro del callback temporizado,
el búfer no podía incorporar una TF recién llegada. Un hueco transitorio se
convertía en `STOP/TF_UNAVAILABLE`, aun cuando la TF estaba disponible en el
siguiente ciclo.

## Corrección

El monitor conserva un `Buffer` propio, pero su `TransformListener` se crea
con nodo y ejecutor dedicados (`TransformListener(buffer, None,
spin_thread=True)`). Así la recepción de `/tf` progresa mientras el monitor
espera la TF del sello exacto.

No se cambió la política A-02a: se mantiene `world`, el sello de adquisición
del obstáculo, el timeout acotado de 40 ms y el `STOP/TF_UNAVAILABLE` ante una
TF realmente ausente. No se tocó hardware ni se activó salida física.

## Verificación

Se compiló `thesis_core` y se ejecutaron 28 pruebas: contratos de listener,
timeout TF, A-02a, frescura y obstáculo; todas aprobaron.

Con Gazebo, GUI articular y RViz activos se observaron dos ventanas pasivas de
15 s. La segunda registró 213 mensajes de proximidad: todos `ALLOW` y cero
`TF_UNAVAILABLE`; `/joint_states` aportó 1091 muestras. El supervisor se
recuperó gradualmente del estado transitorio de inicio hasta `ALLOW/CLEAR`,
sin nuevas paradas por TF.

## Alcance pendiente

La validación confirma la estabilidad de TF bajo la pila de presentación. Una
prueba manual prolongada de jog y los escenarios cartesianos X/Y/Z deben
repetirse por separado antes de sustituir la evidencia funcional previa.
