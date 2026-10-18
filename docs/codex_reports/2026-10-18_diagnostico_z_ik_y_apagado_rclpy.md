# Diagnóstico Z (+20 mm) y apagado de `rclpy`

Fecha: 2026-10-18.

## Alcance

Se revisaron los últimos resultados Y/Z, la cinemática, el ciclo de vida de
`system_readiness` y sus pruebas. No se modificó código, no se inició ROS ni
Gazebo y no se publicaron comandos. Las comprobaciones adicionales fueron
cálculos Python puros sobre el estado articular registrado.

## Rechazo IK de Z

### Causa confirmada

- La pila Z estaba prácticamente en la configuración declarada de Gazebo
  `(0, π, π, 0, 0, 0)`, con el efector en `z ≈ 1.18108 m`. Es una
  extensión vertical singular: para el estado articular capturado, la norma de
  la fila Z del Jacobiano numérico es solo `5.17e-6 m/rad`.
- Al repetir en cálculo puro el mismo objetivo relativo `+0.020 m`, el solver
  DLS devuelve exactamente la semilla, `success=False` y residuo `0.020000 m`.
  No encuentra un descenso que acepte la búsqueda lineal, por lo que el
  rechazo observado de `19.9 mm` es coherente con el algoritmo y no con la
  tolerancia de `0.5 mm`.
- El objetivo nominal tiene `z=1.201 m`. Una búsqueda acotada dejando incluso
  libre la orientación encontró `z_max ≈ 1.186582 m` dentro de los límites
  articulares: faltan al menos unos `14.4 mm` hasta el objetivo. Aumentar la
  tolerancia no es una corrección admisible.
- Y no contradice este resultado: Y pudo aprovechar pequeñas perturbaciones de
  la simetría y cerró con `0.497 mm`; Z pide continuar elevando un brazo casi
  totalmente extendido y conservando la orientación.

El solver es explícitamente local: forma el Jacobiano y el paso DLS y abandona
si ninguno de ocho factores reduce el coste. Además, el texto de salida indica
correctamente que el objetivo no es alcanzable desde la solución local.

### Hipótesis pendiente

La búsqueda de `z_max` fue numérica y multisemilla, no una demostración
analítica del espacio de trabajo completo. No obstante, no es necesaria para
explicar el rechazo concreto: la singularidad, el Jacobiano y la salida sin
cambio del solver se reproducen exactamente con el estado capturado.

### Prueba mínima propuesta

Agregar dos regresiones unitarias a `test_cartesian_kinematics.py`:

1. Desde el estado Z capturado, `+20 mm` en Z y orientación constante debe ser
   rechazado con residuo muy superior a `0.5 mm`. Esto conserva el rechazo de
   objetivos no alcanzables.
2. Desde la postura no singular ya usada por el núcleo
   `(0°, 150°, 210°, 0°, 20°, 0°)`, el mismo `+20 mm` debe converger. El
   cálculo puro actual converge en 3 iteraciones, con `0.147 mm` de residuo
   lineal y `0.087°` angular.

Una futura prueba integrada de Z debe arrancar en una postura validada con
margen vertical (preferiblemente mediante una configuración inicial exclusiva
de prueba), comprobar TF estable y recién entonces emitir el único objetivo.
No debe reinterpretar el rechazo desde la postura extendida como fallo del
solver.

### Parche mínimo propuesto y criterio de autorización

No cambiar tolerancias ni añadir multistart a la IK. El cambio mínimo es de
precondición/fixture: usar para la prueba Z una postura inicial no singular con
margen demostrado, y mantener una regresión que exija el rechazo desde la
postura extendida. Autorizarlo solo si las dos pruebas anteriores pasan, no se
debilitan los rechazos inalcanzables, y una pila headless nueva confirma
READY, TF, supervisor y ejecución sin usar hardware.

## Excepción de `rclpy` al apagar

### Causa confirmada

- Ocurrió después de READY y del rechazo IK, al enviar SIGINT al launch. Por
  orden temporal no causó el fallo Z.
- La traza sale de `rclpy.executors._take_subscription()`/
  `take_message()` como `RuntimeError: Unable to convert call argument to
  Python object` mientras se desmonta la pila.
- `system_readiness_node.main()` solo captura `KeyboardInterrupt`; cualquier
  `RuntimeError` del executor escapa. En cambio, el helper existente
  `thesis_core.ros_runtime.spin_node()` captura excepciones de cierre y solo
  suprime una excepción genérica cuando `rclpy.ok()` ya es falso. Si el contexto
  sigue activo, la vuelve a lanzar. `thesis_simulation` ya declara dependencia
  de `thesis_core`.

### Hipótesis pendiente

El mensaje de pybind es compatible con una carrera de apagado: SIGINT invalida
el contexto mientras el executor intenta extraer una suscripción. Para
confirmar el punto exacto dentro de `rclpy` haría falta instrumentar o repetir
el cierre; no hay evidencia de mensaje mal tipado durante operación normal.

### Prueba mínima propuesta

Extender `test_ros_runtime.py` con una prueba donde `rclpy.spin()` lanza ese
`RuntimeError` y `rclpy.ok()` devuelve falso: debe destruir el nodo sin
propagarlo. Conservar la prueba existente donde el mismo `RuntimeError`, con
`rclpy.ok()` verdadero, sí se propaga. Después, ejecutar una pila headless
pasiva y cerrarla tras READY, sin publicar intenciones.

### Parche mínimo propuesto y criterio de autorización

Reemplazar exclusivamente el bloque manual de `main()` por el helper compartido
`spin_node(SystemReadinessNode())`. Autorizar si ambas ramas unitarias
(contexto invalidado y contexto vivo) pasan y el cierre pasivo termina sin
traceback ni procesos residuales. Este filtro no oculta fallos reales mientras
ROS permanece activo.

