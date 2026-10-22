# Fase 6C — Auditoría de sombra sin cambios

Fecha: 2026-10-21. Alcance: auditoría de solo lectura. No se iniciaron nodos,
no se publicaron TF ni mensajes de sombra, y no se editó código, configuración,
launch, CMake, pruebas ni scripts.

## Capacidades auditadas

- `jaco_self_filter_node` y su lógica geométrica existen en
  `thesis_jaco_perception` como componentes de filtrado en sombra.
- El paquete contiene configuración de sombra JACO–D435i y depende de
  `sensor_msgs`, `geometry_msgs`, `diagnostic_msgs`, `rclcpp` y `tf2_ros`.
- La sesión RGB-D actual entrega la nube en el marco de profundidad de la
  D435i, pero no existe una extrínseca `world -> d435i_depth_optical_frame`
  validada y persistida.
- El modelo del JACO2 contiene marcos y cápsulas que el filtro de sombra debe
  consumir junto con TF y `joint_states` reales.

## Brecha que impide una sesión limpia con componentes existentes

La prueba solicitada requiere demostrar, antes de arrancar, una cadena completa
y sin supuestos: launch físico exclusivamente de lectura con
`hardware_output=false`, `joint_states` reales, TF del robot, TF temporal de
sesión, nube transformada y exactamente los tres tópicos de sombra, sin que el
launch conecte extractor, `/thesis/obstacle_input` o supervisor.

Con la extrínseca todavía hipotética y sin una auditoría verificable de ese
contrato de lanzamiento completo, no es seguro inferir nombres de argumentos,
marcos, cápsulas ni tópicos. El principio de esta fase prohíbe inventarlos o
añadir adaptadores. Por ello no se inició el JACO2 en lectura ni se lanzó la
sesión temporal.

## Acción necesaria antes de reintentar

Proporcionar o dejar verificable en el repositorio un contrato existente que
enumere explícitamente: launch de lectura del JACO2, argumento
`hardware_output=false`, tópico real de `joint_states`, marcos de base/cámara,
las tres salidas de sombra y las garantías de aislamiento respecto de extractor
y supervisor. Con ese contrato se podrá iniciar una sola sesión efímera y
publicar el TF temporal autorizado, sin persistirlo.

## Resultado

**BLOCKED_ARCHITECTURAL_GAP.** No se continuó a integración, no se creó código
de soporte y no hubo hardware, salida física, TF activo ni conexión al
supervisor.
