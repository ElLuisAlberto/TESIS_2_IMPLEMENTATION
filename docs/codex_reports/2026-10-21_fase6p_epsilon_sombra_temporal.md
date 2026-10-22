# Fase 6p: epsilon geométrico explícito en sombra temporal

## Resultado único

`TEMPORARY_SHADOW_MISALIGNED`

## Contrato añadido

`jaco_d435i_shadow_session.launch.py` incorpora
`session_epsilon_total_m`, con default vacío. Si `enable_self_filter=true`,
el launch rechaza antes de crear el nodo un valor vacío, no numérico, NaN,
infinito o negativo. El valor se entrega únicamente como `epsilon_total_m` al
`jaco_self_filter_node` mediante `jaco_rgbd_shadow.launch.py`.

No se modificó el YAML seguro por defecto: conserva `epsilon_total_m: -1.0`
y el filtro desactivado. Tampoco se alteraron FPS, voxel, rango, TF candidata,
cápsulas, extractor, supervisor ni hardware.

## Pruebas sin hardware

- `test_shadow_session_contract.py`: 20 correctas; cubre aceptación de `0.05`
  y rechazo de vacío, `-1`, `NaN`, infinito y texto no numérico.
- `colcon build --packages-select thesis_jaco_perception`: correcto.
- `colcon test --packages-select thesis_jaco_perception`: 64 pruebas, 0
  errores, 0 fallos, 0 omitidas.
- `git diff --check`: correcto.

## Sesión temporal única

La única sesión se lanzó con JACO2 en solo lectura, cámara inmóvil,
`hardware_output=false`, YAML temporal existente y
`session_epsilon_total_m:=0.05`.

- El adaptador informó `salida_fisica_permitida=false, armado=false`.
- La TF temporal se publicó como `world -> d435i_link`.
- El preprocesador entregó nube en `world` y el filtro informó
  `tf_status=AVAILABLE`, configuración válida y edad de nube de 0.093 s.
- Diagnóstico de clasificación: 1316 puntos totales, 0
  `self_confirmed`, 1316 externos y 0 ocluidos.
- Muestras de tópicos: `points_self_confirmed` publicó ancho 0 y
  `points_external_candidate` ancho 1284. El diagnóstico del mismo ciclo
  registró 0 ocluidos; no se obtuvo una muestra serializada del tópico vacío
  `points_occluded_unknown` antes del cierre.

`ros2 topic hz` no acepta los indicadores obligatorios `--no-daemon`
`--spin-time 2`; por ello no se empleó sin esas salvaguardas. Se usaron lecturas
separadas `ros2 topic echo --once --no-daemon --spin-time 2` para las muestras.

RViz inició correctamente, pero esta sesión no produjo puntos confirmados del
brazo, por lo que no hay coincidencia visual del brazo que validar. La ausencia
de `self_confirmed` con clasificación y TF válidas indica que la pose temporal
no alinea el modelo de cápsulas con la nube observada. No se ajustó la
extrínseca ni se realizó reintento.
