# Fase 7H: contrato seguro de ROI en sombra

## Dictamen

`ROI_SHADOW_CONTRACT_READY`

## Causa y cambio aplicado

La sesión de sombra no exponía el ROI y el sublaunch de cámara fijaba el crop
en `false` con bounds internos. Por ello no era posible solicitar un ROI de
`world` desde la sesión sin editar el launch.

Se modificaron exclusivamente:

- `src/thesis_jaco_perception/launch/jaco_d435i_shadow_session.launch.py`
- `src/thesis_jaco_perception/launch/d435i_shadow_camera_pipeline.launch.py`
- `src/thesis_jaco_perception/test/test_shadow_session_contract.py`

Ambos launches declaran `enable_crop`, `min_x`, `max_x`, `min_y`, `max_y`,
`min_z` y `max_z`. La sesión los reenvía explícitamente al pipeline de cámara;
el pipeline los mantiene dentro de su `GroupAction(scoped=True,
forwarding=False)` de procesamiento y los entrega sólo al launch del
preprocesador. El ámbito RealSense queda sellado y no recibe estos argumentos.

## Defaults preservados

| Argumento | Default |
|---|---:|
| `enable_crop` | `false` |
| `min_x`, `max_x` | `-1.5`, `1.5` |
| `min_y`, `max_y` | `-1.5`, `1.5` |
| `min_z`, `max_z` | `-1.0`, `2.5` |

Por tanto el comportamiento por defecto sigue con crop desactivado y bounds
históricos. No se modificaron self-filter, supervisor, hardware bridge,
cápsulas, extrínseca, YAML final, nodo preprocesador ni pipeline estándar. El
rechazo existente de `hardware_output=true` se conserva.

## Validación previa a procesos

Los dos launches validan los seis límites antes de crear sus acciones hijas:

- todos deben ser numéricos y finitos;
- si `enable_crop=true`, se exige `min_x < max_x`, `min_y < max_y` y
  `min_z < max_z`;
- cualquier valor no finito o intervalo invertido aborta el launch antes de
  iniciar cámara, hardware, TF o procesamiento.

## Verificación

- `colcon build --packages-select thesis_jaco_perception --symlink-install`:
  correcto.
- `test_shadow_session_contract`: **31 pasadas, 0 fallidas**. Cubre defaults
  históricos, forwarding sesión→pipeline→preprocesador, aislamiento de
  RealSense, rechazo de bounds invertidos/no finitos y
  `hardware_output=true`.
- `ros2 launch thesis_jaco_perception jaco_d435i_shadow_session.launch.py
  --show-args`: muestra los siete argumentos nuevos y sus defaults.
- `ros2 launch thesis_jaco_perception d435i_shadow_camera_pipeline.launch.py
  --show-args`: muestra los mismos siete argumentos y defaults.
- `git diff --check`: correcto.

No se inició ningún launch, D435i, JACO2, RViz, TF ni Gazebo. La sesión nativa
ya activa no fue cerrada, reiniciada ni modificada; no hubo comandos físicos,
servicios de movimiento ni cambios de `hardware_output`.
