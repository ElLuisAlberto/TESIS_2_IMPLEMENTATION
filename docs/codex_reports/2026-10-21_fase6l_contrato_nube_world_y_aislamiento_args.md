# Fase 6l — contrato de nube en world y aislamiento de argumentos

Fecha: 2026-10-21
Decisión: `READY_FOR_WORLD_SHADOW_RETRY`

## Causa raíz

La sesión incluía `thesis_perception/d435i_standalone.launch.py`, que compone en un mismo contexto de launch la cámara RealSense y el preprocesador. El `rs_launch.py` de `realsense2_camera` recorre todas las configuraciones activas para informar parámetros no soportados. Por ello veía también argumentos internos de la sesión sombra, aunque no fuesen parámetros de la cámara.

Además, el launcher independiente conserva `target_frame` vacío por defecto y el preprocesador usa `use_latest_transform=true` por defecto. La sesión no fijaba esos dos valores, de modo que la nube filtrada conservaba `d435i_depth_optical_frame` en lugar de transformarse a `world` al sello original.

## Corrección aplicada

- Se añadió `launch/d435i_shadow_camera_pipeline.launch.py`, una composición exclusiva de la sesión.
- RealSense se ejecuta dentro de un `GroupAction(scoped=True, forwarding=False)` que contiene solo parámetros admitidos por RealSense; los argumentos de sesión no llegan a su contexto.
- El preprocesador se ejecuta en otro ámbito sellado con:
  - `target_frame=world`;
  - `use_latest_transform=false`, por lo que exige la TF correspondiente al sello de la nube;
  - `input_topic=/camera/d435i/depth/color/points`;
  - `filtered_topic=/thesis/perception/points_filtered`;
  - voxel `0.01 m` y rango máximo `1.5 m` preservados.
- `jaco_d435i_shadow_session.launch.py` usa esa composición local y conserva en el self-filter `input_topic=/thesis/perception/points_filtered`, `expected_frame=world` y `camera_optical_frame=d435i_depth_optical_frame`.
- No se modificó `d435i_standalone.launch.py` ni su comportamiento independiente.

## Contrato final

| Productor/consumidor | Tópico | Frame/condición |
| --- | --- | --- |
| D435i | `/camera/d435i/depth/color/points` | `d435i_depth_optical_frame` |
| Preprocesador | `/thesis/perception/points_filtered` | `world`; TF exacta al sello, sin fallback a la más reciente |
| Self-filter | entrada anterior | espera `world` |
| Self-filter | `points_self_confirmed`, `points_external_candidate`, `points_occluded_unknown` | salidas de clasificación sombra |

No se incluyen extractor, `obstacle_input` ni supervisor. Los defaults de la sesión permanecen desactivados.

## Validación sin hardware

- `python3 -m pytest src/thesis_jaco_perception/test/test_shadow_session_contract.py -q`: 9 aprobadas.
- `colcon build --packages-select thesis_jaco_perception`: correcto.
- `colcon test --packages-select thesis_jaco_perception`: 53 pruebas, 0 errores, 0 fallos, 0 omitidas.
- `git diff --check`: correcto.

No se inició JACO2, D435i, RViz, TF, Gazebo ni ninguna sesión física.
