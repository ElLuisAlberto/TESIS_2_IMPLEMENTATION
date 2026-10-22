# Fase 6n: extrínseca temporal hacia la raíz D435i

## Resultado

`READY_FOR_CONNECTED_WORLD_SHADOW_RETRY`

No se inició hardware, ROS, cámara, JACO2, RViz ni TF durante esta fase.

## Raíz y cadena TF

La inspección estática del launch de cámara usado por la sombra identifica la
raíz RealSense como `d435i_link`:

- El proyecto configura `camera_name: d435i`.
- `rs_launch.py` de RealSense define `base_frame_id: link` como raíz del árbol
  de sensores.
- El launch de sombra ahora fija explícitamente `base_frame_id: link` y
  `tf_prefix: ""`, cuya composición del driver es `d435i_link`.

Cadena interna esperada, publicada por el driver y no por la extrínseca:

```text
world -> d435i_link -> d435i_depth_frame -> d435i_depth_optical_frame
```

La nube conserva `d435i_depth_optical_frame` como frame óptico; el
preprocesador puede transformarla a `world` atravesando esa cadena.

## Cambios

- `src/thesis_jaco_perception/launch/jaco_d435i_shadow_session.launch.py`
  valida y publica solo `world -> d435i_link`; rechaza todo `child_frame` que
  termine en `_optical_frame`, y exige el frame óptico separado.
- `src/thesis_jaco_perception/launch/d435i_shadow_camera_pipeline.launch.py`
  hace explícitos `base_frame_id=link` y `tf_prefix=""` para fijar la raíz
  configurada de RealSense, sin cambiar perfiles, FPS ni filtros.
- `evidence/codex_reports/2026-10-21_d435i_extrinsic_session_candidate.yaml`
  sigue siendo temporal/no final y ahora usa `child_frame: d435i_link` con
  `camera_optical_frame: d435i_depth_optical_frame`.
- `src/thesis_jaco_perception/test/test_shadow_session_contract.py` cubre el
  rechazo de children ópticos, la única unión externa hacia la raíz, la cadena
  interna declarada y la ausencia de extractor, supervisor y obstacle input.

## Validación sin hardware

- `colcon build --packages-select thesis_jaco_perception`: correcto.
- `colcon test --packages-select thesis_jaco_perception`: 58 pruebas, 0
  errores, 0 fallos, 0 omitidas.
- `test_shadow_session_contract.py`: 14 pruebas correctas, incluido el YAML
  temporal real.
- `git diff --check`: correcto.

El contrato tiene una única arista externa (`world -> d435i_link`); no añade
padres a frames internos de RealSense y por ello no define árboles TF
desconectados. No se realizó reintento de sombra en esta fase.
