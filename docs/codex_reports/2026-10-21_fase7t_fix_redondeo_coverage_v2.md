# Fase 7T-FIX: corrección conservadora de redondeo `coverage_v2`

## Dictamen

`SHADOW_COVERAGE_V2_PROFILE_CORRECTED`

## Corrección aplicada

Se modificaron exclusivamente:

- `src/thesis_jaco_perception/config/jaco_self_filter_shadow_coverage_v2.yaml`
- `src/thesis_jaco_perception/test/test_shadow_session_contract.py`

El vector v2 pasó de `[0.105, 0.105, 0.100, 0.113, 0.122, 0.126]` a
`[0.105, 0.105, 0.100, 0.114, 0.123, 0.126]` m. Los radios 0, 1 y 2 no
cambiaron; la cápsula 2 permanece exactamente en `0.100 m`.

| Cápsula | P95 medido m | Radio antes m | Radio final m | Margen final m |
|---:|---:|---:|---:|---:|
| 3 | 0.11342283441561084 | 0.113 | 0.114 | 0.00057716558438916 |
| 4 | 0.12218581675213991 | 0.122 | 0.123 | 0.00081418324786009 |
| 5 | 0.12591543346805348 | 0.126 | 0.126 | 0.00008456653194652 |

Cada radio final cumple `radio_v2 >= P95` y `radio_v2 - P95 < 0.001 m`.
Las pruebas añaden esas dos aserciones para las cápsulas 3–5, y conservan las
verificaciones de metadata, frames, aislamiento, radios 0–2 y extracción sin
segundo `--params-file`.

## Validación

```text
colcon build --packages-select thesis_jaco_perception --symlink-install
colcon test --packages-select thesis_jaco_perception --ctest-args -R test_shadow_session_contract
colcon test-result --test-result-base build/thesis_jaco_perception --verbose
git diff --check
```

Resultados: build `1 package finished`; resumen de pruebas `90 tests, 0
errors, 0 failures, 0 skipped`; `git diff --check` sin salida ni errores.

No se inició ROS, launch, cámara, JACO2, RViz, TF, Gazebo ni sonda. La sesión
coverage_v1 activa no fue cerrada, reiniciada ni modificada; no hubo comandos,
trayectorias, jog, acciones ni servicios físicos. coverage_v2 no fue iniciado
ni cargado.
