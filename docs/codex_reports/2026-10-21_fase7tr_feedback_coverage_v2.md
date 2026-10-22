# Fase 7T-R: feedback técnico de `shadow_coverage_v2`

## 1. DICTAMEN

`SHADOW_COVERAGE_V2_PROFILE_READY`

## 2. CAMBIOS REALES

Los cambios atribuibles a Fase 7T, verificados en el informe de esa fase y en
los artefactos inspeccionados, son exactamente:

- creado: `src/thesis_jaco_perception/config/jaco_self_filter_shadow_coverage_v2.yaml`;
- modificado: `src/thesis_jaco_perception/test/test_shadow_session_contract.py`;
- creado en esta auditoría: este informe.

Para Fase 7T: CMake **no**, `package.xml` **no**, self-filter **no**,
epsilon **no**, ROI **no**, TF **no**, extrínseca **no**, supervisor **no** y
hardware **no**. La evidencia es el alcance documentado en Fase 7T, el YAML
v2 que sólo contiene tres listas de cápsulas y la prueba que sólo añade su
contrato. `git status --short` sí contiene cambios preexistentes y no
relacionados (por ejemplo `thesis_perception/CMakeLists.txt` y
`thesis_perception/package.xml`); se preservaron y no se atribuyen a v2.

## 3. TABLA V1 VS V2

| Cápsula | Radio v1 | Radio v2 | Cambio | Motivo Fase 7S | Estado |
|---:|---:|---:|---:|---|---|
| 0 | 0.105 | 0.105 | 0.000 | Sin residual asociado. | Conservada |
| 1 | 0.105 | 0.105 | 0.000 | Sin residual asociado. | Conservada |
| 2 | 0.100 | 0.100 | 0.000 | P95 requerido 0.13128643739932974 m; incremento excesivo. | Bloqueada |
| 3 | 0.090 | 0.113 | +0.023 | P95 local 0.11342283441561084 m, 72 puntos. | Candidata local |
| 4 | 0.100 | 0.122 | +0.022 | P95 local 0.12218581675213991 m, 105 puntos. | Candidata local |
| 5 | 0.105 | 0.126 | +0.021 | P95 local 0.12591543346805348 m, 85 puntos. | Candidata local |

Los seis pares de frames `start/end` son idénticos entre v1 y v2.

## 4. TRAZABILIDAD NUMÉRICA

| Cápsula | P95 medido m | Radio v2 m | Diferencia v2 − P95 m | Redondeo |
|---:|---:|---:|---:|---|
| 3 | 0.11342283441561084 | 0.113 | -0.00042283441561084 | A 0.001 m |
| 4 | 0.12218581675213991 | 0.122 | -0.00018581675213991 | A 0.001 m |
| 5 | 0.12591543346805348 | 0.126 | +0.00008456653194652 | A 0.001 m |

Para la cápsula 2: P95 requerido `0.13128643739932974 m`; incremento
requerido `0.031286437399329736 m`; límite permitido `0.025 m`. No se cambió
porque excede el límite y Fase 7S la etiquetó
`EXCESSIVE_OR_AMBIGUOUS_EXPANSION`.

## 5. AISLAMIENTO Y SEGURIDAD

El YAML v2 declara literalmente `session_only=true`, `not_for_safety=true` y
`profile_kind=shadow_coverage`. El contrato probado valida la metadata y
extrae sólo el diccionario de las tres listas; no puede entregar v2 como un
segundo `--params-file`. El test verifica forwarding únicamente al self-filter
de sombra y ausencia de alcance hacia supervisor, bridge, extractor,
`obstacle_input` o control. El contrato de hardware conserva
`hardware_output=false`. La sesión coverage_v1 activa no fue tocada durante
esta auditoría.

## 6. VALIDACIÓN

Comando de build ejecutado en Fase 7T:

```text
colcon build --packages-select thesis_jaco_perception --symlink-install
```

Prueba ejecutada:

```text
colcon test --packages-select thesis_jaco_perception --ctest-args -R test_shadow_session_contract
```

Resultado del contrato: `40` pruebas, `0` errores, `0` fallos. Resultado
global de `colcon test-result --test-result-base build/thesis_jaco_perception
--verbose`: `90 tests, 0 errors, 0 failures, 0 skipped`. `git diff --check`
ejecutado en esta auditoría no produjo salida ni errores.

`git status --short` muestra como preexistentes modificaciones en `AGENTS.md`
y `thesis_perception/*`, junto con numerosos informes y directorios no
rastreado. Los artefactos creados en Fase 7T se distinguen por su contenido y
por el informe de fase: `jaco_self_filter_shadow_coverage_v2.yaml` y la
extensión de `test_shadow_session_contract.py`; no se alteraron los cambios
preexistentes.

## 7. LÍMITES TÉCNICOS

v2 busca reducir rojo de antebrazo distal, muñeca y efector mediante cápsulas
3–5. Puede permanecer residual en la carcasa/antebrazo asociado a cápsula 2.
La cápsula 2 sigue bloqueada porque exige +0.031286437399329736 m en P95,
superior al límite de +0.025 m. v2 no es calibración final: depende de una
sola sesión temporal, no sustituye soporte fijo ni calibración con marcador y
no es un perfil de seguridad.

## 8. SIGUIENTE PRUEBA EXACTA

1. Cerrar coverage_v1 sólo cuando el operador lo ordene expresamente.
2. Iniciar una única sesión v2 con el mismo TF temporal v2, ROI temporal y epsilon 0.07 m.
3. Mantener el brazo inmóvil, JACO2 sólo lectura y `hardware_output=false`.
4. Sin caja inicialmente, comparar en RViz verde, rojo y amarillo contra la captura v1.
5. No ajustar radios ni parámetros runtime; conservar la sesión para captura del operador.
