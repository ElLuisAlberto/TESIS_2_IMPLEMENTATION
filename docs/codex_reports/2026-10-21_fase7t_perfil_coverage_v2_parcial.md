# Fase 7T: perfil temporal parcial `shadow_coverage_v2`

## Dictamen

`SHADOW_COVERAGE_V2_PROFILE_READY`

## Perfil creado y alcance

Se creó exclusivamente
`src/thesis_jaco_perception/config/jaco_self_filter_shadow_coverage_v2.yaml`
y se extendió exclusivamente
`src/thesis_jaco_perception/test/test_shadow_session_contract.py`.

El YAML v2 conserva la metadata literal requerida:

```yaml
shadow_coverage_metadata:
  session_only: true
  not_for_safety: true
  profile_kind: shadow_coverage
```

Sus comentarios lo declaran experimental, temporal, no apto para seguridad,
derivado de Fase 7S y no sustitutivo de soporte fijo más calibración con
marcador. También registran que la cápsula 2 se conserva deliberadamente por
expansión ambigua/excesiva.

No se activó ni cargó v2. No se modificaron v1, el perfil histórico, epsilon,
ROI, TF, extrínseca, self-filter, supervisor, hardware bridge, extractor,
obstacle bridge, CMake, `package.xml` ni `thesis_core`.

## Evidencia y radios aprobados

Fase 7S-R aprobó radios P95 locales para cápsulas 3–5 y bloqueó la cápsula 2,
cuyo incremento P95 fue `0.031286437399329736 m`, superior al límite de
`0.025 m`. Los valores de v2 redondean sólo esos P95 a 0.001 m y no reducen
ningún radio.

| Cápsula | Segmento | Radio v1 m | Radio v2 m | Cambio | Evidencia Fase 7S-R |
|---|---|---:|---:|---:|---|
| 0 | `base_to_shoulder` | 0.105 | 0.105 | 0.000 | Sin residual; conservar. |
| 1 | `shoulder_to_upper_arm` | 0.105 | 0.105 | 0.000 | Sin residual; conservar. |
| 2 | `upper_arm_to_forearm` | 0.100 | 0.100 | 0.000 | Bloqueada; P95 requerido 0.13128643739932974 m. |
| 3 | `forearm_to_wrist_1` | 0.090 | 0.113 | 0.023 | P95 requerido 0.11342283441561084 m. |
| 4 | `wrist_1_to_wrist_2` | 0.100 | 0.122 | 0.022 | P95 requerido 0.12218581675213991 m. |
| 5 | `wrist_2_to_tool` | 0.105 | 0.126 | 0.021 | P95 requerido 0.12591543346805348 m. |

El archivo conserva exactamente los mismos seis pares `capsule_start_frames` /
`capsule_end_frames` de v1 y sólo contiene las tres listas permitidas:
`capsule_start_frames`, `capsule_end_frames` y `capsule_inner_radii_m`.

## Contrato y pruebas estáticas

La prueba extendida verifica que v2 tiene metadata exacta, seis cápsulas,
frames idénticos a v1, radios exactos, cápsulas 0–2 sin cambio y cápsulas 3–5
con los valores aprobados. También verifica que v2 pasa el validador estricto,
que no tiene claves adicionales y que el launch extrae su diccionario de tres
listas mediante `_validate_shadow_coverage_profile`; nunca agrega la ruta de
v2 como segundo `--params-file`. El argumento vacío conserva el perfil
histórico, cubierto por la prueba existente.

Se ejecutaron únicamente:

```text
colcon build --packages-select thesis_jaco_perception --symlink-install
colcon test --packages-select thesis_jaco_perception --ctest-args -R test_shadow_session_contract
colcon test-result --test-result-base build/thesis_jaco_perception --verbose
git diff --check
```

Resultados literales:

- Build: `1 package finished`.
- `test_shadow_session_contract`: `40` pruebas, `0` errores, `0` fallos.
- `colcon test-result --verbose`: `90 tests, 0 errors, 0 failures, 0 skipped`.
- `git diff --check`: sin salida ni errores.

No se inició launch, ROS, D435i, JACO2, RViz, TF ni Gazebo durante esta fase.
La sesión coverage_v1 activa no se cerró, reinició ni modificó; no hubo
movimiento, comandos, trayectorias, jog, acciones o servicios físicos.
