# Fase 7S-R: revisión técnica de propuesta `coverage_v2`

## Dictamen

`COVERAGE_V2_PARTIAL_ONLY`

## Evidencia revisada

Fuente única: `evidence/codex_reports/2026-10-21_fase7s_medicion_residual_coverage_v1.md`.
La medición usó explícitamente los radios v1
`[0.105, 0.105, 0.100, 0.090, 0.100, 0.105]` m y tres nubes con sellos
`1792628226.432065918`, `1792628226.498781250` y
`1792628226.532139160`. Informó `points_valid=365`, `external=365`,
`self=0`, `occluded=0`, `no_intersection=365`, `invalid=0`,
`discarded_tf=0` y `discarded_invalid_cloud=0`.

La fórmula registrada fue
`required_radius_m = distance_to_finite_segment_m + 0.003`; los incrementos
revisados son `P95(required_radius_m) - radio_v1` y
`P99(required_radius_m) - radio_v1`. Los frames extremos concretos no quedaron
registrados en el informe 7S, por lo que no se inventan en esta revisión.

## Evaluación por cápsula

| Cápsula | Segmento/frame | Radio v1 | Puntos | P50 | P90 | P95 | P99 | Máximo | Incremento P95 | Incremento P99 | Etiqueta de Fase 7S | Recomendación |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 0 | `base_to_shoulder`; frames no registrados en 7S | 0.105 | 0 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | `NO_RESIDUAL` | Conservar 0.105 m; no existe evidencia para cambio. |
| 1 | `shoulder_to_upper_arm`; frames no registrados en 7S | 0.105 | 0 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | `NO_RESIDUAL` | Conservar 0.105 m; no existe evidencia para cambio. |
| 2 | `upper_arm_to_forearm`; frames no registrados en 7S | 0.10000000000000001 | 103 | 0.1182404435585387 | 0.12967944900221676 | 0.13128643739932974 | 0.13216047003626613 | 0.13252507336605851 | 0.031286437399329736 | 0.032160470036266126 | `EXCESSIVE_OR_AMBIGUOUS_EXPANSION` | `BLOQUEADO`: incremento P95 > 0.025 m; no proponer v2 automática. |
| 3 | `forearm_to_wrist_1`; frames no registrados en 7S | 0.089999999999999997 | 72 | 0.10098267448204741 | 0.11026314871239322 | 0.11342283441561084 | 0.1145264918126311 | 0.11452649214143613 | 0.023422834415610841 | 0.024526491812631099 | `LOCAL_COVERAGE_CANDIDATE` | Proponer `0.113 m` (P95 redondeado sólo a 0.001 m). |
| 4 | `wrist_1_to_wrist_2`; frames no registrados en 7S | 0.10000000000000001 | 105 | 0.11117308749760933 | 0.11998168904405469 | 0.12218581675213991 | 0.12375411521146593 | 0.12375411521146593 | 0.022185816752139903 | 0.02375411521146592 | `LOCAL_COVERAGE_CANDIDATE` | Proponer `0.122 m` (P95 redondeado sólo a 0.001 m). |
| 5 | `wrist_2_to_tool`; frames no registrados en 7S | 0.105 | 85 | 0.11782837170678802 | 0.1254721879602462 | 0.12591543346805348 | 0.12789819607381778 | 0.12898382195908234 | 0.020915433468053482 | 0.022898196073817781 | `LOCAL_COVERAGE_CANDIDATE` | Proponer `0.126 m` (P95 redondeado sólo a 0.001 m). |

Ninguna cápsula candidata presenta `P99-P95 > 0.020 m`; por tanto no se
marca `DISPERSIÓN_ALTA`. No se usa P99 ni máximo como radio automático y no
se propone ninguna reducción.

## Proporción y recomendación global

Las candidatas 3, 4 y 5 reúnen `72 + 105 + 85 = 262` de los `365` puntos
residuales: `262 / 365 = 0.7178082191780822` (71.78082191780822 %). La cápsula
2 conserva 103 puntos bloqueados por su incremento P95 de
`0.031286437399329736 m`. Por ello existen propuestas locales verificables,
pero no una justificación para cobertura v2 completa.

Radios propuestos sólo para revisión de un eventual perfil temporal:

- cápsula 3: `0.113 m`;
- cápsula 4: `0.122 m`;
- cápsula 5: `0.126 m`.

Para los 103 puntos bloqueados hace falta evidencia adicional que separe
carcasa/forma no cilíndrica, error de segmento o efecto de oclusión; no basta
con expandir la cápsula 2. No se propone perfil, YAML ni cambio activo.

## Por qué no aumentar epsilon globalmente

Los 365 puntos medidos no tuvieron intersección de rayo con el modelo, por lo
que aumentar epsilon no es una cobertura geométrica dirigida para ellos.
Un epsilon global amplía la banda de aceptación de todas las cápsulas y puede
ocultar puntos externos válidos cerca del brazo. También altera por igual las
fronteras `self/external/occluded`, sin resolver la expansión bloqueada de la
cápsula 2. La evidencia soporta sólo ajustes locales revisables, no un cambio
global.

## Alcance preservado

No se ejecutó ninguna medición, sonda, build, prueba, `git diff` ni consulta
ROS. No se modificaron código, YAML, launch, CMake, parámetros, ROI, epsilon,
TF, extrínseca, cápsulas o perfiles. La sesión coverage_v1, cámara, RViz y
JACO2 no se cerraron ni reiniciaron; no hubo comandos ni servicios físicos.
