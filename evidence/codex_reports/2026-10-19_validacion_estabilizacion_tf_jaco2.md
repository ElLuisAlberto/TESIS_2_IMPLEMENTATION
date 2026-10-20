# Evidencia de validación TF JACO2 físico

Fecha: 2026-10-19 (America/Lima)

## Capturas

| Captura | Duración | `/joint_states` | `/tf` | Proximidad |
| --- | ---: | ---: | ---: | --- |
| Antes | 30.500 s | 611 | 368 | 476 ALLOW, 116 STOP (`TF_UNAVAILABLE`) |
| Después | 30.219 s | 605 | 605 | 605 ALLOW, 0 STOP |

- Antes: `evidence/rosbags/2026-10-19_jaco2_tf_regimen_30s/`
- Después: `evidence/rosbags/2026-10-19_jaco2_tf_regimen_30s_corregido/`

## Criterios de aceptación

- 30 s continuos sin STOP por TF: **cumplido**.
- Estados articulares y diagnósticos válidos: **cumplido**.
- Brazo desarmado y tasa de comandos físicos 0 Hz: **cumplido**.

El adaptador permaneció `armed=false`, en `hold`, sin objetivos ni jog
activos. No se publicaron comandos ni se movió el JACO2.
