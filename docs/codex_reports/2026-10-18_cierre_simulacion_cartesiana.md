# Cierre de validación cartesiana en simulación

Fecha: 2026-10-18

## Resultado de cierre

La regresión headless final de Z **aprobó**. Desde `initial_pose:=z_ready` se
publicó una única intención cartesiana de +20 mm en Z. El observador
correlacionó correctamente la intención, su conversión IK, la decisión del
supervisor y el estado terminal de ejecución:

```text
command_id: codex-z-ready-20261018-final
paso solicitado: 0.020000 m
supervisión: ALLOW
ejecución: SUCCEEDED
error final de posición: 0.000147 m
error final de orientación: 0.087 grados
resultado del validador: OK
```

Las precondiciones fueron `READY`, proximidad `ALLOW` y despeje mínimo de
`1.202309 m`. El objetivo conservó X/Y y orientación y se calculó desde la TF
viva, sumando únicamente 0,020 m a Z.

## Estado de la incidencia previa

La incidencia anterior `COMANDO=AUSENTE` no se reprodujo. Antes de publicar se
confirmaron los dos suscriptores esperados en
`/thesis/cartesian_candidate`, incluido `cartesian_sim_validator`; todos sus
checks terminaron en `OK`.

Tampoco se reprodujeron las excepciones rclpy del cierre anterior. Todos los
procesos de la pila finalizaron limpiamente y no quedaron procesos residuales.

## Verificaciones ya disponibles antes de la regresión

La tarea interrumpida había dejado terminadas la compilación de los paquetes
afectados y sus pruebas:

- `thesis_core`: `139 passed, 1 skipped`.
- `thesis_simulation`: `41 passed, 1 skipped`.

En esta continuación no se recompiló, no se repitieron esas suites y no se
editó ni rehízo el parche. Se revisaron previamente `git diff`, el resultado
más reciente y los informes existentes, preservando todos los cambios del
árbol de trabajo.

## Alcance de seguridad

La ejecución fue exclusivamente en Gazebo headless, con
`simulation_output_enabled:=true`, GUI y RViz deshabilitados y el obstáculo de
prueba ubicado en X=2,0 m. No se activó `hardware_output`, no se accedió al
Kinova Jaco2 físico y no se usó `sudo`.

Evidencia completa:
[2026-10-18_regresion_final_z_headless.md](../../evidence/codex_reports/2026-10-18_regresion_final_z_headless.md).
