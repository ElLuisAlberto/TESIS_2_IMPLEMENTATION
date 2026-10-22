# Fase 6E — Primera sesión física temporal de sombra

Fecha: 2026-10-21. Resultado: **BLOCKED_RUNTIME_PREREQUISITE**.

## YAML temporal preparado y validado

Se creó exclusivamente para esta sesión:

`evidence/codex_reports/2026-10-21_d435i_extrinsic_session_candidate.yaml`

El guard existente lo aceptó. Declara `session_only: true`,
`accepted_for_session: true`, `parent_frame: world`,
`child_frame: d435i_depth_optical_frame`, traslación `[0.68, -0.03, 0.30]` y
cuaternión de la hipótesis pitch -10 grados. Sus métricas señalan
`approximate_not_calibrated`, `sample_count: 0` y `final_extrinsic: false`; no
es una extrínseca final ni persistente.

## Único intento de launch

Se ejecutó una única vez el launch autorizado con:

```text
start_jaco_read_only:=true
start_camera:=true
start_rviz:=true
enable_self_filter:=true
publish_session_extrinsic:=true
hardware_output:=false
```

El launch terminó antes de crear procesos por este prerrequisito faltante:

```text
No such file or directory:
install/thesis_hardware/share/thesis_hardware/launch/thesis_hardware.launch.py
```

Por tanto no fue posible verificar diagnóstico físico de solo lectura,
`joint_states`, TF `world -> d435i_depth_optical_frame`, perfil de nube,
tópicos `points_self_confirmed`, `points_external_candidate`,
`points_occluded_unknown` ni diagnósticos del filtro. No hay capturas de RViz.

## Seguridad y cierre

La comprobación posterior no encontró launch de sesión, D435i, RViz,
`thesis_hardware`, `jaco_self_filter_node` ni `static_transform_publisher`
activos. El bloqueo ocurrió antes de conexión de hardware: no hubo movimiento,
salida física, TF temporal, publicaciones de sombra, `/thesis/obstacle_input`
ni supervisor. No se hicieron reintentos, ajustes de hipótesis ni cambios de
código/launch/configuración productiva en esta fase.

La corrección necesaria para una fase posterior es alinear el contrato de
`jaco_d435i_shadow_session.launch.py` con el nombre/ruta de launch de solo
lectura realmente instalado por `thesis_hardware`; esta fase no la implementa.
