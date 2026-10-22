# 2026-10-21 — Fase 8M: vista RViz mínima de nube mesh sin brazo

## Dictamen

`BLOCKED_RVIZ_RUNTIME`

## Causa y cambio estático

La Fase 8L se bloqueó porque la instancia RViz de la comparación dejó de existir: las seis salidas conservaron publicador pero volvieron a cero suscriptores. No hubo evidencia de fallo de mesh, TF o cámara.

Se creó exclusivamente `src/thesis_jaco_perception/rviz/jaco_d435i_mesh_arm_removed.rviz`. El perfil fija `world`, habilita `RobotModel` y `TF`, y contiene exactamente un display `rviz_default_plugins/PointCloud2`:

| Nombre | Tópico | Enabled | Color | QoS |
|---|---|---|---|---|
| Mesh External Candidate — Arm Removed | `/thesis/jaco_perception/mesh_shadow/points_external_candidate` | true | magenta (`255; 0; 255`) | Best Effort, Volatile, Keep Last, 5 |

No contiene nube filtrada, cápsulas, mesh self/occluded ni rutas de supervisor, horizonte, extractor, comandos, `obstacle_input` o control. El contrato estático exige esta forma y fija los hashes de los perfiles existentes de alineación y comparación, que no fueron modificados.

## Validación estática

```text
colcon build --packages-select thesis_jaco_perception --symlink-install
Summary: 1 package finished [0.64s]

colcon test --packages-select thesis_jaco_perception --ctest-args -R test_shadow_session_contract
Summary: 1 package finished [1.15s]

git diff --check
# sin salida; correcto
```

## Intento runtime único de la ventana adicional

Antes de abrir RViz, la nube mesh external tenía `Publisher count: 1` y `Subscription count: 0`. Se abrió una única ventana adicional con el perfil nuevo, en un grupo de sesión separado, sin cerrar ni modificar la sesión ROS. Tras ocho segundos:

```text
Publisher count: 1
Subscription count: 0
```

No quedó proceso RViz y `/tmp/fase8m_rviz.log` no contiene error literal. Por la regla de no reintento no se abrió otra ventana, no se modificó la sesión y no se ajustó QoS ni ningún nodo. Por ello no pudo verificarse que el modelo gris y la única nube magenta fueran visibles.

## Seguridad e integridad

La sesión original continúa activa con `hardware_output=false`, JACO2 sólo lectura y desarmado. No hubo movimiento, órdenes al brazo, cambios en launch/filtros/TF, conexiones a seguridad/control ni captura de pantalla. La captura solicitada queda `CAPTURE_UNAVAILABLE` porque la ventana no permaneció activa.
