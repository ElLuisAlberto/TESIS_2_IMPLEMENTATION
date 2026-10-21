# Evidencia offline del control cartesiano físico

Fecha: 2026-10-20

```bash
colcon build --packages-select \
  thesis_core thesis_simulation thesis_hardware thesis_ui --symlink-install

colcon test --packages-select \
  thesis_core thesis_simulation thesis_hardware thesis_ui \
  --return-code-on-test-failure
```

Resultados por paquete:

```text
Summary: 174 tests, 0 errors, 0 failures, 1 skipped
Summary: 43 tests, 0 errors, 0 failures, 1 skipped
Summary: 13 tests, 0 errors, 0 failures, 1 skipped
Summary: 28 tests, 0 errors, 0 failures, 1 skipped
```

La prueba focal modificada terminó con `58 passed`. No hubo ejecución ROS ni
movimiento físico.

## Integración mock

Se inició el launch físico en `ROS_DOMAIN_ID=79` con:

```text
mock_hardware=true
hardware_output_enabled=false
start_gui=false
start_rviz=false
```

Los tres adaptadores cartesianos aparecieron en el grafo. El interbloqueo
FK–TF pasó de espera inicial a `error=0.000000 m`, readiness publicó `READY`,
`/thesis/hardware/armed=false` y no apareció ninguna intención en
`/thesis/cartesian_jog_intent`. El launch se cerró limpiamente con SIGINT.

