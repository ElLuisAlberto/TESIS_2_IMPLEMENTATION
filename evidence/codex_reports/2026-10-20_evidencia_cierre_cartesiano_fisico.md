# Evidencia de cierre cartesiano físico

Fecha: 2026-10-20

## Pruebas aisladas

```bash
colcon build --symlink-install --packages-select \
  thesis_core thesis_simulation thesis_ui thesis_hardware

colcon test --packages-select \
  thesis_core thesis_simulation thesis_ui thesis_hardware \
  --test-result-base \
  evidence/codex_reports/2026-10-20_cartesian_close_test_results \
  --return-code-on-test-failure

colcon test-result --test-result-base \
  evidence/codex_reports/2026-10-20_cartesian_close_test_results --verbose
```

Resultado: `259 tests, 0 errors, 0 failures, 4 skipped`.

## Integración mock

Launch en `ROS_DOMAIN_ID=79`, `mock_hardware=true`,
`hardware_output_enabled=false`, `use_demo_obstacle=true`, sin GUI/RViz.

Observado:

- nodos `cartesian_command_adapter`, `cartesian_jog_adapter` y
  `cartesian_trajectory_adapter` activos;
- `system_readiness=READY`;
- `kinematic_tf_status=VALID error=0.000000m tolerance=0.005000m`;
- `armed=false`, `jog_active=false`, `mode=hold`;
- entrada JOG y salida física `0.000000 Hz`.

## Preflight físico de la configuración final

Se inició temporalmente con `mock_hardware=false`, salida deshabilitada,
`use_demo_obstacle=false`, sin GUI ni RViz. No se armó ni se publicaron
candidatos.

Resultados:

```text
/thesis/system_readiness:
  NOT_READY: proximidad ausente u obsoleta

/thesis/obstacle_input:
  Publisher count: 0
  Subscription count: 1 (obstacle_input_bridge)

/thesis/kinematic_tf_status:
  VALID error=0.000000m tolerance=0.005000m

diagnóstico:
  backend=kinova_usb
  connected=true
  api_open=true
  fault=false
  state_valid=true
  feedback_rate_hz_measured=19.999693
  output_permitted=false
  armed=false
  jog_active=false
  proximity_valid=false
  system_ready=false
  output_command_rate_hz_measured=0.000000
  mode=hold
```

El launch se cerró ordenadamente con SIGINT y la API Kinova liberó su sesión.
No se abrió el launch final armable porque no cumplía `READY`.
