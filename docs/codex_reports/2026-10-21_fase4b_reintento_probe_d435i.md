# Fase 4B: reintento de probe pasivo D435i

Fecha: 2026-10-21
Dictamen: **PASS_CAMERA_OBSERVATION**

## Causa raíz y reparación

La causa de `No executable found` fue el destino de instalación de ejecutables en `src/thesis_jaco_perception/CMakeLists.txt`: `RUNTIME DESTINATION bin` instalaba los binarios en `install/thesis_jaco_perception/bin/`, pero `ros2 run` y `ros2 pkg executables` descubren ejecutables de un paquete ament en `lib/<paquete>/`.

Se modificó únicamente:

- `src/thesis_jaco_perception/CMakeLists.txt`: `RUNTIME DESTINATION` cambió de `bin` a `lib/${PROJECT_NAME}`.

No se modificó la lógica del probe, Fases 0–3, geometría, YAML sombra, percepción, supervisor ni hardware.

## Evidencia local de empaquetado

Tras reconstruir exclusivamente `thesis_jaco_perception`, la salida de `ros2 pkg executables thesis_jaco_perception` fue:

```text
thesis_jaco_perception d435i_observation_probe
thesis_jaco_perception jaco_self_filter_node
```

La compilación terminó correctamente y la suite exclusiva terminó con:

```text
28 tests, 0 errors, 0 failures, 0 skipped
```

`git diff --check` no reportó errores.

## Resultado del probe físico pasivo

La D435i fue detectada por el driver como `RealSense D435I`, serial `405622074532`; el driver alcanzó `RealSense Node Is Up`. El probe terminó su ventana de 30 s con:

```text
D435i passive observation complete: state=COMPLETE, clouds=450, rate_hz=15, finite_points=10858986
```

Artefactos generados:

- JSON: [D435I_OBSERVACION_BASELINE_20261021_110550.json](/home/luis/Descargas/D435I_OBSERVACION_BASELINE_20261021_110550.json)
- Log: [D435I_OBSERVACION_BASELINE_20261021_110550.log](/home/luis/Descargas/D435I_OBSERVACION_BASELINE_20261021_110550.log)

### Nube y CameraInfo observados

| Métrica | Resultado |
|---|---:|
| `frame_id` PointCloud2 | `d435i_depth_optical_frame` |
| Layout PointCloud2 | 24168 × 1, `point_step=20`, campos `x,y,z,rgb` |
| CameraInfo | Recibido |
| Frame CameraInfo | `d435i_depth_optical_frame` |
| Resolución CameraInfo | 640 × 480 |
| Modelo de distorsión | `plumb_bob` |
| Frecuencia observada | 15 Hz |
| Edad del último sello | 0.0699456 s |
| Sellos nulos / futuros / vencidos | 0 / 0 / 0 |

### Distribución radial observada

Total de puntos finitos: **10,858,986**.

| Intervalo | Puntos | Proporción de finitos |
|---|---:|---:|
| Menores de 0.10 m | 0 | 0.000000 % |
| Entre 0.10 y 1.50 m | 9,835,641 | 90.576054 % |
| Mayores de 1.50 m | 1,023,345 | 9.423946 % |

Estos resultados caracterizan la cámara temporalmente apoyada; no estiman ni calibran la pose `world -> cámara`.

## Cierre y seguridad

Una vez escrito el JSON y confirmado `COMPLETE`, se detuvo el proceso del probe y después se cerró únicamente el launch RealSense. No quedaron procesos de cámara o probe activos.

- El JACO2 no fue iniciado ni movido.
- `hardware_output` permaneció desactivado.
- No se iniciaron `thesis_hardware`, GUI física, supervisor, Gazebo ni RViz.
- No se publicó `/thesis/obstacle_input`.
- No hubo conexión del probe con el supervisor ni se habilitó `jaco_self_filter_node`.

La aprobación se limita a la observación pasiva de cámara. No autoriza calibración extrínseca, TF estático, auto-filtrado activo ni integración posterior.
