# Fase 4B: observación física pasiva D435i

Fecha: 2026-10-21
Dictamen: **BLOCKED_CAMERA_OBSERVATION**

## Resultado

La Intel RealSense D435i fue detectada correctamente por el driver y llegó al estado `RealSense Node Is Up`. Se identificó como `RealSense D435I`, serial `405622074532`, USB 3.2, con perfiles Depth y RGB configurados a 640×480@15 Hz.

La caracterización no pudo comenzar porque la orden autorizada para el probe devolvió inmediatamente:

```text
No executable found
```

Conforme a la instrucción de detenerse ante fallo, no se recompiló, reparó, instaló ni modificó ningún parámetro. El launch de la D435i iniciado para esta prueba fue cerrado y no se realizaron pruebas adicionales.

## Resultado del probe y artefactos

- Probe: no iniciado; no alcanzó `WAITING_FOR_CLOUD`, `COLLECTING`, `COMPLETE`, WARN ni ERROR propios del nodo.
- JSON solicitado: `/home/luis/Descargas/D435I_OBSERVACION_BASELINE_20261021_110010.json`
- JSON generado: no; el ejecutable no existía para `ros2 run`.
- Log: [D435I_OBSERVACION_BASELINE_20261021_110010.log](/home/luis/Descargas/D435I_OBSERVACION_BASELINE_20261021_110010.log)

## Métricas no disponibles

No hay datos de nube porque el probe no se ejecutó. Por tanto no se observaron ni se infieren:

- `frame_id` real de PointCloud2;
- CameraInfo;
- frecuencia de nube;
- edad o calidad de sellos;
- proporciones de puntos menores de 0.10 m, entre 0.10 y 1.50 m, o mayores de 1.50 m.

La detección del driver no sustituye estas métricas y no constituye calibración de `world -> cámara`.

## Confirmaciones de seguridad

- El JACO2 no fue iniciado ni movido.
- `thesis_hardware`, launch físico, GUI física, Gazebo y RViz no fueron iniciados.
- `hardware_output` no fue activado.
- No se publicó `/thesis/obstacle_input`.
- No hubo conexión al supervisor ni se habilitó `jaco_self_filter_node`.
- No se modificó código, configuración ni se instaló software durante esta ejecución.

## Decisión final

**BLOCKED_CAMERA_OBSERVATION**: la cámara fue detectada, pero falta disponibilidad del ejecutable instalado `d435i_observation_probe`; sin él no existe JSON ni evidencia de las métricas requeridas. Se requiere resolver esa disponibilidad en una tarea separada y autorizada antes de repetir la observación pasiva.
