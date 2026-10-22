# thesis_jaco_perception

Preparación aislada del auto-filtrado para un JACO2 visto por una D435i fija.
No es una calibración ni una fuente de decisión preventiva.

## Arquitectura por fases

**Fase 1** proporciona la biblioteca C++ pura de visibilidad rayo-cápsula. La
biblioteca clasifica un retorno contra un modelo que debe proporcionarse de
forma explícita:

- `SELF_CONFIRMED`: el retorno coincide, dentro de `epsilon_total`, con la
  primera superficie prevista del modelo.
- `EXTERNAL_CONFIRMED`: no hay intersección con el modelo en ese rayo, o el
  retorno está delante de la primera superficie prevista.
- `OCCLUDED_UNKNOWN`: el retorno está detrás de la primera superficie prevista;
  nunca significa espacio libre.

**Fase 2** añade `jaco_self_filter_node`. El nodo recibe una `PointCloud2` que
ya debe estar expresada en `world`, consulta TF exacto al sello de adquisición
para la cámara óptica y los extremos de cápsulas, y particiona los puntos.
Nunca sustituye TF por la última transformación disponible. Publica solo:

- `/thesis/jaco_perception/points_self_confirmed`
- `/thesis/jaco_perception/points_external_candidate`
- `/thesis/jaco_perception/points_occluded_unknown`

El nodo conserva sello y campos de cada punto cuando los publica, usa QoS de
sensor con profundidad uno y emite diagnósticos. No consulta `JointState`
directamente.

**Fase 3** añade `config/jaco_d435i_shadow.yaml` y
`launch/jaco_rgbd_shadow.launch.py`. El launch inicia exclusivamente el nodo
sombra y carga ese YAML, que mantiene `enabled=false`. No inicia cámara,
visualizador, publicador de estado del robot, TF estático ni stack físico.

## Configuración sombra y calibración física

El YAML de sombra es un contrato de bloqueo deliberado, no una calibración
física: el frame óptico y las listas de cápsulas están vacíos, y
`epsilon_total_m=-1.0` es inválido a propósito. El nodo debe permanecer
deshabilitado hasta contar con configuración física validada externamente.

Antes de habilitarlo se deben medir y aprobar, como mínimo:

1. La pose `world -> cámara`.
2. El frame óptico real.
3. Los radios interiores del modelo de auto-filtrado.
4. `epsilon_total_m` con sus contribuciones trazables.
5. La latencia extremo a extremo.
6. El volumen observable validado.

No copiar radios preventivos de `thesis_core` como radios interiores. La
calibración física y los volúmenes certificados pertenecen a una fase posterior.

El paquete **no publica obstáculos** y está explícitamente prohibido conectarlo
al supervisor o a `/thesis/obstacle_input`. No contiene extrínsecos,
intrínsecos, radios internos ni calibración física. Las cápsulas presentes en
las pruebas son únicamente casos geométricos sintéticos y no son la máscara
preventiva del supervisor.

## Passive D435i observation probe (Phase 4A)

`d435i_observation_probe` is observation-only. It passively subscribes to a
point cloud and, only when a user explicitly provides an optional CameraInfo
topic, to CameraInfo. It does not estimate or publish `world -> camera`, does
not use TF, does not transform clouds, and does not publish PointCloud2 or
obstacles. It is not connected to the supervisor or `jaco_self_filter_node`.

During its finite observation window it reports the real cloud frame/layout,
rate and stamp quality, finite-point/radial-range buckets, and CameraInfo only
when received. `report_path` is empty by default; only an explicit nonempty
path receives a compact JSON report. It never stores clouds, images or bags.

This probe is not calibration or physical validation. Its first future use must
be with the JACO immobile and disarmed. Its report is input to the next mounting
and extrinsic-calibration phase, never authorization to enable the self-filter
or connect anything to the supervisor.
