# Fase 8Q-AUTO — evaluación de máscara diagnóstica automática

## Dictamen único: `MISSING_GEOMETRIC_SNAPSHOT`

No se generó una máscara automática. La captura 8P conserva imagen RGB, nube
filtrada, intrínsecas y TF de cámara en el sello `1792637873543140137 ns`,
pero no conserva las poses `world -> link` de las mallas de colisión en ese
sello ni un `robot_description` o referencia verificable al XML exacto usado
por el nodo mesh-shadow. Sin las poses de los links no se puede reconstruir
la silueta de malla sobre `rgb.png`. Una consulta TF actual o `latest` no
representaría necesariamente la postura de la captura. La geometría visible
en RGB tampoco permite distinguir de manera fiable brazo, persona, mesa y
fondo sin la restricción de silueta exigida por esta fase.

## Evidencia revisada

Se inspeccionaron sin modificar:

- `evidence/codex_reports/2026-10-22_fase8p_snapshot/rgb.png`:
  imagen RGB8 de 640 × 480 con el JACO2, una persona y mobiliario.
- `evidence/codex_reports/2026-10-22_fase8p_snapshot/rgb_proyeccion_nube.png`:
  proyección cian de la nube; es una observación de puntos, no la silueta
  geométrica de las mallas.
- `evidence/codex_reports/2026-10-22_fase8p_snapshot/metadata.json`:
  sellos RGB, CameraInfo y nube idénticos; `K`, `D`, `P`, TF
  `world <- d435i_depth_optical_frame` y
  `d435i_color_optical_frame <- world` al sello de nube.
- `evidence/codex_reports/2026-10-22_fase8p_snapshot/cloud_world_xyz.npz`:
  1 242 coordenadas XYZ en `world`; no contiene poses de links.

| Dato para reconstruir la silueta | Estado en snapshot 8P |
|---|---|
| Intrínsecas y distorsión RGB | Disponible |
| TF mundo/cámara óptica al sello | Disponible |
| XYZ de la nube y sello común | Disponible |
| `world -> link` de las 13 mallas al sello | **Ausente** |
| XML exacto `robot_description` o referencia inmutable verificable | **Ausente** |

Los archivos de launch y URDF presentes en el repositorio no sustituyen el
TF histórico de cada link. El archivo temporal de parámetros del proceso,
si aún existiera, tampoco identifica por sí solo las poses de los links en
`1792637873543140137 ns`. Por ello no se aplicaron GrabCut, erosión,
dilatación ni umbrales de aceptación. El margen inicial propuesto de 45 px
queda **sin aplicar**; no se calculan área, componentes, solape o cobertura
por regiones porque no existe una silueta válida. No se crearon
`mesh_silhouette.png`, `auto_mask_candidate.png`,
`rgb_auto_mask_overlay.png`, `auto_mask_metrics.json` ni
`mask_brazo_auto.png`. Tampoco se tocó `mask_brazo_manual.png`.

## Propuesta de captura complementaria, sin ejecutarla

En una futura captura **única y de sólo lectura**, mantener el JACO2 inmóvil
y la sesión existente. Tomar RGB, CameraInfo y nube filtrada con un sello
común (o diferencias absolutas de hasta 33 ms, registradas). En el sello de
la nube, guardar las matrices completas `world -> link` de **cada** link con
`collision_mesh` en el `robot_description` efectivo, junto con el TF
`world -> d435i_depth_optical_frame` y el TF necesario para proyectar en
`d435i_color_optical_frame`. Si cualquier TF de link falta en ese sello,
rechazar la captura completa.

Guardar también el XML exacto del `robot_description` efectivo, o una
referencia inmutable al archivo exacto más SHA-256 de XML y de cada malla
referenciada. Registrar nombres de link, URI de malla, `collision origin` y
escala, para que la silueta se pueda reconstruir offline sin consultar TF
actual ni geometría que pudiera cambiar. Los nuevos artefactos deben quedar
en `evidence/codex_reports/`, separados de configuración operativa. Esta
propuesta no autoriza ejecutar otra captura ahora.

## Estado y validación

Sólo se añadió este informe. No se añadió código ni se ejecutaron pruebas
de proyección, binariedad o conectividad: la condición geométrica previa
impide crear una máscara candidata verificable. No se requirió compilación.
`git diff --check` no mostró errores.

No se hizo ninguna consulta ROS de runtime, captura nueva, publicación ni
cambio de sesión. No se enviaron órdenes al JACO2 ni se cambió
`hardware_output=false`, TF, extrínseca, epsilon, ROI, mallas, URDF/Xacro,
radios, filtros o clasificación. No se creó una máscara para ROS, seguridad
o control.
