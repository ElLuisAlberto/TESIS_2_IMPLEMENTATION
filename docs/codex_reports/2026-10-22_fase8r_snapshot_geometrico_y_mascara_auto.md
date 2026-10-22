# Fase 8R — captura geométrica y evaluación de máscara automática

## Dictamen único: `AUTO_MASK_NOT_TRUSTWORTHY`

La captura geométrica se completó una sola vez con RGB, CameraInfo y nube
filtrada en el mismo sello, más TF de cámara y los 13 links de malla de
colisión en ese sello. La máscara candidata offline **no** superó la
aceptación conservadora: 9 995 de 20 152 píxeles blancos (49,6 %) quedaron
fuera de la silueta proyectada de la malla. El overlay muestra desacuerdo
espacial entre silueta y brazo visible. No se creó `mask_brazo_auto.png` ni
`residuals_masked_by_link.json`; no se infiere corrección de TF.

## Captura, procedencia y sincronía

Se extendió el capturador observacional C++ de 8P, sin un segundo nodo de
cámara. Antes de ejecutarlo se verificó que el proceso activo
`jaco_mesh_shadow_node` PID `110060` usaba
`/tmp/launch_params_1mvt_im_`. Su parámetro tipado `robot_description`
contiene 13 links con `collision_mesh`; las ocho mallas referenciadas eran
accesibles y tenían fecha de modificación anterior al inicio del proceso.
El XML exacto y los bytes de cada malla se conservaron antes de la captura.
El cliente C++ sólo se suscribe a RGB, CameraInfo, nube y TF; sus opciones
desactivan `rosout`, servicios y eventos de parámetros. No publica nada.

| Dato | Frame | Sello de consulta o mensaje (ns) | Diferencia frente a nube |
|---|---|---:|---:|
| RGB `/camera/d435i/color/image_raw` | `d435i_color_optical_frame` | 1792639154117031982 | 0 ns |
| CameraInfo color | `d435i_color_optical_frame` | 1792639154117031982 | 0 ns |
| Nube `/thesis/perception/points_filtered` | `world` | 1792639154117031982 | referencia |
| TF `world <- d435i_depth_optical_frame` | `world` | 1792639154117031982 | 0 ns |
| TF `world <- d435i_color_optical_frame` | `world` | 1792639154117031982 | 0 ns |
| TF `d435i_color_optical_frame <- world` | color optical | 1792639154117031982 | 0 ns |
| TF `world <-` cada uno de los 13 links | `world` | 1792639154117031982 | 0 ns |

El capturador exige diferencia absoluta de RGB y CameraInfo de hasta 33 ms
respecto de la nube; en esta captura ambas diferencias fueron exactamente
cero. El paquete se rechazaba si faltaba un solo TF de link. El archivo
`link_transforms_at_stamp.json` conserva 13 entradas únicas y todos sus
sellos devueltos son `1792639154117031982 ns`. Se guardaron 1 211 puntos
XYZ de la nube filtrada en `world`.

El SHA-256 del `robot_description.urdf` congelado es
`b2d9d0d5f1366915ebe70583b8ebb048a7c445b97353b6b9424148f5644352d2`.
El manifiesto conserva URI, escala y `collision origin` de cada instancia:

| Link | Malla | SHA-256 de archivo congelado |
|---|---|---|
| `j2n6s300_link_base` | `base.dae` | `84d6c46a5cf934f8d9abd669928948857ef8eec0d537ea226e9e44cd72e5133c` |
| `j2n6s300_link_1` | `shoulder.dae` | `c94f20e293244a5cf55b724cc19573fb5f4dd3e7fa3ba6fb17f1582bc9bf9963` |
| `j2n6s300_link_2` | `arm.dae` | `34fbe87b12d0878546ad8f01541c61d4a54d204e52b03f3aa83955d47b5fc067` |
| `j2n6s300_link_3` | `forearm.dae` | `4da6f9148f2b3fd0d4fdfa13129f1a1f2080277a07598540636c4b9eccabc170` |
| `j2n6s300_link_4`, `link_5` | `wrist.dae` | `dd7f14d83ee829804b780d152adb878034d2fc3e142a3a2ea9ac26ed8e68c4a4` |
| `j2n6s300_link_6` | `hand_3finger.dae` | `018d65704e40f198c2af21ce69de59b955f13c238875f0df229ac7f8436bb003` |
| `j2n6s300_link_finger_1`, `_2`, `_3` | `finger_proximal.dae` | `ce16acaa3571b4023c131c7310744fc28894c9703c9833a3d274bb38a704fa33` |
| `j2n6s300_link_finger_tip_1`, `_2`, `_3` | `finger_distal.dae` | `73f6662e0e9c47e43b9f3018069cf6285f40e163d3555a6928607f058ed7cffb` |

## Reconstrucción y aceptación offline

El análisis leyó únicamente `rgb.png`, nube XYZ, intrínsecas, TF, XML y
copias de mallas de la carpeta 8R. Verificó SHA-256 del XML y de las ocho
mallas antes de cargar geometría. Assimp trianguló las 13 instancias de
colisión congeladas (89 728 triángulos), aplicó escala, `collision origin` y
TF de link congelados, y proyectó sus vértices con el TF de color y las
intrínsecas guardadas. De esa unión se obtuvo la silueta binaria.

Se erosionó la silueta 8 px para la semilla interna y se dilató 45 px para
la banda incierta. GrabCut se aplicó sólo dentro de esa banda, con fondo
seguro fuera de ella. El soporte de profundidad provino de los puntos XYZ
proyectados, dilatados 6 px; el umbral de borde de profundidad fue 0,12 m.
Los píxeles sin soporte no podían quedar blancos. Los componentes sin contacto
con la semilla se eliminaron. La candidata se usó sólo para métricas y overlay,
sin publicarla ni guardarla como máscara válida.

| Métrica de aceptación | Valor |
|---|---:|
| Silueta / banda dilatada | 27 970 / 118 648 px |
| Semilla con soporte de nube | 7 230 px |
| Candidata blanca | 20 152 px; cuatro componentes |
| Píxeles blancos fuera de la banda | 0 (0 %; supera límite <2 %) |
| Píxeles blancos fuera de la silueta | 9 995 (49,6 %; falla umbral conservador ≤5 %) |
| Componentes aislados eliminados | 0; los cuatro tocaban la semilla |
| Regiones visibles cubiertas | cinco: base/hombro, brazo superior, antebrazo, muñeca y efector |
| Puntos de nube proyectados / rechazados por borde | 1 211 / 0 |
| Hashes de malla y XML | verificados |

La condición adicional de ≤5 % fuera de la silueta se fijó para evitar
aceptar una segmentación que depende de una banda amplia mientras la
geometría proyectada discrepa del brazo visible. El overlay demuestra esa
discrepancia; la persona y el fondo próximos vuelven ambigua la pertenencia
de los píxeles en la banda. Los criterios de binariedad, conectividad, banda
y cobertura no compensan este fallo. Por ello se detuvo la fase antes de
calcular residuos. No hay tabla de P50/P90/P95/P99 para puntos etiquetados
como brazo, ni dictamen global/local defendible a partir de esta máscara.

## Artefactos y validación

La carpeta nueva
`evidence/codex_reports/2026-10-22_fase8r_geometric_snapshot/`
contiene `rgb.png`, `cloud_world_xyz.npz`, `metadata.json`,
`robot_description.urdf`, `link_transforms_at_stamp.json`,
`collision_mesh_manifest.json`, `hashes.json`, las ocho copias en `meshes/`,
`mesh_silhouette.png`, `rgb_auto_mask_overlay.png` y
`auto_mask_metrics.json`. No existe `mask_brazo_auto.png` ni
`residuals_masked_by_link.json` porque la máscara fue rechazada.

Se compiló manualmente el cliente C++ extendido y el exportador offline de
triángulos; `colcon build --packages-select thesis_jaco_perception` terminó
correctamente. Las pruebas nuevas de procedencia y paquete de captura pasaron
2/2; las pruebas offline de hashes, proyección, binariedad, conectividad y
rechazo de componentes externos pasaron 4/4. `git diff --check` no mostró
errores.

Se hizo una sola captura runtime, sin reinicio de cámara, launch ni RViz.
Los procesos originales seguían activos tras el análisis. No se movió ni
armó el JACO2, no se enviaron órdenes y `hardware_output=false` permaneció
en la sesión. No se alteraron TF, extrínseca, epsilon, ROI, radios, mallas,
URDF/Xacro, filtros, mesh-shadow o clasificación. La candidata nunca se
conectó a ROS, seguridad ni control; no se optimizó ni editó TF.
