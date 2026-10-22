# Fase 8P — captura sincronizada y máscara diagnóstica del brazo

## Inspección estática previa al código

La sesión activa ofrece `/camera/d435i/color/image_raw` (`sensor_msgs/Image`,
frame `d435i_color_optical_frame`), `/camera/d435i/color/camera_info`
(`CameraInfo`, mismo frame), `/camera/d435i/aligned_depth_to_color/image_raw`
y su `camera_info` (ambos en `d435i_color_optical_frame`), además de
`/thesis/perception/points_filtered` (`PointCloud2`, frame `world`).
`/camera/d435i/depth/color/points` es la nube fuente de la filtrada. El código
del preprocesador conserva su sello (`restamp_output=false`) y transforma al
frame `world` con TF al sello (`use_latest_transform=false`). El TF requerido
es `world -> d435i_depth_optical_frame`; para proyectar en RGB se necesita
también `world -> d435i_color_optical_frame` al mismo sello de nube.

Los sellos vistos por consultas CLI separadas prueban que cada tópico está
activo, pero al ser observaciones sucesivas no prueban sincronía entre ellos.
La herramienta aceptará una captura cuando RGB, nube filtrada y CameraInfo
color tengan sellos idénticos o diferencia absoluta máxima de **0.033 s**
respecto de la nube. Rechazará diferencias mayores y registrará la diferencia
exacta de cada mensaje. La profundidad alineada existe, pero la nube filtrada
es el dato de profundidad necesario; la captura no exigirá una imagen de
profundidad adicional para evitar una quinta asociación temporal innecesaria.
Las intrínsecas RGB `K`, distorsión `D` y el TF `world -> color optical` servirán
para la previsualización de proyección. `rclpy`, `numpy`, `PIL` y `cv2` ya están
disponibles; no se instalarán paquetes.

## Dictamen único: `SNAPSHOT_READY_WAITING_OPERATOR_MASK`

Se obtuvo una captura válida con coincidencia exacta de sellos. El cliente ROS
fue `mesh_arm_snapshot_once.cpp`: tres suscripciones de datos (RGB, CameraInfo
color y nube filtrada) más las suscripciones internas del listener TF. Sus
opciones de nodo desactivan `rosout`, servicios de parámetros y publicador de
eventos de parámetros. No crea otros publicadores, TF, servicios o parámetros.
Acepta una sola terna válida y termina; su plazo máximo de captura es 10 s.
La primera invocación de la versión Python se detuvo antes de crear el nodo o
suscripciones porque `rclpy` Humble no acepta la opción de desactivar el
publicador implícito de eventos de parámetros. Se corrigió la implementación
antes de cualquier captura. El cliente C++ se ejecutó una sola vez.

| Mensaje o TF | Tópico / consulta | Frame | Sello (ns) | Diferencia frente a nube |
|---|---|---|---:|---:|
| RGB | `/camera/d435i/color/image_raw` | `d435i_color_optical_frame` | 1792637873543140137 | 0 ns |
| CameraInfo RGB | `/camera/d435i/color/camera_info` | `d435i_color_optical_frame` | 1792637873543140137 | 0 ns |
| Nube filtrada | `/thesis/perception/points_filtered` | `world` | 1792637873543140137 | referencia |
| TF profundidad | `world <- d435i_depth_optical_frame` | `world` | 1792637873543140137 | 0 ns |
| TF proyección RGB | `d435i_color_optical_frame <- world` | color optical | 1792637873543140137 | 0 ns |

La imagen es RGB8 de 640 × 480 píxeles. Se guardaron `K`, `D`, `P`, modelo de
distorsión y los dos TF en la metadata. La nube guardada contiene solamente
las coordenadas XYZ en `world` de 1 242 puntos; los 1 242 proyectaron dentro
de la imagen RGB. La previsualización muestra puntos cian sobre el brazo, pero
también contiene fondo, mobiliario y una persona. No se generó una máscara
automática ni se interpretó todo píxel como brazo.

Artefactos, todos en `evidence/codex_reports/2026-10-22_fase8p_snapshot/`:

| Archivo | Contenido |
|---|---|
| `rgb.png` | Imagen original para dibujar una máscara del mismo tamaño. |
| `rgb_para_mascara.png` | Copia con instrucciones visibles; el texto tapa la franja superior, por lo que debe usarse sólo como guía. |
| `rgb_proyeccion_nube.png` | Previsualización diagnóstica de XYZ proyectado en cian. |
| `cloud_world_xyz.npz` | Coordenadas XYZ estrictamente necesarias para el análisis posterior. |
| `metadata.json` | Sellos, diferencias, frames, intrínsecas, TF y conteos. |

Se espera un archivo **nuevo** `mask_brazo_manual.png` en esa misma carpeta:
escala de grises, 640 × 480, sólo valores 0 y 255. Blanco (255) debe marcar
únicamente base, brazo superior, antebrazo, muñeca, mano y dedos **visibles**;
negro (0) debe cubrir fondo, persona, mesa, cables y objetos externos. Se debe
anotar sobre `rgb.png`, sin usar los puntos cian ni el banner como frontera.
No se dispone aquí de una UI de anotación integrada; puede usarse una
herramienta gráfica ya instalada. La máscara será evidencia nueva, nunca
configuración operativa. Al no existir aún, no se ejecutó la validación offline
de residuos ni se construyó una herramienta de calibración.

## Validación y estado seguro

La fuente C++ se compiló manualmente contra las dependencias existentes, sin
modificar CMake ni `package.xml`. `colcon build --packages-select
thesis_jaco_perception` finalizó correctamente. Las tres pruebas offline de
sincronía, proyección y validación de máscara pasaron (`3/3`). `git diff
--check` finalizó sin errores; los archivos nuevos se comprobaron también
sin errores de espacios.

No se movió ni armó el JACO2, no se enviaron órdenes, y `hardware_output=false`
continuó en la sesión. Los procesos de cámara, TF temporal, preprocesador,
ambos filtros, RViz y launch seguían activos después de la captura. La máscara
no se conectó a ningún filtro operativo. No se cambió TF, epsilon, ROI,
geometría de malla ni clasificación mesh-shadow. No se reinició la sesión ni
se inició calibración automática.
