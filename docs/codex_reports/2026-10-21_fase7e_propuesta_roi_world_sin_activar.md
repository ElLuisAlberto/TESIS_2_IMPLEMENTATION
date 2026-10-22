# Fase 7E: propuesta de ROI en `world` sin activar

## Dictamen

`BLOCKED_WORKSPACE_BOUNDS_UNDEFINED`

No se fija ni se activa un ROI numérico. La evidencia disponible confirma el
frame y el mecanismo correctos, pero no delimita de forma medible el fondo ni
el volumen operativo futuro; escoger límites ahora podría descartar un
obstáculo válido.

## Pipeline real y frame de crop

El publicador de `/thesis/perception/points_filtered` es
`/pointcloud_preprocessor`, configurado en la sesión con `target_frame=world`.
La implementación de `pointcloud_preprocessor_node.cpp` ejecuta, en este orden:

1. TF de la nube de entrada a `target_frame` (`world`) con el sello del mensaje
   porque `use_latest_transform=false`.
2. Conversión a PCL y filtro radial de coordenadas ya transformadas.
3. `pcl::CropBox` XYZ si `enable_crop=true`.
4. `pcl::VoxelGrid` si `enable_voxel=true`.

Por ello, tanto el rango ejecutado por este nodo como el CropBox se evalúan en
`world` durante esta sesión. El nombre `max_valid_range_m` no implica un rango
en el frame óptico: el cálculo ocurre después de `tf2::doTransform`.

El launch de sombra deja `enable_crop=false`, `enable_voxel=true`, hoja voxel
`0.01 m`, `min_valid_range_m=0.10 m`, límites XYZ latentes
`x=[-1.5,1.5]`, `y=[-1.5,1.5]`, `z=[-1.0,2.5]`; el parámetro del servidor
`max_valid_range_m` figura como `1.2 m` por el ajuste previo de runtime.

## Parámetros y comportamiento dinámico

Los parámetros de ROI son exactamente `enable_crop`, `min_x`, `max_x`,
`min_y`, `max_y`, `min_z` y `max_z`; el descriptor ROS los expone como
`boolean`/`double`, sin una restricción declarada de solo lectura.

Sin embargo, el nodo declara esos parámetros una vez en su constructor y copia
sus valores a miembros privados (`enable_crop_`, `min_x_`, etc.). No registra
un callback `on_set_parameters_callback` ni relee los parámetros dentro de
`cloud_callback`. En consecuencia, el servicio de parámetros puede aceptar una
escritura, pero ésta **no cambia el CropBox efectivo del proceso ya iniciado**.
Un ROI efectivo requiere valores al arranque/reinicio de una futura sesión de
sombra; esa acción no se realizó aquí.

## Geometría revisada

El self-filter usa seis cápsulas entre los frames JACO2 consecutivos, con
radios conservadores `[0.105, 0.105, 0.085, 0.075, 0.085, 0.090]` m. El URDF
incluye J1, J4--J6 continuos y J2/J3 con límites de `47°..313°` y
`19°..341°`, respectivamente. Por tanto, la envolvente de una sola postura no
representa el volumen de trabajo completo.

La muestra actual de `/joint_states` estaba inmóvil y produjo esta envolvente
de cápsulas en `world` (extremos de segmento inflados por su radio):

| Fuente geométrica | x mín..máx (m) | y mín..máx (m) | z mín..máx (m) |
|---|---:|---:|---:|
| Cápsulas JACO2, postura observada | -0.376772 .. 0.293186 | -0.106597 .. 0.109647 | -0.105000 .. 0.671332 |
| Nube filtrada | no medido | no medido | no medido |
| Grupo externo rojo | no medido; solo confirmado visualmente como lejano | no medido | no medido |

La ejecución actual no publica bounds ni centroides de `PointCloud2`.
Obtenerlos exigiría un suscriptor/analizador adicional o una herramienta de
medición no presente, que queda fuera de esta inspección solo observacional.

Como referencia de cobertura, los escenarios existentes sitúan una esfera de
radio efectivo `0.12 m` en `world`: el perfil de cápsulas conserva el escenario
base `(0.60, 0.0, 0.65)` y el YAML de escenarios conserva `PARK=(1.22, 0.0,
0.60)`. La cifra de aproximadamente `0.8 m` documentada para PARK es una
**separación cápsula--obstáculo**, no un límite espacial del ROI. Un ROI que
excluya ese volumen invalidaría escenarios de obstáculo cercanos/permitidos.

## Propuesta y riesgo

Existe soporte arquitectónico para un ROI rectangular en `world`, pero no hay
un `ROI candidato` numérico seguro todavía. La expresión que deberá evaluarse
en una sesión futura es:

`ROI_prueba = envolvente de cápsulas para las posturas autorizadas ∪ volumen de obstáculos válido ∪ margen de percepción`.

La propuesta temporal debe cubrir, como mínimo, la envolvente de todas las
posturas que se vayan a observar, el volumen de PARK y demás obstáculos
permitidos, más margen para radio de obstáculo, voxelización, error de
extrínseca y desplazamiento humano. El ROI final no debe fijarse hasta definir
el soporte, el volumen operativo y la zona de inserción/manipulación. Reducir
por intuición X/Y/Z podría ocultar un obstáculo real cerca del borde, por lo
que no es aceptable para seguridad.

## Dato faltante y validación mínima futura

Faltan: (1) bounds y centroide en `world` del fondo/grupo rojo en varias
muestras, (2) bounds del brazo para el conjunto de posturas autorizadas, y (3)
límites físicos del soporte, mesa y volumen operativo de los objetos.

Cuando se autorice una futura sesión de sombra aislada, el plan mínimo es:

1. Capturar métricas pasivas de bounds/centroides antes y después de un ROI de
   prueba, sin activar extracción ni control.
2. Verificar que todas las cápsulas y los escenarios de obstáculo autorizados
   permanecen dentro del ROI con margen documentado.
3. Revisar RViz para confirmar exclusión del fondo y conservación de brazo y
   obstáculos; sólo entonces valorar una configuración temporal, nunca una
   entrada de seguridad.

## Alcance preservado

No se cambiaron parámetros, `enable_crop`, rango, epsilon, TF ni extrínseca.
No se editó código, YAML, launch, CMake ni RViz; no se cerró o reinició la
sesión. No hubo comandos físicos ni inicio de supervisor, bridge, extractor,
GUI física, Gazebo o nodos de mando.
