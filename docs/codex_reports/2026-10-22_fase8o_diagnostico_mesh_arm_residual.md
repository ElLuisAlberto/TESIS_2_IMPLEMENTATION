# Fase 8O — diagnóstico cuantitativo del residual mesh-shadow

## Dictamen único: `INSUFFICIENT_OBSERVABILITY`

La nube magenta visible sobre el brazo persiste. La sonda encontró 612 primeras
intersecciones de rayo con mallas de colisión en tres nubes, pero sus residuos no
tienen signo ni magnitud semejantes entre links. Tampoco se concentran de forma
inequívoca en sólo uno o dos links: hay valores de ambos signos en hombro y
muñeca, positivos en brazo y mano y negativos fuertes en antebrazo. Además,
3 140 de 3 752 puntos muestreados no intersectaron una malla; base y
`link_4` no tuvieron intersecciones. Una intersección por sí sola no demuestra
que el punto observado sea superficie del robot: puede ser una superficie del
fondo detrás de la malla. Por ello estos datos no permiten escoger con rigor
entre extrínseca global y discrepancia local, ni estimar una corrección de TF.

El siguiente dato mínimo sería una captura RGB/depth sincronizada con la nube
filtrada y la pose TF de **una misma marca de tiempo**, con una máscara de los
píxeles que pertenecen al brazo visible. Puede tomarse en la sesión actual y
con el brazo inmóvil. Permitirá calcular los residuos sólo sobre puntos
identificados como brazo, incluido el contorno de base, antebrazo, muñeca y
efector; no se requiere otra postura ni una nueva sesión de cámara.

## Sesión observada y configuración efectiva

La sesión activa era PID `110022`, dominio ROS `78`, con
`hardware_output:=false`, `start_jaco_read_only:=true`, D435i activa, ROI
temporal activo, filtro de cápsulas y mesh-shadow. En la lista ROS aparecieron
`/jaco_mesh_shadow_node`, `/jaco_self_filter_node`, `/camera/d435i`,
`/pointcloud_preprocessor`, `/d435i_session_candidate_extrinsic` y
`/robot_state_publisher`. Sus procesos fueron respectivamente PID `110060`,
`110062`, `110054`, `110056`, `110046` y `110048`. RViz PID `112320`
seguía abierto. El adaptador hardware PID `110050` tenía en sus argumentos
efectivos `hardware_output_enabled: false`.

El mesh node desactiva servicios de parámetros. Por ello sus valores efectivos
se corroboraron en su archivo de parámetros de launch
`/tmp/launch_params_1mvt_im_`, el comando del proceso y el código de
inicialización: `input_topic=/thesis/perception/points_filtered`, frame de
entrada exigido `world`, `camera_optical_frame=d435i_depth_optical_frame`,
`epsilon_mesh_total_m=0.07`, `robot_description` XML de 23 080 caracteres
con 13 mallas de colisión cargadas. El diagnóstico observado en ROS fue
literalmente `MESH_SHADOW_COMPLETE`, con `session_only=true`,
`not_for_safety=true` y `geometry_kind=collision_mesh`. La nube de entrada
tenía `frame_id=world`. El preprocesador mantenía `target_frame=world`, ROI
`x=[-0.55,1.50]`, `y=[-0.50,0.50]`, `z=[-0.25,1.10]` metros.

## Medición de rayo a malla

La sonda de cápsulas existente no mide mallas. Se creó
`src/thesis_jaco_perception/src/mesh_residual_probe_once.cpp` como fuente
temporal y se compiló de forma independiente contra el núcleo de mallas ya
existente, sin cambiar CMake. Tiene una suscripción de nube al tópico filtrado,
usa TF al sello exacto, no publica ni presta servicios, no declara parámetros,
no escribe y termina tras un máximo de tres nubes o cinco segundos. Se ejecutó
**una vez**. Las primeras nubes recibidas se descartaron mientras se poblaba el
buffer TF; no cuentan como muestras medidas. El algoritmo utilizó la primera
intersección real sobre el rayo en las 13 mallas y definió
`residual = distancia(cámara,punto) - distancia(cámara,primera malla)`.
Residual negativo significa que el punto está delante de la malla; positivo,
que está detrás. No se aplicó epsilon al cálculo de estos percentiles.

| Sello de nube, segundos ROS | Frame | Sello TF solicitado | Puntos | Intersecciones | Sin intersección |
|---|---|---|---:|---:|---:|
| 1792637222.972501709 | world | 1792637222.972501709 | 1 245 | 202 | 1 043 |
| 1792637223.005856934 | world | 1792637223.005856934 | 1 246 | 205 | 1 041 |
| 1792637223.072567383 | world | 1792637223.072567383 | 1 261 | 205 | 1 056 |

En cada sello se resolvieron `world -> d435i_depth_optical_frame` y los TF
`world -> link` de las 13 mallas; si alguno hubiese faltado, la sonda habría
descartado la nube entera. La tabla siguiente asocia cada punto al link de la
primera intersección, no a un link elegido por proximidad. Los percentiles son
de residuo **con signo**, en metros.

| Región / link / malla | n | negativos / positivos | P50 | P90 | P95 | P99 |
|---|---:|---:|---:|---:|---:|---:|
| Base / `link_base` / `base.dae` | 0 | — | — | — | — | — |
| Hombro / `link_1` / `shoulder.dae` | 78 | 63 / 15 | -0.006821 | +0.001452 | +0.003077 | +0.003973 |
| Brazo superior / `link_2` / `arm.dae` | 147 | 3 / 144 | +0.038258 | +0.054747 | +0.055338 | +0.057334 |
| Antebrazo / `link_3` / `forearm.dae` | 193 | 193 / 0 | -0.094902 | -0.048033 | -0.046382 | -0.042651 |
| Muñeca / `link_4` / `wrist.dae` | 0 | — | — | — | — | — |
| Muñeca / `link_5` / `wrist.dae` | 39 | 22 / 17 | -0.000776 | +0.007022 | +0.008443 | +0.008443 |
| Mano / `link_6` / `hand_3finger.dae` | 142 | 0 / 142 | +0.061025 | +0.070931 | +0.073735 | +0.077495 |
| Dedo proximal 2 / `finger_proximal.dae` | 10 | 0 / 10 | +0.079158 | +0.093380 | +0.094063 | +0.094609 |
| Dedo distal 2 / `finger_distal.dae` | 3 | 3 / 0 | -0.010568 | -0.010568 | -0.010568 | -0.010568 |
| Otros cuatro dedos y puntas | 0 | — | — | — | — | — |

La concentración de residuo negativo en `link_3` merece una comprobación
posterior, pero los signos opuestos de `link_2` y mano no prueban errores en
esas mallas: podrían ser puntos de fondo. Una rotación de extrínseca también
puede producir signos opuestos entre zonas en una sola postura. La captura
actual de RViz confirma puntos magenta en varias zonas del brazo, pero no
permite identificar qué puntos 3D de esta medición corresponden a esa
superficie. No hay base cuantitativa para proponer traslación/rotación ni para
editar `forearm.dae` o su TF.

## Cambios y validación

Único archivo de código añadido: sonda temporal de observación. No se aplicó
ninguna corrección de configuración, RViz, launch o geometría. No se enviaron
comandos al JACO2 ni se armó; `hardware_output=false` permaneció en la sesión.
No se cambiaron epsilon, ROI, mallas, radios ni extrínseca temporal o final.
La cámara, los filtros, RViz y el launch existentes no se reiniciaron.

La sonda compiló manualmente contra `self_mesh_geometry`. `colcon build
--packages-select thesis_jaco_perception` finalizó con un paquete correcto.
`test_self_mesh_geometry`: 10/10; `test_mesh_shadow_logic`: 2/2. La ejecución
única de la sonda produjo las tres nubes de la tabla. `git diff --check` no
reportó errores; también se comprobó el archivo nuevo con
`git diff --no-index --check /dev/null <sonda>` sin errores de whitespace.
