# Fase 8T — cadena TF completa y alineación extrínseca offline

## Dictamen único: `EXTRINSIC_CANDIDATE_NOT_TRUSTWORTHY`

La cadena TF que faltaba en 8S quedó congelada y es coherente. La mejor hipótesis hallada offline **no** supera la aceptación: el coste a 640×480 baja sólo 33,5 % (mínimo 40 %), las inicializaciones no convergen a una misma pose, ninguna de las tres regiones exigidas mejora con cobertura conservada y el overlay termina sobre fondo, silla y el robot rojo en vez de seguir el JACO2 negro. No existe una pose candidata aplicable ni se creó `candidate_transform_not_applied.yaml`.

## Captura única e integridad

Se amplió el mismo cliente observacional `mesh_arm_snapshot_once.cpp` de 8R con el modo `--tf-complete`; no se creó un segundo nodo de cámara. Hubo **una** ejecución runtime, sin reintento. La terna RGB, CameraInfo y nube filtrada coincidió exactamente en `1792640495707376953 ns` (ambas diferencias `0 ns`, límite `33 ms`). La nube contiene 1244 puntos XYZ en `world`; los 1244 proyectan a RGB. Los TF se consultaron al sello exacto de nube, nunca con `latest`. Los TF devueltos, incluso el enlace fijo óptico, llevan ese sello.

| TF congelado (`T_destino_origen`) | Destino ← origen | Sello devuelto (ns) | Resultado |
|---|---|---:|---|
| `T_world_d435i_link` | `world ← d435i_link` | 1792640495707376953 | rígido |
| `T_d435i_link_color` | `d435i_link ← d435i_color_optical_frame` | 1792640495707376953 | rígido |
| `T_world_color` | `world ← d435i_color_optical_frame` | 1792640495707376953 | rígido |
| `T_color_world` | `d435i_color_optical_frame ← world` | 1792640495707376953 | inversa verificada |
| `T_color_d435i_link` | `d435i_color_optical_frame ← d435i_link` | 1792640495707376953 | inversa verificada |
| `T_d435i_link_world` | `d435i_link ← world` | 1792640495707376953 | inversa verificada |
| `T_world_depth` | `world ← d435i_depth_optical_frame` | 1792640495707376953 | rígido |
| 13 × `T_world_collision_link` | `world ←` cada link de colisión | 1792640495707376953 | 13/13 rígidos |

La pose inicial de `d435i_link` tiene traslación `[0.03, -0.68, 0.30] m` y cuaternión `xyzw=[0.0616284167, -0.0616284167, 0.7044160264, 0.7044160264]`. El enlace fijo a color tiene traslación `[-0.0003323380, 0.0149482321, -0.0000538399] m` y cuaternión `xyzw=[-0.4993399737, 0.5020958518, -0.4934243934, 0.5050660494]`. La pose óptica directa tiene traslación `[0.0150517679, -0.6803179399, 0.2998892681] m` y cuaternión `xyzw=[-0.6438920598, 0.0012238279, 0.0083704094, 0.7650695745]`. Entre `T_world_color` directo y `T_world_d435i_link × T_d435i_link_color` hay error de traslación `3.47e-18 m` y rotación `1.51e-16 rad`, muy por debajo de `1e-6` en ambas magnitudes. También pasaron las tres comprobaciones de inversa.

La convención es `p_A = T_A_B × p_B`. Se optimizó únicamente una perturbación rígida izquierda de seis parámetros: `T_world_d435i_link_candidato = Delta_T_world × T_world_d435i_link_inicial`; para cada render, `T_world_color_candidato = T_world_d435i_link_candidato × T_d435i_link_color` y `p_color = inverse(T_world_color_candidato) × p_world`. Las intrínsecas `K` congeladas hacen la proyección. La distorsión congelada `D` es cero, por lo que el render pinhole es válido para esta captura.

El XML y las ocho mallas collision se copiaron, sin cambiar bytes, de 8R. Antes de cargar triángulos se verificaron sus SHA-256, incluido el XML `b2d9d0d5f1366915ebe70583b8ebb048a7c445b97353b6b9424148f5644352d2`. `capture_hashes.json` fija además los hashes de RGB, nube, metadata, manifest de TF y geometría (15 archivos); la verificación offline volvió a pasar. La geometría conserva `collision origin`, escala y TF de los 13 links al sello nuevo.

## Ajuste por bordes de profundidad

Se proyectó la nube congelada y se derivaron 5775 píxeles de borde: contorno del soporte de profundidad y discontinuidades mayores de `0.08 m`. No se usó RGB en el coste, máscara automática ni correspondencia punto-triángulo. El renderizador de triángulos usa z-buffer con profundidad perspectivamente correcta y etiqueta de link visible. El coste toma la distancia del contorno renderizado a los bordes depth observados sólo donde hay soporte, con pérdida Huber de transición a `4 px`; penaliza además contorno sin soporte. El porcentaje de cobertura cuenta contorno válido a `≤5 px` de un borde. Este porcentaje condicionado al soporte puede ser alto aunque partes grandes de la malla queden fuera del brazo; por eso la inspección del overlay y la cobertura por región son condiciones independientes.

Se usaron límites por eje `±0.30 m` y `±25°` sobre `Delta_T_world`, cuatro inicializaciones deterministas a 160×120, refinamiento de las dos mejores según coste reducido a 320×240 y validación **de las seis hipótesis** a 640×480. Los costes de distintas resoluciones no se compararon entre sí para elegir la mejor hipótesis final. Ningún resultado terminó en un límite. `Convergió` indica el criterio numérico de Powell; un sí no implica aceptación física.

| Resolución | Inicio | Coste inicio → final local | Coste a 640×480 | Cobertura a 640×480 | Convergió | Límite |
|---|---:|---:|---:|---:|---|---|
| 160×120 | 0 | 11.71 → 9.42 | 24.07 | 50.8 % | sí | no |
| 160×120 | 1 | 10.66 → 8.60 | **17.56** | **65.3 %** | sí | no |
| 160×120 | 2 | 9.54 → 7.19 | 25.93 | 40.1 % | sí | no |
| 160×120 | 3 | 10.73 → 7.96 | 23.35 | 43.1 % | sí | no |
| 320×240 | 2 | 12.30 → 10.32 | 20.35 | 49.6 % | no (presupuesto) | no |
| 320×240 | 3 | 11.79 → 12.85 | 23.52 | 51.0 % | sí | no |

`optimization_runs.json` conserva para cada corrida las seis coordenadas iniciales/finales, ambas matrices `T_world_d435i_link`, costes, evaluaciones, convergencia, límite y cobertura. Entre las cuatro soluciones de baja resolución, la menor separación entre pares es `0.181 m` en traslación y `7.0°` en rotación; no existe un grupo de tres dentro de `0.02 m` y `2°`.

| Métrica a 640×480 | Inicial | Mejor hipótesis | Criterio |
|---|---:|---:|---|
| Coste robusto | 26.415 | 17.560 | Mejora 33.5 %, falla ≥40 % |
| Cobertura de contorno con soporte | 51.3 % | 65.3 % | Supera ≥65 %, condicionado al soporte |
| Brazo superior: borde visible / distancia media | 114 px / 11.79 px | 60 px / 2.66 px | Falla: perdió 47 % del contorno |
| Antebrazo: borde visible / distancia media | 205 px / 3.38 px | 107 px / 4.38 px | Falla: empeoró y perdió 48 % |
| Muñeca/efector: borde visible / distancia media | 207 px / 8.14 px | 53 px / 6.07 px | Falla: perdió 74 % del contorno |

La aparente reducción de distancia en brazo superior y muñeca/efector proviene de conservar sólo un tramo pequeño cerca de bordes observados. El overlay final muestra el resto de contornos desplazados hacia mobiliario y el robot rojo. Por ello `visual_overlay_verified=false`; no se atribuye este mínimo a la extrínseca real. No se reporta pose candidata ni incertidumbre como corrección admisible. Las mejores hipótesis permanecen sólo como datos diagnósticos en JSON.

## Artefactos y validación

Todos los artefactos 8T están en `evidence/codex_reports/2026-10-22_fase8t_tf_complete_snapshot/`: `rgb.png`, `cloud_world_xyz.npz`, `metadata.json`, `link_transforms_at_stamp.json`, `robot_description.urdf`, `collision_mesh_manifest.json`, `hashes.json`, `capture_hashes.json`, `meshes/`, `depth_edges.png`, `initial_mesh_overlay.png`, `optimized_mesh_overlay.png`, `optimization_runs.json` y `candidate_metrics.json`. Los overlays son diagnósticos; no hay YAML candidato.

El cliente C++ ampliado compiló manualmente; `colcon build --packages-select thesis_jaco_perception` terminó con un paquete correcto. El renderizador offline compiló con `g++`; las pruebas nuevas cubren composición e inversión, perturbación conocida, rechazo de TF inválido, z-buffer y coste. Las pruebas nuevas y afectadas pasaron `9/9`; `git diff --check` y la comprobación de espacios de los archivos nuevos no encontraron errores.

No se cerró ni reinició la sesión, no se inició otro launch, cámara o RViz, no se movió ni armó el JACO2 y no se enviaron comandos. `hardware_output=false` permaneció en el launch observado. No se publicaron TF, tópicos, servicios ni parámetros; no se alteraron extrínseca, YAML operativo, ROI, epsilon, radios, mallas, URDF/Xacro ni filtros. La hipótesis no fue aplicada ni cargada por ningún proceso.
