# Fase 8C: optimización exacta BVH mesh-shadow

## Dictamen

`MESH_SHADOW_BVH_OPTIMIZATION_INSUFFICIENT`

La optimización es exacta frente a fuerza bruta en los escenarios ejecutados, pero el escenario mixto de 1,300 rayos tarda 182.108 ms: supera 100 ms y no habilita aún integración experimental.

## Cambios

Se conservaron Assimp, URDF collision, URI `package://`, triángulos y BVH local de 8B. Se añadió `MeshWorldInstance`, que almacena por link el puntero a malla, `T_world_link` externo y AABB world. La actualización transforma solamente las ocho esquinas del AABB raíz local; no transforma triángulos ni reconstruye BVH. Se añadió ruta explícita `first_world_mesh_intersection_brute_force` como referencia y `first_world_mesh_intersection_optimized` con perfil C++.

Para cada consulta se calcula `r=||P-C||`, se normaliza C→P y cada AABB world se prueba en el segmento `[0,r+epsilon]`. Un AABB engloba todos los triángulos de su instancia bajo una transformación rígida; por ello, si el segmento no toca el AABB, ningún triángulo de esa instancia puede ser primer hit en el intervalo. Las instancias restantes convierten C y P al link mediante la inversa rígida y consultan BVH local. No hay backface culling, LOD, decimación, cambio de malla, aproximación ni cambio de epsilon.

## Equivalencia

La prueba usa las 13 instancias reales (89,728 triángulos), posturas rígidas no identidad y epsilon 0.07. Compara fuerza bruta y ruta optimizada en casos de fondo, cruce de AABB sin triángulo, dos links sobre el rayo, hits antes/dentro/después de epsilon, y los 3,900 rayos de benchmark. Para cada rayo exige hit/no-hit idéntico; para hits exige mismo link y distancia con tolerancia 1e-9 m. También compara la clase derivada EXTERNAL/SELF/OCCLUDED. No hubo divergencias.

## Benchmark offline reproducible

Compilador: `/usr/bin/c++`; `CMAKE_BUILD_TYPE` vacío (configuración por defecto de colcon); CPU disponible: 16 lógicas. La medición es offline, sin D435i/TF/runtime.

| Escenario, 1300 rayos | Brute-force ms | Optimizado ms | Speedup | Hits | AABB evaluados / descartados | Links BVH | Nodos BVH | Triángulos |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| fondo (0 cercanos) | 11019.4 | 0.534109 | 20631.4x | 0 | 16900 / 16900 | 0 | 0 | 0 |
| cercanos (1300) | 10815.6 | 835.779 | 12.9408x | 529 | 16900 / 15613 | 1287 | 1287 | 16506474 |
| mixto (250 cercanos) | 11607.8 | 182.108 | 63.7416x | 99 | 16900 / 16658 | 242 | 242 | 3127850 |

La actualización de los 13 AABB world tomó 0.017961 ms. El contador de rayos es 1,300 por escenario. El tiempo de consulta es la columna optimizada; el perfil no registra publicación, TF, cámara, copias de nube ni sincronía.

Comparación: fondo cumple ampliamente 50 ms y 100 ms; cercanos y mixto no cumplen ninguno. Por tanto el buen speedup no basta para afirmar 20--30 Hz ni autorizar una futura integración experimental. El coste restante se concentra en rayos que alcanzan AABB y atraviesan BVH de mallas densas, especialmente mano/dedos.

## Validación y aislamiento

Se ejecutaron únicamente `colcon build --packages-select thesis_jaco_perception --symlink-install`, `colcon test --packages-select thesis_jaco_perception --ctest-args -R test_self_mesh_geometry`, `colcon test-result --test-result-base build/thesis_jaco_perception --verbose` y `git diff --check`. Resultado final: 1/1 CTest de mesh, 9 GoogleTest en el ejecutable; agregado 99 pruebas, 0 errores y 0 fallos; diff limpio.

No se modificó `coverage_v1`; no se inició ROS graph, D435i, JACO2, RViz, TF, Gazebo, launch ni sondas. No hubo hardware, LOD, GPU, nodo ROS, launch, tópico, YAML ni conexión de seguridad.

## Limitaciones restantes

La optimización debe mejorar aún rayos cercanos (orden de visita BVH, poda por distancia y/o estructura superior más selectiva), siempre manteniendo equivalencia exacta. También faltan TF sellado, sincronía, cámara, validación con obstáculos reales y cualquier decisión de seguridad.
