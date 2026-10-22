# Fase 8D: auditoría y corrección exacta de BVH

## Dictamen

`MESH_SHADOW_BVH_PERFORMANCE_READY`

El dictamen se limita a viabilidad computacional offline de las consultas mesh-shadow; no autoriza nodo ROS, cámara, hardware ni uso de seguridad.

## Causa raíz demostrada

La construcción anterior guardaba `auto& n = m->bvh[id]` y hacía llamadas recursivas que ejecutaban `push_back` sobre el mismo vector. Una realocación invalidaba `n`; las escrituras posteriores de `left`, `right` y `count=0` eran comportamiento indefinido. Como resultado, nodos raíz podían conservar `count` completo y recorrerse como una hoja: Fase 8C registró exactamente un nodo por link BVH y ~12,800 triángulos por consulta.

La corrección no conserva referencias a elementos de `bvh` a través de recursión. Conserva el índice estable `id`, crea ambos hijos y escribe sus índices y `count=0` mediante `m->bvh[id]` después de volver. El desempate de centroides iguales usa índice de triángulo, por lo que sigue dividiendo determinísticamente hasta ocho triángulos por hoja.

## Contrato estructural comprobado

Cada nodo es hoja con 1--8 índices y sin hijos, o interno con dos hijos válidos y cero índices directos. El test recorre cada BVH, verifica cada hijo, suma hojas, inserta cada índice en un conjunto sin repetición y exige que la cobertura sea igual al total de triángulos.

| Grupo | nodos por malla | hojas por malla | profundidad | máximo hoja |
|---|---:|---:|---:|---:|
| base (17068 triángulos) | 5463 | 2732 | 12 | 8 |
| mano (27908) | 8191 | 4096 | 12 | 7 |
| links 1--3 | 2047--2095 | 1024--1048 | 10--11 | 7--8 |
| links 4--5 | 1023 | 512 | 9 | 8 |
| dedos proximales | 1023 | 512 | 9 | 6 |
| dedos distales | 511 | 256 | 8 | 8 |

Las 13 instancias collision, 89,728 triángulos, cumplen nodos>1, hojas>1 y profundidad>=2. Se mantienen Assimp, URDF, URI y mallas exactamente iguales.

## Recorrido y equivalencia

El recorrido prueba AABB de nodo, cuenta nodo/AABB, evalúa hojas e intersecciones de triángulo. La capa world actualiza el AABB de link transformando sólo ocho esquinas, y descarta links fuera del segmento `[0,r+epsilon]`; no transforma triángulos por rayo ni reconstruye la BVH. El orden de pila actual conserva exactitud; la poda válida procede del mejor hit de AABB local. No hay backface culling ni cambio de epsilon.

Fuerza bruta se conserva. Las pruebas comparan hit/no-hit, link y distancia <=1e-9 m, y clase EXTERNAL/SELF/OCCLUDED con epsilon 0.07, con posturas rígidas no identidad, fondo, AABB sin triángulo, dos links alineados y hits antes/dentro/después de epsilon. Se compararon los 3,900 rayos de benchmark sin divergencias.

## Benchmark offline, 13 instancias y 1300 rayos por escenario

Compilador `/usr/bin/c++`; CPU disponible: 16 lógicas. Normal: `CMAKE_BUILD_TYPE` vacío. Release temporal aislado: `-DCMAKE_BUILD_TYPE=Release`, flags `-O3 -DNDEBUG`; no modificó build/install normales.

| Escenario | Normal ms / us-rayo | Release ms / us-rayo | links BVH | nodos / hojas / triángulos | hits |
|---|---:|---:|---:|---:|---:|
| fondo | 0.681 / 0.524 | 0.185 / 0.142 | 0 | 0 / 0 / 0 | 0 |
| cercanos | 8.545 / 6.573 | 2.023 / 1.556 | 1287 | 45951 / 3399 / 23322 | 529 |
| mixto (250 cercanos) | 1.846 / 1.420 | 0.622 / 0.479 | 242 | 8358 / 615 / 4228 | 99 |

En normal, los AABB de link evaluados/descartados fueron 16900/16900 (fondo), 16900/15613 (cercanos) y 16900/16658 (mixto). La actualización de 13 AABB world fue 0.020814 ms normal y 0.005257 ms Release. Cada contador de nodos corresponde a AABB de nodo probados; hojas indica hojas atravesadas; triángulos indica pruebas reales de Möller--Trumbore.

Las dos configuraciones cumplen el objetivo <=50 ms y el umbral <=100 ms para los tres escenarios offline. No es evidencia de 20--30 Hz con D435i: faltan TF al sello, sincronía, copia/procesamiento de nube, ROS y validación física.

## Validación y aislamiento

Build normal y `test_self_mesh_geometry`: 100 pruebas agregadas, 0 errores, 0 fallos. Build Release temporal y prueba equivalente: 11 pruebas del árbol temporal, 0 errores, 0 fallos. `git diff --check` limpio.

No hubo LOD, decimación, voxelización, GPU, aproximación, cambio de geometría o epsilon. No se tocó `coverage_v1`; no se inició ROS graph, launch, D435i, JACO2, RViz, TF, Gazebo ni sondas, y no se enviaron comandos al hardware. No se creó nodo ROS ni launch.
