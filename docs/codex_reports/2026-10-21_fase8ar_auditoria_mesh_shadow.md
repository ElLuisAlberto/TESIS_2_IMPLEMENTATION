# Fase 8A-R: auditoría técnica del núcleo self-mesh offline

Fecha: 2026-10-21. Sólo se leyeron la evidencia Fase 8A, los tres archivos del núcleo, CMake/package y Git. Por la restricción de insumos, no se abrieron directamente URDF/Xacro/DAE.

## Dictamen

`MESH_SHADOW_NODE_REQUIRES_BVH_OR_LOADER`

El núcleo intersecta correctamente los triángulos que recibe y Fase 8A declara mallas collision reales accesibles. Pero no carga URDF/Xacro/DAE, no resuelve URI ni asocia nombres de link a TF. Las pruebas inyectan sólo triángulos sintéticos. Antes de comparar en sombra requiere loader real y BVH/AABB. Nunca es de seguridad: su provenance fija `not_for_safety=true`.

## 1. Geometría real

Inventario declarado por el informe Fase 8A (no reabierto en esta fase). El código no consume aún ninguna de estas mallas; recibe `MeshTriangle` ya en `world`. Los triángulos de las pruebas son planos sintéticos con índices 7 y 9. “Apto para sombra” sólo significa candidato tras implementar el loader.

| Link/grupo | Fuente declarada | collision o visual | URI declarada | Formato | Existe (Fase 8A) | Triángulos | Sombra | Seguridad |
|---|---|---|---|---|---|---|---|---|
| base | `base` | ambos; misma malla | `package://thesis_description/meshes/base.dae` | DAE | sí | no medido | sí, pendiente loader | no |
| link 1 | `shoulder` + `ring_big` | ambos + anillo | `package://thesis_description/meshes/shoulder.dae`, `ring_big.dae` | DAE | sí | no medido | sí, pendiente loader | no |
| link 2 | `arm` + `ring_big` | ambos + anillo | `package://thesis_description/meshes/arm.dae`, `ring_big.dae` | DAE | sí | no medido | sí, pendiente loader | no |
| link 3 | `forearm` + `ring_big` | ambos + anillo | `package://thesis_description/meshes/forearm.dae`, `ring_big.dae` | DAE | sí | no medido | sí, pendiente loader | no |
| links 4-5 | `wrist` + `ring_small` | ambos + anillo | `package://thesis_description/meshes/wrist.dae`, `ring_small.dae` | DAE | sí | no medido | sí, pendiente loader | no |
| link 6 | `hand_3finger` + `ring_small` | ambos + anillo | `package://thesis_description/meshes/hand_3finger.dae`, `ring_small.dae` | DAE | sí | no medido | sí, pendiente loader | no |

Fase 8A afirma que visual y collision comparten URI. No hay malla sólo visual declarada; si se eligiera una visual distinta, sería sólo de sombra y nunca de seguridad. No debe confundirse el núcleo de intersección correcto con un modelo físico JACO2 cargado: éste aún no existe en runtime.

## 2. Trazabilidad URDF

La evidencia autorizada de Fase 8A reporta el Xacro efectivo `src/thesis_description/urdf/j2n6s300.xacro`, que incluye `kinova_common.xacro`, y URI `package://thesis_description/meshes/${link_mesh}.dae`. No fue inspeccionado directamente en 8A-R. El núcleo no resuelve `package://`, `file://` ni rutas relativas; CMake/package tampoco declaran Assimp o FCL.

Aunque Fase 8A menciona seis links principales, no hay mapa frame/nombre/índice ni TF en el núcleo. Por tanto no se demuestra asociación de las seis cadenas cinemáticas con mallas reales. Tampoco se confirma cobertura de dedos, gripper, carcasas, base o enlaces críticos; sólo consta documentalmente `hand_3finger`, anillos y base. Es trazabilidad pendiente, no ausencia probada.

## 3. Matemática y convención

| Confirmación | Evidencia / límite |
|---|---|
| `p_world = T_world_link * p_link` | `transform_triangle` aplica una 3x4 fila-major `[R|t]`. Es esa ecuación sólo si el llamador entrega T_world_link y p_link; no hay tipos/verificación de frame. |
| Cámara y punto en world | se usa `observed-camera`; ambos y los triángulos deben estar en el mismo frame, pero el código no comprueba que sea world. |
| Primera positiva C→P | normaliza `(P-C)` y retiene el menor `s > 1e-12`. |
| `delta_ray = r-s` | se almacena exactamente así. |
| Doble cara | `abs(det)`: no hay backface culling. |
| Degenerados, NaN y negativos | rechaza triángulos degenerados/no finitos, cámara/punto no finitos, rayo nulo y `s <= 1e-12`; rechaza epsilon no finito/negativo. |

Ambigüedades no probadas: matriz no finita/no afín/no rígida, NaN producido tras transformar, unidades, orientación óptica y TF al sello. La última fila se ignora y `1e-12` no está justificado frente al sensor.

## 4. Rendimiento

No hay conteos DAE ni benchmark. El coste es O(P*T) y memoria O(T), P=puntos y T=triángulos ya transformados. Para ~1300 puntos: 1300*T intersecciones/nube; a 20-30 Hz: 26,000*T a 39,000*T/s. T=10,000 implica 26-39 millones/s y T=100,000 implica 260-390 millones/s, sin TF, carga, transformación, copias o publicación. No se puede afirmar tiempo real. Deben cachearse mallas, transformarse una vez por link/sello y añadirse BVH/AABB o reducción de malla antes de runtime.

## 5. Aislamiento

El núcleo sólo usa C++ estándar: no contiene rclcpp, TF, publicadores, servicios ni hardware. CMake crea `self_mesh_geometry` sin enlazarlo a ejecutables; sólo la prueba lo enlaza. No existe launch/nodo mesh. Con los archivos permitidos no está conectado al self-filter actual, supervisor, bridge, extractor, `obstacle_input` o control, ni altera coverage_v1/v2.

## 6. Validación disponible

Sin reejecutar nada, Fase 8A registra build correcto y `ctest -R test_self_mesh_geometry --output-on-failure`: **1/1 CTest aprobado, 0 fallos, 0 errores**. El binario contiene cuatro casos:

| Caso | Demuestra |
|---|---|
| `FirstAndLink` | primer hit y conservación de índice. |
| `TransformAndClasses` | traslación simple y antes/sobre/detrás con epsilon. |
| `Rejects` | vacío, degenerado y ausencia de hit. |
| `Provenance` | textos de provenance. |

No demuestra carga/URI de mallas reales, TF runtime, cámara, sincronía, rendimiento, completitud física ni separación frente a obstáculos reales.

## 7. Riesgos/bloqueos

- Falta loader DAE/URDF y mapa mesh-link-frame verificable.
- Falta BVH/AABB, conteo de triángulos y benchmark a 20-30 Hz.
- Frames implícitos y sin TF sellado: una convención invertida invalida el resultado.
- Cobertura de dedos, gripper, carcasas y enlaces críticos no verificada.
- `not_for_safety=true`: no sustituye el filtro de cápsulas ni realiza seguridad.

## 8. Siguiente fase mínima

1. Conservar filtro de cápsulas y `hardware_output=false`.
2. Cargar offline mallas collision reales, resolver URI y mapear mesh-link-frame.
3. Medir T, cachear y añadir BVH/AABB o reducción.
4. Crear nodo de comparación sólo sombra con TF exacto al sello.
5. Publicar tópicos nuevos separados; nunca sustituir salidas ni conectar control.
6. Validar brazo inmóvil y después caja a 0.8 m, siempre en sombra.

## Observación Git y garantías

`git diff --check` terminó sin salida. `git status --short` mostró cambios y no rastreables preexistentes, incluidos artefactos Fase 8A; esta auditoría no los modificó. La única salida nueva es este informe. No se cerró, reinició ni modificó coverage_v1; no se inició ROS, launch, D435i, JACO2, RViz, TF, Gazebo, sondas, build o tests, y no hubo comandos, trayectorias, acciones, servicios ni hardware.
