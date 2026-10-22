# Fase 8A: viabilidad y núcleo offline de malla JACO2

## Dictamen

`SHADOW_MESH_CORE_READY`

El Xacro efectivo es `src/thesis_description/urdf/j2n6s300.xacro`; incluye `kinova_common.xacro`, donde visual y collision usan la misma URI DAE `package://thesis_description/meshes/${link_mesh}.dae`. Se verificaron mallas DAE legibles para `base`, `shoulder`, `arm`, `forearm`, `wrist`, `hand_3finger` y anillos; los seis links principales tienen collision mesh real.

| Link | Collision/visual | URI/formato | Disponible |
|---|---|---|---|
| base | ambos | `base.dae` | sí |
| link 1 | ambos + anillo | `shoulder.dae`, `ring_big.dae` | sí |
| link 2 | ambos + anillo | `arm.dae`, `ring_big.dae` | sí |
| link 3 | ambos + anillo | `forearm.dae`, `ring_big.dae` | sí |
| link 4/5 | ambos + anillo | `wrist.dae`, `ring_small.dae` | sí |
| link 6 | ambos + anillo | `hand_3finger.dae`, `ring_small.dae` | sí |

Assimp y FCL están instalados y sus cabeceras son accesibles. No se declaran ni usan aún: el núcleo puro recibe triángulos ya transformados a `world`, por lo que no introduce loader ni parser DAE en esta fase.

Se crearon `self_mesh_geometry.hpp`, `self_mesh_geometry.cpp` y `test_self_mesh_geometry.cpp`; CMake sólo añadió la librería y prueba. El núcleo usa Möller–Trumbore de doble cara, elige la primera intersección positiva, conserva índice de link, calcula `delta_ray=r-s`, valida datos finitos, degenerados, malla vacía y rayo nulo, y expone clasificación experimental. Su provenance siempre contiene `session_only=true`, `not_for_safety=true`, `geometry_kind=collision_mesh`. No contiene ROS, TF, nodos, launch, YAML operativo ni acceso a hardware.

La prueba cubre primera intersección/link, dos triángulos, transformación, antes/sobre/detrás con epsilon, sin intersección, degenerado, vacío y provenance. Para ~1300 puntos el coste inicial es O(puntos×triángulos); el siguiente paso necesitará BVH/AABB por malla, tras transformar una vez por link.

Validación final: `colcon build --packages-select thesis_jaco_perception --symlink-install` correcto; `ctest -R test_self_mesh_geometry --output-on-failure`: 1/1 pasó; `git diff --check` correcto. La primera prueba falló sólo por dos macros GoogleTest en una línea; se corrigió formato y la ejecución final pasó.

No se inició ROS, cámara, JACO2, RViz, TF, Gazebo ni launch. La sesión coverage_v1 siguió intacta y no hubo comandos físicos. El núcleo no está conectado al self-filter ni a seguridad.
