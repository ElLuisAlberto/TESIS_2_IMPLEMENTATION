# Fase 8S — prevalidación offline de alineación silueta-malla

## Dictamen único: `BLOCKED_IMPLEMENTATION`

La captura 8R permite proyectar las mallas en RGB con la pose óptica congelada, pero no permite reconstruir la pose inicial solicitada de `T_world_d435i_link`. Faltan en los artefactos congelados tanto `T_world_d435i_link` como la transformación fija `T_d435i_link_d435i_color_optical_frame`. No se puede convertir una mejora de la pose óptica en una corrección de `d435i_link` identificable y verificable. Por ello no se inició la búsqueda, no hay hipótesis de transformación aplicable y no se emitió YAML candidato. Este bloqueo es de datos congelados, no un juicio sobre la calidad visual de una solución optimizada.

Con la convención `p_A = T_A_B p_B`, la proyección disponible es `p_color = T_color_world p_world`, seguida por `K` y `D` de `CameraInfo`. Se comprobó que `T_color_world = inverse(T_world_color)` dentro de un error máximo de `2.22e-16` por entrada. Para reconstruir la pose pedida haría falta `T_world_link = T_world_color × inverse(T_link_color)`. La segunda matriz no fue congelada. Para cualquier `T_link_color` rígida supuesta existe una `T_world_link` distinta que reproduce exactamente la misma pose óptica; escoger una sería introducir información externa prohibida.

Las matrices de cámara efectivamente usadas, en orden de filas `[R|t]`, son:

```text
T_world_color_optical =
  [ 0.999856877 -0.014383917 -0.008906653  0.015051768 ]
  [ 0.011231865  0.170665903  0.985264936 -0.680317940 ]
  [-0.012651907 -0.985223961  0.170803035  0.299889268 ]
  [ 0           0            0            1           ]

T_color_optical_world =
  [ 0.999856877  0.011231865 -0.012651907 -0.003614203 ]
  [-0.014383917  0.170665903 -0.985223961  0.411781671 ]
  [-0.008906653  0.985264936  0.170803035  0.619205475 ]
  [ 0            0            0            1           ]
```

Se verificó el hash del XML (`b2d9d0d5f1366915ebe70583b8ebb048a7c445b97353b6b9424148f5644352d2`) y de las ocho mallas congeladas. RGB, `CameraInfo`, nube, los TF ópticos y los 13 TF de links corresponden exactamente al sello `1792639154117031982` ns; las diferencias de RGB y `CameraInfo` respecto de la nube son `0` ns. La descripción congelada del robot no contiene la cadena de la cámara.

Se prepararon bordes observados sin crear una máscara de brazo: Canny RGB 80/160 en píxeles con soporte de profundidad (`6151` píxeles) y discontinuidades de profundidad de la nube proyectada (`5760` píxeles), con umbral de salto de `0.08 m`. El mapa combinado conserva RGB en rojo y profundidad en azul. La nube proyecta `1211` puntos dentro de la imagen. El soporte de profundidad procede de puntos dispersos interpolados hasta seis píxeles; no acredita por sí mismo que un borde pertenezca al JACO2. El overlay inicial dibuja el contorno de la silueta 8R congelada sobre RGB, sin ajustar pose.

Los límites previstos para una eventual búsqueda eran `±0.30 m` y `±25°` por eje, tal como exige la fase: acotan una hipótesis local y excluyen una solución que termine en el límite. Ninguna inicialización se ejecutó, pues falta la pose inicial en el frame requerido.

| Inicialización | Coste inicial | Coste final | Convergencia | Límite |
|---|---:|---:|---|---|
| Ninguna: falta `T_world_d435i_link` | — | — | No ejecutada | No evaluado |

La mejora de coste y de cobertura por brazo superior, antebrazo y muñeca/efector queda **no evaluada**. No se puede verificar el umbral de mejora del 40 %, la convergencia de tres inicializaciones, la cobertura del 65 %, ni la consistencia visual regional. Tampoco hay incertidumbre o transformación candidata que reportar.

Los artefactos están en `evidence/codex_reports/2026-10-22_fase8s_silhouette_alignment/`: `initial_mesh_overlay.png`, `rgb_edges.png`, `depth_edges.png`, `combined_edges.png`, `optimization_runs.json` (lista vacía con motivo) y `candidate_metrics.json` (integridad, matrices, conteos y bloqueo). No existe `optimized_mesh_overlay.png` porque no hubo optimización; no existe `candidate_transform_not_applied.yaml` porque ninguna candidatura pasó aceptación.

La prueba offline de composición con un punto sintético y la comprobación de la cadena congelada pasaron (`2/2`). No se añadió código compilado, así que no correspondió ejecutar un build. `git diff --check` y la comprobación de espacios de los archivos nuevos finalizaron sin errores. No se inició ROS ni una captura nueva; no se modificó la sesión, JACO2, TF, YAML operativo, extrínseca, filtros, ROI, epsilon, radios ni mallas. Para desbloquear la fase haría falta una captura congelada que incluya `T_world_d435i_link` y `T_d435i_link_d435i_color_optical_frame` al sello de la imagen, o una cadena estática equivalente verificable, antes de estimar una corrección de ese link.
