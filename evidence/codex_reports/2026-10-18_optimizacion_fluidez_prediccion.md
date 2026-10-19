# Optimización de fluidez de la predicción en simulación

Fecha: 2026-10-18
Clasificación: **PASS**

## Problema observado

El rosbag inicial mostró que la entrada de la GUI era regular a 20 Hz, pero el
supervisor entregaba comandos a 14.96 Hz y acumulaba pausas de hasta 303 ms.
La evaluación predictiva repetía cinemática y validaciones geométricas en cada
iteración de búsqueda de escala. La vista de RViz, además, usaba el reloj
simulado y por ello su frecuencia aparente descendía con el factor de tiempo
real de Gazebo.

## Corrección

- Se redujeron asignaciones en la cinemática directa y se reutilizaron las
  transformaciones constantes del JACO2.
- La evaluación del horizonte valida los datos una vez y usa una ruta interna
  prevalidada para sus 21 muestras y seis cápsulas.
- El temporizador de la visualización usa reloj monotónico. Los mensajes
  conservan tiempo ROS, de modo que no cambia la semántica de trayectoria ni
  la lógica de seguridad.
- La prueba final se ejecutó con Gazebo headless, manteniendo GUI y RViz, para
  dedicar los recursos gráficos a la visualización necesaria.

## Resultado medido

| Métrica | Antes | Después |
|---|---:|---:|
| Salida JOG supervisada | 14.96 Hz | 20.02 Hz |
| Cálculo predictivo p95 | 260.7 ms | 16.2 ms |
| Pausas de salida mayores de 200 ms | 12 | 0 |
| Salto máximo entre extremos proyectados | 0.212 rad | 0.053 rad |
| Publicación de la proyección en RViz | 6.96 Hz | 10.01 Hz |
| Intervalo p95 de la proyección | 163.3 ms | 106.3 ms |
| Cálculo visual mediano | 30.7 ms | 18.7 ms |

La captura final duró 15.1 s. Registró 153 predicciones articulares, 152
volúmenes visuales y 214 estados de proximidad, todos `ALLOW` y ninguno
`TF_UNAVAILABLE`.

## Verificación

- `colcon build --packages-select thesis_core --symlink-install`: correcto.
- Pruebas funcionales de `thesis_core`: **165 passed**, 3 pruebas de lint
  excluidas de esa ejecución.
- Comparación geométrica añadida para comprobar que la ruta optimizada conserva
  el mínimo, la muestra limitante y el testigo geométrico.

Rosbags utilizados:

- `evidence/rosbags/2026-10-18_proyeccion_futuro_gui_15s/`
- `evidence/rosbags/2026-10-18_proyeccion_futuro_optimizada_controlada_15s/`
- `evidence/rosbags/2026-10-18_proyeccion_futuro_reloj_monotono_15s/`

## Conclusión

**PASS.** El atasco de 200–300 ms en el control fue eliminado y la proyección
visual mantiene aproximadamente 10 Hz de pared. La simulación permanece
separada del hardware y no se habilitó ninguna salida física.
