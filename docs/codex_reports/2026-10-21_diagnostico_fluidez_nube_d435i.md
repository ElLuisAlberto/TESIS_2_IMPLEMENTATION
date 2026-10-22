# Diagnóstico de fluidez y densidad de nube D435i

Fecha: 2026-10-21. Alcance: observación pasiva de la D435i y de RViz; no se
modificaron parámetros, código, launch ni configuración.

## Configuración estática mínima

`d435i_standalone.launch.py` referencia `perception_processing.launch.py`, el
cual carga `thesis_perception/config/perception.yaml`.

- Perfil de profundidad y RGB: `640,480,15` (640 x 480 a 15 FPS).
- Nube de cámara: habilitada, alineación de profundidad a color habilitada y
  `pointcloud.stream_filter=2`.
- Filtros de profundidad activos por defecto: decimación x2, espacial y
  temporal. IMU deshabilitada.
- Preprocesado: recorte de espacio deshabilitado; voxelización habilitada con
  celda de 0.02 m; intervalo de rango válido 0.10--1.50 m. Los límites XYZ
  están configurados, pero no intervienen mientras el recorte esté apagado.
- Raw esperado: `/camera/d435i/depth/color/points`. Filtrado esperado:
  `/thesis/perception/points_filtered`.

## Medición pasiva

Se ejecutó `d435i_observation_probe` durante 30 s por tópico, sin `report_path`
y sin rosbag. Los dos tópicos son `sensor_msgs/msg/PointCloud2`, con QoS
BEST_EFFORT y un solo publicador. La comparación no es simultánea, por lo que
las frecuencias se interpretan con la precisión de la ventana de observación.

| Métrica | Raw D435i | Filtrada |
|---|---:|---:|
| Ventana / mensajes | 30 s / 161 | 30 s / 170 |
| Frecuencia observada | 5.37 Hz | 5.67 Hz |
| Puntos finitos totales / aprox. por mensaje | 4,592,119 / 28,522 | 69,685 / 410 |
| Estructura observada | `height=1`; muestra `width=28,546`; campos `x,y,z,rgb` | `height=1`; muestra `width=406`; campos `x,y,z,rgb` |
| Rango | Raw evaluada pasivamente con umbral 0.10--1.50 m; no hay recorte del preprocesador en este tramo | 0.10--1.50 m configurado antes del voxel; sin recorte XYZ |
| Sellos / nubes vacías | Probe finalizó `COMPLETE`: cero sellos nulos, futuros o obsoletos; sin nube vacía | Probe finalizó `COMPLETE`: cero sellos nulos, futuros o obsoletos; sin nube vacía |
| Anomalías | Un cliente de eco puntual notificó pérdidas; no se usa como conteo de pérdida del publicador | Diagnóstico de salud: `DEGRADED: frecuencia de nube baja`, 7.00 Hz, edad de nube 0.06 s, edad de sello 0.17 s, 405 puntos |

La densidad filtrada equivale aproximadamente al 1.4 % de los puntos finitos
raw observados (una reducción de ~98.6 %). Esto es coherente con la
voxelización de 2 cm junto con el filtro de rango y los filtros de profundidad,
no con una pérdida adicional de frecuencia en el preprocesador: las dos
ventanas miden una tasa similar.

## Causa principal

**Cámara limitada por perfil/FPS.** El perfil configurado fija un techo de 15
FPS y, en esta sesión, la nube raw llegó a solo 5.37 Hz. La salida filtrada no
es más lenta (5.67 Hz), por lo que la evidencia no atribuye la falta de
fluidez al preprocesador ni a RViz. La baja cantidad de puntos visible en la
salida filtrada es una reducción intencional de densidad, pero no explica por
sí sola la frecuencia raw baja. El diagnóstico de salud independiente coincide
al reportar frecuencia baja y no muestra sellos envejecidos.

## Ajustes candidatos (no aplicados)

1. Menor riesgo: en una futura sesión de observación, revisar el estado físico
   de USB y eliminar cualquier consumidor adicional de la D435i antes de
   comparar otra ventana raw de 30 s; no modifica el pipeline.
2. Riesgo medio: reducir la hoja de voxel de 0.02 m para recuperar densidad en
   RViz, midiendo antes el impacto de CPU y latencia.
3. Mayor riesgo: elevar el perfil de profundidad/RGB por encima de 15 FPS y
   reevaluar USB, carga y estabilidad; puede aumentar ancho de banda y carga de
   procesamiento.

## Cierre de seguridad

La D435i permaneció en observación aislada. No se detuvo ni reinició el launch
activo, no se publicaron comandos ni obstáculos, y el JACO2 no se inició ni se
movió. No hubo conexión ni arranque del supervisor, `thesis_hardware`,
`hardware_output`, Gazebo, auto-filtrado ni TF estático.
