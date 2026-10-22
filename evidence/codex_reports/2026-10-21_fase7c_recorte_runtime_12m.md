# Fase 7C: recorte de rango temporal a 1.2 m

## Dictamen

`RANGE_12M_VIEW_READY`

## Cambio temporal de runtime

El publicador de `/thesis/perception/points_filtered` es el nodo real
`/pointcloud_preprocessor`. Se identificó el parámetro dinámico
`max_valid_range_m` (tipo `double`), con valor inicial `1.5` m. Su descriptor
no declaró restricción de solo lectura y la actualización dinámica respondió
`Set parameter successful`.

Se cambió exclusivamente en runtime:

| Parámetro | Antes | Después |
|---|---:|---:|
| `/pointcloud_preprocessor.max_valid_range_m` | 1.5 m | 1.2 m |

No se editaron archivos y no se reinició el launch, RViz, D435i ni JACO2.
El valor `1.2` m fue releído desde el servicio de parámetros del nodo después
de la actualización.

## Verificación posterior

- `/pointcloud_preprocessor` permanece como único publicador de
  `/thesis/perception/points_filtered`.
- Una muestra posterior conserva `frame_id=world` y contiene `2117` puntos.
- `/rviz` sigue suscrito a esa nube con `BEST_EFFORT` y `VOLATILE`, compatibles
  con el publicador; RViz continúa recibiendo el tópico.
- La D435i permanece activa como `/camera/d435i`.
- JACO2 permanece conectado y en solo lectura: el estado de armado publicado
  sigue siendo `false`; `hardware_output=false` se conserva en el launch
  activo.

## Seguridad y alcance

No se modificaron código, YAML, launch, CMake, TF, cápsulas, epsilon, la
extrínseca v2 ni el perfil RViz. No se enviaron comandos, trayectorias, jog ni
servicios de movimiento al JACO2. No se inició ni detuvo supervisor, obstacle
bridge, extractor, GUI física, Gazebo ni adaptadores de comando, y no se usó el
módulo multi-pose. Las dos terminales nativas se dejan abiertas para la
captura del operador.
