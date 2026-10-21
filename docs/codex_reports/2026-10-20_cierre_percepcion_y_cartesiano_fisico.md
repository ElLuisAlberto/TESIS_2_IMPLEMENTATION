# Cierre de percepción y control cartesiano físico

Fecha: 2026-10-20  
Dictamen: **BLOQUEADO — NO ABRIR SALIDA ARMABLE**

## Resultado ejecutivo

El software cartesiano, USB Kinova, estado articular y coherencia FK–TF ya
están aprobados en reposo. La compuerta de percepción real no pudo aprobarse:

1. La D435i no aparece en USB ni en librealsense. `rs-enumerate-devices -s`
   respondió `No device detected` y el driver informó repetidamente
   `No RealSense devices were found`.
2. El repositorio no contiene una extrínseca física medida
   `world -> d435i_link`. Los únicos valores disponibles son ceros de plantilla
   y su publicación está correctamente desactivada por defecto.
3. Los límites de recorte del espacio de trabajo continúan desactivados y sin
   calibración. Conectar directamente el extractor sin validarlos podría
   seleccionar mesa, suelo, pared o el propio robot como el obstáculo principal.

Por estas razones no se remapeó
`/thesis/perception/obstacle_candidate` a `/thesis/obstacle_input`, no se
inició el stack físico, no se abrió una interfaz armable y no se armó ni movió
el JACO2.

## Cadena inspeccionada

La topología existente es coherente y conserva una frontera de seguridad:

```text
/camera/d435i/depth/color/points (PointCloud2, QoS sensor)
 -> pointcloud_preprocessor
 -> /thesis/perception/points_filtered
 -> obstacle_extractor
 -> /thesis/perception/obstacle_candidate (Obstacle)
```

La promoción deliberada todavía pendiente es:

```text
/thesis/perception/obstacle_candidate
 -> /thesis/obstacle_input
 -> obstacle_input_bridge
 -> /thesis/obstacle_in_model_frame (world)
 -> proximity_monitor
 -> system_readiness
```

El preprocesador preserva el sello de adquisición. El extractor conserva el
header y publica centro, velocidad, radio e incertidumbre. El bridge rechaza
frame vacío, números inválidos, sello obsoleto/futuro y TF ausente al sello del
mensaje. No se encontró un defecto en tipos, sellos o nombres que justificara
debilitar esa frontera.

## Prueba independiente de cámara

El launch aislado inició los nodos de driver, preprocesamiento, extracción y
salud, manteniendo `publish_static_extrinsic=false` y el tópico candidato
aislado.

Observaciones:

```text
/camera/d435i/depth/color/points:
  publisher count: 0
  subscription count: 1 (pointcloud_preprocessor)

/thesis/perception/diagnostics:
  level: ERROR
  message: LOST: nube no disponible
  cloud_hz: 0.00
  cloud_age_s: -1.00
  cloud_points: 0

/thesis/perception/obstacle_candidate:
  publisher count: 1 (obstacle_extractor)
  mensajes: ninguno, porque no hubo nubes
```

Esto confirma fallo cerrado: la ausencia de cámara no produce un obstáculo
ficticio ni disponibilidad preventiva.

## Build y pruebas

`thesis_perception` compiló correctamente con `--symlink-install`. Su suite
terminó con:

```text
28 tests, 0 errors, 0 failures, 3 skipped
7/7 contratos focales de seguridad aprobados
```

No se modificó código: la causa no es reparable con una constante o remapeo
sin crear geometría física falsa.

## Parámetros requeridos tras calibrar

Una futura validación debe conectar físicamente la D435i, medir y documentar
la pose fija `world -> d435i_link`, verificarla con objetos en coordenadas
conocidas y definir el recorte del espacio de trabajo. Solo entonces procede
ejecutar percepción con:

```text
target_frame=world
use_latest_transform=false
restamp_output=false
publish_static_extrinsic=true
extrinsic_x/y/z/roll/pitch/yaw=<valores medidos>
enable_crop=true
min/max x/y/z=<volumen validado>
obstacle_topic=/thesis/obstacle_input
```

Antes de permitir armado se debe comprobar pérdida de cámara, nube vacía,
timestamp vencido, TF ausente y desconexión. Todos deben mantener
`NOT_READY`/STOP según corresponda.

## Veredicto

`LISTO_PARA_PRUEBA_MANUAL_CARTESIANA=NO`

Causa técnica exacta: **D435i ausente del bus USB y extrínseca física
`world -> d435i_link` inexistente/no calibrada**. La interfaz final no puede
abrirse como armable hasta resolver y validar ambas condiciones.
