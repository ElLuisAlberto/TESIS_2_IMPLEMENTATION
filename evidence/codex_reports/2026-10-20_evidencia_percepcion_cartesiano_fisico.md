# Evidencia de percepción para el cierre cartesiano físico

Fecha: 2026-10-20

## Detección de hardware

```text
$ lsusb
Bus 003 Device 016: 22cd:0000 Kinova Robotics Inc. Jaco Robotic Arm
No aparece ningún dispositivo Intel RealSense.

$ rs-enumerate-devices -s
No device detected. Is it plugged in?
```

Solo estaban disponibles `/dev/video0` y `/dev/video1`, correspondientes a la
cámara web integrada; no se detectó la D435i.

## Ejecución aislada

Se inició `thesis_perception/d435i_standalone.launch.py` en
`ROS_DOMAIN_ID=78`, sin RViz, con extrínseca desactivada y salida candidata
aislada.

El driver RealSense v4.58.4 repitió:

```text
No RealSense devices were found!
```

El grafo mostró:

```text
/camera/d435i
/pointcloud_preprocessor
/obstacle_extractor
/perception_health
```

La entrada de nube tuvo cero publicadores. Salud publicó
`LOST: nube no disponible`, `cloud_hz=0.00` y `cloud_points=0`. El extractor
anunció `/thesis/perception/obstacle_candidate`, pero no emitió estimaciones.
El launch se detuvo ordenadamente con SIGINT.

## Calibración

Una búsqueda de configuración, documentación y evidencia no encontró valores
medidos para `world -> d435i_link`. Los argumentos `extrinsic_*` existentes son
plantillas `0.0`, protegidas por `publish_static_extrinsic=false`. Tampoco hay
límites de crop validados; `enable_crop=false` es el valor seguro actual.

## Pruebas

```bash
colcon build --symlink-install --packages-select thesis_perception
colcon test --packages-select thesis_perception \
  --return-code-on-test-failure
colcon test-result --test-result-base \
  build/thesis_perception/test_results --verbose
```

Resultado:

```text
28 tests, 0 errors, 0 failures, 3 skipped
```

No se inició el JACO2 físico durante esta validación, no se habilitó salida,
no hubo armado, JOG, candidato físico ni movimiento.
