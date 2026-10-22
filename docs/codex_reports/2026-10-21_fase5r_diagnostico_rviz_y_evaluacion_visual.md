# Fase 5R — diagnóstico de RViz y evaluación visual

Fecha: 2026-10-21

## Decisión

**`BLOCKED_RVIZ_ENVIRONMENT`**

No se repitió la evaluación visual de la D435i. El binario ROS de RViz existe,
pero falla antes de abrir debido a una biblioteca `libpthread` inyectada desde
Snap. Corregir o aislar esas bibliotecas exigiría un cambio de entorno o del
sistema, acciones que no estaban autorizadas.

## Inspección estática

Se ejecutaron, sin modificar el entorno:

```bash
command -v rviz2
type -a rviz2
readlink -f "$(command -v rviz2)"
ls -l /opt/ros/humble/lib/rviz2/rviz2
env | grep -E '^(PATH|LD_LIBRARY_PATH|LD_PRELOAD|SNAP|ROS_)' || true
```

Resultados relevantes:

- `command -v rviz2` no devolvió ruta y `type -a rviz2` indicó que no se
  encontró el comando en el `PATH` de la sesión inicial.
- Existe el binario ROS instalado:
  `/opt/ros/humble/lib/rviz2/rviz2` (ejecutable regular).
- La sesión heredó variables de Snap, entre ellas `SNAP=/snap/code/266`,
  `SNAP_NAME=code` y `SNAP_LIBRARY_PATH`; el `PATH` también contiene
  `/snap/bin`.

Por tanto, no había una resolución inicial de `rviz2` hacia un ejecutable Snap.
Sin embargo, la ejecución del binario ROS queda afectada por el entorno Snap
heredado por el proceso del IDE.

## Prueba mínima del binario ROS

Con únicamente ROS Humble cargado y sin cámara, JACO2 ni launch, se ejecutó:

```bash
set +u
source /opt/ros/humble/setup.bash
set -u
/opt/ros/humble/lib/rviz2/rviz2
```

El proceso terminó inmediatamente con código 127:

```text
/opt/ros/humble/lib/rviz2/rviz2: symbol lookup error:
/snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0:
undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
```

La causa técnica probable es la contaminación del entorno de la sesión Snap
por bibliotecas de `core20`, que no son compatibles con la `glibc` contra la
que se ejecuta RViz ROS Humble. No se alteraron `PATH`, `LD_LIBRARY_PATH` ni
`LD_PRELOAD`; tampoco se hicieron reparaciones de bibliotecas, enlaces,
instalaciones o cambios de Snap.

## Evaluación visual

No se inició la D435i ni el launch de percepción en esta repetición. En
consecuencia, no se evaluaron:

| Elemento | Resultado |
| --- | --- |
| Hombro | No evaluado. |
| Codo | No evaluado. |
| Muñeca | No evaluado. |
| Efector final | No evaluado. |
| Objeto en posición izquierda | No evaluado. |
| Objeto en posición central | No evaluado. |
| Objeto en posición derecha | No evaluado. |
| Distinción cualitativa objeto/brazo | No evaluada. |

## Seguridad y aislamiento

- El JACO2 no fue iniciado ni movido y permaneció desarmado.
- `hardware_output` no fue activado.
- No se inició `thesis_hardware`, GUI física, Gazebo, supervisor preventivo ni
  `jaco_self_filter_node`.
- No se publicó TF estático ni una transformación `world -> cámara`.
- No se inició cámara, no se publicó `/thesis/obstacle_input` y no hubo
  conexión al supervisor.
- No se modificaron código, YAML, launch, CMake ni configuración del sistema.

Para retomar la Fase 5 será necesario disponer de una sesión de RViz ROS que
no herede las bibliotecas incompatibles de Snap; esta fase no autoriza ni
realiza esa corrección.
