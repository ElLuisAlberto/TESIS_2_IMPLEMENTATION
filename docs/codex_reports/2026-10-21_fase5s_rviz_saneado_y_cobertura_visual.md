# Fase 5S — RViz saneado y cobertura visual de la D435i

Fecha: 2026-10-21

## Decisión

**`BLOCKED_RVIZ_ENVIRONMENT`**

La prueba mínima de RViz ROS volvió a fallar antes de abrir. Por la regla de
detención de esta fase no se inició la D435i, no se ejecutó el launch de
percepción y no se intentaron reparaciones adicionales.

## Entorno limitado al proceso de prueba

La prueba se ejecutó en una subshell. Se eliminaron solo para ese proceso:

```text
SNAP SNAP_ARCH SNAP_COOKIE SNAP_CONTEXT SNAP_DATA SNAP_INSTANCE_KEY
SNAP_LIBRARY_PATH SNAP_NAME SNAP_REEXEC SNAP_REVISION SNAP_USER_COMMON
SNAP_USER_DATA LD_LIBRARY_PATH LD_PRELOAD
```

No se usó `env -i`; por ello se conservaron `DISPLAY` y las demás variables de
la sesión gráfica. No se modificó ninguna variable global, instalación Snap,
paquete, biblioteca, enlace simbólico ni archivo de configuración.

## Prueba mínima de RViz

Comando ejecutado, sin cámara ni JACO2:

```bash
unset SNAP SNAP_ARCH SNAP_COOKIE SNAP_CONTEXT SNAP_DATA SNAP_INSTANCE_KEY SNAP_LIBRARY_PATH SNAP_NAME SNAP_REEXEC SNAP_REVISION SNAP_USER_COMMON SNAP_USER_DATA LD_LIBRARY_PATH LD_PRELOAD
set +u
source /opt/ros/humble/setup.bash
set -u
/opt/ros/humble/lib/rviz2/rviz2
```

Resultado: el proceso terminó de inmediato con código 127 y no abrió RViz:

```text
/opt/ros/humble/lib/rviz2/rviz2: symbol lookup error:
/snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0:
undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
```

La persistencia de la carga desde `/snap/core20` después de eliminar las
variables permitidas confirma que la causa no puede aislarse con este
procedimiento de entorno acotado. No se alteró ninguna biblioteca ni se
ensayó otro visor.

## Launch y evaluación visual

No se ejecutó el launch de Fase 5; por tanto no existe un comando de launch
ejecutado en esta fase. Tampoco se tomaron capturas.

| Elemento de cobertura | Resultado |
| --- | --- |
| Hombro | No evaluado. |
| Codo | No evaluado. |
| Muñeca | No evaluado. |
| Efector final | No evaluado. |
| Objeto, izquierda | No evaluado. |
| Objeto, centro | No evaluado. |
| Objeto, derecha | No evaluado. |
| Capturas | Ninguna. |

## Seguridad y aislamiento

- JACO2 no fue iniciado ni movido; se mantuvo desarmado.
- `hardware_output` no se activó.
- No se iniciaron `thesis_hardware`, GUI física, Gazebo, supervisor preventivo,
  `jaco_self_filter_node` ni la D435i.
- No hubo TF estático ni transformación `world -> cámara`.
- No hubo auto-filtrado ni conexión o publicación hacia
  `/thesis/obstacle_input`.
- No se modificaron código, CMake, YAML ni launch.

## Verificación final

No se ejecutaron pruebas: no se modificaron archivos de código ni
configuración. Se ejecutaron `git diff --check` y `git status --short` al
final de la fase; `git diff --check` no reportó errores. El árbol conserva
cambios preexistentes y agrega únicamente este informe para la Fase 5S.
