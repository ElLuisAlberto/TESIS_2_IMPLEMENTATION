# Fase 5T — prueba de RViz desde terminal nativa

**Fecha:** 2026-10-21
**Dictamen:** `RVIZ_NATIVE_OK`

## Objetivo y contexto

Documentar la ejecución de la Fase 5T realizada por el operador en una terminal nativa de Ubuntu. La prueba se orientó a aislar el bloqueo previo asociado al entorno Snap heredado por la terminal integrada de VS Code. No se repitieron comandos gráficos desde el entorno integrado ni se realizaron acciones de corrección de Snap, bibliotecas o configuración del sistema.

## Evidencia de la terminal nativa

El operador confirmó que abrió una terminal nativa mediante `Ctrl+Alt+T`. La evidencia de la shell fue:

```text
PID 21216, PPID 21193, COMMAND bash, ARGS bash
ejecutable: /usr/bin/bash
```

Se confirmó la existencia del binario:

```text
/opt/ros/humble/lib/rviz2/rviz2
```

En esa terminal se cargó:

```bash
/opt/ros/humble/setup.bash
```

Y se ejecutó:

```bash
/opt/ros/humble/lib/rviz2/rviz2
```

## Evidencia de ausencia de variables Snap

La consulta ejecutada por el operador fue:

```bash
env | grep -E '^(SNAP|LD_LIBRARY_PATH|LD_PRELOAD)=' || true
```

No mostró ninguna variable. Por tanto, durante esta prueba no estaban presentes variables `SNAP`, `LD_LIBRARY_PATH` ni `LD_PRELOAD` en la terminal nativa indicada.

## Resultado de RViz y OpenGL

RViz se inició correctamente y el operador confirmó visualmente que la aplicación abrió. La salida registrada fue:

```text
[INFO] [1792600601.994210388] [rviz2]: Stereo is NOT SUPPORTED
[INFO] [1792600601.994263765] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[INFO] [1792600602.013556929] [rviz2]: Stereo is NOT SUPPORTED
```

El mensaje sobre estéreo no soportado es informativo; la inicialización de OpenGL 4.6 con GLSL 4.6 y la apertura visual confirmada de RViz sustentan el dictamen único `RVIZ_NATIVE_OK`.

## Alcance y límites

Esta evidencia valida que RViz puede abrirse en una terminal nativa de Ubuntu con el entorno ROS 2 Humble cargado. No valida todavía la cámara D435i ni su ubicación física o cobertura visual.

No se inició la D435i, JACO2, `thesis_hardware`, `hardware_output`, Gazebo, supervisor, auto-filtro, TF estático ni ningún launch ROS. No hubo movimiento físico, comandos al hardware, instalación, reparación de Snap ni edición de configuración del sistema. Tampoco hubo integración con TF, auto-filtro u obstáculo/supervisor.

## Próximo paso recomendado

Repetir desde una terminal nativa la evaluación visual de cobertura de la D435i, manteniendo el JACO2 inmóvil y desarmado.
