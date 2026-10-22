# Fase 5 — evaluación visual de ubicación provisional de la D435i

Fecha: 2026-10-21

## Resultado

**Dictamen: `PARTIAL_CANDIDATE_CAMERA_PLACEMENT`.**

El operador confirmó la preparación física solicitada: JACO2 inmóvil y
desarmado, y D435i apoyada de forma estable frente y ligeramente al costado
del brazo. La especificación cualitativa de la posición provisional fue una
distancia horizontal aproximada de 0.9 a 1.2 m de la base, altura aproximada
de 0.9 a 1.2 m e inclinación descendente aproximada de 20 a 30 grados. No se
puede confirmar visualmente esa posición ni recomendar aún reubicarla: la
visualización falló antes de poder realizar la inspección.

## Ejecución realizada

Se inició una sola vez el launch aislado solicitado de `thesis_perception`,
con RViz solicitado, extracción y salud habilitadas, sin `target_frame`, sin
restamp, sin recorte y con extrínseca estática deshabilitada. La D435i fue
detectada y el driver alcanzó `RealSense Node Is Up`. También iniciaron el
preprocesador, el extractor y el nodo de salud de percepción.

RViz terminó inmediatamente con código 127, antes de mostrar la nube:

```
symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0:
undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
```

No se intentó reparar el entorno, no se ejecutó un segundo launch y el único
launch abierto fue cerrado con `SIGINT`. Se verificó que no quedaron procesos
de la cámara, RViz ni los nodos de percepción.

## Criterios visuales

| Comprobación | Resultado |
| --- | --- |
| Hombro visible | No evaluado: RViz no inició. |
| Codo visible | No evaluado: RViz no inició. |
| Muñeca visible | No evaluado: RViz no inició. |
| Efector final visible | No evaluado: RViz no inició. |
| Caja/cilindro en las tres posiciones | No realizado. No se colocó ni movió el objeto de prueba. |
| Brazo domina la extracción / objeto separado | No evaluado. |
| `/thesis/perception/diagnostics` | No evaluado visualmente; no se hicieron consultas ROS adicionales. |
| Capturas de pantalla | No existen. |

## Aislamiento y seguridad

- El JACO2 no se inició ni se movió; no se inició `thesis_hardware` ni GUI física.
- `hardware_output` no se activó.
- No se inició Gazebo ni se abrió una sesión funcional de RViz.
- No se publicó ni configuró una transformación `world -> cámara`; no se inició un publicador de TF estático.
- `jaco_self_filter_node` no se inició.
- No hubo conexión a `/thesis/obstacle_input` ni al supervisor preventivo. El extractor permaneció como candidato aislado.
- No se editaron código, YAML ni launch, y no se realizaron fases posteriores.

## Próximo paso bloqueado

Antes de decidir `PASS_CANDIDATE_CAMERA_PLACEMENT` o
`REPOSITION_CAMERA_REQUIRED`, se requiere una ejecución futura de la misma
evaluación visual con un entorno RViz funcional. Esa ejecución deberá seguir
siendo aislada, con el JACO2 inmóvil y desarmado, y repetir las tres posiciones
del objeto antes de emitir una decisión sobre la ubicación.
