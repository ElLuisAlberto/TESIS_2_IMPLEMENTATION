# Implementación del control cartesiano físico

Fecha: 2026-10-20

Se añadieron: rechazo por manipulabilidad de Yoshikawa, interbloqueo periódico
FK–TF en el supervisor, IDs JOG ligados a la época de armado, adaptadores
cartesianos en el launch físico y pestaña cartesiana continua en la GUI física.

El adaptador cartesiano permanece detrás de `/thesis/jog_intent`, el supervisor
y `/thesis/supervised_jog_command`; no existe salida directa a Kinova. La GUI
exige hardware armado, epoch vigente, estado fresco y READY antes de activar el
modo cartesiano.

No se ejecutó el launch físico, no se armó el brazo y no se publicaron comandos.

Compilación:

```text
thesis_core, thesis_simulation, thesis_hardware, thesis_ui: OK
```

Pruebas:

```text
thesis_core:       174, 0 fallos, 1 omitida
thesis_simulation:  43, 0 fallos, 1 omitida
thesis_hardware:    13, 0 fallos, 1 omitida
thesis_ui:          28, 0 fallos, 1 omitida
```

La validación física requiere autorización separada.

La integración mock adicional confirmó los tres nodos cartesianos, coherencia
FK–TF de `0.000000 m`, `READY`, armado falso y ausencia de intenciones. Se usó
un dominio aislado y `hardware_output_enabled=false`.

