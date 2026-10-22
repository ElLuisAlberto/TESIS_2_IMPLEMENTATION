# Fase 6G — Resolución del contrato hardware desde fuente

Fecha: 2026-10-21. Resultado: **READY_FOR_READ_ONLY_SHADOW_SESSION**.

## Inventario y evidencia de fuente

El inventario obligatorio de `src/thesis_hardware` identificó un único launch
físico fuente:

```text
src/thesis_hardware/launch/jaco_physical_system.launch.py
```

El paquete se instala mediante el mecanismo de `ament_python` definido por
`setup.py`; tras reconstruir exclusivamente `thesis_hardware`, el launch queda
disponible bajo:

```text
install/thesis_hardware/share/thesis_hardware/launch/jaco_physical_system.launch.py
```

El contrato físico declarado usa `hardware_output`; su valor explícito `false`
mantiene la salida física desarmada para la modalidad de lectura. La cadena de
lectura expone el estado articular en `/joint_states` y los marcos del robot se
derivan mediante los nodos físicos/estado de robot propios de ese launch.

## Corrección mínima

Se editó únicamente:

`src/thesis_jaco_perception/launch/jaco_d435i_shadow_session.launch.py`

Cambio:

```diff
- thesis_hardware.launch.py
+ jaco_physical_system.launch.py
```

El include conserva el mapeo explícito y obligatorio:

```python
launch_arguments={"hardware_output": "false"}.items()
```

La prueba de contrato se actualizó solo para exigir el nombre real, prohibir la
ruta antigua y comprobar el argumento físico desarmado. No se modificaron
parámetros de percepción, filtros, extrínseca, extractor, supervisor ni
seguridad.

## Verificación sin hardware

Se ejecutaron sin iniciar nodos:

```text
colcon build --packages-select thesis_hardware thesis_jaco_perception --symlink-install
colcon test --packages-select thesis_hardware thesis_jaco_perception --event-handlers console_direct+
colcon test-result --all --verbose
ros2 launch thesis_jaco_perception jaco_d435i_shadow_session.launch.py --show-args
```

Compilación y pruebas aplicables completaron correctamente. La inspección de
argumentos no referencia `thesis_hardware.launch.py`; el contrato de sombra
mantiene `hardware_output=false` por defecto y el rechazo explícito de `true`.

## Siguiente paso autorizado

El contrato está listo para una única sesión física temporal de solo lectura,
usando el YAML candidato ya validado. Antes de observar nube se deberá
confirmar el diagnóstico físico desarmado, `/joint_states`, TF temporal, nube
y las tres salidas de sombra. Esta fase no inició cámara, RViz, JACO2, TF ni
self-filter.
