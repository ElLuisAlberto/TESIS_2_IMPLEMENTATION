# Evidencia — A-03: interbloqueo de armado físico

Fecha: 2026-10-18

## Alcance aplicado antes de la compilación

- El puente incorpora entradas de `system_ready`, `ProximityStatus`, estado
  STOP y presencia del supervisor para decidir ARM.
- El launch físico establece `require_armed: false` para que
  `system_ready` pueda ser verdadero antes de ARM, sin modificar el valor por
  defecto de `hardware_output_enabled` (`false`).
- Se añadieron pruebas puras del interbloqueo para cada precondición y para
  pérdida posterior de seguridad.

No se inició ROS, Gazebo, hardware ni cámara.

## Compilación solicitada

Entorno temporal:

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
```

Comando:

```bash
colcon build --packages-select thesis_hardware thesis_hardware_bridge
```

Resultado: **falló `thesis_hardware_bridge`; `thesis_hardware` no fue
procesado**.

Errores de compilación:

```text
error: too many initializers for ‘const std::array<double, 10>’
```

La lista de parámetros positivos incorporó `safety_input_timeout_sec_`, pero
el tamaño declarado del `std::array` permanece en 10.

```text
error: passing ‘const JacoHardwareNode’ as ‘this’ argument discards qualifiers
```

`safety_supervisor_available() const` llama
`get_node_graph_interface()`, cuya interfaz de ROS Humble no es `const`.

## Estado

Por la regla de detención no se corrigieron esos errores, no se ejecutaron
pruebas y no se validó A-03.
