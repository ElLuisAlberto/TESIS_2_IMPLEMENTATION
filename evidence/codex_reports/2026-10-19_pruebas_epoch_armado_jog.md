# Evidencia de regresión de época de armado JOG

```text
colcon build --packages-select thesis_hardware_bridge thesis_ui --symlink-install
colcon test --packages-select thesis_hardware_bridge thesis_ui

thesis_hardware_bridge: 20 tests, 0 errores, 0 fallos
thesis_ui:              26 tests, 0 errores, 0 fallos, 1 omitida
```

Casos verificados automáticamente:

1. HOLD armado no transmite comandos físicos periódicos.
2. `jog_continuous` y `jog:arm-<previa>:...` son rechazados.
3. Solo `jog:arm-<actual>:...` es admisible.
4. Con modo físico desarmado, la GUI no publica JOG aunque exista una
   intención de rueda local.
5. Tras armado y token válido, una rueda nueva genera el único ID admisible.

No se accedió al USB, no se armó y no se publicó movimiento durante esta
validación.
