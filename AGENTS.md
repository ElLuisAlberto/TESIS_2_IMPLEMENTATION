# Reglas del proyecto

- Trabajar inicialmente solo en simulación.
- Preservar todos los cambios existentes.
- No borrar ni sobrescribir archivos modificados sin autorización.
- No usar `sudo` ni instalar paquetes sin autorización.
- No activar `hardware_output`.
- No enviar comandos al Kinova Jaco2 físico.
- Se permite leer, analizar, editar código nuevo, compilar y probar la simulación.
- Antes de cambios importantes, explicar la causa.
- Después de modificar, compilar solo los paquetes afectados, ejecutar las pruebas y generar un informe breve.

## Organización de informes

- No crear informes en la raíz del proyecto.
- Guardar los diagnósticos técnicos en `docs/codex_reports/`.
- Guardar los informes de pruebas y feedback en `evidence/codex_reports/`.
- Crear esas carpetas automáticamente cuando no existan.
- Mantener en la raíz solo `README.md`, `AGENTS.md` y archivos generales de configuración.
- No mover, eliminar ni sobrescribir archivos existentes sin autorización explícita.
- Usar nombres con fecha y descripción, por ejemplo `docs/codex_reports/2026-10-18_diagnostico_cartesiano.md`.

## Consultas ROS 2 de observación

- Toda consulta de nodos o tópicos con el CLI `ros2` debe usar `--no-daemon --spin-time 2` para evitar el daemon de `ros2cli`.
- Ejecutar cada cliente de observación por separado y esperar a que termine antes de iniciar el siguiente.
- `tf2_echo` se ejecuta como herramienta independiente y no acepta esas opciones; detener solo ese cliente, sin enviar señales al launch.
