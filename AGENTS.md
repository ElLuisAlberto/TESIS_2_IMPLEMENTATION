# Reglas del proyecto

- Trabajar inicialmente solo en simulación; se permite pasar al JACO2 físico cuando el usuario autorizado lo solicite explícitamente.
- Preservar todos los cambios existentes.
- No borrar ni sobrescribir archivos modificados sin autorización.
- No usar `sudo` ni instalar paquetes sin autorización.
- Codex puede acceder al repositorio, editar archivos, compilar y ejecutar comandos desde su terminal integrada.
- Si una aplicación gráfica, como RViz, falla porque la terminal integrada hereda el entorno Snap de VS Code, diagnosticar primero sin modificar el sistema.
- Si se requiere probar una aplicación fuera de ese entorno, generar un bloque exacto para que el operador lo ejecute desde una terminal nativa abierta con `Ctrl+Alt+T`; no interpretar esta limitación como falta de acceso al repositorio.
- No solicitar una terminal nativa para tareas que puedan realizarse de forma segura desde la terminal integrada.
- Se permite activar `hardware_output` y enviar comandos al Kinova Jaco2 físico únicamente cuando el usuario haya confirmado que es operador autorizado, el área está despejada y la parada de emergencia es accesible.
- Antes de la primera prueba física, verificar el estado de conexión, diagnóstico, armado y disponibilidad del sistema sin enviar movimiento.
- Para movimientos físicos, comenzar con desplazamientos mínimos y velocidad reducida; solicitar confirmación del operador antes de cada etapa que pueda mover el brazo.
- Se permite leer, analizar, editar código nuevo, compilar y probar tanto simulación como la interfaz física autorizada.
- Antes de cambios importantes, explicar la causa.
- Después de modificar, compilar solo los paquetes afectados, ejecutar las pruebas aplicables y generar un informe breve.

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