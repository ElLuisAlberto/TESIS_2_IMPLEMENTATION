# Validación C-01 — restauración del entorno ROS

Fecha: 2026-10-18  
Resultado: **C-01 NO VALIDADA**

## Entorno e importación

Se usó el procedimiento solicitado desde la raíz: desactivar temporalmente
`set -u`, cargar `/opt/ros/humble/setup.bash`, cargar `install/setup.bash` y
restaurar `set -u`. `rclpy` importó correctamente desde el prefijo ROS Humble.

## Paquetes compilados

`thesis_core` y `thesis_hardware_bridge` compilaron correctamente con
`colcon build --packages-select`.

## Pruebas ejecutadas

Se ejecutaron únicamente las pruebas C-01 del supervisor y sus dependencias
directas: 9 aprobadas y 5 fallidas. Las fallas son aserciones de
`test_point_command_admission.py`: el mock captura el argumento de texto de
rechazo, pero la prueba lo compara con el código estructurado esperado. No se
ejecutaron las pruebas del puente después del fallo.

## Decisión

C-01 no queda validada. No se modificó nada al detectar el fallo, conforme a
la regla de detención. Para reanudar hace falta resolver esa discrepancia de
la prueba o del contrato de llamada, repetir primero el supervisor y solo si
pasa ejecutar `test_bridge_logic` para sellos cero/vencido/futuro, ID
vacío/reutilizado, `ExecutionControl` ajeno y STOP prioritario.

Evidencia completa:
[2026-10-18_validacion_c01_entorno_ros.md](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/evidence/codex_reports/2026-10-18_validacion_c01_entorno_ros.md).
