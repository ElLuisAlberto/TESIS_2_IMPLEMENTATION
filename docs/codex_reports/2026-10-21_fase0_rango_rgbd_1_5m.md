# Fase 0 RGB-D: límite de profundidad D435i de 1.50 m

Fecha: 2026-10-21
Alcance: cambio de configuración por defecto y prueba estática en `thesis_perception` exclusivamente.

## Resultado

El preprocesador de nubes de la D435i ahora descarta por defecto retornos con rango radial superior a **1.5 m** antes de voxelización y extracción de obstáculos. El límite mínimo se mantiene en **0.10 m**.

## Archivos modificados

- `thesis_perception/config/perception.yaml`: `max_valid_range_m: 1.5` y comentario que documenta el descarte del fondo del salón.
- `thesis_perception/launch/perception_processing.launch.py`: argumento `max_valid_range_m` con default `"1.5"`.
- `thesis_perception/launch/d435i_standalone.launch.py`: argumento `max_valid_range_m` con default `"1.5"`.
- `thesis_perception/test/test_perception_safety_contract.py`: comprueba exactamente mínimo 0.10 m y máximo 1.5 m en YAML, y que ambos launch exponen 1.5 m sin argumentos de terminal.

## Verificación ejecutada

```text
source /opt/ros/humble/setup.bash
colcon build --packages-select thesis_perception
# Resultado: 1 paquete terminado correctamente.

source install/setup.bash
colcon test --packages-select thesis_perception
colcon test-result --test-result-base build/thesis_perception --verbose
# Resultado: 38 tests, 0 errors, 0 failures, 3 skipped.
```

También se ejecutó `git diff --check`, sin errores de espacios.

## Límites preservados

- `enable_crop` continúa en `false`.
- `target_frame` continúa vacío y `restamp_output` continúa en `false`.
- No se inventó ni publicó la extrínseca `world -> d435i_link`; el opt-in de TF estático continúa desactivado.
- El candidato conserva `/thesis/perception/obstacle_candidate`; no se conecta ni remapea a `/thesis/obstacle_input`.
- No se modificaron `thesis_core`, `thesis_hardware`, `thesis_hardware_bridge`, `thesis_description` ni la ruta física.
- No se iniciaron ROS, RViz, RealSense, Gazebo ni el JACO2 y no se activó `hardware_output`.

Por tanto, la cámara sigue aislada del supervisor preventivo.
