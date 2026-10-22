# Fase 6k — verificación runtime de sesión sombra

Fecha: 2026-10-21
Resultado: `BLOCKED_RUNTIME_PREREQUISITE`

## Alcance

Se observó pasivamente la sesión de sombra ya iniciada. No se modificó configuración, no se publicaron comandos y `hardware_output` permaneció en `false`. La sesión se cerró después con SIGINT porque no se cumplieron las precondiciones para mantenerla activa.

## Perfil y nubes D435i

La D435i estuvo activa y la cámara de color informó 640 x 480 a 30.02 Hz. Durante la muestra pasiva, la nube raw `/camera/d435i/depth/color/points` entregó 132 mensajes, 21.94 Hz medidos y 28,890 puntos en el último mensaje. La nube filtrada `/thesis/perception/points_filtered` entregó 135 mensajes, 26.64 Hz medidos y 1,291 puntos en el último mensaje. Ambas usaban `d435i_depth_optical_frame`.

Por tanto, no se confirma que las dos nubes se mantuvieran cerca de 30 FPS. Tampoco se pudo confirmar voxel de 0.01 m: la consulta pasiva de `voxel_size` al preprocesador devolvió `Failed to get parameters: voxel_size` y no hay una declaración de voxel en la configuración de la sesión revisada.

## Estado físico y TF

Al inicio, el adaptador informó `salida_fisica_permitida=false, armado=false`; el comprobador de preparación informó `READY`. Durante la muestra, `/joint_states` fue reciente (121 mensajes, 20.00 Hz, 12 articulaciones) y `/tf` fue reciente (120 mensajes, 20.00 Hz), con los 12 enlaces cinemáticos del JACO2. No se observaron errores de TF. El transform temporal `world -> d435i_depth_optical_frame` fue publicado al arranque de la sesión (0.680, -0.030, 0.300; cuaternión 0, -0.087156, 0, 0.996195).

## Self-filter

Los tres tópicos existían, pero durante la muestra no emitieron mensajes:

| Tópico | Mensajes | Frecuencia | Conteo de puntos |
| --- | ---: | ---: | --- |
| `/thesis/jaco_perception/points_self_confirmed` | 0 | 0 Hz | no disponible |
| `/thesis/jaco_perception/points_external_candidate` | 0 | 0 Hz | no disponible |
| `/thesis/jaco_perception/points_occluded_unknown` | 0 | 0 Hz | no disponible |

En consecuencia, `self_confirmed` no pudo confirmarse no vacío con el brazo visible. Esta condición impide validar la clasificación en RViz y obliga a cerrar la sesión conforme al criterio indicado.

## Advertencias RealSense observadas

El nodo RealSense informó como no soportados los siguientes parámetros: `start_jaco_read_only`, `start_camera`, `start_rviz`, `enable_self_filter`, `publish_session_extrinsic`, `candidate_extrinsic_file`, `hardware_output`, `expected_serial_number`, `kinova_library_dir`, `start_extractor`, `publish_static_extrinsic`, `restamp_output`, `enable_crop`, `extrinsic_x`, `extrinsic_y`, `extrinsic_z`, `extrinsic_roll`, `extrinsic_pitch`, `extrinsic_yaw` y `global_params`.

La advertencia los clasifica como argumentos ignorados por `realsense2_camera`: no son parámetros del perfil de sensor ni de filtrado de nube. La cámara sí abrió profundidad 640 x 480 x 30 y color 640 x 480 x 30. No se aplicó ninguna corrección en esta fase.

## Cierre

Se envió SIGINT una sola vez. Todos los procesos finalizaron limpiamente, incluidos cámara, adaptador JACO2, RViz, preprocesador y self-filter. No se ajustó extrínseca, cámara ni brazo.

Para una siguiente ejecución, el operador debe observar en RViz si los puntos clasificados del brazo coinciden aproximadamente con el brazo real; esta verificación no pudo realizarse en esta ejecución al no existir puntos clasificados.
