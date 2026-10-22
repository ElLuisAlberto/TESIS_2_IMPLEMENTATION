# Fase 8U — preparación estática de calibración ChArUco temporal

## Dictamen: `CHARUCO_CALIBRATION_PREPARED`

Se preparó un tablero imprimible y una herramienta observacional para una fase futura. **No se capturó ninguna imagen ni se estimó o aplicó una extrínseca en esta fase.** La referencia física inequívoca será un tablero ChArUco fijado a la base del JACO2, siempre que se mida y documente su pose respecto de `world`.

## Capacidades y tablero

El OpenCV instalado es `4.5.4` y dispone de `cv2.aruco`, `DICT_4X4_50`, `CharucoBoard_create`, interpolación de esquinas y `solvePnPRansac`. Esta versión no expone el constructor nuevo `cv2.aruco.CharucoBoard`; se usó la API compatible `CharucoBoard_create`. PIL y ReportLab ya estaban instalados y permitieron generar PNG y PDF sin instalar ni descargar paquetes.

El patrón tiene **7 cuadros horizontales × 5 verticales**, cuadrados de **30 mm**, marcadores de **22 mm** y tamaño útil exacto **210 × 150 mm**. Contiene 24 esquinas ChArUco interiores y 17 marcadores. El PDF es una página A4 horizontal con el patrón dibujado a 210 × 150 mm; el PNG mide 2100 × 1500 píxeles a 254 ppp. Se inspeccionó el patrón y se comprobó el tamaño de página del PDF y la resolución/densidad del PNG. Los SHA-256 de PDF y PNG constan en la especificación.

Archivos en `evidence/codex_reports/2026-10-22_fase8u_charuco/`:

| Archivo | Uso |
|---|---|
| `charuco_board.pdf` | Imprimir en A4 horizontal al 100 %, sin ajustar a página. |
| `charuco_board.png` | Vista e inspección del patrón; 2100 × 1500 px. |
| `charuco_specification.yaml` | Geometría nominal, frames, resolución y hashes. |
| `instrucciones_impresion.md` | Verificación con regla, montaje y condiciones de captura. |
| `board_pose_relative_to_world_TEMPLATE.yaml` | Plantilla de medición futura, deliberadamente incompleta. |

Las instrucciones exigen verificar con regla cuadros de 30 mm, pegar el papel plano sobre cartón rígido mate, fijarlo temporalmente a una parte rígida y visible de la base, no sostenerlo con la mano y mantener cámara, tablero y brazo inmóviles con `hardware_output=false`. La impresión que no mida correctamente no sirve para calibrar.

## Pose física aún pendiente

No se inventó `T_world_charuco_board`. La plantilla marca `tx_m`, `ty_m`, `tz_m`, `roll_rad`, `pitch_rad` y `yaw_rad` como `REQUIRED`, junto con las dimensiones impresas verificadas, método, tolerancias, fecha/responsable, montaje y referencias de origen/ejes de `world`. El frame del tablero tiene origen en la esquina exterior superior izquierda vista de frente; `x` va a la derecha, `y` hacia abajo y `z` entra en la hoja. El operador deberá medir la posición tridimensional de esa esquina **y** la orientación del plano respecto del origen y ejes de `world`. Una sola distancia no determina la pose 6D. Estos datos se solicitarán únicamente cuando se prepare una captura real futura.

La herramienta rechaza valores `REQUIRED`, dimensiones que difieran de las nominales más de 0,5 mm por cuadro/marcador o 1 mm en el patrón total, y una incertidumbre declarada mayor de 5 mm en posición o 2° en orientación. Esos límites son condiciones conservadoras del cliente de evidencia, no ajustes operativos.

## Herramienta futura y aceptación

`tools/perception/charuco_observational_once.py` queda preparada, **sin ejecutarse en ROS**. Al invocarse más adelante con una pose medida completa, la metadata congelada de la cadena fija `d435i_link ← color optical` y una ruta nueva bajo `evidence/codex_reports/`, creará como máximo un cliente de RGB y CameraInfo durante 10 s. No contiene publicador TF, comando de brazo ni escritura de YAML operativo. Rechaza diferencia temporal mayor de 33 ms o frames/dimensiones discordantes. Detecta marcadores y esquinas ChArUco automáticamente; exige al menos 12 esquinas distintas y no colineales, al menos 12 inliers RANSAC, RMS reproyectivo ≤1,5 px, error máximo por esquina ≤3 px, puntos delante de la cámara, pose rígida y procedencia física completa. Sólo entonces escribirá un YAML **nuevo de evidencia** con `session_only=true` y `not_for_safety=true`; nunca lo cargará ni publicará.

La convención es `p_A = T_A_B p_B`. `solvePnP` estima `T_color_charuco`; la composición preparada es `T_world_color = T_world_charuco × inverse(T_color_charuco)` y `T_world_d435i_link = T_world_color × inverse(T_d435i_link_color)`. El transformado fijo se lee de la metadata 8T congelada y se valida contra su hash, sin consultar TF actual. Se prueban matrices rígidas y el orden de composición con datos sintéticos. La salida futura representa sólo una hipótesis temporal; no se promueve a extrínseca final ni a seguridad.

## Validación y estado

Las cinco pruebas offline nuevas pasaron: detección del patrón generado, recuperación de pose sintética y composición, rechazo de menos de 12 esquinas, rechazo de RMS alto, rechazo de pose incompleta, de matriz no rígida y de tolerancias físicas insuficientes. `py_compile` pasó para ambos programas. No se añadió código compilado; por tanto no correspondió un build de `thesis_jaco_perception`. `git diff --check` y la comprobación de espacios de los archivos nuevos no encontraron errores.

No se inició ni modificó ROS, D435i, JACO2, RViz, launch, TF, Gazebo, extrínseca, filtros, ROI, epsilon, radios, mallas o configuración operativa. No se movió ni armó el brazo, y no se emitió ningún YAML candidato en esta fase.
