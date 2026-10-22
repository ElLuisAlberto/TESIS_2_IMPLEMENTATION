# Fase 6A — Diseño de calibración extrínseca D435i–JACO2

Fecha: 2026-10-21. Este documento es solo de diseño estático. No se inició ni
consultó `thesis_hardware`, no se movió el JACO2, no se creó/publicó TF y la
sesión aislada de D435i/RViz no fue detenida.

## Revisión local y contrato

`thesis_description` describe el modelo J2N6S300 con la base como referencia
del manipulador y la cadena articular de seis juntas del brazo más las juntas
de dedos. Para esta fase, el marco de trabajo de integración es `world` y la
referencia rígida del robot es su enlace de base; las juntas no intervienen en
la estimación porque el brazo debe permanecer inmóvil y desarmado.

El contrato actual de `thesis_jaco_perception` es preparatorio para una cámara
fija: declara `geometry_msgs`, `sensor_msgs`, `diagnostic_msgs`, `rclcpp` y
`tf2_ros`, y su configuración conserva el auto-filtrado en sombra. No declara
ni implementa detección de AprilTag, ArUco, OpenCV, `cv_bridge` o
`image_transport`. La D435i proporciona las nubes en el marco observado
`d435i_depth_optical_frame`; el objetivo de calibración es exactamente:

`world -> d435i_depth_optical_frame`.

No se encontraron dependencias locales declaradas que permitan implementar o
ejecutar hoy un detector de AprilTag/ArUco. Esta conclusión no supone que no
puedan existir paquetes del sistema: no se usaron búsquedas web ni se instaló
nada.

## Método recomendado

**Tablero ChArUco rígido, registrable mecánicamente a la base JACO2, detectado
en RGB y compuesto a profundidad.** Un tablero ChArUco combina las esquinas de
chessboard con IDs ArUco; permite rechazar capturas incompletas y estimar una
pose con más redundancia que un único tag.

La pieza debe acoplarse con topes y tornillos a una placa/adaptador con relación
de diseño conocida respecto a la base del JACO2. Así `world -> tablero` procede
de la geometría del útil y de la referencia de base, no de medir a mano una
posición/orientación. La detección entrega `d435i_color_optical_frame ->
tablero`; con la transformación intrínseca de fábrica de la D435i entre color y
profundidad se compone la transformación objetivo. La publicación futura de
TF solo se autorizará después de aceptar la estimación.

## Método de respaldo

**Malla rígida de cuatro AprilTags grandes**, coplanares o en dos planos de un
útil registrable a la misma base. Es más fácil de imprimir y detectar a mayor
distancia, pero ofrece menor precisión de esquinas que ChArUco y es más sensible
a la geometría coplanar. No se acepta un solo tag como resultado final.

## Marcos TF previstos

1. `world`: referencia fija de la celda/JACO2.
2. Enlace de base de `thesis_description`: referencia física rígida del JACO2
   usada para definir el útil; no se depende de las juntas del brazo.
3. `calibration_board`: marco futuro del tablero, fijado por diseño al útil.
4. `d435i_color_optical_frame`: marco de la imagen RGB que detecta el patrón.
5. `d435i_depth_optical_frame`: marco objetivo de la nube y de la extrínseca
   final.

La composición a validar antes de publicar nada será:
`world -> calibration_board -> d435i_color_optical_frame ->
d435i_depth_optical_frame`, con las inversiones de pose necesarias según la
convención del detector. No se usará una traslación ni orientación medida a
mano como sustituto de `world -> calibration_board`.

## Materiales concretos

- Tablero ChArUco impreso en papel mate rígido o PVC, recomendado A3: 5 x 7
  cuadrados, 35 mm por cuadrado y marcadores de 25--27 mm, con diccionario
  definido y documentado antes de imprimir.
- Placa plana rígida, soporte con topes o adaptador atornillable que registre
  repetiblemente el tablero respecto a la base del JACO2; no cinta, imanes ni
  sujeción flexible.
- Calibre/regla solo para verificar la fabricación del tablero, no para definir
  la extrínseca final.
- Iluminación difusa sin reflejos y fijación de cámara ya usada para encuadrar
  el brazo.
- Detector local que soporte ChArUco/ArUco y acceso a la imagen RGB de la
  D435i, más el mecanismo futuro y revisado para almacenar la estimación.

## Procedimiento físico mínimo futuro

1. Con JACO2 apagado/desarmado, montar el útil registrable en la base y fijar
   el tablero sin ocluir los topes mecánicos.
2. Verificar visualmente que el tablero queda completo, enfocado y no saturado
   en RGB; no mover la cámara ni el brazo durante la toma.
3. Adquirir al menos 20 capturas estáticas, separadas en el tiempo, con el
   tablero ocupando aproximadamente 15--40 % del ancho de imagen. Repetir la
   colocación del útil y añadir 10 capturas para comprobar repetibilidad.
4. Para cada captura, detectar todas las esquinas/IDs, estimar pose y expresar
   el resultado en `world -> d435i_depth_optical_frame` mediante la composición
   indicada. Rechazar automáticamente detecciones parciales, borrosas o con
   reproyección alta.
5. Calcular mediana de traslación y rotación, dispersión y error de
   reproyección; no aceptar una sola toma.
6. Tras la aceptación y una autorización posterior, publicar TF de prueba y
   validar en RViz que una nube/axes del tablero coincide con el tablero real.
   Esta Fase 6A no realiza ese paso.

## Criterios de aceptación propuestos

- Al menos 20 capturas válidas y al menos 90 % de detecciones esperadas.
- Error de reproyección mediano <= 0.5 px; ninguna captura aceptada > 1.0 px.
- Dispersión entre capturas <= 3 mm en traslación y <= 0.5 grados en rotación.
- Repetición tras desmontar/remontar el útil: diferencia <= 5 mm y <= 1 grado.
- En la futura validación RViz, ejes y geometría del tablero coinciden sin
  inversión de eje, reflexión ni desplazamiento apreciable (> 1 cm).

## Riesgos y detección

| Riesgo | Detección |
|---|---|
| Tablero flexible, tamaño impreso incorrecto o soporte no registrable | Error de reproyección alto o mala repetibilidad tras remontaje |
| Confundir marco RGB y profundidad | Desplazamiento sistemático de la nube respecto al tablero en RViz |
| Convención/inversión de pose errónea | Ejes orientados hacia atrás, reflexión o traslación de orden grande |
| Pocas esquinas, desenfoque o reflejos | Bajo porcentaje de detección y dispersión alta |
| Usar relación tablero-base manual | Resultado no reproducible; incumple el criterio de referencia mecánica |
| Integración prematura con seguridad | Detectar cualquier uso de `/thesis/obstacle_input`, supervisor o auto-filtro; debe bloquearse |

## Información faltante antes de calibrar físicamente

1. Plano o dimensión nominal verificable del adaptador que registre el tablero
   respecto a la referencia de base/JACO2 y la definición exacta de
   `world -> calibration_board` en ese útil.
2. Diccionario del marcador, fichero imprimible y medida real de cuadrados/
   tags tras imprimir.
3. Confirmación de disponibilidad local de un detector ChArUco/ArUco (o,
   alternativamente, AprilTag) y sus dependencias ROS/C++ antes de implementar.
4. Confirmación de los tópicos RGB/CameraInfo que se usarán y de la
   transformación interna color–profundidad de la D435i durante la futura
   sesión.
5. Autorización explícita posterior para implementar el detector, tomar las
   capturas y publicar un TF de validación.

## Decisión

**BLOCKED_MISSING_MARKER_OR_DEPENDENCY.** El diseño y el método son
reproducibles, pero falta el útil marcador registrable y no hay una dependencia
de detector visual declarada en `thesis_jaco_perception`. No se continúa a
implementación ni a calibración física.
