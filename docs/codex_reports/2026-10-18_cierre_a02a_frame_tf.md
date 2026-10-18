# Cierre A-02a — interbloqueo de frame, STOP upstream y TF temporal

Fecha: 2026-10-18  
Estado: **NO VALIDADA — DETENIDA POR ENTORNO DE PRUEBAS**

## Cambios realizados

Se implementó el subconjunto A-02a en `thesis_core`:

- admisión exclusiva de `reference_frame=world` para proximidad;
- rechazo estructurado de frame inválido como indisponibilidad;
- prioridad irreversible de STOP upstream antes de la predicción;
- consultas TF al sello de adquisición de la medida;
- STOP `TF_UNAVAILABLE` si TF falta, está vencido para ese sello o falla la
  transformación;
- restricción de los frames de seguridad del monitor y bridge a `world`.

También se añadió una prueba directa para frame válido/inválido, STOP upstream,
consulta TF al sello de adquisición, TF ausente/vencido/fallido y publicación
de STOP sin `ALLOW`.

## Validación

`colcon build --packages-select thesis_core` pasó correctamente.

Las pruebas A-02a y sus dependencias directas no llegaron a ejecutarse porque
el entorno ROS cargado no tenía el ejecutable `pytest` en PATH. Por la regla de
detención no se instaló ni ajustó el entorno, ni se intentó un corredor
alternativo.

Por ello, aunque el cambio está **IMPLEMENTADO**, A-02a no puede marcarse como
**VALIDADA** todavía.

## A-02b pendiente

La comparación completa TF--modelo y el uso efectivo de
`kinematic_tf_tolerance_m` siguen **BLOQUEADOS POR CONTRATO**. Requieren una
fuente de posiciones articulares sincronizada y una transformación explícita
del origen del modelo a `world`; no se introdujo esa topología en A-02a.

Evidencia completa:
[2026-10-18_validacion_a02a_frame_tf.md](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/evidence/codex_reports/2026-10-18_validacion_a02a_frame_tf.md).

