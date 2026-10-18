# Cierre validado A-02a — frame, STOP upstream y TF temporal

Fecha: 2026-10-18  
Estado: **A-02a VALIDADA**

La validación offline finalizó con **23/23 pruebas aprobadas** usando
`python3 -m pytest` en el entorno ROS cargado desde `/opt/ros/humble` e
`install/setup.bash`.

El resultado cubre los interbloqueos ya implementados: `reference_frame=world`
estricto, rechazo de frame inválido, STOP upstream prioritario, consulta TF al
sello de adquisición y STOP seguro ante TF ausente, vencido o fallido.

No se modificó código ni configuración durante esta validación, ni se inició
ningún componente ROS o físico.

**A-02b permanece BLOQUEADA POR CONTRATO**: falta una fuente sincronizada de
estado articular y una relación explícita modelo→`world` para comparar TF con
el modelo y aplicar `kinematic_tf_tolerance_m`.

Evidencia completa:
[2026-10-18_validacion_final_a02a_frame_tf.md](/home/luis/Escritorio/TESIS_2_IMPLEMENTATION/evidence/codex_reports/2026-10-18_validacion_final_a02a_frame_tf.md).

