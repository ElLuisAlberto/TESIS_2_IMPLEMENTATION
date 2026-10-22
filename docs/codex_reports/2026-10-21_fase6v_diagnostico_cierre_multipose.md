# Diagnóstico del cierre fail-closed multipose

## Estado actual y evidencia

Los fuentes son legibles: `multipose_alignment.cpp` existe, modo `0664`, siete
líneas físicas, y contiene todo `estimate` en su línea 7. La asignación real
es `r.accepted=true; r.reason="TEMPORARY_SESSION_ONLY_NOT_FOR_SAFETY"` al
final de esa línea. El header declara `MultiPoseResult` en la línea 7;
`multipose_association.hpp` declara la correspondencia y
`associate_multipose`; `test_multipose_alignment.cpp` contiene en su línea 5
una prueba que espera `ASSERT_TRUE(r.accepted)`.

`test_temporary_alignment.cpp:5` exige correctamente rechazo con
`INSUFFICIENT_DISTINCT_POSES`. CMake registra ambos tests en líneas 100--103.

## Causa raíz del parche fallido

El parche intentó modificar
`src/thesis_jaco_perception/src/multipose_alignment.cpp` buscando el fragmento
aislado `r.accepted=true;r.reason="TEMPORARY_SESSION_ONLY_NOT_FOR_SAFETY";return r;} }`.
El texto real está embebido en una única línea minificada junto al cuerpo
completo de `estimate`; la coincidencia exacta falló por contexto de línea,
no por archivo, símbolo ni permisos. La expectativa original exacta no quedó
registrada fuera del error del parche: `PATCH_EXPECTATION_NOT_RECORDED`.

## Pruebas

El último CTest con salida literal disponible ejecutó 10 pruebas: 9 `PASS` y
`test_multipose_alignment` `FAIL_IMPLEMENTATION`: estimó
`(-0.06508064612016172, 0.030625837969948496, ...)` frente a
`(0.04, -0.03, 0.02, ...)`. La infraestructura compiló y las demás pruebas
pasaron. La ejecución solicitada de build/CTest se lanzó nuevamente en esta
iteración; no se inició ROS ni hardware. `git diff --check` previamente fue
correcto.

## Corrección mínima propuesta

1. Reescribir solo el bloque final de
`MultiPoseAlignmentEstimator::estimate` en `multipose_alignment.cpp:7` con
formato legible: comportamiento actual acepta incondicionalmente; el deseado
es `accepted=false`, razón `EXPERIMENTAL_NOT_VALIDATED`.
2. Cambiar `test_multipose_alignment.cpp:5`: comportamiento actual exige
aceptación; el deseado exige rechazo y esa razón. Riesgo: bajo; cierra una ruta
experimental sin afectar self-filter, supervisor, hardware, YAML ni TF.
3. Mantener `test_temporary_alignment.cpp:5` sin cambios.
