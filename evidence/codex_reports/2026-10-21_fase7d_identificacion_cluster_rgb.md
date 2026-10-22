# Fase 7D: inspección RGB del cluster externo

## Estado

`RGB_INSPECTION_READY`

## Tópico y perfil

La D435i publica RGB en `/camera/d435i/color/image_raw`. Se creó el perfil
aislado `src/thesis_jaco_perception/rviz/d435i_rgb_inspection.rviz` con un
único display `Image` para ese tópico. Su QoS es `BEST_EFFORT`, `VOLATILE`,
`KEEP_LAST` y profundidad `5`.

## Ventanas activas

Se abrió la segunda ventana nativa RViz `TESIS_RVIZ_RGB_INSPECTION` con el
perfil RGB. Permanecen abiertas, sin reinicio, la sesión de sombra epsilon
`0.07`, la ventana de alineación 3D existente y la nueva ventana RGB para que
el operador identifique visualmente el objeto asociado al bloque rojo externo.

## Seguridad y alcance

No se modificaron TF, epsilon, rango de 1.2 m, YAML, código, launch, cápsulas,
CMake ni el perfil RViz de alineación. `hardware_output=false` continúa en el
launch activo. No se enviaron comandos al JACO2 y no se iniciaron supervisor,
obstacle bridge, extractor, GUI física, Gazebo ni nodos de mando. No se usó
sudo ni se instalaron paquetes.
