# Escenarios de obstáculo virtual esférico para JACO2

Fecha: 2026-10-19
Alcance: preparación y validación sin armado ni movimiento físico.

## Resultado

Se amplió el emisor existente `obstacle_demo_source`; no se creó otro
pipeline. El mismo mensaje `Obstacle` atraviesa
`obstacle_demo_source -> obstacle_input_bridge -> proximity_monitor` y
representa una única esfera estacionaria.

La esfera tiene radio geométrico configurable de `0.10 m` y margen de
incertidumbre de `0.02 m`. El radio efectivo empleado por proximidad y por la
predicción es por tanto `0.12 m`. El frame común es `world` y la publicación
nominal es de `10 Hz`.

## Cálculo reproducible

Estado articular físico medido usado para la calibración, en radianes:

```text
[-1.551814113493139, 2.7496557685950496, 4.1168651479621845,
 5.844626591509698, 5.973422388148109, 2.0808752988032992]
```

La trayectoria de referencia se calculó de manera local, sin publicar un
candidato: J1 `+3 deg`, duración `1.0 s`, 25 muestras. Para cada muestra se
evaluaron las seis cápsulas y sus radios configurados
`[0.105, 0.105, 0.085, 0.075, 0.085, 0.090] m`. Se conservaron los umbrales
existentes: STOP `0.05 m`, REDUCTION `0.15 m` y WARNING `0.30 m`.

Las tres poses se obtienen con `derive_physical_scenario_poses()`. PARK se
busca sobre una rejilla reproducible de 0.10 m en +X, en el plano Z cuantizado
del extremo del brazo superior, y se elige el primer punto cuyo mínimo en todo
el horizonte supera WARNING por 1.45 m; con el estado medido resulta
`(2.0, 0.0, 0.65)`. Las poses STOP se obtienen desplazando
la esfera desde el punto medio de la cápsula `wrist_2_to_tool` una distancia
igual a radio de cápsula + radio efectivo de esfera + `0.015 m` de separación
positiva, equivalente al 30% del umbral STOP configurado. Los normales
documentados son `(azimut=195 deg, z=0.25)` para
`BLOCK_START` y `(azimut=10 deg, z=0.35)` para `INSERT_STOP`; fueron los
resultados deterministas de una búsqueda angular discreta contra todas las
cápsulas.

| Escenario | Centro en `world` (m) | Separación actual (m) | Mínimo proyectado (m) | Instante/muestra del mínimo | Entrada STOP | TTC volumen protector | TTC colisión | Estado |
|---|---|---:|---:|---|---:|---:|---:|---|
| PARK | `(2.0, 0.0, 0.65)` | 1.799150020 | 1.791544434 | 1.000 s, 24/24 | no existe | no existe | no existe | ALLOW |
| BLOCK_START | `(-0.254134455, 0.473147106, 0.787705975)` | 0.014999302 | 0.014999302 | 0.000 s, 0/24 | 0.000 s | 0.000 s | no existe | STOP |
| INSERT_STOP antes | PARK | 1.799150020 | 1.791544434 | 1.000 s, 24/24 | no existe | no existe | no existe | ALLOW |
| INSERT_STOP después | `(0.191637718, 0.567693309, 0.810205975)` | 0.040555118 | 0.014999995 | 1.000 s, 24/24 | 0.000 s | 0.805834 s | no existe | STOP |

`INSERT_STOP` reutiliza PARK mientras `insert_stop_active=false` y cambia a la
pose STOP únicamente al activar ese parámetro. La esfera queda espacialmente
calibrada contra el final de J1 `+3 deg`: su mínimo ocurre en la última muestra.
Al insertarse, el estado STOP es inmediato y no ambiguo; así, en la futura
prueba física el movimiento observable ocurre antes de la inserción, no entre
la inserción y el STOP. La separación siempre es positiva y no se presupone
contacto físico.

## Cambios mínimos

- `src/thesis_core/thesis_core/virtual_obstacle_scenarios.py`: cálculo y
  selección deterministas de la única esfera.
- `src/thesis_core/thesis_core/obstacle_demo_source.py`: parámetros dinámicos
  `scenario`, `insert_stop_active` y las tres poses; se conserva `FIXED` para
  compatibilidad.
- `src/thesis_core/config/virtual_obstacle_scenarios.yaml`: radio, margen,
  frame, estado de referencia y centros reproducibles.
- `src/thesis_hardware/launch/jaco_physical_system.launch.py`: carga de la
  configuración y argumentos `demo_obstacle_scenario` y
  `demo_insert_stop_active` sobre el nodo existente.
- `src/thesis_core/test/test_virtual_obstacle_scenarios.py`: estados
  geométricos, transición explícita y coincidencia entre cálculo/configuración.

No se modificaron el supervisor, sus umbrales, los márgenes protectores, el
watchdog ni el comportamiento fail-closed.

## Compilación y pruebas

```text
colcon build --packages-select thesis_core thesis_hardware --symlink-install
  resultado: correcto

python3 -m pytest -q src/thesis_core/test/test_virtual_obstacle_scenarios.py
  4 passed

ament_flake8 (archivos nuevos/modificados de escenarios)
  sin problemas

ament_pep257 (archivos nuevos/modificados de escenarios)
  sin problemas

thesis_hardware
  12 passed, 1 skipped
```

La suite completa de `thesis_core` conserva 2 fallos de estilo anteriores y
ajenos a este cambio: W391 en `test_a02a_frame_tf_interlock.py:208` y D213 en
`safety_supervisor_node.py:114`. El resultado funcional es 169 pruebas pasadas,
1 omitida y esos 2 fallos de estilo preexistentes.

## Validación desarmada

Se reinició ordenadamente una sola sesión física para cargar la configuración.
La salida permaneció desarmada. RViz quedó activo y recibió las cápsulas, el
horizonte y la esfera. La evidencia numérica completa está en
`evidence/codex_reports/2026-10-19_evidencia_escenarios_obstaculo_virtual.md` y
el rosbag SQLite3 en
`evidence/rosbags/2026-10-19_virtual_obstacle_scenarios_disarmed/`.

Estado final: `PARK`, ALLOW, `system_ready=true`, `armed=false`, `mode=hold`,
`fault=false`, JOG 0 Hz y salida física 0 Hz.

## Veredicto

ESCENARIOS VIRTUALES LISTOS PARA PRUEBA FÍSICA STOP
