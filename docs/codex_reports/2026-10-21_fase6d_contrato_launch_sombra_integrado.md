# Fase 6D — Contrato de launch JACO + RGB-D + self-filter en sombra

Fecha: 2026-10-21. Se creó y probó un contrato de lanzamiento; no se inició
hardware, cámara, RViz, Gazebo, supervisor ni TF de ejecución.

## Contrato creado

Archivo principal:
`src/thesis_jaco_perception/launch/jaco_d435i_shadow_session.launch.py`.

Argumentos explícitos y seguros:

| Argumento | Valor por defecto | Función |
|---|---:|---|
| `start_jaco_read_only` | `false` | Incluye hardware solo cuando se solicite lectura |
| `start_camera` | `false` | Incluye la pila RGB-D aislada |
| `start_rviz` | `false` | Solo se propaga cuando se habilite cámara |
| `enable_self_filter` | `false` | Incluye filtro de sombra existente |
| `publish_session_extrinsic` | `false` | No crea TF de sesión por defecto |
| `candidate_extrinsic_file` | `""` | YAML candidato requerido para TF efímero |
| `hardware_output` | `false` | Se rechaza explícitamente si vale `true` |

El guard de launch rechaza un YAML inexistente, incompleto, no válido, no
aceptado para sesión, persistente, con marcos vacíos, con marcos distintos de
`world -> d435i_depth_optical_frame` o con vectores de transformación de tamaño
incorrecto. El TF solo se crea mediante `static_transform_publisher` cuando
`publish_session_extrinsic=true` y el YAML supera esa validación; al finalizar
el launch, ese proceso desaparece y no persiste nada.

## Topología documentada

```text
/joint_states + TF robot (lectura) ─┐
world -> d435i_depth_optical_frame ─┼─> nube world exacta ─> jaco_self_filter_node
/camera/d435i/depth/color/points ───┘                         ├─ self shadow output
                                                                ├─ external shadow output
                                                                └─ unknown/occluded shadow output
```

La nube raw se documenta como `/camera/d435i/depth/color/points`; la nube de
trabajo se define como `/thesis/perception/points_filtered` en `world`. El
contrato propaga `use_latest_transform=false`: no acepta fallback TF latest.
El marco de base es `world`, el de cámara es
`d435i_depth_optical_frame`, y los tres resultados pertenecen al nodo existente
`jaco_self_filter_node` en modo sombra.

## Garantía de aislamiento

El launch no contiene `obstacle_extractor`, `/thesis/obstacle_input`,
supervisor ni `jaco_self_filter_node` fuera de su inclusión condicionada. La
inclusión RGB-D fuerza `start_extractor=false`,
`publish_static_extrinsic=false`, `restamp_output=false` y `enable_crop=false`;
no modifica perfil, voxel o rango validados. No hay remapeos a rutas de
seguridad.

## Archivos modificados

- `src/thesis_jaco_perception/launch/jaco_d435i_shadow_session.launch.py`
- `src/thesis_jaco_perception/test/test_shadow_session_contract.py`
- `src/thesis_jaco_perception/CMakeLists.txt`
- `src/thesis_jaco_perception/package.xml`

## Pruebas

Se ejecutaron, sin hardware:

```text
colcon build --packages-select thesis_jaco_perception --symlink-install
colcon test --packages-select thesis_jaco_perception --event-handlers console_direct+
colcon test-result --all --verbose
ros2 launch thesis_jaco_perception jaco_d435i_shadow_session.launch.py --show-args
```

Resultado: compilación correcta y todas las pruebas del paquete afectado
correctas. La nueva prueba cubre defaults seguros, rechazo de
`hardware_output=true`, YAML ausente/no aceptado/persistente/inválido o con
marcos vacíos, ausencia de rutas a extractor/supervisor y ausencia de TF de
sesión por defecto.

## Antes de la primera sesión física temporal

Se requerirá una candidata YAML de sesión válida, una autorización operativa
separada, la confirmación de los marcos/temas efectivos del hardware en lectura
y la extrínseca temporal revisada. La sesión deberá activar explícitamente cada
argumento necesario; ningún valor por defecto inicia cámara, robot, RViz,
filtro ni TF.
