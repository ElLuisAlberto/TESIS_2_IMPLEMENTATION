# Auditoría local: integración RGB-D D435i con el supervisor preventivo JACO2

Fecha: 2026-10-21
Alcance: inspección estática local de archivos y `git status`/`git diff`; no se inició ningún proceso ROS, Gazebo, RViz, cámara ni hardware, no se compiló y no se enviaron comandos al JACO2.

## Dictamen

La base de integración segura ya existe, pero la D435i **no debe conectarse aún** a `/thesis/obstacle_input`. El candidato actual es deliberadamente aislado y no dispone de auto-filtrado del JACO2 ni de una extrínseca física calibrada. Con el brazo físico, Gazebo no participa en la ruta propuesta ni se considera fuente de estado o geometría.

El riesgo dominante no es publicar una nube: es confundir superficie propia y entorno. Un filtro que borre por una cápsula exterior puede eliminar un obstáculo en contacto o muy cercano. Por ello, la propuesta solo descarta retornos con evidencia positiva de pertenecer al brazo; los ambiguos se preservan y se informan. Esto prioriza evitar falsos negativos, aunque puede dejar residuo propio y causar falsos positivos/reducción preventiva.

## A. Inventario real

### Percepción (`thesis_perception`)

| Nodo | Entrada / QoS | Salida / QoS | Marco y tiempo | Función |
|---|---|---|---|---|
| `pointcloud_preprocessor_node` | `/camera/d435i/depth/color/points`, `SensorDataQoS`, profundidad 1 | `/thesis/perception/points_filtered`, mismo QoS | Conserva `header.stamp` salvo `restamp_output`; conserva marco si `target_frame=""` | Elimina NaN y rango radial, crop box opcional y voxel de 0.02 m por defecto. Si hay `target_frame`, transforma por TF; por defecto usa TF más reciente, no el del sello. |
| `obstacle_extractor_node` | nube filtrada, `SensorDataQoS`, profundidad 1 | `/thesis/perception/obstacle_candidate` (`Obstacle`, reliable 10), diagnósticos y marcador reliable 10 | Copia el `header` de la nube | Agrupación euclídea; publica **un único** clúster, el de menor distancia superficie-origen. Radio = máximo radio del AABB-centro; incertidumbre fija 0.03 m; velocidad suavizada entre candidatos. |
| `perception_health_node` | nube, QoS sensor profundidad 10; IMU opcional | `/thesis/perception/diagnostics`, reliable 10 | Comprueba frame esperado, edad/sello, frecuencia, timeout y número de puntos | Por defecto IMU no requerida; nube mínima 10 Hz, timeout/edad 0.5 s. |

`perception_processing.launch.py` expone los tres nodos y, solo bajo opt-in, publica `world -> d435i_link` estático. Sus valores son plantilla cero y `publish_static_extrinsic=false`. `d435i_standalone.launch.py` habilita la nube RealSense (640x480@15 Hz), QoS `SENSOR_DATA`, decimación, filtros espacial y temporal; sigue separado del supervisor. El test `test_perception_safety_contract.py` protege explícitamente que el candidato no sea `/thesis/obstacle_input`.

El contrato actual es:

```text
Obstacle
  header.stamp     tiempo de adquisición; obligatorio y no cero en el bridge
  header.frame_id  marco de center y velocity; obligatorio
  center, velocity  geometría en ese marco
  radius > 0; uncertainty >= 0
```

### Núcleo preventivo (`thesis_core`)

| Componente | Ruta / comportamiento comprobado |
|---|---|
| `obstacle_input_bridge` | Recibe `/thesis/obstacle_input`, valida finitud, radio, incertidumbre y edad (máx. 0.5 s; futuro máx. 0.05 s). Consulta TF `world <- frame_origen` **al sello de adquisición** y publica `/thesis/obstacle_in_model_frame`. Rechaza, sin reutilizar, TF o muestra inválidos. |
| `proximity_monitor` | En modo `topic` recibe el anterior, suma `radius + uncertainty`, obtiene los extremos TF de las seis cápsulas al mismo sello y publica `/thesis/proximity_status` a 20 Hz. TF incompleto produce `STOP` con `TF_UNAVAILABLE`; obstáculo vencido deja de aportar estado. Solo acepta `world`. |
| `horizon_clearance` | Propaga linealmente el centro por velocidad para cada muestra; calcula distancia superficie-superficie cápsula-esfera. Margen protector por defecto: modelo 0.010 + muestreo 0.005 + latencia 0.005 = 0.020 m. |
| `safety_supervisor` | Consume `ProximityStatus`; monitor de ejecución: horizonte 1.0 s, 21 muestras, 10 Hz. Usa cinemática directa y cápsulas para evaluar la trayectoria e impone la política preventiva. |
| `horizon_preview` | Publica `/thesis/joint_trajectory_prediction` y visualización; por defecto 1.0 s, 21 muestras. Es observación/preview, no fuente de decisión. |
| `jaco_kinematics` | FK analítica de seis articulaciones y seis ejes de cápsulas en `world`; nombres y radios canónicos: 0.105, 0.105, 0.085, 0.075, 0.085, 0.090 m. |

Las cápsulas usan los segmentos `base_to_shoulder` a `wrist_2_to_tool`. Su configuración TF está en `jaco_capsules.yaml`, entre `j2n6s300_link_base`, `j2n6s300_link_1...5` y `j2n6s300_end_effector`. La coherencia estricta entre este YAML y `jaco_kinematics.CAPSULE_RADII` ya se verifica al iniciar el monitor.

### Descripción y hardware

`thesis_description` aporta mallas visuales DAE y Xacro para base, hombro, brazo, antebrazo, muñecas, mano y dedos. El árbol del JACO2 parte de `world -> root -> j2n6s300_link_base -> ... -> j2n6s300_link_6 -> j2n6s300_end_effector`; las seis articulaciones se llaman `j2n6s300_joint_1...6`. Las geometrías declaradas son visuales; no hay una representación de colisión calibrada para clasificar nube propia.

El adaptador físico publica `/joint_states` reliable a 20 Hz, `/thesis/hardware/{connected,armed,arm_epoch,diagnostics}` y el árbol TF se deriva mediante `robot_state_publisher` del Xacro y esos estados. Su configuración deja `hardware_output_enabled=false`. `hardware_readiness` exige conexión, articulaciones frescas y `ProximityStatus` fresco en `world` (además de armado si se configura); no evalúa salud de cámara ni la calidad del auto-filtrado. El launch físico configura `obstacle_source_mode=topic`; Gazebo queda fuera de esa ruta.

Interfaces pedidas:

| Mensaje | Campos y papel |
|---|---|
| `Obstacle` | Header, esfera (centro, radio), velocidad e incertidumbre; entrada normalizada del monitor. |
| `ProximityStatus` | sello y `reference_frame`, estado `ALLOW/WARNING/REDUCTION/STOP`, clearance, segmento limitante, puntos testigo y obstáculo usado. |
| `JointTrajectoryPrediction` | sello, sello de origen, fuente, nombres articulares,  número/muestreo/horizonte/duración y posiciones planas por muestra. Es publicado por preview. |

## B. Rutas exacta actual y propuesta

Ruta actual de la D435i (aislada):

```text
/camera/d435i/depth/color/points [PointCloud2]
  -> pointcloud_preprocessor_node
  -> /thesis/perception/points_filtered [PointCloud2]
  -> obstacle_extractor_node
  -> /thesis/perception/obstacle_candidate [Obstacle]
```

Ruta actual del físico con una fuente ya validada (por defecto no hay D435i conectada):

```text
/thesis/obstacle_input [Obstacle]
  -> obstacle_input_bridge (validación + TF a world al sello)
  -> /thesis/obstacle_in_model_frame [Obstacle/world]
  -> proximity_monitor (cápsulas TF al sello)
  -> /thesis/proximity_status [ProximityStatus]
  -> safety_supervisor + hardware_readiness + jaco_hardware_adapter

/joint_states -> robot_state_publisher -> /tf -> proximity_monitor/supervisor
```

Ruta propuesta, inicialmente en modo sombra y sin remap a la entrada de seguridad:

```text
D435i PointCloud2 --(sello cámara)--> preprocessor
  --(world, TF exacto; sin restamp)--> /thesis/perception/points_world
  -> jaco_self_filter [estado articular/TF exactos + modelo JACO calibrado]
  -> /thesis/jaco_perception/points_external_candidate
  -> extractor de candidatos en sombra
  -> /thesis/jaco_perception/obstacle_shadow [Obstacle/world]
  -> comparador/telemetría sombra

Solo tras criterios de aceptación:
... -> extractor validado -> /thesis/obstacle_input -> cadena física existente
```

La transformación nube-a-`world` debe pedir TF al sello de la nube (`use_latest_transform=false`), mantener `restamp_output=false` y depender de una extrínseca medida. La D435i fija no requiere IMU para esta geometría.

## C. Reutilización sin cambios

- `Obstacle`, `obstacle_contract.py` y `obstacle_input_bridge.py` para contrato, frescura y transformación temporal.
- `proximity_monitor.py`, `clearance_geometry.py`, `jaco_kinematics.py`, `horizon_clearance.py`, `safety_supervisor_node.py` y las cápsulas actuales para la decisión preventiva.
- `robot_state_publisher`, `/joint_states`, `/tf`, diagnóstico y readiness físico existentes.
- Preprocesado genérico (NaN/rango/voxel/crop) y salud D435i, después de fijar parámetros físicos validados.
- `ObstacleExtractor` como punto de partida **solo después** de que su entrada sea nube externa; hoy no resuelve múltiples obstáculos ni filtro propio.

## D–E. Componentes nuevos y ubicación

Crear un paquete nuevo, propuesto como `src/thesis_jaco_perception/`. Mantiene `thesis_perception` genérico y evita mezclar percepción específica con el nodo de salida física de `thesis_hardware`.

| Componente | Ubicación recomendada | Responsabilidad |
|---|---|---|
| `jaco_self_filter_node` | `src/thesis_jaco_perception/src/` | Clasificación conservadora nube propia/externa usando pose real, geometría y cámara. |
| biblioteca `jaco_self_filter_geometry` + pruebas | mismo paquete | Ray-casting, distancia a cápsula/malla, sincronización temporal y cálculo de incertidumbre; sin ROS en el núcleo testeable. |
| `jaco_d435i_calibration.yaml` | `src/thesis_jaco_perception/config/` | Extrínseca medida, intrínsecos/óptica, tolerancias, versión y método de calibración; no valores plantilla. |
| launch `jaco_rgbd_shadow.launch.py` | `src/thesis_jaco_perception/launch/` | Pipeline sombra explícito; no incluye ni remapea el launch físico. |
| `self_filter_diagnostics` | producido por el filtro | Conteos propio/externo/ambiguo, edad, TF, estado y presupuesto de incertidumbre. |
| adaptador de candidatos múltiples (si se aprueba) | mismo paquete | Selección/persistencia de varios obstáculos antes del contrato actual de una esfera, o evolución versionada del contrato. |
| inclusión opcional, posterior, de launch físico | `thesis_hardware/launch/jaco_physical_system.launch.py` | Solo una composición con argumentos `shadow_only` y compuertas; nunca lógica perceptiva específica. |

## F. Diseño preciso: `jaco_self_filter_node`

### Entradas y salidas

- Entrada primaria: `sensor_msgs/PointCloud2` ya expresada en `world`, `/thesis/perception/points_world`; QoS sensor, `keep_last(1)`.
- Entradas de estado: `/joint_states` y TF de `world` a los enlaces JACO. El filtro usa una muestra articular completa con sello no posterior al de la nube y TF/interpolación exacta al sello de adquisición. No usa pose de Gazebo.
- Configuración inmutable en ejecución: extrínseca medida `world <- d435i_optical_frame`, modelo de oclusión/cápsula o malla de auto, tolerancias y límites de edad.
- Salida: `/thesis/jaco_perception/points_external_candidate`, `PointCloud2`, `frame_id=world`, **mismo `header.stamp` de la nube**. No se publica una nube re-sellada.
- Salidas auxiliares: nube de auto confirmada, nube ambigua conservada, marcador/modelo y `diagnostic_msgs/DiagnosticArray`. Ninguna salida será `/thesis/obstacle_input` en fases iniciales.

### Geometría y regla de clasificación

1. A partir de los seis joints al sello, reconstruir el modelo JACO en `world`. Inicialmente se pueden usar cápsulas calibradas para descarte interior; para retirar con utilidad las superficies visibles se requiere un modelo de oclusión ray-cast de malla/volumen de colisión calibrado, no las mallas visuales sin validar como verdad geométrica.
2. Transformar el punto a `d435i_optical_frame`, proyectar su rayo y ray-castear el **primer** cruce esperado del JACO contra ese rayo. Considerar solo partes que sean visibles desde la cámara (z-buffer); nunca una parte auto-oculta.
3. El punto se clasifica `SELF_CONFIRMED` solo si coincide con el primer cruce previsto dentro de una banda conservadora de profundidad y registro. Puntos detrás del primer cruce son `OCCLUDED_UNKNOWN`, no externos confirmados ni auto descartables.
4. Si un punto está en la banda donde un obstáculo puede tocar o estar delante de la superficie del brazo, clasificar `AMBIGUOUS` y conservarlo en la nube que llega al extractor. El filtro no puede desambiguar ese caso solamente con una vista de profundidad.
5. Como salvaguarda independiente, se permite eliminar puntos estrictamente dentro de un volumen propio interior certificado: cápsula con radio `r_capsule - epsilon_total > 0`. Este descarte es muy seguro pero no basta para eliminar la piel observada; se registra separado.

`epsilon_total` debe sumar, como mínimo y con unidades explícitas: incertidumbre de profundidad dependiente de rango/ángulo, error intrínseco, error extrínseco, error TF/joint temporal, error de modelo/malla y discretización de voxel. La banda de coincidencia se calibra y versiona; debe crecer al aumentar rango, incidencia oblicua, edad o latencia. Si no existe cota válida, no se elimina ningún punto por el modelo.

### Sellos, freshness y fallos

| Condición | Salida / diagnóstico requerido |
|---|---|
| nube sin sello, vencida, futura o frame distinto de `world` | No promover salida externa; diagnóstico ERROR `CLOUD_INVALID`. |
| TF `world <- cámara` o TF/link JACO ausente al sello | No filtrar/publicar como externa; ERROR `TF_AT_STAMP_UNAVAILABLE`. Nunca TF más reciente. |
| `JointState` incompleto, no sincronizable o vencido | No filtrar/promover; ERROR `JOINT_STATE_STALE_OR_UNAVAILABLE`. |
| intrínsecos/extrínseca/modelo no cargados, no validados o presupuesto no finito | No filtrar/promover; ERROR `CALIBRATION_INVALID`. |
| nube válida pero punto ambiguo/ocluido | Conservarlo, contabilizarlo; WARN según fracción. No tratarlo como auto. |
| pérdida de cámara/procesamiento atrasado | La cadena futura debe perder frescura y quedar en `NOT_READY`/STOP; no se reemite la última detección. |

Para una futura conexión al supervisor, además del filtro se requiere una compuerta de salud de percepción que publique una condición de disponibilidad y que el readiness/arm interlock la exija. Hoy `hardware_readiness` no suscribe diagnósticos de D435i: debe ampliarse solo después de validar el modo sombra.

## G. Riesgos y mitigaciones

| Riesgo | Efecto | Mitigación requerida |
|---|---|---|
| Oclusión por el brazo | El entorno detrás del brazo es inobservable; no debe declararse libre. | z-buffer y estado `OCCLUDED_UNKNOWN`; no eliminar ni inferir ausencia de obstáculo. |
| Sobre-filtrado | Falso negativo si el obstáculo roza la cápsula. | Regla de evidencia positiva, banda ambigua conservada, pruebas de contacto y margen de incertidumbre. |
| Latencia 15 Hz + filtros + PCL | Posición/velocidad del obstáculo desfasada frente a la predicción de 1 s. | Presupuestar y medir extremo a extremo; sello original, `keep_last(1)`, sin restamp, aumentar incertidumbre o bloquear promoción si supera presupuesto. |
| Desalineación cámara–world | Eliminar entorno o ubicar esfera erróneamente. | Extrínseca metrológica, verificación con dianas/objetos conocidos, TF exacto al sello y recalibración tras mover soporte. |
| Nube propia residual | El extractor actual puede escoger el brazo como el clúster más cercano. | Auto-filtro de superficie, métrica de residuo y gate que impide promoción si el residuo/ambigüedad excede umbral. |
| Múltiples obstáculos | El extractor actual reduce todos a una sola esfera más cercana; puede ignorar otra relevante durante la trayectoria. | Confirmar estrategia: envolvente conservadora, top-K con evaluación multi-esfera, o evolución del contrato. No asumir que una esfera basta. |
| Filtro temporal RealSense | Puede introducir retardo o estelas. | Medir su contribución, comparar filtros on/off en sombra, incluirla en incertidumbre. |

## H. Plan por fases y criterios

1. **Especificación y calibración sin supervisor.** Medir/registrar extrínseca e intrínsecos, montar rígidamente la cámara, caracterizar profundidad y latencia. Pruebas unitarias puras: intersección rayo-cápsula/malla, primeras intersecciones, clasificación, propagación de incertidumbre, joints por nombre y rechazo de sellos/TF. Criterio: la calibración tiene trazabilidad y cotas numéricas; ningún valor cero plantilla.
2. **Filtro offline y replay.** Usar nubes y joint states grabados de posturas estáticas/movimiento lento, con objetos conocidos junto al brazo. Pruebas: brazo solo, objeto separado, objeto adosado, objeto detrás del brazo, bordes, dedos, nube vacía y pérdida de TF. Criterio: cero borrados de los blancos externos etiquetados en casos de contacto/proximidad; los ambiguos se conservan; no se publica como válida ante datos faltantes.
3. **Modo sombra conectado, salida aislada.** Ejecutar D435i y filtro, pero solo publicar `/thesis/jaco_perception/*` y telemetría; no remap a `/thesis/obstacle_input`, no armar. Medir p50/p95/p99 de edad nube-a-candidato, tasa de puntos por clase, residuo propio y estabilidad de candidatos. Criterio: presupuesto de latencia compatible con 0.5 s de frescura y margen revisado; no hay selección sistemática del brazo como obstáculo.
4. **Sombra comparativa de seguridad.** Alimentar un evaluador espejo que calcule `ProximityStatus`/horizonte pero no controla salida. Comparar contra geometría de referencia y escenarios físicos pasivos. Criterio: cada pérdida de cámara, TF, joint state o calibración produce indisponibilidad/STOP espejo antes de que venza el dato; casos cercanos no presentan falsos negativos observados.
5. **Promoción limitada y revisión.** Tras aprobar métricas, añadir compuerta de salud al readiness y promover mediante un launch explícito, todavía con salida física desarmada. Revisar registros. Solo una autorización posterior que confirme operador, área despejada y parada de emergencia permite una prueba física gradual; el primer movimiento debe ser mínimo y lento, con confirmación por etapa.

Antes de conectar al supervisor se exige, como mínimo: extrínseca validada, modelo de auto validado, trazabilidad de incertidumbre, p99 de edad dentro del presupuesto aprobado, pruebas de degradación fail-closed aprobadas, evidencia de objetos cercanos conservados y decisión resuelta para múltiples obstáculos.

## I. Decisiones que requieren confirmación

1. Marco físico definitivo: ¿`world` coincide con la base física y se autoriza fijar/publicar la extrínseca medida `world -> d435i_*`?
2. ¿Cuál es la ubicación, orientación, rigidez y campo de visión definitivos de la D435i, y se dispone de intrínsecos/calibración verificable?
3. ¿Qué precisión/límite de latencia aceptable se asigna a profundidad, registro, TF y procesamiento para fijar `epsilon_total` y el margen del supervisor?
4. ¿Se acepta que regiones auto-ocultas y de contacto permanezcan ambiguas (conservadas) en vez de intentar borrarlas?
5. ¿El requisito operativo admite una única esfera o debe representar simultáneamente múltiples obstáculos? Esta decisión define si se conserva o evoluciona `Obstacle`/el monitor.
6. ¿Qué objetos, distancias mínimas y posturas constituyen el conjunto de validación etiquetado y los umbrales objetivos de falso negativo/falso positivo?
7. ¿Se autoriza crear el nuevo paquete específico `thesis_jaco_perception` y, tras modo sombra, ampliar `hardware_readiness` para exigir salud de percepción?
8. ¿Qué proceso documentará recalibración tras mover cámara, brazo, base o soporte?

## Cambios locales inspeccionados

`git status --short` no devolvió archivos modificados ni sin seguimiento y `git diff --name-only` no devolvió diferencias. Por tanto, en esta auditoría no hay cambios locales no subidos que afecten `thesis_perception`, `thesis_core`, `thesis_description`, `thesis_hardware`, `thesis_hardware_bridge` o `thesis_interfaces`.
