# Validación de ruta manual cartesiana de GUI — simulación

Fecha: 2026-10-18  
Estado: **VALIDADA EN PRUEBAS OFFLINE**

## Alcance y archivos modificados

Se leyó `AGENTS.md` y el diagnóstico de ruta cartesiana. Se modificó solamente:

- `src/thesis_ui/test/test_gui_state.py`

No se modificaron `joint_control_gui.py`, los tres adaptadores cartesianos,
mensajes ROS, límites, watchdogs, IK, supervisor ni ninguna configuración.
No se inició ROS, Gazebo, RViz, GUI real, cámara ni hardware; no se publicó
ningún comando a un robot.

## Cobertura añadida

La regresión headless ahora captura mensajes ROS reales emitidos por la GUI y
usa dobles de TF/estado/IK para los callbacks de adaptadores.

- JOG: X/Y/Z/Roll/Pitch/Yaw, ambos signos, `world`, m/s para lineal, °/s→rad/s
  para angular y expiración a cero tras 0.25 s.
- Objetivo puntual: seis ejes, incrementos ±, conversión °→rad, publicación de
  `CartesianCommand`, `frame_id=world` y pose absoluta basada en la pose
  medida copiada.
- Trayectoria: seis ejes y signos, conversión de unidades, `RELATIVE` y
  `ABSOLUTE`, y publicación de `CartesianTrajectoryCommand` en `world`.
- Adaptadores: salida `JointCommand` válida para punto, JOG y trayectoria;
  rechazo sin salida de movimiento para frame distinto de `world` y estado
  ausente; HOLD articular para JOG con TF o IK fallidos y velocidad inválida;
  rechazo seguro de TF, IK y límite de desplazamiento en trayectoria.

Los rechazos del adaptador puntual y de trayectoria no publican un nuevo
`JointCommand` porque no hay una trayectoria activa que detener; esto es el
contrato existente de no generar movimiento. El JOG, que es continuo, publica
la postura actual como HOLD ante fallo de TF/IK o velocidad inválida.

## Validación

Entorno:

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
```

Compilación:

```bash
colcon build --packages-select thesis_ui thesis_simulation
```

Resultado: correcto, `2 packages finished`. Colcon mostró únicamente la
advertencia habitual de overlay para `thesis_ui` ya presente en `install/`.

Regresión final ejecutada:

```bash
python3 -m pytest -q \
  src/thesis_ui/test/test_gui_state.py \
  src/thesis_simulation/test/test_cartesian_jog_model.py \
  src/thesis_simulation/test/test_cartesian_trajectory_model.py
```

Resultado exacto:

```text
......................................                                   [100%]
38 passed in 0.87s
```

Resultado final: **38 aprobadas, 0 fallidas**.

## Conclusión

La ruta manual cartesiana de la GUI queda **VALIDADA EN SIMULACIÓN OFFLINE**
para los contratos cubiertos: mapeo de ejes/signo/unidad/frame, modos de pose y
trayectoria, watchdog de JOG, conversión a intención articular y fallos
fail-safe del adaptador.

Esto no valida ejecución física ni elimina los límites ya documentados de
singularidad, alcance de IK, TF disponible o A-02b (coherencia completa
TF--modelo), que permanecen fuera de este alcance.

