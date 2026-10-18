# Evidencia final de validación — A-02a frame, STOP upstream y TF temporal

Fecha: 2026-10-18  
Estado: **APROBADA**

## Entorno usado

Desde la raíz del workspace se cargó únicamente el entorno solicitado:

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
```

Comprobación del corredor:

```text
$ python3 -m pytest --version
pytest 6.2.5
```

No se modificaron código, configuración, PATH ni PYTHONPATH de forma
persistente. No se inició ROS, Gazebo, cámara ni hardware y no se publicaron
comandos.

## Pruebas ejecutadas

```bash
python3 -m pytest -q \
  src/thesis_core/test/test_a02a_frame_tf_interlock.py \
  src/thesis_core/test/test_source_freshness.py \
  src/thesis_core/test/test_obstacle_contract.py
```

Resultado:

```text
.......................                                                  [100%]
23 passed in 0.18s
```

La suite cubre admisión de `world`, rechazo de frames vacío/ajeno, prioridad
de STOP upstream, consulta TF al sello de adquisición, TF ausente/vencido/con
fallo de transformación y la ruta de STOP `TF_UNAVAILABLE`; las dependencias
directas conservan la frescura de sellos y el contrato de obstáculo.

## Estado resultante

- A-02a: **VALIDADA** por pruebas unitarias offline (23/23).
- A-02b: **BLOQUEADA POR CONTRATO**. No se implementó la comparación completa
  TF--modelo ni el uso de `kinematic_tf_tolerance_m` contra el modelo
  articulado.

