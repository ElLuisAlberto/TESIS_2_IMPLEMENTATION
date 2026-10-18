# Corrección acotada de espera TF A-02a

Fecha: 2026-10-18  
Resultado: **VALIDADA OFFLINE**

## Alcance y archivos modificados

- `src/thesis_core/thesis_core/proximity_monitor_node.py`
- `src/thesis_core/test/test_a02a_frame_tf_interlock.py`
- `src/thesis_core/test/test_proximity_tf_timeout.py` (nuevo)

No se modificaron mensajes ROS, supervisor, readiness, IK, GUI, hardware,
cámara, launch ni parámetros de simulación. No se inició ROS, Gazebo, GUI,
RViz, cámara o hardware y no se publicaron comandos de movimiento.

## Cambio aplicado

Se añadió al monitor el parámetro ROS:

| Parámetro | Predeterminado | Restricción |
|---|---:|---|
| `tf_lookup_timeout_sec` | `0.04` s | positivo, finito y no mayor que el período de evaluación |

El monitor se configura por defecto a `evaluation_rate_hz=20.0`, cuyo período
es 50 ms. El valor predeterminado de 40 ms deja margen antes de la siguiente
evaluación. Se construye un único límite temporal al comenzar cada evaluación;
cada consulta de segmento recibe solamente el tiempo restante. Así no se
acumula una espera de 40 ms por frame ni se bloquea indefinidamente.

La llamada TF sigue siendo:

```python
lookup_transform(reference_frame, frame_name, Time.from_msg(source_stamp),
                 timeout=remaining_timeout)
```

Por tanto el sello solicitado continúa siendo exactamente el `source_stamp`
del obstáculo. No se consulta `Time()`/última TF como alternativa, no se
transforman frames distintos de `world`, y al expirar el tiempo se conserva la
salida existente:

```
state=STOP
limiting_segment=TF_UNAVAILABLE
```

La política de frame `world`, frescura de obstáculo/sello y prioridad de STOP
upstream no fueron alteradas.

## Cobertura offline

Se añadieron o ajustaron pruebas para verificar:

1. TF exacta disponible de inmediato: devuelve el punto con el sello exacto.
2. TF exacta entregada por TF2 durante un timeout positivo: se acepta sin
   convertirla en STOP por carrera temporal.
3. Timeout o excepción de TF: `evaluate()` publica únicamente
   `STOP/TF_UNAVAILABLE`.
4. Búfer con sólo TF más reciente: se solicita el sello de adquisición y la
   excepción se propaga; no existe fallback a TF latest.
5. A-02a mantiene frame inválido como indisponibilidad y STOP upstream como
   prioritario.
6. Frescura continúa rechazando sellos cero, vencidos y futuros.

Comandos de validación:

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
colcon build --packages-select thesis_core --event-handlers console_direct+

python3 -m pytest -q \
  src/thesis_core/test/test_proximity_tf_timeout.py \
  src/thesis_core/test/test_a02a_frame_tf_interlock.py \
  src/thesis_core/test/test_source_freshness.py \
  src/thesis_core/test/test_obstacle_contract.py
```

Resultado de compilación: `thesis_core` compilado correctamente (1 paquete).

Resultado de pruebas: **27 aprobadas, 0 fallidas** en `0.16 s`.

Nota de ejecución: una primera corrida de pytest antes de compilar cargó el
módulo instalado anterior y falló por la firma previa de `_frame_point`; no fue
una regresión del cambio. Tras compilar y recargar el entorno con `set +u`, la
suite final anterior pasó íntegramente.

## Limitación

La corrección está validada offline; no se ha repetido la prueba funcional
Gazebo X/Y/Z por petición expresa. La espera no relaja A-02a: si la TF exacta
no se incorpora al búfer dentro del presupuesto, la decisión sigue siendo
fail-closed.
