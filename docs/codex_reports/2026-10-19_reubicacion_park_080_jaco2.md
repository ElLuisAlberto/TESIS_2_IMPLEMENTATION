# Reubicación de PARK a 0.8 m

Fecha: 2026-10-19
Alcance: una esfera virtual, validación con el JACO2 desarmado y sin comandos.

## Resultado

La pose PARK de la esfera existente se cambió de `(2.0, 0.0, 0.65)` a
`(1.0, 0.0, 0.65)` en frame `world`. No se creó otro nodo, objeto o pipeline.
El radio geométrico continúa en `0.10 m`, la incertidumbre en `0.02 m` y el
radio efectivo evaluado en `0.12 m`.

| Métrica | PARK anterior | PARK nueva |
|---|---:|---:|
| Separación superficie–superficie actual | 1.799150020 m | 0.804254339 m |
| Mínimo proyectado J1 +3° | 1.791544434 m | 0.797032588 m |
| Mínimo proyectado J1 −3° | 1.799150020 m | 0.804254339 m |
| Peor margen sobre WARNING (0.30 m) | 1.491544434 m | 0.497032588 m |
| Estado observado | ALLOW | ALLOW |
| `system_ready` | true | true |

## Cálculo geométrico

Se usó la postura física observada, sin publicar candidatos:

```text
[-1.551814113493139, 2.7496557685950496, 4.1168651479621845,
 5.844626591509698, 5.973422388148109, 2.0808752988032992]
```

El calibrador evalúa las seis cápsulas reales del modelo, el radio efectivo de
la esfera y 25 muestras a lo largo de cada horizonte J1 ±3°. PARK se busca en
una rejilla determinista de 0.10 m sobre +X; Z se obtiene del extremo del brazo
superior y se cuantiza a 0.05 m. Se elige el primer punto con separación actual
de al menos 0.80 m y separación proyectada de al menos 0.75 m en ambos sentidos.
El resultado reproducible es `(1.0, 0.0, 0.65)`.

El mínimo actual y los mínimos proyectados están dentro del intervalo solicitado
de 0.75–0.85 m. La distancia es superficie–superficie entre esfera y cápsula,
no una distancia a la base ni una medición visual de RViz.

## Escenarios STOP preservados

La actualización de PARK no cambió las coordenadas derivadas de los otros dos
estados:

| Escenario | Centro (m) | Actual | Mínimo J1 +3° | Resultado |
|---|---|---:|---:|---|
| BLOCK_START | `(-0.254134455, 0.473147106, 0.787705975)` | 0.014999302 m | 0.014999302 m, t=0 s | STOP |
| INSERT_STOP insertado | `(0.191637718, 0.567693309, 0.810205975)` | 0.040555118 m | 0.014999995 m, t=1 s | STOP |

`INSERT_STOP` no insertado reutiliza la nueva PARK y permanece ALLOW.

## Cambios

- `src/thesis_core/thesis_core/virtual_obstacle_scenarios.py`: PARK se deriva
  para los dos horizontes J1 ±3° y el objetivo de 0.80 m.
- `src/thesis_core/config/virtual_obstacle_scenarios.yaml`: solo `park_pose`
  cambió a `[1.0, 0.0, 0.65]`; BLOCK_START e INSERT_STOP permanecieron iguales.
- `src/thesis_core/test/test_virtual_obstacle_scenarios.py`: valida el rango
  0.75–0.85 m, ambos sentidos y la permanencia de los escenarios STOP.

No se modificaron supervisor, STOP/WARNING/REDUCTION, márgenes, watchdog ni
lógica de seguridad.

## Compilación y pruebas

```text
colcon build --packages-select thesis_core thesis_hardware --symlink-install
  correcto

pytest test_virtual_obstacle_scenarios.py
  4 passed

ament_flake8 / ament_pep257 sobre archivos modificados
  sin problemas

thesis_hardware
  12 passed, 1 skipped
```

La suite completa de `thesis_core` conserva dos fallos de estilo preexistentes
y ajenos: W391 en `test_a02a_frame_tf_interlock.py:208` y D213 en
`safety_supervisor_node.py:114`. El resultado restante fue 169 pruebas pasadas
y 1 omitida.

## Estado final

PARK nueva quedó activa en RViz y en la fuente. El stack permanece
`armed=false`, `jog_active=false`, `mode=hold`, `fault=false`,
`system_ready=true` y salida física 0 Hz. No hubo movimiento articular.

PARK A 0.8 m VALIDADO Y LISTO PARA PRUEBA FÍSICA STOP
