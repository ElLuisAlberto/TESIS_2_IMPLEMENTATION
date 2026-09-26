# Evidencia del mínimo dentro del horizonte

La distancia `d_min` del monitor de ejecución se calcula recorriendo todas
las muestras del horizonte corto, incluida la configuración actual (`t=0`).
Esta revisión publica también dónde se encontró ese mínimo:

- `minimum_time_from_now`: tiempo relativo desde la configuración medida;
- `minimum_sample_index` y `minimum_sample_count`: muestra que limita y
  cantidad total de muestras;
- `minimum_horizon_fraction`: posición normalizada dentro del horizonte.

La explicación textual de `/thesis/execution_control` y la etiqueta de RViz
ahora muestran, por ejemplo, `t_min=0.80s; muestra=17/21`. Si aparece
`t_min=0.00s; muestra=1/21`, el estado actual es el peor. Si el tiempo es
mayor que cero, la distancia mínima pertenece a una configuración futura de
la proyección.

Para verificarlo durante una ejecución:

```bash
ros2 topic echo /thesis/execution_control
```

El campo `time_to_collision` conserva su significado anterior: es el primer
instante con solapamiento, no necesariamente el instante de la distancia
mínima positiva.
