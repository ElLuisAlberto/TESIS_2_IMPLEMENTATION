# Prueba reversible de voxelización D435i a 1 cm

Fecha: 2026-10-21. Alcance: D435i, percepción aislada y RViz, manteniendo
640x480@30 FPS. No se compiló, no se grabaron rosbags y no se modificó ningún
parámetro distinto de la hoja de voxel activa.

## Parámetro y cambio aplicado

El parámetro efectivo es `voxel_leaf_size` declarado por
`thesis_perception/launch/d435i_standalone.launch.py`; se propaga al launch de
procesamiento y sobreescribe el mismo valor de respaldo del YAML. Se confirmó
su valor activo inicial de 0.02 m. El único cambio fue:

```diff
- DeclareLaunchArgument("voxel_leaf_size", default_value="0.02"),
+ DeclareLaunchArgument("voxel_leaf_size", default_value="0.01"),
```

No se cambiaron FPS, perfiles RGB/depth, resolución, rango de 1.5 m,
decimación, filtros espacial/temporal, recorte ni otro archivo.

## Procesos

Se identificó el grupo PGID 39438 como la única sesión aislada activa
`d435i_standalone.launch.py`, con los hijos D435i, preprocesador, extractor,
salud y RViz. Recibió SIGINT; la comprobación posterior confirmó que no quedó
ninguno de esos procesos. Se inició una sola sesión de prueba, launch PID
40177, con D435i y RViz activos a 640x480@30.

## Comparación cuantitativa

Las ventanas son pasivas, duran 30 s y usan `d435i_observation_probe` sobre
`/thesis/perception/points_filtered`. Los puntos por mensaje se calculan como
puntos finitos recibidos dividido entre mensajes.

| Métrica | Voxel 2 cm (línea base) | Voxel 1 cm | Cambio |
|---|---:|---:|---:|
| Frecuencia filtrada | 21.73 Hz | 26.03 Hz | +4.30 Hz |
| Mensajes en 30 s | 652 | 781 | +19.8 % |
| Puntos finitos | 261,298 | 999,287 | +282.4 % |
| Puntos/mensaje | ~401 | ~1,279 | +219 % |
| Nubes vacías | 0 | 0 | Sin anomalía |
| Sellos nulos, futuros u obsoletos | 0 | 0 | Sin anomalía |
| Errores nuevos del driver | 0 observados | 0 observados | Sin error observado |
| Diagnóstico de salud | muestra previa variable | `OK`; 29 Hz, edad 0.01 s, sello 0.05 s | Salud OK |

## Decisión

**Conservar voxel de 0.01 m.** La nube filtrada supera los criterios de la
prueba: ~1,279 puntos por mensaje (mínimo 800), 26.03 Hz (mínimo 15), cero
nubes vacías y cero sellos obsoletos según el probe, sin errores nuevos del
driver. La sesión a 30 FPS con voxel de 1 cm queda activa para RViz.

## Seguridad

El JACO2 permaneció inmóvil y desarmado. No se iniciaron `thesis_hardware`,
`hardware_output`, Gazebo, supervisor, auto-filtro ni TF estático. No se
enviaron comandos al brazo ni se publicó nada en `/thesis/obstacle_input`; no
hubo conexión ni integración con el supervisor.
