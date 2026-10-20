# Corrección del retiro protectivo desde STOP

Fecha: 2026-10-19

## Causa raíz

El supervisor ya implementaba `PROTECTIVE_WITHDRAWAL`: dentro de STOP acepta
exclusivamente un JOG cuya trayectoria completa no reduzca la separación y
termine al menos 2 mm más lejos, aplicando escala 0.10. Dos compuertas externas
impedían usarlo:

1. El adaptador exigía `system_ready=true` y ausencia de STOP para armar. Como
   readiness pasa a false precisamente por STOP, el armado de recuperación era
   imposible.
2. La GUI cancelaba el JOG articular siempre que `system_ready=false`, incluso
   cuando el STOP era geométrico, fresco y apto para retiro supervisado.

## Corrección mínima

- El adaptador reconoce `protective_recovery_active` solamente cuando el STOP
  es fresco, válido, en frame `world` y su segmento limitante pertenece a una
  de las seis cápsulas conocidas. `TF_UNAVAILABLE` y segmentos desconocidos no
  habilitan recuperación.
- En ese estado permite armar en HOLD sin relajar las demás condiciones:
  backend, estado articular, proximidad y readiness deben estar frescos y el
  supervisor debe existir.
- La GUI permite únicamente JOG articular continuo durante esta recuperación.
  Los candidatos por objetivo y el control cartesiano continúan bloqueados.
- El supervisor conserva la decisión final: aproximación adicional produce
  STOP; solo un retiro monotónico genera `PROTECTIVE_WITHDRAWAL` reducido.
- El armado sigue creando una época nueva, limpia JOG previo y deja salida 0 Hz
  hasta un input deliberado posterior.

No se cambiaron umbrales, márgenes, watchdog ni la política del supervisor.

## Validación sin movimiento

Compilación limitada a `thesis_hardware_bridge` y `thesis_ui`: correcta.

```text
thesis_hardware_bridge: 23/23 pruebas pasan
thesis_ui:              26/26 pruebas activas pasan, 1 omitida
```

La sesión física se reinició desarmada. Estado observado:

```text
minimum_clearance:          0.0496642953 m
limiting_segment:           wrist_2_to_tool
system_ready:               false (STOP geométrico esperado)
proximity_valid:            true
safety_stop_active:         true
protective_recovery_active: true
armed:                      false
mode:                       hold
output_command_rate:        0 Hz
fault:                      false
```

La GUI y RViz quedaron abiertos. No se armó ni se envió movimiento durante la
corrección.

## Direcciones calculadas en la postura actual

Evaluación local de trayectorias de un solo joint, sin publicación ROS:

| Dirección | Separación inicial | Separación final a 1° | Retiro válido |
|---|---:|---:|---|
| J1 negativa | 0.049664 m | 0.057440 m | sí |
| J1 positiva | 0.049664 m | 0.041841 m | no |
| J2 positiva | 0.049664 m | 0.055191 m | sí |
| J2 negativa | 0.049664 m | 0.044440 m | no |
| J3 negativa | 0.049664 m | 0.052326 m | sí |
| J3 positiva | 0.049664 m | 0.047140 m | no |

J4–J6 no producen el progreso mínimo requerido en esta configuración y serán
rechazados para recuperación.
