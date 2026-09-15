# Corrección de prevención continua: histéresis

Esta etapa corrige la alternancia rápida entre `WARNING` y `REDUCTION`
observada durante una ejecución.

Cambios principales:

- Después de entrar en `REDUCTION`, el supervisor conserva la reducción
  mientras la distancia proyectada sea menor que `0.22 m`.
- La velocidad nominal solo se recupera después de cinco muestras
  consecutivas con margen suficiente.
- El adaptador aplica un intervalo mínimo de `0.75 s` antes de restaurar
  velocidad nominal y vuelve a dar prioridad inmediata a `STOP`.

Esto evita cancelaciones y replanteamientos repetitivos de la misma
trayectoria. El comando continúa siendo evaluado a 10 Hz y `STOP` no queda
retenido por el intervalo de recuperación.
