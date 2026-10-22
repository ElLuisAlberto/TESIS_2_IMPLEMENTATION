# Recuperación de Webview de Codex en VS Code `.deb`

Fecha: 2026-10-21

## Acciones realizadas

- Se verificó la instalación ejecutable de VS Code `.deb` en `/usr/bin/code`.
- Se verificó la extensión `openai.chatgpt` en `~/.vscode/extensions/openai.chatgpt-26.51007.21434-linux-x64`.
- Se creó el perfil aislado y reversible `~/.config/Code-Codex-Native`, que no existía previamente.
- Se abrió el repositorio con el perfil aislado y conservando el directorio de extensiones existente:

```bash
/usr/bin/code \
  --user-data-dir "$HOME/.config/Code-Codex-Native" \
  --extensions-dir "$HOME/.vscode/extensions" \
  "$HOME/Escritorio/TESIS_2_IMPLEMENTATION"
```

## Límites aplicados

No se modificó `~/.config/Code`, no se eliminaron cachés, extensiones, tokens ni sesiones, y no se inició ningún componente ROS, cámara, RViz o JACO2. No se realizaron reparaciones adicionales.

## Verificación pendiente del operador

Confirmar en la ventana abierta que el panel Codex ya no presenta el error de Webview/ServiceWorker. Si VS Code lo solicita, iniciar sesión manualmente. Conservar abiertas e intactas las instalaciones y perfiles anteriores.
