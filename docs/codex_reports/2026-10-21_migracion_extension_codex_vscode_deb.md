# Verificación de extensión Codex/OpenAI en VS Code `.deb`

**Fecha:** 2026-10-21
**Resultado:** no fue necesaria una instalación adicional.

## Detección

Se compararon las extensiones de VS Code Snap (`/snap/bin/code`) y VS Code `.deb` (`/usr/bin/code`). Ambas listas contienen las mismas extensiones de OpenAI:

- `openai.chatgpt@26.1007.21434`
- `openai.codex-audio@26.1007.21434`

Por tanto, no existe una extensión identificable de Codex/OpenAI presente en Snap y ausente en la instalación `.deb`. No se ejecutó ninguna instalación de extensión.

## Verificación y apertura

La comprobación de `/usr/bin/code --list-extensions --show-versions` confirmó ambas extensiones. Se solicitó la apertura del repositorio mediante:

```bash
/usr/bin/code /home/luis/Escritorio/TESIS_2_IMPLEMENTATION
```

El operador puede recargar la ventana `.deb` si VS Code lo solicita. Es posible que deba iniciar sesión manualmente en Codex/OpenAI dentro de esa ventana; no se copiaron ni manipularon tokens, credenciales o sesiones.

## Alcance preservado

- No se modificó ningún archivo fuente ni de configuración del repositorio; este informe es el único artefacto creado bajo `docs/codex_reports/`.
- No se eliminó ni modificó la instalación VS Code Snap.
- No se instaló ninguna extensión adicional.
