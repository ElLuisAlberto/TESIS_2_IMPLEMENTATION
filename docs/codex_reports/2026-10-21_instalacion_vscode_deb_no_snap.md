# Instalación de Visual Studio Code oficial `.deb` junto a Snap

**Fecha:** 2026-10-21
**Estado:** completada; ambas instalaciones permanecen disponibles.

## Estado previo

- Arquitectura: `amd64`.
- VS Code Snap existente: paquete `code`, revisión `266`, canal `latest/stable`.
- Paquete Debian `code`: no estaba instalado.
- Repositorio confirmado: `/home/luis/Escritorio/TESIS_2_IMPLEMENTATION`.

## Instalador e instalación

- Origen: `https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64`.
- Instalador: `/home/luis/Descargas/vscode_instalador/code_stable_amd64.deb`.
- Tamaño: `243195326` bytes.
- SHA-256: `f045a886dbfd1aa47ec49dd6880d79a95570c9afde627510897b5cf692db5f2a`.
- Instalación realizada por el operador mediante `sudo apt install` sobre ese archivo exclusivamente.

## Estado posterior

- Paquete `.deb`: `code 1.143.0-1792533444`, estado `install ok installed`.
- Binario verificado: `/usr/bin/code --version` informa `1.143.0`, commit `2d0b7506a36c1d6db9a466440131a8e01eab2289`, arquitectura `x64`.
- Resolución de ejecutables: `/usr/bin/code`, `/bin/code`, y `/snap/bin/code` están disponibles.
- El Snap `code` continúa instalado en revisión `266`; no fue eliminado, desinstalado ni modificado.

## Apertura del repositorio

Se ejecutó explícitamente:

```bash
/usr/bin/code /home/luis/Escritorio/TESIS_2_IMPLEMENTATION
```

Se comprobó el proceso principal `/usr/share/code/code /home/luis/Escritorio/TESIS_2_IMPLEMENTATION`, lo que confirma que la instancia `.deb` abrió el mismo repositorio. La confirmación visual pendiente del operador es:

1. Explorador de archivos del repositorio visible.
2. Terminal integrada disponible.
3. Ventana correspondiente a la instalación `.deb`, no a `/snap/code/...`.

No se instalaron extensiones ni se migraron extensiones, preferencias o configuraciones.

## Reversión

- Para volver atrás sin cambios, siga usando VS Code Snap mediante `/snap/bin/code`.
- Si el operador solicita expresamente retirar la variante `.deb`, puede ejecutar `sudo apt remove code`. Esta acción no debe aplicarse sin esa solicitud y no afecta al paquete Snap.
