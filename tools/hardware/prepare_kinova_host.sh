#!/usr/bin/env bash
set -Eeo pipefail
# ROS 2 Humble setup files are not guaranteed to be nounset-safe.  Disable a
# nounset option inherited through SHELLOPTS before sourcing any environment.
set +u

REPO="${REPO:-$HOME/Escritorio/TESIS_2_IMPLEMENTATION}"
KINOVA_ROOT="${KINOVA_ROOT:-$HOME/Escritorio/TESIS_2_DEPENDENCIES/kinova-ros/kinova_driver}"
REPORT="${REPORT:-$HOME/Descargas/PUENTE_KINOVA_PREPARACION_SIN_BRAZO_V2.txt}"
RULE_SOURCE="$REPO/src/thesis_hardware/udev/70-thesis-kinova.rules"
RULE_TARGET="/etc/udev/rules.d/70-thesis-kinova.rules"
KINOVA_LIBRARY="$KINOVA_ROOT/lib/x86_64-linux-gnu/USBCommandLayerUbuntu.so"
BRIDGE_EXECUTABLE="$REPO/install/thesis_hardware_bridge/lib/thesis_hardware_bridge/jaco_hardware_node"

mkdir -p "$(dirname "$REPORT")"
: >"$REPORT"
exec > >(tee -a "$REPORT") 2>&1

on_error() {
  code=$?
  echo
  echo "RESULTADO=FALLO"
  echo "CODIGO=$code"
  echo "No conecte todavía el brazo."
  echo "INFORME=$REPORT"
  exit "$code"
}
trap on_error ERR

echo "=== PREPARACIÓN DEL HOST KINOVA SIN BRAZO V2 ==="
echo "INICIO=$(date --iso-8601=seconds)"
echo "REPO=$REPO"
echo "KINOVA_ROOT=$KINOVA_ROOT"
echo "BRAZO_REQUERIDO=NO"

test -d "$REPO/.git"
test -f "$RULE_SOURCE"
test -f "$KINOVA_ROOT/include/kinova/Kinova.API.USBCommandLayerUbuntu.h"
test -f "$KINOVA_LIBRARY"
test -x "$BRIDGE_EXECUTABLE"

source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"

echo
echo "=== HERRAMIENTAS DEL SISTEMA ==="
missing_packages=()
command -v lsusb >/dev/null || missing_packages+=(usbutils)
command -v udevadm >/dev/null || missing_packages+=(udev)
command -v nm >/dev/null || missing_packages+=(binutils)
command -v ldd >/dev/null || missing_packages+=(libc-bin)
if (( ${#missing_packages[@]} > 0 )); then
  echo "INSTALANDO=${missing_packages[*]}"
  sudo apt-get update
  sudo apt-get install -y "${missing_packages[@]}"
else
  echo "PAQUETES_BASE=YA_INSTALADOS"
fi

echo
echo "=== GRUPO DE ACCESO USB ==="
if ! getent group plugdev >/dev/null; then
  sudo groupadd --system plugdev
fi
relogin_required=false
if id -nG "$USER" | tr ' ' '\n' | grep -qx plugdev; then
  echo "USUARIO_EN_PLUGDEV=SI"
else
  sudo usermod -aG plugdev "$USER"
  relogin_required=true
  echo "USUARIO_EN_PLUGDEV=AÑADIDO"
fi

echo
echo "=== INSTALANDO REGLA UDEV ==="
sudo install -o root -g root -m 0644 "$RULE_SOURCE" "$RULE_TARGET"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
cmp "$RULE_SOURCE" "$RULE_TARGET"
stat -c 'UDEV_RULE=%n owner=%U group=%G mode=%a' "$RULE_TARGET"

echo
echo "=== BIBLIOTECA KINOVA ==="
file "$KINOVA_LIBRARY"
library_ldd="$(ldd "$KINOVA_LIBRARY")"
echo "$library_ldd"
if grep -q 'not found' <<<"$library_ldd"; then
  echo "BIBLIOTECA_KINOVA_DEPENDENCIAS=FALTANTES"
  false
fi
echo "BIBLIOTECA_KINOVA_DEPENDENCIAS=OK"

required_symbols=(InitAPI CloseAPI GetDevices SetActiveDevice GetAngularPosition StartControlAPI StopControlAPI SendBasicTrajectory EraseAllTrajectories SetAngularControl)
exported_symbols="$(nm -D --defined-only "$KINOVA_LIBRARY" | awk '{print $3}')"
for symbol in "${required_symbols[@]}"; do
  grep -qx "$symbol" <<<"$exported_symbols"
done
echo "SIMBOLOS_KINOVA_REQUERIDOS=OK"

echo
echo "=== EJECUTABLE ROS 2 CON OVERLAY CARGADO ==="
bridge_ldd="$(ldd "$BRIDGE_EXECUTABLE")"
echo "$bridge_ldd"
if grep -q 'not found' <<<"$bridge_ldd"; then
  echo "EJECUTABLE_DEPENDENCIAS=FALTANTES"
  false
fi
echo "EJECUTABLE_DEPENDENCIAS=OK"
readelf -d "$BRIDGE_EXECUTABLE" | grep -E 'RPATH|RUNPATH'

echo
echo "=== PAQUETES ROS 2 ==="
ros2 pkg prefix thesis_hardware_bridge
ros2 pkg prefix thesis_hardware
ros2 pkg prefix thesis_core
ros2 pkg prefix thesis_description
ros2 pkg prefix thesis_simulation
echo "PAQUETES_ROS2=OK"

echo
echo "=== CONFIGURACIÓN SEGURA PREDETERMINADA ==="
grep -Eq '^[[:space:]]*mock_hardware:[[:space:]]*false' "$REPO/src/thesis_hardware_bridge/config/jaco_hardware.yaml"
grep -Eq '^[[:space:]]*hardware_output_enabled:[[:space:]]*false' "$REPO/src/thesis_hardware_bridge/config/jaco_hardware.yaml"
echo "MOCK_HARDWARE_DEFAULT=false"
echo "HARDWARE_OUTPUT_DEFAULT=false"

echo
if lsusb -d 22cd:0000 >/dev/null 2>&1; then
  echo "KINOVA_USB_CONNECTED=SI"
else
  echo "KINOVA_USB_CONNECTED=NO_ESPERADO"
fi
echo "RELOGIN_REQUIRED=$relogin_required"
echo "RESULTADO=OK"
echo "HOST_KINOVA=PREPARADO"
echo "SIGUIENTE_FASE=CONECTAR_BRAZO_EN_SOLO_LECTURA"
echo "INFORME=$REPORT"
