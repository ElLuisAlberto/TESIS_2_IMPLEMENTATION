#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE="/home/luis/Escritorio/TESIS_2_IMPLEMENTATION"
REPORT_DIR="$WORKSPACE/tools/realsense_d435i_test/reports"
STAMP="$(date +%Y%m%d_%H%M%S)"

CAMERA_LOG="$REPORT_DIR/camera_${STAMP}.log"
MADGWICK_LOG="$REPORT_DIR/madgwick_${STAMP}.log"
RVIZ_LOG="$REPORT_DIR/rviz_${STAMP}.log"

source /opt/ros/humble/setup.bash
mkdir -p "$REPORT_DIR"

cleanup()
{
    trap - EXIT INT TERM
    echo
    echo "Cerrando cámara, Madgwick y RViz..."

    if [[ -n "${MADGWICK_PGID:-}" ]]; then
        kill -INT -- "-${MADGWICK_PGID}" 2>/dev/null || true
    fi

    if [[ -n "${CAMERA_PGID:-}" ]]; then
        kill -INT -- "-${CAMERA_PGID}" 2>/dev/null || true
    fi

    wait 2>/dev/null || true
}

trap cleanup EXIT INT TERM

echo "=========================================="
echo "D435i: RGB-D + nube + IMU + Madgwick + RViz"
echo "=========================================="

echo
echo "[1/3] Iniciando RealSense D435i..."

setsid ros2 launch realsense2_camera rs_launch.py \
    camera_name:=d435i \
    enable_color:=true \
    enable_depth:=true \
    enable_gyro:=true \
    enable_accel:=true \
    unite_imu_method:=2 \
    gyro_fps:=200 \
    accel_fps:=100 \
    enable_sync:=true \
    pointcloud.enable:=true \
    pointcloud.stream_filter:=2 \
    align_depth.enable:=true \
    depth_module.depth_profile:=640,480,15 \
    rgb_camera.color_profile:=640,480,15 \
    > >(tee "$CAMERA_LOG") 2>&1 &

CAMERA_PGID=$!

CAMERA_READY=false

for _ in $(seq 1 30); do
    if ! kill -0 "$CAMERA_PGID" 2>/dev/null; then
        echo "ERROR: el proceso de la cámara terminó."
        exit 1
    fi

    TOPICS="$(ros2 topic list 2>/dev/null || true)"

    if grep -qx "/camera/d435i/imu" <<< "$TOPICS" &&
       grep -qx "/camera/d435i/depth/color/points" <<< "$TOPICS"; then
        CAMERA_READY=true
        break
    fi

    sleep 1
done

if [[ "$CAMERA_READY" != true ]]; then
    echo "ERROR: la cámara no publicó IMU y nube de puntos."
    exit 1
fi

echo
echo "[2/3] Iniciando filtro Madgwick..."

setsid ros2 run imu_filter_madgwick imu_filter_madgwick_node \
    --ros-args \
    -r imu/data_raw:=/camera/d435i/imu \
    -r imu/data:=/camera/d435i/imu_filtered \
    -p use_mag:=false \
    -p publish_tf:=true \
    -p reverse_tf:=true \
    -p fixed_frame:=world \
    -p world_frame:=enu \
    > >(tee "$MADGWICK_LOG") 2>&1 &

MADGWICK_PGID=$!

if timeout 10 ros2 topic echo \
    --once /camera/d435i/imu_filtered >/dev/null 2>&1
then
    echo "IMU filtrada recibida correctamente."
else
    echo "ADVERTENCIA: no se recibió /camera/d435i/imu_filtered."
fi

echo
echo "[3/3] Abriendo RViz..."
echo "Al cerrar RViz también se cerrarán la cámara y Madgwick."
echo

rviz2 2>&1 | tee "$RVIZ_LOG"
