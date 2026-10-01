#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y \
  ros-humble-imu-filter-madgwick \
  ros-humble-pcl-conversions \
  ros-humble-pcl-ros \
  ros-humble-tf2-sensor-msgs

echo "Dependencias de thesis_perception instaladas."
