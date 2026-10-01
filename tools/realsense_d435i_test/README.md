# Prueba funcional Intel RealSense D435i

Este directorio conserva la prueba de adquisición RGB-D e inercial
previa a la integración con el supervisor preventivo.

## Ejecución

```bash
cd ~/Escritorio/TESIS_2_IMPLEMENTATION
xhost +SI:localuser:root
sudo -E ./tools/realsense_d435i_test/scripts/run_pointcloud_imu_rviz.sh
```

El procesamiento de percepción se mantiene en el paquete
`thesis_perception`.
