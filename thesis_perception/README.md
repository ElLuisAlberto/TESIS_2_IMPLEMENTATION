# thesis_perception

Robot-independent RGB-D processing for the preventive thesis architecture.
The package can be characterized with the Intel RealSense D435i before any
simulated or physical manipulator is connected.

## Safety boundary

The default output is:

```text
/thesis/perception/obstacle_candidate
```

It is intentionally different from `/thesis/obstacle_input`. Therefore,
starting this package cannot modify a preventive decision. The candidate must
only be routed to the supervisor after validating the workspace crop, camera
extrinsic, timestamps, uncertainty and dropout response.

## Pipeline

```text
D435i PointCloud2
  -> pointcloud_preprocessor
  -> /thesis/perception/points_filtered
  -> obstacle_extractor
  -> /thesis/perception/obstacle_candidate
```

`pointcloud_preprocessor` rejects non-finite points, optionally transforms the
cloud, rejects physically implausible ranges before voxelization, applies a
configurable workspace crop and reduces the result with a voxel grid.

`obstacle_extractor` performs Euclidean clustering, selects the nearest valid
cluster, encloses it in a conservative sphere and estimates a bounded velocity.
The output uses the generic `thesis_interfaces/Obstacle` contract.

`perception_health` publishes `READY`, `DEGRADED` or `LOST` through
`/thesis/perception/diagnostics`. A fixed camera does not require IMU data by
default.

The extractor publishes `/thesis/perception/extraction_diagnostics` on every
processed cloud. Its `has_obstacle` key distinguishes a healthy empty scene
from a detected candidate; loss of clouds is reported separately by
`perception_health`.

## Standalone mode

After building the workspace:

```bash
ros2 launch thesis_perception d435i_standalone.launch.py \
  start_rviz:=true
```

While that launch is running, a terminal-only report can be generated with:

```bash
bash tools/realsense_d435i_test/scripts/validate_perception_standalone.sh
```

The report uses the lightweight `perception_health` counter for cloud rate.
Direct `ros2 topic hz` probes are disabled because copying large PointCloud2
messages can reduce the rate being measured. They can be enabled explicitly
for troubleshooting with `PERCEPTION_RUN_RATE_PROBES=true`.

This preserves the camera frame and does not publish a fabricated extrinsic.
The crop is disabled until the actual workspace bounds have been measured.
Accelerometer and gyroscope acquisition is also disabled by default because a
fixed RGB-D camera does not need inertial data for obstacle geometry. Enable it
explicitly with `enable_imu:=true` only after validating host IIO permissions.
The standalone launch also exposes `enable_crop`, `min_x`, `max_x`, `min_y`,
`max_y`, `min_z`, `max_z`, `enable_voxel` and `voxel_leaf_size` so the final
workspace can be tuned from the terminal without modifying source files.
For a fixed D435i, spatial and temporal depth filters are enabled by default to
reduce wall flicker. Depth decimation with magnitude 2 reduces the upstream
PointCloud2 workload before PCL processing. The camera PointCloud2 publisher
uses the `SENSOR_DATA` QoS profile, while downstream consumers retain only the
newest sample so stale geometry is not accumulated.

## Extrinsic calibration

The processing launch can publish a static `world -> d435i_link` transform,
but the option is disabled by default. Do not enable it with zero values. Once
the measured translation and roll-pitch-yaw are available, they can be supplied
as launch arguments:

```text
publish_static_extrinsic:=true
extrinsic_parent_frame:=world
camera_link_frame:=d435i_link
extrinsic_x:=...
extrinsic_y:=...
extrinsic_z:=...
extrinsic_roll:=...
extrinsic_pitch:=...
extrinsic_yaw:=...
target_frame:=world
```

The RealSense driver publishes the internal transforms from `d435i_link` to
its optical frames. The thesis package only owns the measured transform between
the external reference and the camera body.

## Clock policy

- Standalone or physical operation: `use_sim_time:=false` and
  `restamp_output:=false`.
- Real camera with a simulated robot: `use_sim_time:=true` and
  `restamp_output:=true` at the perception gateway.

The health node detects stale timestamps and timestamps too far in the future.

## Integration gate

Before routing the candidate to `/thesis/obstacle_input`, validate:

1. Stable cloud frequency and diagnostic state `READY`.
2. Correct `world -> d435i_link` calibration.
3. Crop bounds that exclude floor, walls and irrelevant areas.
4. Candidate position against objects placed at known coordinates.
5. Empty-scene behavior and USB disconnection behavior.
6. Recorded processing latency and calibrated uncertainty.
7. Self-filter behavior when the JACO2 is later introduced.

The current extractor intentionally models one principal obstacle. Multiple
simultaneous obstacles require a future array or multi-sphere contract.
