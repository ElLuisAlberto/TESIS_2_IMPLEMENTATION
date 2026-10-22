"""Isolated D435i camera and world-cloud pipeline for a JACO shadow session.

This launcher deliberately scopes the RealSense launch context.  The upstream
RealSense launcher reports every active launch configuration as a potential
camera parameter, so session-only values must never share its context.
"""

from launch import LaunchDescription
import math

from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


CROP_LIMIT_NAMES = ("min_x", "max_x", "min_y", "max_y", "min_z", "max_z")


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes")


def _validate_crop_values(enable_crop, values):
    """Reject invalid world ROI values before camera or processing actions start."""
    bounds = {}
    for name in CROP_LIMIT_NAMES:
        try:
            value = float(values[name])
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError(f"{name} must be numeric") from error
        if not math.isfinite(value):
            raise RuntimeError(f"{name} must be finite")
        bounds[name] = value
    if _as_bool(enable_crop) and not (
            bounds["min_x"] < bounds["max_x"] and
            bounds["min_y"] < bounds["max_y"] and
            bounds["min_z"] < bounds["max_z"]):
        raise RuntimeError("enabled crop requires min_x < max_x, min_y < max_y, min_z < max_z")
    return bounds


def _validate_crop_bounds(context):
    _validate_crop_values(
        LaunchConfiguration("enable_crop").perform(context),
        {name: LaunchConfiguration(name).perform(context) for name in CROP_LIMIT_NAMES})
    return []


def generate_launch_description():
    """Create the sealed camera scope and exact-stamp world cloud pipeline."""
    realsense_launch = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"
    ])
    processing_launch = PathJoinSubstitution([
        FindPackageShare("thesis_perception"),
        "launch",
        "perception_processing.launch.py",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("thesis_perception"),
        "rviz",
        "d435i_perception.rviz",
    ])

    camera = GroupAction(
        scoped=True,
        forwarding=False,
        launch_configurations={
            "camera_namespace": "camera",
            "camera_name": "d435i",
            # rs_launch composes camera_name + base_frame_id as d435i_link,
            # the only frame to which an external world extrinsic may attach.
            "base_frame_id": "link",
            "tf_prefix": "",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_gyro": "false",
            "enable_accel": "false",
            "unite_imu_method": "2",
            "gyro_fps": "200",
            "accel_fps": "100",
            "enable_sync": "true",
            "pointcloud.enable": "true",
            "pointcloud.stream_filter": "2",
            "align_depth.enable": "true",
            "decimation_filter.enable": "true",
            "decimation_filter.filter_magnitude": "2",
            "spatial_filter.enable": "true",
            "temporal_filter.enable": "true",
            "depth_module.depth_profile": "640,480,30",
            "rgb_camera.color_profile": "640,480,30",
        },
        actions=[
            SetParameter(name="pointcloud.pointcloud_qos", value="SENSOR_DATA"),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(realsense_launch)),
        ],
    )

    processing = GroupAction(
        scoped=True,
        forwarding=False,
        launch_configurations={
            "use_sim_time": "false",
            "input_topic": "/camera/d435i/depth/color/points",
            "filtered_topic": "/thesis/perception/points_filtered",
            "target_frame": "world",
            "use_latest_transform": "false",
            "restamp_output": "false",
            "enable_crop": LaunchConfiguration("enable_crop"),
            "enable_voxel": "true",
            "voxel_leaf_size": "0.01",
            "min_valid_range_m": "0.10",
            "max_valid_range_m": "1.5",
            "min_x": LaunchConfiguration("min_x"),
            "max_x": LaunchConfiguration("max_x"),
            "min_y": LaunchConfiguration("min_y"),
            "max_y": LaunchConfiguration("max_y"),
            "min_z": LaunchConfiguration("min_z"),
            "max_z": LaunchConfiguration("max_z"),
            "start_extractor": "false",
            "start_health": "true",
            "imu_topic": "/camera/d435i/imu",
            "imu_required": "false",
            "hardware_id": "Intel RealSense D435i",
            "publish_static_extrinsic": "false",
            "extrinsic_parent_frame": "world",
            "camera_link_frame": "d435i_link",
            "extrinsic_x": "0.0",
            "extrinsic_y": "0.0",
            "extrinsic_z": "0.0",
            "extrinsic_roll": "0.0",
            "extrinsic_pitch": "0.0",
            "extrinsic_yaw": "0.0",
        },
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(processing_launch)
            ),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="d435i_perception_rviz",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument("enable_crop", default_value="false"),
        DeclareLaunchArgument("min_x", default_value="-1.5"),
        DeclareLaunchArgument("max_x", default_value="1.5"),
        DeclareLaunchArgument("min_y", default_value="-1.5"),
        DeclareLaunchArgument("max_y", default_value="1.5"),
        DeclareLaunchArgument("min_z", default_value="-1.0"),
        DeclareLaunchArgument("max_z", default_value="2.5"),
        OpaqueFunction(function=_validate_crop_bounds),
        camera,
        processing,
        rviz,
    ])
