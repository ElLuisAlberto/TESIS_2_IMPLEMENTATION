"""Launch the robot-independent RGB-D perception processing pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _boolean(name):
    """Return a launch value explicitly converted to bool."""
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _floating(name):
    """Return a launch value explicitly converted to float."""
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    """Build preprocessing, candidate extraction and health monitoring."""
    config = PathJoinSubstitution([
        FindPackageShare("thesis_perception"),
        "config",
        "perception.yaml",
    ])

    common_time = {
        "use_sim_time": _boolean("use_sim_time"),
    }

    preprocessor = Node(
        package="thesis_perception",
        executable="pointcloud_preprocessor_node",
        name="pointcloud_preprocessor",
        output="screen",
        parameters=[
            config,
            common_time,
            {
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("filtered_topic"),
                "target_frame": LaunchConfiguration("target_frame"),
                "use_latest_transform": _boolean(
                    "use_latest_transform"
                ),
                "restamp_output": _boolean("restamp_output"),
                "enable_crop": _boolean("enable_crop"),
                "enable_voxel": _boolean("enable_voxel"),
                "voxel_leaf_size": _floating("voxel_leaf_size"),
                "min_x": _floating("min_x"),
                "max_x": _floating("max_x"),
                "min_y": _floating("min_y"),
                "max_y": _floating("max_y"),
                "min_z": _floating("min_z"),
                "max_z": _floating("max_z"),
            },
        ],
    )

    extractor = Node(
        package="thesis_perception",
        executable="obstacle_extractor_node",
        name="obstacle_extractor",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_extractor")),
        parameters=[
            config,
            common_time,
            {
                "input_topic": LaunchConfiguration("filtered_topic"),
                "obstacle_topic": LaunchConfiguration("obstacle_topic"),
            },
        ],
    )

    health = Node(
        package="thesis_perception",
        executable="perception_health_node",
        name="perception_health",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_health")),
        parameters=[
            config,
            common_time,
            {
                "cloud_topic": LaunchConfiguration("filtered_topic"),
                "expected_frame": LaunchConfiguration("target_frame"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "imu_required": _boolean("imu_required"),
                "hardware_id": LaunchConfiguration("hardware_id"),
            },
        ],
    )

    static_extrinsic = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="d435i_static_extrinsic",
        output="screen",
        condition=IfCondition(
            LaunchConfiguration("publish_static_extrinsic")
        ),
        arguments=[
            "--x", LaunchConfiguration("extrinsic_x"),
            "--y", LaunchConfiguration("extrinsic_y"),
            "--z", LaunchConfiguration("extrinsic_z"),
            "--roll", LaunchConfiguration("extrinsic_roll"),
            "--pitch", LaunchConfiguration("extrinsic_pitch"),
            "--yaw", LaunchConfiguration("extrinsic_yaw"),
            "--frame-id", LaunchConfiguration("extrinsic_parent_frame"),
            "--child-frame-id", LaunchConfiguration("camera_link_frame"),
        ],
    )

    declarations = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "input_topic",
            default_value="/camera/d435i/depth/color/points",
        ),
        DeclareLaunchArgument(
            "filtered_topic",
            default_value="/thesis/perception/points_filtered",
        ),
        DeclareLaunchArgument(
            "obstacle_topic",
            default_value="/thesis/perception/obstacle_candidate",
            description=(
                "Safe candidate output. Remap to /thesis/obstacle_input "
                "only after calibration and validation."
            ),
        ),
        DeclareLaunchArgument(
            "target_frame",
            default_value="",
            description=(
                "Empty preserves the camera frame. Use world only with a "
                "measured extrinsic."
            ),
        ),
        DeclareLaunchArgument("use_latest_transform", default_value="true"),
        DeclareLaunchArgument(
            "restamp_output",
            default_value="false",
            description=(
                "Use true only at the real-camera/sim-clock gateway."
            ),
        ),
        DeclareLaunchArgument("enable_crop", default_value="false"),
        DeclareLaunchArgument("enable_voxel", default_value="true"),
        DeclareLaunchArgument("voxel_leaf_size", default_value="0.02"),
        DeclareLaunchArgument("min_x", default_value="-1.5"),
        DeclareLaunchArgument("max_x", default_value="1.5"),
        DeclareLaunchArgument("min_y", default_value="-1.5"),
        DeclareLaunchArgument("max_y", default_value="1.5"),
        DeclareLaunchArgument("min_z", default_value="-1.0"),
        DeclareLaunchArgument("max_z", default_value="2.5"),
        DeclareLaunchArgument("start_extractor", default_value="true"),
        DeclareLaunchArgument("start_health", default_value="true"),
        DeclareLaunchArgument(
            "imu_topic", default_value="/camera/d435i/imu"
        ),
        DeclareLaunchArgument(
            "imu_required",
            default_value="false",
            description="A fixed RGB-D camera does not require IMU data.",
        ),
        DeclareLaunchArgument(
            "hardware_id", default_value="Intel RealSense D435i"
        ),
        DeclareLaunchArgument(
            "publish_static_extrinsic",
            default_value="false",
            description=(
                "Never enable until the camera pose has been measured."
            ),
        ),
        DeclareLaunchArgument(
            "extrinsic_parent_frame", default_value="world"
        ),
        DeclareLaunchArgument("camera_link_frame", default_value="d435i_link"),
        DeclareLaunchArgument("extrinsic_x", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_y", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_z", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_roll", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_pitch", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_yaw", default_value="0.0"),
    ]

    return LaunchDescription(
        declarations + [
            static_extrinsic,
            preprocessor,
            extractor,
            health,
        ]
    )
