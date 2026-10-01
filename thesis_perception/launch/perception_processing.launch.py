from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("thesis_perception"),
        "config",
        "perception.yaml",
    ])

    use_sim_time = LaunchConfiguration("use_sim_time")
    target_frame = LaunchConfiguration("target_frame")
    start_health = LaunchConfiguration("start_health")

    preprocessor = Node(
        package="thesis_perception",
        executable="pointcloud_preprocessor_node",
        name="pointcloud_preprocessor",
        output="screen",
        parameters=[
            config,
            {
                "use_sim_time": ParameterValue(
                    use_sim_time, value_type=bool
                ),
                "target_frame": target_frame,
            },
        ],
    )

    health = Node(
        package="thesis_perception",
        executable="perception_health_node",
        name="perception_health",
        output="screen",
        condition=IfCondition(start_health),
        parameters=[
            config,
            {
                "use_sim_time": ParameterValue(
                    use_sim_time, value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "target_frame",
            default_value="base_link",
        ),
        DeclareLaunchArgument(
            "start_health",
            default_value="true",
        ),
        preprocessor,
        health,
    ])
