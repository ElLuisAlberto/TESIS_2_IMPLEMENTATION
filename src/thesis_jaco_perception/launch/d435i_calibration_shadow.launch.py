"""Inactive-by-default temporary calibration contract; never publishes TF."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enabled = LaunchConfiguration("enabled")
    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("candidate_extrinsic_file", default_value=""),
        DeclareLaunchArgument("session_only", default_value="true"),
        Node(
            package="thesis_jaco_perception",
            executable="d435i_calibration_candidate_node",
            name="d435i_calibration_candidate",
            condition=IfCondition(enabled),
            parameters=[{
                "session_only": LaunchConfiguration("session_only"),
                "candidate_extrinsic_file": LaunchConfiguration("candidate_extrinsic_file"),
                "detector_available": False,
                "explicit_final_acceptance": False,
            }],
        ),
    ])
