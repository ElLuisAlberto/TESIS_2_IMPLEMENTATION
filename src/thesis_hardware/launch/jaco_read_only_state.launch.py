"""Start the physical JACO2 state path without any command or safety stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes")


def _reject_physical_output_value(value):
    if _as_bool(value):
        raise RuntimeError(
            "hardware_output=true is forbidden by jaco_read_only_state.launch.py"
        )


def _reject_physical_output(context):
    _reject_physical_output_value(
        LaunchConfiguration("hardware_output").perform(context)
    )
    return []


def generate_launch_description():
    """Compose USB feedback, /joint_states, model TF and read-only diagnostics."""
    bridge_share = get_package_share_directory("thesis_hardware_bridge")
    description_share = get_package_share_directory("thesis_description")
    hardware_config = os.path.join(bridge_share, "config", "jaco_hardware.yaml")
    xacro_file = os.path.join(
        description_share, "urdf", "j2n6s300_standalone.xacro"
    )
    kinova_library_dir = LaunchConfiguration("kinova_library_dir")
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file]), value_type=str
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "hardware_output", default_value="false",
            description="Must remain false; physical output is unavailable in this launch.",
        ),
        DeclareLaunchArgument(
            "expected_serial_number", default_value="",
            description="Exact serial required when more than one arm exists.",
        ),
        DeclareLaunchArgument(
            "kinova_library_dir",
            default_value=os.path.join(
                os.environ.get(
                    "KINOVA_ROOT",
                    os.path.expanduser(
                        "~/Escritorio/TESIS_2_DEPENDENCIES/kinova-ros/kinova_driver"
                    ),
                ),
                "lib", "x86_64-linux-gnu",
            ),
        ),
        OpaqueFunction(function=_reject_physical_output),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {
                "use_sim_time": False,
                "publish_frequency": 40.0,
            }],
        ),
        Node(
            package="thesis_hardware_bridge",
            executable="jaco_hardware_node",
            name="jaco_hardware_adapter",
            output="screen",
            additional_env={
                "LD_LIBRARY_PATH": [
                    kinova_library_dir, ":",
                    EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
                ],
            },
            parameters=[hardware_config, {
                "mock_hardware": False,
                "hardware_output_enabled": False,
                "expected_serial_number": LaunchConfiguration("expected_serial_number"),
            }],
        ),
        Node(
            package="thesis_hardware",
            executable="hardware_readiness",
            name="hardware_readiness",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "require_armed": False,
                "require_proximity_status": False,
                "timeout_sec": 0.50,
                "rate_hz": 2.0,
            }],
        ),
    ])
