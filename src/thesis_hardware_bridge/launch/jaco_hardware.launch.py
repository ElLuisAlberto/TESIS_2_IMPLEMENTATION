"""Start the native ROS 2 JACO adapter and its robot description."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the read-only-by-default hardware launch."""
    bridge_share = get_package_share_directory(
        'thesis_hardware_bridge'
    )
    description_share = get_package_share_directory(
        'thesis_description'
    )
    default_config_file = os.path.join(
        bridge_share, 'config', 'jaco_hardware.yaml'
    )
    xacro_file = os.path.join(
        description_share, 'urdf', 'j2n6s300_standalone.xacro'
    )
    rviz_file = os.path.join(
        description_share, 'rviz', 'jaco.rviz'
    )

    mock_hardware = LaunchConfiguration('mock_hardware')
    output_enabled = LaunchConfiguration('hardware_output_enabled')
    expected_serial = LaunchConfiguration('expected_serial_number')
    config_file = LaunchConfiguration('hardware_config_file')
    start_rviz = LaunchConfiguration('start_rviz')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str,
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Use the deterministic offline backend.',
        ),
        DeclareLaunchArgument(
            'hardware_output_enabled',
            default_value='false',
            description=(
                'Permit runtime arming. False guarantees read-only operation.'
            ),
        ),
        DeclareLaunchArgument(
            'expected_serial_number',
            default_value='',
            description=(
                'Exact JACO serial. Empty is accepted only with one device.'
            ),
        ),
        DeclareLaunchArgument(
            'hardware_config_file',
            default_value=default_config_file,
            description='JACO adapter ROS 2 parameter file.',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 with the JACO model.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': False}],
        ),
        Node(
            package='thesis_hardware_bridge',
            executable='jaco_hardware_node',
            name='jaco_hardware_adapter',
            output='screen',
            parameters=[
                config_file,
                {
                    'mock_hardware': ParameterValue(
                        mock_hardware, value_type=bool
                    ),
                    'hardware_output_enabled': ParameterValue(
                        output_enabled, value_type=bool
                    ),
                    'expected_serial_number': expected_serial,
                },
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(start_rviz),
            parameters=[{'use_sim_time': False}],
            arguments=['-d', rviz_file],
        ),
    ])
