from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulation_output_enabled = LaunchConfiguration(
        'simulation_output_enabled'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation_output_enabled',
            default_value='false',
            description=(
                'Habilita salida únicamente al controlador de Gazebo'
            ),
        ),

        Node(
            package='thesis_core',
            executable='safety_supervisor',
            output='screen',
        ),

        Node(
            package='thesis_simulation',
            executable='simulation_command_adapter',
            output='screen',
            parameters=[{
                'simulation_output_enabled':
                    simulation_output_enabled,
            }],
        ),
    ])
