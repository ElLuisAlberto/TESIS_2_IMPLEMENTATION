from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    simulation_output_enabled = LaunchConfiguration(
        'simulation_output_enabled'
    )
    require_proximity_status = LaunchConfiguration(
        'require_proximity_status'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation_output_enabled',
            default_value='false',
            description=(
                'Habilita salida únicamente al controlador de Gazebo'
            ),
        ),

        DeclareLaunchArgument(
            'require_proximity_status',
            default_value='true',
            description=(
                'Reject commands when proximity status is unavailable'
            ),
        ),

        Node(
            package='thesis_core',
            executable='safety_supervisor',
            output='screen',
            parameters=[{
                'require_proximity_status': ParameterValue(
                    require_proximity_status,
                    value_type=bool,
                ),
            }],
        ),

        Node(
            package='thesis_simulation',
            executable='simulation_command_adapter',
            output='screen',
            parameters=[{
                'simulation_output_enabled':
                    ParameterValue(
                        simulation_output_enabled,
                        value_type=bool,
                    ),
            }],
        ),
    ])
