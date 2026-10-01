"""Launch the complete JACO simulation and preventive control stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build one launch description for simulation, safety and UI."""
    simulation_share = get_package_share_directory('thesis_simulation')
    launch_dir = os.path.join(simulation_share, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    obstacle_source_mode = LaunchConfiguration('obstacle_source_mode')
    obstacle_x = LaunchConfiguration('obstacle_x')
    obstacle_y = LaunchConfiguration('obstacle_y')
    obstacle_z = LaunchConfiguration('obstacle_z')
    simulation_output_enabled = LaunchConfiguration(
        'simulation_output_enabled'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'jaco_gazebo.launch.py')
        ),
        launch_arguments={
            'obstacle_source_mode': obstacle_source_mode,
            'obstacle_x': obstacle_x,
            'obstacle_y': obstacle_y,
            'obstacle_z': obstacle_z,
        }.items(),
    )

    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'safety_pipeline.launch.py')
        ),
        launch_arguments={
            'simulation_output_enabled': simulation_output_enabled,
            'require_proximity_status': 'true',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    demo_obstacle = Node(
        package='thesis_core',
        executable='obstacle_demo_source',
        name='obstacle_demo_source',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_demo_obstacle')),
        parameters=[{
            'use_sim_time': ParameterValue(
                use_sim_time, value_type=bool
            ),
            'x': ParameterValue(obstacle_x, value_type=float),
            'y': ParameterValue(obstacle_y, value_type=float),
            'z': ParameterValue(obstacle_z, value_type=float),
            'radius': ParameterValue(
                LaunchConfiguration('obstacle_radius'), value_type=float
            ),
            'uncertainty': ParameterValue(
                LaunchConfiguration('obstacle_uncertainty'),
                value_type=float,
            ),
            'rate_hz': ParameterValue(
                LaunchConfiguration('obstacle_rate_hz'), value_type=float
            ),
        }],
    )

    horizon_preview = Node(
        package='thesis_core',
        executable='horizon_preview',
        name='horizon_preview',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(
                use_sim_time, value_type=bool
            ),
            'source': LaunchConfiguration('preview_source'),
            'horizon_sec': ParameterValue(
                LaunchConfiguration('horizon_sec'), value_type=float
            ),
            'samples': ParameterValue(
                LaunchConfiguration('samples'), value_type=int
            ),
            'margin_m': ParameterValue(
                LaunchConfiguration('margin_m'), value_type=float
            ),
            'rate_hz': ParameterValue(
                LaunchConfiguration('preview_rate_hz'), value_type=float
            ),
        }],
    )

    gui = Node(
        package='thesis_ui',
        executable='joint_gui',
        name='joint_control_gui_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_gui')),
        parameters=[{
            'use_sim_time': ParameterValue(
                use_sim_time, value_type=bool
            ),
            'require_system_readiness': True,
        }],
    )

    readiness = Node(
        package='thesis_simulation',
        executable='system_readiness',
        name='system_readiness',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'obstacle_source_mode': obstacle_source_mode,
            'use_demo_obstacle': ParameterValue(
                LaunchConfiguration('use_demo_obstacle'), value_type=bool
            ),
            'require_gui': ParameterValue(
                LaunchConfiguration('start_gui'), value_type=bool
            ),
            'readiness_timeout_sec': ParameterValue(
                LaunchConfiguration('readiness_timeout_sec'),
                value_type=float,
            ),
            'readiness_rate_hz': ParameterValue(
                LaunchConfiguration('readiness_rate_hz'), value_type=float
            ),
        }],
    )

    declarations = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo simulation clock.',
        ),
        DeclareLaunchArgument(
            'obstacle_source_mode', default_value='topic',
            description='Obstacle source consumed by the proximity monitor.',
        ),
        DeclareLaunchArgument(
            'use_demo_obstacle', default_value='true',
            description=(
                'Publish a synthetic obstacle for integration testing. '
                'Set false when a real perception source publishes the input.'
            ),
        ),
        DeclareLaunchArgument(
            'obstacle_x', default_value='0.60',
            description='Obstacle X coordinate in world, metres.',
        ),
        DeclareLaunchArgument(
            'obstacle_y', default_value='0.0',
            description='Obstacle Y coordinate in world, metres.',
        ),
        DeclareLaunchArgument(
            'obstacle_z', default_value='0.65',
            description='Obstacle Z coordinate in world, metres.',
        ),
        DeclareLaunchArgument(
            'obstacle_radius', default_value='0.12',
            description='Synthetic obstacle radius, metres.',
        ),
        DeclareLaunchArgument(
            'obstacle_uncertainty', default_value='0.02',
            description='Synthetic obstacle uncertainty, metres.',
        ),
        DeclareLaunchArgument(
            'obstacle_rate_hz', default_value='10.0',
            description='Synthetic obstacle publication rate.',
        ),
        DeclareLaunchArgument(
            'simulation_output_enabled', default_value='true',
            description=(
                'Send supervisor-approved commands to the Gazebo controller.'
            ),
        ),
        DeclareLaunchArgument(
            'horizon_sec', default_value='1.0',
            description='Predictive preview horizon, seconds.',
        ),
        DeclareLaunchArgument(
            'samples', default_value='21',
            description='Number of discrete samples in the preview horizon.',
        ),
        DeclareLaunchArgument(
            'margin_m', default_value='0.02',
            description='Added spatial margin for the preview, metres.',
        ),
        DeclareLaunchArgument(
            'preview_rate_hz', default_value='10.0',
            description='Predictive preview update rate.',
        ),
        DeclareLaunchArgument(
            'preview_source', default_value='auto',
            description='Preview source: auto, intent or execution.',
        ),
        DeclareLaunchArgument(
            'start_gui', default_value='true',
            description='Start the joint control GUI.',
        ),
        DeclareLaunchArgument(
            'readiness_timeout_sec', default_value='0.75',
            description='Wall-time limit for live ROS data, seconds.',
        ),
        DeclareLaunchArgument(
            'readiness_rate_hz', default_value='1.0',
            description='System availability check rate.',
        ),
    ]

    return LaunchDescription(
        declarations + [
            gazebo,
            safety,
            demo_obstacle,
            horizon_preview,
            gui,
            readiness,
        ]
    )
