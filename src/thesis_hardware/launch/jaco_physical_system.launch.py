"""Start the complete preventive stack for the physical Kinova JACO2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the physical stack with output disabled by default."""
    bridge_share = get_package_share_directory(
        'thesis_hardware_bridge'
    )
    description_share = get_package_share_directory(
        'thesis_description'
    )
    core_share = get_package_share_directory(
        'thesis_core'
    )

    default_hardware_config = os.path.join(
        bridge_share, 'config', 'jaco_hardware.yaml'
    )
    default_capsule_config = os.path.join(
        core_share, 'config', 'jaco_capsules.yaml'
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
    hardware_config = LaunchConfiguration('hardware_config_file')
    capsule_config = LaunchConfiguration('capsule_config_file')
    obstacle_input_topic = LaunchConfiguration('obstacle_input_topic')
    use_demo_obstacle = LaunchConfiguration('use_demo_obstacle')
    start_gui = LaunchConfiguration('start_gui')
    start_rviz = LaunchConfiguration('start_rviz')
    allow_gui_arm_control = LaunchConfiguration(
        'allow_gui_arm_control'
    )
    kinova_library_dir = LaunchConfiguration('kinova_library_dir')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str,
        )
    }

    declarations = [
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Use an offline JACO state and motion backend.',
        ),
        DeclareLaunchArgument(
            'hardware_output_enabled',
            default_value='false',
            description=(
                'Permit runtime arming of physical or mock motion.'
            ),
        ),
        DeclareLaunchArgument(
            'expected_serial_number',
            default_value='',
            description='Exact serial required when more than one arm exists.',
        ),
        DeclareLaunchArgument(
            'hardware_config_file',
            default_value=default_hardware_config,
            description='JACO adapter ROS 2 parameter file.',
        ),
        DeclareLaunchArgument(
            'capsule_config_file',
            default_value=default_capsule_config,
            description='Physical-arm capsule geometry parameter file.',
        ),
        DeclareLaunchArgument(
            'obstacle_input_topic',
            default_value='/thesis/obstacle_input',
            description='Obstacle input before transformation to world.',
        ),
        DeclareLaunchArgument(
            'use_demo_obstacle',
            default_value='false',
            description=(
                'Publish a synthetic obstacle. Use only for offline/bench tests.'
            ),
        ),
        DeclareLaunchArgument(
            'demo_obstacle_x',
            default_value='2.0',
            description='Synthetic test obstacle X coordinate in metres.',
        ),
        DeclareLaunchArgument(
            'demo_obstacle_y',
            default_value='0.0',
            description='Synthetic test obstacle Y coordinate in metres.',
        ),
        DeclareLaunchArgument(
            'demo_obstacle_z',
            default_value='0.65',
            description='Synthetic test obstacle Z coordinate in metres.',
        ),
        DeclareLaunchArgument(
            'start_gui',
            default_value='true',
            description='Start the joint command interface.',
        ),
        DeclareLaunchArgument(
            'allow_gui_arm_control',
            default_value='true',
            description=(
                'Show arm/disarm controls in the physical GUI. Runtime '
                'arming still requires hardware_output_enabled=true.'
            ),
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2.',
        ),
        DeclareLaunchArgument(
            'kinova_library_dir',
            default_value=os.path.join(
                os.environ.get(
                    'KINOVA_ROOT',
                    os.path.expanduser(
                        '~/Escritorio/TESIS_2_DEPENDENCIES/'
                        'kinova-ros/kinova_driver'
                    ),
                ),
                'lib',
                'x86_64-linux-gnu',
            ),
            description=(
                'Directory containing the Kinova command and communication '
                'layer shared libraries.'
            ),
        ),
    ]

    nodes = [
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
            additional_env={
                'LD_LIBRARY_PATH': [
                    kinova_library_dir,
                    ':',
                    EnvironmentVariable(
                        'LD_LIBRARY_PATH', default_value=''
                    ),
                ],
            },
            parameters=[
                hardware_config,
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
            package='thesis_core',
            executable='obstacle_input_bridge',
            name='obstacle_input_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'input_topic': obstacle_input_topic,
                'output_topic': '/thesis/obstacle_in_model_frame',
                'target_frame': 'world',
                'max_age_sec': 0.5,
            }],
        ),
        Node(
            package='thesis_core',
            executable='obstacle_demo_source',
            name='obstacle_demo_source',
            output='screen',
            condition=IfCondition(use_demo_obstacle),
            parameters=[{
                'use_sim_time': False,
                'topic': obstacle_input_topic,
                'frame_id': 'world',
                'x': ParameterValue(
                    LaunchConfiguration('demo_obstacle_x'),
                    value_type=float,
                ),
                'y': ParameterValue(
                    LaunchConfiguration('demo_obstacle_y'),
                    value_type=float,
                ),
                'z': ParameterValue(
                    LaunchConfiguration('demo_obstacle_z'),
                    value_type=float,
                ),
                'radius': 0.10,
                'uncertainty': 0.02,
                'rate_hz': 10.0,
            }],
        ),
        Node(
            package='thesis_core',
            executable='proximity_monitor',
            name='proximity_monitor',
            output='screen',
            parameters=[
                capsule_config,
                {
                    'use_sim_time': False,
                    'obstacle_source_mode': 'topic',
                    'obstacle_input_topic':
                        '/thesis/obstacle_in_model_frame',
                },
            ],
        ),
        Node(
            package='thesis_core',
            executable='safety_supervisor',
            name='safety_supervisor_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'require_proximity_status': True,
                'runtime_recovery_required_samples': 5,
                'runtime_max_scale_increment': 0.10,
                'runtime_recovery_sample_period_sec': 0.10,
                'jog_command_timeout_sec': 0.25,
                'max_jog_message_age_sec': 0.15,
            }],
        ),
        Node(
            package='thesis_core',
            executable='horizon_preview',
            name='horizon_preview',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'source': 'auto',
                'horizon_sec': 1.0,
                'samples': 21,
                'margin_m': 0.02,
                'rate_hz': 10.0,
            }],
        ),
        Node(
            package='thesis_core',
            executable='capsule_visualizer',
            name='capsule_visualizer',
            output='screen',
            parameters=[capsule_config, {'use_sim_time': False}],
        ),
        Node(
            package='thesis_hardware',
            executable='hardware_readiness',
            name='hardware_readiness',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'require_armed': True,
                'require_proximity_status': True,
                'timeout_sec': 0.50,
                'rate_hz': 2.0,
            }],
        ),
        Node(
            package='thesis_ui',
            executable='joint_gui',
            name='joint_control_gui_node',
            output='screen',
            condition=IfCondition(start_gui),
            parameters=[{
                'use_sim_time': False,
                'require_system_readiness': True,
                'operation_mode': 'hardware',
                'allow_hardware_arm_control': ParameterValue(
                    allow_gui_arm_control, value_type=bool
                ),
            }],
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
    ]
    return LaunchDescription(declarations + nodes)
