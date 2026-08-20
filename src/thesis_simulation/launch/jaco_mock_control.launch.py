import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    simulation_share = get_package_share_directory('thesis_simulation')
    description_share = get_package_share_directory('thesis_description')

    xacro_file = os.path.join(
        simulation_share,
        'urdf',
        'jaco_mock.ros2_control.xacro'
    )

    controllers_file = os.path.join(
        simulation_share,
        'config',
        'jaco_controllers.yaml'
    )

    rviz_config = os.path.join(
        description_share,
        'rviz',
        'jaco.rviz'
    )

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ',
                xacro_file
            ]),
            value_type=str
        )
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            controllers_file
        ],
        remappings=[
            ('~/robot_description', '/robot_description')
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            controllers_file
        ],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            controllers_file
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=[
            '-d',
            rviz_config
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        rviz
    ])
