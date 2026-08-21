import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    simulation_share = get_package_share_directory(
        'thesis_simulation'
    )

    description_share = get_package_share_directory(
        'thesis_description'
    )

    xacro_file = os.path.join(
        simulation_share,
        'urdf',
        'jaco_gazebo.ros2_control.xacro'
    )

    world_file = os.path.join(
        simulation_share,
        'worlds',
        'jaco_empty.sdf'
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


    # =========================================================
    # Robot description
    # =========================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description
        ]
    )


    # =========================================================
    # Gazebo Fortress
    # =========================================================

    gazebo = ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            '-r',
            '-v',
            '3',
            world_file
        ],
        output='screen'
    )


    # =========================================================
    # Spawn JACO from /robot_description
    # =========================================================

    spawn_jaco = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_jaco',
        output='screen',
        arguments=[
            '-name',
            'jaco',
            '-topic',
            'robot_description',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.0'
        ]
    )


    # =========================================================
    # Controllers
    # =========================================================

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ]
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager'
        ]
    )


    # =========================================================
    # RViz
    # =========================================================

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

        gazebo,

        # Wait for Gazebo server before spawning robot
        TimerAction(
            period=2.0,
            actions=[
                spawn_jaco
            ]
        ),

        # gz_ros2_control creates controller_manager
        # when the robot is inserted.
        TimerAction(
            period=5.0,
            actions=[
                joint_state_broadcaster_spawner
            ]
        ),

        TimerAction(
            period=6.0,
            actions=[
                arm_controller_spawner
            ]
        ),

        rviz
    ])
