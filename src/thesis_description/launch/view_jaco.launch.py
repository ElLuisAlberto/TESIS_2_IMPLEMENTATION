from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_share = FindPackageShare('thesis_description')

    xacro_file = PathJoinSubstitution([
        package_share,
        'urdf',
        'j2n6s300_standalone.xacro'
    ])

    rviz_config = PathJoinSubstitution([
        package_share,
        'rviz',
        'jaco.rviz'
    ])

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ',
                xacro_file
            ]),
            value_type=str
        )
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[robot_description]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
