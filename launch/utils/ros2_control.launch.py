from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller",
                   "--controller-manager", "/controller_manager"],
        output='screen'
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ])
