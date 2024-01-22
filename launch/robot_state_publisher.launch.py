from inspect import isfunction
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true')
    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control', default_value='false', description='Use ros2_control(Gazebo) if true , Use gazebo_plugin if false. gazebo_ros2_control is under development and deprecated')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='false')

    xacro_dir = os.path.join(get_package_share_directory(
        'megarover_samples_ros2'), 'robots')
    xacro_file = os.path.join(xacro_dir, 'vmegarover.urdf.xacro')
    robot_description_content = Command(
        ['xacro', ' ', xacro_file, ' use_ros2_control:=', use_ros2_control])

    robot_description = {
        'use_sim_time': use_sim_time,
        'robot_description': robot_description_content
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        condition=IfCondition(use_ros2_control)
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller",
                   "--controller-manager", "/controller_manager"],
        condition=IfCondition(use_ros2_control)
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner]
        )
    )
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_ros2_control,

        robot_state_publisher,
        joint_state_publisher,
        # execute if use_ros2_control is true
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ])
