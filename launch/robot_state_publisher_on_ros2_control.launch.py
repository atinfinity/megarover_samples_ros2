import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_dir = os.path.join(get_package_share_directory(
        'megarover_samples_ros2'), 'robots')
    xacro_file = os.path.join(xacro_dir, 'vmegarover.urdf.xacro')
    robot_description = {'robot_description': Command(
        ['xacro', ' ', xacro_file, ' use_ros2_control:=true'])}

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('megarover_samples_ros2'),
            'config',
            'diff_drive_controller.yaml',
        ]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}, robot_controllers],
        output="screen"
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
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
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "-c", "/controller_manager"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner]
        )
    )

    cmd_vel_relay = Node(package='topic_tools', executable='relay', name='cmd_vel_relay', output='screen',
                         parameters=[{'input_topic': '/cmd_vel', 'output_topic': '/diff_drive_controller/cmd_vel_unstamped'}])
    odom_relay = Node(package='topic_tools', executable='relay', name='odom_relay', output='screen',
                      parameters=[{'input_topic': '/diff_drive_controller/odom', 'output_topic': '/odom'}])

    return LaunchDescription([
        declare_use_sim_time,
        control_node,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        cmd_vel_relay,
        odom_relay
    ])
