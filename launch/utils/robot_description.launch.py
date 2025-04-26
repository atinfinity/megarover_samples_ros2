from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control', default_value='false',
        choices=['true', 'false'],
        description='Use ros2_control(Gazebo) if true , Use gazebo_plugin if false.')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    xacro_file = PathJoinSubstitution([
        FindPackageShare("megarover_samples_ros2"),
        'robots',
        'vmegarover.urdf.xacro'
    ])
    robot_description_content = Command(
        ['xacro', ' ', xacro_file, ' ',
         'use_ros2_control:=', use_ros2_control])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': robot_description_content
            }
        ]
    )
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_ros2_control,

        robot_state_publisher,
    ])
