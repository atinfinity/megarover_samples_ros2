from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        choices=['true', 'false'],
        description='Set to "false" to run headless.')
    declare_world_fname = DeclareLaunchArgument(
        'world_fname', default_value='',
        description='gazebo world file name')

    gui = LaunchConfiguration('gui')
    world_fname = LaunchConfiguration('world_fname')

    pkg_megarover_samples_ros2 = FindPackageShare('megarover_samples_ros2')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            # 'gz_args': [' -r -v 4 ', world_fname]  # headless : --headless-rendering
            'gz_args': [' -r -v 0 empty.sdf']  # headless : --headless-rendering
        }.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
                '-entity', 'vmegarover',
                '-x', '0',
                '-y', '0',
                '-z', '1',
                '-topic', 'robot_description',
        ]
    )
    return LaunchDescription([
        declare_gui,
        declare_world_fname,

        gazebo,
        spawn_entity,
    ])
