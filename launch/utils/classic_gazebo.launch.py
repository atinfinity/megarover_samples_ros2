from os import pathsep

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, LaunchConfiguration,
                                  PathJoinSubstitution)


def generate_launch_description():
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        choices=['true', 'false'],
        description='Set to "false" to run headless.')
    declare_world_fname = DeclareLaunchArgument(
        'world_fname', default_value='',
        description='gazebo world name (no extension)')

    gui = LaunchConfiguration('gui')
    world_fname = LaunchConfiguration('world_fname')

    pkg_megarover_samples_ros2 = FindPackageShare('megarover_samples_ros2')

    set_env_gazebo_resource = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value=''),
            pathsep,
            '/usr/share/gazebo-11',
            pathsep,
            PathJoinSubstitution([pkg_megarover_samples_ros2, 'worlds', 'classic'])]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': [world_fname, '.world'],
            'gui': gui,
            'params_file': PathJoinSubstitution([
                pkg_megarover_samples_ros2,
                'config', 'gazebo_publish_rate.yaml'
            ])
        }.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
                '-entity', 'vmegarover',
                '-x', '0',
                '-y', '0',
                '-z', '0.3',
                '-topic', 'robot_description',
        ]
    )
    return LaunchDescription([
        declare_gui,
        declare_world_fname,

        set_env_gazebo_resource,

        gazebo,
        spawn_entity,
    ])
