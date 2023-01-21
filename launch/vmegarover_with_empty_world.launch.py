import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable


def generate_launch_description():
    sdf_dir = os.path.join(get_package_share_directory('megarover_samples_ros2'), 'models/vmegarover')
    sdf_file = os.path.join(sdf_dir, 'vmegarover.sdf')

    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true', description='Set to "false" to run headless.')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')

    pkg_megarover_samples_ros2 = get_package_share_directory(
        'megarover_samples_ros2')
    launch_file_dir = os.path.join(pkg_megarover_samples_ros2, 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'gui': gui
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
                '-z', '1',
                '-file', sdf_file,
        ]
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/robot_state_publisher.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        declare_gui,
        gazebo,
        spawn_entity,
        robot_state_publisher,
    ])
