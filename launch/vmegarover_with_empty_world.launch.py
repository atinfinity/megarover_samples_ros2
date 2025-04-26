from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control', default_value='false',
        choices=['true', 'false'],
        description='Use ros2_control(Gazebo) if true , Use gazebo_plugin if false.')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        choices=['true', 'false'],
        description='Set to "false" to run headless.')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    gui = LaunchConfiguration('gui')

    launch_file_dir = PathJoinSubstitution([FindPackageShare('megarover_samples_ros2'), 'launch'])

    # setup gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_file_dir, 'utils', 'gz.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': use_ros2_control,
            'gui': gui,
            'world_fname': 'empty'
        }.items(),
    )
    # setup robot_description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_file_dir, 'utils', 'robot_description.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': use_ros2_control,
        }.items()
    )
    # setup ros2_control
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_file_dir, 'utils', 'ros2_control.launch.py'])
        ),
        condition=IfCondition(use_ros2_control)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_ros2_control,
        declare_gui,

        gazebo_launch,

        robot_description_launch,
        ros2_control_launch
    ])
