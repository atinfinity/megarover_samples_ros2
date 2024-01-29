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
    declare_gazebo = DeclareLaunchArgument(
        'gazebo', default_value='classic',
        choices=['classic', 'ignition'],
        description='Which gazebo simulator to use')
    declare_world_fname = DeclareLaunchArgument(
        'world_fname', default_value='vmegarover_sample',
        description='gazebo world name (no extension)')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    gui = LaunchConfiguration('gui')
    gazebo_simulator = LaunchConfiguration('gazebo')
    world_fname = LaunchConfiguration('world_fname')

    launch_file_dir = PathJoinSubstitution([FindPackageShare('megarover_samples_ros2'), 'launch'])

    # setup classic gazebo
    classic_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_file_dir, 'utils', 'classic_gazebo.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'world_fname': world_fname
        }.items(),
        condition=LaunchConfigurationEquals("gazebo", "classic")
    )
    # setup ignition gazebo
    ignition_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_file_dir, 'utils', 'ignition_gazebo.launch.py'])
        ),
        launch_arguments={
            'gui': gui,
            'world_fname': world_fname
        }.items(),
        condition=LaunchConfigurationEquals("gazebo", "ignition")
    )
    # setup robot_description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_file_dir, 'utils', 'robot_description.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': use_ros2_control,
            'gazebo': gazebo_simulator
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
        declare_gazebo,
        declare_world_fname,

        classic_gazebo_launch,
        ignition_gazebo_launch,

        robot_description_launch,
        ros2_control_launch
    ])
