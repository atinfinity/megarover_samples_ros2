import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration


def generate_launch_description():
    world_file_name = 'ign_depot.sdf'
    world = os.path.join(get_package_share_directory(
        'megarover_samples_ros2'), 'worlds', world_file_name)

    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control', default_value='false', description='Use ros2_control(Gazebo) if true , Use gazebo_plugin if false. gazebo_ros2_control is under development and deprecated')
    declare_world_fname = DeclareLaunchArgument(
        'world_fname', default_value=world, description='absolute path of gazebo world file')

    world_fname = LaunchConfiguration('world_fname')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='false')

    pkg_megarover_samples_ros2 = get_package_share_directory(
        'megarover_samples_ros2')
    launch_file_dir = os.path.join(pkg_megarover_samples_ros2, 'launch')
    scripts_file_dir = os.path.join(pkg_megarover_samples_ros2, 'scripts')

    pkg_gazebo_ros_sim = get_package_share_directory('ros_gz_sim')

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # 'gz_args': [' -r -v 4 ', world_fname]  # headless : --headless-rendering
            'gz_args': [' -r -v 0 empty.sdf']  # headless : --headless-rendering
        }.items(),
    )

    # xacro_file = os.path.join(
    #     pkg_megarover_samples_ros2, 'robots', 'vmegarover.urdf.xacro')
    create_fix_urdf_ignition = ExecuteProcess(
        # python3 [pkg]/create_fix_urdf_ignition.py (true|false)
        cmd=[[
            FindExecutable(name='python3'),
            ' ',
            scripts_file_dir+'/create_fix_urdf_ignition.py',
            ' ',
            use_ros2_control
        ]],
        shell=True
    )
    # generate by `create_fix_urdf_ignition`
    urdf_file = os.path.join(pkg_megarover_samples_ros2,
                             'robots', 'vmegarover.urdf')
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
                '-name', 'vmegarover',
                '-x', '0',
                '-y', '0',
                '-z', '0.5',
                '-file', urdf_file,
        ]
    )

    xacro_file = os.path.join(pkg_megarover_samples_ros2,
                             'robots', 'vmegarover.urdf.xacro')
    robot_description_content = Command(
        ['xacro', ' ', xacro_file, ' use_ros2_control:=', use_ros2_control])
    robot_description_params = {
        'use_sim_time': use_sim_time,
        'robot_description': robot_description_content
    }
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_params],
    )

    # Delay start of spawn_entity after `create_fix_urdf_ignition`
    delay_spawn_entity_after_create_fix_urdf_ignition = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_fix_urdf_ignition,
            on_exit=[spawn_entity]
        )
    )

    # use diff_drive_controller on ros2_control
    # robot_state_publisher_on_ros2_control_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [launch_file_dir, '/robot_state_publisher.launch.py']),
    #     launch_arguments={'use_sim_time': use_sim_time,
    #                       'use_ros2_control': use_ros2_control}.items(),
    # )

    return LaunchDescription([
        declare_use_ros2_control,
        declare_world_fname,
        gazebo_sim,
        create_fix_urdf_ignition,
        delay_spawn_entity_after_create_fix_urdf_ignition,       # execute spawn_entity
        # robot_state_publisher,
        # robot_state_publisher_on_ros2_control_launch,
    ])
