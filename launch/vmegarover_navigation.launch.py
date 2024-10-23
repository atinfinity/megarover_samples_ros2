import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    map_dir = os.path.join(get_package_share_directory(
        'megarover_samples_ros2'), 'maps')
    map_file = LaunchConfiguration('map', default=os.path.join(
        map_dir, 'vmegarover_samplemap.yaml'))

    param_dir = os.path.join(get_package_share_directory(
        'megarover_samples_ros2'), 'config')
    nav2_param_file = LaunchConfiguration(
        'params', default=os.path.join(param_dir, 'navigation_param.yaml'))
    nav2_ros2_controll_param_file = LaunchConfiguration(
        'params', default=os.path.join(param_dir, 'navigation_ros2_control_param.yaml'))

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(
        get_package_share_directory('megarover_samples_ros2'), 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'navigation.rviz')

    declare_map_cmd = DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load')
    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control', default_value='false',
        choices=['true', 'false'],
        description='Use ros2_control(Gazebo)')

    nav2_bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_file}.items(),
            condition=UnlessCondition(use_ros2_control)
        )
    nav2_bringup_ros2_control_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': nav2_ros2_controll_param_file}.items(),
            condition=IfCondition(use_ros2_control)
        )
    cmd_vel_relay_cmd = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        parameters=[
            {'use_sim_time': use_sim_time,},
            {'input_topic': "/cmd_vel"},
            {'output_topic': '/diff_drive_controller/cmd_vel'},
        ],
        condition=IfCondition(use_ros2_control)
    )
    rviz2_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_ros2_control)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(nav2_bringup_ros2_control_cmd)
    ld.add_action(cmd_vel_relay_cmd)
    ld.add_action(rviz2_cmd)
    return ld

    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
    """
