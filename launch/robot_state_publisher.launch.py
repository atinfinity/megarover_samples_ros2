import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_dir = os.path.join(get_package_share_directory(
        'megarover_samples_ros2'), 'robots')
    xacro_file = os.path.join(xacro_dir, 'vmegarover.urdf.xacro')
    robot_description = {'robot_description': Command(
        ['xacro', ' ', xacro_file])}

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
        
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, robot_description])

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    cmd_vel_relay = Node(package='topic_tools', executable='relay', name='cmd_vel_relay', output='screen',
                         parameters=[{'input_topic': '/cmd_vel', 'output_topic': '/vmegarover/cmd_vel'}])
    odom_relay = Node(package='topic_tools', executable='relay', name='odom_relay', output='screen',
                      parameters=[{'input_topic': '/vmegarover/odom', 'output_topic': '/odom'}])

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        joint_state_publisher,
        cmd_vel_relay,
        odom_relay
    ])
