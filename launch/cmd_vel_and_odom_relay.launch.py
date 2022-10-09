from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    prefix_str = LaunchConfiguration('prefix_str', default='/vmegarover')

    cmd_vel_relay = Node(package='topic_tools', executable='relay', name='cmd_vel_relay', output='screen',
                         parameters=[{'input_topic': '/cmd_vel', 'output_topic': [prefix_str, '/cmd_vel']}])
    odom_relay = Node(package='topic_tools', executable='relay', name='odom_relay', output='screen',
                         parameters=[{'input_topic': [prefix_str,'/odom'], 'output_topic': '/odom'}])

    return LaunchDescription([
        DeclareLaunchArgument(
            'prefix_str',
            default_value='/vmegarover',
            description='Prefix of target cmd_vel and odom topic'),
        cmd_vel_relay,
        odom_relay
    ])
