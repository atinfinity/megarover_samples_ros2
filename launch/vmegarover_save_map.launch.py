import os

from launch import LaunchDescription
from launch_ros.actions.node import Node
from pathlib import Path

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_saver_cli',
            name='map_saver_cli',
            output='screen',
            arguments=['-f', str(Path.home())+'/maps/vmegarover_samplemap'],
            parameters=[{'save_map_timeout': 10000}]),
    ])
#    map_saver_cli = Node(
#        package='nav2_map_server',
#        executable='map_saver_cli',
#        parameters=[{'save_map_timeout': 10000}],
#        arguments=['-f', str(Path.home())+'/maps/vmegarover_samplemap'],
#        output='screen'
#    )

#    ld = LaunchDescription()
#    ld.add_action(map_saver_cli)
#    return ld
