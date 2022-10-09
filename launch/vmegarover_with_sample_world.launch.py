import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'vmegarover_sample.world'
    world = os.path.join(get_package_share_directory('megarover_samples_ros2'), 'worlds', world_file_name)
    
    pkg_megarover_samples_ros2 = get_package_share_directory('megarover_samples_ros2')
    xacro_file = os.path.join(pkg_megarover_samples_ros2, 'robots', 'vmegarover.urdf.xacro')
    launch_file_dir = os.path.join(get_package_share_directory('megarover_samples_ros2'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        )
    
    gzclient = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        )

    # load to xacro
    doc = xacro.process_file(xacro_file)
    # generate urdf robot_description
    robot_desc = doc.toprettyxml(indent='  ')
    # fix descrption : `package://megarover_samples_ros2` to absolute path
    # ref. <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1191>
    fix_robot_desc = robot_desc.replace('package://megarover_samples_ros2', pkg_megarover_samples_ros2)
    # write urdf file for spawn_entity.py
    urdf_file = os.path.join(pkg_megarover_samples_ros2, 'robots', 'vmegarover.urdf')
    with open(urdf_file, 'w') as f:
        f.write(fix_robot_desc)
        
    spawn_entiry = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
                '-entity', 'vmegarover',
                '-x', '0',
                '-y', '0',
                '-z', '1',
                '-file', urdf_file,
        ]
    )

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/robot_state_publisher.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    cmd_vel_and_odom_relay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/cmd_vel_and_odom_relay.launch.py']))

    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_entiry,
        robot_state_publisher_launch,
        cmd_vel_and_odom_relay
    ])
