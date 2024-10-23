from os import pathsep

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        choices=['true', 'false'],
        description='Set to "false" to run headless.')
    declare_world_fname = DeclareLaunchArgument(
        'world_fname', default_value='',
        description='gazebo world name (no extension)')
    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control', default_value='false',
        choices=['true', 'false'],
        description='Use ros2_control(Gazebo) if true , Use gazebo_plugin if false.')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world_fname = LaunchConfiguration('world_fname')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    pkg_megarover_samples_ros2 = FindPackageShare('megarover_samples_ros2')
    worlds_dir = PathJoinSubstitution([pkg_megarover_samples_ros2, 'worlds', 'gz'])

    set_env_gazebo_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            pathsep,
            worlds_dir]
    )

    gz_args = [worlds_dir, '/', world_fname, '.sdf', ' ',
               # log level : 1
               '-v 1', ' ',
               # autostart
               '-r', ' ',
               # headless
               __headless_rendering(gui)]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': gz_args
        }.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
                '-entity', 'vmegarover',
                '-x', '0',
                '-y', '0',
                '-z', '0.3',
                '-topic', 'robot_description',
        ]
    )

    base_topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name='base_topic_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            # ros <-  gz sync : clock, tf(odom to base_footprinf), odom, scan, depth_image, image, points
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            # ros <-> gz sync : cmd_vel, odom
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        ],
        condition=UnlessCondition(use_ros2_control)
    )
    base_topic_bridge_ros2_control = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name='base_topic_bridge_ros2_control',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            # ros <-  gz sync : clock, tf(odom to base_footprinf), odom, scan, depth_image, image, points
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            # ros <-> gz sync : cmd_vel, odom
            "/cmd_vel@geometry_msgs/msg/TwistStamped@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        ],
        condition=IfCondition(use_ros2_control)
    )
    scan_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name='scan_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ]
    )
    image_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name='image_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            "/front_camera_sensor/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/front_camera_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        remappings=[
            ("/front_camera_sensor/depth_image", "/front_camera_sensor/depth/image_raw"),
            ("/front_camera_sensor/image", "/front_camera_sensor/image_raw"),
        ]
    )
    points_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name='points_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            "/front_camera_sensor/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ]
    )
    rgb_camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name='rgb_camera_info_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            "/front_camera_sensor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
    )
    depth_camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name='depth_camera_info_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            "/front_camera_sensor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/front_camera_sensor/camera_info", "/front_camera_sensor/depth/camera_info")
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        declare_world_fname,
        declare_use_ros2_control,

        set_env_gazebo_resource,

        gazebo,
        spawn_entity,

        # bridges
        base_topic_bridge,
        base_topic_bridge_ros2_control,
        scan_bridge,
        image_bridge,
        points_bridge,
        rgb_camera_info_bridge,
        depth_camera_info_bridge
    ])

def __headless_rendering(gui):
    cmd = ['"" if "true" == "', gui, '" else "--headless-rendering -s"']
    py_cmd = PythonExpression(cmd)
    return py_cmd