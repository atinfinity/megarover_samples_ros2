from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


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
        description='gazebo world file name')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world_fname = LaunchConfiguration('world_fname')

    pkg_megarover_samples_ros2 = FindPackageShare('megarover_samples_ros2')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            # 'gz_args': [' -r -v 4 ', world_fname]  # headless : --headless-rendering
            'gz_args': [' -r -v 1 empty.sdf']  # headless : --headless-rendering
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
                '-z', '1',
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
            # ros <-  ignitoin sync : clock, tf(odom to base_footprinf), odom, scan, depth_image, image, points
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/front_camera_sensor/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/front_camera_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/front_camera_sensor/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            # ros <-> ignition sync : cmd_vel
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
        ],
        remappings=[
            ("/front_camera_sensor/depth_image", "/front_camera_sensor/depth/image_raw"),
            ("/front_camera_sensor/image", "/front_camera_sensor/image_raw"),
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

        gazebo,
        spawn_entity,
        base_topic_bridge,
        rgb_camera_info_bridge,
        depth_camera_info_bridge
    ])
