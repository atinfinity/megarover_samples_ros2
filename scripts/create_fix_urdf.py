import os
import sys
import xacro
from ament_index_python.packages import get_package_share_directory

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage : python create_urdf.py (true|false)")
        raise Exception
    
    use_ros2_control = 'false' if sys.argv[1]=='false' else 'true'

    pkg_megarover_samples_ros2 = get_package_share_directory(
        'megarover_samples_ros2')
    xacro_file = os.path.join(
        pkg_megarover_samples_ros2, 'robots', 'vmegarover.urdf.xacro')

    # load to xacro
    doc = xacro.process_file(xacro_file, mappings={
                             'use_ros2_control': use_ros2_control})
    # generate urdf robot_description
    robot_desc = doc.toprettyxml(indent='  ')
    # fix descrption : `package://megarover_samples_ros2` to absolute path
    # ref. <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1191>
    fix_robot_desc = robot_desc.replace(
        'package://megarover_samples_ros2', pkg_megarover_samples_ros2)
    # write urdf file for spawn_entity.py
    urdf_file = os.path.join(pkg_megarover_samples_ros2,
                             'robots', 'vmegarover.urdf')
    with open(urdf_file, 'w') as f:
        f.write(fix_robot_desc)
