import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_file_name = 'tolya.urdf'

    pkg_tolya = get_package_share_directory('tolya')
    pkg_tolya_description = get_package_share_directory('tolya_description')

    urdf_file = os.path.join(pkg_tolya_description, 'urdf', robot_file_name)

    assert os.path.exists(
        urdf_file), "{robot_file_name} doesnt exist in {urdf_file}"

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', urdf_file])}]
    )

    start_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tolya, 'launch', 'bringup_launch.py')
        )
    )

    # start_cube_detection_node = Node(
    #     package='tolya', executable='cube_detection')


    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(start_bringup)
    # ld.add_action(start_cube_detection_node)

    return ld
