import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('tolya')

    start_localization = Node(
        parameters=[
            os.path.join(pkg_dir, 'params', 'loc_slam_params.yaml')],
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(start_localization)

    return ld
