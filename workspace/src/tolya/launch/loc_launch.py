import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument


def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    pkg_dir = get_package_share_directory('tolya')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    slam_launch_file = os.path.join(
        slam_toolbox_dir, 'launch', 'online_sync_launch.py')


    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_dir, 'params', 'loc_slam_params.yaml'))


    start_slam_toolbox_with_params = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'use_sim_time': use_sim_time,
                          'slam_params_file': slam_params_file}.items())

    ld = LaunchDescription()
    ld.add_action(declare_slam_params_file)

    ld.add_action(start_slam_toolbox_with_params)


    return ld
