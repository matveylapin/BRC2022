import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('tolya')
    pkg_launch_dir = os.path.join(pkg_dir, 'launch')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_slam = LaunchConfiguration('use_slam')
    map_yaml_file = LaunchConfiguration('map_yaml')

    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true')

    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml',
        default_value=os.path.join(
            pkg_dir, 'maps', 'brc_map.yaml'),
        condition=UnlessCondition(use_slam))

    start_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_launch_dir, 'slam_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file}.items(),
        condition=IfCondition(use_slam))

    start_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_launch_dir, 'loc_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'map_yaml': map_yaml_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file}.items(),
        condition=UnlessCondition(use_slam))

    start_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_launch_dir, 'navigation_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'params_file': params_file}.items())

    ld = LaunchDescription()
    ld.add_action(declare_autostart)
    ld.add_action(declare_map_yaml)
    ld.add_action(start_slam)
    ld.add_action(start_localization)
    ld.add_action(start_navigation)

    return ld
