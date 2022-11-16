import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from yaml import safe_load
from json import dumps


def generate_launch_description():
    robot_file_name = 'tolya.urdf'

    pkg_tolya = get_package_share_directory('tolya')
    pkg_brc = get_package_share_directory('brc')
    urdf_file = os.path.join(pkg_tolya, 'urdf', robot_file_name)
    markers_file = os.path.join(pkg_brc, 'params', 'markers.yaml')

    assert os.path.exists(
        urdf_file), "{robot_file_name} doesnt exist in {urdf_file}"

    if os.path.exists(markers_file):
        with open(markers_file, 'r') as f:
            markers = dumps(safe_load(f.read()))
    else:
        markers = []

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam = LaunchConfiguration('use_slam')
    params_file = LaunchConfiguration('params_file')

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='')

    declare_use_slam = DeclareLaunchArgument(
        'use_slam', default_value='false')

    declare_nav2_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_tolya, 'params', 'nav2_params.yaml'))

    start_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', urdf_file])}]
    )

    start_yandex_maps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tolya, 'launch', 'yandex_maps_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'use_slam': use_slam}.items()
    )

    start_cv_node = Node(
        package='tolya', executable='cv_node', parameters=[{'markers': markers}])

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_slam)
    ld.add_action(declare_nav2_params_file)
    ld.add_action(start_robot_state_publisher_node)
    ld.add_action(start_yandex_maps)
    ld.add_action(start_cv_node)

    return ld
