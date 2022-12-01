import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from yaml import safe_load
from json import dumps

def generate_launch_description():
    pkg_tolya = get_package_share_directory('tolya')
    pkg_brc = get_package_share_directory('brc')

    points_file = os.path.join(pkg_brc, 'params', 'points.yaml')
    
    if os.path.exists(points_file):
        with open(points_file, 'r') as f:
            points = dumps(safe_load(f.read()))
    else:
        points = {}

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    tolya_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tolya, 'launch', 'tolya_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    start_task = Node(
        package='brc',
        executable='main_task', 
        parameters=[{'points': points}])

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(tolya_launch_cmd)
    ld.add_action(start_task)

    return ld
