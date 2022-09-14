import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'tolya.world'
    robot_file_name = 'tolya.xacro'

    pkg_tolya_description = get_package_share_directory('tolya_description')
    xacro_file = os.path.join(pkg_tolya_description, 'urdf', robot_file_name)   
    assert os.path.exists(xacro_file), "The tolya.xacro doesnt exist in " + str(xacro_file)

    world = os.path.join(get_package_share_directory(
        'tolya_gazebo'), 'worlds', world_file_name)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml().replace('"', '\\"')
    swpan_args = '{name: \"tolya\", xml: \"' + robot_desc + '\" }'
    
    return LaunchDescription([
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'tolya_gazebo', 'gazebo_simulation']),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', world,
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo',
                 'use_sim_time', use_sim_time],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity',
                 'gazebo_msgs/SpawnEntity', swpan_args],
            output='screen'),
    ])