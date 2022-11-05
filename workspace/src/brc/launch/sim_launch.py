import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    world_file_name = 'tolya.world'
    robot_file_name = 'tolya.urdf'

    pkg_tolya = get_package_share_directory('tolya')
    pkg_tolya_description = get_package_share_directory('tolya_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_brc = get_package_share_directory('brc')

    urdf_file = os.path.join(pkg_tolya_description, 'urdf', robot_file_name)
    world_file = os.path.join(pkg_brc, 'worlds', world_file_name)

    assert os.path.exists(
        urdf_file), "{robot_file_name} doesnt exist in {urdf_file}"
    assert os.path.exists(
        world_file), "{world_file_name} doesnt exist in {world_file}"

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui_config = DeclareLaunchArgument(name='gui', default_value='True')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_brc, 'configs', 'brc.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    start_gz_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    start_gz_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tolya',
            '-file', urdf_file,
            '-x', '0',
            '-y', '0',
            '-z', '0.1',
            '-Y', '1.5707'
        ],
        output='screen',
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    
    tolya_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tolya, 'launch', 'tolya_launch.py')),
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(gui_config)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_gz_server_cmd)
    ld.add_action(start_gz_client_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(joint_state_publisher_node)
    
    ld.add_action(tolya_launch_cmd)

    return ld
