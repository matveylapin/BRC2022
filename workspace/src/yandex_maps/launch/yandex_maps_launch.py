from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    start_controller = Node(
        package='yandex_maps', executable='yandex_maps_controller')
    start_planner = Node(
        package='yandex_maps', executable='yandex_maps_planner')
    start_navigator = Node(
        package='yandex_maps', executable='yandex_maps_navigator')

    ld = LaunchDescription()
    ld.add_action(start_controller)
    # ld.add_action(start_planner)
    # ld.add_action(start_navigator)

    return ld
