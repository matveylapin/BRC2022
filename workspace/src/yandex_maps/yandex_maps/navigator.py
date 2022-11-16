import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import math
from copy import deepcopy

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import ExtrapolationException, ConnectivityException, LookupException


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('NavigatorNode')
        self.pose_publisher = self.create_publisher(
            PoseStamped, '/path_pose', 10)

        self.path_sub = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.path = []

    def timer_callback(self):
        if len(self.path) > 0:
            self.pose_publisher.publish(self.path.pop())

    def path_callback(self, data):
        self.path = data.poses
        self.path.reverse()


def main(args=None):
    rclpy.init(args=args)
    navigator_node = NavigatorNode()

    rclpy.spin(navigator_node)

    navigator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
