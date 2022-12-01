import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import math
from copy import deepcopy
from time import sleep

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import ExtrapolationException, ConnectivityException, LookupException


DIST_TOLERANCE = 0.1
ANGLE_TOLERANCE = 0.05


def normilize_angle(_a):
    a = _a
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def euler_from_quaternion(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


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
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path = []
        self.busy = False
        self.current_point = None

    def get_transform(self, parent_frame, child_frame):
        while True:
            try:
                return self.tf_buffer.lookup_transform(
                    parent_frame, child_frame, rclpy.time.Time())
            except (LookupException, ConnectivityException, ExtrapolationException):
                self.get_logger().warn('waiting for transform from ' +
                                       parent_frame + ' to ' + child_frame)
            sleep(1)

    def point_reached(self, pose_transform, target):
        dist = math.sqrt((pose_transform.transform.translation.x - target.pose.position.x)
                        ** 2 + (pose_transform.transform.translation.y - target.pose.position.y) ** 2)
        pose_rotation = euler_from_quaternion(pose_transform.transform.rotation)[2]
        target_rotation = euler_from_quaternion(target.pose.orientation)[2]
        return dist <= DIST_TOLERANCE and normilize_angle(pose_rotation - target_rotation) <= ANGLE_TOLERANCE


    def timer_callback(self):
        if self.busy:
            return
        self.busy = True
        if len(self.path) > 0:
            if self.current_point == None:
                self.current_point = self.path.pop()
                self.pose_publisher.publish(self.current_point)

            self_pose_t = self.get_transform("map", "base_link")
            if self.point_reached(self_pose_t, self.current_point):
                self.current_point = self.path.pop()
                self.pose_publisher.publish(self.current_point)
        self.busy = False

    def path_callback(self, data):
        self.path = data.poses
        self.current_point = None
        self.path.reverse()


def main(args=None):
    rclpy.init(args=args)
    navigator_node = NavigatorNode()

    rclpy.spin(navigator_node)

    navigator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
