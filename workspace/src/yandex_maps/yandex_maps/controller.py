import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import math
from copy import deepcopy

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import ExtrapolationException, ConnectivityException, LookupException

MAX_ANGULAR_SPEED = 1.0
MIN_ANGULAR_SPEED = 0.0
ANGULAR_P = 1.5
MAX_LINEAR_SPEED = 0.3
MIN_LINEAR_SPEED = 0.05
LINEAR_P = 4


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


def normilize_angle(_a):
    a = _a
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class ControllerNode(Node):

    def __init__(self):
        super().__init__('ControllerNode')
        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            # '/goal_pose',
            '/path_pose',
            self.goal_pose_callback,
            qos_profile_sensor_data)

        self.current_position = None
        self.target_position = None

    def goal_pose_callback(self, data):
        self.target_position = np.array([
            data.pose.position.x,
            data.pose.position.y,
            euler_from_quaternion(data.pose.orientation)[2]
        ])

    def update_position(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time())
            self.current_position = np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                euler_from_quaternion(t.transform.rotation)[2]
            ])
            self.update_velocity()

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('waiting for map transform')

    def update_velocity(self):
        if type(self.target_position) != np.ndarray:
            return
        movement = Twist()

        diff = self.target_position - self.current_position

        angle_diff = normilize_angle(diff[2])

        mul = angle_diff / abs(angle_diff)
        movement.angular.z = mul * \
            min(MAX_ANGULAR_SPEED, max(
                MIN_ANGULAR_SPEED, abs(angle_diff) * ANGULAR_P))

        a, b = math.cos(self.current_position[2]), math.sin(
            self.current_position[2])
        R = np.array([[a, b],
                      [-b, a]])

        vel = R.dot(diff[0:2])
        vel *= LINEAR_P

        x_mul = vel[0] / abs(vel[0])
        y_mul = vel[1] / abs(vel[1])
        movement.linear.x = x_mul * \
            min(MAX_LINEAR_SPEED, max(MIN_LINEAR_SPEED, abs(vel[0])))
        movement.linear.y = y_mul * \
            min(MAX_LINEAR_SPEED, max(MIN_LINEAR_SPEED, abs(vel[1])))

        self.publisher.publish(movement)

    def timer_callback(self):
        self.update_position()


def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()

    rclpy.spin(controller_node)

    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
